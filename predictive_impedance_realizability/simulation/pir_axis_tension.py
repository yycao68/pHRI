#!/usr/bin/env python3
"""The tension between PIR's two axes, measured across everything already run.

PIR's naming rests on realizability having *two* axes: feasibility (saturation,
reported by ``r_con``) and passivity (energy, reported by ``r_auth``).  This
script asks the question that decides whether the evidence supports that
framing: **is there an operating point where both axes are simultaneously
load-bearing and the controller is well behaved?**

It aggregates every closed-loop run and sweep point already stored in
``results/`` and classifies each by which authorization actually fired, then
adds the one calculation the stored data does not contain: how large a push
would have to be, at each pose, to bring the anchor to the torque envelope.

The answer is not comfortable, and it is the reason this script exists rather
than a paragraph of prose:

* At `phri2`'s pose the feasibility axis fires and the passivity axis is
  inert (``r_auth`` is exactly zero on the headline runs).
* At the recommended pose the passivity axis carries 49% of the realization
  residual and the feasibility axis never fires at all -- ``alpha_tau`` stays
  at 1.0 across every sweep point.
* The only points where both fire are at `phri2`'s pose under 4-12x
  disturbance, which is also where the anchor headroom is negative and the
  workspace excursion reaches 164 mm.

The mechanism is structural rather than incidental, and Panel B shows it: both
authorizations act on the same object (the residual) through the same torque
budget.  A nominal stiff enough to make saturation bind leaves a residual too
small to drain the tank; a nominal soft enough to leave the residual real work
leaves the torque envelope slack.  Only a large *disturbance* raises velocity
(draining the tank) and the anchor (loading the envelope) at once.

Run::

    python3 pir_axis_tension.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

import pir_common as pc

TOL = 1e-9
RECOMMENDED_POSE = (-1.30, -1.30, 1.571)
PUSH_GRID = (20.0, 40.0, 60.0, 80.0, 100.0, 120.0)

#: (stored file, human label, which pose it was run at)
CLOSED_LOOP = (
    ("pir_closed_loop_push_derated_joint4.json", "phri2 pose / push", "phri2"),
    ("pir_closed_loop_merged_derated_joint4.json", "phri2 pose / merged", "phri2"),
    ("pir_closed_loop_pose_q2m13_q4m13_push_derated_joint4.json",
     "recommended / push", "recommended"),
    ("pir_closed_loop_pose_q2m13_q4m13_merged_derated_joint4.json",
     "recommended / merged", "recommended"),
)
SWEEPS = (
    ("pir_e0_sweep.json", "phri2"),
    ("pir_e0_sweep_pose_q2m13_q4m13.json", "recommended"),
)


def _fires(alpha: float) -> bool:
    return alpha < 1.0 - TOL


def classify(results: Path) -> dict:
    """Which axis fired, per closed-loop run and per sweep point."""
    runs = []
    for name, label, pose in CLOSED_LOOP:
        path = results / name
        if not path.exists():
            continue
        s = json.loads(path.read_text())["variants"]["pir"]
        runs.append({
            "label": label, "pose": pose,
            "alpha_tau_min": s["alpha_tau_min"], "alpha_E_min": s["alpha_E_min"],
            "feasibility_fires": _fires(s["alpha_tau_min"]),
            "passivity_fires": _fires(s["alpha_E_min"]),
            "r_con_fast_rms": s["r_con_fast_rms"], "r_auth_rms": s["r_auth_rms"],
            "rms_realization_residual": s["rms_realization_residual"],
            "r_auth_share": s["r_auth_rms"] / max(s["rms_realization_residual"], 1e-12),
            "anchor_headroom": s["anchor_headroom"],
            "max_abs_e_axis_m": s["max_abs_e_axis_m"],
        })

    points = []
    for name, pose in SWEEPS:
        path = results / name
        if not path.exists():
            continue
        report = json.loads(path.read_text())
        for key in ("e0_sweep", "disturbance_sweep"):
            for x in report[key]:
                if x["variant"] != "pir":
                    continue
                points.append({
                    "pose": pose, "sweep": key, "E0": x["E0"],
                    "disturbance_scale": x["disturbance_scale"],
                    "alpha_tau_min": x["alpha_tau_min"], "alpha_E_min": x["alpha_E_min"],
                    "feasibility_fires": _fires(x["alpha_tau_min"]),
                    "passivity_fires": _fires(x["alpha_E_min"]),
                    "r_auth_rms": x["r_auth_rms"],
                    "rms_realization_residual": x["rms_realization_residual"],
                    "max_abs_e_axis_m": x["max_abs_e_axis_m"],
                })
    return {"runs": runs, "points": points}


def push_to_saturation() -> dict:
    """How hard the push must be, per pose, to bring the anchor to the envelope.

    Uses the static proxy ``g(q) - J_v^T F_h`` of Section 9.1.  That proxy is
    *optimistic* about feasibility (it omits Coriolis and the auxiliary
    torques), so the magnitudes it reports are lower bounds -- which only
    strengthens the conclusion, since even the lower bound at the recommended
    pose is far outside the pHRI range.
    """
    env = pc.FR3MuJoCoEnv(timestep=0.001)
    cap = pc.torque_envelope("derated_joint4")
    axis = pc.PUSH_AXIS
    out = {}
    poses = {"phri2": pc.Q_NEUTRAL.copy(), "recommended": pc.Q_NEUTRAL.copy()}
    poses["recommended"][1], poses["recommended"][3], poses["recommended"][5] = RECOMMENDED_POSE
    for label, q in poses.items():
        env.data.qpos[:7] = q
        env.data.qvel[:7] = 0.0
        env.clear_applied_forces()
        mujoco.mj_forward(env.model, env.data)
        dyn, _ = env.get_dynamics_and_state()
        J_v = dyn.J[:3, :]
        curve = [(float(f), float(np.max(np.abs(dyn.Cq_dot - J_v.T @ (f * axis)) / cap)))
                 for f in PUSH_GRID]
        # Linear interpolation to the ratio = 1 crossing.
        crossing = None
        for (f0, r0), (f1, r1) in zip(curve, curve[1:]):
            if r0 < 1.0 <= r1:
                crossing = f0 + (1.0 - r0) * (f1 - f0) / (r1 - r0)
                break
        out[label] = {
            "tau_base_floor": float(np.max(np.abs(dyn.Cq_dot) / cap)),
            "curve": curve,
            "push_N_to_saturate": crossing,
        }
    return out


def make_figure(report: dict, outdir: Path) -> Path:
    runs = report["classification"]["runs"]
    points = report["classification"]["points"]
    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.3))

    # --- A: which axis fires, per closed-loop run ------------------------
    labels = [r["label"] for r in runs]
    x = np.arange(len(labels))
    axes[0].bar(x - 0.2, [100 * r["r_con_fast_rms"] / max(r["rms_realization_residual"], 1e-12)
                          for r in runs], 0.4, label=r"feasibility ($r_{con}$, fast)",
                color="tab:red")
    axes[0].bar(x + 0.2, [100 * r["r_auth_share"] for r in runs], 0.4,
                label=r"passivity ($r_{auth}$)", color="tab:blue")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, rotation=25, ha="right", fontsize=8)
    axes[0].set_ylabel("share of realization residual [%]")
    axes[0].set_title("A. Which axis explains the deviation", fontsize=9)
    axes[0].legend(fontsize=8)
    # The feasibility bars are ~0 at both poses, so label every bar: an
    # invisible bar and a missing bar look identical, and here the difference
    # is the whole point.
    for i, r in enumerate(runs):
        con = 100 * r["r_con_fast_rms"] / max(r["rms_realization_residual"], 1e-12)
        for dx, val in ((-0.2, con), (0.2, 100 * r["r_auth_share"])):
            axes[0].annotate(f"{val:.1f}", xy=(i + dx, val), xytext=(0, 3),
                             textcoords="offset points", ha="center", fontsize=7)

    # --- B: co-activation map over every sweep point ---------------------
    for pose, marker in (("phri2", "o"), ("recommended", "s")):
        sel = [p for p in points if p["pose"] == pose]
        if not sel:
            continue
        axes[1].scatter([max(p["alpha_tau_min"], 1e-3) for p in sel],
                        [max(p["alpha_E_min"], 1e-3) for p in sel],
                        marker=marker, s=46, alpha=0.8, label=f"{pose} pose")
    axes[1].axvline(1.0, color="0.6", ls="--", lw=1)
    axes[1].axhline(1.0, color="0.6", ls="--", lw=1)
    axes[1].set_xscale("log")
    axes[1].set_yscale("log")
    axes[1].set_xlabel(r"$\min \alpha_\tau$   (feasibility fires $\to$ left)")
    axes[1].set_ylabel(r"$\min \alpha_E$   (passivity fires $\to$ down)")
    axes[1].set_title("B. Both axes fire only in the lower-left\n"
                      "(and only at phri2's pose, under disturbance)", fontsize=9)
    axes[1].legend(fontsize=8, loc="lower left")

    # --- C: push needed to load the feasibility axis ---------------------
    for label, colour in (("phri2", "tab:orange"), ("recommended", "tab:green")):
        d = report["push_to_saturation"][label]
        f, r = zip(*d["curve"])
        crossing = d["push_N_to_saturate"]
        tag = f" ({crossing:.0f} N)" if crossing else " (> 120 N)"
        axes[2].plot(f, r, "o-", color=colour, label=f"{label} pose{tag}", ms=4)
    axes[2].axhline(1.0, color="k", ls="--", lw=1)
    axes[2].axvline(pc.PUSH_MAGNITUDE_N, color="0.5", ls=":", lw=1.2)
    axes[2].annotate("benchmark\n20 N", xy=(pc.PUSH_MAGNITUDE_N, 2.1),
                     fontsize=8, ha="center", color="0.4")
    axes[2].set_xlabel("push magnitude [N]")
    axes[2].set_ylabel(r"static anchor proxy $/\ \bar\tau$")
    axes[2].set_title("C. Force needed to make saturation bind", fontsize=9)
    axes[2].legend(fontsize=8)

    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle("PIR's two axes are anti-correlated: each binds where the other "
                 "does not", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_axis_tension.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    args = parser.parse_args()

    report = {
        "classification": classify(args.outdir),
        "push_to_saturation": push_to_saturation(),
    }
    points = report["classification"]["points"]
    both = [p for p in points if p["feasibility_fires"] and p["passivity_fires"]]
    report["summary"] = {
        "n_sweep_points": len(points),
        "n_both_axes_fire": len(both),
        "both_axes_points": [
            {k: p[k] for k in ("pose", "E0", "disturbance_scale", "alpha_tau_min",
                               "alpha_E_min", "max_abs_e_axis_m")} for p in both],
        "feasibility_ever_fires_at_recommended_pose": any(
            p["feasibility_fires"] for p in points if p["pose"] == "recommended"),
    }
    (args.outdir / "pir_axis_tension.json").write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir)

    print(f"{'closed-loop run':<24} {'a_tau':>7} {'a_E':>8} {'r_con%':>7} {'r_auth%':>8} "
          f"{'headroom':>9}")
    for r in report["classification"]["runs"]:
        share = 100 * r["r_con_fast_rms"] / max(r["rms_realization_residual"], 1e-12)
        print(f"{r['label']:<24} {r['alpha_tau_min']:>7.4f} {r['alpha_E_min']:>8.4f} "
              f"{share:>6.1f}% {100 * r['r_auth_share']:>7.1f}% "
              f"{100 * r['anchor_headroom']:>8.1f}%")
    s = report["summary"]
    print(f"\nsweep points where BOTH axes fire: {s['n_both_axes_fire']} / {s['n_sweep_points']}")
    for p in s["both_axes_points"]:
        print(f"  {p['pose']:<12} E0={p['E0']:<6g} dist={p['disturbance_scale']:<4g} "
              f"a_tau={p['alpha_tau_min']:.4f} a_E={p['alpha_E_min']:.4f} "
              f"|e|={1e3 * p['max_abs_e_axis_m']:.0f} mm")
    print(f"feasibility axis ever fires at the recommended pose: "
          f"{s['feasibility_ever_fires_at_recommended_pose']}")
    print(f"\npush needed to make saturation bind:")
    for label, d in report["push_to_saturation"].items():
        crossing = d["push_N_to_saturate"]
        print(f"  {label:<12} tau_base floor {d['tau_base_floor']:.3f}   "
              + (f"{crossing:.0f} N" if crossing else "> 120 N"))
    print(f"\nwrote {args.outdir / 'pir_axis_tension.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
