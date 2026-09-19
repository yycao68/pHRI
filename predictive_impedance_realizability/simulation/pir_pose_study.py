#!/usr/bin/env python3
"""Decision 2: can any pose let the merge claim impedance_residual's envelope?

The draft's decision 2 asks which torque envelope the merged paper is written
against.  Section 5.1 showed rho = 0.28 is structurally dead at phri2's nominal
pose -- not a tuning failure -- and Section 8.4 showed that even the fixed
controller ultimately rests on ``tau_base`` alone fitting the envelope, which
is a property of the pose.  Both trace to one number: at Q_NEUTRAL the bias
torque uses **77.8%** of joint 4's rho = 0.28 cap before any control authority
is applied at all.

So the decision is empirical.  This screens poses and push directions for the
two quantities that decide it, in increasing cost:

**Stage 1 -- the gravity floor.**  With ``q_null`` set to the candidate pose
itself and the arm at rest, ``compute_tau_base`` reduces *exactly* to the
gravity torque: the null-space term is zero because ``q = q_null``, and the
orientation term is zero because ``R_d`` is the pose's own orientation.  So the
irreducible floor is a forward-kinematics computation, and a fine pose grid
costs milliseconds.

**Stage 2 -- the fallback anchor.**  Section 6.2 established that at the
alpha -> 0 equilibrium ``K0 e = F_h``, so ``J_v^T F_nom = -J_v^T F_h``
independently of the gains: the anchor there is a property of the pose and the
push, not of the design.  Its cheap proxy, evaluated at the nominal pose, is
``g(q) - J_v(q)^T F_h``; this is what the anchor tends to as K0 grows, and it
is what decides whether an envelope is reachable *at all*.

**Stage 3 -- the conditioning guard, which the first two stages need.**  The
anchor proxy alone is *necessary but not sufficient*, and on its own it is
actively misleading: it rewards poses that resist the push through kinematic
structure rather than through control.  The first run of this screen picked
``(q2, q4, q6) = (-0.20, -0.90, 2.60)``, which clears rho = 0.28 with an anchor
ratio of 0.685 -- and is near-singular.  Its task-space inertia along the push
axis is 47.2 kg against the neutral pose's 4.56 kg, and sigma_min(J_v) falls
from 0.256 to 0.080.  The arm is braced, not better: at 47 kg of apparent
inertia the realization layer cannot render a 2 kg desired impedance in the
push direction at all, which defeats the point of the controller.

So a pose is only a candidate if it also keeps the push direction
*controllable*: ``axis^T Lambda axis`` no worse than twice the neutral pose's,
and ``sigma_min(J_v)`` no worse than 0.7x it.  Both thresholds are judgement
calls and both are reported per pose, so a different line can be drawn from the
stored data without re-running.

**Stage 4 -- the along-trajectory floor, because stages 1-3 are optimistic.**
Stages 1-2 evaluate the gravity floor at the nominal pose and at rest.  The
quantity the gate actually uses is ``max_t |tau_base,t| / cap`` along the
operating trajectory, where ``tau_base`` also carries Coriolis terms and the
null-space and orientation torques that become nonzero the moment ``q`` leaves
``q_null``.  The gap is not small: the first candidate this screen produced for
rho = 0.28 scored 0.470 statically and **1.287** along the trajectory, with the
binding joint moving from 4 to 5.  That is the other half of the lesson --
relieving joint 4 can simply move the bottleneck to joints 5-7, whose
rho = 0.28 caps are only 3.36 N.m.

Stage 4 therefore replays the phri2 benchmark at each shortlisted pose and
reports the real floor.  It costs ~6 s per pose instead of milliseconds, so it
runs on the shortlist rather than the grid.

Stages 1-4 are a screen, not a gate: a pose that survives them still has to
pass the full ``(K0, D0)`` scan.  What they buy is knowing which poses are
worth scanning, at a cost of seconds instead of 19 minutes each.

Run::

    python3 pir_pose_study.py
"""

from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

import pir_common as pc

#: Push directions worth trying.  -z is phri2's own (gravity-aligned, which is
#: what loads the shoulder/elbow pitch joints hardest); the horizontal ones
#: load a different set.
PUSH_AXES: dict[str, np.ndarray] = {
    "-z": np.array([0.0, 0.0, -1.0]),
    "+x": np.array([1.0, 0.0, 0.0]),
    "+y": np.array([0.0, 1.0, 0.0]),
}

#: Joints 2 and 4 carry essentially all of the gravity load in this arm's
#: shoulder-elbow plane, so they are what the grid varies.  Joint 6 is swept
#: coarsely with them because it sets how much of the wrist hangs off the
#: elbow.  Everything else is held at Q_NEUTRAL.
Q2_GRID = np.round(np.linspace(-1.60, 0.60, 23), 4)
Q4_GRID = np.round(np.linspace(-3.00, -0.30, 28), 4)
Q6_GRID = (1.571, 2.100, 2.600)


def pose_from(q2: float, q4: float, q6: float) -> np.ndarray:
    q = pc.Q_NEUTRAL.copy()
    q[1], q[3], q[5] = q2, q4, q6
    return q


class PoseProbe:
    """Forward-kinematics probe: gravity and the Jacobian at an arbitrary pose.

    Reuses the existing FR3 MuJoCo environment rather than building a second
    model; only the configuration is moved.
    """

    def __init__(self) -> None:
        self.env = pc.FR3MuJoCoEnv(timestep=0.001)
        self.env.reset()
        self.lo = self.env.model.jnt_range[:7, 0].copy()
        self.hi = self.env.model.jnt_range[:7, 1].copy()

    def in_limits(self, q: np.ndarray) -> bool:
        # A small margin: a pose sitting exactly on a joint stop is not one the
        # arm can hold a nominal at.
        margin = 0.05
        return bool(np.all(q >= self.lo + margin) and np.all(q <= self.hi - margin))

    def at(self, q: np.ndarray) -> dict:
        self.env.data.qpos[: self.env.nv] = q
        self.env.data.qvel[: self.env.nv] = 0.0
        self.env.clear_applied_forces()
        mujoco.mj_forward(self.env.model, self.env.data)
        dyn, state = self.env.get_dynamics_and_state()
        J_v = dyn.J[:3, :]
        lam = np.linalg.inv(J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3))
        # At rest with q_null = q and R_d = this pose's orientation, both
        # auxiliary terms of compute_tau_base vanish identically, so tau_base
        # IS the bias torque.  (Asserted in the test suite.)
        return {"gravity": dyn.Cq_dot.copy(), "J_v": J_v.copy(),
                "ee_pos": state.ee_pos.copy(), "Lambda": lam,
                "sigma_min": float(np.linalg.svd(J_v, compute_uv=False)[-1])}


#: How much worse than phri2's own pose the push direction may be before the
#: pose is rejected as braced rather than better.  See the module docstring.
MAX_LAMBDA_RATIO = 2.0
MIN_SIGMA_RATIO = 0.7


def screen(probe: PoseProbe) -> list[dict]:
    rows: list[dict] = []
    caps = {name: pc.torque_envelope(name) for name in pc.ENVELOPE_NAMES}
    ref = probe.at(pc.Q_NEUTRAL)
    ref_sigma = ref["sigma_min"]
    ref_lambda = {a: float(v @ ref["Lambda"] @ v) for a, v in PUSH_AXES.items()}
    for q6 in Q6_GRID:
        for q2 in Q2_GRID:
            for q4 in Q4_GRID:
                q = pose_from(q2, q4, q6)
                if not probe.in_limits(q):
                    continue
                at = probe.at(q)
                row = {
                    "q2": float(q2), "q4": float(q4), "q6": float(q6),
                    "ee_pos": at["ee_pos"].tolist(),
                    # Reachability sanity: a pose folded into the base or
                    # below the table is not a usable interaction pose.
                    "ee_height_m": float(at["ee_pos"][2]),
                    "ee_reach_m": float(np.linalg.norm(at["ee_pos"][:2])),
                    "sigma_min": at["sigma_min"],
                    "sigma_ratio": at["sigma_min"] / ref_sigma,
                }
                for axis_name, axis in PUSH_AXES.items():
                    lam_axis = float(axis @ at["Lambda"] @ axis)
                    row[f"lambda_axis_{axis_name}"] = lam_axis
                    row[f"lambda_ratio_{axis_name}"] = lam_axis / ref_lambda[axis_name]
                    row[f"controllable_{axis_name}"] = bool(
                        lam_axis / ref_lambda[axis_name] <= MAX_LAMBDA_RATIO
                        and at["sigma_min"] / ref_sigma >= MIN_SIGMA_RATIO)
                for env_name, cap in caps.items():
                    row[f"gravity_ratio_{env_name}"] = float(
                        np.max(np.abs(at["gravity"]) / cap))
                    for axis_name, axis in PUSH_AXES.items():
                        anchor = at["gravity"] - at["J_v"].T @ (pc.PUSH_MAGNITUDE_N * axis)
                        row[f"anchor_proxy_{env_name}_{axis_name}"] = float(
                            np.max(np.abs(anchor) / cap))
                rows.append(row)
    return rows


#: How many poses per (envelope, push axis) go on to the along-trajectory check.
SHORTLIST_N = 8


def _trajectory_floor(args: tuple) -> dict:
    """Stage 4 for one pose: the real ``max_t |tau_base| / cap`` on the replay."""
    q2, q4, q6, axis_name = args
    pose = pose_from(q2, q4, q6)
    axis = PUSH_AXES[axis_name]
    out = {"q2": q2, "q4": q4, "q6": q6, "axis": axis_name}
    try:
        traj = pc.load_or_generate_trajectory(pose=pose, push_axis=axis)
    except Exception as exc:  # a pose the phri2 MPC cannot drive at all
        out["error"] = f"{type(exc).__name__}: {exc}"
        return out
    for env_name in pc.ENVELOPE_NAMES:
        cap = pc.torque_envelope(env_name)
        base = pc.base_only_anchor_ratio(traj, cap)
        out[f"traj_base_ratio_{env_name}"] = base["base_only_anchor_ratio"]
        out[f"traj_worst_joint_{env_name}"] = base["base_only_worst_joint"] + 1
        out[f"traj_realized_ratio_{env_name}"] = float(
            np.max(np.abs(traj.tau_realized) / cap[None, :]))
    return out


def stage4(rows: list[dict], workers: int = 4) -> list[dict]:
    usable = [r for r in rows if r["ee_height_m"] > 0.20 and r["ee_reach_m"] > 0.25]
    shortlist: set[tuple] = set()
    for env_name in pc.ENVELOPE_NAMES:
        for axis_name in PUSH_AXES:
            key = f"anchor_proxy_{env_name}_{axis_name}"
            ok = [r for r in usable
                  if r[f"controllable_{axis_name}"] and r[key] < 1.0]
            for r in sorted(ok, key=lambda r: r[key])[:SHORTLIST_N]:
                shortlist.add((r["q2"], r["q4"], r["q6"], axis_name))
    # phri2's own pose is the reference row.
    shortlist.add((-0.785, -2.356, 1.571, "-z"))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(_trajectory_floor, sorted(shortlist)))


def summarise(rows: list[dict]) -> dict:
    """Pick, per envelope and push axis, the pose with the most anchor margin.

    Restricted to poses that are actually usable: the end effector has to be
    above the base plane and out in front of the arm, or the "pose" is a folded
    configuration no one would interact at.
    """
    usable = [r for r in rows if r["ee_height_m"] > 0.20 and r["ee_reach_m"] > 0.25]
    out: dict = {"n_poses": len(rows), "n_usable": len(usable),
                 "max_lambda_ratio": MAX_LAMBDA_RATIO,
                 "min_sigma_ratio": MIN_SIGMA_RATIO, "best": {}}
    for env_name in pc.ENVELOPE_NAMES:
        for axis_name in PUSH_AXES:
            key = f"anchor_proxy_{env_name}_{axis_name}"
            ok = [r for r in usable if r[f"controllable_{axis_name}"]]
            feasible = [r for r in ok if r[key] < 1.0]
            # Best is chosen among CONTROLLABLE poses only; ranking by anchor
            # alone selects for bracing, which is what the guard exists to stop.
            best = min(feasible, key=lambda r: r[key]) if feasible else None
            braced_best = min(usable, key=lambda r: r[key]) if usable else None
            out["best"][f"{env_name}|{axis_name}"] = {
                "n_controllable": len(ok),
                "n_feasible": len(feasible),
                "fraction_feasible": len(feasible) / max(1, len(usable)),
                "best_anchor_proxy": best[key] if best else None,
                "best_pose": {"q2": best["q2"], "q4": best["q4"], "q6": best["q6"]} if best else None,
                "best_gravity_ratio": best[f"gravity_ratio_{env_name}"] if best else None,
                "best_ee_pos": best["ee_pos"] if best else None,
                "best_lambda_ratio": best[f"lambda_ratio_{axis_name}"] if best else None,
                "best_sigma_ratio": best["sigma_ratio"] if best else None,
                # What the unguarded screen would have picked, kept so the
                # guard's effect is visible rather than silent.
                "unguarded_best_anchor_proxy": braced_best[key] if braced_best else None,
                "unguarded_best_lambda_ratio": (
                    braced_best[f"lambda_ratio_{axis_name}"] if braced_best else None),
            }
    # The reference point: phri2's own nominal pose.
    neutral = [r for r in rows
               if abs(r["q2"] + 0.785) < 1e-6 and abs(r["q4"] + 2.356) < 1e-6
               and abs(r["q6"] - 1.571) < 1e-6]
    out["neutral_reference"] = neutral[0] if neutral else None
    return out


def make_figure(rows: list[dict], summary: dict, outdir: Path) -> Path:
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.2))
    q6_ref = 1.571
    subset = [r for r in rows if abs(r["q6"] - q6_ref) < 1e-6]
    q2s = sorted({r["q2"] for r in subset})
    q4s = sorted({r["q4"] for r in subset})
    for row, env_name in enumerate(pc.ENVELOPE_NAMES):
        for col, axis_name in enumerate(PUSH_AXES):
            grid = np.full((len(q2s), len(q4s)), np.nan)
            for r in subset:
                if (r["ee_height_m"] > 0.20 and r["ee_reach_m"] > 0.25
                        and r[f"controllable_{axis_name}"]):
                    grid[q2s.index(r["q2"]), q4s.index(r["q4"])] = \
                        r[f"anchor_proxy_{env_name}_{axis_name}"]
            ax = axes[row, col]
            mesh = ax.pcolormesh(q4s, q2s, grid, cmap="RdYlGn_r", vmin=0.0, vmax=2.0,
                                 shading="nearest")
            if np.nanmin(grid) < 1.0 < np.nanmax(grid):
                ax.contour(q4s, q2s, grid, levels=[1.0], colors="k", linewidths=1.6)
            ax.plot(-2.356, -0.785, marker="*", ms=14, color="white",
                    markeredgecolor="black", markeredgewidth=0.8, linestyle="none")
            ax.set_xlabel(r"$q_4$ [rad]")
            ax.set_ylabel(r"$q_2$ [rad]")
            n = summary["best"][f"{env_name}|{axis_name}"]["n_feasible"]
            ax.set_title(f"{env_name}, push {axis_name}\n{n} usable poses feasible",
                         fontsize=9)
            fig.colorbar(mesh, ax=ax)
    fig.suptitle("Decision 2: fallback-anchor proxy over pose and push direction. "
                 r"$q_6$ = 1.571; star = phri2's pose; black line = 1; "
                 "blank = rejected as\nuncontrollable in the push direction "
                 "(braced by kinematics, not better)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_pose_study.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    probe = PoseProbe()
    rows = screen(probe)
    summary = summarise(rows)
    shortlist = stage4(rows, workers=args.workers)
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_pose_study.json").write_text(
        json.dumps({"summary": summary, "shortlist": shortlist, "rows": rows}, indent=2))
    figure = make_figure(rows, summary, args.outdir)

    ref = summary["neutral_reference"]
    print(f"poses in limits: {summary['n_poses']}, usable: {summary['n_usable']}\n")
    if ref:
        print("phri2's nominal pose, for reference:")
        for env_name in pc.ENVELOPE_NAMES:
            print(f"  {env_name:<16} gravity floor {ref[f'gravity_ratio_{env_name}']:.3f}"
                  + "".join(f"   anchor {a} {ref[f'anchor_proxy_{env_name}_{a}']:.3f}"
                            for a in PUSH_AXES))
    print(f"\nguard: lambda_axis <= {summary['max_lambda_ratio']}x neutral, "
          f"sigma_min >= {summary['min_sigma_ratio']}x neutral\n")
    print(f"{'envelope | push':<24} {'ctrlable':>9} {'feasible':>9} {'best anchor':>12} "
          f"{'lam':>5} {'sig':>5} {'best pose (q2,q4,q6)':>25} {'unguarded':>10}")
    for key, b in summary["best"].items():
        pose = b["best_pose"]
        pose_s = (f"({pose['q2']:+.2f}, {pose['q4']:+.2f}, {pose['q6']:.2f})"
                  if pose else "none")
        anchor = f"{b['best_anchor_proxy']:.3f}" if b["best_anchor_proxy"] else "-"
        lam = f"{b['best_lambda_ratio']:.2f}" if b["best_lambda_ratio"] else "-"
        sig = f"{b['best_sigma_ratio']:.2f}" if b["best_sigma_ratio"] else "-"
        print(f"{key:<24} {b['n_controllable']:>9} {b['n_feasible']:>9} {anchor:>12} "
              f"{lam:>5} {sig:>5} {pose_s:>25} "
              f"{b['unguarded_best_anchor_proxy']:>10.3f}")
    print("\nStage 4 -- the REAL floor: max_t |tau_base| / cap along the replay")
    print(f"{'pose (q2,q4,q6)':<24} {'push':>5} {'rho base':>9} {'j':>3} "
          f"{'rho realized':>13} {'der. base':>10} {'j':>3}")
    key = "traj_base_ratio_rho_0.28"
    for r in sorted(shortlist, key=lambda r: r.get(key, 9e9)):
        label = f"({r['q2']:+.2f}, {r['q4']:+.2f}, {r['q6']:.2f})".ljust(24)
        if "error" in r:
            print(f"{label} {r['axis']:>5}   {r['error']}")
            continue
        print(f"{label} {r['axis']:>5} {r[key]:>9.3f} "
              f"{r['traj_worst_joint_rho_0.28']:>3} "
              f"{r['traj_realized_ratio_rho_0.28']:>13.3f} "
              f"{r['traj_base_ratio_derated_joint4']:>10.3f} "
              f"{r['traj_worst_joint_derated_joint4']:>3}")
    print(f"\nwrote {args.outdir / 'pir_pose_study.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
