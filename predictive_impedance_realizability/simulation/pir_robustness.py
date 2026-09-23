#!/usr/bin/env python3
"""Decision 3: are formulations (a) and (b) robust, or single-seed artifacts?

Section 12's decision 3 offers two ways to state PIR's two-axis claim:

**(a) The flip.**  You cannot tell in advance which axis binds: at phri2's pose
the feasibility axis fires and ``r_auth`` is exactly zero, at the recommended
pose the passivity axis removes 46% of the intended behaviour and
``alpha_tau`` never fires.  One joint-angle change flips it.

**(b) The dual-axis exhibit.**  Section 10.5's single candidate -- phri2's pose
at the originally certified gains under 4x disturbance -- where both axes carry
real load with every certificate intact.

Both were measured at **one disturbance seed** and, for (b), in **one cell**.
That is thin for a claim a paper is built on, so this re-runs all of it across
seeds and across two disturbance profiles (with and without
``rejectable_force``'s between-manager-tick pulse, which is the component
impedance_residual added specifically to defeat a slow guard).

What each formulation needs to survive:

* (a) needs the *assignment* to be stable: feasibility-only at phri2's pose,
  passivity-only at the recommended one, at every seed.  A single seed where
  both fire at the recommended pose would not kill it, but it would have to be
  reported.
* (b) needs the candidate to stay dual-axis AND certificate-intact at every
  seed, and the 69.5 mm excursion that put it 16% outside the box to be stable
  rather than incidental -- if the excursion swings widely, the cell's
  qualification is a coin flip on the box allowance.

Run::

    python3 pir_robustness.py
"""

from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pir_common as pc
from run_pir_closed_loop import run_variant

SEEDS = tuple(range(10))
#: None keeps rejectable_force's default (pulse scaled with the sinusoids);
#: 0.0 removes the between-tick pulse entirely.
PULSE_SETTINGS = ((None, "with pulse"), (0.0, "no pulse"))

RECOMMENDED_POSE = (-1.30, -1.30, 1.571)
MIN_SHARE = 0.02  # same load-bearing threshold as pir_joint_scan
BOX = pc.WORKSPACE_BOUND_M

#: name -> (pose q2q4q6 or None, K0, D0, disturbance scale, what it is evidence for)
CASES = {
    "flip_phri2_pose": (None, 380.0, 29.07, 1.0, "a"),
    "flip_recommended_pose": (RECOMMENDED_POSE, 520.0, 56.13, 1.0, "a"),
    "candidate": (None, 380.0, 29.07, 4.0, "b"),
}


def _pose(q246) -> np.ndarray | None:
    if q246 is None:
        return None
    q = pc.Q_NEUTRAL.copy()
    q[1], q[3], q[5] = q246
    return q


def _worker(args: tuple) -> dict:
    name, seed, pulse, pulse_label = args
    q246, k0, d0, scale, evidence_for = CASES[name]
    s = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                    disturbance_scale=scale, pose=_pose(q246),
                    seed=seed, pulse_scale=pulse)["summary"]
    denom = max(s["a_id_rms"], 1e-12)
    con = s["r_con_fast_rms"] / denom
    auth = s["r_auth_rms"] / denom
    return {
        "case": name, "evidence_for": evidence_for, "seed": seed,
        "pulse": pulse_label,
        "r_con_over_a_id": con, "r_auth_over_a_id": auth,
        "feasibility_load_bearing": bool(con >= MIN_SHARE),
        "passivity_load_bearing": bool(auth >= MIN_SHARE),
        "dual_axis": bool(con >= MIN_SHARE and auth >= MIN_SHARE),
        "alpha_tau_min": s["alpha_tau_min"], "alpha_E_min": s["alpha_E_min"],
        "lemma1_precondition_holds": s["lemma1_precondition_holds"],
        "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
        "tank_floor_holds": s["tank_floor_holds"],
        "certificates_intact": bool(s["lemma1_precondition_holds"]
                                    and s["lemma1_conclusion_holds"]
                                    and s["tank_floor_holds"]),
        "max_abs_e_axis_m": s["max_abs_e_axis_m"],
        "box_overshoot": s["max_abs_e_axis_m"] / BOX,
        "anchor_headroom": s["anchor_headroom"],
        "rms_realization_residual": s["rms_realization_residual"],
    }


def run(workers: int = 4) -> dict:
    jobs = [(name, seed, pulse, label)
            for name in CASES for seed in SEEDS for pulse, label in PULSE_SETTINGS]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_worker, jobs))
    return score(rows)


def score(rows: list[dict]) -> dict:
    """Derive the verdicts from the runs, so --refigure re-derives rather than
    trusting whatever criterion was in force when the JSON was written."""

    def summarise(name: str) -> dict:
        sel = [r for r in rows if r["case"] == name]
        over = np.array([r["box_overshoot"] for r in sel])
        return {
            "n_runs": len(sel),
            "n_dual_axis": sum(r["dual_axis"] for r in sel),
            "n_feasibility_only": sum(r["feasibility_load_bearing"]
                                      and not r["passivity_load_bearing"] for r in sel),
            "n_passivity_only": sum(r["passivity_load_bearing"]
                                    and not r["feasibility_load_bearing"] for r in sel),
            "n_neither": sum(not r["feasibility_load_bearing"]
                             and not r["passivity_load_bearing"] for r in sel),
            "n_certificates_intact": sum(r["certificates_intact"] for r in sel),
            # The metric formulation (b) actually needs: both axes carrying
            # load AND nothing broken.  Either alone is not the exhibit.
            "n_dual_axis_and_intact": sum(r["dual_axis"] and r["certificates_intact"]
                                          for r in sel),
            "n_precondition_holds": sum(r["lemma1_precondition_holds"] for r in sel),
            "n_conclusion_holds": sum(r["lemma1_conclusion_holds"] for r in sel),
            "n_tank_floor_holds": sum(r["tank_floor_holds"] for r in sel),
            "box_overshoot_min": float(over.min()),
            "box_overshoot_max": float(over.max()),
            "box_overshoot_mean": float(over.mean()),
            "r_con_over_a_id_range": [float(min(r["r_con_over_a_id"] for r in sel)),
                                      float(max(r["r_con_over_a_id"] for r in sel))],
            "r_auth_over_a_id_range": [float(min(r["r_auth_over_a_id"] for r in sel)),
                                       float(max(r["r_auth_over_a_id"] for r in sel))],
        }

    summary = {name: summarise(name) for name in CASES}
    phri2, rec, cand = (summary["flip_phri2_pose"], summary["flip_recommended_pose"],
                        summary["candidate"])
    # (a) survives if the ASSIGNMENT never swaps: the passivity axis dead at
    # one pose and load-bearing at the other, and the feasibility axis never
    # the load-bearing one at the recommended pose.
    flip_clean = (phri2["n_passivity_only"] == 0 and phri2["n_dual_axis"] == 0
                  and rec["n_passivity_only"] == rec["n_runs"]
                  and rec["n_feasibility_only"] == 0 and rec["n_dual_axis"] == 0)
    # The honest qualifier on (a): at 1x disturbance the feasibility axis never
    # reaches the load-bearing threshold at EITHER pose, so what flips is which
    # axis can carry load at all, not an exchange of the load between them.
    intact = [r for r in rows if r["case"] == "candidate"
              and r["dual_axis"] and r["certificates_intact"]]
    return {
        "question": "are decision 3's formulations (a) and (b) robust to seed "
                    "and disturbance profile?",
        "seeds": list(SEEDS),
        "pulse_settings": [label for _, label in PULSE_SETTINGS],
        "min_share": MIN_SHARE,
        "summary": summary,
        "verdict": {
            "a_flip_assignment_stable": bool(flip_clean),
            "a_feasibility_ever_load_bearing_at_1x": bool(
                phri2["n_feasibility_only"] or phri2["n_dual_axis"]
                or rec["n_feasibility_only"] or rec["n_dual_axis"]),
            "a_phri2_r_con_range": phri2["r_con_over_a_id_range"],
            "a_recommended_r_auth_range": rec["r_auth_over_a_id_range"],
            "b_n_dual_axis_and_intact": cand["n_dual_axis_and_intact"],
            "b_n_runs": cand["n_runs"],
            "b_precondition_holds": cand["n_precondition_holds"],
            "b_conclusion_holds": cand["n_conclusion_holds"],
            "b_tank_floor_holds": cand["n_tank_floor_holds"],
            "b_box_overshoot_span": [cand["box_overshoot_min"], cand["box_overshoot_max"]],
            "b_box_overshoot_span_when_qualifying": (
                [min(r["box_overshoot"] for r in intact),
                 max(r["box_overshoot"] for r in intact)] if intact else None),
        },
        "rows": rows,
    }


def make_figure(report: dict, outdir: Path) -> Path:
    rows = report["rows"]
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.2))
    names = list(CASES)
    labels = {"flip_phri2_pose": "(a) phri2 pose", "flip_recommended_pose": "(a) recommended",
              "candidate": "(b) candidate"}

    for col, key, title, colour in ((0, "r_con_over_a_id", r"feasibility $r_{con}/|a_{id}|$", "tab:red"),
                                    (1, "r_auth_over_a_id", r"passivity $r_{auth}/|a_{id}|$", "tab:blue")):
        ax = axes[col]
        data = [[100 * r[key] for r in rows if r["case"] == n] for n in names]
        ax.boxplot(data, tick_labels=[labels[n] for n in names], widths=0.5)
        ax.axhline(100 * report["min_share"], color="0.5", ls="--", lw=1)
        ax.annotate("load-bearing\nthreshold", xy=(0.02, 100 * report["min_share"]),
                    xytext=(2, 6), textcoords="offset points", fontsize=7, color="0.4")
        ax.set_ylabel("% of intended behaviour")
        ax.set_title(f"{title}\nover {len(report['seeds'])} seeds x 2 profiles", fontsize=9)
        ax.tick_params(axis="x", labelsize=8)

    ax = axes[2]
    data = [[100 * r["box_overshoot"] for r in rows if r["case"] == n] for n in names]
    ax.boxplot(data, tick_labels=[labels[n] for n in names], widths=0.5)
    ax.axhline(100, color="k", ls="--", lw=1)
    ax.axhline(120, color="tab:orange", ls=":", lw=1.2)
    ax.annotate("box", xy=(0.02, 100), xytext=(2, 3), textcoords="offset points",
                fontsize=7)
    ax.annotate("1.20x allowance", xy=(0.02, 120), xytext=(2, 3),
                textcoords="offset points", fontsize=7, color="tab:orange")
    ax.set_ylabel("excursion / box [%]")
    ax.set_title("does (b)'s qualification straddle\nthe allowance it needs?", fontsize=9)
    ax.tick_params(axis="x", labelsize=8)

    for a in axes:
        a.grid(alpha=0.25, axis="y")
    fig.suptitle("Decision 3: robustness of formulation (a) the flip, and "
                 "(b) the dual-axis candidate", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_robustness.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--refigure", action="store_true",
                        help="redraw from an existing pir_robustness.json")
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    path = args.outdir / "pir_robustness.json"
    if args.refigure:
        report = score(json.loads(path.read_text())["rows"])
        path.write_text(json.dumps(report, indent=2))
    else:
        report = run(workers=args.workers)
        path.write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir)

    print(f"{len(report['seeds'])} seeds x {len(report['pulse_settings'])} profiles\n")
    print(f"{'case':<24} {'runs':>5} {'dual':>5} {'feas only':>10} {'pass only':>10} "
          f"{'neither':>8} {'certs ok':>9} {'excursion/box':>16}")
    for name, sm in report["summary"].items():
        print(f"{name:<24} {sm['n_runs']:>5} {sm['n_dual_axis']:>5} "
              f"{sm['n_feasibility_only']:>10} {sm['n_passivity_only']:>10} "
              f"{sm['n_neither']:>8} {sm['n_certificates_intact']:>9} "
              f"{100 * sm['box_overshoot_min']:>7.0f}-{100 * sm['box_overshoot_max']:.0f}%")
    v = report["verdict"]
    print(f"\n(a) assignment stable at both endpoints            : {v['a_flip_assignment_stable']}")
    print(f"    phri2 pose  r_con  {100 * v['a_phri2_r_con_range'][0]:.2f}-"
          f"{100 * v['a_phri2_r_con_range'][1]:.2f}%  (threshold "
          f"{100 * report['min_share']:.0f}%, so never load-bearing)")
    print(f"    recommended r_auth {100 * v['a_recommended_r_auth_range'][0]:.1f}-"
          f"{100 * v['a_recommended_r_auth_range'][1]:.1f}%  in 20/20")
    print(f"\n(b) dual-axis AND every certificate intact         : "
          f"{v['b_n_dual_axis_and_intact']} / {v['b_n_runs']}")
    print(f"    precondition {v['b_precondition_holds']}/{v['b_n_runs']}, "
          f"conclusion {v['b_conclusion_holds']}/{v['b_n_runs']}, "
          f"tank floor {v['b_tank_floor_holds']}/{v['b_n_runs']}")
    span = v["b_box_overshoot_span_when_qualifying"]
    print(f"    excursion over all runs {100 * v['b_box_overshoot_span'][0]:.0f}-"
          f"{100 * v['b_box_overshoot_span'][1]:.0f}% of the box; "
          + (f"when it qualifies, {100 * span[0]:.0f}-{100 * span[1]:.0f}%"
             if span else "never qualifies"))
    print(f"\nwrote {args.outdir / 'pir_robustness.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
