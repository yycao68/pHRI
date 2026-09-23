#!/usr/bin/env python3
"""Section 10.5: is there a well-behaved operating point where BOTH axes bind?

Section 10 found PIR's two axes anti-correlated: at phri2's pose the
feasibility axis fires and ``r_auth`` is exactly zero; at the recommended pose
``r_auth`` carries 49% of the realization residual and ``alpha_tau`` never
fires at all.  The only dual-axis points in the data sit at phri2's pose under
4-12x disturbance, where the anchor headroom is negative and the excursion
reaches 108 mm.

Pose and ``K0`` have only ever been swept SEPARATELY, so the question is open:
the two sweeps could be cutting across a diagonal ridge neither of them
resolves.  This scans them jointly.

**The pose family** interpolates in joint space between the two poses that
bracket the tension::

    q(lambda) = (1 - lambda) * q_phri2 + lambda * q_recommended

Only q2 and q4 differ between them (q6 is 1.571 at both), so this is a clean
one-dimensional family, and the torque-budget tightness -- which Section 10.2
identifies as what actually controls the competition -- varies monotonically
along it.

**D0** is set from a fixed damping ratio against each pose's OWN task-space
inertia along the push axis, so ``K0`` is the only gain varying and cells at
different poses stay comparable.

**Load-bearing** is deliberately stricter than "fired once": an axis counts
only if its residual term removes at least ``MIN_SHARE`` of the *intended*
behaviour ``a_id``.  An ``alpha_tau`` of 0.9999 for one tick is not a
feasibility axis doing work, and Section 10.1 already found one such point.

The denominator matters and the first version of this script got it wrong.
Normalising a term by the NET realization residual is not a share: the four
terms sum to the net but can oppose one another, so a single term can exceed
it -- the first run reported ``r_auth`` "shares" of 337% and 468%.  ``a_id``,
the behaviour the controller is trying to render, is the honest denominator:
it makes each number "how much of the intended behaviour this axis removed",
and it does not move with the disturbance (``a_id`` is defined against the
interaction force alone).  It is reported as a RATIO, not a share, and under a
large disturbance it legitimately exceeds 1: the QP's residual is then fighting
something much bigger than the intended behaviour, so the authorization can
remove several times ``|a_id|``.

Run::

    python3 pir_joint_scan.py
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
from run_pir_closed_loop import run_variant

PHRI2_Q24 = (-0.785, -2.356)
RECOMMENDED_Q24 = (-1.30, -1.30)
Q6 = 1.571

LAMBDA_GRID = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
K0_GRID = (150.0, 200.0, 250.0, 300.0, 380.0, 460.0, 520.0, 600.0)
DISTURBANCE_GRID = (1.0, 4.0)

#: Damping ratio held fixed across the scan (the certified cell's value).
ZETA = 0.349
#: An axis counts as load-bearing only if it explains this much of the
#: realization residual.  0.9999 of a tick is not an axis doing work.
MIN_SHARE = 0.02
#: The workspace box is slack-relaxed in phri2's QP, so a small overshoot is
#: not a violation.  10% is the headline allowance, but the verdict turns out
#: to be sensitive to it, so the scan reports the whole sensitivity curve
#: rather than a single pass/fail -- see ``box_slack_sensitivity`` in the JSON.
BOX_SLACK = 1.10
BOX_SLACK_SWEEP = (1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30)


def pose_at(lam: float) -> np.ndarray:
    q = pc.Q_NEUTRAL.copy()
    q[1] = (1 - lam) * PHRI2_Q24[0] + lam * RECOMMENDED_Q24[0]
    q[3] = (1 - lam) * PHRI2_Q24[1] + lam * RECOMMENDED_Q24[1]
    q[5] = Q6
    return q


def _task_inertia_on_push_axis(q: np.ndarray) -> float:
    env = pc.FR3MuJoCoEnv(timestep=0.001)
    env.data.qpos[:7] = q
    env.data.qvel[:7] = 0.0
    env.clear_applied_forces()
    mujoco.mj_forward(env.model, env.data)
    dyn, _ = env.get_dynamics_and_state()
    J_v = dyn.J[:3, :]
    lam = np.linalg.inv(J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3))
    return float(pc.PUSH_AXIS @ lam @ pc.PUSH_AXIS)


def damping_for(k0: float, lambda_axis: float) -> float:
    return float(2.0 * ZETA * np.sqrt(k0 * lambda_axis))


def _fallback_worker(args: tuple) -> dict:
    lam, k0, d0 = args
    r = pc.run_fallback_equilibrium(k0, d0, pose=pose_at(lam))
    return {"lambda": lam, "K0": k0, "D0": d0,
            "diag3_e_ss_m": r["e_ss_axis"],
            "diag3_passes": bool(r["e_ss_axis"] <= pc.WORKSPACE_BOUND_M)}


def _cell_worker(args: tuple) -> dict:
    lam, k0, d0, scale = args
    s = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                    disturbance_scale=scale, pose=pose_at(lam))["summary"]
    denom = max(s["a_id_rms"], 1e-12)
    con_share = s["r_con_fast_rms"] / denom
    auth_share = s["r_auth_rms"] / denom
    return {
        "lambda": lam, "K0": k0, "D0": d0, "disturbance_scale": scale,
        "alpha_tau_min": s["alpha_tau_min"], "alpha_E_min": s["alpha_E_min"],
        "r_con_over_a_id": con_share, "r_auth_over_a_id": auth_share,
        "r_con_rms": s["r_con_fast_rms"], "r_auth_rms": s["r_auth_rms"],
        "a_id_rms": s["a_id_rms"],
        "feasibility_load_bearing": bool(con_share >= MIN_SHARE),
        "passivity_load_bearing": bool(auth_share >= MIN_SHARE),
        "lemma1_precondition_holds": s["lemma1_precondition_holds"],
        "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
        "tank_floor_holds": s["tank_floor_holds"],
        "anchor_headroom": s["anchor_headroom"],
        "max_abs_e_axis_m": s["max_abs_e_axis_m"],
        "rms_realization_residual": s["rms_realization_residual"],
        "alpha_nom_min": s["alpha_nom_min"],
    }


def run(workers: int = 4) -> dict:
    inertia = {lam: _task_inertia_on_push_axis(pose_at(lam)) for lam in LAMBDA_GRID}
    gains = {(lam, k0): damping_for(k0, inertia[lam])
             for lam in LAMBDA_GRID for k0 in K0_GRID}

    with ProcessPoolExecutor(max_workers=workers) as pool:
        fallbacks = list(pool.map(
            _fallback_worker,
            [(lam, k0, gains[(lam, k0)]) for lam in LAMBDA_GRID for k0 in K0_GRID]))
        cells = list(pool.map(
            _cell_worker,
            [(lam, k0, gains[(lam, k0)], s) for lam in LAMBDA_GRID
             for k0 in K0_GRID for s in DISTURBANCE_GRID]))

    diag3 = {(f["lambda"], f["K0"]): f for f in fallbacks}
    for c in cells:
        f = diag3[(c["lambda"], c["K0"])]
        c["diag3_e_ss_m"] = f["diag3_e_ss_m"]
        c["diag3_passes"] = f["diag3_passes"]
        c["dual_axis"] = bool(c["feasibility_load_bearing"] and c["passivity_load_bearing"])
        c["well_behaved"] = bool(
            c["lemma1_precondition_holds"] and c["lemma1_conclusion_holds"]
            and c["tank_floor_holds"] and c["diag3_passes"]
            and c["max_abs_e_axis_m"] <= BOX_SLACK * pc.WORKSPACE_BOUND_M)
        c["target"] = bool(c["dual_axis"] and c["well_behaved"])

    hits = [c for c in cells if c["target"]]
    # The verdict turns on how much workspace overshoot counts as acceptable,
    # and that is a judgement call this scan should not quietly make for the
    # reader.  Report the whole curve.
    sensitivity = []
    for slack in BOX_SLACK_SWEEP:
        winners = [c for c in cells
                   if c["dual_axis"] and c["lemma1_precondition_holds"]
                   and c["lemma1_conclusion_holds"] and c["tank_floor_holds"]
                   and c["diag3_passes"]
                   and c["max_abs_e_axis_m"] <= slack * pc.WORKSPACE_BOUND_M]
        sensitivity.append({
            "box_slack": slack, "n_target": len(winners),
            "cells": [{"lambda": w["lambda"], "K0": w["K0"],
                       "disturbance_scale": w["disturbance_scale"]} for w in winners]})
    return {
        "box_slack_sensitivity": sensitivity,
        "question": ("is there a (pose, K0) with both axes load-bearing AND the "
                     "controller well behaved?"),
        "definitions": {
            "share_denominator": "RMS of a_id, the intended behaviour",
            "min_share_of_intended_behaviour": MIN_SHARE,
            "box_slack_allowance": BOX_SLACK,
            "zeta": ZETA,
            "well_behaved": ("Lemma 1 precondition and conclusion hold, tank floor "
                             "holds, diagnostic 3 passes, excursion within "
                             f"{BOX_SLACK}x the 0.06 m box"),
        },
        "pose_family": {"phri2_q2_q4": PHRI2_Q24, "recommended_q2_q4": RECOMMENDED_Q24,
                        "q6": Q6, "lambda_grid": list(LAMBDA_GRID),
                        "task_inertia_on_push_axis": {str(k): v for k, v in inertia.items()}},
        "grid": {"K0": list(K0_GRID), "disturbance_scale": list(DISTURBANCE_GRID)},
        "n_cells": len(cells),
        "n_dual_axis": sum(c["dual_axis"] for c in cells),
        "n_well_behaved": sum(c["well_behaved"] for c in cells),
        "n_target": len(hits),
        "targets": hits,
        "cells": cells,
    }


def make_figure(report: dict, outdir: Path) -> Path:
    cells = report["cells"]
    lams = report["pose_family"]["lambda_grid"]
    k0s = report["grid"]["K0"]
    fig, axes = plt.subplots(2, 3, figsize=(14.0, 7.2))
    for row, scale in enumerate(report["grid"]["disturbance_scale"]):
        sel = [c for c in cells if c["disturbance_scale"] == scale]
        grids = {}
        for key in ("r_con_over_a_id", "r_auth_over_a_id"):
            g = np.full((len(lams), len(k0s)), np.nan)
            for c in sel:
                g[lams.index(c["lambda"]), k0s.index(c["K0"])] = 100 * c[key]
            grids[key] = g
        status = np.zeros((len(lams), len(k0s)))
        for c in sel:
            i, j = lams.index(c["lambda"]), k0s.index(c["K0"])
            status[i, j] = (3 if c["target"] else 2 if c["dual_axis"]
                            else 1 if c["well_behaved"] else 0)

        for col, (key, title, cmap) in enumerate((
                ("r_con_over_a_id", r"feasibility: $r_{con}/|a_{id}|$ [%]", "Reds"),
                ("r_auth_over_a_id", r"passivity: $r_{auth}/|a_{id}|$ [%]", "Blues"))):
            ax = axes[row, col]
            m = ax.pcolormesh(k0s, lams, grids[key], cmap=cmap, shading="nearest",
                              vmin=0, vmax=60)
            fig.colorbar(m, ax=ax)
            ax.set_title(f"{title}\ndisturbance {scale:g}x", fontsize=9)
        ax = axes[row, 2]
        m = ax.pcolormesh(k0s, lams, status, cmap="viridis", shading="nearest",
                          vmin=0, vmax=3)
        cb = fig.colorbar(m, ax=ax, ticks=[0, 1, 2, 3])
        cb.ax.set_yticklabels(["neither", "well behaved", "dual axis", "BOTH"],
                              fontsize=7)
        ax.set_title(f"verdict, disturbance {scale:g}x", fontsize=9)
        for ax in axes[row]:
            ax.set_xlabel(r"$K_0$ [N/m]")
            ax.set_ylabel(r"$\lambda$   (0 = phri2 pose, 1 = recommended)")
    fig.suptitle("Joint pose x $K_0$ scan: is there a well-behaved dual-axis "
                 "operating point?", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_joint_scan.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    report = run(workers=args.workers)
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_joint_scan.json").write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir)

    print(f"cells: {report['n_cells']}   dual-axis: {report['n_dual_axis']}   "
          f"well behaved: {report['n_well_behaved']}   BOTH: {report['n_target']}\n")
    print(f"{'lam':>5} {'K0':>6} {'D0':>6} {'dist':>5} {'r_con%':>7} {'r_auth%':>8} "
          f"{'dual':>5} {'ok':>4} {'headrm':>7} {'|e|mm':>6} {'(3)mm':>6}")
    for c in report["cells"]:
        print(f"{c['lambda']:>5.1f} {c['K0']:>6.0f} {c['D0']:>6.1f} "
              f"{c['disturbance_scale']:>5.0f} {100 * c['r_con_over_a_id']:>6.2f}% "
              f"{100 * c['r_auth_over_a_id']:>7.2f}% "
              f"{'yes' if c['dual_axis'] else '-':>5} "
              f"{'yes' if c['well_behaved'] else '-':>4} "
              f"{100 * c['anchor_headroom']:>6.1f}% "
              f"{1e3 * c['max_abs_e_axis_m']:>6.1f} {1e3 * c['diag3_e_ss_m']:>6.1f}")
    if report["targets"]:
        print("\nTARGET CELLS (dual-axis AND well behaved):")
        for c in report["targets"]:
            print(f"  lambda={c['lambda']:.1f} K0={c['K0']:.0f} D0={c['D0']:.1f} "
                  f"dist={c['disturbance_scale']:.0f}x  r_con {100 * c['r_con_over_a_id']:.1f}% "
                  f"r_auth {100 * c['r_auth_over_a_id']:.1f}%  headroom "
                  f"{100 * c['anchor_headroom']:.1f}%")
    else:
        print(f"\nNo target cell at the headline {BOX_SLACK}x box allowance.")
    print("\nverdict vs the workspace-box allowance (the judgement call):")
    for row in report["box_slack_sensitivity"]:
        who = ", ".join(f"lam{c['lambda']:.1f}/K{c['K0']:.0f}/d{c['disturbance_scale']:.0f}"
                        for c in row["cells"])
        print(f"  {row['box_slack']:.2f}x  ->  {row['n_target']} target(s)  {who or '-'}")
    print(f"\nwrote {args.outdir / 'pir_joint_scan.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
