#!/usr/bin/env python3
"""Independent verification of the PIR ``(K0, D0)`` go/no-go result.

``pir_knot_scan.py`` answers the design question on a 24 x 16 grid using one
measurement per diagnostic.  A NO-GO is a strong claim -- it kills the
passive-nominal split and with it the merged proof -- so this script attacks
the same result three more ways:

**Check A -- equilibrium by root-find, not by simulation.**
  Diagnostic 3 and diagnostic 1b are read off a settled closed-loop run.  An
  underdamped spring settles slowly, so "settled" is a judgement call.  This
  check instead solves ``qacc(q) = 0`` directly on MuJoCo's own equations of
  motion, with no time integration at all, and compares.  Two methods that
  disagree would mean the scan is measuring a transient.

**Check B -- the anchor at the fallback equilibrium is (K0, D0)-independent.**
  At the alpha -> 0 equilibrium the nominal force must balance the push:
  ``K0 e = F_h``, so ``J_v^T F_nom = -J_v^T F_h`` *whatever* ``K0`` is.  If
  that holds numerically, diagnostic 1b's failure is a property of the push
  and the pose, not of the gains -- which is the difference between "tune it
  harder" and "this envelope cannot hold this push".

**Check C -- refine the grid around the near miss.**
  The coarse grid could step over a thin feasible sliver.  It did: on a
  24 x 16 grid the phri2 derated-joint-4 envelope reports an empty region,
  and this refinement finds 14 feasible cells inside it.  The main scan's
  grid was densified in response; this check is the independent re-derivation
  of that window, run on its own linear grid rather than the scan's.

Run::

    python3 pir_verify.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy import optimize

import mujoco

import pir_common as pc
from pir_knot_scan import run_scan


# ---------------------------------------------------------------------------
# Check A -- static equilibrium solved on the plant, no integration
# ---------------------------------------------------------------------------


def equilibrium_by_rootfind(k: float, d: float, magnitude: float = pc.PUSH_MAGNITUDE_N) -> dict:
    """Solve ``qacc(q) = 0`` for the alpha -> 0 law under a constant push.

    Uses MuJoCo's own forward dynamics as the residual, so no equation of
    motion is re-derived here: the only thing this adds over the simulation is
    that it never integrates, and therefore cannot be fooled by a slow
    transient.
    """
    cfg = pc.FR3MPCConfig()
    imp_params = pc.make_default_impedance_params(cfg)
    env = pc.FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    _, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()
    q0 = env.q.copy()
    force = magnitude * pc.PUSH_AXIS

    def residual(q: np.ndarray) -> np.ndarray:
        env.data.qpos[: env.nv] = q
        env.data.qvel[: env.nv] = 0.0
        env.clear_applied_forces()
        mujoco.mj_forward(env.model, env.data)
        dyn, state = env.get_dynamics_and_state(
            f_ext_override=np.concatenate([force, np.zeros(3)])
        )
        tau_base, J_v, _ = pc.compute_tau_base(
            dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
        )
        e = state.ee_pos - p_nominal
        tau = tau_base + J_v.T @ pc.nominal_force(k, d, e, np.zeros(3))
        env.apply_torque(tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        mujoco.mj_forward(env.model, env.data)
        out = env.data.qacc[: env.nv].copy()
        env.clear_applied_forces()
        return out

    sol = optimize.root(residual, q0, method="hybr", tol=1e-10)
    q_eq = sol.x
    env.data.qpos[: env.nv] = q_eq
    env.data.qvel[: env.nv] = 0.0
    env.clear_applied_forces()
    mujoco.mj_forward(env.model, env.data)
    dyn, state = env.get_dynamics_and_state(
        f_ext_override=np.concatenate([force, np.zeros(3)])
    )
    tau_base, J_v, _ = pc.compute_tau_base(
        dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
    )
    e = state.ee_pos - p_nominal
    f_nom = pc.nominal_force(k, d, e, np.zeros(3))
    return {
        "converged": bool(sol.success),
        "residual_norm": float(np.linalg.norm(residual(q_eq))),
        "e": e,
        "e_axis": float(abs(e[pc.PUSH_AXIS_INDEX])),
        "anchor": tau_base + J_v.T @ f_nom,
        "tau_nom_from_f_nom": J_v.T @ f_nom,
        "tau_from_push": J_v.T @ force,
    }


def check_a(cells: list[tuple[float, float]]) -> dict:
    rows = []
    for k, d in cells:
        rf = equilibrium_by_rootfind(k, d)
        sim = pc.run_fallback_equilibrium(k, d)
        rows.append({
            "k": k,
            "d": d,
            "e_axis_rootfind_m": rf["e_axis"],
            "e_axis_simulated_m": sim["e_ss_axis"],
            "abs_gap_m": abs(rf["e_axis"] - sim["e_ss_axis"]),
            "rel_gap": abs(rf["e_axis"] - sim["e_ss_axis"]) / max(rf["e_axis"], 1e-12),
            "simulation_settled": bool(sim["settled"]),
            "rootfind_converged": rf["converged"],
            "rootfind_residual_norm": rf["residual_norm"],
            "anchor_joint4_rootfind_Nm": float(abs(rf["anchor"][pc.DERATED_JOINT])),
            "anchor_joint4_simulated_Nm": float(sim["anchor_max_abs"][pc.DERATED_JOINT]),
        })
    return {"rows": rows, "max_rel_gap": max(r["rel_gap"] for r in rows)}


# ---------------------------------------------------------------------------
# Check B -- the equilibrium anchor does not depend on (K0, D0)
# ---------------------------------------------------------------------------


def check_b(k_values: list[float], d: float = 25.0) -> dict:
    rows = []
    for k in k_values:
        rf = equilibrium_by_rootfind(k, d)
        # At equilibrium the nominal force balances the push, so J^T F_nom
        # should be the negative of the push's own joint torque.
        balance = rf["tau_nom_from_f_nom"] + rf["tau_from_push"]
        rows.append({
            "k": k,
            "e_axis_m": rf["e_axis"],
            "k_times_e_axis_N": float(k * rf["e_axis"]),
            "anchor_joint4_Nm": float(abs(rf["anchor"][pc.DERATED_JOINT])),
            "force_balance_residual_inf_Nm": float(np.abs(balance).max()),
        })
    anchors = np.array([r["anchor_joint4_Nm"] for r in rows])
    return {
        "rows": rows,
        "anchor_joint4_spread_Nm": float(anchors.max() - anchors.min()),
        "anchor_joint4_mean_Nm": float(anchors.mean()),
        "max_force_balance_residual_Nm": max(r["force_balance_residual_inf_Nm"] for r in rows),
        "joint4_cap_derated_Nm": pc.DERATED_LIMIT_NM,
        "joint4_cap_rho_Nm": float(pc.RHO * pc.TAU_LIMIT[pc.DERATED_JOINT]),
    }


# ---------------------------------------------------------------------------
# Check C -- dense local refinement around the near miss
# ---------------------------------------------------------------------------


def check_c(workers: int = 4) -> dict:
    k_fine = np.round(np.linspace(300.0, 500.0, 11), 1)  # deliberately NOT the main grid's points
    d_fine = np.round(np.linspace(8.0, 40.0, 9), 2)
    report = run_scan(k_fine, d_fine, workers=workers)
    return {
        "k_grid": k_fine.tolist(),
        "d_grid": d_fine.tolist(),
        "envelopes": {
            name: {
                "common_region_nonempty": block["common_region_nonempty"],
                "common_region_size": block["common_region_size"],
                "n_cells": block["n_cells"],
                "best_worst_normalized_violation": float(
                    np.maximum(
                        np.maximum(
                            np.array(block["diag1_anchor_ratio_replay"]),
                            np.array(block["diag1b_anchor_ratio_fallback"]),
                        ),
                        np.array(block["diag3_e_ss_axis_m"]) / pc.WORKSPACE_BOUND_M,
                    ).min()
                ),
            }
            for name, block in report["envelopes"].items()
        },
    }


def main() -> None:
    near_miss_cells = [(423.8, 21.3), (344.2, 21.3), (344.2, 5.9), (974.5, 40.7)]
    out = {
        "check_a_equilibrium_rootfind_vs_simulation": check_a(near_miss_cells),
        "check_b_anchor_independent_of_gains": check_b([100.0, 200.0, 400.0, 800.0, 1200.0]),
        "check_c_grid_refinement": check_c(),
    }
    path = pc.RESULTS / "pir_verify.json"
    path.write_text(json.dumps(out, indent=2, default=float))

    a = out["check_a_equilibrium_rootfind_vs_simulation"]
    print("CHECK A -- equilibrium: root-find vs simulation")
    for r in a["rows"]:
        print(f"  k={r['k']:7.1f} d={r['d']:5.1f}  root-find |e_z| = {r['e_axis_rootfind_m']:.5f} m, "
              f"simulated = {r['e_axis_simulated_m']:.5f} m  (rel gap {r['rel_gap'] * 100:.2f}%, "
              f"settled={r['simulation_settled']}, converged={r['rootfind_converged']})")
    print(f"  worst relative gap: {a['max_rel_gap'] * 100:.2f}%")

    b = out["check_b_anchor_independent_of_gains"]
    print("\nCHECK B -- anchor at the alpha->0 equilibrium vs K0")
    for r in b["rows"]:
        print(f"  k={r['k']:7.1f}  |e_z| = {r['e_axis_m']:.5f} m, k|e_z| = {r['k_times_e_axis_N']:6.2f} N, "
              f"|anchor|_j4 = {r['anchor_joint4_Nm']:6.2f} Nm, "
              f"force-balance residual = {r['force_balance_residual_inf_Nm']:.2e} Nm")
    print(f"  joint-4 anchor spread across a 12x range of k: {b['anchor_joint4_spread_Nm']:.3f} Nm "
          f"(mean {b['anchor_joint4_mean_Nm']:.2f} Nm)")
    print(f"  caps: derated-joint-4 {b['joint4_cap_derated_Nm']:.2f} Nm, rho=0.28 {b['joint4_cap_rho_Nm']:.2f} Nm")

    c = out["check_c_grid_refinement"]
    print(f"\nCHECK C -- refinement over k in [{c['k_grid'][0]}, {c['k_grid'][-1]}] x "
          f"d in [{c['d_grid'][0]}, {c['d_grid'][-1]}]")
    for name, block in c["envelopes"].items():
        print(f"  {name}: common region nonempty = {block['common_region_nonempty']} "
              f"({block['common_region_size']} / {block['n_cells']} cells); "
              f"best worst-normalized violation = {block['best_worst_normalized_violation']:.4f}")
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
