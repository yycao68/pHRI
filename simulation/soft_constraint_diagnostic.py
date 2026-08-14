"""Diagnostic for the Soft/Hard torque-constraint contradiction flagged by
review: does fair_offset_free_comparison.run()'s max_command_excess_Nm
metric correctly capture torque excess relative to the TIGHTENED budget
(phri.TAU_MAX_SCALE * BASE_TAU_MAX) used in the QP constraint, or against
the plant's fixed physical limit TAU_LIMIT (unscaled)? And: when Hard's QP
reports infeasible, is that true primal infeasibility or a solver/iteration
artifact? When Soft avoids that failure, is its slack actually zero (pure
numerical-conditioning fix) or nonzero (genuine constraint relaxation)?

Reimplements the fair_offset_free_comparison.run() loop with per-tick
diagnostics instead of only aggregate metrics.
"""
import sys
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import phri
from fr3_mujoco import FR3MuJoCoEnv, TAU_LIMIT
from fair_offset_free_comparison import R_D


def run_diag(spec, cycles, tau_max_scale):
    env = FR3MuJoCoEnv(timestep=0.001)
    duration = cycles * phri.PERIOD
    n_steps = int(round(duration / env.dt))
    env.reset()
    controller = phri.EpisodeController(spec, env, dt_mpc=0.01)
    mpc = controller.mpc_ctrl
    true_tau_max = TAU_LIMIT * (tau_max_scale if tau_max_scale is not None else 1.0)

    status_counter = Counter()
    n_qp = 0
    n_fail = 0
    first_fail_step = None
    slack_max_log = []
    slack_rms_log = []
    excess_true_log = []   # excess over the TIGHTENED budget actually being tested
    excess_full_log = []   # excess over the FULL physical limit (what the paper's
                            # metric currently measures)
    per_tick = []  # (step, t, qp_success, status, slack_max, slack_rms, excess_true, excess_full)
    t_log = []
    err_log = []

    for i in range(n_steps):
        t = env.time
        p_d, dp_d, ddp_d = phri.circular_ref(t)
        wrench = phri.human_wrench(t)
        dyn, state = env.get_dynamics_and_state(f_ext_override=wrench)
        if np.any(wrench):
            env.apply_ee_wrench(wrench)
        tau, _ = controller.compute(state, dyn, p_d, dp_d, ddp_d, R_D, wrench, i, t=t)

        if mpc is not None and i % controller.mpc_every == 0:
            n_qp += 1
            ok = mpc.last_qp_success
            status = mpc.last_qp_status
            status_counter[status] += 1
            if not ok:
                n_fail += 1
                if first_fail_step is None:
                    first_fail_step = i
            slack = mpc.last_slack
            smax = float(np.max(slack)) if slack is not None else 0.0
            srms = float(np.sqrt(np.mean(slack**2))) if slack is not None else 0.0
            slack_max_log.append(smax)
            slack_rms_log.append(srms)
            exc_true = float(np.max(np.abs(tau) - true_tau_max))
            exc_full = float(np.max(np.abs(tau) - TAU_LIMIT))
            excess_true_log.append(exc_true)
            excess_full_log.append(exc_full)
            per_tick.append((i, t, ok, status, smax, srms, exc_true, exc_full))

        tau_app = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        env.apply_torque(tau)
        env.step()

        t_log.append(t)
        err_log.append(float(np.linalg.norm(state.ee_pos - p_d)))

    m = phri._episode_metrics(np.array(t_log), np.array(err_log))

    return {
        "rms_contact": m["rms_contact"],
        "n_qp": n_qp, "n_fail": n_fail,
        "fail_frac": n_fail / n_qp if n_qp else None,
        "status_counter": status_counter,
        "first_fail_step": first_fail_step,
        "slack_max": max(slack_max_log) if slack_max_log else None,
        "slack_rms_overall": float(np.sqrt(np.mean(np.array(slack_rms_log) ** 2))) if slack_rms_log else None,
        "slack_nonzero_frac": float(np.mean(np.array(slack_max_log) > 1e-6)) if slack_max_log else None,
        "excess_true_max": max(excess_true_log) if excess_true_log else None,
        "excess_full_max": max(excess_full_log) if excess_full_log else None,
        "per_tick": per_tick,
    }


def main():
    cycles = 3
    phri.SOFT_TORQUE_RHO_OVERRIDE = 1.0  # match the paper's reported rho=1 (code default is 1e4!)

    for scale in (0.32, 0.2):
        phri.TAU_MAX_SCALE = scale
        print(f"=== scale={scale}, cycles={cycles}, rho={phri.SOFT_TORQUE_RHO_OVERRIDE} ===\n")

        for label, name in [
            ("Hard(C5)", "DI-MPC + Kalman 100 Hz"),
            ("Unconstrained", "DI-MPC + Kalman + NoTauRow 100 Hz"),
            ("Soft", "DI-MPC + Kalman + Soft 100 Hz"),
        ]:
            r = run_diag(name, cycles, scale)
            print(f"--- {label} ({name}) ---")
            print(f"  contact RMS = {1e3*r['rms_contact']:.3f} mm")
            print(f"  qp solves={r['n_qp']}  failures={r['n_fail']} ({100*r['fail_frac']:.1f}%)")
            print(f"  status breakdown: {dict(r['status_counter'])}")
            print(f"  first_fail_step={r['first_fail_step']}")
            print(f"  slack: max={r['slack_max']}, rms={r['slack_rms_overall']}, nonzero_frac={r['slack_nonzero_frac']}")
            print(f"  excess vs TRUE tightened budget (scale*TAU_LIMIT): max={r['excess_true_max']:.4f} Nm")
            print(f"  excess vs FULL physical TAU_LIMIT (paper's current metric): max={r['excess_full_max']:.4f} Nm")
            print()

    phri.TAU_MAX_SCALE = None
    phri.SOFT_TORQUE_RHO_OVERRIDE = None


if __name__ == "__main__":
    main()
