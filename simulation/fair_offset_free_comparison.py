"""Focused fair comparison for the offset-free interaction-error MPC paper.

The original C1--C5 numerical ratio mixes controller architecture with a large
difference in realized stiffness.  This script adds only the comparisons needed
to separate those effects:

* nominal 300 N/m impedance (the original C1 reference),
* response-matched impedance, calibrated against DI-MPC without estimation,
* tuned torque-aware anti-windup PI impedance,
* filtered measured-force cancellation on the response-matched impedance
  backbone,
* DI-MPC at 100 Hz, with and without the augmented Kalman estimate.

Calibration uses one trajectory cycle.  Reported test metrics use a separate
three-cycle run.  Every controller is evaluated at the same 1 kHz physics rate
and is clipped to the same published FR3 joint-torque limits.
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import phri
from fr3_impedance import cartesian_impedance_control, make_impedance_params
from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL, TAU_LIMIT


OUT_JSON = HERE / "fair_offset_free_comparison.json"
OUT_FIG = HERE.parent / "arXiv" / "figures" / "fair_offset_free_comparison.pdf"
R_D = np.eye(3)


@dataclass
class Baseline:
    kind: str
    stiffness: float = 300.0
    damping_ratio: float = 1.0
    integral_gain: float = 0.0
    integral_limit: float = 0.05
    antiwindup_gain: float = 0.5
    observer_time_constant: float = 0.05

    def __post_init__(self):
        self.params = make_impedance_params(
            k_pos=self.stiffness, k_rot=20.0,
            damping_ratio=self.damping_ratio, q_null=Q_NEUTRAL)
        self.integral = np.zeros(3)
        self.force_hat = np.zeros(3)

    def torque(self, env, state, dyn, p_d, dp_d, ddp_d, wrench):
        dx_d = np.r_[dp_d, np.zeros(3)]
        ddx_d = np.r_[ddp_d, np.zeros(3)]
        base = cartesian_impedance_control(
            state, dyn, p_d, R_D, dx_d, ddx_d, self.params)
        base += env.null_space_gravity_comp(dyn)
        if self.kind == "impedance":
            return base

        jv = dyn.J[:3]
        lam = np.linalg.inv(jv @ np.linalg.inv(dyn.M) @ jv.T + 1e-6 * np.eye(3))
        if self.kind == "pi":
            error = p_d - state.ee_pos
            candidate = self.integral + env.dt * error
            candidate = np.clip(candidate, -self.integral_limit, self.integral_limit)
            tau_i = jv.T @ (lam @ (self.integral_gain * candidate))
            tau_unsat = base + tau_i
            tau_sat = np.clip(tau_unsat, -TAU_LIMIT, TAU_LIMIT)
            # Back-calculate the task-acceleration correction created by joint
            # saturation.  This prevents the integrator from charging further
            # in a direction that the shared actuator budget cannot realize.
            wrench_residual = np.linalg.pinv(jv.T) @ (tau_sat - tau_unsat)
            accel_residual = np.linalg.solve(lam, wrench_residual)
            self.integral = candidate + env.dt * self.antiwindup_gain * (
                accel_residual / max(self.integral_gain, 1e-9))
            self.integral = np.clip(
                self.integral, -self.integral_limit, self.integral_limit)
            return base + jv.T @ (lam @ (self.integral_gain * self.integral))

        if self.kind == "force_observer":
            alpha = 1.0 - np.exp(-env.dt / self.observer_time_constant)
            self.force_hat += alpha * (wrench[:3] - self.force_hat)
            return base - jv.T @ self.force_hat

        raise ValueError(self.kind)


def run(spec, cycles: int, *, verbose: bool = False):
    env = FR3MuJoCoEnv(timestep=0.001)
    duration = cycles * phri.PERIOD
    n_steps = int(round(duration / env.dt))
    env.reset()
    if isinstance(spec, str):
        controller = phri.EpisodeController(spec, env, dt_mpc=0.01)
    else:
        controller = spec

    t_log = np.zeros(n_steps)
    err_log = np.zeros(n_steps)
    zerr_log = np.zeros(n_steps)
    tau_cmd_log = np.zeros((n_steps, 7))
    tau_app_log = np.zeros((n_steps, 7))
    dq_log = np.zeros((n_steps, 7))
    qp_solve_count = 0
    qp_failure_count = 0
    qp_update_mask = np.zeros(n_steps, dtype=bool)

    for i in range(n_steps):
        t = env.time
        p_d, dp_d, ddp_d = phri.circular_ref(t)
        wrench = phri.human_wrench(t)
        dyn, state = env.get_dynamics_and_state(f_ext_override=wrench)
        if np.any(wrench):
            env.apply_ee_wrench(wrench)
        if isinstance(spec, str):
            tau, _ = controller.compute(
                state, dyn, p_d, dp_d, ddp_d, R_D, wrench, i, t=t)
            if controller.mpc_ctrl is not None and i % controller.mpc_every == 0:
                qp_update_mask[i] = True
                qp_solve_count += 1
                if not controller.mpc_ctrl.last_qp_success:
                    qp_failure_count += 1
        else:
            tau = controller.torque(env, state, dyn, p_d, dp_d, ddp_d, wrench)
        tau_app = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        env.apply_torque(tau)
        env.step()

        t_log[i] = t
        delta = state.ee_pos - p_d
        err_log[i] = np.linalg.norm(delta)
        zerr_log[i] = abs(delta[2])
        tau_cmd_log[i] = tau
        tau_app_log[i] = tau_app
        dq_log[i] = state.dq

    metrics = phri._episode_metrics(t_log, err_log)
    t_cyc = t_log % phri.PERIOD
    contact = (t_cyc >= phri.T_FORCE_ON) & (t_cyc <= phri.T_FORCE_OFF)
    steady = (t_cyc >= phri.T_FORCE_OFF - 0.2) & (t_cyc <= phri.T_FORCE_OFF)
    magnitude_excess = np.abs(tau_cmd_log) - TAU_LIMIT[None, :]
    # OSQP's accepted feasibility tolerance is 1e-6; count only a physically
    # meaningful command excess (>1e-3 Nm) as magnitude saturation and report
    # the raw maximum excess separately.
    clipped = magnitude_excess > 1e-3
    power = tau_app_log * dq_log
    metrics.update({
        "z_ss_error": float(np.mean(zerr_log[steady])),
        "clip_sample_fraction": float(np.mean(np.any(clipped, axis=1))),
        "max_command_excess_Nm": float(max(0.0, np.max(magnitude_excess))),
        "clip_excess_threshold_Nm": 1e-3,
        "peak_applied_torque": float(np.max(np.abs(tau_app_log))),
        "rms_applied_torque": float(np.sqrt(np.mean(tau_app_log**2))),
        "peak_positive_joint_power": float(np.max(np.maximum(power, 0.0))),
        "positive_joint_energy": float(np.sum(np.maximum(power, 0.0)) * env.dt),
        "qp_solve_count": qp_solve_count,
        "qp_failure_count": qp_failure_count,
        "qp_failure_fraction": (
            float(qp_failure_count / qp_solve_count) if qp_solve_count else None),
        "magnitude_clip_fraction_at_qp_update": (
            float(np.mean(np.any(clipped[qp_update_mask], axis=1)))
            if np.any(qp_update_mask) else None),
        "magnitude_clip_fraction_between_qp_updates": (
            float(np.mean(np.any(clipped[~qp_update_mask], axis=1)))
            if np.any(qp_update_mask) else None),
    })
    if verbose:
        print(json.dumps(metrics, indent=2))
    return {"t": t_log, "error": err_log, "contact": contact, "metrics": metrics}


def response_match(target):
    # The equilibrium estimate gives the first principled stiffness guess;
    # damping is then selected by matching the entire contact-onset response.
    k_equilibrium = float(np.linalg.norm(phri.F_HUMAN) / target["metrics"]["ss_err"])
    onset = (target["t"] >= phri.T_FORCE_ON) & (target["t"] <= phri.T_FORCE_ON + 1.5)
    trials = []
    for scale in (0.8, 1.0, 1.2):
        for zeta in (0.7, 1.0, 1.3):
            stiffness = k_equilibrium * scale
            result = run(Baseline("impedance", stiffness, zeta), 1)
            shape_rmse = float(np.sqrt(np.mean(
                (result["error"][onset] - target["error"][onset]) ** 2)))
            steady_gap = abs(result["metrics"]["ss_err"] - target["metrics"]["ss_err"])
            score = shape_rmse + steady_gap
            trials.append((score, stiffness, zeta, shape_rmse, steady_gap))
    return min(trials), k_equilibrium, trials


def tune_pi():
    trials = []
    for gain in (120.0, 300.0, 700.0):
        for limit in (0.02, 0.06, 0.12):
            result = run(Baseline(
                "pi", integral_gain=gain, integral_limit=limit,
                antiwindup_gain=0.7), 1)
            m = result["metrics"]
            score = m["ss_err"] + 0.25 * m["rms_contact"] + 0.01 * m["clip_sample_fraction"]
            trials.append((score, gain, limit))
    return min(trials), trials


def serializable(result):
    return {"metrics": result["metrics"]}


def main():
    print("Calibration: DI-MPC 100 Hz target")
    target = run("DI-MPC 100 Hz", 1)
    match, k_equilibrium, match_trials = response_match(target)
    _, k_match, zeta_match, shape_rmse, steady_gap = match
    pi_choice, pi_trials = tune_pi()
    _, pi_gain, pi_limit = pi_choice
    print(f"Matched impedance: K={k_match:.1f} N/m, zeta={zeta_match:.2f}")
    print(f"Tuned AWPI: Ki={pi_gain:.1f}, integral limit={pi_limit:.3f} m s")

    specs = {
        "Nominal impedance": Baseline("impedance", 300.0, 1.0),
        "Response-matched impedance": Baseline("impedance", k_match, zeta_match),
        "Tuned anti-windup PI": Baseline(
            "pi", integral_gain=pi_gain, integral_limit=pi_limit,
            antiwindup_gain=0.7),
        "Matched impedance + measured-force cancellation": Baseline(
            "force_observer", stiffness=k_match, damping_ratio=zeta_match,
            observer_time_constant=0.05),
        "DI-MPC 100 Hz": "DI-MPC 100 Hz",
        "DI-MPC + Kalman 100 Hz": "DI-MPC + Kalman 100 Hz",
    }
    results = {}
    for label, spec in specs.items():
        print(f"Test: {label}")
        results[label] = run(spec, 3)

    payload = {
        "protocol": {
            "calibration_cycles": 1,
            "test_cycles": 3,
            "inner_rate_hz": 1000,
            "mpc_rate_hz": 100,
            "force_N": phri.F_HUMAN.tolist(),
            "joint_torque_limits_Nm": TAU_LIMIT.tolist(),
            "matched_impedance": {
                "equilibrium_stiffness_guess_N_per_m": k_equilibrium,
                "selected_stiffness_N_per_m": k_match,
                "selected_damping_ratio": zeta_match,
                "calibration_shape_rmse_m": shape_rmse,
                "calibration_steady_gap_m": steady_gap,
                "grid": [
                    {
                        "score_m": score,
                        "stiffness_N_per_m": stiffness,
                        "damping_ratio": zeta,
                        "shape_rmse_m": trial_shape_rmse,
                        "steady_gap_m": trial_steady_gap,
                    }
                    for score, stiffness, zeta, trial_shape_rmse, trial_steady_gap
                    in match_trials
                ],
            },
            "antiwindup_pi": {
                "selected_integral_gain": pi_gain,
                "selected_integral_limit_m_s": pi_limit,
                "back_calculation_gain": 0.7,
                "grid": [
                    {"score_m": score, "integral_gain": gain,
                     "integral_limit_m_s": limit}
                    for score, gain, limit in pi_trials
                ],
            },
        },
        "results": {key: serializable(value) for key, value in results.items()},
    }
    OUT_JSON.write_text(json.dumps(payload, indent=2) + "\n")

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.15, 2.65))
    colors = ["#8c8c8c", "#1f77b4", "#d62728", "#9467bd", "#ff7f0e", "#2ca02c"]
    plot_labels = {
        "Nominal impedance": "Nominal impedance",
        "Response-matched impedance": "Matched impedance",
        "Tuned anti-windup PI": "Tuned AWPI",
        "Matched impedance + measured-force cancellation": "Matched + measured force",
        "DI-MPC 100 Hz": "MPC",
        "DI-MPC + Kalman 100 Hz": "MPC + estimate",
    }
    for (label, result), color in zip(results.items(), colors):
        mask = (result["t"] >= 2.5) & (result["t"] <= 6.2)
        ax0.plot(result["t"][mask], 1e3 * result["error"][mask],
                 label=plot_labels[label], color=color, lw=1.2)
    ax0.axvspan(phri.T_FORCE_ON, phri.T_FORCE_OFF, color="0.9", zorder=-1)
    ax0.set(xlabel="Time (s)", ylabel=r"$\|p-p_d\|_2$ (mm)",
            xlim=(2.5, 6.2), ylim=(0, 55))
    ax0.grid(alpha=0.25)

    labels = list(results)
    ss = [1e3 * results[key]["metrics"]["ss_err"] for key in labels]
    ax1.barh(np.arange(len(labels)), ss, color=colors)
    ax1.set_yticks(np.arange(len(labels)), [
        "Nominal imp.", "Matched imp.", "AWPI", "Matched+force",
        "MPC", "MPC+estimate"])
    ax1.invert_yaxis()
    ax1.set_xlabel("Steady-state error (mm)")
    ax1.grid(axis="x", alpha=0.25)
    ax0.legend(fontsize=6.3, ncol=2, loc="upper left", frameon=False)
    fig.tight_layout(pad=0.5)
    OUT_FIG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_FIG, bbox_inches="tight")
    print(f"Wrote {OUT_JSON}")
    print(f"Wrote {OUT_FIG}")
    for label, result in results.items():
        m = result["metrics"]
        print(f"{label:30s} contact={1e3*m['rms_contact']:7.3f} mm "
              f"ss={1e3*m['ss_err']:7.3f} mm clip={100*m['clip_sample_fraction']:5.2f}%")


if __name__ == "__main__":
    main()
