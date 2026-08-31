"""
Time-varying interaction-force experiment (supplementary Table IV).

Complements the constant/step-force benchmarks (Benchmark I/II) with a
*time-varying* interaction force, and directly measures the one- and N-step
disturbance-prediction errors defined in Section III-D.

Scene: a STATIC end-effector hold (fixed p_d), so the only source of
disturbance-estimate variation is the applied human force itself (on a moving
trajectory the configuration-driven terms of (6a) would confound the metric).
A sinusoidal force F_z = -A sin(2 pi f t) of fixed amplitude A and increasing
frequency f sweeps the injected-force rate A * 2 pi f.  The reported theorem
bound uses an empirical pointwise rate of the reconstructed aggregate d_k,
not the injected-force rate alone.

At each 100 Hz manager instant, simulator-ground-truth acceleration is obtained
from the applied (clipped) torque and rigid-body dynamics, then

  d_acc,true = e_ddot + Lambda^-1 F_mpc,
  d_true     = -Lambda d_acc,true.

The flat prediction made at k is d_hat[k], so the direct i-step error is
RMS_k ||d_true[k+i] - d_hat[k]||.  We also retain the former estimator
self-consistency proxy for diagnostic comparison.  Controller: DI-MPC + Kalman,
100 Hz QP (C5), same as Table I.

Run:  python time_varying_experiment.py
"""
import json
from pathlib import Path

import numpy as np
from numpy.linalg import norm

import phri
from phri import EpisodeController, CENTER
from fr3_mujoco import FR3MuJoCoEnv
from fr3_mujoco import TAU_LIMIT

CTRL      = "DI-MPC + Kalman 100 Hz"
AMP       = 12.0            # N, force amplitude
F_ON      = 2.0            # s, force onset (after settle)
F_OFF     = 10.0           # s, force offset
DURATION  = 11.0           # s
N_HORIZON = 10             # MPC horizon (= paper N); eps_N look-back in QP steps
FREQS_HZ  = [0.0, 0.1, 0.2, 0.4, 0.8]   # 0 Hz == constant force
OUT_JSON = Path(__file__).with_name("time_varying_ground_truth_results.json")


def _force(freq: float, t: float) -> np.ndarray:
    w = np.zeros(6)
    if F_ON <= t <= F_OFF:
        w[2] = -AMP * (np.sin(2 * np.pi * freq * (t - F_ON)) if freq > 0 else 1.0)
    return w


def run_one(freq: float) -> dict:
    """Static hold under a sinusoidal push at frequency `freq`; return metrics."""
    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    ctrl = EpisodeController(CTRL, env, hifreq_dt=phri.MPC_DT_FAST)
    R_d = np.eye(3)
    dt = env.dt
    p_d = CENTER.copy()
    zero3 = np.zeros(3)

    d_hat, d_true, d_acc_true, err, t_log = [], [], [], [], []
    for i in range(int(DURATION / dt)):
        t = env.time
        w = _force(freq, t)
        dyn, state = env.get_dynamics_and_state()
        if np.any(w[:3] != 0):
            env.apply_ee_wrench(w)
        tau, F_mpc = ctrl.compute(state, dyn, p_d, zero3, zero3, R_d, w, i)
        tau_applied = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)

        # Direct ground truth from the simulated rigid-body dynamics at the
        # same instant as the manager state.  For static p_d, e_ddot=-p_ddot.
        M_inv = np.linalg.inv(dyn.M)
        qdd = M_inv @ (tau_applied + dyn.J.T @ w - dyn.Cq_dot)
        pdd = dyn.J[:3] @ qdd + dyn.dJ[:3] @ state.dq
        e_ddot = -pdd
        lambda_inv = dyn.J[:3] @ M_inv @ dyn.J[:3].T + 1e-6 * np.eye(3)
        d_acc = e_ddot + lambda_inv @ F_mpc
        d_force = -np.linalg.solve(lambda_inv, d_acc)

        env.apply_torque(tau)
        env.step()
        if i % ctrl.mpc_every == 0:                       # log d_hat at the QP rate
            d_hat.append(ctrl.mpc_ctrl.x_aug[6:9].copy())
            d_true.append(d_force.copy())
            d_acc_true.append(d_acc.copy())
            err.append(norm(state.ee_pos - p_d))
            t_log.append(t)

    d_hat = np.asarray(d_hat)
    d_true = np.asarray(d_true)
    d_acc_true = np.asarray(d_acc_true)
    err = np.asarray(err) * 1e3                            # mm
    t_log = np.asarray(t_log)
    N = N_HORIZON

    # Steady origin window: exclude estimator convergence and ensure the entire
    # N-step target remains inside the force window.
    dt_qp = ctrl.mpc_every * dt
    steady = ((t_log >= F_ON + 0.7)
              & (t_log <= F_OFF - 0.15 - N * dt_qp))
    step1 = np.array([norm(d_hat[k] - d_hat[k - 1]) for k in range(N, len(d_hat))])
    stepN = np.array([norm(d_hat[k] - d_hat[k - N]) for k in range(N, len(d_hat))])
    proxy_mask = steady[N:]
    proxy_eps1 = float(np.sqrt(np.mean(step1[proxy_mask] ** 2)))
    proxy_epsN = float(np.sqrt(np.mean(stepN[proxy_mask] ** 2)))

    origins = np.flatnonzero(steady)
    origins = origins[origins + N < len(d_true)]
    current_err_vec = np.array([d_true[k] - d_hat[k] for k in origins])
    current_err = np.linalg.norm(current_err_vec, axis=1)
    true_err1 = np.array([norm(d_true[k + 1] - d_hat[k]) for k in origins])
    true_errN = np.array([norm(d_true[k + N] - d_hat[k]) for k in origins])
    eK = float(np.sqrt(np.mean(current_err ** 2)))
    bias_vec = np.mean(current_err_vec, axis=0)
    bias_norm = float(norm(bias_vec))
    fluctuation_rms = float(np.sqrt(np.mean(np.sum(
        (current_err_vec - bias_vec) ** 2, axis=1))))
    eps1 = float(np.sqrt(np.mean(true_err1 ** 2)))
    epsN = float(np.sqrt(np.mean(true_errN ** 2)))
    injected_rate = AMP * 2 * np.pi * freq
    steady_idx = np.flatnonzero(steady)
    consecutive = steady_idx[np.isin(steady_idx + 1, steady_idx)]
    aggregate_rates = np.array([
        norm(d_true[k + 1] - d_true[k]) / dt_qp for k in consecutive
    ])
    Ld_emp = float(np.max(aggregate_rates))
    Ld_emp_p99 = float(np.quantile(aggregate_rates, 0.99))
    Ld_emp_rms = float(np.sqrt(np.mean(aggregate_rates ** 2)))
    extrapolation_term = Ld_emp * N * dt_qp
    return dict(
        freq=freq, injected_force_peak_rate_N_per_s=injected_rate,
        aggregate_Ld_emp_max_N_per_s=Ld_emp,
        aggregate_rate_p99_N_per_s=Ld_emp_p99,
        aggregate_rate_rms_N_per_s=Ld_emp_rms,
        current_estimation_rms=eK,
        current_estimation_bias_vector_N=bias_vec.tolist(),
        current_estimation_bias_norm_N=bias_norm,
        current_estimation_fluctuation_rms_N=fluctuation_rms,
        true_eps1=eps1, true_epsN=epsN,
        extrapolation_term=extrapolation_term,
        total_bound=eK + extrapolation_term,
        proxy_eps1=proxy_eps1, proxy_epsN=proxy_epsN,
        rms_d_acc_true=float(np.sqrt(np.mean(np.sum(d_acc_true[origins] ** 2, axis=1)))),
        rms_track=float(np.sqrt(np.mean(err[steady] ** 2))),
        prediction_origins=int(len(origins)),
    )


def main():
    print("=" * 78)
    print("Time-varying interaction force — static hold, DI-MPC+Kalman (C5), 100 Hz QP")
    print(f"  amplitude {AMP} N, force active [{F_ON}, {F_OFF}] s, horizon N={N_HORIZON}")
    print("=" * 78)
    print(f"{'freq[Hz]':>8} {'inj.rate':>9} {'e_K[N]':>8} {'true eps1':>9} "
          f"{'true epsN':>9} {'Ld_emp':>9} {'bound':>9} {'track[mm]':>10}")
    results = []
    for f in FREQS_HZ:
        r = run_one(f)
        results.append(r)
        print(f"{f:8.1f} {r['injected_force_peak_rate_N_per_s']:9.2f} "
              f"{r['current_estimation_rms']:8.3f} "
              f"{r['true_eps1']:9.3f} {r['true_epsN']:9.3f} "
              f"{r['aggregate_Ld_emp_max_N_per_s']:9.2f} {r['total_bound']:9.3f} "
              f"{r['rms_track']:10.3f}")
    payload = {
        "protocol": {
            "controller": CTRL, "amplitude_N": AMP,
            "force_window_s": [F_ON, F_OFF], "duration_s": DURATION,
            "horizon_steps": N_HORIZON, "manager_dt_s": 0.01,
            "frequencies_Hz": FREQS_HZ,
            "ground_truth_definition": "d_acc=e_ddot+Lambda_inv*F_mpc; d=-Lambda*d_acc",
            "injected_rate_definition": "peak pointwise rate A*2*pi*f",
            "bound_rate_definition": "max ||d_true[k+1]-d_true[k]||/manager_dt over settled aggregate-disturbance log",
        },
        "results": results,
    }
    OUT_JSON.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"\nWrote {OUT_JSON}")


if __name__ == "__main__":
    main()
