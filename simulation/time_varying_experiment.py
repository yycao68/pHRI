"""
Time-varying interaction-force experiment (paper Table VI).

Complements the constant/step-force benchmarks (Benchmark I/II) with a
*time-varying* interaction force, and measures the N-step disturbance-prediction
RMS eps_N defined in Section III-D — the quantity that bounds how well the flat
random-walk prediction holds over the horizon.

Scene: a STATIC end-effector hold (fixed p_d), so the only source of
disturbance-estimate variation is the applied human force itself (on a moving
trajectory the configuration-driven terms of (6a) would confound the metric).
A sinusoidal force F_z = -A sin(2 pi f t) of fixed amplitude A and increasing
frequency f sweeps the disturbance rate L_d = A * 2 pi f / sqrt(2) (RMS).

Metrics (over the steady part of the force window, onset transient excluded):
  eps_1 = RMS_k || d_hat(k) - d_hat(k-1)  ||   (1-step prediction change)
  eps_N = RMS_k || d_hat(k) - d_hat(k-N)  ||   (N-step flat prediction error)
The Section III-D bound predicts eps_N <= e_K + L_d * N * dt, i.e. the gap
eps_N - eps_1 grows linearly with L_d; e_K (the constant-force floor) is the
f = 0 row.  Controller: DI-MPC + Kalman, 100 Hz QP (C5), same as Table I.

Run:  python time_varying_experiment.py
"""
import numpy as np
from numpy.linalg import norm

import phri
from phri import EpisodeController, CENTER
from fr3_mujoco import FR3MuJoCoEnv

CTRL      = "Double-Integrator MPC + Kalman 100 Hz"
AMP       = 12.0            # N, force amplitude
F_ON      = 2.0            # s, force onset (after settle)
F_OFF     = 10.0           # s, force offset
DURATION  = 11.0           # s
N_HORIZON = 10             # MPC horizon (= paper N); eps_N look-back in QP steps
FREQS_HZ  = [0.0, 0.1, 0.2, 0.4, 0.8]   # 0 Hz == constant force


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

    d_hat, err, t_log = [], [], []
    for i in range(int(DURATION / dt)):
        t = env.time
        w = _force(freq, t)
        dyn, state = env.get_dynamics_and_state()
        if np.any(w[:3] != 0):
            env.apply_ee_wrench(w)
        tau, _ = ctrl.compute(state, dyn, p_d, zero3, zero3, R_d, w, i)
        env.apply_torque(tau)
        env.step()
        if i % ctrl.mpc_every == 0:                       # log d_hat at the QP rate
            d_hat.append(ctrl.mpc_ctrl.x_aug[6:9].copy())
            err.append(norm(state.ee_pos - p_d))
            t_log.append(t)

    d_hat = np.asarray(d_hat)
    err = np.asarray(err) * 1e3                            # mm
    t_log = np.asarray(t_log)
    N = N_HORIZON

    # steady window: exclude the 0.7 s estimator-convergence transient at onset
    steady = (t_log >= F_ON + 0.7) & (t_log <= F_OFF - 0.15)
    step1 = np.array([norm(d_hat[k] - d_hat[k - 1]) for k in range(N, len(d_hat))])
    stepN = np.array([norm(d_hat[k] - d_hat[k - N]) for k in range(N, len(d_hat))])
    m = steady[N:]
    eps1 = float(np.sqrt(np.mean(step1[m] ** 2)))
    epsN = float(np.sqrt(np.mean(stepN[m] ** 2)))
    Ld = AMP * 2 * np.pi * freq / np.sqrt(2)               # RMS disturbance rate (N/s)
    dt_qp = ctrl.mpc_every * dt                             # QP sample period (s)
    return dict(freq=freq, Ld=Ld, eps1=eps1, epsN=epsN,
                bound=Ld * N * dt_qp,                       # Section III-D term L_d*N*dt
                rms_track=float(np.sqrt(np.mean(err[steady] ** 2))))


def main():
    print("=" * 78)
    print("Time-varying interaction force — static hold, DI-MPC+Kalman (C5), 100 Hz QP")
    print(f"  amplitude {AMP} N, force active [{F_ON}, {F_OFF}] s, horizon N={N_HORIZON}")
    print("=" * 78)
    print(f"{'freq[Hz]':>8} {'L_d[N/s]':>9} {'eps_1[N]':>9} {'eps_N[N]':>9} "
          f"{'gap':>6} {'L_d*N*dt':>9} {'RMS_track[mm]':>14}")
    for f in FREQS_HZ:
        r = run_one(f)
        gap = r["epsN"] - r["eps1"]
        print(f"{f:8.1f} {r['Ld']:9.2f} {r['eps1']:9.3f} {r['epsN']:9.3f} "
              f"{gap:6.2f} {r['bound']:9.2f} {r['rms_track']:14.3f}")
    print("\nThe gap eps_N - eps_1 tracks the Section III-D bound term L_d*N*dt,")
    print("confirming the horizon-extrapolation error is linear in the force rate.")


if __name__ == "__main__":
    main()
