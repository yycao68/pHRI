"""Low-stiffness reference-holding: transient compliance then offset-free recovery.

Every other benchmark in the paper calibrates or tunes the realized
closed-loop stiffness fairly high (the calibrated-impedance comparison
lands on K=5441 N/m; the LQR-tuned MPC's own realized gain is comparable),
so peak deflection under the 15 N push is only a few mm -- closer to a
stiff trajectory tracker with a disturbance observer than to a controller
that visibly yields to contact and then recovers. This script deliberately
commands a LOW prescribed stiffness (K_d=100 N/m, matching the paper's own
"admittance" virtual-spring stiffness elsewhere) via the impedance-track
QP branch (ImpedanceMPCParams.impedance_track: F=K_d*e+D_d*edot-d_hat,
exact by construction when unconstrained -- see eq:unconstrained's
derivation) and compares WITH vs WITHOUT the Kalman disturbance estimate
at that SAME stiffness:

    NoEst  "ImpTrack MPC 100 Hz"            F=K_d*e+D_d*edot        (d_hat=0)
    Est    "ImpTrack MPC + Kalman 100 Hz"   F=K_d*e+D_d*edot-d_hat

D_d=2*zeta*sqrt(K_d) assumes a unit effective mass; Lambda^-1 is generally
anisotropic (same caveat already disclosed for the C8 backbone), so zeta=1
is only nominal critical damping and is visibly underdamped at this low
K_d (checked by tracing the raw error signal -- oscillates, does not
cleanly settle). zeta=3.0 was chosen by a small sweep {1,2,3,4,6} as the
smallest value giving clean, non-oscillatory settling.

Reports peak displacement, settling time (first time the error drops below
a tolerance band and stays there through force-off), steady-state error,
and peak positive joint power, for one representative cycle.

Run:  python3 low_stiffness_reference_holding.py
Writes low_stiffness_reference_holding.json.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import phri
from phri import FR3MuJoCoEnv

NO_EST = "ImpTrack MPC 100 Hz"
EST = "ImpTrack MPC + Kalman 100 Hz"
ZETA = 3.0
K_IMP = 100.0
SETTLE_TOL_M = 0.002  # 2 mm


def run_traced(name: str, n_cycles: int = 3) -> dict:
    env = FR3MuJoCoEnv(timestep=0.001)
    duration = n_cycles * phri.PERIOD
    n_steps = int(round(duration / env.dt))
    env.reset()
    ctrl = phri.EpisodeController(name, env, dt_mpc=0.01)

    t_log = np.zeros(n_steps)
    err_log = np.zeros(n_steps)
    tau_log = np.zeros((n_steps, 7))
    dq_log = np.zeros((n_steps, 7))

    for i in range(n_steps):
        t = env.time
        p_d, dp_d, ddp_d = phri.circular_ref(t)
        wrench = phri.human_wrench(t)
        dyn, state = env.get_dynamics_and_state(f_ext_override=wrench)
        if np.any(wrench):
            env.apply_ee_wrench(wrench)
        tau, _ = ctrl.compute(state, dyn, p_d, dp_d, ddp_d, np.eye(3), wrench, i, t=t)
        env.apply_torque(tau)
        env.step()
        t_log[i] = t
        err_log[i] = np.linalg.norm(p_d - state.ee_pos)
        tau_log[i] = tau
        dq_log[i] = state.dq

    # Representative cycle: the 2nd force event, well past initial transients.
    cyc = 1
    t0 = cyc * phri.PERIOD + phri.T_FORCE_ON
    t1 = cyc * phri.PERIOD + phri.T_FORCE_OFF
    mask = (t_log >= t0) & (t_log <= t1)
    t_rel = t_log[mask] - t0
    e = err_log[mask]

    peak = float(np.max(e))
    peak_t = float(t_rel[np.argmax(e)])
    ss = float(np.mean(e[t_rel >= (t1 - t0 - 0.2)]))

    below = e <= SETTLE_TOL_M
    settle_t = float('nan')
    for k in range(len(below)):
        if below[k:].all():
            settle_t = float(t_rel[k])
            break

    power = np.maximum(tau_log * dq_log, 0.0)
    peak_power = float(np.max(np.sum(power, axis=1)))

    return dict(peak_defl_mm=peak * 1e3, peak_time_s=peak_t,
                settle_time_s=settle_t, settle_tol_mm=SETTLE_TOL_M * 1e3,
                ss_err_mm=ss * 1e3, peak_positive_power_W=peak_power)


def main():
    phri.ZETA_IMP_OVERRIDE = ZETA
    phri.IMPEDANCE_K_OVERRIDE = K_IMP
    results = {}
    for label, name in (("No estimator", NO_EST), ("With estimator", EST)):
        print(f"Running {label} ({name}) ...")
        r = run_traced(name)
        results[label] = r
        print(f"  peak={r['peak_defl_mm']:.2f}mm @t={r['peak_time_s']:.2f}s  "
              f"settle(<{r['settle_tol_mm']:.0f}mm)={r['settle_time_s']:.2f}s  "
              f"ss={r['ss_err_mm']:.2f}mm  peak_power={r['peak_positive_power_W']:.1f}W")
    phri.ZETA_IMP_OVERRIDE = None
    phri.IMPEDANCE_K_OVERRIDE = None

    out = HERE / "low_stiffness_reference_holding.json"
    out.write_text(json.dumps({
        "k_imp_N_per_m": K_IMP, "zeta_imp": ZETA,
        "results": results,
    }, indent=2))
    print(f"\nresults -> {out}")


if __name__ == "__main__":
    main()
