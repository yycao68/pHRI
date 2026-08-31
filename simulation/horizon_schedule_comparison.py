#!/usr/bin/env python3
"""Reference-scheduled vs. frozen-Jacobian horizon torque constraint.

FINDING: reference scheduling was investigated and NOT adopted. The
frozen-Jacobian formulation is retained. Full rationale and data are recorded
in "The horizon-scheduling negative result" at the bottom of this docstring;
that text was previously stable_backbone_mpc.md §7, which has been retired --
this script is where its companion code lives, so the result now sits next to
the code that produced it.

Compares
three controllers on the shared FR3 circular-trajectory + step-force
scenario of phri.py, all using the DEFAULT LQ-MPC branch (no backbone) so
the ONLY thing varied is how the horizon-wide torque-realizability
constraint is built:

    C5  DI-MPC + Kalman 500 Hz             baseline: constrains i=0 only (9b)
    C8  DI-MPC + Kalman + Frozen 500 Hz    horizon-wide, frozen at J_v(q_k)
    C9  DI-MPC + Kalman + Schedule 500 Hz  horizon-wide, coasted along
                                            q̄_i = q_k + i·dt·clip(q̇_k, ±cap)
                                            (horizon_torque_schedule=True)

Under the paper's normal 15 N push scenario, F_max=150 N is already the
binding constraint (see stable_backbone_comparison.py) and the torque rows
never actually saturate -- C5/C8/C9 are then indistinguishable regardless of
how the (inactive) torque constraint is built. To actually exercise the
frozen-vs-scheduled difference, this script sweeps tau_max down via
phri.TAU_MAX_SCALE so the torque-realizability rows start to bind while
F_max stays generous (150 N, so F_mpc's box constraint is never the
limiting factor -- isolates the torque-row behavior).

Run:  python3 horizon_schedule_comparison.py
Writes horizon_schedule_comparison.json and horizon_schedule_comparison.png.

================================================================================
The horizon-scheduling negative result (retired stable_backbone_mpc.md §7)
================================================================================

The frozen-Jacobian formulation is retained. A constant-velocity "coast"
extrapolation was tried first and removed again -- it added a second,
hard-to-justify approximation (why constant velocity? how large is the error?
why is it acceptable?) on top of the linearization/sampled-data/discrete-MPC
approximations already carried, for no measurable benefit. What replaced it --
reference scheduling along the trajectory generator's own (q_d, qd_d, qdd_d),
the theoretically cleaner choice -- was implemented and tested rather than
assumed, and turned out to be not merely unnecessary but measurably WORSE in
the one regime clean enough to judge. That is a stronger reason to keep the
frozen form than "no difference": scheduling carries a real downside here, not
just unrealized upside.

Reference-scheduled horizon model
---------------------------------
Along the horizon, robot-dependent quantities are evaluated on the nominal
reference trajectory:

    J_i = J_v(q_d,i),  Lambda_i = (J_i M(q_d,i)^-1 J_i^T)^-1,
    tau_ff,i = tau_ff(q_d,i, qd_d,i, qdd_d,i)

precomputed before the QP solve and held constant during optimization, so the
horizon-wide torque constraint stays affine in the decision variable:

    -tau_max <= tau_ff,i + J_i^T F_i <= tau_max,   i = 0..N-1

This controller has no joint-space q_d(t) of its own, though: circular_ref()
gives only a CARTESIAN reference p_d(t), and redundancy is resolved online (the
null-space centering term), not against a precomputed joint trajectory.
phri.precompute_joint_reference() builds one via closed-loop resolved-rate IK
sharing the controller's own redundancy objective, integrated once offline
before the episode, using the position/Jacobian-only
FR3MuJoCoEnv.shadow_kinematics() (~0.004 ms/call). xdd_d,i needs no such
construction -- it is exact from circular_ref() at t_k + i*dt_mpc.

The backbone's closed-loop prediction is scheduled consistently:
A_cl,i = A_d + B_d,i G_bb (_build_scheduled_closed_loop_horizon, an LTV
generalization of the frozen A_cl = A_d + B_d G_bb used by default), rather
than leaving the backbone prediction frozen while the torque map varies. Both
are unit-verified: the LTV builder reduces exactly to the frozen one when
unscheduled, and matches a manual forward simulation to 9e-16 for a genuinely
time-varying case.

Why it is not used: negligible when it would help, harmful when it matters
-------------------------------------------------------------------------
At the paper's 500 Hz / 10-step (N*dt_mpc = 20 ms) horizon, frozen and
reference-scheduled are indistinguishable under the normal 15 N push (both
0.151 / 0.758 / 0.0206 mm RMS / peak / SS) -- over 20 ms the Jacobian, inertia
and gravity vector simply do not change enough to matter.

Sweeping tau_max down (phri.TAU_MAX_SCALE, this script) so the constraint
actually binds tells the more important story:

    tau_max scale | Frozen RMS_c | Sched RMS_c | Frozen SS | Sched SS   [mm]
    --------------+--------------+-------------+-----------+---------
             1.00 |        0.151 |       0.151 |    0.0206 |   0.0206
             0.30 |        0.493 |       0.582 |    0.0215 |    0.567
             0.15 |       441.96 |      956.68 |     335.5 |    965.5

At scale 0.30 -- constraint clearly binding, system not otherwise collapsed,
the cleanest test point -- reference scheduling is WORSE: RMS up 18%,
steady-state error up 26x. The mechanism: q_d(t) is the UNDISTURBED reference,
but the torque constraint binds hardest exactly when the human push has
deflected the real arm away from it, so scheduling against the planned
configuration evaluates the wrong local model precisely when it matters. (At
0.15 both are in a separate failure mode -- the QP-independent
gravity/orientation/null-space part of tau_base alone approaches the scaled
tau_max, making the QP near-infeasible regardless of scheduling; scheduled is
worse there too, but that regime tests neither approach cleanly.)

An error-decay correction (q_i -> q_d,i + rho^i (q_k - q_d,k),
ImpedanceMPCParams.schedule_rho) mostly recovers the scale-0.30 gap at
rho ~ 0.9 (0.507 vs frozen 0.493 mm RMS) -- unsurprising, since high rho mostly
just tracks the actual state -- but does not beat frozen, and does not help in
deep saturation. It confirms the diagnosis rather than motivating adoption.

Conclusion
----------
Freezing the robot-dependent quantities over the horizon introduces negligible
error at this controller's 20 ms prediction horizon, verified experimentally
rather than assumed, so the frozen-Jacobian formulation is kept. Reference
scheduling remains implemented (ImpedanceMPCParams.horizon_torque_schedule,
phri.precompute_joint_reference) for a future revision with a longer horizon or
slower QP rate where 20 ms may stop being negligible, but is NOT recommended at
the current operating point: it adds implementation complexity (an offline
IK-based reference generator, shadow kinematics/dynamics queries, an untuned
correction parameter) while measurably regressing exactly where the torque
constraint matters most -- and even corrected, only ties the much simpler
frozen version.
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import phri
from phri import FR3MuJoCoEnv, run_episode, MPC_DT_FAST

BASELINE = "DI-MPC + Kalman 500 Hz"
FROZEN   = "DI-MPC + Kalman + Frozen 500 Hz"
SCHEDULE = "DI-MPC + Kalman + Schedule 500 Hz"

_SCALAR_KEYS = ("rms_total", "rms_contact", "peak_defl", "ss_err")


def _run(env, name, n_cycles=1):
    m = run_episode(name, env, n_cycles=n_cycles,
                    hifreq_dt=MPC_DT_FAST, verbose=False)
    return {k: m[k] for k in _SCALAR_KEYS}


def normal_benchmark(env, n_cycles=1):
    """tau_max at its full default -- checks Frozen/Schedule cost nothing
    relative to the baseline when the torque rows never actually bind."""
    print(f"== normal benchmark (tau_max scale=1.0, {n_cycles} cycle(s)) ==")
    out = {}
    for name in (BASELINE, FROZEN, SCHEDULE):
        m = _run(env, name, n_cycles=n_cycles)
        out[name] = m
        print(f"  {name:38s}  RMS_c={m['rms_contact']*1e3:7.3f} mm  "
              f"peak={m['peak_defl']*1e3:7.3f} mm  SS={m['ss_err']*1e3:7.4f} mm")
    return out


def tau_scale_sweep(env, scale_values, n_cycles=1):
    """Sweep tau_max down via phri.TAU_MAX_SCALE so the horizon-wide torque
    rows start to actually bind (F_max stays at its generous 150 N default
    throughout, so the force box constraint is never the limiting factor)."""
    print(f"\n== tau_max scale sweep ({n_cycles} cycle(s)) ==")
    out = {FROZEN: {"rms_contact": [], "peak": [], "ss": []},
           SCHEDULE: {"rms_contact": [], "peak": [], "ss": []}}
    for scale in scale_values:
        phri.TAU_MAX_SCALE = float(scale)
        for name in (FROZEN, SCHEDULE):
            m = _run(env, name, n_cycles=n_cycles)
            out[name]["rms_contact"].append(m["rms_contact"] * 1e3)
            out[name]["peak"].append(m["peak_defl"] * 1e3)
            out[name]["ss"].append(m["ss_err"] * 1e3)
            print(f"  tau_scale={scale:5.2f}  {name:38s}  "
                  f"RMS_c={m['rms_contact']*1e3:8.3f} mm  "
                  f"peak={m['peak_defl']*1e3:8.3f} mm  "
                  f"SS={m['ss_err']*1e3:8.4f} mm")
    phri.TAU_MAX_SCALE = None  # restore default
    return out


def main(n_cycles=1, suffix=""):
    env = FR3MuJoCoEnv(timestep=0.001)

    normal_res = normal_benchmark(env, n_cycles=n_cycles)

    scale_values = [1.0, 0.5, 0.3, 0.2, 0.15]
    sweep_res = tau_scale_sweep(env, scale_values, n_cycles=n_cycles)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.6))
    ax1.plot(scale_values, sweep_res[FROZEN]["rms_contact"], "o-",
             color="#E91E63", label="Frozen-Jacobian horizon constraint")
    ax1.plot(scale_values, sweep_res[SCHEDULE]["rms_contact"], "s-",
             color="#2ca02c", label="Reference-scheduled (coast) constraint")
    ax1.set_xlabel("tau_max scale  [1.0 = full FR3 limits]")
    ax1.set_ylabel("Contact-window RMS error (mm)")
    ax1.set_title("(a) Tracking error vs. torque-limit tightness")
    ax1.legend(fontsize=7); ax1.grid(alpha=.3)
    ax1.invert_xaxis()

    ax2.plot(scale_values, sweep_res[FROZEN]["peak"], "o-",
             color="#E91E63", label="Frozen-Jacobian horizon constraint")
    ax2.plot(scale_values, sweep_res[SCHEDULE]["peak"], "s-",
             color="#2ca02c", label="Reference-scheduled (coast) constraint")
    ax2.set_xlabel("tau_max scale  [1.0 = full FR3 limits]")
    ax2.set_ylabel("Peak deflection during contact (mm)")
    ax2.set_title("(b) Peak deflection vs. torque-limit tightness")
    ax2.grid(alpha=.3)
    ax2.invert_xaxis()

    fig.tight_layout()
    out_png = Path(__file__).parent / f"horizon_schedule_comparison{suffix}.png"
    fig.savefig(out_png, dpi=150)
    print(f"\nfigure  -> {out_png}")

    out_json = Path(__file__).parent / f"horizon_schedule_comparison{suffix}.json"
    out_json.write_text(json.dumps(
        {"n_cycles": n_cycles,
         "normal_benchmark": normal_res,
         "tau_scale_values": scale_values,
         "tau_scale_sweep_mm": sweep_res}, indent=2))
    print(f"results -> {out_json}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--n-cycles", type=int, default=1)
    args = p.parse_args()
    suffix = f"_{args.n_cycles}cycle" if args.n_cycles != 1 else ""
    main(n_cycles=args.n_cycles, suffix=suffix)
