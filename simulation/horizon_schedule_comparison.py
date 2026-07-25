#!/usr/bin/env python3
"""Reference-scheduled vs. frozen-Jacobian horizon torque constraint.

Companion code for stable_backbone_mpc.md (Remark in §2c / new §7). Compares
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
