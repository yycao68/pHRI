#!/usr/bin/env python3
"""Impedance-backbone architecture: standard benchmark + saturation stress test.

Companion code for the impedance-backbone (C8) ablation in
arXiv/phri_combined.tex (the retired double_integrator_phri_ieee.md draft
carried the same material)
Sec. VI-I / arXiv/phri_main.tex's "Correction-Authority Robustness" ablation
(Table VIII). Controller IDs below match the PAPER's numbering (Table
"Benchmarked Controllers", Sec. VI-A) -- NOT the internal "C5/C6" shorthand
used in earlier drafts of the retired stable_backbone_mpc.md design note, which predates this being
folded into the manuscript and would collide with the paper's own C6
("DI-MPC, 500 Hz, no Kalman"). Compares three controllers on the shared FR3
circular-trajectory + step-force scenario of phri.py:

    C1  Impedance                      classical baseline (D1 in the paper)
    C7  DI-MPC + Kalman 500 Hz         current proposed controller (D7)
    C8  DI-MPC + Kalman + Backbone 500 Hz   fixed restoring/damped
                                        impedance backbone commanded
                                        independently of the QP, which only
                                        shapes a
                                        bounded additional correction
                                        (backbone_track=True), plus the
                                        torque-realizability constraint
                                        extended to the whole horizon
                                        (horizon_torque_constraint=True)

Experiment 1 (normal_benchmark): the standard 1-cycle push benchmark, no
saturation stress -- checks C8 loses little/nothing relative to C7 when the
QP is never actually saturated.

Experiment 2 (fmax_stress_sweep): sweeps the MPC corrective-force bound
F_max down to (and including) 0 N via phri.F_MAX_OVERRIDE, emulating
increasingly severe loss of additive corrective-force authority. This is
the key comparison: C7's entire corrective torque comes from the box-
constrained F_mpc, so F_max -> 0 leaves ONLY tau_ff applied (no restoring
stiffness at all); C8's corrective torque is backbone (QP-independent) +
bounded additional term, so F_max -> 0 degrades gracefully to the restoring
backbone alone. This is an authority-loss test, not a
runtime injection of solver timeout/crash behavior.

Run:  python3 stable_backbone_comparison.py --n-cycles 3
Writes stable_backbone_comparison_3cycle.json/.png (the 3-cycle run is what
matches the paper's Benchmark I / Table VIII protocol).
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import phri
from phri import FR3MuJoCoEnv, run_episode, MPC_DT_FAST

CLASSICAL = "Impedance"
PROPOSED  = "DI-MPC + Kalman 500 Hz"
BACKBONE  = "DI-MPC + Kalman + Backbone 500 Hz"


_SCALAR_KEYS = ("rms_total", "rms_contact", "peak_defl", "ss_err")


def _run(env, name, n_cycles=1):
    """run_episode() also returns full per-sample logs (t, ee_pos, tau, ...);
    keep only the four scalar metrics so results stay JSON-serializable."""
    m = run_episode(name, env, n_cycles=n_cycles,
                    hifreq_dt=MPC_DT_FAST, verbose=False)
    return {k: m[k] for k in _SCALAR_KEYS}


def normal_benchmark(env, n_cycles=1):
    """Standard push benchmark, F_max at its normal 150 N default.

    n_cycles=3 reproduces the paper's exact Benchmark I protocol (Table I:
    3 push events over 24 s, metrics averaged over all three) so C8's
    numbers are directly comparable to the published C1/C7 rows, not just
    to the 1-cycle snapshot used in the original single-seed pass."""
    print(f"== normal benchmark (F_max=150 N, no induced saturation, "
          f"{n_cycles} cycle(s)) ==")
    out = {}
    for name in (CLASSICAL, PROPOSED, BACKBONE):
        m = _run(env, name, n_cycles=n_cycles)
        out[name] = m
        print(f"  {name:38s}  RMS={m['rms_total']*1e3:7.2f} mm  "
              f"RMS_c={m['rms_contact']*1e3:7.2f} mm  "
              f"peak={m['peak_defl']*1e3:7.2f} mm  SS={m['ss_err']*1e3:7.3f} mm")
    return out


def fmax_stress_sweep(env, f_max_values, n_cycles=1):
    """Sweep the additive corrective-force bound down to 0 N.

    n_cycles=3 averages each F_max condition over three push events (same
    protocol as normal_benchmark) instead of the single-event snapshot, so
    the degrade-gracefully claim isn't resting on one push realization."""
    print(f"\n== F_max stress sweep (loss of additive correction authority, "
          f"{n_cycles} cycle(s)) ==")
    out = {PROPOSED: {"rms_contact": [], "peak": [], "ss": []},
           BACKBONE: {"rms_contact": [], "peak": [], "ss": []}}
    for f_max in f_max_values:
        phri.F_MAX_OVERRIDE = float(f_max)
        for name in (PROPOSED, BACKBONE):
            m = _run(env, name, n_cycles=n_cycles)
            out[name]["rms_contact"].append(m["rms_contact"] * 1e3)
            out[name]["peak"].append(m["peak_defl"] * 1e3)
            out[name]["ss"].append(m["ss_err"] * 1e3)
            print(f"  F_max={f_max:6.1f} N  {name:38s}  "
                  f"RMS_c={m['rms_contact']*1e3:8.2f} mm  "
                  f"peak={m['peak_defl']*1e3:8.2f} mm  "
                  f"SS={m['ss_err']*1e3:8.3f} mm")
    phri.F_MAX_OVERRIDE = None  # restore default
    return out


def main(n_cycles=1, suffix=""):
    env = FR3MuJoCoEnv(timestep=0.001)

    normal_res = normal_benchmark(env, n_cycles=n_cycles)

    f_max_values = [150.0, 20.0, 5.0, 1.0, 0.0]
    stress_res = fmax_stress_sweep(env, f_max_values, n_cycles=n_cycles)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.6))
    ax1.plot(f_max_values, stress_res[PROPOSED]["rms_contact"], "o-",
             color="#E91E63", label="C7 / D7 (no backbone)")
    ax1.plot(f_max_values, stress_res[BACKBONE]["rms_contact"], "s-",
             color="#2ca02c", label="C8 (+ backbone)")
    ax1.set_xlabel("F_max (N)  [150 = normal, 0 = zero additive authority]")
    ax1.set_ylabel("Contact-window RMS error (mm)")
    ax1.set_title("(a) Tracking error vs. corrective-force bound")
    ax1.legend(fontsize=7); ax1.grid(alpha=.3)
    ax1.invert_xaxis()

    ax2.plot(f_max_values, stress_res[PROPOSED]["peak"], "o-",
             color="#E91E63", label="C7 / D7 (no backbone)")
    ax2.plot(f_max_values, stress_res[BACKBONE]["peak"], "s-",
             color="#2ca02c", label="C8 (+ backbone)")
    ax2.set_xlabel("F_max (N)  [150 = normal, 0 = zero additive authority]")
    ax2.set_ylabel("Peak deflection during contact (mm)")
    ax2.set_title("(b) Peak deflection vs. corrective-force bound")
    ax2.grid(alpha=.3)
    ax2.invert_xaxis()

    fig.tight_layout()
    out_png = Path(__file__).parent / f"stable_backbone_comparison{suffix}.png"
    fig.savefig(out_png, dpi=150)
    print(f"\nfigure  -> {out_png}")

    out_json = Path(__file__).parent / f"stable_backbone_comparison{suffix}.json"
    out_json.write_text(json.dumps(
        {"n_cycles": n_cycles,
         "normal_benchmark": normal_res,
         "f_max_values": f_max_values,
         "fmax_stress_sweep_mm": stress_res}, indent=2))
    print(f"results -> {out_json}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--n-cycles", type=int, default=1, help=
                   "1 = original single-push snapshot; 3 = paper's exact "
                   "Benchmark I protocol (Table I: 3 push events / 24 s, "
                   "metrics averaged over all three), writes to "
                   "*_3cycle.json/.png instead of overwriting the snapshot.")
    args = p.parse_args()
    suffix = f"_{args.n_cycles}cycle" if args.n_cycles != 1 else ""
    main(n_cycles=args.n_cycles, suffix=suffix)
