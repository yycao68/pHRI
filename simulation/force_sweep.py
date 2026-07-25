#!/usr/bin/env python3
"""Human-force sweep (Sec. VI-E).

Varies the magnitude and shape of the sustained human force and records the
steady-state and peak end-effector deflection, for the proposed offset-free
predictive interaction-dynamics controller (D7: DI-MPC + Kalman,
500 Hz) versus classical impedance (D1). Reuses the FR3 MuJoCo plant,
controllers, and metric definitions of phri.py; only the human-force profile
is varied (monkeypatched into phri.F_HUMAN / phri.human_wrench).

Run:  python3 force_sweep.py
Writes force_sweep.json and force_sweep.png.
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import phri
from phri import FR3MuJoCoEnv, run_episode, MPC_DT_FAST

PROPOSED  = "DI-MPC + Kalman 500 Hz"
CLASSICAL = "Impedance"


def _run(env, name, n_cycles=1):
    return run_episode(name, env, n_cycles=n_cycles,
                       hifreq_dt=MPC_DT_FAST, verbose=False)


def magnitude_sweep(env, mags):
    """Constant (step) human force of increasing magnitude."""
    out = {CLASSICAL: {"ss": [], "peak": []}, PROPOSED: {"ss": [], "peak": []}}
    print("== magnitude sweep (constant/step force) ==")
    for mag in mags:
        phri.F_HUMAN = np.array([0.0, 0.0, -float(mag)])
        for name in (CLASSICAL, PROPOSED):
            m = _run(env, name)
            out[name]["ss"].append(abs(m["ss_err"]) * 1e3)   # mm
            out[name]["peak"].append(m["peak_defl"] * 1e3)   # mm
            print(f"  F={mag:5.1f} N  {name:38s}  "
                  f"SS={abs(m['ss_err'])*1e3:8.3f} mm  peak={m['peak_defl']*1e3:7.2f} mm")
    phri.F_HUMAN = np.array([0.0, 0.0, -15.0])  # restore default
    return out


def shape_sweep(env, mag=15.0):
    """Fixed-magnitude force with different temporal shapes."""
    T_ON, T_OFF, P = phri.T_FORCE_ON, phri.T_FORCE_OFF, phri.PERIOD

    def _wrench(fz):
        return np.concatenate([[0.0, 0.0, fz], np.zeros(3)])

    def step(t):
        tc = t % P
        return _wrench(-mag) if T_ON <= tc <= T_OFF else np.zeros(6)

    def ramp(t):
        tc = t % P
        if T_ON <= tc <= T_OFF:
            return _wrench(-mag * (tc - T_ON) / (T_OFF - T_ON))   # 0 -> mag
        return np.zeros(6)

    def sine(t):
        tc = t % P
        if T_ON <= tc <= T_OFF:
            return _wrench(-mag * np.sin(2 * np.pi * 1.0 * (tc - T_ON)))  # 1 Hz
        return np.zeros(6)

    profiles = {"step": step, "ramp": ramp, "sine_1Hz": sine}
    orig = phri.human_wrench
    res = {}
    print(f"== shape sweep ({mag:.0f} N) ==")
    for pname, fn in profiles.items():
        phri.human_wrench = fn
        row = {}
        for name in (CLASSICAL, PROPOSED):
            m = _run(env, name)
            row[name] = {"rms_contact": m["rms_contact"] * 1e3,
                         "peak": m["peak_defl"] * 1e3}
            print(f"  shape={pname:9s} {name:38s} "
                  f"RMS_c={m['rms_contact']*1e3:7.2f} mm  peak={m['peak_defl']*1e3:7.2f} mm")
        res[pname] = row
    phri.human_wrench = orig
    return res


def main():
    env = FR3MuJoCoEnv(timestep=0.001)
    mags = [5, 10, 15, 20, 25]

    mag_res = magnitude_sweep(env, mags)
    shape_res = shape_sweep(env, 15.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.4))
    ax1.plot(mags, mag_res[CLASSICAL]["ss"], "o-", color="#d62728", label="Classical impedance (D1)")
    ax1.plot(mags, mag_res[PROPOSED]["ss"],  "s-", color="#2ca02c", label="Proposed (D7, offset-free)")
    ax1.set_xlabel("Human force magnitude (N)")
    ax1.set_ylabel("Steady-state deflection (mm)")
    ax1.set_title("(a) Steady-state deflection")
    ax1.legend(fontsize=8); ax1.grid(alpha=.3)

    ax2.plot(mags, mag_res[CLASSICAL]["peak"], "o-", color="#d62728", label="Classical impedance (D1)")
    ax2.plot(mags, mag_res[PROPOSED]["peak"],  "s-", color="#2ca02c", label="Proposed (D7)")
    ax2.set_xlabel("Human force magnitude (N)")
    ax2.set_ylabel("Peak deflection (mm)")
    ax2.set_title("(b) Peak deflection during contact")
    ax2.grid(alpha=.3)

    fig.tight_layout()
    out_png = Path(__file__).parent / "force_sweep.png"
    fig.savefig(out_png, dpi=150)
    print(f"\nfigure  -> {out_png}")

    out_json = Path(__file__).parent / "force_sweep.json"
    out_json.write_text(json.dumps(
        {"mags_N": mags, "magnitude_mm": mag_res, "shape_15N_mm": shape_res}, indent=2))
    print(f"results -> {out_json}")


if __name__ == "__main__":
    main()
