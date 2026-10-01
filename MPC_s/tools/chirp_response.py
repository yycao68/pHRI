#!/usr/bin/env python3
"""Open-loop frequency-response (chirp) diagnostic -- NOT a controller test.

Why this exists: real hardware shows a tightly-clustered ~7.4-7.9Hz
sign-alternating, amplitude-growing oscillation on every diverged run
regardless of task or gain (MPC_s_Hardware_results_2026-09-29/
divergence_analysis.md). That frequency being gain-independent is
consistent with EITHER of two different things, which need different
fixes (next_steps_test_plan.md item 3/4):
  (a) a genuine MECHANICAL/structural resonance (gearbox backlash, link
      flexibility, mount compliance) that a high enough loop gain excites
      but does not create -- fix: a notch filter, or hardware work; or
  (b) a closed-loop CRITICAL FREQUENCY from the control loop's own total
      delay (dt + compute + comm) -- fix: reduce that delay (--use-jit,
      Return_Delay_Time, USB latency timer).
This script's only job is telling the two apart, by measuring the PLANT
(arm + servo + transmission) with the control loop entirely absent -- if
(a) is the cause, its signature survives with the loop removed; if (b) is
the cause, it cannot appear at all (there is no loop left to be delayed).

Commands ONLY (same approach as tools/test_gravity_compensation.py, which
read before using this):
    tau = gravity_scale * dyn.gravity(q) - damping * dq + chirp(t) [on ONE
          chosen joint only] + a joint-limit barrier (safety backstop,
          zero unless a joint strays near its limit)
i.e. NO position/velocity error feedback and no control law of any kind on
top of the chirp itself -- so whatever shows up in the measured response is
a property of the physical arm, not of this project's controller. The
chirp is a logarithmic (exponential) swept sine from --f0 to --f1 over
--duration seconds, amplitude --amplitude-nm (default 0.3 Nm, comfortably
under every shipped config's smallest tau_max_Nm entry):
    f(t)     = f0 * k**t,                 k = (f1/f0)**(1/duration)
    phase(t) = 2*pi*f0*(k**t - 1)/ln(k)
    chirp(t) = amplitude_nm * sin(phase(t))
A log sweep spends roughly equal time per OCTAVE rather than per Hz, which
matters here: --f0 3 --f1 15 (the default) is a ~2.3-octave span straddling
the ~8Hz band this is actually hunting for on both sides.

--f0 cannot be pushed much BELOW ~2Hz without tripping --max-dev-rad: with
no position feedback at all, a joint's response to a torque at frequency f
is (for the same torque amplitude) proportional to 1/f^2 -- this is just a
double integrator (tau -> M*qddot), not a resonance effect -- so an 0.5Hz
sweep start was tried and drifted past 0.3 rad in well under half a second
even at a small amplitude. This is a real, correct property of removing all
feedback, not a bug: it just means this script cannot usefully probe much
below a couple Hz, which is fine since ~8Hz is the only band in question.

Analysis (`--plot`): the instantaneous frequency is known analytically (it
is what was COMMANDED, not estimated), so the response is a sliding-window
RMS of the excited joint's measured velocity, time-aligned to the logged
`f_inst_hz` column -- not a full FFT-based transfer-function/phase
estimate. This is a deliberate complexity choice, matching this project's
other diagnostic tools (tools/benchmark_io.py, tools/plot_traj.py): good
enough to see whether there IS an amplitude peak near ~8Hz, which is the
actual question. A real Bode-style magnitude+phase estimate would be the
right next step only if this script finds something worth characterizing
more precisely.

SAFETY:
  - Same gravity-compensation-only design as tools/test_gravity_compensation.py
    -- no feedback law, so no possibility of closed-loop instability from a
    sign/offset error.
  - A hard abort (same pattern as run_hardware.py's --max-err-mm) if the
    excited joint's measured angle strays more than --max-dev-rad (default
    0.3 rad) from where it was right after move_to_start settled. This is
    NOT the thing being measured -- it is a backstop in case the chosen
    amplitude/damping turns out to be wrong for this arm.
  - `move_to_start` (same as run_hardware.py) brings the arm to the
    config's posture_q_rad first, so results are comparable across joints
    and across runs, and so the chirp never starts from an arbitrary pose.
  - Validate with `--backend sim` FIRST, always -- but know what it can and
    cannot tell you: sim's rigid-body model has no backlash/flexibility/
    mount compliance, so it HAS no mechanical resonance to find. A flat
    sim response does not mean the real arm's response is flat; sim here
    only validates this script's own plumbing (chirp generation, logging,
    the abort check, the plot), exactly like tools/benchmark_io.py's own
    docstring warns for its sim backend.

Usage:
    # 1. Validate the script itself in sim (expect a flat response -- sim has
    #    no mechanical resonance to find; this only checks the plumbing):
    python3 tools/chirp_response.py --backend sim --config configs/hold.yaml \
        --joint 1 --duration 30 --output results/chirp/sim_j1.csv
    python3 tools/chirp_response.py --plot results/chirp/sim_j1.csv

    # 2. On real hardware, one joint at a time (recommended: run all three):
    python3 tools/chirp_response.py --backend dynamixel --port COM3 \
        --config configs/hold.yaml --joint 0 --duration 30 \
        --output results/chirp/hw_j0.csv
    python3 tools/chirp_response.py --backend dynamixel --port COM3 \
        --config configs/hold.yaml --joint 1 --duration 30 \
        --output results/chirp/hw_j1.csv
    python3 tools/chirp_response.py --backend dynamixel --port COM3 \
        --config configs/hold.yaml --joint 2 --duration 30 \
        --output results/chirp/hw_j2.csv
    python3 tools/chirp_response.py --plot results/chirp/hw_j1.csv
    # A visible amplitude peak near ~7.4-7.9Hz on any joint supports the
    # mechanical-resonance explanation; a flat response on all three,
    # given the SAME closed-loop oscillation is real on hardware
    # (divergence_analysis.md), supports the loop-delay explanation instead.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from move_to_start import move_to_start  # noqa: E402
from dynamixel_backend import create_backend  # noqa: E402

# The band the real closed-loop oscillation was measured at -- drawn on the
# frequency-response plot as a reference, not used in the collection script.
OBSERVED_OSCILLATION_HZ = (7.4, 7.9)


def chirp_freq_and_tau(t: float, f0: float, f1: float, duration: float, amplitude_nm: float) -> tuple[float, float]:
    """Instantaneous frequency [Hz] and commanded torque [Nm] of a
    logarithmic (exponential) swept sine at time t (0 <= t <= duration)."""
    t = min(max(t, 0.0), duration)
    k = (f1 / f0) ** (1.0 / duration)
    f_inst = f0 * k ** t
    phase = 2.0 * np.pi * f0 * (k ** t - 1.0) / np.log(k)
    return f_inst, amplitude_nm * np.sin(phase)


def collect(args: argparse.Namespace) -> None:
    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    robot = config.get("robot", {})
    n = 3
    if not (0 <= args.joint < n):
        raise SystemExit(f"--joint must be 0, 1, or 2 (got {args.joint})")

    kin = OpenManipulatorKinematics()
    dyn = OpenManipulatorDynamics()
    backend = create_backend(args.backend, config, kin, args)

    g_scale = float(robot.get("gravity_scale", 1.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [3.5, 3.0, 2.0]), dtype=float)
    if args.amplitude_nm >= 0.5 * tau_max[args.joint]:
        raise SystemExit(
            f"--amplitude-nm {args.amplitude_nm} is not comfortably below joint {args.joint}'s "
            f"tau_max_Nm={tau_max[args.joint]} (need < {0.5 * tau_max[args.joint]:.2f} for this "
            f"script's own safety margin) -- this is a diagnostic probe, not a stress test; "
            f"pick a smaller amplitude.")
    jmin = np.asarray(robot.get("joint_min_rad", [-1.8, -1.6, -1.8]), dtype=float)
    jmax = np.asarray(robot.get("joint_max_rad", [1.6, 1.4, 1.8]), dtype=float)
    barrier_k = float(robot.get("joint_barrier_gain", 4.0))
    barrier_margin = float(robot.get("joint_barrier_margin_rad", 0.15))
    q_nom = np.asarray(robot.get("posture_q_rad", robot.get("sim_home_q_rad", [-0.6, 0.3, 0.3])), dtype=float)

    backend.enable()
    fh = None; writer = None; sample = 0
    try:
        move_to_start(
            backend, dyn, q_nom, dt=0.01, tau_max=tau_max,
            kp=float(robot.get("move_to_start_kp", 0.5)), ki=float(robot.get("move_to_start_ki", 0.05)),
            kd=float(robot.get("move_to_start_kd", 0.03)), gravity_scale=g_scale,
            timeout_s=args.move_to_start_timeout_s,
            pos_eps=float(robot.get("move_to_start_pos_eps_rad", 0.1)),
            vel_eps=float(robot.get("move_to_start_vel_eps_rad_s", 0.05)),
            settle_time_s=float(robot.get("move_to_start_settle_time_s", 1.0)),
            jmin=jmin, jmax=jmax, barrier_k=barrier_k, barrier_margin=barrier_margin)
        if hasattr(backend, "reset_time"):
            backend.reset_time()

        io0 = backend.read_state()
        q_start = io0.q.copy()
        print(f"[chirp] joint={args.joint}  f0={args.f0}Hz f1={args.f1}Hz duration={args.duration}s  "
              f"amplitude={args.amplitude_nm}Nm  q_start={np.round(q_start, 4)}")
        print(f"[chirp] abort if joint {args.joint} strays > {args.max_dev_rad} rad from q_start[{args.joint}]="
              f"{q_start[args.joint]:.4f}")

        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            fh = open(args.output, "w", newline="")

        start = time.monotonic(); next_tick = start; last_print = start
        while True:
            wall = time.monotonic()
            t = wall - start if args.backend == "dynamixel" else sample * 0.01
            if t >= args.duration:
                break

            io = backend.read_state()
            f_inst, chirp_tau = chirp_freq_and_tau(t, args.f0, args.f1, args.duration, args.amplitude_nm)

            dev = abs(float(io.q[args.joint] - q_start[args.joint]))
            if dev > args.max_dev_rad:
                print(f"\n[chirp] AUTO-STOP: joint {args.joint} deviation {dev:.3f} rad > "
                      f"--max-dev-rad {args.max_dev_rad} at t={t:.2f}s, f={f_inst:.2f}Hz -- "
                      f"this is a safety backstop, not the measurement. Lower --amplitude-nm "
                      f"or raise --damping and retry.")
                break

            over_hi = max(0.0, io.q[args.joint] - (jmax[args.joint] - barrier_margin))
            over_lo = max(0.0, (jmin[args.joint] + barrier_margin) - io.q[args.joint])
            tau = g_scale * dyn.gravity(io.q) - args.damping * io.dq
            tau[args.joint] += chirp_tau - barrier_k * over_hi + barrier_k * over_lo
            tau = np.clip(tau, -tau_max, tau_max)
            backend.send_torque(tau)

            if fh is not None:
                row = {"sample": sample, "t": t, "joint": args.joint, "f_inst_hz": f_inst,
                       "chirp_tau_nm": chirp_tau}
                for name, vec in (("q", io.q), ("dq", io.dq), ("tau", tau), ("cur", io.current_A)):
                    for i, v in enumerate(np.asarray(vec).reshape(-1)):
                        row[f"{name}_{i}"] = float(v)
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row.keys())); writer.writeheader()
                writer.writerow(row)

            sample += 1; next_tick += 0.01
            if wall - last_print >= 0.5:
                print(f"\r[chirp] t={t:6.2f}s  f={f_inst:5.2f}Hz  dev={dev:.3f}rad  "
                      f"dq[{args.joint}]={io.dq[args.joint]:+.3f}rad/s   ", end="", flush=True)
                last_print = wall
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        backend.disable()
        if fh is not None:
            fh.close()
        print(f"\n[chirp] torque disabled; {sample} samples")


def _load_csv(path: Path) -> dict:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    if d.shape == ():
        d = d.reshape(1)
    return {name: np.asarray(d[name]) for name in d.dtype.names}


def plot(path: Path, window_s: float, output: Path | None) -> None:
    import matplotlib.pyplot as plt

    d = _load_csv(path)
    joint = int(d["joint"][0])
    t = d["t"].astype(float)
    f_inst = d["f_inst_hz"].astype(float)
    dq = d[f"dq_{joint}"].astype(float)
    q = d[f"q_{joint}"].astype(float)
    tau = d[f"tau_{joint}"].astype(float)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    win = max(1, int(round(window_s / dt)))

    # Sliding-window RMS of dq, then mapped to the frequency the WINDOW
    # CENTER was at -- this is the "response amplitude vs frequency" curve.
    n = len(dq)
    centers, amps = [], []
    for i in range(0, n - win, max(1, win // 4)):  # 75%-overlapping windows
        seg = dq[i:i + win]
        amps.append(float(np.sqrt(np.mean(seg ** 2))))
        centers.append(f_inst[i + win // 2])
    centers = np.asarray(centers); amps = np.asarray(amps)
    order = np.argsort(centers)
    centers, amps = centers[order], amps[order]

    fig1, ax = plt.subplots(figsize=(7, 5))
    ax.plot(centers, amps, "-o", ms=3, color="tab:blue")
    ax.axvspan(*OBSERVED_OSCILLATION_HZ, color="tab:red", alpha=0.15,
               label=f"closed-loop oscillation band ({OBSERVED_OSCILLATION_HZ[0]}-"
                     f"{OBSERVED_OSCILLATION_HZ[1]}Hz, divergence_analysis.md)")
    ax.set_xscale("log")
    ax.set_xlabel("frequency [Hz] (log scale)"); ax.set_ylabel(f"RMS of dq[{joint}] [rad/s]")
    ax.set_title(f"open-loop frequency response -- joint {joint} ({path.name})\n"
                 f"a peak inside the red band supports a mechanical resonance; a flat\n"
                 f"curve there supports a loop-delay explanation instead")
    ax.legend(fontsize=8, loc="best")
    fig1.tight_layout()

    fig2, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    ax1.plot(t, q, color="tab:blue"); ax1.set_ylabel(f"q[{joint}] [rad]")
    ax1.set_title(f"raw time trace -- joint {joint} ({path.name})")
    ax2.plot(t, dq, color="tab:orange"); ax2.set_ylabel(f"dq[{joint}] [rad/s]")
    ax3.plot(t, tau, color="tab:green"); ax3.set_ylabel(f"tau[{joint}] [Nm]"); ax3.set_xlabel("t [s]")
    fig2.tight_layout()

    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        for suffix, fig in (("response", fig1), ("trace", fig2)):
            out = output.with_name(f"{output.stem}_{suffix}{output.suffix or '.png'}")
            fig.savefig(out, dpi=110)
            print(f"[chirp] saved {out}")
    else:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plot", type=Path, default=None,
                     help="instead of running, plot a previously-collected CSV (see --output below)")
    ap.add_argument("--plot-window-s", type=float, default=0.5,
                     help="sliding-window length [s] for the RMS response-vs-frequency plot")
    ap.add_argument("--plot-output", type=Path, default=None,
                     help="save plots as PNGs here (suffixed _response/_trace) instead of opening windows")
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--backend", choices=["sim", "dynamixel"], default="sim")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--joint", type=int, default=None,
                     help="which joint (0, 1, or 2) to inject the chirp into -- REQUIRED; "
                          "run once per joint (see module docstring's Usage)")
    ap.add_argument("--f0", type=float, default=3.0,
                     help="chirp start frequency [Hz] -- see module docstring for why this "
                          "can't be pushed much below ~2Hz without a feedback-free joint's "
                          "own 1/f^2 low-frequency response tripping --max-dev-rad")
    ap.add_argument("--f1", type=float, default=15.0, help="chirp end frequency [Hz]")
    ap.add_argument("--duration", type=float, default=30.0, help="chirp sweep duration [s]")
    ap.add_argument("--amplitude-nm", type=float, default=0.3,
                     help="chirp torque amplitude [Nm] -- must stay well under the excited "
                          "joint's tau_max_Nm (this script refuses amplitudes >= half of it)")
    ap.add_argument("--damping", type=float, default=0.15,
                     help="joint-space safety damping (Nm per rad/s), same role as "
                          "tools/test_gravity_compensation.py's --damping")
    ap.add_argument("--max-dev-rad", type=float, default=0.3,
                     help="abort if the excited joint strays more than this [rad] from its "
                          "post-move_to_start starting angle (safety backstop, not the "
                          "measurement)")
    ap.add_argument("--move-to-start-timeout-s", type=float, default=20.0)
    ap.add_argument("--output", type=Path, default=None)
    args = ap.parse_args()

    if args.plot:
        plot(args.plot, args.plot_window_s, args.plot_output)
        return
    if args.joint is None:
        raise SystemExit("--joint is required (0, 1, or 2) unless using --plot")
    if args.backend == "dynamixel" and not args.port:
        raise SystemExit("--port is required for --backend dynamixel")
    collect(args)


if __name__ == "__main__":
    main()
