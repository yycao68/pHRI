#!/usr/bin/env python3
"""Gravity-compensation-ONLY test -- deliberately bypasses the closed control
loop of BOTH controllers (run_hardware.py and run_pid.py).

Why this exists: if joint_sign is wrong, either closed loop can go
UNSTABLE (wild oscillation) rather than just fail quietly, because a wrong
sign flips the effective feedback loop from negative (stabilizing) to positive
(destabilizing) -- the controller "thinks" it's correcting the error but the
actual commanded current pushes further into it. That makes it hard to safely
iterate on sign/offset guesses using the full controller.

This script commands ONLY:
    tau = gravity_scale * dyn.gravity(q) - damping * dq
i.e. physics-based gravity compensation plus a small safety damping term.
There is NO position/velocity error feedback and no control law at all -- so there
is no possibility of closed-loop instability from a sign error. Worst case
with wrong sign/offset: the arm just isn't properly supported and sags/drifts
under real gravity (calm, not violent) -- diagnosable and safe.

Use this to nail down joint_sign / joint_offset_rad (the arm should feel like
it's "floating", roughly weightless, when correct) BEFORE running either
run_hardware.py or run_pid.py on real hardware.

SAFETY: arm supported by hand, current_limit_ticks kept low (from config),
torque disabled on any exit (normal or exception/Ctrl+C).

Usage:
    python3 tools/test_gravity_compensation.py --port COM3 --baud 1000000 \
        --config configs/hold.yaml --duration 15 --damping 0.15
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
from dynamixel_backend import DynamixelCurrentBackend  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--damping", type=float, default=0.15,
                     help="joint-space safety damping (Nm per rad/s); pure -b*dq, "
                          "prevents runaway if something unexpected happens; does "
                          "NOT depend on sign/offset correctness")
    ap.add_argument("--output", type=Path, default=None)
    args = ap.parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    kin = OpenManipulatorKinematics()
    dyn = OpenManipulatorDynamics()
    robot = config.get("robot", {})
    g_scale = float(robot.get("gravity_scale", 1.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [3.5, 3.0, 2.0]), dtype=float)

    fake_args = argparse.Namespace(port=args.port, baud=args.baud)
    backend = DynamixelCurrentBackend(config, port=args.port, baud=args.baud)

    fh = None; writer = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fh = open(args.output, "w", newline="")

    print(f"[gravity-only] port={args.port} baud={args.baud} damping={args.damping} "
          f"duration={args.duration}s (Ctrl+C to stop early)")
    print("This test ONLY compensates gravity -- no position control. The arm")
    print("should feel roughly weightless if joint_sign/joint_offset_rad are correct.\n")

    backend.enable()
    start = time.monotonic(); last_print = start; sample = 0
    try:
        while True:
            wall = time.monotonic()
            if wall - start >= args.duration:
                break
            io = backend.read_state()
            g = dyn.gravity(io.q)
            tau = g_scale * g - args.damping * io.dq
            tau = np.clip(tau, -tau_max, tau_max)
            backend.send_torque(tau)

            if fh is not None:
                row = {"sample": sample, "t": wall - start}
                for name, vec in (("q", io.q), ("dq", io.dq), ("tau", tau), ("cur", io.current_A)):
                    for i, v in enumerate(np.asarray(vec).reshape(-1)):
                        row[f"{name}_{i}"] = float(v)
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row.keys())); writer.writeheader()
                writer.writerow(row)

            if wall - last_print >= 0.3:
                print(f"\r[gravity-only] t={wall-start:5.1f}s  q={np.round(io.q,3)}  "
                      f"dq={np.round(io.dq,2)}  tau={np.round(tau,2)}   ", end="", flush=True)
                last_print = wall
            sample += 1
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        backend.disable()
        if fh is not None:
            fh.close()
        print(f"\n[gravity-only] torque disabled; {sample} samples")


if __name__ == "__main__":
    main()
