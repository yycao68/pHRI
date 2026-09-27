#!/usr/bin/env python3
"""Benchmark a control law's per-tick compute cost in isolation, for either
controller, so you can choose `dt` before going to hardware.

Why this is a separate script rather than a number read off a sim run: under
--backend sim the loop also integrates the plant (RNEA at 1 ms substeps), and
that cost does not exist on real hardware, where read_state() is a fast serial
read. Timing the full sim loop therefore mixes "how fast is this PC at the
control math" with "how fast is this PC at faking physics" -- and the second
part grows with dt, since a larger dt means more substeps per tick. That is
enough to make the measured per-tick time move the wrong way when you raise dt,
which is a property of the measurement rather than of the controller.

What is timed here is only the real per-tick work, the same calls both runners
make once per control tick regardless of backend: forward kinematics, the
Jacobian, the mass matrix and gravity (RNEA), the operational-space mass matrix
Lambda(q), the null-space projector, and the controller's own feedback step.

This number is a lower bound on what your loop costs. Add real serial I/O on
top: a GroupSyncRead plus a GroupSyncWrite round trip, typically a few
milliseconds on USB at 1 Mbps for a handful of servos, and worth measuring
rather than assuming.

Usage:
    python tools/benchmark_compute.py --config configs/hold.yaml --controller mpc
    python tools/benchmark_compute.py --config configs/hold.yaml --controller pid
    python tools/benchmark_compute.py --config configs/circle.yaml --controller mpc --n 2000
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

from interaction_mpc import (  # noqa: E402
    ControllerConfig, NormalizedInteractionMPC, RandomWalkDisturbanceObserver,
)
from pid_controller import PIDConfig, TaskSpacePID  # noqa: E402
from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--controller", choices=["mpc", "pid"], default="mpc",
                     help="which feedback law to time: run_hardware.py's or run_pid.py's")
    ap.add_argument("--n", type=int, default=500, help="number of ticks to time")
    ap.add_argument("--use-jit", action="store_true",
                     help="Numba-JIT the RNEA/FISTA hot paths (see run_hardware.py --help); "
                          "run this benchmark with and without the flag to see the real "
                          "speedup on this machine before deciding whether to use it live.")
    args = ap.parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    # The task space is the 2D x-z plane, not full 3D -- this arm's 3 joints all
    # rotate about the same axis, so the y column of the Jacobian is structurally
    # zero and a 3D Lambda(q) would be singular. Same convention as both runners.
    xz = [0, 2]
    dim = 2

    c = config.get("controller", {})
    dt = float(c.get("dt", 0.01))
    if args.controller == "mpc":
        cfg = ControllerConfig(
            dt=dt, horizon=int(c.get("horizon", 25)),
            q_pos=float(c.get("q_pos", 60.0)), q_vel=float(c.get("q_vel", 12.0)),
            r=float(c.get("r", 0.05)),
            u_max=np.asarray(c.get("u_max", [300.0] * 3), dtype=float)[:dim],
            observer_q_d=float(c.get("observer_q_d", 3.0)),
            observer_r_y=float(c.get("observer_r_y", 0.0004)),
            qp_iters=int(c.get("qp_iters", 200)),
            use_jit=args.use_jit,
        )
        cfg.dim = dim
        ctrl = NormalizedInteractionMPC(cfg)
        obs = RandomWalkDisturbanceObserver(dim, dt, cfg.observer_q_d, cfg.observer_r_y)
        label = "MPC + Kalman observer" + (" [jit]" if args.use_jit else "")
    else:
        p = config.get("pid", {})
        pcfg = PIDConfig(
            dim=dim, dt=float(p.get("dt", dt)),
            kp=float(p.get("kp", 20.0)), ki=float(p.get("ki", 0.0)), kd=float(p.get("kd", 6.0)),
            i_max=float(p.get("i_max", 0.02)),
            f_max=np.asarray(p.get("f_max", [8.0, 8.0]), dtype=float),
        )
        ctrl = TaskSpacePID(pcfg)
        obs = None
        label = "task-space PID" + (" [jit]" if args.use_jit else "")

    kin = OpenManipulatorKinematics()
    dyn = OpenManipulatorDynamics(use_jit=args.use_jit)
    if args.use_jit:
        t_warm = dyn.warmup() + (ctrl.warmup() if args.controller == "mpc" else 0.0)
        print(f"[jit] warm-up compile: {t_warm:.2f}s (excluded from the timed loop below)")

    robot = config.get("robot", {})
    lam_damp = float(robot.get("lambda_damping", 2e-3))
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))
    n = kin.n

    q = np.asarray(robot.get("sim_home_q_rad", robot.get("posture_q_rad", [-0.6, 0.3, 0.3])), dtype=float)
    dq = np.zeros(n)
    d_hat = np.zeros(dim)
    p_d = kin.fk(q)[xz]
    J_xz_prev = None

    times_ms = []
    print(f"Benchmarking {label} over {args.n} ticks "
          f"(config dt={dt}s, target {1.0 / dt:.0f} Hz)...")
    for _ in range(args.n):
        t0 = time.perf_counter()

        ee = kin.fk(q)
        J = kin.jacobian(q)
        J_xz = J[xz, :]
        Jdot_xz = (J_xz - J_xz_prev) / dt if J_xz_prev is not None else np.zeros_like(J_xz)
        J_xz_prev = J_xz.copy()
        ee_vel_xz = (J @ dq)[xz]
        y = ee[xz] - p_d

        Mq = dyn.mass_matrix(q) + armature * np.eye(n)
        Minv = np.linalg.inv(Mq)
        Lam = np.linalg.inv(J_xz @ Minv @ J_xz.T + lam_damp * np.eye(dim))
        N = np.eye(n) - J_xz.T @ (Lam @ J_xz @ Minv)

        if obs is not None:
            u = ctrl.solve(np.r_[y, ee_vel_xz], d_hat)
            d_hat, _, _ = obs.step(y, u)
            F_task = Lam @ u
        else:
            F_task = ctrl.step(y, ee_vel_xz)

        # Matches run_hardware.py/run_pid.py's actual per-tick call pattern: the
        # task-space Coriolis/centrifugal decoupling term mu_xz and the joint-space
        # bias torque share the SAME coriolis(q,dq) value, computed once.
        cor = dyn.coriolis(q, dq)
        mu_xz = Lam @ (J_xz @ Minv @ cor - Jdot_xz @ dq)
        tau = J_xz.T @ (F_task + mu_xz) + N @ np.zeros(n) + dyn.gravity(q) + cor

        times_ms.append(1000.0 * (time.perf_counter() - t0))
        # nudge the state so every tick is not a repeat of the same arithmetic
        q = q + 1e-4 * np.random.randn(n)

    times_ms = np.array(times_ms)
    budget_ms = 1000.0 * dt
    print(f"\n{label}: per-tick compute time (no sim physics, no serial I/O):")
    print(f"  mean   = {times_ms.mean():.3f} ms")
    print(f"  p50    = {np.percentile(times_ms, 50):.3f} ms")
    print(f"  p99    = {np.percentile(times_ms, 99):.3f} ms")
    print(f"  max    = {times_ms.max():.3f} ms")
    print(f"\nconfigured dt budget = {budget_ms:.1f} ms ({1.0 / dt:.0f} Hz)")
    # p99, not the mean: a control loop that misses its deadline one tick in a
    # hundred is a loop that misses its deadline.
    margin = budget_ms - np.percentile(times_ms, 99)
    if margin > budget_ms * 0.3:
        print(f"  -> comfortable margin ({margin:.1f} ms headroom at p99). "
              f"dt={dt} should be achievable; add real hardware I/O overhead on top.")
    elif margin > 0:
        print(f"  -> tight margin ({margin:.1f} ms headroom at p99). "
              f"Real serial round-trip time may push you over budget occasionally.")
    else:
        print(f"  -> OVER budget by {-margin:.1f} ms at p99. This dt is not achievable on this "
              f"PC; raise dt (lower the control frequency) or optimise the control law.")


if __name__ == "__main__":
    main()
