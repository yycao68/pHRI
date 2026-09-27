#!/usr/bin/env python3
"""Split a real control tick into read_state() / compute / send_torque(), so
"is the ~6.8ms per tick compute-bound or communication-bound?" has an actual
measured answer instead of staying an open hypothesis.

Why this exists: the original stage report's own Table 9 (Section 6.6) flags
this explicitly and admits it never checked it --

    "The PID's 6.73 ms is most likely communication-bound rather than
    computation-bound, since the control law itself is nearly free; that
    hypothesis has not been tested."

`tools/benchmark_compute.py` deliberately times ONLY the algorithm (no sim
physics, no serial I/O -- see its own docstring) and this session's timing
fix (implementation_fix.md) targeted exactly that algorithmic path (removing
redundant RNEA calls, `mass_matrix()` via a closed-form Jacobian formula
instead of 3 RNEA calls). That fix is only worth what it says it's worth if
the ~6.8ms real-hardware figure was actually dominated by compute -- if it
was instead mostly `read_state()`/`send_torque()` (GroupSyncRead/
GroupSyncWrite serial round trips), the earlier "~2.45x speedup -> ~2.8ms"
extrapolation in implementation_fix.md is optimistic, because compute was
never the majority of the number in the first place. This script settles
that by timing all three phases of the SAME real loop `run_hardware.py`/
`run_pid.py` actually run, separately, over many ticks.

On --backend sim, `send_torque()` genuinely is a free numpy assignment, but
`read_state()` is NOT free -- `SimArmBackend.read_state()` integrates the
plant's own physics (RNEA gravity/mass-matrix calls at 1ms substeps, the
same "faking physics" cost `tools/benchmark_compute.py`'s own docstring
already warns about) before returning. So on --backend sim, "read" time here
measures sim-substep integration cost, NOT communication -- it is neither
zero nor a stand-in for real serial I/O. This script is only useful for
exercising its own logic/output format before hardware is available; the
compute-vs-comm question this file exists to answer requires
--backend dynamixel on the real robot, where read_state()/send_torque() are
genuine GroupSyncRead/GroupSyncWrite round trips.

Usage:
    python tools/benchmark_io.py --config configs/hold.yaml --controller pid --backend sim
    python tools/benchmark_io.py --config configs/hold.yaml --controller mpc \\
        --backend dynamixel --port /dev/ttyUSB0 --baud 1000000 --n 500
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

from interaction_mpc import NormalizedInteractionMPC, RandomWalkDisturbanceObserver  # noqa: E402
from pid_controller import TaskSpacePID  # noqa: E402
from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from dynamixel_backend import assert_startup_pose_plausible, create_backend  # noqa: E402
from move_to_start import move_to_start  # noqa: E402

sys.path.insert(0, str(ROOT))
from run_hardware import controller_config  # noqa: E402
from run_pid import pid_config  # noqa: E402


def _percentiles(label: str, arr: np.ndarray, budget_ms: float) -> None:
    print(f"  {label:10s} mean={arr.mean():7.3f}ms  p50={np.percentile(arr,50):7.3f}ms  "
          f"p99={np.percentile(arr,99):7.3f}ms  max={arr.max():7.3f}ms")


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--controller", choices=["mpc", "pid"], default="mpc")
    ap.add_argument("--backend", choices=["sim", "dynamixel"], default="sim")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--n", type=int, default=500, help="number of control ticks to time")
    ap.add_argument("--move-to-start-timeout-s", type=float, default=20.0,
                     help="same as run_hardware.py -- <=0 skips it (sim only; do not skip "
                          "on real hardware, see docs/03_hardware_safety.md)")
    ap.add_argument("--use-jit", action="store_true",
                     help="Numba-JIT the RNEA/FISTA hot paths (see run_hardware.py --help) -- "
                          "run this both with and without the flag to see whether the compute "
                          "share of the tick (not read/send) actually shrinks on real hardware.")
    args = ap.parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    robot = config.get("robot", {})

    kin = OpenManipulatorKinematics()
    if "links" in config.get("kinematics", {}):
        kin.links.update(config["kinematics"]["links"])
    dyn = OpenManipulatorDynamics(use_jit=args.use_jit)
    n = kin.n
    backend = create_backend(args.backend, config, kin, args)

    xz = [0, 2]
    g_scale = float(robot.get("gravity_scale", 1.0))
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [3.5, 3.0, 2.0]), dtype=float)
    lam_damp = float(robot.get("lambda_damping", 2e-3))
    q_nom = np.asarray(robot.get("posture_q_rad", robot.get("sim_home_q_rad", [-0.6, 0.3, 0.3])),
                        dtype=float)
    post_kp = float(robot.get("posture_kp", 0.6))
    post_kd = float(robot.get("posture_kd", 0.12))

    if args.controller == "mpc":
        cfg = controller_config(config)
        cfg.dim = 2
        cfg.u_max = cfg.u_max[:2]
        cfg.use_jit = args.use_jit
        ctrl = NormalizedInteractionMPC(cfg)
        obs = RandomWalkDisturbanceObserver(2, cfg.dt, cfg.observer_q_d, cfg.observer_r_y,
                                             cfg.observer_d_hat_max)
        dt = cfg.dt
    else:
        pcfg = pid_config(config)
        ctrl = TaskSpacePID(pcfg)
        obs = None
        dt = pcfg.dt

    if args.use_jit:
        t_warm = dyn.warmup() + (ctrl.warmup() if args.controller == "mpc" else 0.0)
        print(f"[jit] warm-up compile: {t_warm:.2f}s (excluded from the timed loop below)")

    if args.backend == "dynamixel":
        jmin = np.asarray(robot.get("joint_min_rad", [-1.8, -1.6, -1.8]), dtype=float)
        jmax = np.asarray(robot.get("joint_max_rad", [1.6, 1.4, 1.8]), dtype=float)
        io_check = backend.read_state()
        assert_startup_pose_plausible(io_check.q, jmin, jmax,
                                       tol_rad=float(robot.get("startup_pose_max_overshoot_rad", 0.5)))

    backend.enable()
    try:
        if args.move_to_start_timeout_s > 0:
            barrier_k = float(robot.get("joint_barrier_gain", 4.0))
            barrier_margin = float(robot.get("joint_barrier_margin_rad", 0.15))
            jmin = np.asarray(robot.get("joint_min_rad", [-1.8, -1.6, -1.8]), dtype=float)
            jmax = np.asarray(robot.get("joint_max_rad", [1.6, 1.4, 1.8]), dtype=float)
            move_to_start(backend, dyn, q_nom, dt=dt, tau_max=tau_max,
                           kp=float(robot.get("move_to_start_kp", 0.10)),
                           ki=float(robot.get("move_to_start_ki", 0.05)),
                           kd=float(robot.get("move_to_start_kd", 0.03)),
                           gravity_scale=g_scale,
                           timeout_s=args.move_to_start_timeout_s,
                           pos_eps=float(robot.get("move_to_start_pos_eps_rad", 0.1)),
                           vel_eps=float(robot.get("move_to_start_vel_eps_rad_s", 0.05)),
                           settle_time_s=float(robot.get("move_to_start_settle_time_s", 2.0)),
                           jmin=jmin, jmax=jmax, barrier_k=barrier_k, barrier_margin=barrier_margin)
            if hasattr(backend, "reset_time"):
                backend.reset_time()
            if hasattr(backend, "resync"):
                backend.resync()

        io = backend.read_state()
        p0 = kin.fk(io.q)
        d_hat = np.zeros(2)
        J_xz_prev = None

        t_read = np.zeros(args.n)
        t_compute = np.zeros(args.n)
        t_send = np.zeros(args.n)

        print(f"Timing {args.n} real ticks: controller={args.controller} backend={args.backend} "
              f"config={args.config.name}")
        for k in range(args.n):
            c0 = time.perf_counter()
            io = backend.read_state()
            c1 = time.perf_counter()

            ee = kin.fk(io.q)
            J = kin.jacobian(io.q)
            ee_vel = J @ io.dq
            J_xz = J[xz, :]
            Jdot_xz = (J_xz - J_xz_prev) / dt if J_xz_prev is not None else np.zeros_like(J_xz)
            J_xz_prev = J_xz.copy()
            ee_xz = ee[xz]
            ee_vel_xz = ee_vel[xz]

            y_err = ee_xz - p0[xz]
            x_state = np.r_[y_err, ee_vel_xz]

            Mq = dyn.mass_matrix(io.q) + armature * np.eye(n)
            Minv = np.linalg.inv(Mq)
            Lam = np.linalg.inv(J_xz @ Minv @ J_xz.T + lam_damp * np.eye(2))
            Nproj = np.eye(n) - J_xz.T @ (Lam @ J_xz @ Minv)
            post_err = io.q - q_nom
            tau_post = -post_kp * post_err - post_kd * io.dq
            cor = dyn.coriolis(io.q, io.dq)
            mu_xz = Lam @ (J_xz @ Minv @ cor - Jdot_xz @ io.dq)
            tau_base = Nproj @ tau_post + g_scale * dyn.gravity(io.q) + cor + J_xz.T @ mu_xz

            if obs is not None:
                u = ctrl.solve(x_state, d_hat)
                d_hat, _, _ = obs.step(y_err, u)
                F_task = Lam @ u
            else:
                F_task = ctrl.step(y_err, ee_vel_xz)

            tau = J_xz.T @ F_task + tau_base
            tau = np.clip(tau, -tau_max, tau_max)
            c2 = time.perf_counter()

            backend.send_torque(tau)
            c3 = time.perf_counter()

            t_read[k] = 1000.0 * (c1 - c0)
            t_compute[k] = 1000.0 * (c2 - c1)
            t_send[k] = 1000.0 * (c3 - c2)
    finally:
        backend.disable()

    total = t_read + t_compute + t_send
    budget_ms = 1000.0 * dt
    print(f"\nPer-phase timing over {args.n} ticks (dt budget = {budget_ms:.1f}ms):")
    _percentiles("read", t_read, budget_ms)
    _percentiles("compute", t_compute, budget_ms)
    _percentiles("send", t_send, budget_ms)
    _percentiles("TOTAL", total, budget_ms)

    read_share = 100.0 * t_read.mean() / total.mean() if total.mean() > 0 else 0.0
    send_share = 100.0 * t_send.mean() / total.mean() if total.mean() > 0 else 0.0
    comm_share = read_share + send_share
    compute_share = 100.0 - comm_share
    print(f"\nShare of mean total tick time: read={read_share:.0f}% send={send_share:.0f}% "
          f"(comm={comm_share:.0f}%)  compute={compute_share:.0f}%")
    if args.backend == "sim":
        print("(--backend sim: 'read' above is SimArmBackend's own physics-substep "
              "integration cost, not communication -- send is genuinely free, but read is "
              "neither zero nor a stand-in for real serial I/O. This run only validates the "
              "script's logic/output format; it does NOT answer the compute-vs-comm question. "
              "Re-run with --backend dynamixel on the real robot for that.)")
    elif comm_share > 50.0:
        print("-> majority of the tick is communication (read_state/send_torque), NOT "
              "compute: the original report's own 'PID's 6.73ms is most likely "
              "communication-bound' hypothesis would be CONFIRMED here, and the compute "
              "optimizations in implementation_fix.md are capped in how much real-hardware "
              "benefit they can deliver by whatever this comm share leaves on the table.")
    else:
        print("-> majority of the tick is compute, NOT communication: the report's "
              "'communication-bound' hypothesis would be REJECTED here, and the compute "
              "speedup already measured in implementation_fix.md should translate close to "
              "directly into a lower real per-tick time.")


if __name__ == "__main__":
    main()
