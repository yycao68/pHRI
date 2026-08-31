#!/usr/bin/env python3
"""Torque-level pHRI interaction-dynamics verification on the OpenManipulator-X.

Control law (operational-space-lite, Current Control Mode):
    x_ee = FK(q),  J = Jacobian(q),  ee_vel = J dq
    u      = MPC(x_ee - p_d, ee_vel - dp_d, d_hat)       # residual accel
    d_hat  = Observer(x_ee - p_d, u)                      # applied u fed in
    F_task = m_eff (ddp_d + u)                            # Cartesian force
    tau    = J^T F_task + gravity_scale * g(q) + barrier  # joint torque
    Goal_Current = tau / K_t                              # per XM430 servo

Because the arm is torque-driven, the offset-free property (SS error -> 0 under a
constant disturbance) actually holds -- the reason this needs a torque interface.

SAFETY: start with the arm supported and a low current_limit_ticks; torque is
disabled on any exit. Use --backend sim to validate everything off-hardware.

Usage:
  python3 run_openmanipulator_hardware.py --backend sim --config configs/push.yaml --duration 14 --output results/hardware/sim_push.csv
  python3 run_openmanipulator_hardware.py --backend dynamixel --port /dev/ttyUSB0 --config configs/hold.yaml --duration 20 --output results/hardware/J1_hold.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "lib"))

from interaction_mpc import (  # noqa: E402
    CartesianTrajectory, ControllerConfig, NormalizedInteractionMPC, RandomWalkDisturbanceObserver,
)
from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from dynamixel_backend import create_backend  # noqa: E402


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def controller_config(config: dict) -> ControllerConfig:
    c = config.get("controller", {})
    return ControllerConfig(
        dt=float(c.get("dt", 0.01)), horizon=int(c.get("horizon", 25)),
        q_pos=float(c.get("q_pos", 60.0)), q_vel=float(c.get("q_vel", 12.0)),
        r=float(c.get("r", 0.05)), u_max=np.asarray(c.get("u_max", [4.0, 4.0, 4.0]), dtype=float),
        observer_q_d=float(c.get("observer_q_d", 0.02)), observer_r_y=float(c.get("observer_r_y", 0.0004)),
    )


def build_kinematics(config: dict) -> OpenManipulatorKinematics:
    k = config.get("kinematics", {})
    kin = OpenManipulatorKinematics()
    if "links" in k:
        kin.links.update(k["links"])
    if "link_masses" in k:
        kin.link_masses = list(k["link_masses"])
    return kin


def run(args: argparse.Namespace) -> Path | None:
    config = load_yaml(args.config)
    kin = build_kinematics(config)
    dyn = OpenManipulatorDynamics()
    backend = create_backend(args.backend, config, kin, args)

    cfg = controller_config(config)
    mpc = NormalizedInteractionMPC(cfg)
    obs = RandomWalkDisturbanceObserver(3, cfg.dt, cfg.observer_q_d, cfg.observer_r_y)

    robot = config.get("robot", {})
    m_eff = float(robot.get("task_mass_kg", 0.6))
    g_scale = float(robot.get("gravity_scale", 1.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [1.5, 2.5, 1.5, 1.0]), dtype=float)
    jmin = np.asarray(robot.get("joint_min_rad", [-2.6, -1.8, -1.6, -1.8]), dtype=float)
    jmax = np.asarray(robot.get("joint_max_rad", [2.6, 1.6, 1.4, 1.8]), dtype=float)
    barrier_k = float(robot.get("joint_barrier_gain", 4.0))
    barrier_margin = float(robot.get("joint_barrier_margin_rad", 0.15))
    startup_ramp = float(robot.get("startup_ramp_s", 2.0))
    q_nom = np.asarray(robot.get("posture_q_rad", robot.get("sim_home_q_rad", [0.0, -0.6, 0.3, 0.3])), dtype=float)
    post_kp = float(robot.get("posture_kp", 0.6))
    post_kd = float(robot.get("posture_kd", 0.12))
    lam_damp = float(robot.get("lambda_damping", 2e-3))

    backend.enable()
    io = backend.read_state()
    p0 = kin.fk(io.q)
    traj = CartesianTrajectory(config.get("trajectory", {}), p0, 0.0)
    d_hat = np.zeros(3)

    fh = None; writer = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fh = open(args.output, "w", newline="")

    print(f"[omx] backend={args.backend} ee0={np.round(p0, 4)} dt={cfg.dt}")
    start = time.monotonic(); next_tick = start; sample = 0; t = 0.0
    try:
        while True:
            wall = time.monotonic()
            if args.duration is not None and wall - start >= args.duration:
                break
            t = wall - start if args.backend == "dynamixel" else sample * cfg.dt
            c0 = time.monotonic()

            io = backend.read_state()
            ee = kin.fk(io.q)
            J = kin.jacobian(io.q)
            ee_vel = J @ io.dq

            p_d, dp_d, ddp_d = traj.sample(t)
            y = ee - p_d
            x = np.r_[y, ee_vel - dp_d]
            u = mpc.solve(x, d_hat)
            d_hat, innovation, nis = obs.step(y, u)

            # operational-space realization with null-space posture control:
            #   F = Lambda(q)(xdd_d + u) + gravity;  N = I - J^T (Lambda J M^-1)
            ramp = min(1.0, t / max(startup_ramp, 1e-6))
            Mq = dyn.mass_matrix(io.q)
            Minv = np.linalg.inv(Mq)
            Lam = np.linalg.inv(J @ Minv @ J.T + lam_damp * np.eye(3))
            F_task = ramp * (Lam @ (ddp_d + u))
            tau_task = J.T @ F_task
            N = np.eye(4) - J.T @ (Lam @ J @ Minv)             # dyn-consistent null-space
            tau_post = -post_kp * (io.q - q_nom) - post_kd * io.dq
            tau = tau_task + N @ tau_post + g_scale * dyn.gravity(io.q)

            # joint-limit barrier (CBF-lite): push away from soft limits
            over_hi = np.maximum(0.0, io.q - (jmax - barrier_margin))
            over_lo = np.maximum(0.0, (jmin + barrier_margin) - io.q)
            tau = tau - barrier_k * over_hi + barrier_k * over_lo
            tau = np.clip(tau, -tau_max, tau_max)
            backend.send_torque(tau)

            compute_ms = 1000.0 * (time.monotonic() - c0)
            err_mm = 1000.0 * float(np.linalg.norm(y))
            if fh is not None:
                row = {"sample": sample, "t": t, "mode": config.get("trajectory", {}).get("type", "?"),
                       "err_mm": err_mm, "nis": nis, "compute_ms": compute_ms}
                for name, vec in (("ee", ee), ("p_d", p_d), ("ee_vel", ee_vel), ("u", u),
                                  ("d_hat", d_hat), ("F", F_task), ("q", io.q), ("tau", tau),
                                  ("cur", io.current_A)):
                    for i, v in enumerate(np.asarray(vec).reshape(-1)):
                        row[f"{name}_{i}"] = float(v)
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row.keys())); writer.writeheader()
                writer.writerow(row)

            sample += 1; next_tick += cfg.dt
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        backend.disable()
        if fh is not None:
            fh.close()
        print(f"[omx] torque disabled; {sample} samples")
    return args.output


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--backend", choices=["sim", "mjc", "dynamixel"], default="sim")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
