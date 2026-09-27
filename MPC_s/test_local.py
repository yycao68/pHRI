#!/usr/bin/env python3
"""No-hardware, no-dynamixel-sdk smoke test for the 3-DOF waist-removed arm.

Checks:
  1. kinematics.py: FK/Jacobian shapes and a finite-difference consistency check.
  2. dynamics.py: mass_matrix is symmetric positive-definite, gravity(q) is finite.
  3. Full sim loop (SimArmBackend) for a few seconds converges toward the hold
     target without diverging -- once for the MPC + observer, once for the
     task-space PID, since both are shipped and both should be smoke-tested.

Run:
    python3 test_local.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "lib"))

from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from dynamixel_backend import SimArmBackend  # noqa: E402
from interaction_mpc import (  # noqa: E402
    CartesianTrajectory, ControllerConfig, NormalizedInteractionMPC, RandomWalkDisturbanceObserver,
)
from pid_controller import PIDConfig, TaskSpacePID  # noqa: E402


def check_kinematics() -> None:
    kin = OpenManipulatorKinematics()
    q = np.array([0.1, -0.3, 0.2])
    ee = kin.fk(q)
    assert ee.shape == (3,), "fk() must return a 3-vector"
    J = kin.jacobian(q)
    assert J.shape == (3, 3), "jacobian() must be 3x3 for the 3-DOF arm"

    # finite-difference check: J @ dq should predict fk(q+dq) - fk(q)
    dq = np.array([0.01, -0.01, 0.005])
    predicted = ee + J @ dq
    actual = kin.fk(q + dq)
    err = np.linalg.norm(predicted - actual)
    assert err < 1e-3, f"Jacobian inconsistent with FK, err={err}"
    print(f"[OK] kinematics: fk/jacobian consistent (fd err={err:.2e} m)")


def check_dynamics() -> None:
    dyn = OpenManipulatorDynamics()
    q = np.array([0.1, -0.3, 0.2])
    M = dyn.mass_matrix(q)
    assert M.shape == (3, 3)
    eigvals = np.linalg.eigvalsh(M)
    assert np.all(eigvals > 0), f"mass matrix not positive-definite, eigvals={eigvals}"
    g = dyn.gravity(q)
    assert np.all(np.isfinite(g)) and g.shape == (3,)
    print(f"[OK] dynamics: M(q) PD (eigvals min={eigvals.min():.4f}), gravity finite: {np.round(g, 3)}")

    # Regression check: the fast, Jacobian-formula mass_matrix() and the
    # single-pass bias_force() must stay numerically identical to the
    # original (slower) n-RNEA-call / separate-gravity+coriolis approach --
    # not just fast, but provably the same answer.
    rng = np.random.default_rng(0)
    max_M_err = 0.0
    max_bias_err = 0.0
    for _ in range(50):
        q_r = rng.uniform(-2.0, 2.0, 3)
        dq_r = rng.uniform(-3.0, 3.0, 3)
        gscale = rng.choice([0.0, 0.5, 1.0, 1.3])
        max_M_err = max(max_M_err, np.max(np.abs(dyn.mass_matrix(q_r) - dyn._mass_matrix_rnea(q_r))))
        bias_fast = dyn.bias_force(q_r, dq_r, gravity_scale=gscale)
        bias_slow = gscale * dyn.gravity(q_r) + dyn.coriolis(q_r, dq_r)
        max_bias_err = max(max_bias_err, np.max(np.abs(bias_fast - bias_slow)))
    assert max_M_err < 1e-9, f"fast mass_matrix() diverged from _mass_matrix_rnea by {max_M_err}"
    assert max_bias_err < 1e-9, f"bias_force() diverged from gravity()+coriolis() by {max_bias_err}"
    print(f"[OK] dynamics: fast mass_matrix()/bias_force() match the slow reference "
          f"(max err {max_M_err:.2e} N.m, {max_bias_err:.2e} N.m over 50 random samples)")

    # Regression check for the optional Numba JIT path (use_jit=True): must
    # match the plain-numpy _rnea() exactly (within floating-point noise),
    # not just be fast -- skipped, not failed, if numba isn't installed,
    # since it's an optional dependency (see requirements.txt).
    try:
        import numba  # noqa: F401
        numba_available = True
    except ImportError:
        numba_available = False
    if numba_available:
        dyn_jit = OpenManipulatorDynamics(use_jit=True)
        warm_s = dyn_jit.warmup()
        max_jit_err = 0.0
        for _ in range(50):
            q_r = rng.uniform(-2.0, 2.0, 3)
            dq_r = rng.uniform(-3.0, 3.0, 3)
            gscale = rng.choice([0.0, 0.5, 1.0, 1.3])
            max_jit_err = max(max_jit_err, np.max(np.abs(
                dyn_jit.bias_force(q_r, dq_r, gravity_scale=gscale)
                - dyn.bias_force(q_r, dq_r, gravity_scale=gscale))))
        assert max_jit_err < 1e-9, f"use_jit=True diverged from plain numpy by {max_jit_err}"
        print(f"[OK] dynamics: use_jit=True matches plain numpy "
              f"(max err {max_jit_err:.2e} N.m, warm-up {warm_s:.2f}s)")
    else:
        print("[SKIP] dynamics: use_jit regression check (numba not installed -- optional)")


def check_mpc_jit() -> None:
    """Same idea as check_dynamics()'s use_jit check, for
    NormalizedInteractionMPC's FISTA solve instead of RNEA. Skipped, not
    failed, if numba isn't installed."""
    try:
        import numba  # noqa: F401
    except ImportError:
        print("[SKIP] interaction_mpc: use_jit regression check (numba not installed -- optional)")
        return
    cfg_common = dict(dim=2, dt=0.01, horizon=25, q_pos=11293.6, q_vel=0.0, r=0.15,
                       u_max=np.array([100.0, 100.0]))
    mpc_np = NormalizedInteractionMPC(ControllerConfig(use_jit=False, **cfg_common))
    mpc_jit = NormalizedInteractionMPC(ControllerConfig(use_jit=True, **cfg_common))
    warm_s = mpc_jit.warmup()
    rng = np.random.default_rng(2)
    max_err = 0.0
    for _ in range(50):
        x = rng.uniform(-0.05, 0.05, 4)
        d_hat = rng.uniform(-5, 5, 2)
        max_err = max(max_err, np.max(np.abs(mpc_np.solve(x, d_hat) - mpc_jit.solve(x, d_hat))))
    assert max_err < 1e-9, f"MPC use_jit=True diverged from plain numpy by {max_err}"
    print(f"[OK] interaction_mpc: use_jit=True matches plain numpy "
          f"(max err {max_err:.2e}, warm-up {warm_s:.2f}s)")


def check_sim_hold(config_path: Path, controller: str = "mpc", duration_s: float = 8.0) -> None:
    """`controller` selects which feedback law drives the loop: "mpc" for
    run_hardware.py's, "pid" for run_pid.py's. Everything else is identical,
    which is the same property the two runners have."""
    with open(config_path, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    kin = OpenManipulatorKinematics()
    dyn = OpenManipulatorDynamics()
    backend = SimArmBackend(config, kin)

    c = config.get("controller", {})
    cfg = ControllerConfig(
        dt=float(c.get("dt", 0.01)), horizon=int(c.get("horizon", 25)),
        q_pos=float(c.get("q_pos", 60.0)), q_vel=float(c.get("q_vel", 12.0)),
        r=float(c.get("r", 0.05)), u_max=np.asarray(c.get("u_max", [8.0] * 3), dtype=float)[:2],
        observer_q_d=float(c.get("observer_q_d", 3.0)), observer_r_y=float(c.get("observer_r_y", 0.0004)),
    )
    cfg.dim = 2  # x-z task space only (see run_hardware.py for why)
    mpc = obs = pid = None
    if controller == "mpc":
        mpc = NormalizedInteractionMPC(cfg)
        obs = RandomWalkDisturbanceObserver(2, cfg.dt, cfg.observer_q_d, cfg.observer_r_y,
                                             float(c.get("observer_d_hat_max", 15.0)))
    else:
        pcf = config.get("pid", {})
        pid = TaskSpacePID(PIDConfig(
            dim=2, dt=float(pcf.get("dt", cfg.dt)),
            kp=float(pcf.get("kp", 20.0)), ki=float(pcf.get("ki", 0.0)),
            kd=float(pcf.get("kd", 6.0)), i_max=float(pcf.get("i_max", 0.02)),
            f_max=np.asarray(pcf.get("f_max", [8.0, 8.0]), dtype=float)))

    robot = config.get("robot", {})
    g_scale = float(robot.get("gravity_scale", 1.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [3.5, 3.0, 2.0]), dtype=float)
    lam_damp = float(robot.get("lambda_damping", 1e-2))
    startup_ramp = float(robot.get("startup_ramp_s", 2.0))
    # Must match SimArmBackend's own copy of this key (run_hardware.py's own
    # comment on this key says so): the controller's model of Mq and the
    # simulated plant's actual Mq have to agree, or this loop is testing a
    # controller against a plant it was not designed for.
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))

    backend.enable()
    io = backend.read_state()
    p0 = kin.fk(io.q)
    traj = CartesianTrajectory(config.get("trajectory", {}), p0, 0.0)
    d_hat = np.zeros(2)

    n_steps = int(duration_s / cfg.dt)
    errs = []
    xz = [0, 2]
    for step in range(n_steps):
        t = step * cfg.dt
        io = backend.read_state()
        ee = kin.fk(io.q)
        J = kin.jacobian(io.q)
        ee_vel = J @ io.dq
        J_xz = J[xz, :]
        p_d, dp_d, ddp_d = traj.sample(t)
        y_err = ee[xz] - p_d[xz]
        vel_err = ee_vel[xz] - dp_d[xz]

        ramp = min(1.0, t / max(startup_ramp, 1e-6))
        Mq = dyn.mass_matrix(io.q) + armature * np.eye(kin.n)
        Minv = np.linalg.inv(Mq)
        Lam = np.linalg.inv(J_xz @ Minv @ J_xz.T + lam_damp * np.eye(2))
        if mpc is not None:
            u = mpc.solve(np.r_[y_err, vel_err], d_hat)
            d_hat, _, _ = obs.step(y_err, u)
            F_task = ramp * (Lam @ (ddp_d[xz] + u))
        else:
            F_task = ramp * pid.step(y_err, vel_err)
        tau = J_xz.T @ F_task + g_scale * dyn.gravity(io.q)
        tau = np.clip(tau, -tau_max, tau_max)
        backend.send_torque(tau)
        errs.append(1000.0 * np.linalg.norm(y_err))

    backend.disable()
    errs = np.array(errs)
    ss_err = np.mean(errs[-100:])
    print(f"[OK] sim hold loop ({controller}) ran {n_steps} steps, final steady-state err "
          f"~= {ss_err:.2f} mm (max err {errs.max():.2f} mm)")
    assert np.all(np.isfinite(errs)), "simulation diverged (non-finite error)"
    assert ss_err < 15.0, (f"steady-state error too large ({ss_err:.2f} mm) with controller="
                            f"{controller} -- check config/tuning")


if __name__ == "__main__":
    print("Running no-hardware smoke tests for the 3-DOF waist-removed OpenManipulator-X...\n")
    check_kinematics()
    check_dynamics()
    check_mpc_jit()
    check_sim_hold(ROOT / "configs" / "hold.yaml", controller="mpc")
    check_sim_hold(ROOT / "configs" / "hold.yaml", controller="pid")
    print("\nAll smoke tests passed. This does NOT replace testing with --backend sim "
          "via run_hardware.py (which also exercises the CSV logging and period_ms "
          "timing you'll want to check on the PC before touching hardware).")
