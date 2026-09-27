"""Slow, smooth joint-space point-to-point motion that brings the arm from
wherever it currently is to a target joint configuration, run ONCE before the
real experiment's control loop begins.

Why this exists: the null-space posture term (tau_post, in run_hardware.py)
pulls the redundant DOF toward posture_q_rad starting the instant torque is
enabled -- if the arm's actual starting joint configuration differs from
posture_q_rad (which it always does to some degree, since it's placed by
hand), that pull dominates during the low-authority startup ramp and yanks
the arm through a large, uncontrolled motion before the real trajectory
tracking ever gets a say. This module removes the mismatch BEFORE the real
controller starts, using a separate, simple joint-space PID -- not the
MPC/observer under test -- so the experiment always starts from a clean,
motionless, correctly positioned state.

No fixed duration: this function runs its own control loop until the arm has
actually settled at q_target (position error below pos_eps AND velocity
below vel_eps, sustained for settle_time_s), however long that takes, and
only then returns control to the caller; timeout_s is a safety cap only (in
case the arm is physically blocked or mistuned), not a target duration.

Gravity compensation is included and load-bearing, not optional: this arm's
joint-space mass matrix is fairly ill-conditioned, so a PID gain low enough
to stay stable at a 100 Hz discrete loop does not have enough authority to
fight gravity through feedback alone -- without an explicit gravity term the
arm either stalls a couple of radians short of the target or, if you raise
the gain enough to compensate, oscillates.
"""
from __future__ import annotations

import time

import numpy as np


def move_to_start(backend, dyn, q_target: np.ndarray, dt: float, tau_max: np.ndarray,
                   kp: float, ki: float, kd: float, gravity_scale: float = 1.0, i_max: float = 2.0,
                   pos_eps: float = 0.1, vel_eps: float = 0.05, settle_time_s: float = 2.0,
                   timeout_s: float = 20.0,
                   jmin: np.ndarray | None = None, jmax: np.ndarray | None = None,
                   barrier_k: float = 0.0, barrier_margin: float = 0.15,
                   print_prefix: str = "[move-to-start]") -> None:
    """Blocking call: joint-space PID regulator,
        tau = -(kp*err + ki*integral(err) + kd*dq) + gravity_scale*dyn.gravity(q),
    err = q - q_target (actual minus desired). Runs until the arm settles at
    q_target (|err|<pos_eps and |dq|<vel_eps for settle_time_s straight) or
    until timeout_s elapses as a safety cap, then returns. Does not touch or
    know about the caller's trajectory/controller/observer state.
    """
    q_target = np.asarray(q_target, dtype=float)
    io = backend.read_state()
    n = len(io.q)
    integral = np.zeros(n)
    have_limits = jmin is not None and jmax is not None and barrier_k > 0
    settle_needed = max(1, int(round(settle_time_s / dt)))
    max_steps = max(1, int(round(timeout_s / dt)))
    settle_count = 0
    converged = False

    print(f"{print_prefix} moving from q={np.round(io.q, 4)} to q_target={np.round(q_target, 4)} "
          f"(joint PID + gravity comp, runs until settled; timeout {timeout_s:.0f}s)...")
    start = time.monotonic(); next_tick = start; last_print = start
    for _ in range(max_steps):
        io = backend.read_state()
        err = io.q - q_target
        integral += err * dt
        integral = np.clip(integral, -i_max, i_max)
        tau_fb = -(kp * err + ki * integral + kd * io.dq)
        tau = tau_fb + gravity_scale * dyn.gravity(io.q)
        if have_limits:
            over_hi = np.maximum(0.0, io.q - (jmax - barrier_margin))
            over_lo = np.maximum(0.0, (jmin + barrier_margin) - io.q)
            tau = tau - barrier_k * over_hi + barrier_k * over_lo
        tau = np.clip(tau, -tau_max, tau_max)
        backend.send_torque(tau)

        if np.all(np.abs(err) < pos_eps) and np.all(np.abs(io.dq) < vel_eps):
            settle_count += 1
        else:
            settle_count = 0

        now = time.monotonic()
        if now - last_print > 0.5:
            print(f"\r{print_prefix} t={now - start:5.1f}s  q={np.round(io.q, 3)}  "
                  f"|err|={np.linalg.norm(err):.4f}rad  |dq|={np.linalg.norm(io.dq):.4f}rad/s   ",
                  end="", flush=True)
            last_print = now

        if settle_count >= settle_needed:
            converged = True
            break

        next_tick += dt
        time.sleep(max(0.0, next_tick - time.monotonic()))

    io = backend.read_state()
    resid = np.linalg.norm(io.q - q_target)
    elapsed = time.monotonic() - start
    if converged:
        print(f"\n{print_prefix} converged after {elapsed:.2f}s; "
              f"residual |q-q_target|={resid:.4f}rad, |dq|={np.linalg.norm(io.dq):.4f}rad/s")
    else:
        print(f"\n{print_prefix} WARNING: did not settle within timeout_s={timeout_s:.0f}s "
              f"(elapsed {elapsed:.2f}s); residual |q-q_target|={resid:.4f}rad, "
              f"|dq|={np.linalg.norm(io.dq):.4f}rad/s -- proceeding anyway from whatever pose this is")
