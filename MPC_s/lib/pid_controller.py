"""Task-space PID controller -- an independent baseline to compare against
lib/interaction_mpc.py's LQR + Kalman-disturbance-observer controller.

This file has zero dependency on interaction_mpc.py: no shared classes, no
shared state. It only knows about Cartesian position and velocity error and
produces a task-space force. It has no notion of a horizon, a Riccati solve or
a disturbance estimate. run_pid.py, the runner that uses it, never imports
interaction_mpc.py either, so a PID run cannot accidentally exercise MPC code
or the other way round.

Classical operational-space PID: given task-space position error
e = ee - p_d and velocity error edot = ee_vel - dp_d,

    F = -(Kp*e + Ki*integral(e) + Kd*edot)

mapped to joint torques exactly the way the MPC path already does
(tau = J^T F + gravity + Coriolis + null-space posture), so the only thing
swapped out relative to the MPC path is the feedback law that turns tracking
error into a task-space force -- not the kinematics, not the gravity model,
not the hardware backend.

Note what this law does NOT have, because it is the whole point of the
comparison: there is no estimate of the disturbance acting on the arm. A
constant disturbance can only be worked off through the integral term, whose
gain is bounded by stability -- see docs/01_concepts.md, section 5.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PIDConfig:
    dim: int = 2
    dt: float = 0.01
    kp: float = 20.0
    ki: float = 0.0
    kd: float = 6.0
    i_max: float = 0.02               # anti-windup clamp on the integral term [m*s]
    f_max: np.ndarray | None = None   # output saturation on the task-space force [N]


class TaskSpacePID:
    """F = -(Kp*e + Ki*integral(e) + Kd*edot). Independent of interaction_mpc.py."""

    def __init__(self, cfg: PIDConfig):
        self.cfg = cfg
        dim = cfg.dim
        self.kp = np.full(dim, cfg.kp, dtype=float) if np.isscalar(cfg.kp) else np.asarray(cfg.kp, dtype=float)
        self.ki = np.full(dim, cfg.ki, dtype=float) if np.isscalar(cfg.ki) else np.asarray(cfg.ki, dtype=float)
        self.kd = np.full(dim, cfg.kd, dtype=float) if np.isscalar(cfg.kd) else np.asarray(cfg.kd, dtype=float)
        self.f_max = np.asarray(cfg.f_max, dtype=float) if cfg.f_max is not None else np.full(dim, np.inf)
        self._integral = np.zeros(dim)

    def reset(self) -> None:
        self._integral[:] = 0.0

    def step(self, pos_error: np.ndarray, vel_error: np.ndarray) -> np.ndarray:
        """pos_error = ee - p_d, vel_error = ee_vel - dp_d (actual minus desired,
        the same sign convention interaction_mpc.py's y_err/x_state uses).
        Returns the task-space force F to add into tau = J^T F + gravity + ...

        The integral is clamped to +/-i_max BEFORE it is multiplied by Ki. That
        clamp is a real design parameter, not a formality: it bounds how much
        steady-state authority the integral term can ever accumulate, so
        setting it too low silently caps the controller's ability to work off a
        constant disturbance no matter how long it runs."""
        pos_error = np.asarray(pos_error, dtype=float).reshape(self.cfg.dim)
        vel_error = np.asarray(vel_error, dtype=float).reshape(self.cfg.dim)
        self._integral += pos_error * self.cfg.dt
        self._integral = np.clip(self._integral, -self.cfg.i_max, self.cfg.i_max)
        F = -(self.kp * pos_error + self.ki * self._integral + self.kd * vel_error)
        return np.clip(F, -self.f_max, self.f_max)
