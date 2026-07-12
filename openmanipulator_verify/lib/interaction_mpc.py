"""Exact-ZOH interaction-dynamics MPC + Kalman disturbance observer.

This is the same normalized double-integrator controller used throughout the
pHRI paper: the decision is a residual Cartesian acceleration u, the observer
estimates a constant interaction disturbance d, and the offset-free equilibrium
is u = -d_hat. On a TORQUE-controlled arm (unlike the position-level JetCobot),
the plant really is the double integrator, so the observer must be fed the
applied u and offset-free regulation holds.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ControllerConfig:
    dim: int = 3
    dt: float = 0.01
    horizon: int = 25
    q_pos: float = 60.0
    q_vel: float = 12.0
    r: float = 0.05
    u_max: np.ndarray | None = None
    observer_q_d: float = 0.02
    observer_r_y: float = 0.0004


class NormalizedInteractionMPC:
    """Finite-horizon LQR for x+ = A x + B (u + d). Feedback: u = -K0 x - d_hat."""

    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        n, dt = cfg.dim, cfg.dt
        self.A = np.block([[np.eye(n), dt * np.eye(n)], [np.zeros((n, n)), np.eye(n)]])
        self.B = np.vstack((0.5 * dt * dt * np.eye(n), dt * np.eye(n)))
        self.Q = np.diag([cfg.q_pos] * n + [cfg.q_vel] * n)
        self.R = cfg.r * np.eye(n)
        self.K0 = self._finite_horizon_first_gain()
        self.u_max = np.asarray(cfg.u_max if cfg.u_max is not None else np.inf * np.ones(n), dtype=float)

    def _finite_horizon_first_gain(self) -> np.ndarray:
        P = self.Q.copy()
        K = np.zeros((self.cfg.dim, 2 * self.cfg.dim))
        for _ in range(self.cfg.horizon):
            H = self.R + self.B.T @ P @ self.B
            K = np.linalg.solve(H, self.B.T @ P @ self.A)
            P = self.Q + self.A.T @ P @ (self.A - self.B @ K)
        return K

    def solve(self, x: np.ndarray, d_hat: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float).reshape(2 * self.cfg.dim)
        d_hat = np.asarray(d_hat, dtype=float).reshape(self.cfg.dim)
        u = -self.K0 @ x - d_hat
        return np.clip(u, -self.u_max, self.u_max)


class RandomWalkDisturbanceObserver:
    """Kalman observer for [pos error, vel error, constant disturbance d]."""

    def __init__(self, dim: int, dt: float, q_d: float, r_y: float):
        self.dim = dim
        self.A = np.block([[np.eye(dim), dt * np.eye(dim)], [np.zeros((dim, dim)), np.eye(dim)]])
        self.B = np.vstack((0.5 * dt * dt * np.eye(dim), dt * np.eye(dim)))
        self.Aa = np.block([[self.A, self.B], [np.zeros((dim, 2 * dim)), np.eye(dim)]])
        self.Ba = np.vstack((self.B, np.zeros((dim, dim))))
        self.C = np.hstack((np.eye(dim), np.zeros((dim, dim)), np.zeros((dim, dim))))
        self.Q = np.diag([1e-7] * dim + [1e-5] * dim + [q_d] * dim)
        self.R = r_y * np.eye(dim)
        self.z = np.zeros(3 * dim)
        self.P = np.eye(3 * dim)

    def reset(self) -> None:
        self.z[:] = 0.0
        self.P = np.eye(3 * self.dim)

    def step(self, y_pos_error: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        y = np.asarray(y_pos_error, dtype=float).reshape(self.dim)
        u = np.asarray(u, dtype=float).reshape(self.dim)
        self.z = self.Aa @ self.z + self.Ba @ u
        self.P = self.Aa @ self.P @ self.Aa.T + self.Q
        innovation = y - self.C @ self.z
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        self.z = self.z + K @ innovation
        self.P = (np.eye(3 * self.dim) - K @ self.C) @ self.P
        nis = float(innovation @ np.linalg.solve(S, innovation))
        return self.z[2 * self.dim:].copy(), innovation, nis


class CartesianTrajectory:
    def __init__(self, cfg: dict, origin: np.ndarray, start_time: float):
        self.cfg = cfg
        self.origin = np.asarray(origin, dtype=float).reshape(3)
        self.start_time = float(start_time)

    def sample(self, now: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = max(0.0, float(now) - self.start_time)
        typ = self.cfg.get("type", "hold")
        ramp = float(self.cfg.get("ramp_s", 2.0))
        a = min(1.0, t / max(ramp, 1e-6))
        if typ == "hold":
            return self.origin.copy(), np.zeros(3), np.zeros(3)
        if typ != "circle":
            raise ValueError(f"unknown trajectory type: {typ}")
        radius = float(self.cfg.get("circle_radius_m", 0.04))
        period = float(self.cfg.get("circle_period_s", 12.0))
        plane = str(self.cfg.get("circle_plane", "xz")).lower()
        w = 2.0 * np.pi / max(period, 1e-6)
        s, c = np.sin(w * t), np.cos(w * t)
        pos = np.zeros(3); vel = np.zeros(3); acc = np.zeros(3)
        axes = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}.get(plane)
        if axes is None:
            raise ValueError(f"unsupported circle_plane: {plane}")
        i, j = axes
        pos[i] = radius * (c - 1.0); pos[j] = radius * s
        vel[i] = -radius * w * s; vel[j] = radius * w * c
        acc[i] = -radius * w * w * c; acc[j] = -radius * w * w * s
        return self.origin + a * pos, a * vel, a * acc
