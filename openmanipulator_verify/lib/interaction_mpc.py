"""Exact-ZOH interaction-dynamics MPC + Kalman disturbance observer.

This is the same normalized double-integrator controller used throughout the
pHRI paper: the decision is a residual Cartesian acceleration sequence u_0..
u_{N-1}, the observer estimates a constant interaction disturbance d, and the
offset-free equilibrium is u = -d_hat. On a TORQUE-controlled arm, the plant
really is the double integrator, so the observer must be fed the applied u
and offset-free regulation holds.
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
    qp_iters: int = 200   # FISTA iterations per solve() call


class NormalizedInteractionMPC:
    """Box-constrained finite-horizon QP for x+ = A x + B (u + d).

    Matches the paper's Eq. (9): the decision is the full residual-
    acceleration sequence u_0..u_{N-1}, and the box constraint
    -u_max <= u_k <= u_max is enforced at EVERY horizon step k=0..N-1 (Eq.
    9c), not just clipped on the first-step control after an unconstrained
    solve. Only the first step is applied, then the QP is re-solved next
    tick (receding horizon).

    A, B are configuration-independent here (normalized residual-acceleration
    units), so the horizon maps Phi/Gamma/D_bar and the QP Hessian are built
    once in __init__; only the linear cost term changes per call. The QP is
    solved by warm-started FISTA (accelerated projected gradient): box
    constraints are separable, so a matrix-free iterative solve avoids
    adding a QP-solver dependency (osqp/scipy) to this numpy-only
    verification harness. The terminal cost is the converged discrete
    Riccati solution for (A, B, Q, R), so the finite horizon approximates
    the infinite-horizon LQR tail while the box constraint stays exact
    over the whole planning horizon.
    """

    def __init__(self, cfg: ControllerConfig):
        self.cfg = cfg
        n, dt, N = cfg.dim, cfg.dt, cfg.horizon
        self.A = np.block([[np.eye(n), dt * np.eye(n)], [np.zeros((n, n)), np.eye(n)]])
        self.B = np.vstack((0.5 * dt * dt * np.eye(n), dt * np.eye(n)))
        self.Q = np.diag([cfg.q_pos] * n + [cfg.q_vel] * n)
        self.R = cfg.r * np.eye(n)
        self.u_max = np.asarray(cfg.u_max if cfg.u_max is not None else np.inf * np.ones(n), dtype=float)

        # A_d^k for k = 0..N-1, and the free-response map Phi (A_d^1..A_d^N).
        self._Ad_pow = [np.eye(2 * n)]
        for _ in range(N - 1):
            self._Ad_pow.append(self._Ad_pow[-1] @ self.A)
        self.Phi = np.vstack([self._Ad_pow[k] @ self.A for k in range(N)])          # (2nN, 2n)

        # Input-to-state map Gamma[i,j] = A_d^{i-j} B_d for i >= j, else 0.
        Gam = np.zeros((2 * n * N, n * N))
        for i in range(N):
            for j in range(i + 1):
                Gam[2*n*i:2*n*(i+1), n*j:n*(j+1)] = self._Ad_pow[i - j] @ self.B
        self.Gamma = Gam

        # Disturbance propagation D_bar[k] = (I + A_d + ... + A_d^k) B_d.
        D_bar = np.zeros((2 * n * N, n))
        cumsum = np.zeros((2 * n, n))
        for k in range(N):
            cumsum = cumsum + self._Ad_pow[k] @ self.B
            D_bar[2*n*k:2*n*(k+1)] = cumsum
        self.D_bar = D_bar

        Q_f = self._riccati_terminal_cost()
        self.Q_bar = np.zeros((2 * n * N, 2 * n * N))
        for i in range(N - 1):
            self.Q_bar[2*n*i:2*n*(i+1), 2*n*i:2*n*(i+1)] = self.Q
        self.Q_bar[2*n*(N-1):, 2*n*(N-1):] = Q_f
        self.R_bar = np.kron(np.eye(N), self.R)

        self.H = self.Gamma.T @ self.Q_bar @ self.Gamma + self.R_bar
        self.H = 0.5 * (self.H + self.H.T)
        self._L = float(np.linalg.eigvalsh(self.H)[-1])  # FISTA step size 1/L

        self._lb = np.tile(-self.u_max, N)
        self._ub = np.tile(self.u_max, N)
        self._u_warm = np.zeros(n * N)

    def _riccati_terminal_cost(self) -> np.ndarray:
        """Converged discrete-time Riccati solution for (A, B, Q, R), used as
        the terminal cost so the finite-horizon QP approximates the
        infinite-horizon LQR tail (the original design intent of the
        Riccati-recursion controller this replaces)."""
        P = self.Q.copy()
        for _ in range(500):
            K = np.linalg.solve(self.R + self.B.T @ P @ self.B, self.B.T @ P @ self.A)
            P = self.Q + self.A.T @ P @ (self.A - self.B @ K)
        return P

    def _solve_box_qp(self, h: np.ndarray) -> np.ndarray:
        """min 0.5 u^T H u + h^T u  s.t.  lb <= u <= ub, via warm-started FISTA."""
        u = np.clip(self._u_warm, self._lb, self._ub)
        y = u.copy()
        t = 1.0
        for _ in range(self.cfg.qp_iters):
            grad = self.H @ y + h
            u_new = np.clip(y - grad / self._L, self._lb, self._ub)
            t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t * t))
            y = u_new + ((t - 1.0) / t_new) * (u_new - u)
            u, t = u_new, t_new
        return u

    def solve(self, x: np.ndarray, d_hat: np.ndarray) -> np.ndarray:
        n, N = self.cfg.dim, self.cfg.horizon
        x = np.asarray(x, dtype=float).reshape(2 * n)
        d_hat = np.asarray(d_hat, dtype=float).reshape(n)

        x_free = self.Phi @ x + self.D_bar @ d_hat
        # Offset-free input centering (as in the FR3 controller): penalising
        # V = U + d_seq, not U, makes the steady balancing input u = -d_hat
        # cost-free rather than R-penalised.
        d_seq = np.tile(d_hat, N)
        h = self.Gamma.T @ self.Q_bar @ x_free + self.R_bar @ d_seq

        u_seq = self._solve_box_qp(h)
        # Warm-start next call: shift the horizon by one step.
        self._u_warm = np.concatenate([u_seq[n:], u_seq[-n:]])
        return u_seq[:n]


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
