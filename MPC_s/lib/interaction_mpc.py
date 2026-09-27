"""Exact-ZOH interaction-dynamics MPC + Kalman disturbance observer.

The plant model both classes below share is the simplest one that captures
what actually matters for a torque-controlled robot arm doing task-space
tracking: a double integrator in task-space position error, driven by a
commanded residual acceleration `u` and an unknown, slowly-varying
disturbance `d` (real friction, unmodelled dynamics, payload, ...):

    e_ddot = u + d

This is the same normalized double-integrator controller used throughout the
pHRI paper (Cao & Tang, arXiv:2606.08281) and its already-validated port to
this same robot family (`pHRI/openmanipulator_verify/lib/interaction_mpc.py`):
the decision is a residual Cartesian acceleration sequence u_0..u_{N-1}, the
observer estimates a constant interaction disturbance d, and the offset-free
equilibrium is u=-d_hat. See docs/01_concepts.md for the full derivation and
docs/02_tuning_guide.md for how to choose q_pos/q_vel/r and
observer_q_d/observer_r_y from a target closed-loop bandwidth.

NOTE (2026-09-27): an earlier version of this file replaced the receding-
horizon box-constrained QP below with a static, precomputed LQR gain (no
re-solve, no per-step box constraint over the horizon) -- see
`original_implementation.md` Finding 1 for how that was found and why it
does not match either the report this project produced or the cited paper.
This restores the QP.

Optional JIT (`ControllerConfig.use_jit=True`): the FISTA loop in
`_solve_box_qp()` is Numba-JIT-compiled instead of plain numpy -- measured
at ~926us/call (200 iterations) in numpy vs ~170us/call compiled (~5.4x;
smaller than dynamics.py's _rnea() speedup because H@y here is a real
50-ish-dim matvec, not overhead on a tiny 3x3 matrix -- see
implementation_fix.md's "would C++ help" section). Requires
`pip install numba` (optional, see requirements.txt); raises a clear error
if requested and unavailable. Call `NormalizedInteractionMPC.warmup()` once
before a real-time loop starts -- Numba's first call pays a one-time
compile cost (~seconds, see dynamics.py's own warmup()).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from trajectory import CartesianTrajectory  # noqa: F401  (re-exported for convenience)

try:
    from numba import njit as _njit
    _NUMBA_AVAILABLE = True
except ImportError:
    _NUMBA_AVAILABLE = False


if _NUMBA_AVAILABLE:
    @_njit(cache=True, fastmath=True)
    def _fista_jit(H, h, lb, ub, u_warm, L, iters):
        """Numba-compiled restatement of `_solve_box_qp()`'s FISTA loop,
        identical step for step -- verified bit-for-bit (~1e-13 max abs
        diff, floating-point noise) against the numpy version before this
        was adopted."""
        u = np.clip(u_warm, lb, ub)
        y = u.copy()
        t = 1.0
        for _ in range(iters):
            grad = H @ y + h
            u_new = np.clip(y - grad / L, lb, ub)
            t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t * t))
            y = u_new + ((t - 1.0) / t_new) * (u_new - u)
            u, t = u_new, t_new
        return u


@dataclass
class ControllerConfig:
    dim: int = 2            # task-space dimension (this project's arm: x-z plane, dim=2)
    dt: float = 0.01
    horizon: int = 25
    q_pos: float = 60.0     # cost weight on position error
    q_vel: float = 12.0     # cost weight on velocity error
    r: float = 0.05         # cost weight on control effort (residual acceleration)
    u_max: np.ndarray | None = None
    observer_q_d: float = 0.02      # how fast the observer trusts new evidence about d
    observer_r_y: float = 0.0004    # how much the observer trusts the position measurement
    observer_d_hat_max: float = 15.0
    qp_iters: int = 200     # FISTA iterations per solve() call
    use_jit: bool = False   # Numba-JIT the FISTA loop; see module docstring


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
        if cfg.use_jit and not _NUMBA_AVAILABLE:
            raise RuntimeError(
                "ControllerConfig.use_jit=True requires numba, which is not installed. "
                "pip install numba (see requirements.txt's comment on it)."
            )
        self._warmed_up = False
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
        if self.cfg.use_jit:
            return _fista_jit(self.H, h, self._lb, self._ub, self._u_warm, self._L, self.cfg.qp_iters)
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

    def warmup(self) -> float:
        """Trigger Numba's one-time JIT compile now, not on the control
        loop's first real tick. No-op (returns 0.0) if use_jit is False.
        Returns the wall-clock seconds this took."""
        if not self.cfg.use_jit or self._warmed_up:
            return 0.0
        import time
        t0 = time.perf_counter()
        self._solve_box_qp(np.zeros(self.cfg.dim * self.cfg.horizon))
        self._u_warm = np.zeros(self.cfg.dim * self.cfg.horizon)  # undo the warmup call's own warm-start
        self._warmed_up = True
        return time.perf_counter() - t0

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

    def effective_gain(self) -> np.ndarray:
        """The box-QP's closed-form UNCONSTRAINED first-step gain K_eff, such
        that `u[:n] = -K_eff @ x` whenever the box constraint is not active
        (equivalently, as if u_max=inf) -- i.e. the QP analog of the old
        static-LQR controller's `K0` attribute, kept for the same purpose:
        `docs/02_tuning_guide.md` section 4 and `tools/solve_task_space_gain.py`
        both read this off to compute the actual closed-loop task-space
        stiffness/damping `Lam(q) @ K_eff[:, :n]` / `Lam(q) @ K_eff[:, n:]` at
        a given posture, since q_pos/q_vel/r are dimensionless cost weights,
        not physically meaningful on their own. Exact (not approximate) when
        the constraint doesn't bind, matching the original report's own
        Eq. 20 derivation of K from the unconstrained closed-form QP solution."""
        n = self.cfg.dim
        return np.linalg.solve(self.H, self.Gamma.T @ self.Q_bar @ self.Phi)[:n, :]


class RandomWalkDisturbanceObserver:
    """Kalman filter estimating [pos error, vel error, disturbance d].

    `d` is modelled as a random walk (its own best prediction of tomorrow is
    today's value, plus noise) -- appropriate for something like friction or
    an added payload that changes slowly compared to the control loop, but
    NOT literally constant. `q_d` is that process noise's weight: raise it
    and the observer trusts new evidence about `d` more and reacts faster
    (higher bandwidth), at the cost of more sensitivity to measurement noise.
    `q_vel` is the analogous weight for the velocity-error state -- normally
    left at its default; see docs/02_tuning_guide.md for when and why you
    might free it as a third tuning knob alongside q_d/r_y.

    Includes a simple anti-windup clamp on d_hat: without it, if the arm gets
    physically stuck (hits a joint limit, the table, or is over-saturated by
    controller gains that are too aggressive for the real plant), position
    error stops improving and this observer will integrate d_hat WITHOUT BOUND
    while commanded torque stays pinned at tau_max -- a runaway/windup failure
    mode. Clamping d_hat to d_hat_max keeps the estimate (and therefore the
    resulting commanded torque) bounded even when the loop can't converge.
    """

    def __init__(self, dim: int, dt: float, q_d: float, r_y: float, d_hat_max: float = 15.0,
                 q_vel: float = 1e-5):
        self.dim = dim
        self.A = np.block([[np.eye(dim), dt * np.eye(dim)], [np.zeros((dim, dim)), np.eye(dim)]])
        self.B = np.vstack((0.5 * dt * dt * np.eye(dim), dt * np.eye(dim)))
        self.Aa = np.block([[self.A, self.B], [np.zeros((dim, 2 * dim)), np.eye(dim)]])
        self.Ba = np.vstack((self.B, np.zeros((dim, dim))))
        self.C = np.hstack((np.eye(dim), np.zeros((dim, dim)), np.zeros((dim, dim))))
        self.Q = np.diag([1e-7] * dim + [q_vel] * dim + [q_d] * dim)
        self.R = r_y * np.eye(dim)
        self.z = np.zeros(3 * dim)
        self.P = np.eye(3 * dim)
        self.d_hat_max = float(d_hat_max)

    def reset(self) -> None:
        self.z[:] = 0.0
        self.P = np.eye(3 * self.dim)

    def step(self, y_pos_error: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """One predict+correct Kalman step. Returns (d_hat, innovation, nis).

        `nis` (Normalized Innovation Squared) should average out to roughly
        `dim` for a well-tuned, consistent filter -- a good single-number
        health check while running (see docs/02_tuning_guide.md): values in
        the hundreds mean the observer's estimate has stopped tracking
        reality, usually because it (or the controller) is being pushed
        faster than the real system/measurement can support.
        """
        y = np.asarray(y_pos_error, dtype=float).reshape(self.dim)
        u = np.asarray(u, dtype=float).reshape(self.dim)
        self.z = self.Aa @ self.z + self.Ba @ u
        self.P = self.Aa @ self.P @ self.Aa.T + self.Q
        innovation = y - self.C @ self.z
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        self.z = self.z + K @ innovation
        self.P = (np.eye(3 * self.dim) - K @ self.C) @ self.P
        self.z[2 * self.dim:] = np.clip(self.z[2 * self.dim:], -self.d_hat_max, self.d_hat_max)
        nis = float(innovation @ np.linalg.solve(S, innovation))
        return self.z[2 * self.dim:].copy(), innovation, nis
