"""Rigid-body dynamics (gravity G(q) and joint-space mass matrix M(q)) for the
OpenManipulator-X, via recursive Newton-Euler (RNEA). Inertial parameters are
from the ROBOTIS URDF (validated against MuJoCo). This lets the controller do
proper operational-space control -- F_task = Lambda(q)(xdd_d+u), tau = J^T F + G
-- instead of a scalar task mass + crude gravity, which do not survive real
dynamics.

G(q) is RNEA at zero velocity/acceleration with gravity on. M(q) uses the
standard closed-form Jacobian formula (Siciliano et al., "Robotics", eq.
7.31-ish; one forward-kinematics pass) instead of n separate RNEA calls (the
original approach, kept below as `_mass_matrix_rnea` for a regression check
in `test_local.py`) -- this was the single largest per-tick cost (measured
~1.98ms/call for this n=4 arm vs ~0.50ms for a single RNEA call), the same
fix applied to the sibling 3-DOF project this arm's own design was forked
into (`pHRI/MPC_s`; see its `implementation_fix.md`).
"""
from __future__ import annotations

import numpy as np

G_VEC = np.array([0.0, 0.0, -9.81])
AXES = [np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0])]
# joint-frame offsets (parent frame -> joint i origin), matching kinematics
D = [np.array([0.012, 0.0, 0.0]), np.array([0.0, 0.0, 0.0595]),
     np.array([0.024, 0.0, 0.128]), np.array([0.124, 0.0, 0.0])]
# link inertial params in each link frame (mass [kg], COM [m], inertia_com [kg m^2])
LINKS = [
    dict(mass=0.09841, com=[-0.0003, 0.00054, 0.04743],
         I=[[3.45e-05, 0.0, -4e-07], [0.0, 3.27e-05, 0.0], [-4e-07, 0.0, 1.89e-05]]),
    dict(mass=0.13851, com=[0.01031, 0.00038, 0.1017],
         I=[[3.306e-04, -1e-07, -3.85e-05], [-1e-07, 3.429e-04, -1.6e-06], [-3.85e-05, -1.6e-06, 6.03e-05]]),
    dict(mass=0.13275, com=[0.09091, 0.00039, 0.00022],
         I=[[3.07e-05, -1.3e-06, -3e-07], [-1.3e-06, 2.423e-04, 0.0], [-3e-07, 0.0, 2.516e-04]]),
    dict(mass=0.14328, com=[0.04421, 0.0, 0.00891],
         I=[[8.09e-05, 0.0, -1e-06], [0.0, 7.6e-05, 0.0], [-1e-06, 0.0, 9.31e-05]]),
]


def _rot(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis
    c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


class OpenManipulatorDynamics:
    def __init__(self):
        self.n = 4
        self.m = [L["mass"] for L in LINKS]
        self.c = [np.asarray(L["com"], dtype=float) for L in LINKS]
        self.I = [np.asarray(L["I"], dtype=float) for L in LINKS]

    def _rnea(self, q, dq, ddq, gravity: bool) -> np.ndarray:
        n = self.n
        R = [_rot(AXES[i], q[i]) for i in range(n)]      # {}^{i-1}R_i
        w = [np.zeros(3) for _ in range(n + 1)]
        wd = [np.zeros(3) for _ in range(n + 1)]
        a = [np.zeros(3) for _ in range(n + 1)]
        a[0] = -G_VEC if gravity else np.zeros(3)         # base linear accel (gravity trick)
        F = [np.zeros(3) for _ in range(n)]
        N = [np.zeros(3) for _ in range(n)]
        # forward
        for i in range(n):
            iR = R[i].T                                   # {}^iR_{i-1}
            ax = AXES[i]
            w[i + 1] = iR @ w[i] + dq[i] * ax
            wd[i + 1] = iR @ wd[i] + np.cross(iR @ w[i], dq[i] * ax) + ddq[i] * ax
            a[i + 1] = iR @ (a[i] + np.cross(wd[i], D[i]) + np.cross(w[i], np.cross(w[i], D[i])))
            ci = self.c[i]
            a_ci = a[i + 1] + np.cross(wd[i + 1], ci) + np.cross(w[i + 1], np.cross(w[i + 1], ci))
            F[i] = self.m[i] * a_ci
            N[i] = self.I[i] @ wd[i + 1] + np.cross(w[i + 1], self.I[i] @ w[i + 1])
        # backward
        f = np.zeros(3); nn = np.zeros(3); tau = np.zeros(n)
        for i in range(n - 1, -1, -1):
            if i + 1 < n:
                Rc = R[i + 1]                             # {}^iR_{i+1}
                p = D[i + 1]
                f_child = f.copy(); n_child = nn.copy()
                f = Rc @ f_child + F[i]
                nn = N[i] + Rc @ n_child + np.cross(self.c[i], F[i]) + np.cross(p, Rc @ f_child)
            else:
                f = F[i]
                nn = N[i] + np.cross(self.c[i], F[i])
            tau[i] = nn @ AXES[i]
        return tau

    def gravity(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(4)
        return self._rnea(q, np.zeros(4), np.zeros(4), gravity=True)

    def mass_matrix(self, q) -> np.ndarray:
        """Joint-space mass matrix via the closed-form Jacobian formula --
        see this module's docstring. Verified bit-identical (max abs diff
        ~1e-17 over 300 random q) to `_mass_matrix_rnea` below before this
        was adopted; that check is now a permanent regression test in
        `test_local.py`."""
        q = np.asarray(q, dtype=float).reshape(4)
        n = self.n
        R_before = [np.eye(3)]                    # world orientation BEFORE joint i's own rotation
        for i in range(n):
            R_before.append(R_before[-1] @ _rot(AXES[i], q[i]))
        p = [D[0].copy()]                          # joint-pivot origins in world (fixed D[0] first)
        for i in range(1, n):
            p.append(p[-1] + R_before[i] @ D[i])
        axis_world = [R_before[i] @ AXES[i] for i in range(n)]

        M = np.zeros((n, n))
        for k in range(n):
            Rk = R_before[k + 1]                   # link k's own world orientation
            p_ck = p[k] + Rk @ self.c[k]            # link k COM in world
            Jv = np.zeros((3, n)); Jw = np.zeros((3, n))
            for j in range(k + 1):
                Jv[:, j] = np.cross(axis_world[j], p_ck - p[j])
                Jw[:, j] = axis_world[j]
            Ik_world = Rk @ self.I[k] @ Rk.T
            M += self.m[k] * (Jv.T @ Jv) + Jw.T @ Ik_world @ Jw
        return 0.5 * (M + M.T)

    def _mass_matrix_rnea(self, q) -> np.ndarray:
        """Original approach (n separate RNEA calls, unit accelerations,
        gravity off) -- kept only so test_local.py can regression-check that
        the fast mass_matrix() above stays numerically identical to it."""
        q = np.asarray(q, dtype=float).reshape(4)
        M = np.zeros((4, 4))
        for j in range(4):
            e = np.zeros(4); e[j] = 1.0
            M[:, j] = self._rnea(q, np.zeros(4), e, gravity=False)
        return 0.5 * (M + M.T)

    def task_inertia(self, q, J) -> np.ndarray:
        M = self.mass_matrix(q)
        Minv = np.linalg.inv(M)
        return np.linalg.inv(J @ Minv @ J.T + 1e-6 * np.eye(3))
