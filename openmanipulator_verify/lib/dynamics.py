"""Rigid-body dynamics (gravity G(q) and joint-space mass matrix M(q)) for the
OpenManipulator-X, via recursive Newton-Euler (RNEA). Inertial parameters are
from the ROBOTIS URDF (validated against MuJoCo). This lets the controller do
proper operational-space control -- F_task = Lambda(q)(xdd_d+u), tau = J^T F + G
-- instead of a scalar task mass + crude gravity, which do not survive real
dynamics.

M(q) is built column-by-column with RNEA (unit accelerations, gravity off);
G(q) is RNEA at zero velocity/acceleration with gravity on.
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
