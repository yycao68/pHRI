"""Forward kinematics and translational Jacobian for the 3-DOF arm derived from
OpenManipulator-X after the waist (original joint1, yaw about z) was physically
removed. joint2 is now bolted directly to the base -- no rotating waist stage.

Because the removed joint rotated about the vertical (z) axis and gravity is
also along z, removing it does not change the dynamics of the remaining
pitch joints; the original base->joint1->joint2 offset chain is simply folded
into a single fixed base->joint1(new) translation (`d0e_base`).

Joint order (all pitch, about y): [j1, j2, j3] = original [joint2, joint3, joint4].

IMPORTANT: `d0e_base` below assumes joint2 is mounted at exactly the same
physical offset the original d01+d12 chain implied ([0.012, 0.0, 0.0595] m).
If the actual mounting bracket has different dimensions, measure it with
calipers/ruler and update `d0e_base` accordingly -- this directly sets the
Jacobian and therefore the torque map.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# 3-DOF link offsets [m], after folding the removed waist joint into d0e_base.
DEFAULT_LINKS = {
    "d0e_base": [0.012, 0.0, 0.0595],  # base -> joint1 (fixed, was d01+d12; VERIFY on hardware)
    "d01": [0.024, 0.0, 0.128],        # joint1 (pitch) -> joint2   (was d23)
    "d12": [0.124, 0.0, 0.0],          # joint2 (pitch) -> joint3   (was d34)
    "d1e": [0.126, 0.0, 0.0],          # joint3 (pitch) -> TCP      (was d4e)
}
# link masses [kg], dropped the mass of the removed waist link.
DEFAULT_LINK_MASSES = [0.139, 0.133, 0.143]
G = 9.81


def _rot(axis: str, a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])
    raise ValueError(axis)


@dataclass
class OpenManipulatorKinematics:
    """3-DOF (all pitch) kinematics for the waist-removed OpenManipulator-X."""

    links: dict = field(default_factory=lambda: {k: list(v) for k, v in DEFAULT_LINKS.items()})
    link_masses: list = field(default_factory=lambda: list(DEFAULT_LINK_MASSES))
    n: int = 3

    def frames(self, q: np.ndarray) -> list[np.ndarray]:
        """Return world position of each joint origin and the end-effector.

        Order: [j1, j2, j3, ee] as 3-vectors in the base frame.
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        L = self.links
        R = np.eye(3)
        p = np.zeros(3)
        pts = []
        # fixed base offset (no joint here anymore -- waist removed)
        p = p + R @ np.asarray(L["d0e_base"])
        # joint1 (pitch about y)
        R = R @ _rot("y", q[0]); pts.append(p.copy())
        p = p + R @ np.asarray(L["d01"]); R = R @ _rot("y", q[1]); pts.append(p.copy())
        # joint2
        p = p + R @ np.asarray(L["d12"]); R = R @ _rot("y", q[2]); pts.append(p.copy())
        # joint3
        p = p + R @ np.asarray(L["d1e"]); pts.append(p.copy())
        return pts

    def fk(self, q: np.ndarray) -> np.ndarray:
        """End-effector position [x, y, z] in the base frame [m]."""
        return self.frames(q)[-1]

    def jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """3x3 translational Jacobian d(ee_pos)/d(q), numerically."""
        q = np.asarray(q, dtype=float).reshape(self.n)
        J = np.zeros((3, self.n))
        for i in range(self.n):
            dq = q.copy(); dq[i] += eps
            J[:, i] = (self.fk(dq) - self.fk(q - np.eye(self.n)[i] * eps)) / (2 * eps)
        return J

    def _point_jacobian(self, q: np.ndarray, frame_index: int, eps: float = 1e-6) -> np.ndarray:
        """3xN translational Jacobian of the frame at `frame_index` (0=j1 .. n=ee)."""
        J = np.zeros((3, self.n))
        for i in range(self.n):
            dp = np.eye(self.n)[i] * eps
            p_plus = self.frames(q + dp)[frame_index]
            p_minus = self.frames(q - dp)[frame_index]
            J[:, i] = (p_plus - p_minus) / (2 * eps)
        return J

    def solve_ik_xz(self, target_xz: np.ndarray, q0: np.ndarray, max_iter: int = 100,
                     tol: float = 1e-5, damping: float = 1e-3) -> tuple[np.ndarray, float, float]:
        """Damped-least-squares IK in the x-z plane only (matches the 2D task
        space every controller in this project actually uses). Returns
        (q_solution, final_residual_m, min_singular_value_seen).

        Originally tools/check_circle_workspace.py's own local function;
        moved here so lib/trajectory.py's IK-based posture-reference table
        (see CartesianTrajectory) can reuse the exact same solver instead of
        a second, potentially-drifting copy of the same math."""
        q = np.asarray(q0, dtype=float).copy()
        min_sv = np.inf
        for _ in range(max_iter):
            ee = self.fk(q)
            err = target_xz - ee[[0, 2]]
            if np.linalg.norm(err) < tol:
                break
            J_xz = self.jacobian(q)[[0, 2], :]
            sv = np.linalg.svd(J_xz, compute_uv=False)
            min_sv = min(min_sv, sv.min())
            # damped least squares: dq = J^T (J J^T + damping^2 I)^-1 err
            JJt = J_xz @ J_xz.T + (damping ** 2) * np.eye(2)
            dq = J_xz.T @ np.linalg.solve(JJt, err)
            q = q + np.clip(dq, -0.2, 0.2)  # limit per-iteration step for stability
        residual = float(np.linalg.norm(target_xz - self.fk(q)[[0, 2]]))
        return q, residual, min_sv

    def gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """Approximate gravity-compensation joint torques [N.m] (point-mass model).

        Not used by the main control loop (dynamics.py's RNEA model is used
        there); kept as a cheap sanity-check / fallback.
        """
        q = np.asarray(q, dtype=float).reshape(self.n)
        tau = np.zeros(self.n)
        for k, m_i in enumerate(self.link_masses):  # frame indices 1..n = j2,j3,ee
            Jp = self._point_jacobian(q, frame_index=k + 1)
            tau += Jp.T @ np.array([0.0, 0.0, m_i * G])
        return tau
