"""Forward kinematics, translational Jacobian, and a gravity model for the
ROBOTIS OpenManipulator-X (RM-X52-TNM), a 4-DOF arm (joint1 yaw, joints 2-4
pitch) built entirely from XM430-W350 servos.

Link offsets are the canonical ROBOTIS `open_manipulator_libs` values. VERIFY
them against your friend's URDF before trusting hardware distances -- they set
the Jacobian, and a wrong Jacobian makes the torque map wrong. All values are
configurable from the robot YAML (`kinematics:` block).

The translational Jacobian is computed numerically from the FK, so it stays
consistent with whatever link offsets you set.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# canonical OpenManipulator-X link offsets [m] (translation to the next joint
# frame, expressed in the current joint frame), and joint axes.
# Verified against the ROBOTIS open_manipulator_x URDF (joint origins in MuJoCo):
DEFAULT_LINKS = {
    "d01": [0.012, 0.0, 0.0],     # base   -> joint1 (yaw, z)
    "d12": [0.0, 0.0, 0.0595],    # joint1 -> joint2 (pitch, y)
    "d23": [0.024, 0.0, 0.128],   # joint2 -> joint3 (pitch, y) -- the "kink"
    "d34": [0.124, 0.0, 0.0],     # joint3 -> joint4 (pitch, y)
    "d4e": [0.126, 0.0, 0.0],     # joint4 -> tool center point (TCP)
}
# link masses [kg] from the URDF (link2..link5). Placed at the distal joint
# origin -- an approximation of the true COMs, refined by the observer / on-hw.
DEFAULT_LINK_MASSES = [0.098, 0.139, 0.133, 0.143]
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
    links: dict = field(default_factory=lambda: {k: list(v) for k, v in DEFAULT_LINKS.items()})
    link_masses: list = field(default_factory=lambda: list(DEFAULT_LINK_MASSES))

    def frames(self, q: np.ndarray) -> list[np.ndarray]:
        """Return the world position of each joint origin and the end-effector.

        Order: [j1, j2, j3, j4, ee] as 3-vectors in the base frame.
        """
        q = np.asarray(q, dtype=float).reshape(4)
        L = self.links
        R = np.eye(3)
        p = np.zeros(3)
        pts = []
        # joint1 (yaw about z)
        p = p + R @ np.asarray(L["d01"]); R = R @ _rot("z", q[0]); pts.append(p.copy())
        # joint2 (pitch about y)
        p = p + R @ np.asarray(L["d12"]); R = R @ _rot("y", q[1]); pts.append(p.copy())
        # joint3
        p = p + R @ np.asarray(L["d23"]); R = R @ _rot("y", q[2]); pts.append(p.copy())
        # joint4
        p = p + R @ np.asarray(L["d34"]); R = R @ _rot("y", q[3]); pts.append(p.copy())
        # end-effector
        p = p + R @ np.asarray(L["d4e"]); pts.append(p.copy())
        return pts

    def fk(self, q: np.ndarray) -> np.ndarray:
        """End-effector position [x, y, z] in the base frame [m]."""
        return self.frames(q)[-1]

    def jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """3x4 translational Jacobian d(ee_pos)/d(q), numerically."""
        q = np.asarray(q, dtype=float).reshape(4)
        J = np.zeros((3, 4))
        for i in range(4):
            dq = q.copy(); dq[i] += eps
            J[:, i] = (self.fk(dq) - self.fk(q - np.eye(4)[i] * eps)) / (2 * eps)
        return J

    def _point_jacobian(self, q: np.ndarray, frame_index: int, eps: float = 1e-6) -> np.ndarray:
        """3x4 translational Jacobian of the frame at `frame_index` (0=j1 .. 4=ee)."""
        J = np.zeros((3, 4))
        for i in range(4):
            dp = np.eye(4)[i] * eps
            p_plus = self.frames(q + dp)[frame_index]
            p_minus = self.frames(q - dp)[frame_index]
            J[:, i] = (p_plus - p_minus) / (2 * eps)
        return J

    def gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """Joint torques [N.m] that COMPENSATE gravity (feedforward).

        Point masses m_i placed at the distal end of each link (frames j2, j3,
        j4, ee); tau = sum_i J_i^T (m_i g e_z). Approximate -- scale/calibrate
        with `gravity_scale` in the loop.
        """
        q = np.asarray(q, dtype=float).reshape(4)
        tau = np.zeros(4)
        for k, m_i in enumerate(self.link_masses):  # frame indices 1..4 = j2,j3,j4,ee
            Jp = self._point_jacobian(q, frame_index=k + 1)
            tau += Jp.T @ np.array([0.0, 0.0, m_i * G])
        return tau
