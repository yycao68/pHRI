"""
SO(3) utilities for Cartesian impedance control.

Conventions
-----------
Rotation matrices:  R ∈ SO(3),   R : body → world
Quaternions:        q = [w, x, y, z]   (scalar-first)
Angular velocity:   ω expressed in world frame unless noted
"""

import numpy as np


# ---------------------------------------------------------------------------
# Skew-symmetric map and its inverse
# ---------------------------------------------------------------------------

def skew(v: np.ndarray) -> np.ndarray:
    """hat map ℝ³ → so(3)."""
    v = v.flat
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])


def vee(S: np.ndarray) -> np.ndarray:
    """vee map so(3) → ℝ³  (inverse of skew)."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


# ---------------------------------------------------------------------------
# Rotation error on SO(3)
# ---------------------------------------------------------------------------

def rotation_error_matrix(R_d: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Geometric orientation error expressed in the world frame.

    For the impedance law we need a vector e_R ∈ ℝ³ such that
    feedback torque τ_R = -K_R e_R drives R → R_d.

    Using the formula (Park & Chung 1995 / Bullo & Murray 1999):

        e_R = ½ vee(R_d Rᵀ - R R_dᵀ)
            = ½ vee(R_d Rᵀ - (R_d Rᵀ)ᵀ)

    This is the skew-symmetric part of the relative rotation matrix.
    It is zero iff R = R_d, and its direction is the rotation axis scaled
    by sin(θ) where θ is the geodesic distance on SO(3).

    Parameters
    ----------
    R_d : (3, 3) desired rotation (world frame)
    R   : (3, 3) current rotation (world frame)

    Returns
    -------
    e_R : (3,) orientation error vector, world frame
    """
    Re = R_d @ R.T                      # relative rotation  R_e = R_d R⁻¹
    return 0.5 * vee(Re - Re.T)


def rotation_error_body(R_d: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Same error expressed in the *current* body frame.

    Useful when the angular velocity ω is given in body frame:

        e_R_body = ½ vee(Rᵀ R_d - R_dᵀ R)
    """
    Re = R.T @ R_d
    return 0.5 * vee(Re - Re.T)


# ---------------------------------------------------------------------------
# Quaternion helpers (scalar-first convention: q = [w, x, y, z])
# ---------------------------------------------------------------------------

def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product p ⊗ q, scalar-first."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_error(q_d: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Quaternion orientation error e_q ∈ ℝ³.

    e_q = 2 * vec(q_e)   where  q_e = q_d ⊗ q⁻¹

    This matches the rotation_error_matrix formulation to first order and
    is convenient when computing with quaternion-based state estimates.
    The sign convention is such that the impedance law -K_q e_q drives q → q_d.
    """
    q_e = quat_multiply(q_d, quat_conjugate(q))
    if q_e[0] < 0:          # shortest-path: choose the hemisphere closer to identity
        q_e = -q_e
    return 2.0 * q_e[1:]    # vector part only


def quat_to_rotation(q: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] → rotation matrix R ∈ SO(3)."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y**2 + z**2),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),         1 - 2*(x**2 + z**2),  2*(y*z - w*x)],
        [2*(x*z - w*y),         2*(y*z + w*x),         1 - 2*(x**2 + y**2)],
    ])


def rotation_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion [w,x,y,z] (Shepperd method)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return np.array([0.25 / s,
                         (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s,
                         (R[1, 0] - R[0, 1]) * s])
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s,
                         0.25 * s,
                         (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s,
                         (R[0, 1] + R[1, 0]) / s,
                         0.25 * s,
                         (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[1, 0] - R[0, 1]) / s,
                         (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s,
                         0.25 * s])
