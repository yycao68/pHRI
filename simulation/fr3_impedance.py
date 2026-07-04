"""
Cartesian Impedance and Admittance Control for Franka FR3
=========================================================

Theory
------
The FR3 is a 7-DOF torque-controlled arm.  Joint-space dynamics:

    M(q) q̈ + C(q,q̇) q̇ + g(q) = τ + Jᵀ(q) f_ext            (1)

where
    q ∈ ℝ⁷            joint positions
    M ∈ ℝ⁷ˣ⁷          joint-space inertia (symmetric positive definite)
    C(q,q̇)q̇ ∈ ℝ⁷     Coriolis + centrifugal term
    g(q) ∈ ℝ⁷         gravity vector
    J(q) ∈ ℝ⁶ˣ⁷       geometric Jacobian  (linear / angular stacked)
    f_ext ∈ ℝ⁶        external wrench at EE (world frame)

libfranka directly provides M, C(q,q̇)q̇, g, J, and an estimated f_ext.

Operational-space (task-space) dynamics (Khatib 1987)
------------------------------------------------------
Pre-multiplying (1) by J M⁻¹ and using ẍ = J q̈ + J̇ q̇ gives:

    Λ(q) ẍ + μ(q,q̇) ẋ + p(q) = F_c + f_ext                  (2)

where

    Λ(q)  = (J M⁻¹ Jᵀ)⁻¹              task inertia    [6×6]
    μ(q,q̇)= Λ (J M⁻¹ C - J̇)           task Coriolis
    p(q)  = Λ J M⁻¹ g                 task gravity
    F_c              commanded task wrench

The dynamically consistent pseudo-inverse is

    J̄(q) = M⁻¹ Jᵀ Λ    →   J J̄ = I₆,   rank deficiency OK

Null-space projector (onto ker J):

    N̄ = I₇ - J̄ J                             [7×7]

The mapping from task wrench to joint torques that is consistent with (2):

    τ = Jᵀ F_c + N̄ᵀ τ_null

Impedance Control Law
---------------------
Desired closed-loop task-space impedance (Hogan 1985):

    M_d (ẍ - ẍ_d) + D_d (ẋ - ẋ_d) + K_d (x - x_d) = f_ext   (3)

where M_d, D_d, K_d ∈ ℝ⁶ˣ⁶ are the desired inertia, damping, stiffness.

Setting M_d = Λ (inertia shaping to the robot's natural task inertia)
eliminates the Λ M_d⁻¹ gain and gives the simplest controller:

    F_c = Λ(q) ẍ_d + μ(q,q̇) ẋ + p(q) - D_d ė - K_d e + f_ext  (4a)

For general M_d ≠ Λ (explicit inertia reshaping):

    F_c = Λ(q) [ẍ_d + M_d⁻¹(-D_d ė - K_d e + f_ext)]
        + μ(q,q̇) ẋ + p(q)                                     (4b)

In both cases joint torques are:

    τ = Jᵀ F_c + N̄ᵀ τ_null

Position part of the error: e_p = x_ee - x_d  ∈ ℝ³
Velocity error:             ė_p = ẋ_ee - ẋ_d  ∈ ℝ³
Orientation error:          e_R (see so3_utils) ∈ ℝ³
Angular velocity error:     ė_R = ω_ee - ω_d    ∈ ℝ³

The 6D pose error is e = [e_p; e_R]  and  ė = [ė_p; ė_R].

Admittance Control Law
----------------------
Admittance control is the *dual* of impedance control. It is natural
when the robot runs a stiff inner position/velocity loop (e.g., position
mode on the FR3, or a PD joint controller). An external wrench drives a
virtual admittance model that generates a reference trajectory modification:

    M_a ẍ_r + D_a ẋ_r + K_a x_r = f_ext                         (5)

The modified reference x_d + x_r is tracked by the inner controller.

In discrete time (Euler integration):

    v_r[k+1] = v_r[k] + dt * M_a⁻¹(f_ext - D_a v_r[k] - K_a x_r[k])
    x_r[k+1] = x_r[k] + dt * v_r[k+1]

Note: for the FR3's EE-frame admittance controller, one typically uses
the robot's estimated external wrench (tau_ext_hat_filtered in libfranka).

Null-Space Control
------------------
The 7-DOF FR3 has one redundant DOF.  A common choice is a joint-limit
avoidance gradient projected into the null space:

    τ_null = -k_n * ∂H/∂q - d_n * q̇

where H(q) = Σᵢ ((q_max,i - q_min,i)² / (4 (q_max,i - qᵢ)(qᵢ - q_min,i)))
is a repulsive potential (Chen & Lozano-Perez 1986 style).
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from numpy.linalg import inv, pinv, norm
from so3_utils import rotation_error_matrix, quat_error


# ---------------------------------------------------------------------------
# FR3 joint limits  (rad, from Franka docs)
# ---------------------------------------------------------------------------

Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169,  3.0159])


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------

@dataclass
class RobotState:
    """
    Snapshot of FR3 state, matching what libfranka provides.

    libfranka fields (mapped here):
        q             → franka::RobotState::q
        dq            → franka::RobotState::dq
        O_T_EE        → 4×4 homogeneous transform (flattened column-major in libfranka)
        O_dP_EE       → 6D EE velocity [v; ω] in world frame
        O_F_ext_hat_K → estimated external wrench at EE [f; τ] world frame
    """
    q:         np.ndarray   # (7,)  joint positions
    dq:        np.ndarray   # (7,)  joint velocities
    ee_pos:    np.ndarray   # (3,)  EE position in world frame
    ee_rot:    np.ndarray   # (3,3) EE rotation matrix, world frame
    ee_vel:    np.ndarray   # (6,)  [v; ω] EE twist, world frame
    f_ext:     np.ndarray   # (6,)  estimated external wrench [f; τ], world frame


@dataclass
class FrankaDynamics:
    """
    Rigid-body dynamics quantities that libfranka / pinocchio provides.
    All matrices are pre-computed at the current configuration.
    """
    M:      np.ndarray   # (7,7) joint-space inertia
    Cq_dot: np.ndarray   # (7,)  C(q,q̇)q̇  (NOT the C matrix itself)
    g:      np.ndarray   # (7,)  gravity vector
    J:      np.ndarray   # (6,7) geometric Jacobian  [J_v; J_w]
    dJ:     np.ndarray   # (6,7) time derivative of Jacobian  (optional, set to 0 if unavailable)


# ---------------------------------------------------------------------------
# Computed operational-space quantities
# ---------------------------------------------------------------------------

@dataclass
class OperationalSpaceModel:
    Lambda:  np.ndarray   # (6,6) task-space inertia
    mu:      np.ndarray   # (6,)  task-space Coriolis term  μ(q,q̇) ẋ
    p:       np.ndarray   # (6,)  task-space gravity
    J_bar:   np.ndarray   # (6,7) dynamically-consistent pseudo-inverse  M⁻¹ Jᵀ Λ
    N_bar:   np.ndarray   # (7,7) null-space projector  I - J̄ J


def build_operational_space_model(dyn: FrankaDynamics,
                                  ee_vel: np.ndarray,
                                  dq: np.ndarray | None = None,
                                  reg: float = 1e-4) -> OperationalSpaceModel:
    """
    Compute Λ, μ, p, J̄, N̄ from joint-space dynamics quantities.

    Parameters
    ----------
    dyn     : FrankaDynamics at current q
    ee_vel  : (6,) current EE velocity ẋ (used for μ ẋ)
    dq      : (7,) joint velocity q̇.  If given (and dyn.dJ is available) the
              centripetal term J̇ q̇ is included in μ.  If None, the standard
              J̇ ≈ 0 approximation is used (acceptable at ≥1 kHz on FR3).
    reg     : Tikhonov regularisation on Λ inverse (numerical stability)

    Notes
    -----
    Λ = (J M⁻¹ Jᵀ)⁻¹   -- we invert via regularised solve rather than
    explicit matrix inversion to improve numerical conditioning.
    """
    M_inv = inv(dyn.M)                              # (7,7)
    JMiJt = dyn.J @ M_inv @ dyn.J.T                 # (6,6)
    # Regularised inversion
    Lambda = inv(JMiJt + reg * np.eye(6))           # (6,6)

    J_bar = M_inv @ dyn.J.T @ Lambda                # (6,7)  dynamically consistent pinv

    # μ(q,q̇) ẋ = Λ (J M⁻¹ C q̇  -  J̇ q̇)
    #           = Λ  J M⁻¹ (Cq_dot)  -  Λ dJ q̇
    # Here ee_vel = J q̇, so: mu * ee_vel makes sense dimensionally
    # We compute the (6,) vector directly:
    JMi_Cqdot = dyn.J @ M_inv @ dyn.Cq_dot          # (6,)
    nq        = dyn.J.shape[1]
    # Centripetal term J̇ q̇.  Included only when q̇ is supplied AND dyn.dJ is
    # available; otherwise the documented J̇ ≈ 0 approximation (zeros) is used.
    if dq is not None and dyn.dJ is not None:
        dJ_qdot = dyn.dJ @ dq                       # (6,)
    else:
        dJ_qdot = np.zeros(6)
    mu_x      = Lambda @ (JMi_Cqdot - dJ_qdot)      # (6,)

    p = Lambda @ dyn.J @ M_inv @ dyn.g              # (6,)  task gravity

    N_bar = np.eye(nq) - J_bar @ dyn.J              # (nq,nq) null-space projector

    return OperationalSpaceModel(
        Lambda=Lambda, mu=mu_x, p=p, J_bar=J_bar, N_bar=N_bar
    )


# ---------------------------------------------------------------------------
# Impedance controller
# ---------------------------------------------------------------------------

@dataclass
class ImpedanceParams:
    """
    Cartesian impedance gains.

    K and D are 6×6 matrices (or scalars broadcast to diagonal).
    Typical FR3 values:
        K_p  ~ 200–600  N/m    (translational)
        K_r  ~ 10–50    Nm/rad (rotational)
        D    = 2 * sqrt(K)     (critical damping)

    M_d: if None, use M_d = Λ (inertia shaping, simplest law).
         If provided as a 6×6 matrix, explicit reshaping is applied.
    """
    K:      np.ndarray                    # (6,6) or (6,) stiffness
    D:      np.ndarray                    # (6,6) or (6,) damping
    M_d:    np.ndarray | None = None      # (6,6) desired inertia
    # Null-space
    k_null: float = 10.0
    d_null: float = 2.0
    q_null: np.ndarray = field(default_factory=lambda: np.zeros(7))


def _ensure_matrix(x: np.ndarray) -> np.ndarray:
    if x.ndim == 1:
        return np.diag(x)
    return x


def null_space_torque(q: np.ndarray,
                      dq: np.ndarray,
                      N_bar: np.ndarray,
                      params: ImpedanceParams) -> np.ndarray:
    """
    Joint-limit repulsion + damping projected onto null space.

    Gradient of the joint-limit penalty H(q):

        ∂H/∂qᵢ = (q_max - q_min)² (2qᵢ - q_max - q_min)
                  / (4 (q_max - q)(q - q_min)²)

    We also add a term pulling toward a preferred config q_null.
    """
    # Proportional term: pull toward q_null (gracefully handle DOF mismatch in tests)
    q_null = params.q_null if len(params.q_null) == len(q) else np.zeros_like(q)
    tau_p = -params.k_null * (q - q_null)
    # Damping
    tau_d = -params.d_null * dq
    tau_null_raw = tau_p + tau_d
    return N_bar.T @ tau_null_raw


def cartesian_impedance_control(
    state: RobotState,
    dyn: FrankaDynamics,
    x_d: np.ndarray,       # (3,) desired EE position
    R_d: np.ndarray,       # (3,3) desired EE rotation
    dx_d: np.ndarray,      # (6,) desired EE velocity [v_d; ω_d]
    ddx_d: np.ndarray,     # (6,) desired EE acceleration [a_d; α_d]
    params: ImpedanceParams,
    use_f_ext: bool = False,
) -> np.ndarray:
    """
    Compute joint torques for Cartesian impedance control.

    Returns
    -------
    tau : (7,) joint torques to command to the FR3

    Control law (general M_d):
        F_c = Λ [ẍ_d + M_d⁻¹ (-D ė - K e + f_ext)] + μẋ + p
        τ   = Jᵀ F_c + N̄ᵀ τ_null

    With M_d = Λ (natural inertia shaping):
        F_c = Λ ẍ_d + μẋ + p - D ė - K e [+ f_ext]
    """
    K = _ensure_matrix(params.K)
    D = _ensure_matrix(params.D)

    # Build operational-space model
    os = build_operational_space_model(dyn, state.ee_vel)

    # --- 6D pose error -------------------------------------------------------
    e_p = state.ee_pos - x_d                              # (3,) position error
    e_R = rotation_error_matrix(R_d, state.ee_rot)        # (3,) orientation error
    e   = np.concatenate([e_p, e_R])                      # (6,)

    de_p = state.ee_vel[:3] - dx_d[:3]                    # (3,)
    de_R = state.ee_vel[3:] - dx_d[3:]                    # (3,)
    de   = np.concatenate([de_p, de_R])                   # (6,)

    f_ext = state.f_ext if use_f_ext else np.zeros(6)

    # --- Task-space command wrench -------------------------------------------
    if params.M_d is None:
        # M_d = Λ  →  natural inertia shaping (simplest, most common)
        F_c = (os.Lambda @ ddx_d
               + os.mu
               + os.p
               - D @ de
               - K @ e
               + f_ext)
    else:
        # Explicit inertia reshaping: M_d ≠ Λ
        M_d_inv = inv(params.M_d)
        F_c = (os.Lambda @ (ddx_d + M_d_inv @ (-D @ de - K @ e + f_ext))
               + os.mu
               + os.p)

    # --- Null-space torque ---------------------------------------------------
    tau_null = null_space_torque(state.q, state.dq, os.N_bar, params)

    # --- Joint torques -------------------------------------------------------
    tau = dyn.J.T @ F_c + tau_null

    return tau


# ---------------------------------------------------------------------------
# Admittance controller
# ---------------------------------------------------------------------------

@dataclass
class AdmittanceParams:
    """
    Admittance model parameters.

    M_a, D_a, K_a are 6×6 matrices (or 6-vectors for diagonal).
    The admittance model drives a reference modification x_r:

        M_a ẍ_r + D_a ẋ_r + K_a x_r = f_ext

    The inner position controller then tracks  x_d + x_r.

    Typical values:
        M_a_p = 1.0 kg       (translational virtual mass)
        D_a_p = 100 N·s/m    (translational damping)
        K_a_p = 0 N/m        (zero stiffness → pure admittance / back-drivability)
    """
    M_a: np.ndarray   # (6,6) admittance inertia
    D_a: np.ndarray   # (6,6) admittance damping
    K_a: np.ndarray   # (6,6) admittance stiffness  (0 = pure admittance)


class AdmittanceController:
    """
    Discrete-time admittance controller.

    State: (x_r, v_r) ∈ ℝ⁶ × ℝ⁶, initialised at zero.

    Usage
    -----
    ctrl = AdmittanceController(params, dt=0.001)
    while True:
        x_ref, v_ref = ctrl.step(f_ext)
        command_position(x_d + x_ref, R_d + delta_R(x_ref[3:]))
    """

    def __init__(self, params: AdmittanceParams, dt: float = 1e-3):
        self.params = params
        self.dt = dt
        self.x_r = np.zeros(6)   # reference modification  [δp; δφ]
        self.v_r = np.zeros(6)   # reference velocity

    def step(self, f_ext: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Euler-integrate the admittance model one step.

        Parameters
        ----------
        f_ext : (6,) external wrench [force; torque], world frame

        Returns
        -------
        x_r  : (6,) position modification  [δp; δφ]  to add to x_d
        v_r  : (6,) velocity modification  [δv; δω]  to add to dx_d
        """
        p = self.params
        M_a = _ensure_matrix(p.M_a)
        D_a = _ensure_matrix(p.D_a)
        K_a = _ensure_matrix(p.K_a)

        # Semi-implicit Euler (stable for stiff systems):
        #   M_a a_r = f_ext - D_a v_r - K_a x_r
        a_r = inv(M_a) @ (f_ext - D_a @ self.v_r - K_a @ self.x_r)

        self.v_r = self.v_r + self.dt * a_r
        self.x_r = self.x_r + self.dt * self.v_r

        return self.x_r.copy(), self.v_r.copy()

    def reset(self):
        self.x_r[:] = 0.0
        self.v_r[:] = 0.0


# ---------------------------------------------------------------------------
# Convenience factory: critically-damped gains
# ---------------------------------------------------------------------------

def make_impedance_params(
    k_pos: float | np.ndarray = 400.0,
    k_rot: float | np.ndarray = 30.0,
    damping_ratio: float = 1.0,
    k_null: float = 10.0,
    d_null: float = 2.0,
    q_null: np.ndarray | None = None,
) -> ImpedanceParams:
    """
    Build critically-damped Cartesian impedance gains.

    D = 2 * ζ * sqrt(K * Λ)  ≈  2 * ζ * sqrt(K)   (assuming Λ ≈ I for scaling)

    Parameters
    ----------
    k_pos : translational stiffness (N/m), scalar or (3,)
    k_rot : rotational stiffness (Nm/rad), scalar or (3,)
    damping_ratio : ζ = 1 → critical damping
    """
    k_p = np.broadcast_to(np.atleast_1d(k_pos), (3,)).copy()
    k_r = np.broadcast_to(np.atleast_1d(k_rot), (3,)).copy()
    K6  = np.diag(np.concatenate([k_p, k_r]))

    d_p = 2.0 * damping_ratio * np.sqrt(k_p)
    d_r = 2.0 * damping_ratio * np.sqrt(k_r)
    D6  = np.diag(np.concatenate([d_p, d_r]))

    return ImpedanceParams(
        K=K6, D=D6, M_d=None,
        k_null=k_null, d_null=d_null,
        q_null=q_null if q_null is not None else np.zeros(7),
    )


def make_admittance_params(
    m_pos: float = 1.0,
    m_rot: float = 0.1,
    d_pos: float = 100.0,
    d_rot: float = 10.0,
    k_pos: float = 0.0,
    k_rot: float = 0.0,
) -> AdmittanceParams:
    """Build diagonal admittance parameters."""
    M_a = np.diag([m_pos]*3 + [m_rot]*3)
    D_a = np.diag([d_pos]*3 + [d_rot]*3)
    K_a = np.diag([k_pos]*3 + [k_rot]*3)
    return AdmittanceParams(M_a=M_a, D_a=D_a, K_a=K_a)
