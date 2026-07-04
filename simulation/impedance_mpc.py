"""
Double-integrator predictive Cartesian controller — Franka FR3
==============================================================

Two-layer architecture used by the double-integrator pHRI paper:

    τ = τ_ff   (Layer 1: feedforward nonlinear inversion)
      + J_v^T F_mpc   (Layer 2: receding-horizon QP correction)
      + J_w^T F_orient   (orientation stabilisation, simple impedance)
      + N̄^T τ_null   (null-space joint-limit avoidance)

Layer 1 cancels gravity, Coriolis, and the task-space inertia feedforward:

    τ_ff = qfrc_bias + J_v^T Λ_pos(q) ẍ_d

After this cancellation the residual plant seen by Layer 2 is a
*constant*-A_d double integrator in Cartesian 3D position:

    ë = -Λ_pos^{-1}(q) F_mpc + d(t)

    x_e[k+1] = A_d x_e[k] + B_d(ρ_k) F_mpc[k]
    A_d = [[I3, Δt I3],   (constant — precomputed once)
           [0,  I3    ]]
    B_d = [[0           ],   (parameter-varying via Λ_pos(q))
           [-Λ_pos^{-1} Δt]]

Layer 2 solves a small QP with 3N ≤ 30 decision variables at the selected
MPC rate.

Optional Kalman augmented state [e; ė; d̂] (9D) provides offset-free
tracking under persistent human forces (zero-steady-state-error guarantee).

The file and class names retain "impedance_mpc" for compatibility with the
older simulation scripts; the implemented backbone is the linear residual
double-integrator model used in the current manuscript.
"""

from __future__ import annotations
from dataclasses import dataclass, field
import sys
from pathlib import Path
import numpy as np
from numpy.linalg import inv
from scipy.optimize import LinearConstraint, minimize

# Add simulation/ to path so fr3_impedance, fr3_mujoco, so3_utils are importable
sys.path.insert(0, str(Path(__file__).parent.parent / "simulation"))


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

@dataclass
class ImpedanceMPCParams:
    # Horizon and timing
    N:          int   = 10       # prediction steps
    dt_mpc:     float = 0.01     # MPC sample time (s)  → 100 Hz

    # Position tracking cost  (3×3 positive-definite)
    Q_pos:     np.ndarray = field(default_factory=lambda: 2e4 * np.eye(3))
    Q_vel:     np.ndarray = field(default_factory=lambda: 50.0 * np.eye(3))
    Q_f_scale: float      = 5.0  # terminal cost = Q_f_scale × Q_stage

    # Effort weight (3×3 positive-definite)
    R_u:       np.ndarray = field(default_factory=lambda: 1e-6 * np.eye(3))

    # ── Impedance-equivalence mode (Theorem 1) ────────────────────────────
    # When impedance_track=True the QP tracks the prescribed task-space
    # impedance law F = K_d e + D_d ė (offset-free via −d̂) instead of an
    # LQ-regulation cost.  The unconstrained, disturbance-free solution is
    # then EXACTLY the classical impedance gain (K_d, D_d) — the QP deviates
    # only to honour the F_max box constraint.  D_d = 2ζ√(K_d) (critical).
    # Optional: prescribe the impedance gain directly instead of using the LQ
    # cost.  Left OFF by default — the LQ-MPC (Theorem 1: exact equivalence to
    # the realized impedance Λ K_∞) gives the predictive high-bandwidth tracking
    # used in the experiments.  Set True only if exact (K_d, D_d) assignment is
    # required (trades predictive look-ahead for exact gain matching).
    impedance_track: bool  = False
    k_imp:           float = 300.0   # prescribed translational stiffness (N/m)
    zeta_imp:        float = 1.0     # damping ratio (1 → critical)

    # ── Model-Predictive Variable-Impedance baseline (MPVIC) ─────────────────
    # A predictive variable-impedance comparator in the class of Liu et al. and
    # Roveda et al.: at each QP step the apparent stiffness is selected from a
    # candidate set by rolling out the horizon and minimising a weighted sum of
    # tracking error and applied task force, given the SAME Kalman disturbance
    # estimate d̂ available to the proposed controller.  Crucially, d̂ is used
    # only to *schedule* the stiffness, NOT to cancel the disturbance
    # (F_mpc = K* e + D* ė, with no −d̂ offset-free term).  This isolates the
    # paper's claim: variable impedance adapts the apparent stiffness but leaves
    # a nonzero steady-state error e_∞ = K*⁻¹ F_h under sustained force.
    variable_impedance: bool = False
    vic_K_set: np.ndarray = field(default_factory=lambda: np.array(
        [200.0, 400.0, 800.0, 1500.0, 3000.0]))  # candidate stiffness (N/m)
    vic_zeta:  float = 1.0     # damping ratio for the scheduled stiffness
    vic_w_e:   float = 2e4     # tracking-error weight in the K-selection cost
    vic_w_F:   float = 3e-3    # task-force weight  (interior optimum ⇒ finite K*)

    # MPC corrective force bound (N, ∞-norm per component)
    F_max:     float = 150.0
    tau_max:   np.ndarray = field(default_factory=lambda: np.array(
        [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]))  # FR3 torque limits

    # Orientation stabilisation (runs every inner step, separate from QP)
    K_rot:     float = 20.0   # Nm/rad
    D_rot:     float =  6.0   # Nm·s/rad

    # Null-space joint-limit avoidance
    k_null:    float = 10.0
    d_null:    float =  2.0

    # ── Robot-specific fields — OVERRIDE for platforms other than FR3 ──────────
    # q_null: rest-pose attractor for the centering spring.  Must be a feasible
    #   joint configuration (inside all joint limits).  The FR3 neutral pose is
    #   the default; set this to the robot's standard home pose.
    q_null:    np.ndarray = field(default_factory=lambda: np.array(
        [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]))   # FR3 neutral

    # q_min / q_max: software joint-position limits (rad).  Defaults are FR3
    #   values from libfranka.  Replace with your robot's limits.
    q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065,  0.5445, -3.0159]))  # FR3
    q_max: np.ndarray = field(default_factory=lambda: np.array(
        [ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159]))  # FR3
    # ─────────────────────────────────────────────────────────────────────────

    # Soft joint-limit barrier (inverse-barrier potential, active within barrier_margin)
    k_barrier:      float = 50.0   # Nm — null-space barrier gain
    barrier_margin: float = 0.10   # fraction of joint range that triggers the barrier

    # Cartesian workspace projection (task-space component of joint barrier)
    # Projects the barrier gradient through the Jacobian to offset p_d away from
    # limit-inducing workspace regions, complementing the null-space barrier above.
    k_ws:        float = 5e-4   # m per (Nm/rad) — workspace correction gain
    max_ws_corr: float = 0.06   # m — hard cap on Cartesian reference offset

    # Kalman process / observation noise
    Q_proc_d:  float = 10.0   # disturbance random-walk variance
    R_obs_e:   float = 1e-3   # position-error observation noise
    R_obs_de:  float = 1e-2   # velocity-error observation noise

    # QP solver: 'scipy' (SLSQP, default) or 'osqp' (requires `pip install osqp`)
    # Use 'osqp' to match the OSQP C++ library used in deployment.
    solver: str = 'osqp'


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class ImpedanceMPCController:
    """
    Two-layer double-integrator predictive Cartesian controller for FR3.

    Usage
    -----
    ctrl = ImpedanceMPCController(params, use_kalman=True)
    ctrl.reset()
    for step in range(n_steps):
        tau, F_mpc = ctrl.control(ee_pos, ee_vel, ee_rot,
                                   p_d, dp_d, ddp_d, R_d,
                                   dyn, q, dq)
        env.apply_torque(tau)
        env.step()
    """

    def __init__(self, params: ImpedanceMPCParams, use_kalman: bool = True):
        self.p          = params
        self.use_kalman = use_kalman
        N   = params.N
        dt  = params.dt_mpc

        # Constant A_d (6×6)
        self.A_d = np.block([
            [np.eye(3), dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)],
        ])

        # Precompute A_d^k for k = 0 … N-1
        self._Ad_pow = [np.eye(6)]
        for _ in range(N - 1):
            self._Ad_pow.append(self._Ad_pow[-1] @ self.A_d)

        # Free-response matrix Φ ∈ ℝ^{6N×6}  (A_d^1, A_d^2, …, A_d^N)
        self.Phi = np.vstack(
            [self._Ad_pow[k] @ self.A_d for k in range(N)]
        )

        # Block-diagonal stage cost Q̄ and terminal cost Q_f
        Q_blk = np.block([
            [params.Q_pos,     np.zeros((3, 3))],
            [np.zeros((3, 3)), params.Q_vel],
        ])
        Q_f = params.Q_f_scale * Q_blk
        self.Q_bar = np.zeros((6 * N, 6 * N))
        for i in range(N - 1):
            self.Q_bar[6*i:6*(i+1), 6*i:6*(i+1)] = Q_blk
        self.Q_bar[6*(N-1):, 6*(N-1):] = Q_f

        self.R_bar = np.kron(np.eye(N), params.R_u)

        # Kalman augmented state: [e(3); ė(3); d̂(3)]  ∈ ℝ^9
        self.x_aug       = np.zeros(9)
        self.P_aug       = np.eye(9)
        self._first_step = True
        self._F_prev     = np.zeros(3)         # F_mpc from previous MPC call
        self.F_seq       = np.zeros((params.N, 3))  # full N-step QP solution

        # OSQP instance — created lazily on first QP call, reused thereafter
        self._osqp_prob: object = None

        # MPVIC baseline: last selected stiffness (for logging / warm continuity)
        self._vic_K_last = float(np.median(params.vic_K_set))

    # ------------------------------------------------------------------
    # MPVIC baseline: predictive stiffness selection
    # ------------------------------------------------------------------

    def _select_variable_stiffness(
        self,
        x_e:     np.ndarray,   # (6,) error state [e; ė]
        d_hat:   np.ndarray,   # (3,) force-form disturbance estimate (enters via B_d)
        Lam_inv: np.ndarray,   # (3,3) Λ⁻¹
    ) -> tuple[float, float]:
        """Pick (K*, D*) from the candidate set by horizon rollout.

        For each candidate stiffness K (critically damped D = 2ζ√(K·m̄)), roll
        the error double integrator ë = −Λ⁻¹(K e + D ė + d̂) forward N steps and
        score Σ_k [w_e‖e_k‖² + w_F‖F_k‖²].  Returns the minimiser.  This is the
        predictive variable-impedance step: it looks ahead to trade tracking
        against contact force, but it does NOT cancel d̂ (no offset-free term),
        so the scheduled loop settles to e_∞ = −K*⁻¹ d̂ ≠ 0 under constant force.
        """
        dt = self.p.dt_mpc
        m_bar = 1.0 / max(float(np.mean(np.diag(Lam_inv))), 1e-6)  # ≈ mean Λ
        best_K, best_D, best_J = self._vic_K_last, 0.0, np.inf
        for K in self.p.vic_K_set:
            D = 2.0 * self.p.vic_zeta * np.sqrt(K * m_bar)
            e_k  = x_e[:3].copy()
            de_k = x_e[3:].copy()
            J = 0.0
            for _ in range(self.p.N):
                F_k  = K * e_k + D * de_k
                J   += self.p.vic_w_e * float(e_k @ e_k) + self.p.vic_w_F * float(F_k @ F_k)
                a_k  = -Lam_inv @ (F_k + d_hat)
                e_k  = e_k + dt * de_k
                de_k = de_k + dt * a_k
            if J < best_J:
                best_J, best_K, best_D = J, float(K), float(D)
        self._vic_K_last = best_K
        return best_K, best_D

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def _barrier_gradient(self, q: np.ndarray) -> np.ndarray:
        """
        Joint-space inverse-barrier gradient (Nm).
        Active for each joint within `barrier_margin` fraction of its range.
        Positive value → push joint toward higher q (away from lower limit).
        """
        span     = self.p.q_max - self.p.q_min
        d_lo     = q - self.p.q_min
        d_hi     = self.p.q_max - q
        d_min    = np.minimum(d_lo, d_hi)
        d_thresh = self.p.barrier_margin * span

        g = np.zeros(len(q))
        active = d_min < d_thresh
        if active.any():
            d_c     = np.maximum(d_min[active], 1e-4 * span[active])
            repel   = self.p.k_barrier * (1.0 / d_c - 1.0 / d_thresh[active])
            near_lo = d_lo[active] <= d_hi[active]
            g[active] = np.where(near_lo, repel, -repel)
        return g

    def null_torque(
        self,
        q:     np.ndarray,   # (7,) joint positions
        dq:    np.ndarray,   # (7,) joint velocities
        N_bar: np.ndarray,   # (7,7) dynamically-consistent null-space projector
    ) -> np.ndarray:
        """Null-space torque: centering spring + joint-limit barrier + damping."""
        g = self._barrier_gradient(q)
        return N_bar.T @ (
            -self.p.k_null * (q - self.p.q_null)
            + g
            - self.p.d_null * dq
        )

    def workspace_correction(
        self,
        q:   np.ndarray,   # (7,) joint positions
        J_v: np.ndarray,   # (3,7) translational Jacobian
    ) -> np.ndarray:
        """
        Cartesian reference offset that steers p_d away from joint-limit regions.

        Projects the barrier gradient into the task space:
            δp = k_ws · (J_v J_v^T)^{-1} J_v g
        capped at max_ws_corr metres.  Added to p_d before the QP so the
        receding-horizon optimiser targets a position that is reachable
        without violating joint limits.
        """
        g = self._barrier_gradient(q)
        if not np.any(g):
            return np.zeros(3)
        JJT   = J_v @ J_v.T + 1e-8 * np.eye(3)          # regularised 3×3
        delta = self.p.k_ws * np.linalg.solve(JJT, J_v @ g)
        nrm   = np.linalg.norm(delta)
        if nrm > self.p.max_ws_corr:
            delta *= self.p.max_ws_corr / nrm
        return delta

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _B_d(self, Lam_inv: np.ndarray) -> np.ndarray:
        """B_d(ρ) = [[0_{3×3}], [-Λ_pos^{-1} Δt]]  ∈ ℝ^{6×3}."""
        return np.vstack([np.zeros((3, 3)), -Lam_inv * self.p.dt_mpc])

    def _build_Gamma(self, Lam_inv: np.ndarray) -> np.ndarray:
        """
        Build lower-triangular input-to-state map Γ ∈ ℝ^{6N×3N}.

        Γ[i,j] = A_d^{i-j} B_d   for i ≥ j,  else 0.
        Uses precomputed A_d powers for efficiency.
        """
        N   = self.p.N
        Bd  = self._B_d(Lam_inv)
        Gam = np.zeros((6 * N, 3 * N))
        for i in range(N):
            for j in range(i + 1):
                Gam[6*i:6*(i+1), 3*j:3*(j+1)] = self._Ad_pow[i - j] @ Bd
        return Gam

    # ------------------------------------------------------------------
    # Kalman filter (disturbance augmentation)
    # ------------------------------------------------------------------

    def _kalman_step(
        self,
        e:       np.ndarray,   # (3,) measured position error
        de:      np.ndarray,   # (3,) measured velocity error
        Lam_inv: np.ndarray,   # (3,3)
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Kalman predict + update.  Returns (x_e_hat, d_hat).

        Augmented dynamics:
            [e; ė; d̂][k+1] = A_aug [e; ė; d̂][k] + B_aug F_mpc[k]
            y[k]            = C_aug [e; ė; d̂][k] + noise

        d̂ follows a random-walk model (integrating disturbance).
        """
        Bd = self._B_d(Lam_inv)   # (6×3)

        A_aug = np.zeros((9, 9))
        A_aug[:6, :6] = self.A_d         # error dynamics
        A_aug[:6, 6:] = Bd               # disturbance enters via B_d
        A_aug[6:, 6:] = np.eye(3)        # random walk for d̂

        B_aug = np.zeros((9, 3))
        B_aug[:6, :] = Bd

        C_aug = np.zeros((6, 9))
        C_aug[:6, :6] = np.eye(6)        # observe [e; ė]

        Q_kal = np.zeros((9, 9))
        Q_kal[6:, 6:] = self.p.Q_proc_d * np.eye(3)

        R_kal = np.diag(np.concatenate([
            self.p.R_obs_e  * np.ones(3),
            self.p.R_obs_de * np.ones(3),
        ]))

        if self._first_step:
            self.x_aug[:3] = e
            self.x_aug[3:6] = de
            self._first_step = False

        # Predict
        x_pred = A_aug @ self.x_aug + B_aug @ self._F_prev
        P_pred = A_aug @ self.P_aug @ A_aug.T + Q_kal

        # Update
        y   = np.concatenate([e, de])
        S   = C_aug @ P_pred @ C_aug.T + R_kal
        Kf  = P_pred @ C_aug.T @ inv(S)
        innov = y - C_aug @ x_pred
        self.x_aug = x_pred + Kf @ innov
        self.P_aug = (np.eye(9) - Kf @ C_aug) @ P_pred

        return self.x_aug[:6], self.x_aug[6:]

    # ------------------------------------------------------------------
    # QP solvers
    # ------------------------------------------------------------------

    def _torque_constraint_matrix(self, J_v: np.ndarray | None, n_u: int) -> np.ndarray | None:
        if J_v is None:
            return None
        A_tau = np.zeros((7, n_u))
        A_tau[:, :3] = J_v.T
        return A_tau

    def _torque_constraint_sparse(self, J_v: np.ndarray, n_u: int, sp):
        rows = np.tile(np.arange(7), 3)
        cols = np.repeat(np.arange(3), 7)
        data = J_v.T[:, :3].T.reshape(-1)
        return sp.csc_matrix((data, (rows, cols)), shape=(7, n_u))

    def _solve_qp(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_v: np.ndarray | None = None,
        tau_base: np.ndarray | None = None,
    ) -> np.ndarray:
        """Dispatch to the configured QP solver.  Returns flat solution u ∈ ℝ^{3N}."""
        if self.p.solver == 'osqp':
            return self._solve_qp_osqp(H, h, J_v, tau_base)
        return self._solve_qp_scipy(H, h, J_v, tau_base)

    def _solve_qp_scipy(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_v: np.ndarray | None = None,
        tau_base: np.ndarray | None = None,
    ) -> np.ndarray:
        n_u  = 3 * self.p.N
        Fmax = self.p.F_max
        constraints = []
        A_tau = self._torque_constraint_matrix(J_v, n_u)
        if A_tau is not None and tau_base is not None:
            margin_lo = -self.p.tau_max - tau_base
            margin_hi =  self.p.tau_max - tau_base
            constraints.append(LinearConstraint(A_tau, margin_lo, margin_hi))
        res  = minimize(
            fun=lambda u: 0.5 * float(u @ H @ u) + float(h @ u),
            x0=np.zeros(n_u),
            jac=lambda u: H @ u + h,
            method='SLSQP',
            bounds=[(-Fmax, Fmax)] * n_u,
            constraints=constraints,
            options={'ftol': 1e-9, 'maxiter': 200},
        )
        return res.x

    def _solve_qp_osqp(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_v: np.ndarray | None = None,
        tau_base: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Solve the force- and applied-torque-constrained QP with OSQP.

        Problem form (matches OSQP C++ API 1:1):
            min   ½ uᵀ P u + qᵀ u
            s.t.  l ≤ A u ≤ u_ub
        where P = H (symmetric, upper-triangular stored), q = h,
        the first rows bound every force component, and the final rows bound
        the first applied joint torque:
            -tau_max <= tau_base + J_v.T @ F_mpc(0) <= tau_max.

        The OSQP instance is created once and warm-started on every call.
        """
        import scipy.sparse as sp

        n_u  = 3 * self.p.N
        P_sp = sp.triu(H, format='csc')   # upper-triangular CSC — OSQP convention
        A_force = sp.eye(n_u, format='csc')
        lb_force = -self.p.F_max * np.ones(n_u)
        ub_force =  self.p.F_max * np.ones(n_u)
        A_tau = self._torque_constraint_matrix(J_v, n_u)
        if A_tau is not None and tau_base is not None:
            A_sp = sp.vstack([A_force, self._torque_constraint_sparse(J_v, n_u, sp)], format='csc')
            lb = np.concatenate([lb_force, -self.p.tau_max - tau_base])
            ub = np.concatenate([ub_force,  self.p.tau_max - tau_base])
        else:
            A_sp = A_force
            lb = lb_force
            ub = ub_force

        if self._osqp_prob is None:
            import osqp
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P_sp, h, A_sp, lb, ub,
                warm_starting  = True,
                verbose        = False,
                eps_abs        = 1e-6,
                eps_rel        = 1e-6,
                max_iter       = 4000,
                polish         = False,   # off for real-time; C++ deployment matches
                adaptive_rho   = True,
            )
        else:
            # Reuse the same sparsity pattern — only values change
            self._osqp_prob.update(Px=P_sp.data, q=h, Ax=A_sp.data, l=lb, u=ub)

        res = self._osqp_prob.solve()
        # status_val 1 = solved, 2 = solved (inaccurate but acceptable)
        if res.info.status_val not in (1, 2):
            return np.zeros(n_u)
        return res.x

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self):
        """Reset internal state.  Call between simulation runs."""
        self.x_aug[:]    = 0.0
        self.P_aug[:]    = np.eye(9)
        self._first_step = True
        self._F_prev[:]  = 0.0
        self.F_seq[:]    = 0.0
        # Destroy the OSQP instance so the next control() call re-creates it
        # with fresh rho scaling.  warm_start() alone resets primal/dual
        # variables but leaves adaptive-rho at whatever value the previous
        # episode drove it to, causing mis-scaled iterations at episode start.
        self._osqp_prob = None

    # ------------------------------------------------------------------
    # Main control call
    # ------------------------------------------------------------------

    def control(
        self,
        ee_pos:  np.ndarray,   # (3,)  EE position, world frame
        ee_vel:  np.ndarray,   # (6,)  EE twist [v; ω], world frame
        ee_rot:  np.ndarray,   # (3,3) EE rotation matrix
        p_d:     np.ndarray,   # (3,)  desired position
        dp_d:    np.ndarray,   # (3,)  desired linear velocity
        ddp_d:   np.ndarray,   # (3,)  desired linear acceleration
        R_d:     np.ndarray,   # (3,3) desired orientation
        dyn,                   # FrankaDynamics
        q:       np.ndarray,   # (7,)  joint positions
        dq:      np.ndarray,   # (7,)  joint velocities
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute joint torques.

        Returns
        -------
        tau   : (7,) joint torques
        F_mpc : (3,) MPC corrective force (for logging)
        """
        from fr3_impedance import build_operational_space_model
        from so3_utils import rotation_error_matrix

        J_v   = dyn.J[:3, :]          # translational Jacobian  (3×7)
        J_w   = dyn.J[3:, :]          # rotational Jacobian     (3×7)
        M_inv = inv(dyn.M)

        # Operational-space inertia (3×3) for translational DOF.
        # Λ⁻¹ is exactly (J_v M⁻¹ J_vᵀ + reg); invert once for Λ rather than
        # re-inverting Λ to recover Λ⁻¹ (avoids a redundant 3×3 inverse and the
        # round-trip error it introduces).
        Lam_inv = J_v @ M_inv @ J_v.T + 1e-6 * np.eye(3)
        Lam_pos = inv(Lam_inv)

        # Null-space projector from full 6D operational-space model
        os    = build_operational_space_model(dyn, ee_vel)
        N_bar = os.N_bar

        # ── Layer 1: feedforward ─────────────────────────────────────────
        # Cancels Coriolis + gravity (via qfrc_bias) and task inertia term.
        # After this, the residual plant is a constant-A_d double integrator.
        tau_ff = dyn.Cq_dot + J_v.T @ (Lam_pos @ ddp_d)

        # ── Error state (paper convention: e = p_d − p so ë = −Λ⁻¹ F_mpc) ──
        # Use the nominal p_d for the Kalman observation so d_hat tracks only
        # real external disturbances, not the workspace-correction offset.
        e  = p_d - ee_pos
        de = dp_d - ee_vel[:3]

        # ── Kalman or direct error state ─────────────────────────────────
        if self.use_kalman:
            x_e, d_hat = self._kalman_step(e, de, Lam_inv)
        else:
            x_e   = np.concatenate([e, de])
            d_hat = np.zeros(3)

        # ── Workspace projection: offset p_d away from limit-inducing regions ──
        # Applied after the Kalman update so the correction does not pollute
        # d_hat with a phantom disturbance proportional to joint proximity.
        ws_corr = self.workspace_correction(q, J_v)
        x_e = x_e.copy()
        x_e[:3] = x_e[:3] + ws_corr   # shift position error; velocity error unchanged

        # ── Layer 2: receding-horizon QP ─────────────────────────────────
        Bd    = self._B_d(Lam_inv)

        if self.p.variable_impedance:
            # ---- MPVIC baseline: predictive variable impedance -----------
            # Select the apparent stiffness by horizon rollout (predictive),
            # then track F = K* e + D* ė.  The disturbance estimate d̂ shapes
            # the *choice* of K* but is NOT cancelled (no −d̂ term), so the
            # loop keeps a nonzero steady-state error under sustained force —
            # the behaviour the offset-free controller (Theorem 2) removes.
            K_star, D_star = self._select_variable_stiffness(x_e, d_hat, Lam_inv)
            G    = np.hstack([K_star * np.eye(3), D_star * np.eye(3)])  # (3×6)
            A_cl = self.A_d + Bd @ G
            U_vic = np.zeros(3 * self.p.N)
            xi = x_e.copy()
            for i in range(self.p.N):
                U_vic[3*i:3*i+3] = G @ xi          # no −d̂: not offset-free
                xi = A_cl @ xi
            H = self.R_bar
            h = -self.R_bar @ U_vic
        elif self.p.impedance_track:
            # ---- Impedance-equivalence mode (Theorem 1) ------------------
            # Track the prescribed impedance law F = K_d e + D_d ė − d̂.
            # Cost penalises deviation of the force sequence from the
            # impedance reference, so the unconstrained optimum is EXACTLY
            # the impedance gain; the box constraint clips it only on
            # saturation.  H = R̄ is constant (reuses the constant-matrix
            # structure of the plant).
            K_d = self.p.k_imp * np.eye(3)
            D_d = 2.0 * self.p.zeta_imp * np.sqrt(self.p.k_imp) * np.eye(3)
            G   = np.hstack([K_d, D_d])               # (3×6) impedance gain
            A_cl = self.A_d + Bd @ G                   # offset-free closed loop
            U_imp = np.zeros(3 * self.p.N)
            xi = x_e.copy()
            for i in range(self.p.N):
                U_imp[3*i:3*i+3] = G @ xi - d_hat     # −d̂: offset-free feedforward
                xi = A_cl @ xi
            H = self.R_bar
            h = -self.R_bar @ U_imp
        else:
            Gamma = self._build_Gamma(Lam_inv)

            # Disturbance propagation over N steps: D̄ ∈ ℝ^{6N×3}
            # D̄[k] = (I + A_d + ... + A_d^k) Bd  — geometric sum, not A_d^k Bd.
            # Under constant d̂, the disturbance accumulates at every step, so each
            # horizon row needs the full prefix sum, not just the k-th power term.
            D_bar = np.zeros((6 * self.p.N, 3))
            _cumsum = np.zeros((6, 3))
            for k in range(self.p.N):
                _cumsum = _cumsum + self._Ad_pow[k] @ Bd
                D_bar[6*k:6*(k+1)] = _cumsum

            # Free-response incorporating current disturbance estimate
            x_free = self.Phi @ x_e + D_bar @ d_hat

            # QP matrices
            H = Gamma.T @ self.Q_bar @ Gamma + self.R_bar
            H = 0.5 * (H + H.T)      # ensure symmetric (numerical drift)
            # Offset-free input centering:
            # D_bar = Gamma @ (1_N ⊗ I_3), so constant d_hat can be absorbed by
            # V = U + (1_N ⊗ d_hat).  Penalising V, not U, makes the steady
            # balancing input F_mpc = -d_hat cost-free and removes R-dependent
            # steady-state bias.
            d_seq = np.tile(d_hat, self.p.N)
            h = Gamma.T @ self.Q_bar @ x_free + self.R_bar @ d_seq

        # ── Orientation stabilisation (simple impedance, runs every step) ─
        e_R  = rotation_error_matrix(R_d, ee_rot)    # (3,) axis-angle error
        de_R = ee_vel[3:]                            # angular velocity error
        tau_orient = J_w.T @ (-self.p.K_rot * e_R - self.p.D_rot * de_R)

        # ── Null-space joint-limit avoidance (barrier + centering + damping) ───
        tau_null = self.null_torque(q, dq, N_bar)

        tau_base = tau_ff + tau_orient + tau_null
        u_seq = self._solve_qp(H, h, J_v=J_v, tau_base=tau_base)  # (3N,)
        F_mpc = u_seq[:3]                                        # first step
        self._F_prev = F_mpc.copy()
        self.F_seq   = u_seq.reshape(self.p.N, 3)

        tau_mpc = J_v.T @ F_mpc
        tau = tau_base + tau_mpc
        return tau, F_mpc
