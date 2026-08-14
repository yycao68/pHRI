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
    B_d = [[-½ Δt² Λ_pos^{-1}],   (parameter-varying via Λ_pos(q))
           [-Δt   Λ_pos^{-1}]]

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
    # only to honour the F_max box constraint.  D_d = 2ζ√(K_d), which is the
    # familiar unit-effective-mass tuning; with configuration-dependent Λ it
    # remains positive damping but is not exactly modal-critical in general.
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

    # ── Observer-impedance baseline (non-predictive ablation) ─────────────
    # F_mpc = K_e e + K_edot edot - d_hat using the SAME Kalman estimator as
    # the full MPC, but a FIXED gain [K_e, K_edot] evaluated once (at
    # reset, from the initial Lam_inv via unconstrained_first_move_gain)
    # rather than re-solved by a receding-horizon QP every tick. Isolates
    # the estimator's contribution to offset rejection from the horizon
    # optimization's -- if this matches C5 closely under nominal conditions,
    # the 65.8x steady-state improvement is attributable to the estimator,
    # not to MPC prediction as such.
    observer_only: bool = False
    vic_zeta:  float = 1.0     # damping ratio for the scheduled stiffness
    vic_w_e:   float = 2e4     # tracking-error weight in the K-selection cost
    vic_w_F:   float = 3e-3    # task-force weight  (interior optimum ⇒ finite K*)

    # ── Impedance-backbone mode (stability fallback under saturation) ────────
    # Motivation: when the QP output IS the entire corrective force (every
    # mode above), a saturated/zeroed F_mpc (tight F_max, a solver fault, or
    # an under-realizable predicted torque) leaves ONLY τ_ff applied — pure
    # feedforward, zero corrective stiffness, not a stable regulator. Setting
    # backbone_track=True instead commands a fixed, positively damped
    # impedance law F_bb = K_bb e + D_bb ė outside the QP on every MPC update,
    # independently of what the QP returns, and the QP only
    # searches for a BOUNDED *additional* corrective force F_mpc on top of
    # it, predicted through the backbone dynamics LINEARIZED ALONG A
    # NOMINAL TRAJECTORY — A_cl = A_d + B_d(ρ_k) G_bb frozen at the current
    # configuration by default, or A_cl,i = A_d + B_d,i G_bb scheduled per
    # horizon step when horizon_torque_schedule is also set — rather than
    # the open-loop A_d. So even if F_mpc saturates to exactly zero, the
    # commanded controller still contains the
    # stable restoring backbone (not offset-free —
    # offset-free rejection is what the bounded F_mpc term contributes on
    # top, same −d̂ centering trick as impedance_track). This does not bypass
    # physical actuator saturation: if tau_base itself exceeds tau_max, the
    # plant can still clip the backbone torque.
    backbone_track: bool = False
    k_backbone:     float = 300.0   # backbone translational stiffness (N/m)
    zeta_backbone:  float = 1.0     # unit-effective-mass damping tuning

    # ── Horizon-wide torque realizability (frozen-Jacobian approximation) ──
    # The applied-torque constraint (9b) is, by construction, feasible only
    # for the FIRST predicted step; steps 1..N-1 of the horizon can still
    # imply a torque the actuators cannot realize, which silently feeds a
    # physically-unrealizable predicted trajectory back into the first-step
    # optimum. Setting horizon_torque_constraint=True replicates the same
    # affine row (frozen at the current J_v(q_k), τ_base(q_k)) for every
    # step i=0..N-1 instead of only i=0 — a local approximation (it does not
    # track how J_v or τ_base evolve along the horizon) but one that stops
    # the QP from planning around inputs that are obviously unrealizable
    # from the current configuration.
    horizon_torque_constraint: bool = False

    # ── Reference-scheduled horizon torque realizability ────────────────────
    # Alternative to horizon_torque_constraint's frozen-at-q_k row: instead of
    # reusing today's J_v(q_k)/τ_ff(q_k,q̇_k) for every horizon step, schedule
    # them along the nominal reference trajectory q_d(t)/q̇_d(t) (precomputed
    # offline by phri.precompute_joint_reference and passed in as
    # joint_traj_fn — see control()). τ_ff,i is recomputed at (q̄_i, q̇̄_i)
    # with the desired Cartesian acceleration ẍ_{d,i} taken from the
    # caller's traj_fn at t_k+i·dt_mpc (exact — no extrapolation needed
    # there), using the SAME Layer-1 formula as step 0. Requires
    # `joint_traj_fn`, `dyn_query_fn`, and `traj_fn` to be passed into
    # control(); if `joint_traj_fn` is unavailable, falls back to
    # horizon_torque_constraint's frozen-at-q_k behavior (reps=N, but not
    # scheduled) rather than any extrapolation scheme — there was previously
    # a constant-velocity "coast" fallback here; removed after §7 of
    # stable_backbone_mpc.md found it added implementation complexity
    # (shadow dynamics queries, a velocity cap to tune) without any
    # measurable benefit over just freezing at q_k, which the paper can
    # already justify directly from the 20 ms horizon being too short for
    # the robot-dependent quantities to change appreciably. Orientation/
    # null-space torque and (if backbone_track) F_backbone are NOT
    # rescheduled — they stay frozen at their current-step value, same
    # simplification the frozen-Jacobian version already makes for the whole
    # τ_base.
    horizon_torque_schedule: bool = False

    # ── Applied-torque constraint (9b) on/off ────────────────────────────
    # False removes the first-move torque row entirely -- the QP shapes
    # F_mpc from the force box alone, with no knowledge of tau_max. The
    # plant (FR3MuJoCoEnv.apply_torque) still clips physically regardless
    # of this flag, so the difference is NOT whether the robot ever
    # exceeds tau_max (it never does), but whether the QP's own optimum
    # already accounts for the limit or has to be corrected downstream by
    # clipping -- i.e. whether the commanded and actually-applied torque
    # match. Used by the active-torque-constraint experiment to isolate
    # what embedding (9b) buys under a tightened budget (phri.TAU_MAX_SCALE)
    # where it actually binds.
    apply_torque_constraint: bool = True

    # ── Soft (slack-relaxed) torque row, OSQP path only ──────────────────
    # None -> the hard constraint above (infeasible QP -> zero correction,
    # the default/reported behavior everywhere else in the paper). A float
    # -> the torque row -tau_max-tb <= A_tau u <= tau_max-tb is relaxed to
    # -tau_max-tb-s <= A_tau u <= tau_max-tb+s with slack s>=0 penalized by
    # 0.5*soft_torque_rho*||s||^2 in the cost, added to the SAME QP (never
    # a separate solve) so the box/torque structure and warm-start reuse
    # are unaffected when this is None. Makes the QP ALWAYS feasible
    # (assuming the force box alone is; it is, being a simple decoupled
    # box) instead of falling back to zero correction the moment the exact
    # constraint can't be met -- the commanded torque can then exceed
    # tau_max by an amount the physical clip in FR3MuJoCoEnv.apply_torque
    # absorbs, same as the Unconstrained/NoTauRow variant, but only by as
    # much as the QP's own optimum requires rather than dropping to zero
    # correction outright. Opt-in via the "Soft" controller-name substring;
    # every existing controller/table keeps soft_torque_rho=None untouched.
    soft_torque_rho: float | None = None

    # Error-decay correction blended into reference scheduling: q̄_i =
    # q_d,i + rho^i·(q_k − q_d,k) (and analogously for q̇), rho in [0,1].
    # rho=0 (default) is PURE reference scheduling (q̄_i = q_d,i exactly) —
    # this is what a naive reading of "schedule along q_d(t)" gives, but it
    # is empirically WORSE than frozen-at-q_k in the regime where the torque
    # constraint actually binds without being globally infeasible
    # (stable_backbone_mpc.md §7): q_d(t) is the UNDISTURBED reference, and
    # during contact the real q deliberately deflects away from it, so
    # scheduling J_v,i/τ_ff,i purely on q_d,i can be a WORSE local model
    # than the actual current state. rho>0 blends in a decaying correction
    # toward the actual (q_k, q̇_k) so step i=0 stays exact regardless of
    # rho and later steps relax toward the reference — see the Remark in
    # stable_backbone_mpc.md §7 for when this recovers (but does not beat)
    # frozen-at-q_k empirically. NOT recommended for the manuscript — kept
    # as exploratory code, see §7's conclusion.
    schedule_rho: float = 0.0

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
        self._observer_gain = None  # cached [K_e,K_edot], observer_only mode

        # OSQP instance — created lazily on first QP call, reused thereafter
        self._osqp_prob: object = None
        self.last_qp_success = True
        self.last_qp_status = "not solved"

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
        """Exact zero-order-hold (ZOH) input matrix for the nilpotent double
        integrator:

            B_d(ρ) = ∫₀^{Δt} e^{A_c s} B_c ds
                   = [[-½ Δt² Λ_pos^{-1}],
                      [-Δt   Λ_pos^{-1}]]  ∈ ℝ^{6×3}.

        Because A_c is nilpotent (A_c² = 0), e^{A_c s} = I + A_c s is exact, so
        A_d = e^{A_c Δt} is unchanged; only the input block carries the exact
        ½Δt² top term (previously dropped by the Forward-Euler approximation).
        Used consistently by Γ, the Kalman augmented model (9), and the
        closed-loop / disturbance-propagation analysis, so no ZOH/Euler mixing
        bias arises.
        """
        dt = self.p.dt_mpc
        return np.vstack([-0.5 * dt * dt * Lam_inv, -Lam_inv * dt])

    def _build_closed_loop_horizon(
        self, Bd: np.ndarray, A_cl: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Phi/Gamma/D_bar for a TIME-INVARIANT closed-loop transition
        A_cl = A_d + Bd @ G (e.g. the impedance-backbone loop), mirroring
        Phi/_build_Gamma/D_bar but around A_cl instead of the open-loop
        A_d. Rebuilt every call since A_cl is configuration-dependent
        through Bd (frozen at the CURRENT configuration for the whole
        horizon — used when horizon_torque_schedule is off, or scheduling
        inputs weren't supplied; see _build_scheduled_closed_loop_horizon
        for the horizon-varying counterpart)."""
        N = self.p.N
        Acl_pow = [np.eye(6)]
        for _ in range(N - 1):
            Acl_pow.append(Acl_pow[-1] @ A_cl)
        Phi_cl   = np.vstack([Acl_pow[k] @ A_cl for k in range(N)])
        Gamma_cl = np.zeros((6 * N, 3 * N))
        D_bar_cl = np.zeros((6 * N, 3))
        cumsum   = np.zeros((6, 3))
        for i in range(N):
            for j in range(i + 1):
                Gamma_cl[6*i:6*(i+1), 3*j:3*(j+1)] = Acl_pow[i - j] @ Bd
            cumsum = cumsum + Acl_pow[i] @ Bd
            D_bar_cl[6*i:6*(i+1)] = cumsum
        return Phi_cl, Gamma_cl, D_bar_cl

    def _build_scheduled_closed_loop_horizon(
        self, Bd_list: list[np.ndarray], G_bb: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Phi/Gamma/D_bar for a TIME-VARYING (scheduled) closed-loop
        transition A_cl,i = A_d + Bd_list[i] @ G_bb, i.e. the backbone
        dynamics LINEARIZED ALONG the scheduled nominal trajectory rather
        than frozen at one configuration — the LTV counterpart of
        _build_closed_loop_horizon. Since A_cl,i genuinely varies with i,
        the state-transition/input-map blocks are chain products of the
        per-step A_cl,i rather than powers of a single matrix:

            x_{i+1|k} = (A_cl,i···A_cl,0) x_e
                       + Σ_j (A_cl,i···A_cl,j+1) Bd_j (F_mpc,j|k + d̂)

        O(N^2) 6x6/6x3 matmuls — negligible at N~10. Reduces exactly to
        _build_closed_loop_horizon's output when every Bd_list[i] is the
        same matrix (frozen special case)."""
        N = self.p.N
        Acl_list = [self.A_d + Bd_list[i] @ G_bb for i in range(N)]
        Phi_cl   = np.zeros((6 * N, 6))
        Gamma_cl = np.zeros((6 * N, 3 * N))
        D_bar_cl = np.zeros((6 * N, 3))
        for i in range(N):
            chain = np.eye(6)
            for k in range(i, -1, -1):
                chain = chain @ Acl_list[k]
            Phi_cl[6*i:6*(i+1)] = chain
            cumsum = np.zeros((6, 3))
            for j in range(i + 1):
                coeff = Bd_list[j]
                for k in range(j + 1, i + 1):
                    coeff = Acl_list[k] @ coeff
                Gamma_cl[6*i:6*(i+1), 3*j:3*(j+1)] = coeff
                cumsum = cumsum + coeff
            D_bar_cl[6*i:6*(i+1)] = cumsum
        return Phi_cl, Gamma_cl, D_bar_cl

    def _schedule_horizon(
        self,
        q:        np.ndarray,   # (7,) current joint positions
        dq:       np.ndarray,   # (7,) current joint velocities
        t:        float,        # current simulation time
        J_v0:     np.ndarray,   # (3,7) already-computed step-0 Jacobian
        tau_ff0:  np.ndarray,   # (7,)  already-computed step-0 feedforward
        Lam_inv0: np.ndarray,   # (3,3) already-computed step-0 Λ⁻¹
        traj_fn,                 # t -> (p_d, dp_d, ddp_d)
        dyn_query_fn,             # (q_i, dq_i) -> (J_v_i, Lam_inv_i, Lam_pos_i, Cq_dot_i)
        joint_traj_fn,            # t -> (q_d, q̇_d, q̈_d) — required; see control()
    ) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
        """Reference-scheduled horizon torque realizability AND (when
        backbone_track is also on) the LTV backbone prediction — see
        ImpedanceMPCParams.horizon_torque_schedule. Step 0 reuses the exact
        (J_v0, tau_ff0, Lam_inv0) already computed for the real applied
        torque — no redundant dynamics query and guaranteed consistency
        with the first-step-only constraint (9b) this generalizes. Steps
        1..N-1 evaluate dyn_query_fn along
        q̄_i = q_d,i + schedule_rho^i·(q_k − q_d,k) (and analogously q̇̄_i),
        and τ_ff,i is recomputed with the Layer-1 formula at (q̄_i, q̇̄_i)
        against the exact Cartesian acceleration reference ẍ_{d,i} taken
        from traj_fn. schedule_rho=0 (the default) gives PURE reference
        scheduling, q̄_i = q_d,i exactly — the arm's planned motion,
        ignoring the actual current state entirely past i=0. This is NOT
        always better than tracking the actual state (see
        ImpedanceMPCParams.schedule_rho and stable_backbone_mpc.md §7):
        during contact the real q deliberately deflects away from the
        undisturbed q_d(t), so a nonzero schedule_rho — blending in a
        decaying correction toward (q_k, q̇_k) that vanishes as i grows —
        can matter in practice, not just as a theoretical nicety.

        control() only calls this when `joint_traj_fn` is available;
        otherwise it uses horizon_torque_constraint's frozen-at-q_k rows
        instead (see the ImpedanceMPCParams.horizon_torque_schedule
        docstring — an earlier constant-velocity "coast" fallback lived
        here and was removed, see stable_backbone_mpc.md §7). Does not
        re-integrate through the QP's own solution, which would make the
        constraint depend on the decision variable and break QP
        linearity."""
        N   = self.p.N
        dt  = self.p.dt_mpc
        rho = self.p.schedule_rho
        if rho > 0.0:
            q_d_k, dq_d_k, _ = joint_traj_fn(t)
            delta_q, delta_dq = q - q_d_k, dq - dq_d_k
        J_list        = [J_v0]
        tauff_list    = [tau_ff0]
        lam_inv_list  = [Lam_inv0]
        for i in range(1, N):
            q_i, dq_i, _ = joint_traj_fn(t + i * dt)
            if rho > 0.0:
                w = rho ** i
                q_i, dq_i = q_i + w * delta_q, dq_i + w * delta_dq
            J_v_i, Lam_inv_i, Lam_pos_i, Cq_dot_i = dyn_query_fn(q_i, dq_i)
            _, _, ddp_d_i = traj_fn(t + i * dt)
            tau_ff_i = Cq_dot_i + J_v_i.T @ (Lam_pos_i @ ddp_d_i)
            J_list.append(J_v_i)
            tauff_list.append(tau_ff_i)
            lam_inv_list.append(Lam_inv_i)
        return J_list, tauff_list, lam_inv_list

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

    def unconstrained_first_move_gain(self, Lam_inv: np.ndarray) -> np.ndarray:
        """Realized first-move gain K = [K_e, K_edot] (3x6) of the
        unconstrained receding-horizon law, eq:unconstrained:
        F_mpc,0 = S_0 U* = S_0 (-H^-1 Gamma^T Qbar Phi) x_e = K @ x_e.
        Used by the observer-impedance baseline (a fixed, non-predictive
        law using this same gain, no horizon re-optimization) to isolate
        the estimator's contribution from the receding-horizon prediction's.
        Configuration-dependent through Lam_inv, same as the QP itself; the
        baseline evaluates this once at a reference configuration and holds
        it fixed, which is what makes it non-predictive."""
        Gamma = self._build_Gamma(Lam_inv)
        H = Gamma.T @ self.Q_bar @ Gamma + self.R_bar
        K_full = -np.linalg.solve(H, Gamma.T @ self.Q_bar @ self.Phi)
        return K_full[:3]

    def _default_qp_objective(
        self,
        x_e: np.ndarray,
        d_hat: np.ndarray,
        Lam_inv: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Assemble the default centered-input LQ objective.

        The physical decision variable remains ``U``.  The effort term is
        ``0.5 * ||U + d_seq||^2_Rbar``, so its decision-dependent linear
        contribution is ``Rbar @ d_seq``.  Keeping this assembly in one
        testable routine prevents a silent return to an absolute-input
        penalty, which would introduce an R-dependent steady-state bias.
        """
        Gamma = self._build_Gamma(Lam_inv)
        Bd = self._B_d(Lam_inv)
        D_bar = np.zeros((6 * self.p.N, 3))
        cumsum = np.zeros((6, 3))
        for k in range(self.p.N):
            cumsum = cumsum + self._Ad_pow[k] @ Bd
            D_bar[6*k:6*(k+1)] = cumsum

        x_free = self.Phi @ x_e + D_bar @ d_hat
        H = Gamma.T @ self.Q_bar @ Gamma + self.R_bar
        H = 0.5 * (H + H.T)
        d_seq = np.tile(d_hat, self.p.N)
        h = Gamma.T @ self.Q_bar @ x_free + self.R_bar @ d_seq
        return H, h, Gamma, D_bar, x_free

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

    def _torque_constraint_reps(self) -> int:
        """1 = first-applied-step only (9b); N = horizon-wide realizability,
        either frozen-Jacobian (horizon_torque_constraint) or
        reference-scheduled (horizon_torque_schedule); see ImpedanceMPCParams."""
        return self.p.N if (self.p.horizon_torque_constraint
                            or self.p.horizon_torque_schedule) else 1

    def _torque_constraint_matrix(self, J_list: list[np.ndarray] | None, n_u: int) -> np.ndarray | None:
        if J_list is None:
            return None
        reps  = len(J_list)
        A_tau = np.zeros((7 * reps, n_u))
        for i, J_v in enumerate(J_list):
            A_tau[7*i:7*(i+1), 3*i:3*(i+1)] = J_v.T
        return A_tau

    def _torque_constraint_sparse(self, J_list: list[np.ndarray], n_u: int, sp):
        reps = len(J_list)
        local_rows = np.tile(np.arange(7), 3)
        local_cols = np.repeat(np.arange(3), 7)
        rows = np.concatenate([7*i + local_rows for i in range(reps)])
        cols = np.concatenate([3*i + local_cols for i in range(reps)])
        data = np.concatenate([J_v.reshape(-1) for J_v in J_list])  # matches A_tau[:, i]=J_v.T
        return sp.csc_matrix((data, (rows, cols)), shape=(7 * reps, n_u))

    def _solve_qp(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_list: list[np.ndarray] | None = None,
        tau_base_list: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        """Dispatch to the configured QP solver.  Returns flat solution u ∈ ℝ^{3N}."""
        if self.p.solver == 'osqp':
            return self._solve_qp_osqp(H, h, J_list, tau_base_list)
        return self._solve_qp_scipy(H, h, J_list, tau_base_list)

    def _solve_qp_scipy(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_list: list[np.ndarray] | None = None,
        tau_base_list: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        n_u  = 3 * self.p.N
        Fmax = self.p.F_max
        constraints = []
        A_tau = self._torque_constraint_matrix(J_list, n_u)
        if A_tau is not None and tau_base_list is not None:
            margin_lo = np.concatenate([-self.p.tau_max - tb for tb in tau_base_list])
            margin_hi = np.concatenate([ self.p.tau_max - tb for tb in tau_base_list])
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
        # res.success reflects SLSQP's convergence-TOLERANCE criterion, not
        # feasibility: on a large/tight horizon problem it routinely reports
        # success=False with message="Iteration limit reached" even when
        # res.x is a perfectly good, feasible point (verified: finite, zero
        # constraint violation) -- gating the zero-correction fallback on
        # res.success alone silently zeroed out ~36% of steps in a real
        # running episode. Check feasibility directly instead: finite
        # values, the box bounds, and (if present) the torque-realizability
        # rows, independent of whether SLSQP fully converged to ftol.
        finite = bool(np.all(np.isfinite(res.x)))
        if finite:
            viol = float(np.maximum(np.abs(res.x) - Fmax, 0).max())
            if A_tau is not None and tau_base_list is not None:
                Au = A_tau @ res.x
                viol = max(viol,
                           float(np.maximum(margin_lo - Au, 0).max()),
                           float(np.maximum(Au - margin_hi, 0).max()))
        else:
            viol = np.inf
        self.last_qp_success = bool(finite and viol < 1e-6)
        self.last_qp_status = str(res.message)
        if not self.last_qp_success:
            return np.zeros(n_u)
        return res.x

    def _solve_qp_osqp(
        self,
        H: np.ndarray,
        h: np.ndarray,
        J_list: list[np.ndarray] | None = None,
        tau_base_list: list[np.ndarray] | None = None,
    ) -> np.ndarray:
        """
        Solve the force- and applied-torque-constrained QP with OSQP.

        Problem form (matches OSQP C++ API 1:1):
            min   ½ uᵀ P u + qᵀ u
            s.t.  l ≤ A u ≤ u_ub
        where P = H (symmetric, upper-triangular stored), q = h,
        the first rows bound every force component, and the final rows bound
        the applied joint torque:
            -tau_max <= tau_base_list[i] + J_list[i].T @ F_mpc(i) <= tau_max,
        for i=0 only (default, Eq. 9b), for every i=0..N-1 with J_list[i]/
        tau_base_list[i] frozen at their i=0 values (horizon_torque_constraint=True,
        frozen-Jacobian horizon-wide check — also the fallback for
        horizon_torque_schedule=True when no joint_traj_fn was supplied),
        or for every i=0..N-1 with J_list[i]/tau_base_list[i] reference-
        scheduled along q_d(t) (horizon_torque_schedule=True with
        joint_traj_fn supplied).

        The OSQP instance is created once and warm-started on every call.
        """
        import scipy.sparse as sp

        n_u  = 3 * self.p.N
        A_force = sp.eye(n_u, format='csc')
        lb_force = -self.p.F_max * np.ones(n_u)
        ub_force =  self.p.F_max * np.ones(n_u)
        A_tau = self._torque_constraint_matrix(J_list, n_u)
        soft = self.p.soft_torque_rho is not None and A_tau is not None and tau_base_list is not None

        if soft:
            # Extended decision variable z=[u; s], s>=0, slack-relaxed torque
            # row: lb_tau-s <= A_tau u <= ub_tau+s, penalized 0.5*rho*||s||^2.
            n_s = A_tau.shape[0]
            P_sp = sp.block_diag(
                [sp.triu(H, format='csc'), self.p.soft_torque_rho * sp.eye(n_s)],
                format='csc')
            q_ext = np.concatenate([h, np.zeros(n_s)])
            A_tau_sp = self._torque_constraint_sparse(J_list, n_u, sp)
            lb_tau = np.concatenate([-self.p.tau_max - tb for tb in tau_base_list])
            ub_tau = np.concatenate([ self.p.tau_max - tb for tb in tau_base_list])
            row_force = sp.hstack([A_force, sp.csc_matrix((n_u, n_s))], format='csc')
            row_slack_pos = sp.hstack([sp.csc_matrix((n_s, n_u)), sp.eye(n_s)], format='csc')
            row_tau_upper = sp.hstack([A_tau_sp, -sp.eye(n_s)], format='csc')  # A u - s <= ub_tau
            row_tau_lower = sp.hstack([A_tau_sp,  sp.eye(n_s)], format='csc')  # A u + s >= lb_tau
            A_sp = sp.vstack([row_force, row_slack_pos, row_tau_upper, row_tau_lower], format='csc')
            lb = np.concatenate([lb_force, np.zeros(n_s),
                                  -np.inf * np.ones(n_s), lb_tau])
            ub = np.concatenate([ub_force, np.inf * np.ones(n_s),
                                  ub_tau, np.inf * np.ones(n_s)])
        else:
            P_sp = sp.triu(H, format='csc')   # upper-triangular CSC — OSQP convention
            if A_tau is not None and tau_base_list is not None:
                A_sp = sp.vstack([A_force, self._torque_constraint_sparse(J_list, n_u, sp)], format='csc')
                lb = np.concatenate([lb_force] + [-self.p.tau_max - tb for tb in tau_base_list])
                ub = np.concatenate([ub_force] + [ self.p.tau_max - tb for tb in tau_base_list])
            else:
                A_sp = A_force
                lb = lb_force
                ub = ub_force
            q_ext = h

        if self._osqp_prob is None:
            import osqp
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P_sp, q_ext, A_sp, lb, ub,
                warm_starting  = True,
                verbose        = False,
                eps_abs        = 1e-6,
                eps_rel        = 1e-6,
                max_iter       = 4000,
                polishing      = False,   # off for real-time; C++ deployment matches
                adaptive_rho   = True,
            )
        else:
            # Reuse the same sparsity pattern — only values change
            self._osqp_prob.update(Px=P_sp.data, q=q_ext, Ax=A_sp.data, l=lb, u=ub)

        res = self._osqp_prob.solve(raise_error=False)
        # status_val 1 = solved, 2 = solved (inaccurate but acceptable)
        self.last_qp_success = bool(
            res.info.status_val in (1, 2)
            and res.x is not None
            and np.all(np.isfinite(res.x))
        )
        self.last_qp_status = str(res.info.status)
        if not self.last_qp_success:
            return np.zeros(n_u)
        return res.x[:n_u] if soft else res.x

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
        self.last_qp_success = True
        self.last_qp_status = "not solved"
        # Destroy the OSQP instance so the next control() call re-creates it
        # with fresh rho scaling.  warm_start() alone resets primal/dual
        # variables but leaves adaptive-rho at whatever value the previous
        # episode drove it to, causing mis-scaled iterations at episode start.
        self._osqp_prob = None
        self._observer_gain = None  # re-cached lazily on first control() call

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
        t:            float | None = None,   # current sim time (schedule mode)
        traj_fn=None,           # t -> (p_d, dp_d, ddp_d)     (schedule mode)
        dyn_query_fn=None,      # (q_i,dq_i) -> (J_v,Λ⁻¹,Λ,Cq̇) (schedule mode)
        joint_traj_fn=None,     # t -> (q_d,q̇_d,q̈_d), preferred schedule source
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

        # Horizon scheduling, computed EARLY (before the mode dispatch below)
        # so backbone_track can build the LTV backbone prediction from
        # lam_inv_list, not just the torque-constraint rows built later.
        # J_list/tauff_list are reused verbatim near the end of this method
        # (not recomputed) for the tau_base_list construction.
        # joint_traj_fn is required for reference scheduling (no coast
        # fallback, see ImpedanceMPCParams.horizon_torque_schedule); if
        # horizon_torque_schedule is set but no reference trajectory was
        # supplied, schedule_active stays False and the final dispatch
        # below falls back to horizon_torque_constraint's frozen-at-q_k
        # rows instead.
        schedule_active = (self.p.horizon_torque_schedule
                           and traj_fn is not None and dyn_query_fn is not None
                           and t is not None and joint_traj_fn is not None)
        if schedule_active:
            J_list, tauff_list, lam_inv_list = self._schedule_horizon(
                q, dq, t, J_v, tau_ff, Lam_inv, traj_fn, dyn_query_fn, joint_traj_fn)

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
            F_backbone = np.zeros(3)
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
            F_backbone = np.zeros(3)
        elif self.p.observer_only:
            # ---- Observer-impedance baseline (non-predictive ablation) ---
            # Same construction as impedance_track, but G is the realized
            # first-move gain of the unconstrained MPC (eq:unconstrained),
            # evaluated once and cached, NOT a hand-chosen impedance
            # stiffness and NOT re-solved by horizon optimization every
            # tick. Isolates estimator credit from horizon-prediction
            # credit: matching C5 here under nominal conditions means the
            # steady-state improvement is attributable to d_hat, not to MPC
            # prediction as such.
            if self._observer_gain is None:
                self._observer_gain = self.unconstrained_first_move_gain(Lam_inv)
            G = self._observer_gain
            A_cl = self.A_d + Bd @ G
            U_obs = np.zeros(3 * self.p.N)
            xi = x_e.copy()
            for i in range(self.p.N):
                U_obs[3*i:3*i+3] = G @ xi - d_hat
                xi = A_cl @ xi
            H = self.R_bar
            h = -self.R_bar @ U_obs
            F_backbone = np.zeros(3)
        elif self.p.backbone_track:
            # ---- Impedance-backbone + bounded additive MPC correction ----
            # F_backbone = K_bb e + D_bb ė is commanded independently below
            # (folded into tau_base), so it remains present if the QP's
            # F_mpc saturates or falls back to zero. The QP here only shapes a
            # BOUNDED additional correction on top, with the LQ-regulation
            # cost (same Q_bar/R_bar as the default branch) predicted
            # through the backbone dynamics LINEARIZED ALONG A NOMINAL
            # TRAJECTORY: A_cl = A_d + Bd @ G_bb frozen at the current
            # configuration by default, or A_cl,i = A_d + Bd_i @ G_bb
            # scheduled per horizon step when horizon_torque_schedule is
            # also on (schedule_active) — either way the "residual plant"
            # the QP sees already has the backbone's restoring
            # stiffness/damping in it, just evaluated at one configuration
            # (frozen) or along the schedule (LTV) rather than re-derived
            # from an open-loop A_d.
            K_bb = self.p.k_backbone * np.eye(3)
            D_bb = 2.0 * self.p.zeta_backbone * np.sqrt(self.p.k_backbone) * np.eye(3)
            G_bb = np.hstack([K_bb, D_bb])                    # (3×6) backbone gain
            F_backbone = G_bb @ x_e

            if schedule_active:
                # Bd,i scheduled consistently with J_v,i/τ_ff,i above — the
                # torque MAP and the PREDICTION MODEL now vary together
                # along the horizon, rather than the torque map alone.
                Bd_list = [self._B_d(li) for li in lam_inv_list]
                Phi_cl, Gamma, D_bar = self._build_scheduled_closed_loop_horizon(Bd_list, G_bb)
            else:
                A_cl = self.A_d + Bd @ G_bb
                Phi_cl, Gamma, D_bar = self._build_closed_loop_horizon(Bd, A_cl)
            x_free = Phi_cl @ x_e + D_bar @ d_hat

            H = Gamma.T @ self.Q_bar @ Gamma + self.R_bar
            H = 0.5 * (H + H.T)
            # Same offset-free input-centering trick as the default branch,
            # applied to the ADDITIONAL correction (backbone itself is not
            # offset-free — see the param docstring).
            d_seq = np.tile(d_hat, self.p.N)
            h = Gamma.T @ self.Q_bar @ x_free + self.R_bar @ d_seq
        else:
            F_backbone = np.zeros(3)
            H, h, Gamma, D_bar, x_free = self._default_qp_objective(
                x_e, d_hat, Lam_inv)

        # ── Orientation stabilisation (simple impedance, runs every step) ─
        e_R  = rotation_error_matrix(R_d, ee_rot)    # (3,) axis-angle error
        de_R = ee_vel[3:]                            # angular velocity error
        tau_orient = J_w.T @ (-self.p.K_rot * e_R - self.p.D_rot * de_R)

        # ── Null-space joint-limit avoidance (barrier + centering + damping) ───
        tau_null = self.null_torque(q, dq, N_bar)

        # F_backbone is nonzero only in backbone_track mode; folding it into
        # tau_base means it is commanded independently of the QP (it remains
        # present if the QP below returns F_mpc = 0) and that the
        # torque-realizability constraint(s) in _solve_qp correctly account
        # for it as part of the frozen offset. Physical actuator clipping can
        # still prevent realization if tau_base itself exceeds tau_max.
        tau_base = tau_ff + J_v.T @ F_backbone + tau_orient + tau_null

        # ── Horizon-wide torque constraint rows: J_list/tau_base_list ─────
        if not self.p.apply_torque_constraint:
            # (9b) omitted entirely: _solve_qp receives J_list=None and
            # falls back to the force-box-only QP (see _torque_constraint_
            # matrix / _solve_qp_osqp's `if A_tau is not None` branch).
            J_list        = None
            tau_base_list = None
        elif schedule_active:
            # J_list/tauff_list already computed above (before the mode
            # dispatch, so backbone_track could also use lam_inv_list) —
            # reused here, not recomputed. J_v,i/τ_ff,i are reference-
            # scheduled (see _schedule_horizon); orientation/null/backbone
            # stay frozen at their step-0 value (extra = tau_base -
            # tau_ff), matching how horizon_torque_constraint already
            # freezes those.
            extra = tau_base - tau_ff
            tau_base_list = [tf + extra for tf in tauff_list]
        elif self.p.horizon_torque_constraint or self.p.horizon_torque_schedule:
            # Frozen-at-q_k, horizon-wide (9c). Also the fallback for
            # horizon_torque_schedule=True when no joint_traj_fn was
            # supplied (schedule_active False) — see the
            # ImpedanceMPCParams.horizon_torque_schedule docstring.
            J_list        = [J_v] * self.p.N
            tau_base_list = [tau_base] * self.p.N
        else:
            J_list        = [J_v]
            tau_base_list = [tau_base]

        u_seq = self._solve_qp(H, h, J_list=J_list, tau_base_list=tau_base_list)  # (3N,)
        F_mpc = u_seq[:3]                                        # first step
        self._F_prev = F_mpc.copy()
        self.F_seq   = u_seq.reshape(self.p.N, 3)

        tau_mpc = J_v.T @ F_mpc
        tau = tau_base + tau_mpc
        return tau, F_mpc
