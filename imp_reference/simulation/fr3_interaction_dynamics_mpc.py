"""Generator-realizer interaction-dynamics MPC for a real FR3 manipulator.

Extends the planar point-mass prototype (interaction_dynamics_mpc.py) to a
7-DOF Franka FR3 in MuJoCo. The generator interface and the realization-QP
*concept* are unchanged: a generator specifies a desired task-space
acceleration ``a_id = C x + G f_h``, and a receding-horizon QP picks the
robot's Cartesian correction force sequence that most closely reproduces it.

What changes relative to the planar prototype is the plant. The point mass
is exactly LTI (``m_r p_ddot = u + f_h``), so a single fixed ``A``/``B``
pair is exact over the whole horizon. A manipulator is not: after
feedback-linearizing away gravity/Coriolis (a fixed ``tau_base`` computed
once per solve), the residual translational dynamics are

    e_ddot = Lambda(q)^{-1} (F_cmd + f_h)

where ``Lambda(q)`` is the task-space inertia -- configuration-dependent.
This module freezes ``Lambda(q_k)``, ``tau_base(q_k, qdot_k)``, and the
translational Jacobian ``J_v(q_k)`` at the current configuration across the
horizon (the same frozen-Jacobian approximation already used and validated
by the sibling double-integrator paper's C8 controller), which keeps the
per-solve problem an exact QP while being only a *local* approximation of
the true nonlinear horizon dynamics -- not an exact prediction of future
feasibility, same caveat as that sibling work.

The key correctness property this module is built around: torque
feasibility is enforced at *every* horizon step (frozen-Jacobian), not only
the first. A first-step-only formulation was exactly the gap a reviewer
caught in the sibling paper -- the QP's own internal plan could rely on
later steps that are not realizable by the actuators, even though the one
step actually applied never violates a limit itself.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np
import osqp
from scipy import sparse

_SIM_DIR = Path(__file__).resolve().parents[2] / "simulation"
sys.path.insert(0, str(_SIM_DIR))

from fr3_impedance import (  # noqa: E402
    FrankaDynamics,
    ImpedanceParams,
    RobotState,
)
from fr3_mujoco import Q_NEUTRAL, TAU_LIMIT  # noqa: E402
from so3_utils import rotation_error_matrix  # noqa: E402


# ---------------------------------------------------------------------------
# Generators
# ---------------------------------------------------------------------------


class FR3InteractionDynamicsGenerator(Protocol):
    """Interface implemented by every reference interaction model.

    Identical in spirit to the planar prototype's ``InteractionDynamicsGenerator``:
    the generator's affine law is configuration-independent (a virtual desired
    model, not a function of the real robot's inertia), so no ``Lambda``
    argument is needed here either -- just a larger (3-D, not 2-D) state.
    """

    name: str

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        """Return matrices ``C, G`` in ``a_id = C x + G f_h``, ``x = [e; edot] in R^6``."""

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        """Evaluate the desired interaction acceleration."""


@dataclass(frozen=True)
class ImpedanceReference3D:
    """Desired mass-spring-damper interaction dynamics, 3-D translational.

    ``M_d a_id + D_d edot + K_d e = f_h``.
    """

    mass: float = 2.0
    stiffness: float = 200.0
    damping: float = 28.0
    name: str = "impedance"

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.hstack(
            (
                -(self.stiffness / self.mass) * np.eye(3),
                -(self.damping / self.mass) * np.eye(3),
            )
        )
        g = (1.0 / self.mass) * np.eye(3)
        return c, g

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        c, g = self.affine_law()
        return c @ np.asarray(state) + g @ np.asarray(human_force)


@dataclass(frozen=True)
class AdmittanceReference3D:
    """Force-guided velocity dynamics with no position-restoring term.

    ``T a_id + edot = Y f_h``. After force release, velocity decays and the
    displaced position is retained -- the same qualitatively different
    behavior as the planar prototype's admittance generator.
    """

    time_constant: float = 0.3
    force_to_velocity: float = 0.01
    name: str = "admittance"

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.hstack((np.zeros((3, 3)), -(1.0 / self.time_constant) * np.eye(3)))
        g = (self.force_to_velocity / self.time_constant) * np.eye(3)
        return c, g

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        c, g = self.affine_law()
        return c @ np.asarray(state) + g @ np.asarray(human_force)


# ---------------------------------------------------------------------------
# Configuration and result types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FR3MPCConfig:
    dt: float = 0.02  # QP solve period (s); inner loop can run faster, see run_fr3_experiments.py
    horizon: int = 15
    tau_max: np.ndarray = field(default_factory=lambda: TAU_LIMIT.copy())
    force_limit: float = 150.0  # generous Cartesian-force safety net; torque is the real constraint
    speed_limit: float = 0.35
    position_limit: float = 0.06
    realization_weight: float = 1.0
    # 1e-2, not 1e-6: this Tikhonov-style regularization term on the QP
    # Hessian diagonal was found empirically necessary for OSQP to converge
    # quickly (~2ms vs ~60-200ms per solve) on this problem's conditioning;
    # it does not materially change the realized command.
    force_weight: float = 1.0e-2
    force_rate_weight: float = 1.0e-4
    force_rate_limit: float = 2000.0
    # Position/speed box constraints are slack-relaxed (see _condense), not
    # hard: a large penalty here keeps the QP feasible under model mismatch
    # while still making violation prohibitively costly relative to the
    # realization objective. 1e4 was tried first and let the QP trade away
    # the box far too readily (observed slack of several meters); matches
    # the sibling paper's own CBF slack penalty weight (w_s = 1e8), which
    # solves the identical "keep the QP always feasible without letting it
    # cheaply ignore the soft constraint" problem.
    slack_weight: float = 1.0e8
    K_rot: float = 20.0
    D_rot: float = 6.0
    # 40.0 / 8.0, not 10.0 / 2.0: the weaker gains left the null-space
    # centering torque too small to counteract the residual (regularization-
    # induced, no longer exactly zero -- see compute_tau_base's docstring)
    # leakage from tau_aux over a full multi-second run. That residual
    # leakage is tiny per tick, but it is a slow, one-directional drift, and
    # a weak restoring spring lets it accumulate to over a radian of joint
    # deviation from Q_NEUTRAL by mid-run in several benchmark conditions --
    # confirmed by sweep_null_space_gains.py, saved as
    # results/null_space_gain_sweep.json, tracking max deviation over the
    # full 6 s run (not just an early window) in all four benchmark
    # conditions. Stronger
    # settings (e.g. 100/20) control drift even further but measurably
    # suppress the admittance generator's own task-space response (checked
    # directly, not assumed); 40/8 was the gentlest increase in the sweep
    # that keeps deviation under ~0.2 rad in every benchmark condition
    # without visibly damping the requested interaction dynamics.
    k_null: float = 40.0
    d_null: float = 8.0
    lambda_reg: float = 1.0e-6
    osqp_eps: float = 1.0e-4
    osqp_max_iter: int = 20_000


@dataclass(frozen=True)
class FR3MPCStep:
    command: np.ndarray  # (3,) F_cmd_0, Cartesian correction force
    tau: np.ndarray  # (7,) full joint torque = tau_base + J_v^T @ command
    predicted_residual_rms: float
    active_constraints: tuple[str, ...]
    status: str
    solve_time_s: float
    horizon_tau: np.ndarray  # (horizon, 7) predicted torque at EVERY horizon step, not just i=0
    horizon_slack_max: float  # max slack used across the horizon; 0 = workspace/speed box never traded off


# ---------------------------------------------------------------------------
# Shared torque assembly (used by both the predictive and reactive paths)
# ---------------------------------------------------------------------------


def compute_tau_base(
    dyn: FrankaDynamics,
    state: RobotState,
    R_d: np.ndarray,
    imp_params: ImpedanceParams,
    K_rot: float,
    D_rot: float,
    lambda_reg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Feedforward + orientation-hold + null-space torque, J_v, and the
    known residual acceleration from auxiliary torque plus ``Jdot_v*qdot``.

    ``tau_ff = dyn.Cq_dot`` exactly cancels the joint-space bias C*qdot+g in
    the real dynamics (the nominal interaction pose is held fixed, so there
    is no acceleration-feedforward term -- see ``guidance.py``'s identical
    static-hold simplification in the sibling paper), so it contributes
    nothing to the *residual* translational dynamics the QP reasons about.

    ``tau_orient`` and ``tau_null`` are projected through the null-space
    projector of the POSITION Jacobian ``J_v`` alone (not the full 6-row
    Jacobian): ``N_bar_v = I - J_v_bar @ J_v`` with ``J_v_bar = M^-1 J_v^T
    Lambda_pos``. This is the standard dynamically-consistent decomposition
    (Khatib 1987) restricted to the 3-DOF position task. With an exact inverse
    it gives ``J_v @ M^-1 @ N_bar_v^T == 0``. This implementation uses the
    same Tikhonov regularization (``lambda_reg``, the config's single
    regularization constant -- previously a separate hardcoded value was
    used here, inconsistent with the value recorded in the benchmark
    configuration) in ``Lambda_pos`` for numerical robustness, so the
    decoupling is approximate rather than algebraically exact; the
    remaining, directly computed leakage is retained as ``d_known``.

    An earlier version of this module projected ``tau_null`` through the
    FULL-task null-space projector (built from the 6-row Jacobian including
    orientation) and left ``tau_orient`` entirely unprojected (raw ``J_w.T
    @ F_rot``). Neither choice is null-space-orthogonal to the POSITION
    task alone, so both leaked into translational acceleration. That
    leakage was folded into a ``d_known`` correction term and cancelled by
    the QP after the fact -- but the correction force needed to cancel it
    was itself applied via the same unprojected torque path, creating a
    positive feedback loop: null-space/orientation leakage grows q's
    deviation from neutral, which grows the leakage further, which grows
    the compensating force, which leaks more. This was confirmed to be a
    genuine closed-loop instability (unbounded q drift away from
    Q_NEUTRAL, occurring even during undisturbed hold, insensitive to
    ``k_null``/``d_null``/``K_rot``/``D_rot`` gain changes -- ruling out
    "just tune the gains" as a fix) and not fixable by tuning; only
    preventing the leakage at its source (this projection) fixes it.
    Projecting through ``N_bar_v`` instead makes ``d_known`` small and
    explicitly modeled, removing the large unprojected feedback path without
    pretending the regularized projector is exact.

    Recomputing this every inner-loop tick (not just at QP-solve ticks) is
    what keeps the loop at its claimed rate -- see ``run_fr3_experiments.py``.
    """
    J_v = dyn.J[:3, :]
    J_w = dyn.J[3:, :]

    tau_ff = dyn.Cq_dot  # static hold reference: no ddx_d term

    e_R = rotation_error_matrix(R_d, state.ee_rot)
    tau_orient_raw = J_w.T @ (-K_rot * e_R - D_rot * state.ee_vel[3:])

    tau_null_raw = -imp_params.k_null * (state.q - imp_params.q_null) - imp_params.d_null * state.dq

    M_inv = np.linalg.inv(dyn.M)
    Lam_pos = np.linalg.inv(J_v @ M_inv @ J_v.T + lambda_reg * np.eye(3))
    J_v_bar = M_inv @ J_v.T @ Lam_pos
    n_dof = J_v.shape[1]
    N_bar_v = np.eye(n_dof) - J_v_bar @ J_v

    tau_aux = N_bar_v.T @ (tau_orient_raw + tau_null_raw)
    # The task acceleration is J_v qdd + Jdot_v qdot. Bias cancellation removes
    # C(q,qdot)qdot+g(q) from qdd, but it does not remove the kinematic
    # Jdot_v*qdot term. Both known contributions belong in the residual model.
    d_aux = J_v @ M_inv @ tau_aux  # small but nonzero under regularization
    d_kinematic = dyn.dJ[:3, :] @ state.dq
    d_known = d_aux + d_kinematic

    return tau_ff + tau_aux, J_v, d_known


def combine_full_torque(tau_base: np.ndarray, J_v: np.ndarray, f_cartesian: np.ndarray) -> np.ndarray:
    """tau = tau_base + J_v^T @ f_cartesian, the one formula both controllers share."""
    return tau_base + J_v.T @ f_cartesian


def make_default_impedance_params(cfg: FR3MPCConfig) -> ImpedanceParams:
    return ImpedanceParams(
        K=np.zeros(6),  # unused: this module never calls cartesian_impedance_control
        D=np.zeros(6),
        k_null=cfg.k_null,
        d_null=cfg.d_null,
        q_null=Q_NEUTRAL.copy(),
    )


# ---------------------------------------------------------------------------
# Predictive realization controller
# ---------------------------------------------------------------------------


class FR3RealizationMPC:
    """Condensed QP realizing an affine 3-D interaction model on the FR3."""

    def __init__(
        self,
        generator: FR3InteractionDynamicsGenerator,
        config: FR3MPCConfig | None = None,
    ) -> None:
        self.generator = generator
        self.cfg = config or FR3MPCConfig()
        self.imp_params = make_default_impedance_params(self.cfg)
        self.previous_command = np.zeros(3)

    def reset(self) -> None:
        self.previous_command = np.zeros(3)

    def _condense(
        self,
        dyn: FrankaDynamics,
        state: RobotState,
        p_nominal: np.ndarray,
        R_d: np.ndarray,
        force_forecast: np.ndarray,
    ):
        cfg = self.cfg
        dt = cfg.dt
        H = cfg.horizon
        n_i = 3 * H  # F_cmd_0 .. F_cmd_{H-1}
        n_s = 2 * H  # s_pos_0..s_pos_{H-1}, then s_speed_0..s_speed_{H-1}
        n = n_i + n_s

        def pos_slack_col(k: int) -> int:
            return n_i + k

        def speed_slack_col(k: int) -> int:
            return n_i + H + k

        # --- Freeze Lambda(q_k), tau_base(q_k, qdot_k), J_v(q_k) for the horizon ---
        J_v = dyn.J[:3, :]
        M_inv = np.linalg.inv(dyn.M)
        Lam_inv = J_v @ M_inv @ J_v.T + cfg.lambda_reg * np.eye(3)  # = Lambda(q_k)^{-1}
        tau_base, _, d_known = compute_tau_base(dyn, state, R_d, self.imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg)

        A = np.block(
            [
                [np.eye(3), dt * np.eye(3)],
                [np.zeros((3, 3)), np.eye(3)],
            ]
        )
        B = np.vstack((0.5 * dt**2 * Lam_inv, dt * Lam_inv))
        # d_known is already an acceleration (not a force), so it propagates
        # through the SAME dt/dt^2 integration as B but WITHOUT another
        # Lambda^{-1} factor.
        B_accel = np.vstack((0.5 * dt**2 * np.eye(3), dt * np.eye(3)))

        e0 = state.ee_pos - p_nominal
        edot0 = state.ee_vel[:3]
        x0 = np.concatenate([e0, edot0])

        c_id, g_id = self.generator.affine_law()
        state_map = np.zeros((6, n_i))  # only the F_cmd block; slack columns never affect state
        state_offset = x0.copy()

        residual_maps: list[np.ndarray] = []
        residual_offsets: list[np.ndarray] = []
        constraint_maps: list[np.ndarray] = []
        lowers: list[np.ndarray] = []
        uppers: list[np.ndarray] = []
        labels: list[str] = []

        for k in range(H):
            selector = np.zeros((3, n_i))
            selector[:, 3 * k : 3 * k + 3] = np.eye(3)
            f_k = force_forecast[k]

            actual_map = Lam_inv @ selector
            actual_offset = Lam_inv @ f_k + d_known
            residual_maps.append(actual_map - c_id @ state_map)
            residual_offsets.append(actual_offset - c_id @ state_offset - g_id @ f_k)

            state_map = A @ state_map + B @ selector
            state_offset = A @ state_offset + B @ f_k + B_accel @ d_known

            # Cartesian workspace / speed box -- SOFT (slack-relaxed). Hard
            # box constraints on predicted state have no recursive-feasibility
            # guarantee (paper.md Prop. 3): once the real, nonlinear MuJoCo
            # trajectory drifts from this frozen-Jacobian prediction enough
            # to sit right at an active bound, the very next solve can become
            # genuinely infeasible (confirmed empirically -- tightening or
            # loosening the bound value alone did not remove it, nor did
            # loosening the torque limit, isolating the box constraints as
            # the cause). A large slack penalty keeps the QP always solvable
            # while still strongly discouraging violation, matching the same
            # slack-relaxation pattern the sibling paper's CBF filter already
            # uses for exactly this problem.
            pos_map = np.zeros((6, n))
            pos_map[:, :n_i] = np.vstack([state_map[:3], state_map[:3]])
            pos_map[:3, pos_slack_col(k)] = -1.0  # v - s <= upper
            pos_map[3:, pos_slack_col(k)] = 1.0  # v + s >= lower
            constraint_maps.append(pos_map)
            lowers.append(
                np.concatenate(
                    [-np.inf * np.ones(3), -cfg.position_limit * np.ones(3) - state_offset[:3]]
                )
            )
            uppers.append(
                np.concatenate(
                    [cfg.position_limit * np.ones(3) - state_offset[:3], np.inf * np.ones(3)]
                )
            )
            labels.append(f"position[{k + 1}]")

            speed_map = np.zeros((6, n))
            speed_map[:, :n_i] = np.vstack([state_map[3:], state_map[3:]])
            speed_map[:3, speed_slack_col(k)] = -1.0
            speed_map[3:, speed_slack_col(k)] = 1.0
            constraint_maps.append(speed_map)
            lowers.append(
                np.concatenate(
                    [-np.inf * np.ones(3), -cfg.speed_limit * np.ones(3) - state_offset[3:]]
                )
            )
            uppers.append(
                np.concatenate(
                    [cfg.speed_limit * np.ones(3) - state_offset[3:], np.inf * np.ones(3)]
                )
            )
            labels.append(f"speed[{k + 1}]")

            # Horizon-wide joint-torque feasibility -- the point of this
            # module, and kept HARD: unlike the workspace box above, this is
            # a genuine actuator limit, not a preference to trade off.
            # tau_i = tau_base(q_k) + J_v(q_k)^T F_cmd_i, frozen at q_k,
            # checked for EVERY i, not only i=0.
            torque_map = np.zeros((7, n))
            torque_map[:, 3 * k : 3 * k + 3] = J_v.T
            constraint_maps.append(torque_map)
            lowers.append(-cfg.tau_max - tau_base)
            uppers.append(cfg.tau_max - tau_base)
            labels.append(f"torque[{k}]")

        # Slack non-negativity.
        slack_bounds = np.zeros((n_s, n))
        slack_bounds[:, n_i:] = np.eye(n_s)
        constraint_maps.append(slack_bounds)
        lowers.append(np.zeros(n_s))
        uppers.append(np.inf * np.ones(n_s))
        labels.append("slack_nonneg")

        r_map = np.vstack([m for m in residual_maps])
        r_map = np.hstack([r_map, np.zeros((r_map.shape[0], n_s))])  # residual is slack-independent
        r_offset = np.concatenate(residual_offsets)

        difference = np.zeros((n_i, n_i))
        difference[:3, :3] = np.eye(3)
        for k in range(1, H):
            difference[3 * k : 3 * k + 3, 3 * k : 3 * k + 3] = np.eye(3)
            difference[3 * k : 3 * k + 3, 3 * (k - 1) : 3 * k] = -np.eye(3)
        difference_offset = np.zeros(n_i)
        difference_offset[:3] = -self.previous_command
        difference_full = np.zeros((n_i, n))
        difference_full[:, :n_i] = difference

        p = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_map)
            + cfg.force_rate_weight * (difference_full.T @ difference_full)
        )
        p[:n_i, :n_i] += 2.0 * cfg.force_weight * np.eye(n_i)
        p[n_i:, n_i:] += 2.0 * cfg.slack_weight * np.eye(n_s)
        q = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_offset)
            + cfg.force_rate_weight * (difference_full.T @ difference_offset)
        )
        p += 1.0e-9 * np.eye(n)

        identity = np.zeros((n_i, n))
        identity[:, :n_i] = np.eye(n_i)
        constraint_maps.append(identity)
        lowers.append(-cfg.force_limit * np.ones(n_i))
        uppers.append(cfg.force_limit * np.ones(n_i))
        labels.append("command")

        constraint_maps.append(difference_full)
        rate_step = cfg.force_rate_limit * cfg.dt
        lowers.append(-rate_step * np.ones(n_i) - difference_offset)
        uppers.append(rate_step * np.ones(n_i) - difference_offset)
        labels.append("command_rate")

        return (
            p,
            q,
            np.vstack(constraint_maps),
            np.concatenate(lowers),
            np.concatenate(uppers),
            labels,
            Lam_inv,
            tau_base,
            J_v,
            d_known,
        )

    def control(
        self,
        dyn: FrankaDynamics,
        state: RobotState,
        p_nominal: np.ndarray,
        R_d: np.ndarray,
        force_forecast: np.ndarray,
    ) -> FR3MPCStep:
        force_forecast = np.asarray(force_forecast, dtype=float)
        expected_shape = (self.cfg.horizon, 3)
        if force_forecast.shape != expected_shape:
            raise ValueError(
                f"force_forecast must have shape {expected_shape}, got {force_forecast.shape}"
            )

        import time as _time

        t0 = _time.perf_counter()
        p, q, a_con, lower, upper, labels, Lam_inv, tau_base, J_v, d_known = self._condense(
            dyn, state, p_nominal, R_d, force_forecast
        )
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix(p),
            q=q,
            A=sparse.csc_matrix(a_con),
            l=lower,
            u=upper,
            verbose=False,
            polishing=False,
            eps_abs=self.cfg.osqp_eps,
            eps_rel=self.cfg.osqp_eps,
            max_iter=self.cfg.osqp_max_iter,
        )
        result = solver.solve(raise_error=False)
        solve_time = _time.perf_counter() - t0
        if result.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP failed: {result.info.status}")

        H = self.cfg.horizon
        n_i = 3 * H
        n_s = 2 * H
        sequence = np.asarray(result.x)
        command = sequence[:3].copy()
        self.previous_command = command
        tau = combine_full_torque(tau_base, J_v, command)

        value = a_con @ sequence
        tolerance = 2.0e-4
        active: list[str] = []
        row = 0
        block_sizes = ([6, 6, 7] * H) + [n_s, n_i, n_i]
        for label, block in zip(labels, block_sizes):
            sl = slice(row, row + block)
            if np.any(value[sl] <= lower[sl] + tolerance) or np.any(value[sl] >= upper[sl] - tolerance):
                active.append(label)
            row += block

        # Diagnostics: realization residual and torque feasibility over the
        # WHOLE predicted horizon (not just i=0) -- the property this module
        # exists to guarantee, checked directly rather than only inferred.
        predicted_x = np.concatenate([state.ee_pos - p_nominal, state.ee_vel[:3]])
        A = np.block([[np.eye(3), self.cfg.dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        B = np.vstack((0.5 * self.cfg.dt**2 * Lam_inv, self.cfg.dt * Lam_inv))
        B_accel = np.vstack((0.5 * self.cfg.dt**2 * np.eye(3), self.cfg.dt * np.eye(3)))
        c_id, g_id = self.generator.affine_law()
        residuals = []
        horizon_tau = np.zeros((self.cfg.horizon, 7))
        for k in range(self.cfg.horizon):
            f_k = sequence[3 * k : 3 * k + 3]
            fh_k = force_forecast[k]
            a_actual = Lam_inv @ (f_k + fh_k) + d_known
            a_reference = c_id @ predicted_x + g_id @ fh_k
            residuals.append(a_actual - a_reference)
            horizon_tau[k] = combine_full_torque(tau_base, J_v, f_k)
            predicted_x = A @ predicted_x + B @ (f_k + fh_k) + B_accel @ d_known

        slack_max = float(np.max(sequence[n_i:])) if n_s > 0 else 0.0

        return FR3MPCStep(
            command=command,
            tau=tau,
            predicted_residual_rms=float(np.sqrt(np.mean(np.square(residuals)))),
            active_constraints=tuple(active),
            status=result.info.status,
            solve_time_s=solve_time,
            horizon_tau=horizon_tau,
            horizon_slack_max=slack_max,
        )


# ---------------------------------------------------------------------------
# Reactive baseline
# ---------------------------------------------------------------------------


def fr3_clipped_reference_command(
    generator: FR3InteractionDynamicsGenerator,
    dyn: FrankaDynamics,
    state: RobotState,
    p_nominal: np.ndarray,
    human_force: np.ndarray,
    cfg: FR3MPCConfig,
    previous_command: np.ndarray,
    d_known: np.ndarray | None = None,
) -> np.ndarray:
    """Non-predictive baseline: realize the instantaneous law, then clip.

    Mirrors the planar prototype's ``clipped_reference_command``: evaluate
    the generator's desired acceleration for the *current* state only,
    convert to the Cartesian force that would realize it, then clip to the
    same command/rate box the predictive controller uses. No horizon, no
    lookahead on workspace/speed/torque constraints.

    ``d_known`` (see ``compute_tau_base``) is the known translational
    cross-coupling from the orientation/null-space torques; it is not a
    prediction, just the current instant's already-known physics, so a
    "naive" controller can and should still account for it.
    """
    e = state.ee_pos - p_nominal
    edot = state.ee_vel[:3]
    x = np.concatenate([e, edot])
    a_id = generator.acceleration(x, human_force)
    if d_known is None:
        d_known = np.zeros(3)

    J_v = dyn.J[:3, :]
    M_inv = np.linalg.inv(dyn.M)
    Lam_pos = np.linalg.inv(J_v @ M_inv @ J_v.T + cfg.lambda_reg * np.eye(3))
    desired = Lam_pos @ (a_id - d_known) - human_force

    max_delta = cfg.force_rate_limit * cfg.dt
    desired = np.clip(desired, previous_command - max_delta, previous_command + max_delta)
    return np.clip(desired, -cfg.force_limit, cfg.force_limit)
