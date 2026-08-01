"""Constrained predictive realization of configurable interaction dynamics.

The plant is a planar point mass driven by the robot command ``u`` and an
external human force ``f_h``:

    m_r p_ddot = u + f_h.

A reference generator supplies an affine desired acceleration

    a_id = C x + G f_h,  x = [p_x, p_y, v_x, v_y].

The MPC does not tune the generator parameters.  It chooses the robot force
sequence that minimizes the horizon-wide realization residual
``a_actual - a_id`` while enforcing command, speed, and workspace constraints.
The implementation intentionally stays small and self-contained so that the
paper's central distinction can be inspected directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
import osqp
from scipy import sparse


class InteractionDynamicsGenerator(Protocol):
    """Interface implemented by every reference interaction model."""

    name: str

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        """Return matrices ``C, G`` in ``a_id = C x + G f_h``."""

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        """Evaluate the desired interaction acceleration."""


@dataclass(frozen=True)
class ImpedanceReference:
    """Desired mass--spring--damper interaction dynamics.

    ``M_d a_id + D_d v + K_d p = f_h``.
    """

    mass: float = 2.0
    stiffness: float = 45.0
    damping: float = 18.0
    name: str = "impedance"

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.hstack(
            (
                -(self.stiffness / self.mass) * np.eye(2),
                -(self.damping / self.mass) * np.eye(2),
            )
        )
        g = (1.0 / self.mass) * np.eye(2)
        return c, g

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        c, g = self.affine_law()
        return c @ np.asarray(state) + g @ np.asarray(human_force)


@dataclass(frozen=True)
class AdmittanceReference:
    """Force-guided velocity dynamics with no position-restoring spring.

    ``T a_id + v = Y f_h``.  After force release, velocity decays to zero and
    the displaced position is retained.  This behavior is deliberately
    different from the impedance generator while using the same MPC layer.
    """

    time_constant: float = 0.25
    force_to_velocity: float = 0.025
    name: str = "admittance"

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.hstack(
            (np.zeros((2, 2)), -(1.0 / self.time_constant) * np.eye(2))
        )
        g = (self.force_to_velocity / self.time_constant) * np.eye(2)
        return c, g

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray:
        c, g = self.affine_law()
        return c @ np.asarray(state) + g @ np.asarray(human_force)


@dataclass(frozen=True)
class MPCConfig:
    dt: float = 0.02
    horizon: int = 20
    robot_mass: float = 2.5
    force_limit: float = 18.0
    speed_limit: float = 0.22
    position_limit: float = 0.10
    realization_weight: float = 1.0
    force_weight: float = 2.0e-4
    force_rate_weight: float = 1.0e-3
    force_rate_limit: float = 180.0
    osqp_eps: float = 1.0e-7


@dataclass(frozen=True)
class MPCStep:
    command: np.ndarray
    predicted_residual_rms: float
    regularization_residual: np.ndarray
    constraint_intervention_residual: np.ndarray
    decomposition_closure_error: np.ndarray
    active_constraints: tuple[str, ...]
    status: str


class InteractionDynamicsMPC:
    """Condensed linear MPC for realization of an affine interaction model."""

    def __init__(
        self,
        generator: InteractionDynamicsGenerator,
        config: MPCConfig | None = None,
    ) -> None:
        self.generator = generator
        self.cfg = config or MPCConfig()
        dt = self.cfg.dt
        self.A = np.block(
            [
                [np.eye(2), dt * np.eye(2)],
                [np.zeros((2, 2)), np.eye(2)],
            ]
        )
        self.B = np.vstack(
            (
                0.5 * dt**2 / self.cfg.robot_mass * np.eye(2),
                dt / self.cfg.robot_mass * np.eye(2),
            )
        )
        self.previous_command = np.zeros(2)

    def reset(self) -> None:
        self.previous_command = np.zeros(2)

    def _condense(
        self, state: np.ndarray, force_forecast: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[str]]:
        """Build ``0.5 U' P U + q' U`` and ``lower <= A U <= upper``."""

        cfg = self.cfg
        n_u = 2 * cfg.horizon
        c_id, g_id = self.generator.affine_law()
        state_map = np.zeros((4, n_u))
        state_offset = np.asarray(state, dtype=float).copy()

        residual_maps: list[np.ndarray] = []
        residual_offsets: list[np.ndarray] = []
        constraint_maps: list[np.ndarray] = []
        lowers: list[np.ndarray] = []
        uppers: list[np.ndarray] = []
        labels: list[str] = []

        for k in range(cfg.horizon):
            selector = np.zeros((2, n_u))
            selector[:, 2 * k : 2 * k + 2] = np.eye(2)
            f_k = force_forecast[k]

            actual_map = selector / cfg.robot_mass
            actual_offset = f_k / cfg.robot_mass
            residual_maps.append(actual_map - c_id @ state_map)
            residual_offsets.append(actual_offset - c_id @ state_offset - g_id @ f_k)

            state_map = self.A @ state_map + self.B @ selector
            state_offset = self.A @ state_offset + self.B @ f_k

            constraint_maps.extend((state_map[:2], state_map[2:]))
            lowers.extend(
                (
                    -cfg.position_limit * np.ones(2) - state_offset[:2],
                    -cfg.speed_limit * np.ones(2) - state_offset[2:],
                )
            )
            uppers.extend(
                (
                    cfg.position_limit * np.ones(2) - state_offset[:2],
                    cfg.speed_limit * np.ones(2) - state_offset[2:],
                )
            )
            labels.extend((f"position[{k + 1}]", f"speed[{k + 1}]"))

        r_map = np.vstack(residual_maps)
        r_offset = np.concatenate(residual_offsets)

        difference = np.zeros((n_u, n_u))
        difference[:2, :2] = np.eye(2)
        for k in range(1, cfg.horizon):
            difference[2 * k : 2 * k + 2, 2 * k : 2 * k + 2] = np.eye(2)
            difference[2 * k : 2 * k + 2, 2 * (k - 1) : 2 * k] = -np.eye(2)
        difference_offset = np.zeros(n_u)
        difference_offset[:2] = -self.previous_command

        p = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_map)
            + cfg.force_weight * np.eye(n_u)
            + cfg.force_rate_weight * (difference.T @ difference)
        )
        q = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_offset)
            + cfg.force_rate_weight * (difference.T @ difference_offset)
        )
        p += 1.0e-9 * np.eye(n_u)

        identity = np.eye(n_u)
        constraint_maps.append(identity)
        lowers.append(-cfg.force_limit * np.ones(n_u))
        uppers.append(cfg.force_limit * np.ones(n_u))
        labels.append("command")

        constraint_maps.append(difference)
        rate_step = cfg.force_rate_limit * cfg.dt
        lowers.append(-rate_step * np.ones(n_u) - difference_offset)
        uppers.append(rate_step * np.ones(n_u) - difference_offset)
        labels.append("command_rate")

        return (
            p,
            q,
            np.vstack(constraint_maps),
            np.concatenate(lowers),
            np.concatenate(uppers),
            labels,
        )

    def control(
        self, state: np.ndarray, force_forecast: np.ndarray
    ) -> MPCStep:
        state = np.asarray(state, dtype=float)
        force_forecast = np.asarray(force_forecast, dtype=float)
        expected_shape = (self.cfg.horizon, 2)
        if force_forecast.shape != expected_shape:
            raise ValueError(
                f"force_forecast must have shape {expected_shape}, "
                f"got {force_forecast.shape}"
            )

        p, q, a_con, lower, upper, labels = self._condense(state, force_forecast)
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
            max_iter=10_000,
        )
        result = solver.solve(raise_error=False)
        if result.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP failed: {result.info.status}")

        sequence = np.asarray(result.x)
        command = sequence[:2].copy()

        # Counterfactual diagnostic, not a second controller: minimize the
        # identical horizon objective after removing every physical
        # constraint.  This retains force and slew regularization, including
        # the same previous-command state, and therefore isolates how much of
        # the first-step realization residual exists before constraints act.
        unconstrained_sequence = np.linalg.solve(p, -q)
        unconstrained_command = unconstrained_sequence[:2]
        c_id, g_id = self.generator.affine_law()
        f_0 = force_forecast[0]
        a_id_0 = c_id @ state + g_id @ f_0
        a_unconstrained_0 = (unconstrained_command + f_0) / self.cfg.robot_mass
        a_constrained_0 = (command + f_0) / self.cfg.robot_mass
        regularization_residual = a_unconstrained_0 - a_id_0
        constraint_intervention_residual = a_constrained_0 - a_unconstrained_0
        decomposition_closure_error = (
            a_constrained_0
            - a_id_0
            - regularization_residual
            - constraint_intervention_residual
        )
        self.previous_command = command

        value = a_con @ sequence
        tolerance = 2.0e-4
        active: list[str] = []
        row = 0
        for label, block in zip(
            labels,
            [2] * (2 * self.cfg.horizon)
            + [2 * self.cfg.horizon, 2 * self.cfg.horizon],
        ):
            sl = slice(row, row + block)
            if np.any(value[sl] <= lower[sl] + tolerance) or np.any(
                value[sl] >= upper[sl] - tolerance
            ):
                active.append(label)
            row += block

        predicted_state = state.copy()
        residuals = []
        for k in range(self.cfg.horizon):
            u_k = sequence[2 * k : 2 * k + 2]
            f_k = force_forecast[k]
            a_actual = (u_k + f_k) / self.cfg.robot_mass
            a_reference = c_id @ predicted_state + g_id @ f_k
            residuals.append(a_actual - a_reference)
            predicted_state = self.A @ predicted_state + self.B @ (u_k + f_k)

        return MPCStep(
            command=command,
            predicted_residual_rms=float(np.sqrt(np.mean(np.square(residuals)))),
            regularization_residual=regularization_residual,
            constraint_intervention_residual=constraint_intervention_residual,
            decomposition_closure_error=decomposition_closure_error,
            active_constraints=tuple(active),
            status=result.info.status,
        )


def clipped_reference_command(
    generator: InteractionDynamicsGenerator,
    state: np.ndarray,
    human_force: np.ndarray,
    config: MPCConfig,
    previous_command: np.ndarray,
) -> np.ndarray:
    """Non-predictive baseline: realize the instantaneous law, then clip force.

    It shares the same command and slew limits as the MPC but has no preview of
    workspace or speed constraints.
    """

    a_reference = generator.acceleration(state, human_force)
    desired = config.robot_mass * a_reference - human_force
    max_delta = config.force_rate_limit * config.dt
    desired = np.clip(desired, previous_command - max_delta, previous_command + max_delta)
    return np.clip(desired, -config.force_limit, config.force_limit)


def integrate_point_mass(
    state: np.ndarray,
    command: np.ndarray,
    human_force: np.ndarray,
    mass: float,
    dt: float,
) -> np.ndarray:
    """Exact zero-order-hold update for the planar point mass."""

    acceleration = (np.asarray(command) + np.asarray(human_force)) / mass
    next_state = np.asarray(state, dtype=float).copy()
    next_state[:2] += dt * state[2:] + 0.5 * dt**2 * acceleration
    next_state[2:] += dt * acceleration
    return next_state
