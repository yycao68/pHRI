"""Two-rate predictive saturation-management benchmark.

This module is intentionally self-contained.  It evaluates the architecture in
``predictive_saturation_paper_v3.md`` on a 2-D interaction task:

* an analytic/learned nominal controller runs at 1 kHz;
* a predictive manager runs at 50 Hz;
* robot-specific actuator maps turn desired task acceleration into torque;
* the same abstract state and sampled successor-defect budget are checked
  across three actuator geometries.

The ``fr3_surrogate`` and ``arm6_surrogate`` cases are reduced-order actuator
geometry studies, not rigid-body or hardware validations.  Their purpose is to
exercise cross-realization refinement bookkeeping cheaply and deterministically.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from hashlib import sha256
from itertools import combinations
from time import perf_counter
from typing import Callable, Protocol

import numpy as np
import osqp
from scipy import sparse


Array = np.ndarray


@dataclass(frozen=True)
class BenchmarkConfig:
    fast_dt: float = 0.001
    slow_dt: float = 0.02
    horizon: int = 12
    duration: float = 1.6
    position_limit: float = 0.12
    speed_limit: float = 0.60
    acceleration_limit: float = 12.0
    acceleration_rate_limit: float = 120.0
    correction_weight: float = 1.0
    rate_weight: float = 0.025
    osqp_eps: float = 1.0e-5
    velocity_defect_radius: float = 0.030
    torque_bound_scale: float = 1.0
    state_bound_scale: float = 1.0

    @property
    def fast_per_slow(self) -> int:
        return int(round(self.slow_dt / self.fast_dt))

    def audit_config_hash(self) -> str:
        shared = (
            self.slow_dt,
            self.horizon,
            self.position_limit,
            self.speed_limit,
            self.velocity_defect_radius,
        )
        return sha256(repr(shared).encode("utf-8")).hexdigest()[:12]


@dataclass(frozen=True)
class Scenario:
    name: str
    initial_state: tuple[float, float, float, float]
    goal: Callable[[float], Array]
    human_force: Callable[[float], Array]
    torque_scale: Callable[[float], float]
    mismatch_scale: float = 1.0
    description: str = ""


def _ramp(t: float, t0: float, t1: float) -> float:
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3.0 - 2.0 * x)


def make_scenarios() -> dict[str, Scenario]:
    zero = lambda _t: np.zeros(2)
    one = lambda _t: 1.0
    return {
        "no_saturation": Scenario(
            "no_saturation",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.025 * np.sin(2.0 * np.pi * t / 2.0), 0.015]),
            zero,
            one,
            description="small nominal motion; manager should remain nearly inactive",
        ),
        "slow_saturation": Scenario(
            "slow_saturation",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.18 * _ramp(t, 0.25, 0.58), 0.02]),
            zero,
            lambda t: 1.0 - 0.75 * _ramp(t, 0.35, 1.10),
            description="slow goal and actuator-budget ramp",
        ),
        "sudden_disturbance": Scenario(
            "sudden_disturbance",
            (0.0, 0.0, 0.0, 0.0),
            zero,
            lambda t: np.array([16.0 if 0.76 <= t < 0.88 else 0.0, 0.0]),
            lambda t: 0.28 if 0.65 <= t < 1.05 else 1.0,
            description="force impulse between predictive updates",
        ),
        "directional_collapse": Scenario(
            "directional_collapse",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.015, 0.16 * _ramp(t, 0.30, 0.66)]),
            lambda t: np.array([0.0, 3.0 * _ramp(t, 0.55, 0.90)]),
            lambda t: 1.0 - 0.75 * _ramp(t, 0.45, 1.0),
            description="loss of authority primarily in task y direction",
        ),
        "point_of_no_return": Scenario(
            "point_of_no_return",
            (0.070, 0.0, 0.22, 0.0),
            lambda _t: np.array([0.160, 0.0]),
            zero,
            lambda t: 0.45 - 0.32 * _ramp(t, 0.25, 0.80),
            description="initial outward velocity near the position boundary",
        ),
        "model_mismatch": Scenario(
            "model_mismatch",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.13 * _ramp(t, 0.25, 0.95), -0.035]),
            lambda t: np.array([5.0 if 0.65 <= t < 1.35 else 0.0, -1.5]),
            lambda _t: 0.28,
            mismatch_scale=2.4,
            description="larger realization and force-model mismatch",
        ),
        "preview_mismatch": Scenario(
            "preview_mismatch",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.10 * _ramp(t, 0.20, 0.75), 0.03]),
            lambda t: np.array(
                [8.0 if 0.62 <= t < 1.02 else (-5.0 if 1.02 <= t < 1.28 else 0.0), 0.0]
            ),
            lambda _t: 0.26,
            description="force changes sign inside the prediction horizon",
        ),
        "horizon_ramp": Scenario(
            "horizon_ramp",
            (0.0, 0.0, 0.0, 0.0),
            lambda t: np.array([0.10 if t >= 0.58 else 0.0, 0.0]),
            zero,
            lambda t: 1.0 if t < 0.76 else 0.18,
            description=(
                "dedicated horizon ablation: command step precedes a future "
                "actuator-budget drop"
            ),
        ),
    }


@dataclass(frozen=True)
class RobotModel:
    name: str
    n_act: int
    torque_limits: Array
    mass: float
    h_nominal_fn: Callable[[Array, float], Array]
    base_nominal_fn: Callable[[Array, float], Array]
    h_error_bound: Array
    base_error_bound: Array
    true_h_error_fn: Callable[[float], Array]
    true_base_error_fn: Callable[[float], Array]

    def h_nominal(self, state: Array, t: float) -> Array:
        return np.asarray(self.h_nominal_fn(state, t), dtype=float)

    def h_true(self, state: Array, t: float, mismatch_scale: float) -> Array:
        return self.h_nominal(state, t) + mismatch_scale * self.true_h_error_fn(t)

    def base_nominal(self, state: Array, t: float) -> Array:
        return np.asarray(self.base_nominal_fn(state, t), dtype=float)

    def base_true(self, state: Array, t: float, mismatch_scale: float) -> Array:
        return self.base_nominal(state, t) + mismatch_scale * self.true_base_error_fn(t)

    def torque_error_bound(
        self,
        acceleration: Array,
        mismatch_scale: float,
        force: Array | None = None,
    ) -> Array:
        force_term = (
            np.zeros(2) if force is None else np.asarray(force) / self.mass
        )
        return mismatch_scale * (
            self.base_error_bound
            + self.h_error_bound @ np.abs(np.asarray(acceleration) - force_term)
        )

    def predicted_torque(
        self,
        state: Array,
        acceleration: Array,
        force_forecast: Array,
        t: float,
    ) -> Array:
        f_acc = np.asarray(force_forecast) / self.mass
        return self.base_nominal(state, t) + self.h_nominal(state, t) @ (
            np.asarray(acceleration) - f_acc
        )

    def true_pre_torque(
        self,
        state: Array,
        acceleration: Array,
        force: Array,
        t: float,
        mismatch_scale: float,
    ) -> Array:
        f_acc = np.asarray(force) / self.mass
        return self.base_true(state, t, mismatch_scale) + self.h_true(
            state, t, mismatch_scale
        ) @ (np.asarray(acceleration) - f_acc)

    def actual_acceleration(
        self,
        state: Array,
        applied_torque: Array,
        force: Array,
        t: float,
        mismatch_scale: float,
    ) -> Array:
        h = self.h_true(state, t, mismatch_scale)
        g = np.linalg.pinv(h)
        base = self.base_true(state, t, mismatch_scale)
        unmatched = mismatch_scale * np.array(
            [0.018 * np.sin(4.1 * t), 0.014 * np.cos(3.7 * t)]
        )
        return g @ (np.asarray(applied_torque) - base) + np.asarray(force) / self.mass + unmatched


def make_robots() -> dict[str, RobotModel]:
    def planar_h(state: Array, t: float) -> Array:
        p = state[:2]
        return np.array(
            [
                [2.10 + 0.22 * np.sin(1.2 * t + p[0]), 0.42],
                [0.28, 1.65 + 0.18 * np.cos(0.8 * t - p[1])],
            ]
        )

    def fr3_h(state: Array, t: float) -> Array:
        p = state[:2]
        return np.array(
            [
                [2.8 + 0.20 * np.sin(t), 0.25],
                [1.4, -1.05 - 0.12 * np.cos(1.3 * t)],
                [0.35, 1.85 + 0.15 * np.sin(0.7 * t)],
                [-2.05 - 0.18 * np.sin(p[0] + t), 0.45],
                [0.55, -0.82],
                [-0.38, 0.62 + 0.08 * np.cos(p[1] - t)],
                [0.22, 0.36],
            ]
        )

    def arm6_h(state: Array, t: float) -> Array:
        p = state[:2]
        return np.array(
            [
                [2.2, 0.52 + 0.1 * np.sin(t)],
                [-1.65, 0.35],
                [0.78 + 0.08 * np.cos(p[0]), 1.62],
                [0.42, -1.28 - 0.1 * np.sin(0.9 * t)],
                [-0.55, 0.74],
                [0.31, 0.48 + 0.06 * np.cos(p[1])],
            ]
        )

    def base_factory(scale: Array) -> Callable[[Array, float], Array]:
        def base(state: Array, t: float) -> Array:
            p, v = state[:2], state[2:]
            phase = np.arange(scale.size, dtype=float) * 0.43
            # Includes bias, orientation hold, and null-space consumption.
            return scale * (
                0.65 * np.sin(0.8 * t + phase)
                + 0.22 * p[0]
                - 0.14 * p[1]
                + 0.08 * np.linalg.norm(v)
            )

        return base

    def error_matrix(h: Array, fraction: float) -> Array:
        return fraction * np.maximum(np.abs(h), 0.25)

    def true_error_factory(bound: Array, phases: Array) -> Callable[[float], Array]:
        return lambda t: bound * np.sin(1.7 * t + phases[:, None])

    def true_base_factory(bound: Array) -> Callable[[float], Array]:
        phases = np.arange(bound.size) * 0.71
        return lambda t: bound * np.sin(2.1 * t + phases)

    specs = [
        (
            "planar_2r",
            np.array([9.0, 7.5]),
            2.2,
            planar_h,
            np.array([1.1, 0.9]),
        ),
        (
            "fr3_surrogate",
            np.array([16.0, 13.0, 12.0, 15.0, 6.0, 6.0, 4.5]),
            3.0,
            fr3_h,
            np.array([2.5, 2.0, 1.8, 2.4, 1.1, 1.0, 0.7]),
        ),
        (
            "arm6_surrogate",
            np.array([12.0, 10.0, 9.0, 8.5, 5.5, 4.5]),
            2.6,
            arm6_h,
            np.array([1.8, 1.5, 1.3, 1.2, 0.9, 0.7]),
        ),
    ]
    robots: dict[str, RobotModel] = {}
    for name, limits, mass, h_fn, base_scale in specs:
        h0 = h_fn(np.zeros(4), 0.0)
        h_bound = error_matrix(h0, 0.008)
        b_bound = 0.03 * np.ones(limits.size)
        phases = np.arange(limits.size, dtype=float) * 0.37
        robots[name] = RobotModel(
            name=name,
            n_act=limits.size,
            torque_limits=limits,
            mass=mass,
            h_nominal_fn=h_fn,
            base_nominal_fn=base_factory(base_scale),
            h_error_bound=h_bound,
            base_error_bound=b_bound,
            true_h_error_fn=true_error_factory(h_bound, phases),
            true_base_error_fn=true_base_factory(b_bound),
        )
    return robots


class FastController(Protocol):
    name: str

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        ...


@dataclass
class PDController:
    kp: float = 22.0
    kd: float = 8.0
    name: str = "pd"

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        del force
        return self.kp * (goal - state[:2]) - self.kd * state[2:]


@dataclass
class ImpedanceController:
    stiffness: float = 70.0
    damping: float = 15.0
    apparent_mass: float = 2.5
    name: str = "impedance"

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        return (
            self.stiffness * (goal - state[:2])
            - self.damping * state[2:]
            + force
        ) / self.apparent_mass


@dataclass
class RLPolicyController:
    weights: Array
    bias: Array
    name: str = "rl_policy"

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        feat = np.concatenate((state[:2], state[2:], goal - state[:2], 0.08 * force))
        return 8.0 * np.tanh(self.weights @ feat + self.bias)


@dataclass
class NeuralController:
    w_hidden: Array
    b_hidden: Array
    w_out: Array
    name: str = "neural_policy"

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        feat = np.concatenate((state, goal, 0.08 * force))
        hidden = np.tanh(self.w_hidden @ feat + self.b_hidden)
        return self.w_out @ np.concatenate((hidden, [1.0]))


@dataclass
class AIConditionedController:
    executor: PDController
    name: str = "ai_conditioned_proxy"

    def acceleration(self, state: Array, force: Array, goal: Array) -> Array:
        # ``goal`` is the upstream behavior primitive.  Only execution is tested.
        return self.executor.acceleration(state, force, goal)


def train_rl_policy(seed: int = 4) -> RLPolicyController:
    """Train a tiny policy by deterministic evolution strategy.

    The training objective is nominal double-integrator tracking.  This is a
    real reward-driven policy search, but deliberately small; it is not evidence
    for modern deep-RL performance.
    """

    rng = np.random.default_rng(seed)
    # Start near a stabilizing controller, then improve under nonlinear tanh.
    w = np.zeros((2, 8))
    w[:, 0:2] = -1.4 * np.eye(2)
    w[:, 2:4] = -0.55 * np.eye(2)
    w[:, 4:6] = 1.4 * np.eye(2)
    w[:, 6:8] = 0.16 * np.eye(2)
    b = np.zeros(2)
    theta = np.concatenate((w.ravel(), b))

    def reward(vector: Array) -> float:
        ww = vector[:-2].reshape(2, 8)
        bb = vector[-2:]
        total = 0.0
        dt = 0.01
        for goal in (np.array([0.09, 0.03]), np.array([-0.05, 0.08])):
            state = np.zeros(4)
            for k in range(120):
                force = np.array([3.0 if 35 <= k < 55 else 0.0, 0.0])
                feat = np.concatenate((state[:2], state[2:], goal - state[:2], 0.08 * force))
                acc = 8.0 * np.tanh(ww @ feat + bb)
                state[:2] += dt * state[2:] + 0.5 * dt**2 * acc
                state[2:] += dt * acc
                total -= float(8.0 * np.dot(state[:2] - goal, state[:2] - goal))
                total -= float(0.06 * np.dot(state[2:], state[2:]))
                total -= float(0.001 * np.dot(acc, acc))
        return total

    sigma = 0.08
    alpha = 0.035
    for _ in range(24):
        noise = rng.normal(size=(14, theta.size))
        scores_plus = np.array([reward(theta + sigma * n) for n in noise])
        scores_minus = np.array([reward(theta - sigma * n) for n in noise])
        scale = np.std(np.concatenate((scores_plus, scores_minus))) + 1.0e-9
        gradient = ((scores_plus - scores_minus)[:, None] * noise).mean(axis=0)
        theta += alpha * gradient / (2.0 * sigma * scale)
        sigma *= 0.985
    return RLPolicyController(theta[:-2].reshape(2, 8), theta[-2:])


def train_neural_policy(seed: int = 7) -> NeuralController:
    """Fit a fixed-hidden-layer neural controller to impedance demonstrations."""

    rng = np.random.default_rng(seed)
    n = 3000
    state = rng.uniform([-0.13, -0.13, -0.55, -0.55], [0.13, 0.13, 0.55, 0.55], (n, 4))
    goal = rng.uniform(-0.12, 0.12, (n, 2))
    force = rng.uniform(-8.0, 8.0, (n, 2))
    features = np.hstack((state, goal, 0.08 * force))
    target = (
        70.0 * (goal - state[:, :2]) - 15.0 * state[:, 2:] + force
    ) / 2.5
    w_hidden = rng.normal(scale=0.75, size=(18, 8))
    b_hidden = rng.normal(scale=0.25, size=18)
    hidden = np.tanh(features @ w_hidden.T + b_hidden)
    design = np.hstack((hidden, np.ones((n, 1))))
    w_out = np.linalg.lstsq(design, target, rcond=1.0e-6)[0].T
    return NeuralController(w_hidden, b_hidden, w_out)


def make_controllers() -> dict[str, FastController]:
    return {
        "pd": PDController(),
        "impedance": ImpedanceController(),
        "rl_policy": train_rl_policy(),
        "neural_policy": train_neural_policy(),
        "ai_conditioned_proxy": AIConditionedController(PDController(kp=25.0, kd=8.5)),
    }


def halfspaces_for_acceleration(
    robot: RobotModel,
    state: Array,
    force: Array,
    t: float,
    torque_scale: float,
    mismatch_scale: float,
    tightening: bool,
) -> tuple[Array, Array]:
    """Return A, b such that A acceleration <= b."""

    h = robot.h_nominal(state, t)
    base = robot.base_nominal(state, t)
    f_acc = force / robot.mass
    limits = torque_scale * robot.torque_limits
    if tightening:
        # Conservative bound evaluated at the configured acceleration box.
        delta = mismatch_scale * (
            robot.base_error_bound
            + robot.h_error_bound
            @ (np.full(2, 12.0) + np.abs(force / robot.mass))
        )
    else:
        delta = np.zeros(robot.n_act)
    upper = limits - delta - base + h @ f_acc
    lower = limits - delta + base - h @ f_acc
    return np.vstack((h, -h)), np.concatenate((upper, lower))


def reactive_state_halfspaces(
    state: Array,
    cfg: BenchmarkConfig,
    decay: float = 8.0,
) -> tuple[Array, Array]:
    """Relative-degree-two box CBF and speed braking halfspaces."""

    p = np.asarray(state[:2])
    v = np.asarray(state[2:])
    eye = np.eye(2)
    # Upper position: a <= -2 lambda v + lambda^2 (p_max - p).
    pos_upper = -2.0 * decay * v + decay**2 * (cfg.position_limit - p)
    # Lower position: -a <= 2 lambda v + lambda^2 (p_max + p).
    pos_lower = 2.0 * decay * v + decay**2 * (cfg.position_limit + p)
    speed_upper = decay * (cfg.speed_limit - v)
    speed_lower = decay * (cfg.speed_limit + v)
    return (
        np.vstack((eye, -eye, eye, -eye)),
        np.concatenate((pos_upper, pos_lower, speed_upper, speed_lower)),
    )


def project_polytope_2d(point: Array, a_mat: Array, b_vec: Array) -> Array:
    """Exact Euclidean projection onto a nonempty 2-D polytope."""

    point = np.asarray(point, dtype=float)
    if np.all(a_mat @ point <= b_vec + 1.0e-10):
        return point.copy()
    candidates: list[Array] = []
    for row, bound in zip(a_mat, b_vec):
        denom = float(row @ row)
        if denom <= 1.0e-14:
            continue
        cand = point - ((row @ point - bound) / denom) * row
        if np.all(a_mat @ cand <= b_vec + 1.0e-8):
            candidates.append(cand)
    for i, j in combinations(range(a_mat.shape[0]), 2):
        mat = np.vstack((a_mat[i], a_mat[j]))
        if abs(np.linalg.det(mat)) < 1.0e-10:
            continue
        cand = np.linalg.solve(mat, np.array([b_vec[i], b_vec[j]]))
        if np.all(a_mat @ cand <= b_vec + 1.0e-8):
            candidates.append(cand)
    if not candidates:
        # This can happen only if overly conservative tightening empties the
        # set.  Returning zero is a deterministic emergency fallback.
        return np.zeros(2)
    return min(candidates, key=lambda x: float(np.dot(x - point, x - point))).copy()


@dataclass
class ManagerResult:
    acceleration: Array
    nominal_first: Array
    planned_min_margin: float
    planned_max_violation: float
    planned_min_t2_slack: float
    feasible: bool
    solve_time_s: float
    sequence: Array


class PredictiveManager:
    def __init__(
        self,
        cfg: BenchmarkConfig,
        robot: RobotModel,
        *,
        constraint_steps: int | None = None,
        tightening: bool = True,
        preview_mode: str = "rollout",
        map_mode: str = "updated",
        smoothing: bool = True,
        certificate_constrained: bool = True,
    ) -> None:
        self.cfg = cfg
        self.robot = robot
        self.constraint_steps = constraint_steps or cfg.horizon
        self.tightening = tightening
        self.preview_mode = preview_mode
        self.map_mode = map_mode
        self.smoothing = smoothing
        self.certificate_constrained = certificate_constrained
        self.previous = np.zeros(2)

    def _force_forecast(self, scenario: Scenario, t: float, measured: Array) -> Array:
        if self.preview_mode == "oracle":
            return np.vstack(
                [scenario.human_force(t + i * self.cfg.slow_dt) for i in range(self.cfg.horizon)]
            )
        if self.preview_mode == "zero":
            return np.zeros((self.cfg.horizon, 2))
        return np.tile(measured, (self.cfg.horizon, 1))

    def _nominal_preview(
        self,
        controller: FastController,
        state: Array,
        scenario: Scenario,
        t: float,
        force_forecast: Array,
    ) -> tuple[Array, Array]:
        dt = self.cfg.slow_dt
        rollout = np.asarray(state, dtype=float).copy()
        seq = []
        states = []
        current = controller.acceleration(state, force_forecast[0], scenario.goal(t))
        for i in range(self.cfg.horizon):
            states.append(rollout.copy())
            if self.preview_mode == "zoh":
                acc = current.copy()
            else:
                acc = controller.acceleration(
                    rollout,
                    force_forecast[i],
                    scenario.goal(t + i * dt),
                )
            acc = np.clip(acc, -self.cfg.acceleration_limit, self.cfg.acceleration_limit)
            seq.append(acc)
            rollout[:2] += dt * rollout[2:] + 0.5 * dt**2 * acc
            rollout[2:] += dt * acc
        return np.asarray(seq), np.asarray(states)

    def control(
        self,
        controller: FastController,
        state: Array,
        scenario: Scenario,
        t: float,
        measured_force: Array,
    ) -> ManagerResult:
        started = perf_counter()
        cfg = self.cfg
        h_n = cfg.horizon
        force_forecast = self._force_forecast(scenario, t, measured_force)
        nominal, nominal_states = self._nominal_preview(
            controller, state, scenario, t, force_forecast
        )
        n_u = 2 * h_n
        state_map = np.zeros((4, n_u))
        state_offset = np.asarray(state, dtype=float).copy()
        a_disc = np.block(
            [
                [np.eye(2), cfg.slow_dt * np.eye(2)],
                [np.zeros((2, 2)), np.eye(2)],
            ]
        )
        b_disc = np.vstack(
            (0.5 * cfg.slow_dt**2 * np.eye(2), cfg.slow_dt * np.eye(2))
        )

        con_maps: list[Array] = []
        lowers: list[Array] = []
        uppers: list[Array] = []
        predicted_torque_blocks: list[tuple[Array, Array, Array]] = []
        frozen_state = nominal_states[0]

        # Velocity certificate K_v: the one-step-ahead velocity block is
        # tightened by the certificate radius (velocity_defect_radius), so
        # that any a_req satisfying this constraint keeps the successor
        # inside the speed bound even under a successor defect up to that
        # radius. Position is left untightened: only the velocity-space
        # successor defect is measured and audited (Table III).
        speed_bound = (
            cfg.speed_limit - cfg.velocity_defect_radius
            if self.certificate_constrained
            else cfg.speed_limit
        )

        for i in range(h_n):
            selector = np.zeros((2, n_u))
            selector[:, 2 * i : 2 * i + 2] = np.eye(2)
            state_map = a_disc @ state_map + b_disc @ selector
            state_offset = a_disc @ state_offset
            con_maps.extend((state_map[:2], state_map[2:]))
            lowers.extend(
                (
                    -cfg.position_limit * np.ones(2) - state_offset[:2],
                    -speed_bound * np.ones(2) - state_offset[2:],
                )
            )
            uppers.extend(
                (
                    cfg.position_limit * np.ones(2) - state_offset[:2],
                    speed_bound * np.ones(2) - state_offset[2:],
                )
            )

            if i < self.constraint_steps:
                model_state = frozen_state if self.map_mode == "frozen" else nominal_states[i]
                model_t = t if self.map_mode == "frozen" else t + i * cfg.slow_dt
                h = self.robot.h_nominal(model_state, model_t)
                base = self.robot.base_nominal(model_state, model_t)
                f_acc = force_forecast[i] / self.robot.mass
                limits = scenario.torque_scale(model_t) * self.robot.torque_limits
                if self.tightening:
                    delta = scenario.mismatch_scale * (
                        self.robot.base_error_bound
                        + self.robot.h_error_bound
                        @ (
                            np.full(2, cfg.acceleration_limit)
                            + np.abs(force_forecast[i] / self.robot.mass)
                        )
                    )
                else:
                    delta = np.zeros(self.robot.n_act)
                torque_map = h @ selector
                torque_offset = base - h @ f_acc
                con_maps.append(torque_map)
                lowers.append(-limits + delta - torque_offset)
                uppers.append(limits - delta - torque_offset)
            # Always retain all horizon blocks for the planned-feasibility
            # audit, including steps intentionally omitted by an ablation.
            model_state = frozen_state if self.map_mode == "frozen" else nominal_states[i]
            model_t = t if self.map_mode == "frozen" else t + i * cfg.slow_dt
            h_audit = self.robot.h_nominal(model_state, model_t)
            base_audit = self.robot.base_nominal(model_state, model_t)
            f_acc_audit = force_forecast[i] / self.robot.mass
            limits_audit = scenario.torque_scale(model_t) * self.robot.torque_limits
            predicted_torque_blocks.append(
                (
                    h_audit,
                    base_audit - h_audit @ f_acc_audit,
                    limits_audit,
                )
            )

        identity = np.eye(n_u)
        con_maps.append(identity)
        lowers.append(-cfg.acceleration_limit * np.ones(n_u))
        uppers.append(cfg.acceleration_limit * np.ones(n_u))
        difference = np.zeros((n_u, n_u))
        difference[:2, :2] = np.eye(2)
        for i in range(1, h_n):
            difference[2 * i : 2 * i + 2, 2 * i : 2 * i + 2] = np.eye(2)
            difference[2 * i : 2 * i + 2, 2 * (i - 1) : 2 * i] = -np.eye(2)
        diff_offset = np.zeros(n_u)
        diff_offset[:2] = -self.previous
        con_maps.append(difference)
        rate_step = cfg.acceleration_rate_limit * cfg.slow_dt
        lowers.append(-rate_step * np.ones(n_u) - diff_offset)
        uppers.append(rate_step * np.ones(n_u) - diff_offset)

        q_ref = nominal.reshape(-1)
        con_stack = np.vstack(con_maps)
        lower_stack = np.concatenate(lowers)
        upper_stack = np.concatenate(uppers)

        # Exact pass-through: if the nominal rollout already satisfies every
        # constraint row, skip the QP rather than let the smoothing term
        # perturb an already-feasible request.
        nominal_feasible = bool(
            np.all(con_stack @ q_ref >= lower_stack - 1.0e-9)
            and np.all(con_stack @ q_ref <= upper_stack + 1.0e-9)
        )
        if nominal_feasible:
            sequence = nominal.copy()
            feasible = True
        else:
            rate_weight = cfg.rate_weight if self.smoothing else 0.0
            p_mat = 2.0 * (
                cfg.correction_weight * np.eye(n_u)
                + rate_weight * difference.T @ difference
            )
            q_vec = -2.0 * cfg.correction_weight * q_ref
            q_vec += 2.0 * rate_weight * difference.T @ diff_offset
            p_mat += 1.0e-8 * np.eye(n_u)

            solver = osqp.OSQP()
            solver.setup(
                P=sparse.csc_matrix(p_mat),
                q=q_vec,
                A=sparse.csc_matrix(con_stack),
                l=lower_stack,
                u=upper_stack,
                verbose=False,
                polishing=False,
                eps_abs=cfg.osqp_eps,
                eps_rel=cfg.osqp_eps,
                max_iter=5000,
            )
            result = solver.solve(raise_error=False)
            feasible = result.info.status_val in (1, 2)
            if feasible:
                sequence = np.asarray(result.x[:n_u]).reshape(h_n, 2)
            else:
                a_h, b_h = halfspaces_for_acceleration(
                    self.robot,
                    state,
                    measured_force,
                    t,
                    scenario.torque_scale(t),
                    scenario.mismatch_scale,
                    self.tightening,
                )
                a_state, b_state = reactive_state_halfspaces(state, cfg)
                first = project_polytope_2d(
                    nominal[0],
                    np.vstack((a_h, a_state)),
                    np.concatenate((b_h, b_state)),
                )
                sequence = np.tile(first, (h_n, 1))
        self.previous = sequence[0].copy()

        margins = []
        violations = []
        t2_slacks = []
        for i, (h, offset, limits) in enumerate(predicted_torque_blocks):
            tau = offset + h @ sequence[i]
            margins.append(float(np.min(limits - np.abs(tau))))
            violations.append(float(np.max(np.maximum(np.abs(tau) - limits, 0.0))))
            # T2 slack must use the *same* bar-delta-tau that (T1) is
            # checked against post-hoc -- the as-executed bound evaluated at
            # the acceleration actually solved for, robot.torque_error_bound
            # -- not the QP's own internal worst-case tightening bound
            # (which substitutes cfg.acceleration_limit for the unknown
            # future request and is therefore much more conservative than
            # what (T1) is later checked against). Using the QP's internal
            # bound here previously produced a near-zero/negative slack that
            # did not match Theorem 1's own definition of (T2).
            delta_asexecuted = self.robot.torque_error_bound(
                sequence[i], scenario.mismatch_scale, force_forecast[i]
            )
            t2_slacks.append(
                float(np.min(limits - delta_asexecuted - np.abs(tau)))
            )
        min_margin = min(margins) if margins else float("nan")
        max_violation = max(violations) if violations else float("nan")
        min_t2_slack = min(t2_slacks) if t2_slacks else float("nan")
        return ManagerResult(
            acceleration=sequence[0].copy(),
            nominal_first=nominal[0].copy(),
            planned_min_margin=min_margin,
            planned_max_violation=max_violation,
            planned_min_t2_slack=min_t2_slack,
            feasible=feasible,
            solve_time_s=perf_counter() - started,
            sequence=sequence,
        )


def governor_scale(
    controller: FastController,
    robot: RobotModel,
    state: Array,
    scenario: Scenario,
    t: float,
    force: Array,
    cfg: BenchmarkConfig,
    *,
    tightening: bool = True,
) -> float:
    """Horizon reference governor using one scalar command multiplier."""

    dt = cfg.slow_dt

    def feasible(alpha: float) -> bool:
        x = state.copy()
        for i in range(cfg.horizon):
            ti = t + i * dt
            f_i = force  # measured-force hold, same information as default MPC
            governed_goal = state[:2] + alpha * (scenario.goal(ti) - state[:2])
            v = controller.acceleration(x, f_i, governed_goal)
            v = np.clip(v, -cfg.acceleration_limit, cfg.acceleration_limit)
            a_h, b_h = halfspaces_for_acceleration(
                robot,
                x,
                f_i,
                ti,
                scenario.torque_scale(ti),
                scenario.mismatch_scale,
                tightening,
            )
            if np.any(a_h @ v > b_h + 1.0e-9):
                return False
            x[:2] += dt * x[2:] + 0.5 * dt**2 * v
            x[2:] += dt * v
            if np.any(np.abs(x[:2]) > cfg.position_limit) or np.any(
                np.abs(x[2:]) > cfg.speed_limit
            ):
                return False
        return True

    if feasible(1.0):
        return 1.0
    lo, hi = 0.0, 1.0
    if not feasible(0.0):
        return 0.0
    for _ in range(18):
        mid = 0.5 * (lo + hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


@dataclass
class RunOptions:
    method: str = "proposed"
    tightening: bool = True
    constraint_steps: int | None = None
    preview_mode: str = "rollout"
    map_mode: str = "updated"
    final_projection: bool = True
    cached_torque: bool = False
    smoothing: bool = True
    certificate_constrained: bool = True


def run_case(
    robot: RobotModel,
    controller: FastController,
    scenario: Scenario,
    cfg: BenchmarkConfig,
    options: RunOptions,
) -> dict:
    state = np.array(scenario.initial_state, dtype=float)
    n_steps = int(round(cfg.duration / cfg.fast_dt))
    manager = PredictiveManager(
        cfg,
        robot,
        constraint_steps=options.constraint_steps,
        tightening=options.tightening,
        preview_mode=options.preview_mode,
        map_mode=options.map_mode,
        smoothing=options.smoothing,
        certificate_constrained=options.certificate_constrained,
    )
    held_delta = np.zeros(2)
    held_alpha = 1.0
    held_governed_goal = state[:2].copy()
    held_tau: Array | None = None
    manager_result: ManagerResult | None = None

    log: dict[str, list] = {
        "time": [],
        "state": [],
        "goal": [],
        "force": [],
        "nominal_acceleration": [],
        "requested_acceleration": [],
        "actual_acceleration": [],
        "torque_pre": [],
        "torque_nominal": [],
        "torque_predicted": [],
        "torque_applied": [],
        "torque_limit": [],
        "correction": [],
        "planned_margin": [],
        "planned_violation": [],
        "planned_t2_slack": [],
        "true_margin": [],
        "directional_authority": [],
        "directional_authority_with_disturbance": [],
        "directional_authority_against_disturbance": [],
        "manager_feasible": [],
        "manager_time_s": [],
        "fast_time_s": [],
        "velocity_successor_defect": [],
        "tightening_bound": [],
    }
    previous_slow_state = state.copy()
    previous_slow_v = np.zeros(2)
    previous_slow_time = 0.0
    first_intervention_time: float | None = None
    first_nominal_violation_time: float | None = None

    for k in range(n_steps):
        t = k * cfg.fast_dt
        force = scenario.human_force(t)
        goal = scenario.goal(t)
        fast_started = perf_counter()
        nominal = controller.acceleration(state, force, goal)
        nominal = np.clip(nominal, -cfg.acceleration_limit, cfg.acceleration_limit)
        slow_compute_time = 0.0

        if k % cfg.fast_per_slow == 0:
            if options.method == "proposed":
                manager_result = manager.control(controller, state, scenario, t, force)
                held_delta = manager_result.acceleration - manager_result.nominal_first
                slow_compute_time = manager_result.solve_time_s
                if np.linalg.norm(held_delta) > 1.0e-3 and first_intervention_time is None:
                    first_intervention_time = t
            elif options.method == "reference_governor":
                started = perf_counter()
                held_alpha = governor_scale(
                    controller,
                    robot,
                    state,
                    scenario,
                    t,
                    force,
                    cfg,
                    tightening=options.tightening,
                )
                manager_result = ManagerResult(
                    acceleration=held_alpha * nominal,
                    nominal_first=nominal.copy(),
                    planned_min_margin=float("nan"),
                    planned_max_violation=float("nan"),
                    planned_min_t2_slack=float("nan"),
                    feasible=True,
                    solve_time_s=perf_counter() - started,
                    sequence=np.tile(held_alpha * nominal, (cfg.horizon, 1)),
                )
                slow_compute_time = manager_result.solve_time_s
                held_governed_goal = state[:2] + held_alpha * (goal - state[:2])
                if held_alpha < 0.999 and first_intervention_time is None:
                    first_intervention_time = t

        if options.method == "proposed":
            requested = nominal + held_delta
        elif options.method == "reference_governor":
            requested = controller.acceleration(state, force, held_governed_goal)
            requested = np.clip(
                requested, -cfg.acceleration_limit, cfg.acceleration_limit
            )
            a_torque, b_torque = halfspaces_for_acceleration(
                robot,
                state,
                force,
                t,
                scenario.torque_scale(t),
                scenario.mismatch_scale,
                options.tightening,
            )
            a_state, b_state = reactive_state_halfspaces(state, cfg)
            requested = project_polytope_2d(
                requested,
                np.vstack((a_torque, a_state)),
                np.concatenate((b_torque, b_state)),
            )
        elif options.method == "reactive_filter":
            a_torque, b_torque = halfspaces_for_acceleration(
                robot,
                state,
                force,
                t,
                scenario.torque_scale(t),
                scenario.mismatch_scale,
                options.tightening,
            )
            a_state, b_state = reactive_state_halfspaces(state, cfg)
            requested = project_polytope_2d(
                nominal,
                np.vstack((a_torque, a_state)),
                np.concatenate((b_torque, b_state)),
            )
            if np.linalg.norm(requested - nominal) > 1.0e-3 and first_intervention_time is None:
                first_intervention_time = t
        else:
            requested = nominal.copy()
        requested = np.clip(requested, -cfg.acceleration_limit, cfg.acceleration_limit)

        scale = scenario.torque_scale(t)
        limits = scale * robot.torque_limits
        nominal_tau = robot.true_pre_torque(
            state, nominal, force, t, scenario.mismatch_scale
        )
        if np.any(np.abs(nominal_tau) > limits) and first_nominal_violation_time is None:
            first_nominal_violation_time = t

        tau_pre = robot.true_pre_torque(
            state, requested, force, t, scenario.mismatch_scale
        )
        if options.cached_torque and held_tau is not None and k % cfg.fast_per_slow != 0:
            tau_pre_for_application = held_tau
        else:
            tau_pre_for_application = tau_pre
            if k % cfg.fast_per_slow == 0:
                held_tau = tau_pre.copy()

        if options.method == "nominal_unbounded" or not options.final_projection:
            tau_applied = tau_pre_for_application.copy()
        else:
            tau_applied = np.clip(tau_pre_for_application, -limits, limits)

        actual = robot.actual_acceleration(
            state, tau_applied, force, t, scenario.mismatch_scale
        )
        p_next = state[:2] + cfg.fast_dt * state[2:] + 0.5 * cfg.fast_dt**2 * actual
        v_next = state[2:] + cfg.fast_dt * actual
        next_state = np.concatenate((p_next, v_next))

        # Directional authority in the requested correction direction.  This is
        # computed by ray/halfspace intersection in acceleration coordinates.
        direction = nominal - requested
        if np.linalg.norm(direction) < 1.0e-10:
            direction = -state[2:]
        if np.linalg.norm(direction) < 1.0e-10:
            direction = np.array([1.0, 0.0])
        direction = direction / np.linalg.norm(direction)
        a_h, b_h = halfspaces_for_acceleration(
            robot,
            state,
            force,
            t,
            scale,
            scenario.mismatch_scale,
            options.tightening,
        )
        numer = b_h - a_h @ requested

        def _ray_authority(unit_direction: Array) -> float:
            denom_d = a_h @ unit_direction
            if np.any(numer < -1.0e-10):
                return 0.0
            positive_d = denom_d > 1.0e-10
            value = (
                float(np.min(numer[positive_d] / denom_d[positive_d]))
                if np.any(positive_d)
                else float("inf")
            )
            return max(0.0, value)

        authority = _ray_authority(direction)

        # Fixed-direction authority: unlike `direction` above (the
        # correction direction, which is endogenous and can switch
        # discontinuously as the manager's own output changes), this uses
        # the exogenous disturbance direction itself, so it is comparable
        # across the trajectory and across methods.
        force_norm = np.linalg.norm(force)
        if force_norm > 1.0e-8:
            disturbance_direction = force / force_norm
        else:
            disturbance_direction = np.array([0.0, 1.0])
        authority_with_disturbance = _ray_authority(disturbance_direction)
        authority_against_disturbance = _ray_authority(-disturbance_direction)

        # One-slow-step empirical abstraction defect, evaluated when a new slow
        # sample is reached.  Intermediate samples repeat the last value.
        defect = np.zeros(2)
        if k == 0:
            previous_slow_state = state.copy()
            previous_slow_v = requested.copy()
            previous_slow_time = t
        elif k % cfg.fast_per_slow == 0:
            elapsed = t - previous_slow_time
            abstract_v = previous_slow_state[2:] + elapsed * previous_slow_v
            defect = state[2:] - abstract_v
            previous_slow_state = state.copy()
            previous_slow_v = requested.copy()
            previous_slow_time = t

        delta_bound = robot.torque_error_bound(
            requested, scenario.mismatch_scale, force
        )
        predicted_tau = robot.predicted_torque(
            state, requested, force, t
        )
        true_margin = float(np.min(limits - np.abs(tau_pre)))
        planned_margin = (
            manager_result.planned_min_margin
            if manager_result is not None
            else float("nan")
        )
        planned_violation = (
            manager_result.planned_max_violation
            if manager_result is not None
            else float("nan")
        )
        planned_t2_slack = (
            manager_result.planned_min_t2_slack
            if manager_result is not None
            else float("nan")
        )
        fast_elapsed = max(0.0, perf_counter() - fast_started - slow_compute_time)

        log["time"].append(t)
        log["state"].append(state.copy())
        log["goal"].append(goal.copy())
        log["force"].append(force.copy())
        log["nominal_acceleration"].append(nominal.copy())
        log["requested_acceleration"].append(requested.copy())
        log["actual_acceleration"].append(actual.copy())
        log["torque_pre"].append(tau_pre.copy())
        log["torque_nominal"].append(nominal_tau.copy())
        log["torque_predicted"].append(predicted_tau.copy())
        log["torque_applied"].append(tau_applied.copy())
        log["torque_limit"].append(limits.copy())
        log["correction"].append((requested - nominal).copy())
        log["planned_margin"].append(planned_margin)
        log["planned_violation"].append(planned_violation)
        log["planned_t2_slack"].append(planned_t2_slack)
        log["true_margin"].append(true_margin)
        log["directional_authority"].append(authority)
        log["directional_authority_with_disturbance"].append(
            authority_with_disturbance
        )
        log["directional_authority_against_disturbance"].append(
            authority_against_disturbance
        )
        log["manager_feasible"].append(
            manager_result.feasible if manager_result is not None else True
        )
        log["manager_time_s"].append(
            manager_result.solve_time_s
            if manager_result is not None and k % cfg.fast_per_slow == 0
            else 0.0
        )
        log["fast_time_s"].append(fast_elapsed)
        log["velocity_successor_defect"].append(defect.copy())
        log["tightening_bound"].append(delta_bound.copy())
        state = next_state

    arrays = {key: np.asarray(value) for key, value in log.items()}
    arrays["first_intervention_time"] = first_intervention_time
    arrays["first_nominal_violation_time"] = first_nominal_violation_time
    return arrays


def summarize(log: dict, cfg: BenchmarkConfig) -> dict:
    state = log["state"]
    torque_pre = log["torque_pre"]
    torque_nominal = log["torque_nominal"]
    torque_predicted = log["torque_predicted"]
    torque_applied = log["torque_applied"]
    torque_limit = log["torque_limit"]
    correction = log["correction"]
    nominal = log["nominal_acceleration"]
    requested = log["requested_acceleration"]
    actual = log["actual_acceleration"]
    defects = log["velocity_successor_defect"]
    manager_times = log["manager_time_s"]
    manager_times = manager_times[manager_times > 0.0]
    fast_times = log["fast_time_s"]
    torque_excess = np.maximum(np.abs(torque_pre) - torque_limit, 0.0)
    nominal_torque_excess = np.maximum(
        np.abs(torque_nominal) - torque_limit, 0.0
    )
    applied_excess = np.maximum(np.abs(torque_applied) - torque_limit, 0.0)
    position_excess = np.maximum(np.abs(state[:, :2]) - cfg.position_limit, 0.0)
    speed_excess = np.maximum(np.abs(state[:, 2:]) - cfg.speed_limit, 0.0)
    realization = actual - nominal
    correction_rmse = float(np.sqrt(np.mean(correction**2)))
    # "behavior_realization_rmse_mps2" below is actual - nominal, which
    # bundles the manager's *deliberate* intervention (requested - nominal,
    # already reported as correction_rmse_mps2) together with whatever the
    # real robot does differently from what was actually requested
    # (actual - requested). The second piece is the one most directly tied
    # to certificate transfer (it is the torque/velocity-domain mismatch
    # T1/T3 bound, not a chosen action), so it is reported separately here.
    tracking_defect = actual - requested
    tracking_defect_rmse = float(np.sqrt(np.mean(tracking_defect**2)))
    # Theorem 1's (T3) is stated in the infinity norm; the Euclidean (L2)
    # norm reported alongside it is a stricter, conservative check (L2 >=
    # Linf for any vector), kept as the audit's pass/fail criterion because
    # it was the original metric, but it should not be read as the same
    # quantity the theorem's norm names.
    empirical_defect = float(np.max(np.linalg.norm(defects, axis=1)))
    empirical_defect_linf = float(np.max(np.abs(defects)))
    torque_error = np.abs(torque_pre - torque_predicted)
    torque_bound_residual = log["tightening_bound"] - torque_error
    first_i = log["first_intervention_time"]
    first_v = log["first_nominal_violation_time"]
    warning = (
        float(first_v - first_i)
        if first_i is not None and first_v is not None
        else 0.0
    )
    planned = log["planned_margin"]
    finite_planned = planned[np.isfinite(planned)]
    planned_violation = log["planned_violation"]
    finite_planned_violation = planned_violation[
        np.isfinite(planned_violation)
    ]
    planned_t2_slack = log["planned_t2_slack"]
    finite_planned_t2_slack = planned_t2_slack[np.isfinite(planned_t2_slack)]
    state_satisfied = bool(
        np.max(position_excess) <= 5.0e-4
        and np.max(speed_excess) <= 5.0e-4
    )
    applied_torque_satisfied = bool(np.max(applied_excess) <= 1.0e-9)
    realizability_margin_satisfied = bool(
        np.max(torque_excess) <= 1.0e-9
    )
    defect_satisfied = bool(
        empirical_defect <= cfg.velocity_defect_radius
    )
    torque_bound_satisfied = bool(
        np.min(torque_bound_residual) >= -1.0e-10
    )
    manager_satisfied = bool(np.mean(log["manager_feasible"]) >= 0.999)
    feasible_mask = np.asarray(log["manager_feasible"], dtype=bool)
    slow_modes = feasible_mask[:: cfg.fast_per_slow]
    mode_switches = int(np.sum(slow_modes[1:] != slow_modes[:-1]))

    def masked_peak(values: Array, mask: Array) -> float | None:
        return float(np.max(values[mask])) if np.any(mask) else None

    return {
        "peak_nominal_torque_violation_Nm": float(
            np.max(nominal_torque_excess)
        ),
        "peak_preclip_torque_violation_Nm": float(np.max(torque_excess)),
        "integrated_preclip_torque_violation_Nms": float(
            np.sum(torque_excess) * cfg.fast_dt
        ),
        "peak_applied_torque_violation_Nm": float(np.max(applied_excess)),
        "peak_position_violation_m": float(np.max(position_excess)),
        "peak_speed_violation_mps": float(np.max(speed_excess)),
        "peak_speed_mps": float(np.max(np.abs(state[:, 2:]))),
        "behavior_realization_rmse_mps2": float(np.sqrt(np.mean(realization**2))),
        "correction_rmse_mps2": correction_rmse,
        "tracking_defect_rmse_mps2": tracking_defect_rmse,
        "intervention_fraction": float(
            np.mean(np.linalg.norm(correction, axis=1) > 1.0e-3)
        ),
        "warning_lead_time_s": warning,
        # Raw timestamps, exposed so a shared reference event (e.g. from a
        # single "nominal_unbounded" run) can be substituted for
        # first_nominal_violation_time when comparing lead time across
        # methods -- each method's own first_nominal_violation_time is
        # evaluated on that method's own (already-corrected) state
        # trajectory, so it is not the same event across methods and
        # warning_lead_time_s above is not directly comparable between them.
        "first_intervention_time_s": first_i,
        "first_nominal_violation_time_s": first_v,
        "minimum_directional_authority_mps2": float(
            np.min(log["directional_authority"])
        ),
        "minimum_directional_authority_with_disturbance_mps2": float(
            np.min(log["directional_authority_with_disturbance"])
        ),
        "minimum_directional_authority_against_disturbance_mps2": float(
            np.min(log["directional_authority_against_disturbance"])
        ),
        "minimum_true_torque_margin_Nm": float(np.min(log["true_margin"])),
        "minimum_planned_torque_margin_Nm": (
            float(np.min(finite_planned)) if finite_planned.size else None
        ),
        "minimum_T2_slack_Nm": (
            float(np.min(finite_planned_t2_slack))
            if finite_planned_t2_slack.size
            else None
        ),
        "maximum_planned_torque_violation_Nm": (
            float(np.max(finite_planned_violation))
            if finite_planned_violation.size
            else None
        ),
        "manager_feasibility_rate": float(np.mean(log["manager_feasible"])),
        "fallback_fraction": float(1.0 - np.mean(log["manager_feasible"])),
        "manager_mode_switches": mode_switches,
        "peak_preclip_torque_violation_mpc_mode_Nm": masked_peak(
            torque_excess, feasible_mask
        ),
        "peak_preclip_torque_violation_fallback_mode_Nm": masked_peak(
            torque_excess, ~feasible_mask
        ),
        "peak_position_violation_mpc_mode_m": masked_peak(
            position_excess, feasible_mask
        ),
        "peak_position_violation_fallback_mode_m": masked_peak(
            position_excess, ~feasible_mask
        ),
        "velocity_successor_defect_peak_mps": empirical_defect,
        "velocity_successor_defect_peak_linf_mps": empirical_defect_linf,
        "velocity_defect_radius_mps": cfg.velocity_defect_radius,
        "sampled_velocity_defect_check_satisfied": defect_satisfied,
        "sampled_state_check_satisfied": state_satisfied,
        "realizability_margin_condition_satisfied": (
            realizability_margin_satisfied
        ),
        "minimum_error_bound_residual_Nm": float(
            np.min(torque_bound_residual)
        ),
        "torque_error_bound_min_Nm": float(
            np.min(log["tightening_bound"])
        ),
        "torque_error_bound_max_Nm": float(
            np.max(log["tightening_bound"])
        ),
        "torque_error_bound_satisfied": torque_bound_satisfied,
        "sampled_interface_audit_passed": bool(
            defect_satisfied
            and state_satisfied
            and torque_bound_satisfied
            and realizability_margin_satisfied
            and applied_torque_satisfied
            and manager_satisfied
        ),
        "fast_time_us": {
            "median": float(np.median(fast_times) * 1.0e6),
            "p95": float(np.percentile(fast_times, 95) * 1.0e6),
            "p99": float(np.percentile(fast_times, 99) * 1.0e6),
            "max": float(np.max(fast_times) * 1.0e6),
        },
        "manager_time_ms": {
            "median": float(np.median(manager_times) * 1.0e3) if manager_times.size else 0.0,
            "p95": float(np.percentile(manager_times, 95) * 1.0e3) if manager_times.size else 0.0,
            "p99": float(np.percentile(manager_times, 99) * 1.0e3) if manager_times.size else 0.0,
            "max": float(np.max(manager_times) * 1.0e3) if manager_times.size else 0.0,
        },
    }


def serializable_config(cfg: BenchmarkConfig) -> dict:
    return asdict(cfg)
