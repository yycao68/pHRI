#!/usr/bin/env python3
"""Torque-controlled 7-DoF FR3 benchmark for residual-wrench authorization.

Controllers share the same impedance nominal, residual MPC proposal, torque
limits, force estimator, and MuJoCo plant.  The external baseline implements the
impedance-causal series Passivity Observer/Passivity Controller of Hannaford and
Ryu (TRA 2002, eqs. 13--19), generalized to a 3-D translational port.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_impedance_fr3_mpl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import osqp
from scipy import linalg, sparse, stats


HERE = Path(__file__).resolve().parent
# This script lives in pHRI/impedance/simulation/; the shared FR3/MuJoCo
# utilities (fr3_mujoco.py, so3_utils.py) live in the repo-level
# pHRI/simulation/ -- two directories up, not one.
SIM = HERE.parent.parent / "simulation"
sys.path.insert(0, str(SIM))

from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL, TAU_LIMIT  # noqa: E402
from so3_utils import rotation_error_matrix  # noqa: E402


@dataclass(frozen=True)
class Config:
    duration: float = 4.0
    dt: float = 0.001
    manager_dt: float = 0.02
    horizon: int = 10
    stiffness: float = 180.0
    damping: float = 28.0
    rotation_stiffness: float = 18.0
    rotation_damping: float = 5.0
    null_stiffness: float = 10.0
    null_damping: float = 2.0
    q_position: float = 1.6e4
    q_velocity: float = 80.0
    r_wrench: float = 0.12
    tank_initial: float = 0.08
    tank_minimum: float = 0.02
    tank_maximum: float = 0.30
    # A derated 28% continuous envelope creates actuator-allocation pressure
    # stress while remaining inside the model's absolute FR3 safety limits.
    torque_margin: float = 0.28
    force_limit: float = 25.0
    wall_location: float = 0.035

    @property
    def manager_steps(self) -> int:
        return int(round(self.manager_dt / self.dt))


def intentional_force(t: float) -> np.ndarray:
    f = np.zeros(3)
    if 0.45 <= t < 1.65:
        phase = min(1.0, (t - 0.45) / 0.15, (1.65 - t) / 0.15)
        f[0] = 8.0 * 0.5 * (1.0 - np.cos(np.pi * max(0.0, phase)))
    if 2.25 <= t < 3.30:
        phase = min(1.0, (t - 2.25) / 0.15, (3.30 - t) / 0.15)
        f[2] = -5.0 * 0.5 * (1.0 - np.cos(np.pi * max(0.0, phase)))
    return f


def rejectable_force(t: float, phases: np.ndarray, scale: float) -> np.ndarray:
    f = np.array([
        1.1 * np.sin(2 * np.pi * 0.9 * t + phases[0]),
        0.6 * np.sin(2 * np.pi * 1.4 * t + phases[1]),
        0.9 * np.sin(2 * np.pi * 1.9 * t + phases[2]),
    ])
    if 1.507 <= t < 1.514:  # deliberately between 50 Hz manager ticks
        f += np.array([0.0, 0.0, 12.0])
    return scale * f


def wall_force(displacement: np.ndarray, velocity: np.ndarray, stiffness: float, damping: float) -> np.ndarray:
    penetration = max(0.0, displacement[0] - Config.wall_location)
    result = np.zeros(3)
    if penetration > 0.0:
        result[0] = -stiffness * penetration - damping * velocity[0]
    return result


def zoh(a: np.ndarray, b: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    n, m = b.shape
    block = np.zeros((n + m, n + m))
    block[:n, :n] = a
    block[:n, n:] = b
    out = linalg.expm(block * dt)
    return out[:n, :n], out[:n, n:]


def prediction(a: np.ndarray, b: np.ndarray, horizon: int) -> tuple[np.ndarray, np.ndarray]:
    nx, nu = b.shape
    phi = np.zeros((horizon * nx, nx))
    gamma = np.zeros((horizon * nx, horizon * nu))
    for i in range(horizon):
        phi[i * nx : (i + 1) * nx] = np.linalg.matrix_power(a, i + 1)
        for j in range(i + 1):
            gamma[i * nx : (i + 1) * nx, j * nu : (j + 1) * nu] = np.linalg.matrix_power(a, i - j) @ b
    return phi, gamma


class ResidualMPC3D:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.solve_ms: list[float] = []
        self.failures = 0

    def control(
        self,
        residual: np.ndarray,
        lambda_inv: np.ndarray,
        disturbance_hat: np.ndarray,
        jacobian: np.ndarray,
        tau_nominal: np.ndarray,
    ) -> np.ndarray:
        cfg = self.cfg
        k = cfg.stiffness * np.eye(3)
        d = cfg.damping * np.eye(3)
        ac = np.block([[np.zeros((3, 3)), np.eye(3)], [-lambda_inv @ k, -lambda_inv @ d]])
        bc = np.vstack([np.zeros((3, 3)), lambda_inv])
        ad, bd = zoh(ac, bc, cfg.manager_dt)
        phi, gamma = prediction(ad, bd, cfg.horizon)
        dseq = np.tile(disturbance_hat, cfg.horizon)
        xfree = phi @ residual + gamma @ dseq
        q = np.diag([cfg.q_position] * 3 + [cfg.q_velocity] * 3)
        qbar = sparse.block_diag([q] * cfg.horizon, format="csc")
        rbar = sparse.eye(3 * cfg.horizon, format="csc") * cfg.r_wrench
        hess = np.asarray(gamma.T @ qbar @ gamma + rbar)
        linear = np.asarray(gamma.T @ qbar @ xfree).reshape(-1)

        # Same current J and nominal torque over the short horizon.  The fast
        # layer remains authoritative for the realized nonlinear plant.
        a_tau = sparse.vstack([
            sparse.kron(sparse.eye(cfg.horizon), sparse.csc_matrix(jacobian.T)),
            sparse.eye(3 * cfg.horizon),
        ], format="csc")
        tau_cap = cfg.torque_margin * TAU_LIMIT
        lower_tau = np.tile(-tau_cap - tau_nominal, cfg.horizon)
        upper_tau = np.tile(tau_cap - tau_nominal, cfg.horizon)
        lower = np.concatenate([lower_tau, -np.ones(3 * cfg.horizon) * cfg.force_limit])
        upper = np.concatenate([upper_tau, np.ones(3 * cfg.horizon) * cfg.force_limit])
        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix((hess + hess.T) / 2), q=linear,
                     A=a_tau, l=lower, u=upper, eps_abs=1e-5, eps_rel=1e-5,
                     max_iter=8000, polishing=False, verbose=False)
        start = time.perf_counter_ns()
        answer = solver.solve(raise_error=False)
        self.solve_ms.append((time.perf_counter_ns() - start) * 1e-6)
        if answer.info.status_val not in (1, 2):
            self.failures += 1
            return np.zeros(3)
        return np.asarray(answer.x[:3])


def torque_scale(tau_nominal: np.ndarray, tau_residual: np.ndarray, limits: np.ndarray) -> tuple[float, bool]:
    if np.any(np.abs(tau_nominal) > limits + 1e-10):
        return 0.0, False
    alpha = 1.0
    for nominal, residual, limit in zip(tau_nominal, tau_residual, limits):
        if residual > 1e-14:
            alpha = min(alpha, (limit - nominal) / residual)
        elif residual < -1e-14:
            alpha = min(alpha, (-limit - nominal) / residual)
    return float(np.clip(alpha, 0.0, 1.0)), True


def nominal_torque(dyn, state, p0: np.ndarray, r0: np.ndarray, cfg: Config) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    jv, jw = dyn.J[:3], dyn.J[3:]
    displacement = state.ee_pos - p0
    f_imp = -cfg.stiffness * displacement - cfg.damping * state.ee_vel[:3]
    e_rot = rotation_error_matrix(r0, state.ee_rot)
    f_rot = -cfg.rotation_stiffness * e_rot - cfg.rotation_damping * state.ee_vel[3:]
    m_inv = np.linalg.inv(dyn.M)
    lam_inv = jv @ m_inv @ jv.T + 1e-6 * np.eye(3)
    jbar = m_inv @ jv.T @ np.linalg.inv(lam_inv)
    nbar = np.eye(7) - jbar @ jv
    tau_null = nbar.T @ (-cfg.null_stiffness * (state.q - Q_NEUTRAL) - cfg.null_damping * state.dq)
    tau = dyn.Cq_dot + jv.T @ f_imp + jw.T @ f_rot + tau_null
    return tau, jv, lam_inv


def run_trial(
    controller: str,
    cfg: Config,
    seed: int,
    leakage: float = 0.0,
    wall_stiffness: float = 900.0,
    wall_damping: float = 12.0,
    disturbance_scale: float = 1.0,
    sensor_noise: float = 0.0,
    estimate_delay_ticks: int = 0,
    noise_ar1: float = 0.0,
    velocity_bias: np.ndarray | None = None,
) -> dict:
    if controller not in {"impedance", "unguarded_mpc", "tdpc", "two_rate"}:
        raise ValueError(controller)
    rng = np.random.default_rng(seed)
    phases = rng.uniform(-np.pi, np.pi, 3)
    env = FR3MuJoCoEnv(timestep=cfg.dt)
    env.reset()
    dyn0, state0 = env.get_dynamics_and_state()
    p0, r0 = state0.ee_pos.copy(), state0.ee_rot.copy()
    reference = np.zeros(6)
    mpc = ResidualMPC3D(cfg)
    raw_residual = np.zeros(3)
    tank = cfg.tank_initial
    po_energy = 0.0
    # Manager-tick history of the raw force channels, for the optional
    # estimator communication/processing delay below; and a persistent AR(1)
    # state for optionally-colored sensor noise (same stationary std as the
    # white-noise case, so only the correlation structure is being tested).
    manager_force_history: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    noise_state = np.zeros(3)
    steps = int(round(cfg.duration / cfg.dt))
    log = {key: np.zeros((steps, 3)) for key in (
        "position", "velocity", "reference_position", "residual_position", "intentional",
        "rejectable", "wall", "raw_residual", "applied_residual",
    )}
    log.update({key: np.zeros(steps) for key in (
        "time", "tank", "po_energy", "alpha", "torque_ratio", "nominal_feasible",
        "energy_balance", "authorization_active",
    )})
    supply = 0.0
    initial_storage = cfg.tank_initial

    for index in range(steps):
        t = env.time
        dyn, state = env.get_dynamics_and_state()
        displacement = state.ee_pos - p0
        velocity = state.ee_vel[:3]
        f_int = intentional_force(t)
        f_dist = rejectable_force(t, phases, disturbance_scale)
        f_wall = wall_force(displacement, velocity, wall_stiffness, wall_damping)
        tau_nom, jv, lam_inv = nominal_torque(dyn, state, p0, r0, cfg)
        residual = np.concatenate([displacement - reference[:3], velocity - reference[3:]])

        if index % cfg.manager_steps == 0:
            manager_force_history.append((f_dist.copy(), f_wall.copy(), f_int.copy()))
            if controller == "impedance":
                raw_residual[:] = 0.0
            else:
                if noise_ar1 > 0.0:
                    eps = rng.normal(0.0, sensor_noise, 3)
                    noise_state = noise_ar1 * noise_state + np.sqrt(max(0.0, 1.0 - noise_ar1 ** 2)) * eps
                    noise = noise_state
                else:
                    noise = rng.normal(0.0, sensor_noise, 3)
                lookup = -1 - estimate_delay_ticks
                if -lookup <= len(manager_force_history):
                    f_dist_hat, f_wall_hat, f_int_hat = manager_force_history[lookup]
                else:
                    # Not enough history yet (startup transient): fall back to
                    # the current sample rather than an undefined index.
                    f_dist_hat, f_wall_hat, f_int_hat = f_dist, f_wall, f_int
                disturbance_hat = f_dist_hat + f_wall_hat + leakage * f_int_hat + noise
                controller_residual = residual
                if velocity_bias is not None:
                    controller_residual = residual.copy()
                    controller_residual[3:] += velocity_bias
                raw_residual = mpc.control(controller_residual, lam_inv, disturbance_hat, jv, tau_nom)

        tau_r_raw = jv.T @ raw_residual
        torque_alpha, nominal_ok = torque_scale(tau_nom, tau_r_raw, cfg.torque_margin * TAU_LIMIT)
        candidate = torque_alpha * raw_residual
        alpha = torque_alpha
        authorization_active = torque_alpha < 1.0 - 1e-10

        if controller == "two_rate":
            power = float(candidate @ velocity)
            dissipation = cfg.damping * float(velocity @ velocity)
            available = max(0.0, tank - cfg.tank_minimum + cfg.dt * dissipation)
            energy_alpha = 1.0 if power <= 0.0 else min(1.0, available / (cfg.dt * power + 1e-15))
            alpha *= energy_alpha
            applied_residual = energy_alpha * candidate
            authorization_active = authorization_active or energy_alpha < 1.0 - 1e-10
        elif controller == "tdpc":
            # Hannaford--Ryu impedance-causal series PC: if the next PO value
            # would be negative, add exactly the damping needed to restore it.
            predicted_po = po_energy - cfg.dt * float(candidate @ velocity)
            velocity_sq = float(velocity @ velocity)
            damping_gain = 0.0
            if predicted_po < 0.0 and velocity_sq > 1e-14:
                damping_gain = -predicted_po / (cfg.dt * velocity_sq)
            tdpc_force = candidate - damping_gain * velocity
            tdpc_tau = jv.T @ tdpc_force
            tdpc_scale, nominal_ok_2 = torque_scale(tau_nom, tdpc_tau, cfg.torque_margin * TAU_LIMIT)
            nominal_ok = nominal_ok and nominal_ok_2
            applied_residual = tdpc_scale * tdpc_force
            alpha = torque_alpha * tdpc_scale
            authorization_active = authorization_active or damping_gain > 0.0 or tdpc_scale < 1.0 - 1e-10
            po_energy -= cfg.dt * float(applied_residual @ velocity)
        else:
            applied_residual = candidate

        tau = tau_nom + jv.T @ applied_residual
        # The projection should make this clip a no-op.  Retain it as the plant
        # safety backstop and log any disagreement through torque_ratio.
        torque_ratio = float(np.max(np.abs(tau) / (cfg.torque_margin * TAU_LIMIT)))
        env.apply_torque(tau)
        total_external = f_int + f_dist + f_wall
        env.apply_ee_wrench(np.concatenate([total_external, np.zeros(3)]))

        dissipation = cfg.damping * float(velocity @ velocity)
        if controller == "two_rate":
            tank = min(cfg.tank_maximum, tank + cfg.dt * (dissipation - float(applied_residual @ velocity)))
        elif controller in {"unguarded_mpc", "tdpc"}:
            # Counterfactual common ledger for direct energy-use comparison.
            tank = min(cfg.tank_maximum, tank + cfg.dt * (dissipation - float(applied_residual @ velocity)))

        kinetic = 0.5 * float(state.dq @ dyn.M @ state.dq)
        potential = 0.5 * cfg.stiffness * float(displacement @ displacement)
        supply += cfg.dt * float(total_external @ velocity)
        energy_balance = kinetic + potential + tank - initial_storage - supply

        for key, value in (
            ("position", displacement), ("velocity", velocity),
            ("reference_position", reference[:3]), ("residual_position", residual[:3]),
            ("intentional", f_int), ("rejectable", f_dist), ("wall", f_wall),
            ("raw_residual", raw_residual), ("applied_residual", applied_residual),
        ):
            log[key][index] = value
        for key, value in (
            ("time", t), ("tank", tank), ("po_energy", po_energy), ("alpha", alpha),
            ("torque_ratio", torque_ratio), ("nominal_feasible", float(nominal_ok)),
            ("energy_balance", energy_balance), ("authorization_active", float(authorization_active)),
        ):
            log[key][index] = value

        # Analytical intentional-motion reference; Lambda is the current
        # operational inertia of the same FR3, but this reference never drives torque.
        reference_acc = lam_inv @ (f_int - cfg.damping * reference[3:] - cfg.stiffness * reference[:3])
        reference[:3] += cfg.dt * reference[3:]
        reference[3:] += cfg.dt * reference_acc
        env.step()

    use = log["time"] >= 0.25
    residual_norm = np.linalg.norm(log["residual_position"], axis=1)
    metrics = {
        "residual_rms_mm": float(1e3 * np.sqrt(np.mean(residual_norm[use] ** 2))),
        "residual_peak_mm": float(1e3 * np.max(residual_norm[use])),
        "minimum_tank_j": float(np.min(log["tank"])),
        "tank_violation_j": float(max(0.0, cfg.tank_minimum - np.min(log["tank"]))),
        "minimum_po_energy_j": float(np.min(log["po_energy"])),
        "projection_active_fraction": float(np.mean(log["authorization_active"] > 0.5)),
        "maximum_torque_ratio": float(np.max(log["torque_ratio"])),
        "nominal_infeasible_samples": int(np.sum(log["nominal_feasible"] < 0.5)),
        "maximum_absolute_energy_balance_residual_j": float(np.max(np.abs(log["energy_balance"]))),
        "qp_failures": int(mpc.failures),
        "solve_ms_mean": float(np.mean(mpc.solve_ms)) if mpc.solve_ms else 0.0,
        "solve_ms_p95": float(np.percentile(mpc.solve_ms, 95)) if mpc.solve_ms else 0.0,
        "solve_ms_max": float(np.max(mpc.solve_ms)) if mpc.solve_ms else 0.0,
    }
    return {"metrics": metrics, "log": {key: value.tolist() for key, value in log.items()}}


def summarize(raw: dict[str, list[dict]]) -> dict:
    keys = ("residual_rms_mm", "residual_peak_mm", "minimum_tank_j", "tank_violation_j",
            "minimum_po_energy_j", "projection_active_fraction", "maximum_torque_ratio",
            "nominal_infeasible_samples", "maximum_absolute_energy_balance_residual_j", "qp_failures",
            "solve_ms_mean", "solve_ms_p95", "solve_ms_max")
    result = {}
    for controller, rows in raw.items():
        result[controller] = {}
        for key in keys:
            values = np.asarray([row[key] for row in rows], dtype=float)
            result[controller][key] = {"mean": float(values.mean()),
                                        "std": float(values.std(ddof=1)) if len(values) > 1 else 0.0,
                                        "min": float(values.min()), "max": float(values.max())}
    return result


def run_benchmark(cfg: Config, seeds: int) -> dict:
    controllers = ("impedance", "unguarded_mpc", "tdpc", "two_rate")
    rng = np.random.default_rng(20260802)
    raw = {name: [] for name in controllers}
    trials = []
    for seed in range(seeds):
        parameters = {
            "wall_stiffness": float(rng.uniform(500.0, 1500.0)),
            "wall_damping": float(rng.uniform(8.0, 20.0)),
            "disturbance_scale": float(rng.uniform(0.8, 1.2)),
        }
        trials.append({"seed": seed, **parameters})
        for name in controllers:
            trial = run_trial(name, cfg, seed, **parameters)
            raw[name].append({"seed": seed, **parameters, **trial["metrics"]})
    fast = np.asarray([row["residual_rms_mm"] for row in raw["two_rate"]])
    passive = np.asarray([row["residual_rms_mm"] for row in raw["impedance"]])
    diff = fast - passive
    half = stats.t.ppf(0.975, seeds - 1) * stats.sem(diff) if seeds > 1 else 0.0
    comparison = {"paired_difference_mm": float(diff.mean()),
                  "paired_95_percent_ci_mm": [float(diff.mean() - half), float(diff.mean() + half)],
                  "paired_t_p_value": float(stats.ttest_rel(fast, passive).pvalue) if seeds > 1 else 1.0,
                  "relative_change_percent": float(100 * diff.mean() / passive.mean())}
    return {"seeds": seeds, "trials": trials, "summary": summarize(raw),
            "comparison": comparison, "raw": raw}


def leakage_sweep(cfg: Config, seeds: int) -> dict:
    rows = []
    for leakage in (0.0, 0.1, 0.25, 0.5):
        values = []
        for seed in range(seeds):
            result = run_trial("two_rate", cfg, 100 + seed, leakage=leakage,
                               wall_stiffness=0.0, wall_damping=0.0,
                               disturbance_scale=0.0, sensor_noise=0.05)
            log = result["log"]
            t = np.asarray(log["time"])
            x = np.asarray(log["position"])
            ref = np.asarray(log["reference_position"])
            intentional_window = (t >= 0.60) & (t < 1.50)
            metrics = dict(result["metrics"])
            metrics["intentional_axis_error_rms_mm"] = float(
                1e3 * np.sqrt(np.mean((x[intentional_window, 0] - ref[intentional_window, 0]) ** 2))
            )
            metrics["intentional_response_ratio"] = float(
                np.mean(x[intentional_window, 0]) / np.mean(ref[intentional_window, 0])
            )
            values.append(metrics)
        rows.append({"leakage": leakage,
                     "intentional_axis_error_rms_mm_mean": float(np.mean([v["intentional_axis_error_rms_mm"] for v in values])),
                     "intentional_axis_error_rms_mm_std": float(np.std([v["intentional_axis_error_rms_mm"] for v in values], ddof=1)) if seeds > 1 else 0.0,
                     "intentional_response_ratio_mean": float(np.mean([v["intentional_response_ratio"] for v in values])),
                     "minimum_tank_j_min": float(np.min([v["minimum_tank_j"] for v in values])),
                     "maximum_torque_ratio": float(np.max([v["maximum_torque_ratio"] for v in values])),
                     "qp_failures": int(np.sum([v["qp_failures"] for v in values]))})
    return {"seeds_per_level": seeds, "rows": rows}


# One manager tick (20 ms) of estimator/communication delay, AR(1) sensor
# noise with the same stationary std as the white-noise baseline (isolating
# correlation, not magnitude), and a 5 mm/s constant velocity-estimate bias
# along the intentional-force axis -- the three axes Limitation 4 disclosed
# as untested, run individually and combined at a fixed mid-range leakage.
ESTIMATE_DELAY_TICKS = 1
NOISE_AR1 = 0.9
VELOCITY_BIAS = np.array([0.005, 0.0, 0.0])


def sensing_realism_sweep(cfg: Config, seeds: int, leakage: float = 0.25) -> dict:
    conditions = [
        ("baseline", {}),
        ("delay only", {"estimate_delay_ticks": ESTIMATE_DELAY_TICKS}),
        ("colored noise only", {"noise_ar1": NOISE_AR1}),
        ("velocity bias only", {"velocity_bias": VELOCITY_BIAS}),
        ("all combined", {"estimate_delay_ticks": ESTIMATE_DELAY_TICKS,
                          "noise_ar1": NOISE_AR1, "velocity_bias": VELOCITY_BIAS}),
    ]
    rows = []
    for name, extra in conditions:
        values = []
        for seed in range(seeds):
            result = run_trial("two_rate", cfg, 200 + seed, leakage=leakage,
                               wall_stiffness=0.0, wall_damping=0.0,
                               disturbance_scale=0.0, sensor_noise=0.05, **extra)
            log = result["log"]
            t = np.asarray(log["time"])
            x = np.asarray(log["position"])
            ref = np.asarray(log["reference_position"])
            intentional_window = (t >= 0.60) & (t < 1.50)
            metrics = dict(result["metrics"])
            metrics["intentional_axis_error_rms_mm"] = float(
                1e3 * np.sqrt(np.mean((x[intentional_window, 0] - ref[intentional_window, 0]) ** 2))
            )
            metrics["intentional_response_ratio"] = float(
                np.mean(x[intentional_window, 0]) / np.mean(ref[intentional_window, 0])
            )
            values.append(metrics)
        rows.append({"condition": name,
                     "intentional_axis_error_rms_mm_mean": float(np.mean([v["intentional_axis_error_rms_mm"] for v in values])),
                     "intentional_axis_error_rms_mm_std": float(np.std([v["intentional_axis_error_rms_mm"] for v in values], ddof=1)) if seeds > 1 else 0.0,
                     "intentional_response_ratio_mean": float(np.mean([v["intentional_response_ratio"] for v in values])),
                     "minimum_tank_j_min": float(np.min([v["minimum_tank_j"] for v in values])),
                     "maximum_torque_ratio": float(np.max([v["maximum_torque_ratio"] for v in values])),
                     "qp_failures": int(np.sum([v["qp_failures"] for v in values]))})
    return {"leakage": leakage, "seeds_per_level": seeds, "rows": rows}


def plot_results(results: dict, path: Path) -> None:
    representative = results["representative"]
    colors = {"impedance": "#777777", "unguarded_mpc": "#d95f02",
              "tdpc": "#7570b3", "two_rate": "#1b9e77"}
    labels = {"impedance": "Passive impedance", "unguarded_mpc": "Unguarded MPC",
              "tdpc": "Hannaford--Ryu PO/PC", "two_rate": "Two-rate tank"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), constrained_layout=True)
    for name, result in representative.items():
        log = result["log"]
        t = np.asarray(log["time"])
        residual = 1e3 * np.linalg.norm(np.asarray(log["residual_position"]), axis=1)
        axes[0, 0].plot(t, residual, color=colors[name], lw=1.0, label=labels[name])
        axes[0, 1].plot(t, np.asarray(log["tank"]), color=colors[name], lw=1.0, label=labels[name])
        axes[1, 0].plot(t, np.asarray(log["torque_ratio"]), color=colors[name], lw=1.0, label=labels[name])
    axes[0, 0].set_ylabel("3-D residual norm (mm)")
    axes[0, 1].set_ylabel("common energy ledger (J)")
    axes[0, 1].axhline(Config.tank_minimum, color="k", ls=":", lw=0.8)
    axes[1, 0].set_ylabel("max joint torque / limit")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].axhline(1.0, color="k", ls=":", lw=0.8)
    axes[0, 0].legend(fontsize=8, ncol=2)
    for ax in axes.flat[:3]:
        ax.grid(alpha=0.25)
    leak = results["leakage_sweep"]["rows"]
    axes[1, 1].errorbar([100 * row["leakage"] for row in leak],
                        [row["intentional_axis_error_rms_mm_mean"] for row in leak],
                        yerr=[row["intentional_axis_error_rms_mm_std"] for row in leak],
                        marker="o", capsize=3, color=colors["two_rate"])
    axes[1, 1].set_xlabel("intentional-force leakage into disturbance estimate (%)")
    axes[1, 1].set_ylabel("intentional-axis fidelity error (mm)")
    axes[1, 1].set_title("Estimator leakage sensitivity")
    axes[1, 1].grid(alpha=0.25)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--leakage-seeds", type=int, default=5)
    parser.add_argument("--realism-seeds", type=int, default=5)
    parser.add_argument("--output", type=Path, default=HERE / "fr3_two_rate_results.json")
    parser.add_argument("--figure", type=Path, default=HERE / "fr3_two_rate_results.png")
    args = parser.parse_args()
    cfg = Config()
    representative = {name: run_trial(name, cfg, seed=4) for name in
                      ("impedance", "unguarded_mpc", "tdpc", "two_rate")}
    results = {"protocol": asdict(cfg), "mujoco_version": mujoco.__version__,
               "representative": representative,
               "benchmark": run_benchmark(cfg, args.seeds),
               "leakage_sweep": leakage_sweep(cfg, args.leakage_seeds),
               "sensing_realism_sweep": sensing_realism_sweep(cfg, args.realism_seeds)}
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    plot_results(results, args.figure)
    print(json.dumps({"output": str(args.output), "figure": str(args.figure),
                      "representative": {k: v["metrics"] for k, v in representative.items()},
                      "benchmark": results["benchmark"]["summary"],
                      "comparison": results["benchmark"]["comparison"],
                      "leakage_sweep": results["leakage_sweep"],
                      "sensing_realism_sweep": results["sensing_realism_sweep"]}, indent=2))


if __name__ == "__main__":
    main()
