#!/usr/bin/env python3
"""Two-rate benchmark for impedance-causal nominal control and residual MPC.

The benchmark is intentionally one-dimensional and fully auditable.  A passive
motion-to-wrench impedance is the physical nominal controller.  MPC supplies an
additive residual wrench at 50 Hz.  A 1 kHz projection jointly enforces the
actuator limit and a discrete energy-tank lower bound.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import osqp
from scipy import linalg, sparse, stats


HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Config:
    duration: float = 8.0
    dt: float = 0.001
    manager_dt: float = 0.02
    horizon: int = 15
    mass: float = 4.0
    damping: float = 14.0
    stiffness: float = 90.0
    force_limit: float = 12.0
    tank_initial: float = 0.026
    tank_minimum: float = 0.02
    tank_maximum: float = 0.10
    q_position: float = 2.5e4
    q_velocity: float = 120.0
    r_force: float = 0.08
    observer_alpha: float = 0.18
    observer_limit: float = 10.0

    @property
    def manager_steps(self) -> int:
        return int(round(self.manager_dt / self.dt))


def discretize_impedance(cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    ac = np.array([[0.0, 1.0], [-cfg.stiffness / cfg.mass, -cfg.damping / cfg.mass]])
    bc = np.array([[0.0], [1.0 / cfg.mass]])
    block = np.zeros((3, 3))
    block[:2, :2] = ac
    block[:2, 2:] = bc
    expm = linalg.expm(block * cfg.manager_dt)
    return expm[:2, :2], expm[:2, 2:]


def prediction_matrices(a: np.ndarray, b: np.ndarray, horizon: int) -> tuple[np.ndarray, np.ndarray]:
    phi = np.zeros((2 * horizon, 2))
    gamma = np.zeros((2 * horizon, horizon))
    for i in range(horizon):
        phi[2 * i : 2 * i + 2] = np.linalg.matrix_power(a, i + 1)
        for j in range(i + 1):
            gamma[2 * i : 2 * i + 2, j : j + 1] = np.linalg.matrix_power(a, i - j) @ b
    return phi, gamma


class ResidualMPC:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.a, self.b = discretize_impedance(cfg)
        self.phi, self.gamma = prediction_matrices(self.a, self.b, cfg.horizon)
        # Dense qbar/rbar deliberately, not scipy.sparse: horizon is tiny
        # and mixing dense ndarrays (gamma, phi) with scipy.sparse matrices
        # in the same `@` chain is a known source of platform/BLAS-dependent
        # numerical warnings (an external review reported repeatable
        # divide-by-zero/overflow/invalid-value RuntimeWarnings at this
        # exact Hessian matmul on a different machine, not reproduced here,
        # but the dense/sparse mixing itself is the identifiable, fixable
        # risk regardless of which platform actually triggers it). Sparse
        # is still used below, where OSQP's own API requires it.
        qbar = np.kron(np.eye(cfg.horizon), np.diag([cfg.q_position, cfg.q_velocity]))
        rbar = np.eye(cfg.horizon) * cfg.r_force
        self.h = self.gamma.T @ qbar @ self.gamma + rbar
        self.f = self.gamma.T @ qbar @ self.phi
        self.gd = self.gamma @ np.ones(cfg.horizon)
        self.fd = (self.gamma.T @ qbar @ self.gd).reshape(-1)
        if not (np.all(np.isfinite(self.h)) and np.all(np.isfinite(self.f))
                and np.all(np.isfinite(self.fd))):
            raise FloatingPointError(
                "ResidualMPC: non-finite value in QP Hessian/gradient assembly "
                f"(h finite={np.all(np.isfinite(self.h))}, "
                f"f finite={np.all(np.isfinite(self.f))}, "
                f"fd finite={np.all(np.isfinite(self.fd))})"
            )
        self.problem = osqp.OSQP()
        self.problem.setup(
            P=sparse.csc_matrix((self.h + self.h.T) / 2),
            q=np.zeros(cfg.horizon),
            A=sparse.eye(cfg.horizon, format="csc"),
            l=-np.ones(cfg.horizon) * cfg.force_limit,
            u=np.ones(cfg.horizon) * cfg.force_limit,
            eps_abs=1e-7,
            eps_rel=1e-7,
            max_iter=4000,
            polishing=False,
            verbose=False,
        )
        self.solve_ms: list[float] = []

    # See verify_residual_mpc.py's ResidualMPC.FEAS_TOL for rationale: 100x
    # this problem's own eps_abs=eps_rel=1e-7, checked against the actual
    # primal residual rather than trusting the OSQP status string alone.
    FEAS_TOL = 100 * 1e-7

    def control(self, residual: np.ndarray, disturbance_hat: float, nominal_force: float) -> tuple[float, str]:
        linear = self.f @ residual + self.fd * disturbance_hat
        lo = np.full(self.cfg.horizon, -self.cfg.force_limit - nominal_force)
        hi = np.full(self.cfg.horizon, self.cfg.force_limit - nominal_force)
        start = time.perf_counter_ns()
        self.problem.update(q=linear, l=lo, u=hi)
        answer = self.problem.solve(raise_error=False)
        self.solve_ms.append((time.perf_counter_ns() - start) * 1e-6)
        if answer.info.status_val not in (1, 2) or answer.x is None:
            return 0.0, "infeasible_or_unsolved"
        x = answer.x
        finite = bool(np.all(np.isfinite(x)))
        residual_viol = (
            float(np.maximum(np.maximum(lo - x, 0), np.maximum(x - hi, 0)).max())
            if finite else float("inf")
        )
        self.last_residual = residual_viol
        if not finite or residual_viol > self.FEAS_TOL:
            return 0.0, f"residual_exceeds_tol({residual_viol:.3e})"
        return float(x[0]), answer.info.status


def intentional_force(t: float) -> float:
    if 0.7 <= t < 2.4:
        return 5.0
    if 4.8 <= t < 6.5:
        return -3.5
    return 0.0


def rejectable_force(t: float, phases: np.ndarray, scale: float) -> float:
    # The narrow 7 ms pulse begins between manager ticks and is the stress case
    # for held manager-rate authorization.
    pulse = 9.0 if 3.007 <= t < 3.014 else 0.0
    colored = 1.2 * np.sin(2 * np.pi * 0.83 * t + phases[0])
    colored += 0.7 * np.sin(2 * np.pi * 1.71 * t + phases[1])
    return scale * (pulse + colored)


def environment_force(x: float, v: float, stiffness: float, damping: float) -> float:
    # Unilateral passive wall at x = 0.035 m.
    penetration = max(0.0, x - 0.035)
    return -stiffness * penetration - damping * v * float(penetration > 0.0)


def project_fast(
    raw_residual: float,
    nominal: float,
    velocity: float,
    tank: float,
    cfg: Config,
) -> tuple[float, float]:
    candidate = float(np.clip(nominal + raw_residual, -cfg.force_limit, cfg.force_limit) - nominal)
    power = candidate * velocity
    available = max(0.0, tank - cfg.tank_minimum + cfg.dt * cfg.damping * velocity**2)
    alpha = 1.0 if power <= 0.0 else min(1.0, available / (cfg.dt * power + 1e-15))
    return alpha * candidate, alpha


def run_trial(
    mode: str,
    cfg: Config,
    seed: int,
    environment_stiffness: float = 700.0,
    environment_damping: float = 8.0,
    disturbance_scale: float = 1.0,
) -> dict:
    if mode not in {"impedance", "unguarded_mpc", "manager_guard", "fast_guard"}:
        raise ValueError(mode)
    rng = np.random.default_rng(seed)
    phases = rng.uniform(-np.pi, np.pi, 2)
    physical_mass = cfg.mass * rng.uniform(0.85, 1.15)
    steps = int(round(cfg.duration / cfg.dt))
    plant = np.zeros(2)
    reference = np.zeros(2)
    mpc = ResidualMPC(cfg)
    raw_residual = 0.0
    held_alpha = 1.0
    tank = cfg.tank_initial
    disturbance_hat = 0.0
    previous_residual_velocity = 0.0
    failures = 0
    supply = 0.0
    initial_storage = tank
    log = {key: np.zeros(steps) for key in (
        "t", "x", "v", "x_ref", "v_ref", "z", "z_dot", "intent", "disturbance",
        "environment", "nominal", "raw_residual", "applied_residual", "applied",
        "alpha", "tank", "balance", "saturated",
    )}

    for k in range(steps):
        t = k * cfg.dt
        f_int = intentional_force(t)
        f_dist = rejectable_force(t, phases, disturbance_scale)
        f_env = environment_force(plant[0], plant[1], environment_stiffness, environment_damping)
        residual = plant - reference
        nominal = -cfg.damping * plant[1] - cfg.stiffness * plant[0]

        if k % cfg.manager_steps == 0:
            observed = physical_mass * (residual[1] - previous_residual_velocity) / cfg.manager_dt
            observed += cfg.damping * residual[1] + cfg.stiffness * residual[0] - raw_residual
            disturbance_hat = (1.0 - cfg.observer_alpha) * disturbance_hat + cfg.observer_alpha * float(
                np.clip(observed, -cfg.observer_limit, cfg.observer_limit)
            )
            previous_residual_velocity = residual[1]
            if mode == "impedance":
                raw_residual = 0.0
                status = "not_solved"
            else:
                raw_residual, status = mpc.control(residual, disturbance_hat, nominal)
                failures += int(status not in ("solved", "solved inaccurate"))
            if mode == "manager_guard":
                _, held_alpha = project_fast(raw_residual, nominal, plant[1], tank, cfg)

        if mode == "fast_guard":
            applied_residual, alpha = project_fast(raw_residual, nominal, plant[1], tank, cfg)
        elif mode == "manager_guard":
            candidate = float(np.clip(nominal + raw_residual, -cfg.force_limit, cfg.force_limit) - nominal)
            applied_residual, alpha = held_alpha * candidate, held_alpha
        elif mode == "unguarded_mpc":
            applied_residual = float(np.clip(nominal + raw_residual, -cfg.force_limit, cfg.force_limit) - nominal)
            alpha = 1.0
        else:
            applied_residual, alpha = 0.0, 1.0

        applied = float(np.clip(nominal + applied_residual, -cfg.force_limit, cfg.force_limit))
        applied_residual = applied - nominal
        tank += cfg.dt * (cfg.damping * plant[1] ** 2 - applied_residual * plant[1])
        tank = min(cfg.tank_maximum, tank)

        total_external = f_int + f_dist + f_env
        supply += cfg.dt * total_external * plant[1]
        storage = 0.5 * physical_mass * plant[1] ** 2 + 0.5 * cfg.stiffness * plant[0] ** 2 + tank
        balance = storage - initial_storage - supply

        for key, value in (
            ("t", t), ("x", plant[0]), ("v", plant[1]), ("x_ref", reference[0]),
            ("v_ref", reference[1]), ("z", residual[0]), ("z_dot", residual[1]),
            ("intent", f_int), ("disturbance", f_dist), ("environment", f_env),
            ("nominal", nominal), ("raw_residual", raw_residual),
            ("applied_residual", applied_residual), ("applied", applied), ("alpha", alpha),
            ("tank", tank), ("balance", balance),
            ("saturated", float(abs(applied) >= cfg.force_limit - 1e-9)),
        ):
            log[key][k] = value

        reference_acc = (f_int - cfg.damping * reference[1] - cfg.stiffness * reference[0]) / cfg.mass
        plant_acc = (applied + total_external) / physical_mass
        reference += cfg.dt * np.array([reference[1], reference_acc])
        plant += cfg.dt * np.array([plant[1], plant_acc])

    use = np.arange(steps) * cfg.dt >= 0.5
    metrics = {
        "residual_rms_mm": float(1e3 * np.sqrt(np.mean(log["z"][use] ** 2))),
        "residual_peak_mm": float(1e3 * np.max(np.abs(log["z"][use]))),
        "minimum_tank_j": float(np.min(log["tank"])),
        "tank_violation_j": float(max(0.0, cfg.tank_minimum - np.min(log["tank"]))),
        "maximum_passivity_balance_j": float(np.max(log["balance"])),
        "projection_active_fraction": float(np.mean(log["alpha"] < 1.0 - 1e-10)),
        "saturation_fraction": float(np.mean(log["saturated"])),
        "qp_failures": failures,
        "solve_ms_mean": float(np.mean(mpc.solve_ms)) if mpc.solve_ms else 0.0,
        "solve_ms_p95": float(np.percentile(mpc.solve_ms, 95)) if mpc.solve_ms else 0.0,
        "solve_ms_max": float(np.max(mpc.solve_ms)) if mpc.solve_ms else 0.0,
    }
    return {"metrics": metrics, "log": {key: value.tolist() for key, value in log.items()}}


def monte_carlo(cfg: Config, seeds: int) -> dict:
    modes = ("impedance", "unguarded_mpc", "manager_guard", "fast_guard")
    raw = {mode: [] for mode in modes}
    rng = np.random.default_rng(20260802)
    for seed in range(seeds):
        ke = rng.uniform(300.0, 1400.0)
        be = rng.uniform(3.0, 15.0)
        scale = rng.uniform(0.8, 1.25)
        for mode in modes:
            result = run_trial(mode, cfg, seed, ke, be, scale)
            raw[mode].append({"seed": seed, "environment_stiffness": ke,
                              "environment_damping": be, "disturbance_scale": scale,
                              **result["metrics"]})
    metrics = ("residual_rms_mm", "residual_peak_mm", "minimum_tank_j", "tank_violation_j",
               "maximum_passivity_balance_j", "projection_active_fraction", "saturation_fraction")
    summary = {}
    for mode, rows in raw.items():
        summary[mode] = {}
        for metric in metrics:
            values = np.array([row[metric] for row in rows])
            summary[mode][metric] = {"mean": float(np.mean(values)),
                                      "std": float(np.std(values, ddof=1)),
                                      "max": float(np.max(values)),
                                      "min": float(np.min(values))}
    proposed = np.array([row["residual_rms_mm"] for row in raw["fast_guard"]])
    base = np.array([row["residual_rms_mm"] for row in raw["impedance"]])
    diff = proposed - base
    half = stats.t.ppf(0.975, seeds - 1) * stats.sem(diff)
    comparison = {
        "fast_guard_vs_impedance_residual_rms_relative_change_percent": float(100 * (proposed.mean() - base.mean()) / base.mean()),
        "paired_difference_mm": float(diff.mean()),
        "paired_95_percent_ci_mm": [float(diff.mean() - half), float(diff.mean() + half)],
        "paired_t_p_value": float(stats.ttest_rel(proposed, base).pvalue),
    }
    return {"seeds": seeds, "summary": summary, "comparison": comparison, "raw": raw}


def plot(results: dict, path: Path) -> None:
    colors = {"impedance": "#777777", "unguarded_mpc": "#d95f02",
              "manager_guard": "#7570b3", "fast_guard": "#1b9e77"}
    labels = {"impedance": "Passive impedance", "unguarded_mpc": "Residual MPC",
              "manager_guard": "50 Hz guard", "fast_guard": "50 Hz MPC + 1 kHz guard"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), constrained_layout=True)
    for mode, trial in results["representative"].items():
        log = trial["log"]
        t = np.asarray(log["t"])
        axes[0, 0].plot(t, 1e3 * np.asarray(log["z"]), color=colors[mode], label=labels[mode], lw=1.0)
        axes[0, 1].plot(t, np.asarray(log["tank"]), color=colors[mode], label=labels[mode], lw=1.0)
        axes[1, 0].plot(t, np.asarray(log["applied"]), color=colors[mode], label=labels[mode], lw=1.0)
    axes[0, 0].set_ylabel("residual position (mm)")
    axes[0, 1].set_ylabel("tank ledger (J)")
    axes[0, 1].axhline(Config.tank_minimum, color="k", ls=":", lw=0.9)
    axes[1, 0].set_ylabel("applied wrench (N)")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].axhline(Config.force_limit, color="k", ls=":", lw=0.9)
    axes[1, 0].axhline(-Config.force_limit, color="k", ls=":", lw=0.9)
    axes[0, 0].legend(fontsize=8, ncol=2)
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.grid(alpha=0.25)
    modes = list(colors)
    summary = results["monte_carlo"]["summary"]
    means = [summary[m]["residual_rms_mm"]["mean"] for m in modes]
    stds = [summary[m]["residual_rms_mm"]["std"] for m in modes]
    short_labels = ("Passive\nimpedance", "Unguarded\nMPC", "Manager\nonly", "Two-rate\nguard")
    axes[1, 1].bar(short_labels, means, yerr=stds,
                   color=[colors[m] for m in modes], capsize=3)
    axes[1, 1].set_ylabel("residual RMS (mm)")
    axes[1, 1].set_title(f"{results['monte_carlo']['seeds']}-seed stiffness/mismatch sweep")
    axes[1, 1].grid(axis="y", alpha=0.25)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--output", type=Path, default=HERE / "two_rate_passive_results.json")
    parser.add_argument("--figure", type=Path, default=HERE / "two_rate_passive_results.png")
    args = parser.parse_args()
    cfg = Config()
    modes = ("impedance", "unguarded_mpc", "manager_guard", "fast_guard")
    representative = {mode: run_trial(mode, cfg, seed=4) for mode in modes}
    results = {"protocol": asdict(cfg), "representative": representative,
               "monte_carlo": monte_carlo(cfg, args.seeds)}
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    plot(results, args.figure)
    print(json.dumps({"output": str(args.output), "figure": str(args.figure),
                      "representative": {m: v["metrics"] for m, v in representative.items()},
                      "monte_carlo": results["monte_carlo"]["summary"],
                      "comparison": results["monte_carlo"]["comparison"]}, indent=2))


if __name__ == "__main__":
    main()
