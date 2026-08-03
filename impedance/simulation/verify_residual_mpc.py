#!/usr/bin/env python3
"""Reproducible 1-DOF verification for reference-model impedance + residual MPC.

The model is deliberately small enough to audit.  It tests the structural claim
that exact inverse-dynamics cancellation exposes a gain-independent residual
double integrator, while retaining the important qualification that model error
and total-input constraints still depend on the nominal impedance trajectory.
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
    ts: float = 0.02
    inner_steps: int = 20
    duration: float = 8.0
    horizon: int = 15
    plant_mass: float = 4.0
    plant_damping: float = 6.0
    model_mass: float = 4.0
    model_damping: float = 6.0
    desired_mass: float = 3.0
    desired_stiffness: float = 80.0
    damping_ratio: float = 0.9
    force_limit: float = 10.0
    q_position: float = 2.0e4
    q_velocity: float = 80.0
    r_acceleration: float = 0.35
    disturbance_filter: float = 0.12
    disturbance_estimate_limit: float = 8.0

    @property
    def desired_damping(self) -> float:
        return 2.0 * self.damping_ratio * np.sqrt(
            self.desired_mass * self.desired_stiffness
        )


def zoh_discretize(a: np.ndarray, b: np.ndarray, ts: float) -> tuple[np.ndarray, np.ndarray]:
    n, m = b.shape
    block = np.zeros((n + m, n + m))
    block[:n, :n] = a
    block[:n, n:] = b
    exp_block = linalg.expm(block * ts)
    return exp_block[:n, :n], exp_block[:n, n:]


def residual_matrices(ts: float) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.array([[1.0, ts], [0.0, 1.0]]),
        np.array([[0.5 * ts**2], [ts]]),
    )


def prediction_matrices(a: np.ndarray, b: np.ndarray, horizon: int) -> tuple[np.ndarray, np.ndarray]:
    nx = a.shape[0]
    phi = np.zeros((horizon * nx, nx))
    gamma = np.zeros((horizon * nx, horizon))
    for i in range(horizon):
        phi[i * nx : (i + 1) * nx] = np.linalg.matrix_power(a, i + 1)
        for j in range(i + 1):
            gamma[i * nx : (i + 1) * nx, j : j + 1] = (
                np.linalg.matrix_power(a, i - j) @ b
            )
    return phi, gamma


class ResidualMPC:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.a, self.b = residual_matrices(cfg.ts)
        self.phi, self.gamma = prediction_matrices(self.a, self.b, cfg.horizon)
        q = np.diag([cfg.q_position, cfg.q_velocity])
        qbar = sparse.block_diag([q] * cfg.horizon, format="csc")
        rbar = sparse.eye(cfg.horizon, format="csc") * cfg.r_acceleration
        self.h = np.asarray(self.gamma.T @ qbar @ self.gamma + rbar)
        self.f = np.asarray(self.gamma.T @ qbar @ self.phi)
        self.gd = self.gamma @ np.ones(cfg.horizon)
        self.fd = np.asarray(self.gamma.T @ qbar @ self.gd).reshape(-1)
        self.problem = osqp.OSQP()
        self.problem.setup(
            P=sparse.csc_matrix((self.h + self.h.T) / 2.0),
            q=np.zeros(cfg.horizon),
            A=sparse.eye(cfg.horizon, format="csc"),
            l=-np.ones(cfg.horizon),
            u=np.ones(cfg.horizon),
            eps_abs=1e-7,
            eps_rel=1e-7,
            max_iter=4000,
            polishing=False,
            verbose=False,
        )
        self.solve_times = []

    @property
    def first_move_gain(self) -> np.ndarray:
        return np.linalg.solve(self.h, self.f)[0]

    def control(self, state: np.ndarray, d_hat: float, uff_horizon: np.ndarray) -> tuple[float, str]:
        q = self.f @ state + self.fd * d_hat
        lower = (-self.cfg.force_limit - uff_horizon) / self.cfg.model_mass
        upper = (self.cfg.force_limit - uff_horizon) / self.cfg.model_mass
        start = time.perf_counter_ns()
        self.problem.update(q=q, l=lower, u=upper)
        result = self.problem.solve(raise_error=False)
        self.solve_times.append((time.perf_counter_ns() - start) * 1e-6)
        if result.info.status_val not in (1, 2):
            return 0.0, result.info.status
        return float(result.x[0]), result.info.status


def intentional_force(t: float) -> float:
    if 0.8 <= t < 3.2:
        return 6.0
    if 4.2 <= t < 6.4:
        return -4.0
    return 0.0


def rejectable_disturbance(t: float, phases: np.ndarray, scale: float = 1.0) -> float:
    step = 3.0 if 2.0 <= t < 3.4 else 0.0
    colored = (
        1.2 * np.sin(2 * np.pi * 0.73 * t + phases[0])
        + 0.8 * np.sin(2 * np.pi * 1.31 * t + phases[1])
        + 0.45 * np.sin(2 * np.pi * 2.17 * t + phases[2])
    )
    return scale * (step + colored)


def generator_step(state: np.ndarray, force: float, cfg: Config) -> tuple[np.ndarray, float]:
    md, dd, kd = cfg.desired_mass, cfg.desired_damping, cfg.desired_stiffness
    a_cont = np.array([[0.0, 1.0], [-kd / md, -dd / md]])
    b_cont = np.array([[0.0], [1.0 / md]])
    ad, bd = zoh_discretize(a_cont, b_cont, cfg.ts)
    next_state = ad @ state + bd[:, 0] * force
    acceleration = (force - dd * state[1] - kd * state[0]) / md
    return next_state, float(acceleration)


def preview_feedforward(generator_state: np.ndarray, start_index: int, cfg: Config) -> np.ndarray:
    state = generator_state.copy()
    values = np.zeros(cfg.horizon)
    for i in range(cfg.horizon):
        t = (start_index + i) * cfg.ts
        force = intentional_force(t)
        next_state, acceleration = generator_step(state, force, cfg)
        values[i] = (
            cfg.model_mass * acceleration
            + cfg.model_damping * state[1]
            - force
        )
        state = next_state
    return values


def run_trial(
    controller: str,
    cfg: Config,
    seed: int = 0,
    disturbance: bool = True,
    disturbance_scale: float = 1.0,
) -> dict:
    rng = np.random.default_rng(seed)
    phases = rng.uniform(-np.pi, np.pi, 3)
    steps = int(round(cfg.duration / cfg.ts))
    times = np.arange(steps) * cfg.ts
    plant = np.zeros(2)
    generator = np.zeros(2)
    mpc = ResidualMPC(cfg)

    # Infinite-horizon residual feedback for the reactive baseline.
    a, b = residual_matrices(cfg.ts)
    q = np.diag([cfg.q_position, cfg.q_velocity])
    p = linalg.solve_discrete_are(a, b, q, np.array([[cfg.r_acceleration]]))
    k_lqr = np.linalg.solve(cfg.r_acceleration + b.T @ p @ b, b.T @ p @ a).reshape(-1)

    log = {key: np.zeros(steps) for key in (
        "t", "x", "v", "x_i", "v_i", "z", "z_dot", "f_int", "d",
        "u_ff", "u_c", "u", "d_hat", "headroom",
    )}
    d_hat = 0.0
    previous_z_dot = 0.0
    previous_a = 0.0
    failures = 0

    for k, t in enumerate(times):
        f_int = intentional_force(t)
        d = rejectable_disturbance(t, phases, disturbance_scale) if disturbance else 0.0
        residual = plant - generator

        if k > 0:
            observed_d = (residual[1] - previous_z_dot) / cfg.ts - previous_a
            observed_d = float(np.clip(
                observed_d,
                -cfg.disturbance_estimate_limit,
                cfg.disturbance_estimate_limit,
            ))
            d_hat = (1.0 - cfg.disturbance_filter) * d_hat + cfg.disturbance_filter * observed_d

        uff_horizon = preview_feedforward(generator, k, cfg)
        if controller == "classical_impedance":
            a_c = 0.0
            status = "not_solved"
        elif controller == "residual_lqr":
            a_c = float(-k_lqr @ residual - d_hat)
            status = "analytic"
        elif controller == "residual_mpc":
            a_c, status = mpc.control(residual, d_hat, uff_horizon)
            failures += int(status not in ("solved", "solved inaccurate"))
        else:
            raise ValueError(controller)

        generator_acceleration = (
            f_int
            - cfg.desired_damping * generator[1]
            - cfg.desired_stiffness * generator[0]
        ) / cfg.desired_mass
        u_ff = cfg.model_mass * generator_acceleration + cfg.model_damping * plant[1] - f_int
        if controller == "classical_impedance":
            desired_acceleration = (
                f_int
                - cfg.desired_damping * plant[1]
                - cfg.desired_stiffness * plant[0]
            ) / cfg.desired_mass
            u_c = 0.0
            applied = float(np.clip(
                cfg.model_mass * desired_acceleration + cfg.model_damping * plant[1] - f_int,
                -cfg.force_limit,
                cfg.force_limit,
            ))
        else:
            u_c = cfg.model_mass * a_c
            applied = float(np.clip(u_ff + u_c, -cfg.force_limit, cfg.force_limit))
        headroom = cfg.force_limit - abs(u_ff)

        for key, value in (
            ("t", t), ("x", plant[0]), ("v", plant[1]),
            ("x_i", generator[0]), ("v_i", generator[1]),
            ("z", residual[0]), ("z_dot", residual[1]),
            ("f_int", f_int), ("d", d), ("u_ff", u_ff),
            ("u_c", u_c), ("u", applied), ("d_hat", d_hat),
            ("headroom", headroom),
        ):
            log[key][k] = value

        previous_z_dot = residual[1]
        previous_a = a_c

        # Fast realization loop. Under matched parameters and inactive clipping,
        # plant and reference-model derivatives are identical when z=0.
        inner_dt = cfg.ts / cfg.inner_steps
        for j in range(cfg.inner_steps):
            inner_t = t + j * inner_dt
            inner_f_int = intentional_force(inner_t)
            inner_d = (
                rejectable_disturbance(inner_t, phases, disturbance_scale)
                if disturbance
                else 0.0
            )
            generator_acceleration = (
                inner_f_int
                - cfg.desired_damping * generator[1]
                - cfg.desired_stiffness * generator[0]
            ) / cfg.desired_mass
            if controller == "classical_impedance":
                desired_acceleration = (
                    inner_f_int
                    - cfg.desired_damping * plant[1]
                    - cfg.desired_stiffness * plant[0]
                ) / cfg.desired_mass
                inner_command = (
                    cfg.model_mass * desired_acceleration
                    + cfg.model_damping * plant[1]
                    - inner_f_int
                )
            else:
                inner_uff = (
                    cfg.model_mass * generator_acceleration
                    + cfg.model_damping * plant[1]
                    - inner_f_int
                )
                inner_command = inner_uff + cfg.model_mass * a_c
            inner_applied = float(np.clip(inner_command, -cfg.force_limit, cfg.force_limit))
            plant_acceleration = (
                inner_applied + inner_f_int + inner_d - cfg.plant_damping * plant[1]
            ) / cfg.plant_mass
            plant = plant + inner_dt * np.array([plant[1], plant_acceleration])
            generator = generator + inner_dt * np.array([generator[1], generator_acceleration])

    trim = times >= 0.5
    metrics = {
        "residual_rms_mm": float(1e3 * np.sqrt(np.mean(log["z"][trim] ** 2))),
        "residual_peak_mm": float(1e3 * np.max(np.abs(log["z"][trim]))),
        "velocity_rms_mm_s": float(1e3 * np.sqrt(np.mean(log["z_dot"][trim] ** 2))),
        "impedance_fidelity_rms_mm": float(1e3 * np.sqrt(np.mean((log["x"] - log["x_i"])[trim] ** 2))),
        "saturation_fraction": float(np.mean(np.abs(log["u"]) >= cfg.force_limit - 1e-9)),
        "min_nominal_headroom_n": float(np.min(log["headroom"])),
        "qp_failures": failures,
        "solve_ms_mean": float(np.mean(mpc.solve_times)) if mpc.solve_times else 0.0,
        "solve_ms_p95": float(np.percentile(mpc.solve_times, 95)) if mpc.solve_times else 0.0,
        "solve_ms_max": float(np.max(mpc.solve_times)) if mpc.solve_times else 0.0,
    }
    return {"metrics": metrics, "log": {k: v.tolist() for k, v in log.items()}}


def gain_sweep() -> dict:
    rows = []
    reference_gain = None
    for stiffness in (40.0, 80.0, 160.0, 240.0):
        cfg = Config(desired_stiffness=stiffness)
        mpc = ResidualMPC(cfg)
        gain = mpc.first_move_gain
        if reference_gain is None:
            reference_gain = gain
        exact = run_trial("residual_mpc", cfg, seed=4, disturbance=True)
        rows.append({
            "desired_stiffness_n_m": stiffness,
            "desired_damping_n_s_m": cfg.desired_damping,
            "first_move_gain": gain.tolist(),
            "gain_difference_norm": float(np.linalg.norm(gain - reference_gain)),
            **exact["metrics"],
        })
    return {"rows": rows}


def mismatch_monte_carlo(seeds: int = 30) -> dict:
    rng = np.random.default_rng(20260802)
    controllers = ("classical_impedance", "residual_lqr", "residual_mpc")
    raw = {name: [] for name in controllers}
    for seed in range(seeds):
        mass_ratio = rng.uniform(0.8, 1.2)
        damping_ratio = rng.uniform(0.7, 1.3)
        disturbance_scale = rng.uniform(0.8, 1.2)
        cfg = Config(
            plant_mass=Config.plant_mass * mass_ratio,
            plant_damping=Config.plant_damping * damping_ratio,
        )
        for controller in controllers:
            trial = run_trial(controller, cfg, seed=seed, disturbance=True, disturbance_scale=disturbance_scale)
            raw[controller].append({
                "seed": seed,
                "mass_ratio": mass_ratio,
                "damping_ratio": damping_ratio,
                "disturbance_scale": disturbance_scale,
                **trial["metrics"],
            })
    summary = {}
    for controller, rows in raw.items():
        summary[controller] = {}
        for key in (
            "residual_rms_mm", "residual_peak_mm", "velocity_rms_mm_s",
            "saturation_fraction", "min_nominal_headroom_n", "qp_failures",
            "solve_ms_mean", "solve_ms_p95", "solve_ms_max",
        ):
            values = np.array([row[key] for row in rows], dtype=float)
            summary[controller][key] = {
                "mean": float(np.mean(values)),
                "std": float(np.std(values, ddof=1)),
                "max": float(np.max(values)),
            }
    comparisons = {}
    for baseline in ("classical_impedance", "residual_lqr"):
        comparisons[baseline] = {}
        for key in ("residual_rms_mm", "saturation_fraction"):
            base = np.array([row[key] for row in raw[baseline]], dtype=float)
            proposed = np.array([row[key] for row in raw["residual_mpc"]], dtype=float)
            difference = proposed - base
            sem = stats.sem(difference)
            half_width = float(stats.t.ppf(0.975, seeds - 1) * sem) if sem > 0 else 0.0
            comparisons[baseline][key] = {
                "proposed_minus_baseline_mean": float(np.mean(difference)),
                "paired_95_percent_ci": [
                    float(np.mean(difference) - half_width),
                    float(np.mean(difference) + half_width),
                ],
                "paired_t_p_value": float(stats.ttest_rel(proposed, base).pvalue),
                "relative_change_percent": float(
                    100.0 * (np.mean(proposed) - np.mean(base)) / np.mean(base)
                ),
            }
    return {"seeds": seeds, "summary": summary, "paired_comparisons": comparisons, "raw": raw}


def structural_audit() -> dict:
    cfg = Config()
    a, b = residual_matrices(cfg.ts)
    gains = {}
    hessians = {}
    for stiffness in (40.0, 80.0, 160.0, 240.0):
        local = Config(desired_stiffness=stiffness)
        mpc = ResidualMPC(local)
        gains[str(stiffness)] = mpc.first_move_gain.tolist()
        hessians[str(stiffness)] = mpc.h
    reference = hessians["40.0"]
    return {
        "A": a.tolist(),
        "B": b.tolist(),
        "first_move_gains": gains,
        "maximum_hessian_difference": float(max(
            np.max(np.abs(hessian - reference)) for hessian in hessians.values()
        )),
        "maximum_gain_difference": float(max(
            np.max(np.abs(np.asarray(gain) - np.asarray(gains["40.0"])))
            for gain in gains.values()
        )),
        "qualification": (
            "The residual prediction matrices are gain independent. The total-input "
            "bounds and mismatch term are not: both depend on the generated nominal trajectory."
        ),
    }


def plot_results(results: dict, output: Path) -> None:
    nominal = results["representative"]["classical_impedance"]["log"]
    lqr = results["representative"]["residual_lqr"]["log"]
    mpc = results["representative"]["residual_mpc"]["log"]
    sweep = results["gain_sweep"]["rows"]

    fig, axes = plt.subplots(3, 2, figsize=(11.0, 8.0), constrained_layout=True)
    t = np.asarray(mpc["t"])
    for label, log, color in (
        ("Classical impedance", nominal, "#777777"),
        ("Residual LQR", lqr, "#d95f02"),
        ("Residual MPC", mpc, "#1b9e77"),
    ):
        axes[0, 0].plot(t, 1e3 * np.asarray(log["x"]), label=label, color=color, lw=1.2)
        axes[1, 0].plot(t, 1e3 * np.asarray(log["z"]), label=label, color=color, lw=1.1)
        axes[2, 0].plot(t, np.asarray(log["u"]), label=label, color=color, lw=1.0)
    axes[0, 0].plot(t, 1e3 * np.asarray(mpc["x_i"]), "k--", lw=1.2, label="Impedance reference model")
    axes[0, 0].set_ylabel("position (mm)")
    axes[1, 0].set_ylabel("residual z (mm)")
    axes[2, 0].set_ylabel("applied force (N)")
    axes[2, 0].set_xlabel("time (s)")
    axes[2, 0].axhline(Config.force_limit, color="k", ls=":", lw=0.8)
    axes[2, 0].axhline(-Config.force_limit, color="k", ls=":", lw=0.8)
    axes[0, 0].legend(fontsize=8, ncol=2)
    for ax in axes[:, 0]:
        ax.grid(alpha=0.25)

    controllers = ("classical_impedance", "residual_lqr", "residual_mpc")
    names = ("Classical\nimpedance", "Residual\nLQR", "Residual\nMPC")
    colors = ("#777777", "#d95f02", "#1b9e77")
    mc = results["monte_carlo"]["summary"]
    means = [mc[c]["residual_rms_mm"]["mean"] for c in controllers]
    stds = [mc[c]["residual_rms_mm"]["std"] for c in controllers]
    axes[0, 1].bar(names, means, yerr=stds, color=colors, capsize=3)
    axes[0, 1].set_ylabel("residual RMS (mm)")
    axes[0, 1].set_title("30-seed model-mismatch study")
    axes[0, 1].grid(axis="y", alpha=0.25)

    stiffness = [row["desired_stiffness_n_m"] for row in sweep]
    residual = [row["residual_rms_mm"] for row in sweep]
    headroom = [row["min_nominal_headroom_n"] for row in sweep]
    axes[1, 1].plot(stiffness, residual, "o-", color="#1b9e77")
    axes[1, 1].set_ylabel("residual RMS (mm)")
    axes[1, 1].set_xlabel("desired stiffness (N/m)")
    axes[1, 1].set_title("Fixed residual MPC under impedance retuning")
    axes[1, 1].grid(alpha=0.25)

    axes[2, 1].plot(stiffness, headroom, "s-", color="#7570b3")
    axes[2, 1].axhline(0.0, color="k", ls=":", lw=0.8)
    axes[2, 1].set_ylabel("minimum nominal headroom (N)")
    axes[2, 1].set_xlabel("desired stiffness (N/m)")
    axes[2, 1].set_title("Constraint authority is not gain invariant")
    axes[2, 1].grid(alpha=0.25)
    fig.savefig(output, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--output", type=Path, default=HERE / "residual_mpc_results.json")
    parser.add_argument("--figure", type=Path, default=HERE / "residual_mpc_results.png")
    args = parser.parse_args()

    cfg = Config()
    representative = {
        controller: run_trial(controller, cfg, seed=3, disturbance=True)
        for controller in ("classical_impedance", "residual_lqr", "residual_mpc")
    }
    intentional_only = {
        controller: run_trial(controller, cfg, seed=3, disturbance=False)["metrics"]
        for controller in ("classical_impedance", "residual_lqr", "residual_mpc")
    }
    results = {
        "protocol": asdict(cfg),
        "structural_audit": structural_audit(),
        "intentional_force_only": intentional_only,
        "representative": representative,
        "gain_sweep": gain_sweep(),
        "monte_carlo": mismatch_monte_carlo(args.seeds),
    }
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    plot_results(results, args.figure)
    print(json.dumps({
        "output": str(args.output),
        "figure": str(args.figure),
        "structural_audit": results["structural_audit"],
        "intentional_force_only": intentional_only,
        "representative_metrics": {
            key: value["metrics"] for key, value in representative.items()
        },
        "gain_sweep": results["gain_sweep"],
        "monte_carlo_summary": results["monte_carlo"]["summary"],
    }, indent=2))


if __name__ == "__main__":
    main()
