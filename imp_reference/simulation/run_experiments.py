"""Run the reference-interaction-dynamics experiments and save paper artifacts."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from interaction_dynamics_mpc import (
    AdmittanceReference,
    ImpedanceReference,
    InteractionDynamicsMPC,
    MPCConfig,
    clipped_reference_command,
    integrate_point_mass,
)


def human_force_at(time: float) -> np.ndarray:
    """Smooth 12 N lateral collaboration force applied from 1 s to 3 s."""

    if 1.0 <= time < 1.25:
        scale = 0.5 - 0.5 * np.cos(np.pi * (time - 1.0) / 0.25)
    elif 1.25 <= time < 2.75:
        scale = 1.0
    elif 2.75 <= time < 3.0:
        scale = 0.5 + 0.5 * np.cos(np.pi * (time - 2.75) / 0.25)
    else:
        scale = 0.0
    return np.array([0.0, 12.0 * scale])


def run_case(generator, controller_kind: str, cfg: MPCConfig, duration: float = 6.0):
    steps = int(round(duration / cfg.dt))
    state = np.zeros(4)
    previous_command = np.zeros(2)
    mpc = InteractionDynamicsMPC(generator, cfg)

    log = {
        "time": [],
        "state": [],
        "human_force": [],
        "command": [],
        "actual_acceleration": [],
        "reference_acceleration": [],
        "realization_residual": [],
        "regularization_residual": [],
        "constraint_intervention_residual": [],
        "decomposition_closure_error": [],
        "active": [],
        "solver_status": [],
    }
    for index in range(steps):
        time = index * cfg.dt
        force = human_force_at(time)
        # Measured-force zero-order-hold forecast: no oracle knowledge of the
        # future force profile is given to the predictive controller.
        forecast = np.repeat(force[None, :], cfg.horizon, axis=0)

        if controller_kind == "mpc":
            decision = mpc.control(state, forecast)
            command = decision.command
            active = list(decision.active_constraints)
            status = decision.status
            regularization_residual = decision.regularization_residual
            constraint_intervention_residual = decision.constraint_intervention_residual
            decomposition_closure_error = decision.decomposition_closure_error
        elif controller_kind == "clipped":
            command = clipped_reference_command(
                generator, state, force, cfg, previous_command
            )
            active = []
            status = "not_applicable"
            regularization_residual = np.full(2, np.nan)
            constraint_intervention_residual = np.full(2, np.nan)
            decomposition_closure_error = np.full(2, np.nan)
        else:
            raise ValueError(controller_kind)

        actual_acceleration = (command + force) / cfg.robot_mass
        reference_acceleration = generator.acceleration(state, force)
        residual = actual_acceleration - reference_acceleration

        log["time"].append(time)
        log["state"].append(state.copy())
        log["human_force"].append(force.copy())
        log["command"].append(command.copy())
        log["actual_acceleration"].append(actual_acceleration.copy())
        log["reference_acceleration"].append(reference_acceleration.copy())
        log["realization_residual"].append(residual.copy())
        log["regularization_residual"].append(regularization_residual.copy())
        log["constraint_intervention_residual"].append(
            constraint_intervention_residual.copy()
        )
        log["decomposition_closure_error"].append(
            decomposition_closure_error.copy()
        )
        log["active"].append(active)
        log["solver_status"].append(status)

        state = integrate_point_mass(
            state, command, force, cfg.robot_mass, cfg.dt
        )
        previous_command = command

    for key in (
        "time",
        "state",
        "human_force",
        "command",
        "actual_acceleration",
        "reference_acceleration",
        "realization_residual",
        "regularization_residual",
        "constraint_intervention_residual",
        "decomposition_closure_error",
    ):
        log[key] = np.asarray(log[key])
    return log


def metrics(log, cfg: MPCConfig) -> dict[str, float | int]:
    position = log["state"][:, :2]
    velocity = log["state"][:, 2:]
    command = log["command"]
    residual = log["realization_residual"]
    valid_decomposition = np.all(
        np.isfinite(log["regularization_residual"]), axis=1
    )
    active_steps = sum(bool(items) for items in log["active"])
    return {
        "realization_rmse_mps2": float(np.sqrt(np.mean(residual**2))),
        "regularization_residual_rmse_mps2": float(
            np.sqrt(np.mean(log["regularization_residual"][valid_decomposition] ** 2))
        ) if np.any(valid_decomposition) else None,
        "constraint_intervention_rmse_mps2": float(
            np.sqrt(
                np.mean(
                    log["constraint_intervention_residual"][valid_decomposition] ** 2
                )
            )
        ) if np.any(valid_decomposition) else None,
        "decomposition_max_closure_error_mps2": float(
            np.max(np.abs(log["decomposition_closure_error"][valid_decomposition]))
        ) if np.any(valid_decomposition) else None,
        "max_abs_position_m": float(np.max(np.abs(position))),
        "max_speed_mps": float(np.max(np.abs(velocity))),
        "max_speed_norm_mps": float(np.max(np.linalg.norm(velocity, axis=1))),
        "max_abs_command_N": float(np.max(np.abs(command))),
        "position_violation_m": float(
            max(0.0, np.max(np.abs(position)) - cfg.position_limit)
        ),
        "component_speed_violation_mps": float(
            max(0.0, np.max(np.abs(velocity)) - cfg.speed_limit)
        ),
        "active_constraint_steps": int(active_steps),
        "final_y_m": float(position[-1, 1]),
        "peak_y_m": float(np.max(position[:, 1])),
    }


def make_figure(cases, cfg: MPCConfig, output: Path) -> None:
    colors = {"mpc": "#0072B2", "clipped": "#D55E00"}
    labels = {"mpc": "Predictive realization", "clipped": "Reactive clipping"}
    fig, axes = plt.subplots(4, 2, figsize=(12.0, 10.0), sharex=True)

    for column, generator_name in enumerate(("impedance", "admittance")):
        for controller_kind in ("mpc", "clipped"):
            log = cases[(generator_name, controller_kind)]
            time = log["time"]
            style = "-" if controller_kind == "mpc" else "--"
            color = colors[controller_kind]
            axes[0, column].plot(
                time,
                log["state"][:, 1],
                style,
                color=color,
                label=labels[controller_kind],
                linewidth=2,
            )
            axes[1, column].plot(
                time,
                log["state"][:, 3],
                style,
                color=color,
                linewidth=2,
            )
            axes[2, column].plot(
                time,
                log["command"][:, 1],
                style,
                color=color,
                linewidth=2,
            )
            axes[3, column].plot(
                time,
                np.linalg.norm(log["realization_residual"], axis=1),
                style,
                color=color,
                linewidth=2,
            )

        axes[0, column].axhline(cfg.position_limit, color="0.3", linestyle=":")
        axes[0, column].axhline(-cfg.position_limit, color="0.3", linestyle=":")
        axes[1, column].axhline(cfg.speed_limit, color="0.3", linestyle=":")
        axes[1, column].axhline(-cfg.speed_limit, color="0.3", linestyle=":")
        axes[2, column].axhline(cfg.force_limit, color="0.3", linestyle=":")
        axes[2, column].axhline(-cfg.force_limit, color="0.3", linestyle=":")
        axes[0, column].set_title(f"{generator_name.capitalize()} generator")
        axes[3, column].set_xlabel("Time (s)")
        for row in range(4):
            axes[row, column].grid(alpha=0.25)

    axes[0, 0].set_ylabel("Lateral position (m)")
    axes[1, 0].set_ylabel("Lateral velocity (m/s)")
    axes[2, 0].set_ylabel("Robot force (N)")
    axes[3, 0].set_ylabel(r"$\|a-a^{id}\|$ (m/s²)")
    axes[0, 0].legend(loc="best")
    fig.suptitle(
        "One predictive safety layer realizes two reference interaction dynamics",
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cfg = MPCConfig()
    generators = (ImpedanceReference(), AdmittanceReference())
    cases = {}
    report = {
        "configuration": asdict(cfg),
        "force_forecast": "measured force held constant over the horizon",
        "generators": {g.name: asdict(g) for g in generators},
        "cases": {},
    }

    for generator in generators:
        for controller_kind in ("mpc", "clipped"):
            key = (generator.name, controller_kind)
            cases[key] = run_case(generator, controller_kind, cfg)
            report["cases"][f"{generator.name}_{controller_kind}"] = metrics(
                cases[key], cfg
            )

    make_figure(cases, cfg, args.output_dir / "interaction_dynamics_results.png")
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report["cases"], indent=2))
    print(f"Saved results to {args.output_dir}")


if __name__ == "__main__":
    main()
