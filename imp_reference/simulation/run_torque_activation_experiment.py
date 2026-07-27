"""Torque-active FR3 ablation for the behavior-realization architecture.

This is deliberately a focused runtime experiment, not a new controller
baseline. Joint 4's available torque budget is derated from 87 Nm to 31.5 Nm
so that the frozen-model constraint activates under the paper's existing
20 N impedance push. Two otherwise identical predictive realizers are run:

1. torque feasibility at every predicted step;
2. torque feasibility at the first predicted step only.

The study asks whether the runtime's internal plan remains actuator-feasible
when desired behavior conflicts with the derated budget. Executed MuJoCo
torque is reported separately because the frozen dynamics are only a local
model and therefore cannot guarantee exact nonlinear-plant feasibility.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict, replace
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

from fr3_mujoco import FR3MuJoCoEnv, TAU_LIMIT  # noqa: E402
from fr3_interaction_dynamics_mpc import (  # noqa: E402
    FR3MPCConfig,
    ImpedanceReference3D,
)
from run_fr3_experiments import metrics, run_case  # noqa: E402


DERATED_JOINT = 3
DERATED_LIMIT_NM = 31.5
DURATION_S = 6.0


def make_configs() -> dict[str, FR3MPCConfig]:
    limits = TAU_LIMIT.copy()
    limits[DERATED_JOINT] = DERATED_LIMIT_NM
    full = FR3MPCConfig(tau_max=limits)
    return {
        "horizon_wide": full,
        "first_step_only": replace(full, torque_constraint_steps=1),
    }


def run_experiment() -> tuple[dict, dict]:
    env = FR3MuJoCoEnv(timestep=0.001)
    generator = ImpedanceReference3D()
    configs = make_configs()
    logs = {}
    report = {
        "purpose": "activate the realization runtime's torque constraint",
        "derated_joint_zero_based": DERATED_JOINT,
        "derated_limit_Nm": DERATED_LIMIT_NM,
        "nominal_FR3_limit_Nm": float(TAU_LIMIT[DERATED_JOINT]),
        "duration_s": DURATION_S,
        "generator": asdict(generator),
        "cases": {},
    }
    for name, cfg in configs.items():
        logs[name] = run_case(env, generator, "mpc", cfg, duration=DURATION_S)
        report["cases"][name] = metrics(logs[name], cfg)
        report["cases"][name]["torque_constraint_steps"] = (
            cfg.horizon if cfg.torque_constraint_steps is None else cfg.torque_constraint_steps
        )
    return logs, report


def make_figure(logs: dict, output: Path) -> None:
    colors = {"horizon_wide": "#0072B2", "first_step_only": "#D55E00"}
    labels = {
        "horizon_wide": "Horizon-wide torque feasibility",
        "first_step_only": "First-step-only torque feasibility",
    }
    fig, axes = plt.subplots(4, 1, figsize=(9.0, 9.0), sharex=True)
    for name, log in logs.items():
        t = log["time"]
        color = colors[name]
        axes[0].plot(t, log["tau"][:, DERATED_JOINT], color=color, label=labels[name])
        axes[1].plot(t, log["planned_torque_violation_Nm"], color=color)
        axes[2].plot(
            t,
            np.linalg.norm(log["empirical_realization_residual"], axis=1),
            color=color,
        )
        axes[3].plot(t, np.abs(log["ee_pos"][:, 2]), color=color)

    axes[0].axhline(DERATED_LIMIT_NM, color="0.25", linestyle=":", label="Derated limit")
    axes[0].set_ylabel("Joint 4 torque (Nm)")
    axes[1].set_ylabel("Planned torque\nviolation (Nm)")
    axes[2].set_ylabel(r"$\|a-a^{id}\|$ (m/s²)")
    axes[3].set_ylabel(r"$|e_z|$ (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].axhline(0.06, color="0.25", linestyle=":", label="Workspace bound")
    for ax in axes:
        ax.grid(alpha=0.25)
    axes[0].legend(loc="best")
    fig.suptitle(
        "Torque-active runtime intervention under a derated joint-4 budget",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=200)
    plt.close(fig)


def main() -> None:
    output_dir = Path(__file__).resolve().parents[1] / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    logs, report = run_experiment()
    make_figure(logs, output_dir / "torque_activation_results.png")
    with (output_dir / "torque_activation_metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps(report["cases"], indent=2))


if __name__ == "__main__":
    main()
