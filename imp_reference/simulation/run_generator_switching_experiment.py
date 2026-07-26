"""Online generator switching: one MPC controller instance, no restart, no retuning.

Demonstrates the practical content of Theorem 1 (Affine Generator Independence,
paper.md Section 6) directly rather than only by running two separate
simulations side by side (Section 7's main experiment). A single
``InteractionDynamicsMPC`` instance is constructed once; at each scheduled
switch time, exactly one attribute -- ``controller.generator`` -- is
reassigned to a different generator object. Nothing else about the
controller (its QP structure, weights, constraints, ``previous_command``
state) is touched or reconstructed. The generator's affine law is read fresh
from ``self.generator`` on every solve (see ``_condense``), so the new
generator takes full effect on the very next re-solve after the switch --
there is no separate "reload" or "retune" step.

A small constant lateral force is held for the entire run (no release), so
the external condition never changes; only the requested interaction
dynamics does, at the two switch instants. This isolates the switching
behavior from the constraint-boundary story that Section 7's main
experiment already covers -- the force magnitude here is deliberately small
enough that the workspace/speed bounds are never approached.
"""

from __future__ import annotations

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
    integrate_point_mass,
)

FORCE_MAGNITUDE = 1.0  # N; small enough that workspace/speed bounds are never approached
FORCE_RAMP = 0.1  # s; smooth turn-on only, no release
SWITCH_TIMES = (2.0, 4.0)  # s; impedance -> admittance -> impedance
DURATION = 6.0  # s


def human_force_at(t: float, magnitude: float = FORCE_MAGNITUDE, ramp: float = FORCE_RAMP) -> np.ndarray:
    """Smooth turn-on to a constant force, held for the entire run (no release)."""
    scale = 0.5 - 0.5 * np.cos(np.pi * min(t, ramp) / ramp) if t < ramp else 1.0
    return np.array([0.0, magnitude * scale])


def run_switching_case(cfg: MPCConfig, duration: float = DURATION, switch_times=SWITCH_TIMES):
    impedance = ImpedanceReference()
    admittance = AdmittanceReference()
    # ONE controller instance for the whole run -- this is the point being demonstrated.
    controller = InteractionDynamicsMPC(impedance, cfg)

    schedule = [
        (0.0, impedance, "impedance"),
        (switch_times[0], admittance, "admittance"),
        (switch_times[1], impedance, "impedance"),
    ]
    schedule_idx = 0
    active_name = schedule[0][2]

    steps = int(round(duration / cfg.dt))
    state = np.zeros(4)
    log = {
        "time": [],
        "state": [],
        "command": [],
        "human_force": [],
        "active_generator": [],
        "command_jump_at_switch": {},
    }
    previous_command_before_tick = np.zeros(2)

    for i in range(steps):
        t = i * cfg.dt

        # Advance the switching schedule: reassign controller.generator, nothing else.
        if schedule_idx + 1 < len(schedule) and t >= schedule[schedule_idx + 1][0]:
            schedule_idx += 1
            _, new_generator, active_name = schedule[schedule_idx]
            controller.generator = new_generator  # the only line that changes anything

        force = human_force_at(t)
        forecast = np.tile(force, (cfg.horizon, 1))
        decision = controller.control(state, forecast)

        if schedule_idx > 0 and i > 0 and abs(t - schedule[schedule_idx][0]) < cfg.dt / 2:
            jump = float(np.max(np.abs(decision.command - previous_command_before_tick)))
            log["command_jump_at_switch"][f"t={schedule[schedule_idx][0]}"] = jump

        log["time"].append(t)
        log["state"].append(state.copy())
        log["command"].append(decision.command.copy())
        log["human_force"].append(force.copy())
        log["active_generator"].append(active_name)

        state = integrate_point_mass(state, decision.command, force, cfg.robot_mass, cfg.dt)
        previous_command_before_tick = decision.command.copy()

    for key in ("time", "state", "command", "human_force"):
        log[key] = np.asarray(log[key])
    return log


def metrics(log, cfg: MPCConfig) -> dict:
    position = log["state"][:, :2]
    velocity = log["state"][:, 2:]
    rate_step = cfg.force_rate_limit * cfg.dt
    return {
        "max_abs_position_m": float(np.max(np.abs(position))),
        "max_speed_mps": float(np.max(np.abs(velocity))),
        "max_speed_norm_mps": float(np.max(np.linalg.norm(velocity, axis=1))),
        "position_limit_m": cfg.position_limit,
        "speed_limit_mps": cfg.speed_limit,
        "command_jump_at_switch_N": log["command_jump_at_switch"],
        "command_rate_limit_N": rate_step,
        "no_switch_jump_exceeds_rate_limit": all(
            v <= rate_step + 1e-6 for v in log["command_jump_at_switch"].values()
        ),
    }


def make_figure(log, cfg: MPCConfig, output: Path, switch_times=SWITCH_TIMES) -> None:
    colors = {"impedance": "#0072B2", "admittance": "#D55E00"}
    time = log["time"]
    active = np.asarray(log["active_generator"])

    fig, axes = plt.subplots(3, 1, figsize=(9.0, 8.0), sharex=True)

    # Shade the background by active generator.
    boundaries = [0.0] + list(switch_times) + [time[-1]]
    names = ["impedance", "admittance", "impedance"]
    for ax in axes:
        for start, end, name in zip(boundaries[:-1], boundaries[1:], names):
            ax.axvspan(start, end, color=colors[name], alpha=0.08, lw=0)
        for switch_t in switch_times:
            ax.axvline(switch_t, color="0.3", linestyle=":", linewidth=1.2)

    axes[0].plot(time, log["state"][:, 1], color="black", linewidth=2)
    axes[0].set_ylabel("Lateral position (m)")
    axes[0].axhline(cfg.position_limit, color="0.6", linestyle="--", linewidth=1)
    axes[0].axhline(-cfg.position_limit, color="0.6", linestyle="--", linewidth=1)

    axes[1].plot(time, log["state"][:, 3], color="black", linewidth=2)
    axes[1].set_ylabel("Lateral velocity (m/s)")

    axes[2].plot(time, log["command"][:, 1], color="black", linewidth=2)
    axes[2].set_ylabel("Robot force (N)")
    axes[2].set_xlabel("Time (s)")

    for i, (start, end, name) in enumerate(zip(boundaries[:-1], boundaries[1:], names)):
        axes[0].text(
            (start + end) / 2, cfg.position_limit * 0.85, name,
            ha="center", va="top", fontsize=10, color=colors[name], fontweight="bold",
        )

    for ax in axes:
        ax.grid(alpha=0.25)

    fig.suptitle(
        "Online generator switching: one controller instance, only .generator reassigned",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=200)
    plt.close(fig)


def main() -> None:
    output_dir = Path(__file__).resolve().parents[1] / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg = MPCConfig()
    log = run_switching_case(cfg)
    report = {
        "configuration": asdict(cfg),
        "switch_times_s": SWITCH_TIMES,
        "force_magnitude_N": FORCE_MAGNITUDE,
        "metrics": metrics(log, cfg),
    }

    make_figure(log, cfg, output_dir / "generator_switching_results.png")
    with (output_dir / "generator_switching_metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report["metrics"], indent=2))
    print(f"Saved results to {output_dir}")


if __name__ == "__main__":
    main()
