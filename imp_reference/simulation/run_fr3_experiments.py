"""Run the FR3/MuJoCo generator-realizer experiments and save paper artifacts.

Mirrors ``run_experiments.py`` (the planar prototype's driver) in scope and
output shape, and ``simulation/phri.py``/``guidance.py``'s benchmark-loop
convention: get dynamics/state every tick, compute torque, apply, step, log.

The scenario mirrors the planar prototype's own story rather than the
sibling double-integrator paper's circular-trajectory benchmark: the EE
holds a fixed nominal pose while a sustained human push would, under naive
realization, drive it toward/through a workspace or speed boundary.
Predictive realization should respect the bound; reactive clipping should
not, since it has no lookahead on state constraints.

Inner-loop rate discipline: the QP (or the reactive comparator's own
correction decision) is only re-evaluated every ``mpc_every`` ticks, but
the feedforward/orientation/null-space torque (``compute_tau_base``) is
recomputed from the CURRENT (q, qdot) on every single tick for BOTH
controllers. This is the exact property a reviewer caught missing in the
sibling paper -- see fr3_interaction_dynamics_mpc.py's module docstring.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time as _time
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

from fr3_mujoco import FR3MuJoCoEnv  # noqa: E402

from fr3_interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference3D,
    FR3MPCConfig,
    FR3RealizationMPC,
    ImpedanceReference3D,
    combine_full_torque,
    compute_tau_base,
    fr3_clipped_reference_command,
    make_default_impedance_params,
)


def human_force_at(
    time: float,
    magnitude: float = 20.0,
    axis: tuple[float, float, float] = (0.0, 0.0, -1.0),
    t_on: float = 1.0,
    ramp: float = 0.25,
    hold: float = 2.0,
) -> np.ndarray:
    """Smooth sustained push, same raised-cosine ramp shape as the planar prototype.

    hold=2.0, not 0.5: with the corrected null-space centering (see
    FR3MPCConfig.k_null/d_null), the admittance generator's true reactive
    response to a 0.5 s hold does not approach the workspace bound at all --
    the original 0.5 s figure produced a large apparent reactive overshoot
    only because of the null-space leakage bug, not genuine physics. 2.0 s
    gives the admittance generator's velocity enough time to build up
    (T_a=0.3 s) to produce a real, checked boundary-crossing contrast
    between predictive and reactive under the corrected dynamics; impedance's
    story is unaffected by this change (its peak is set by its equilibrium,
    reached well within 0.5 s, confirmed empirically at both hold values)."""
    axis_vec = np.asarray(axis)
    t_off = t_on + ramp + hold
    t_end = t_off + ramp
    if t_on <= time < t_on + ramp:
        scale = 0.5 - 0.5 * np.cos(np.pi * (time - t_on) / ramp)
    elif t_on + ramp <= time < t_off:
        scale = 1.0
    elif t_off <= time < t_end:
        scale = 0.5 + 0.5 * np.cos(np.pi * (time - t_off) / ramp)
    else:
        scale = 0.0
    return magnitude * scale * axis_vec


def run_case(
    env: FR3MuJoCoEnv,
    generator,
    controller_kind: str,
    cfg: FR3MPCConfig,
    duration: float = 6.0,
):
    env.reset()
    dyn0, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()
    imp_params = make_default_impedance_params(cfg)

    mpc = FR3RealizationMPC(generator, cfg) if controller_kind == "mpc" else None
    mpc_every = max(1, round(cfg.dt / env.dt))

    n_steps = int(round(duration / env.dt))
    f_cmd_held = np.zeros(3)
    previous_command = np.zeros(3)

    log = {
        "time": [],
        "ee_pos": [],
        "ee_vel": [],
        "tau": [],
        "command": [],
        "human_force": [],
        "reference_acceleration": [],
        "predicted_realization_residual": [],
        "active": [],
        "solver_status": [],
        "solve_time_s": [],
        "slack_position": [],
        "slack_speed": [],
    }
    last_residual = 0.0
    last_active: list[str] = []
    last_status = "not_applicable"
    last_solve_time = 0.0
    last_slack_position = 0.0
    last_slack_speed = 0.0
    n_infeasible = 0
    n_solves_with_nontrivial_slack = 0
    slack_epsilon = 1e-6  # below this, treat slack as "not used" (solver/rounding noise)

    for i in range(n_steps):
        t = env.time
        force = human_force_at(t)
        dyn, state = env.get_dynamics_and_state(
            f_ext_override=np.concatenate([force, np.zeros(3)])
        )

        # Recomputed every tick for BOTH controllers -- the inner-loop-rate property.
        tau_base, J_v, d_known = compute_tau_base(dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg)

        if i % mpc_every == 0:
            if controller_kind == "mpc":
                forecast = np.tile(force, (cfg.horizon, 1))
                try:
                    step = mpc.control(dyn, state, p_nominal, R_d, forecast)
                    f_cmd_held = step.command
                    last_residual = step.predicted_residual_rms
                    last_active = list(step.active_constraints)
                    last_status = step.status
                    last_solve_time = step.solve_time_s
                    last_slack_position = step.horizon_slack_max_position
                    last_slack_speed = step.horizon_slack_max_speed
                    if step.horizon_slack_max > slack_epsilon:
                        n_solves_with_nontrivial_slack += 1
                except RuntimeError:
                    # Recursive feasibility is not guaranteed (paper.md
                    # Prop. 3): the frozen-Jacobian prediction and MuJoCo's
                    # real nonlinear dynamics can diverge enough, right at
                    # an active bound, that a solve becomes infeasible.
                    # Falling back to *zero* correction was tried first and
                    # made things worse: with no corrective force at all
                    # the disturbance pushes further, so the next solve is
                    # even more infeasible -- a cascading runaway. Falling
                    # back to the reactive/clipped instantaneous law instead
                    # is always well-defined and keeps applying *some*
                    # corrective force, avoiding the runaway.
                    f_cmd_held = fr3_clipped_reference_command(
                        generator, dyn, state, p_nominal, force, cfg, previous_command, d_known
                    )
                    last_active = ["infeasible_fallback"]
                    last_status = "infeasible_fallback"
                    last_solve_time = 0.0
                    last_slack_position = 0.0
                    last_slack_speed = 0.0
                    n_infeasible += 1
                previous_command = f_cmd_held
            elif controller_kind == "clipped":
                f_cmd_held = fr3_clipped_reference_command(
                    generator, dyn, state, p_nominal, force, cfg, previous_command, d_known
                )
                previous_command = f_cmd_held
                last_active = []
                last_status = "not_applicable"
                last_solve_time = 0.0
                last_slack_position = 0.0
                last_slack_speed = 0.0
            else:
                raise ValueError(controller_kind)

        tau = combine_full_torque(tau_base, J_v, f_cmd_held)

        x = np.concatenate([state.ee_pos - p_nominal, state.ee_vel[:3]])
        a_id = generator.acceleration(x, force)
        M_inv = np.linalg.inv(dyn.M)
        Lam_inv = J_v @ M_inv @ J_v.T + cfg.lambda_reg * np.eye(3)
        a_actual = Lam_inv @ (f_cmd_held + force) + d_known
        predicted_residual = a_actual - a_id

        log["time"].append(t)
        log["ee_pos"].append(state.ee_pos - p_nominal)
        log["ee_vel"].append(state.ee_vel[:3])
        log["tau"].append(tau.copy())
        log["command"].append(f_cmd_held.copy())
        log["human_force"].append(force.copy())
        log["reference_acceleration"].append(a_id.copy())
        log["predicted_realization_residual"].append(predicted_residual)
        log["active"].append(last_active)
        log["solver_status"].append(last_status)
        log["solve_time_s"].append(last_solve_time)
        log["slack_position"].append(last_slack_position)
        log["slack_speed"].append(last_slack_speed)

        env.apply_torque(tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        env.step()

    for key in (
        "time",
        "ee_pos",
        "ee_vel",
        "tau",
        "command",
        "human_force",
        "reference_acceleration",
        "predicted_realization_residual",
        "slack_position",
        "slack_speed",
    ):
        log[key] = np.asarray(log[key])
    # Empirical task acceleration from the MuJoCo trajectory. This includes
    # configuration change, J-dot effects, integration error, and every torque
    # actually applied to the plant; unlike predicted_realization_residual it
    # is not reconstructed through the controller's frozen/local model.
    empirical_acceleration = np.full_like(log["reference_acceleration"], np.nan)
    empirical_acceleration[:-1] = np.diff(log["ee_vel"], axis=0) / env.dt
    log["empirical_acceleration"] = empirical_acceleration
    log["empirical_realization_residual"] = (
        empirical_acceleration - log["reference_acceleration"]
    )
    # Backward-compatible name now means the quantity the paper calls the
    # physically realized residual.
    log["realization_residual"] = log["empirical_realization_residual"]
    log["n_infeasible"] = n_infeasible
    log["n_solves_with_nontrivial_slack"] = n_solves_with_nontrivial_slack
    return log


def metrics(log, cfg: FR3MPCConfig) -> dict:
    position = log["ee_pos"]
    velocity = log["ee_vel"]
    tau = log["tau"]
    residual = log["empirical_realization_residual"]
    predicted_residual = log["predicted_realization_residual"]
    valid_empirical = np.all(np.isfinite(residual), axis=1)
    meaningful_active_steps = sum(
        any(label != "slack_nonneg" for label in items) for items in log["active"]
    )
    torque_active_steps = sum(
        any(label.startswith("torque[") for label in items) for items in log["active"]
    )
    solve_times = [t for t in log["solve_time_s"] if t > 0.0]
    max_per_joint = np.max(np.abs(tau), axis=0)
    torque_utilization = max_per_joint / cfg.tau_max
    slack_position = log["slack_position"]
    slack_speed = log["slack_speed"]
    # These are vector-norm RMSE: the squared per-sample 3-vector residual is
    # summed across x/y/z, then averaged over time, then sqrt'd. The planar
    # study's realization_rmse_mps2 (run_experiments.py) is componentwise
    # RMSE instead: it flattens all components and time samples into one mean
    # before the sqrt. The two are not the same quantity and are not directly
    # comparable across studies -- see paper.md's Table 1/2 captions, which
    # label each one explicitly for this reason.
    return {
        "realization_rmse_mps2": float(
            np.sqrt(np.mean(np.sum(residual[valid_empirical] ** 2, axis=1)))
        ),
        "empirical_realization_rmse_mps2": float(
            np.sqrt(np.mean(np.sum(residual[valid_empirical] ** 2, axis=1)))
        ),
        "predicted_realization_rmse_mps2": float(
            np.sqrt(np.mean(np.sum(predicted_residual**2, axis=1)))
        ),
        "max_position_slack_m": float(np.max(slack_position)) if len(slack_position) else 0.0,
        "max_speed_slack_mps": float(np.max(slack_speed)) if len(slack_speed) else 0.0,
        "n_solves_with_nontrivial_slack": int(log.get("n_solves_with_nontrivial_slack", 0)),
        "max_abs_position_m": float(np.max(np.abs(position))),
        "max_speed_mps": float(np.max(np.abs(velocity))),
        "max_speed_norm_mps": float(np.max(np.linalg.norm(velocity, axis=1))),
        "max_abs_torque_Nm": float(np.max(np.abs(tau))),
        "max_abs_torque_per_joint_Nm": max_per_joint.tolist(),
        "max_torque_utilization": float(np.max(torque_utilization)),
        "max_torque_utilization_per_joint": torque_utilization.tolist(),
        "max_abs_command_N": float(np.max(np.abs(log["command"]))),
        "position_violation_m": float(max(0.0, np.max(np.abs(position)) - cfg.position_limit)),
        "speed_violation_mps": float(
            max(0.0, np.max(np.abs(velocity)) - cfg.speed_limit)
        ),
        "torque_violation_Nm": float(max(0.0, np.max(np.abs(tau) - cfg.tau_max[None, :]))),
        "active_constraint_steps": int(meaningful_active_steps),
        "torque_constraint_active_steps": int(torque_active_steps),
        "mean_solve_time_ms": float(np.mean(solve_times) * 1e3) if solve_times else 0.0,
        "max_solve_time_ms": float(np.max(solve_times) * 1e3) if solve_times else 0.0,
        "final_z_m": float(position[-1, 2]),
        "peak_z_m": float(np.min(position[:, 2])),  # push is in -z
        "n_infeasible_solves": int(log.get("n_infeasible", 0)),
    }


def make_figure(cases, cfg: FR3MPCConfig, output: Path) -> None:
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
                time, log["ee_pos"][:, 2], style, color=color,
                label=labels[controller_kind], linewidth=2,
            )
            axes[1, column].plot(time, log["ee_vel"][:, 2], style, color=color, linewidth=2)
            axes[2, column].plot(time, log["command"][:, 2], style, color=color, linewidth=2)
            axes[3, column].plot(
                time, np.linalg.norm(log["empirical_realization_residual"], axis=1),
                style, color=color, linewidth=2,
            )

        axes[0, column].axhline(-cfg.position_limit, color="0.3", linestyle=":")
        axes[0, column].axhline(cfg.position_limit, color="0.3", linestyle=":")
        axes[1, column].axhline(-cfg.speed_limit, color="0.3", linestyle=":")
        axes[1, column].axhline(cfg.speed_limit, color="0.3", linestyle=":")
        axes[0, column].set_title(f"{generator_name.capitalize()} generator")
        axes[3, column].set_xlabel("Time (s)")
        for row in range(4):
            axes[row, column].grid(alpha=0.25)

    axes[0, 0].set_ylabel("EE z-displacement (m)")
    axes[1, 0].set_ylabel("EE z-velocity (m/s)")
    axes[2, 0].set_ylabel("Correction force z (N)")
    axes[3, 0].set_ylabel(r"$\|a-a^{id}\|$ (m/s²)")
    axes[0, 0].legend(loc="best")
    fig.suptitle(
        "One predictive realization layer on a real FR3: two interaction dynamics, "
        "horizon-wide torque feasibility",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=200)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results",
    )
    parser.add_argument("--duration", type=float, default=6.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cfg = FR3MPCConfig()
    env = FR3MuJoCoEnv(timestep=0.001)
    generators = (ImpedanceReference3D(), AdmittanceReference3D())
    cases = {}
    report = {
        "configuration": {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(cfg).items()},
        "force_forecast": "measured force held constant over the horizon",
        "generators": {g.name: asdict(g) for g in generators},
        "cases": {},
    }

    t_start = _time.perf_counter()
    for generator in generators:
        for controller_kind in ("mpc", "clipped"):
            key = (generator.name, controller_kind)
            cases[key] = run_case(env, generator, controller_kind, cfg, duration=args.duration)
            report["cases"][f"{generator.name}_{controller_kind}"] = metrics(cases[key], cfg)
    wall_time = _time.perf_counter() - t_start

    make_figure(cases, cfg, args.output_dir / "fr3_interaction_dynamics_results.png")
    report["wall_time_s"] = wall_time
    with (args.output_dir / "fr3_metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)

    print(json.dumps(report["cases"], indent=2))
    print(f"Saved results to {args.output_dir} ({wall_time:.1f}s wall time)")


if __name__ == "__main__":
    main()
