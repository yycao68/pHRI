"""Run every simulation experiment for the predictive-saturation draft."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from saturation_benchmark import (
    BenchmarkConfig,
    RunOptions,
    make_controllers,
    make_robots,
    make_scenarios,
    run_case,
    serializable_config,
    summarize,
)


METHODS = (
    "nominal_unbounded",
    "clipping",
    "reactive_filter",
    "reference_governor",
    "proposed",
)

COLORS = {
    "nominal_unbounded": "#6b7280",
    "clipping": "#d97706",
    "reactive_filter": "#059669",
    "reference_governor": "#7c3aed",
    "proposed": "#2563eb",
}

METHOD_LABELS = {
    "nominal_unbounded": "nominal diagnostic (no torque projection)",
    "clipping": "clipping",
    "reactive_filter": "reactive projection",
    "reference_governor": "reference governor + projection",
    "proposed": "proposed",
}

SCENARIO_LABELS = {
    "slow_saturation": "slow\nsaturation",
    "sudden_disturbance": "sudden\ndisturbance",
    "directional_collapse": "directional\ncollapse",
    "point_of_no_return": "near-boundary\nbraking",
    "model_mismatch": "model\nmismatch",
    "preview_mismatch": "preview\nmismatch",
}


def _case_key(*parts: str) -> str:
    return "__".join(parts)


def _run(
    raw: dict,
    metrics: dict,
    key: str,
    robot,
    controller,
    scenario,
    cfg,
    options,
    *,
    keep_raw: bool = False,
):
    log = run_case(robot, controller, scenario, cfg, options)
    metrics[key] = summarize(log, cfg)
    if keep_raw:
        raw[key] = log
    return log


def scenario_matrix(cfg, robots, controllers, scenarios):
    metrics, raw = {}, {}
    robot = robots["fr3_surrogate"]
    controller = controllers["impedance"]
    for scenario_name, scenario in scenarios.items():
        for method in METHODS:
            key = _case_key("scenario", scenario_name, method)
            _run(
                raw,
                metrics,
                key,
                robot,
                controller,
                scenario,
                cfg,
                RunOptions(method=method),
                keep_raw=scenario_name
                in ("directional_collapse", "sudden_disturbance", "point_of_no_return"),
            )
    return metrics, raw


def controller_transfer_matrix(cfg, robots, controllers, scenarios):
    metrics = {}
    robot = robots["fr3_surrogate"]
    for controller_name, controller in controllers.items():
        for scenario_name in ("no_saturation", "slow_saturation", "sudden_disturbance"):
            scenario = scenarios[scenario_name]
            # The full method comparison is already run with impedance in the
            # scenario matrix.  Here the scientific question is whether the
            # same manager interface accepts every controller, so clipping and
            # proposed are sufficient and avoid duplicating 1 kHz baselines.
            for method in ("clipping", "proposed"):
                key = _case_key(
                    "controller", controller_name, scenario_name, method
                )
                log = run_case(
                    robot, controller, scenario, cfg, RunOptions(method=method)
                )
                metrics[key] = summarize(log, cfg)
    return metrics


def robot_transfer_matrix(cfg, robots, controllers, scenarios):
    metrics = {}
    for robot_name, robot in robots.items():
        for controller_name in ("impedance", "rl_policy"):
            controller = controllers[controller_name]
            for scenario_name in ("no_saturation", "slow_saturation"):
                scenario = scenarios[scenario_name]
                for method in ("clipping", "proposed"):
                    key = _case_key(
                        "robot",
                        robot_name,
                        controller_name,
                        scenario_name,
                        method,
                    )
                    log = run_case(
                        robot,
                        controller,
                        scenario,
                        cfg,
                        RunOptions(method=method),
                    )
                    metrics[key] = summarize(log, cfg)
    return metrics


def ablation_matrix(cfg, robots, controllers, scenarios):
    robot = robots["fr3_surrogate"]
    controller = controllers["impedance"]
    metrics = {}
    experiments = (
        ("horizon_ramp", "full", RunOptions(method="proposed")),
        (
            "horizon_ramp",
            "first_step_torque",
            RunOptions(method="proposed", constraint_steps=1),
        ),
        (
            "slow_saturation",
            "cached_full_torque",
            RunOptions(method="proposed", cached_torque=True),
        ),
        (
            "sudden_disturbance",
            "full",
            RunOptions(method="proposed"),
        ),
        (
            "sudden_disturbance",
            "no_final_projection",
            RunOptions(method="proposed", final_projection=False),
        ),
        ("model_mismatch", "full", RunOptions(method="proposed")),
        ("directional_collapse", "full", RunOptions(method="proposed")),
        (
            "directional_collapse",
            "no_tightening",
            RunOptions(method="proposed", tightening=False),
        ),
        (
            "model_mismatch",
            "frozen_realization_map",
            RunOptions(method="proposed", map_mode="frozen"),
        ),
        (
            "model_mismatch",
            "updated_realization_map",
            RunOptions(method="proposed", map_mode="updated"),
        ),
        ("preview_mismatch", "full", RunOptions(method="proposed")),
        (
            "preview_mismatch",
            "preview_zoh",
            RunOptions(method="proposed", preview_mode="zoh"),
        ),
        (
            "preview_mismatch",
            "preview_zero_force",
            RunOptions(method="proposed", preview_mode="zero"),
        ),
        (
            "preview_mismatch",
            "preview_oracle_force",
            RunOptions(method="proposed", preview_mode="oracle"),
        ),
    )
    for scenario_name, variant_name, options in experiments:
        key = _case_key("ablation", scenario_name, variant_name)
        log = run_case(
            robot,
            controller,
            scenarios[scenario_name],
            cfg,
            options,
        )
        metrics[key] = summarize(log, cfg)
    return metrics


def sampled_refinement_audit(cfg, robot_metrics, robots):
    rows = {}
    for robot_name in robots:
        selected = [
            value
            for key, value in robot_metrics.items()
            if key.startswith(f"robot__{robot_name}__")
            and key.endswith("__proposed")
        ]
        peak_defect = max(v["refinement_defect_peak_mps"] for v in selected)
        min_torque_bound = min(
            v["minimum_error_bound_residual_Nm"] for v in selected
        )
        min_error_bound = min(
            v["torque_error_bound_min_Nm"] for v in selected
        )
        max_error_bound = max(
            v["torque_error_bound_max_Nm"] for v in selected
        )
        min_planned_margin = min(
            v["minimum_planned_torque_margin_Nm"] for v in selected
        )
        rows[robot_name] = {
            "shared_audit_config_hash": cfg.audit_config_hash(),
            "shared_successor_defect_budget_mps": cfg.successor_defect_budget,
            "required_empirical_radius_mps": peak_defect,
            "unused_radius_mps": cfg.successor_defect_budget - peak_defect,
            "all_successor_defects_contained": all(
                v["sampled_defect_check_satisfied"] for v in selected
            ),
            "minimum_error_bound_residual_Nm": min_torque_bound,
            "sampled_torque_error_bound_range_Nm": [
                min_error_bound,
                max_error_bound,
            ],
            "minimum_planned_actuator_margin_Nm": min_planned_margin,
            "all_torque_errors_contained": all(
                v["torque_error_bound_satisfied"] for v in selected
            ),
            "all_sampled_refinement_checks_satisfied": all(
                v["sampled_refinement_checks_satisfied"]
                for v in selected
            ),
            "robot_specific_objects": [
                "Pi_r",
                "tau_hat_r",
                "D_tau_r",
                "D_z_r",
                "A_tight_r",
            ],
        }
    return rows


def _save_raw_npz(raw: dict, output: Path):
    payload = {}
    for key, log in raw.items():
        for field, value in log.items():
            if isinstance(value, np.ndarray):
                payload[f"{key}__{field}"] = value
    np.savez_compressed(output, **payload)


def _make_stress_case_figure(
    raw: dict,
    cfg: BenchmarkConfig,
    scenario: str,
    position_index: int,
    position_label: str,
    title: str,
    output: Path,
):
    methods = ("clipping", "reactive_filter", "reference_governor", "proposed")
    fig, axes = plt.subplots(4, 1, figsize=(9.0, 9.5), sharex=True)
    for method in methods:
        log = raw[_case_key("scenario", scenario, method)]
        t = log["time"]
        color = COLORS[method]
        label = METHOD_LABELS[method]
        ratio = np.max(
            np.abs(log["torque_pre"]) / np.maximum(log["torque_limit"], 1.0e-9),
            axis=1,
        )
        axes[0].plot(t, ratio, color=color, label=label, linewidth=1.6)
        axes[1].plot(t, log["state"][:, position_index], color=color, linewidth=1.6)
        axes[2].plot(
            t,
            np.linalg.norm(log["correction"], axis=1),
            color=color,
            linewidth=1.6,
        )
        axes[3].plot(
            t,
            log["directional_authority"],
            color=color,
            linewidth=1.6,
        )
    axes[0].axhline(1.0, color="black", linestyle=":", linewidth=1.0)
    axes[0].set_ylabel("pre-clip\nutilization")
    axes[1].axhline(cfg.position_limit, color="black", linestyle=":")
    axes[1].set_ylabel(position_label)
    axes[2].set_ylabel(r"$\|\Delta a\|$" + "\n" + r"(m/s$^2$)")
    axes[3].set_ylabel("directional\nauthority")
    axes[3].set_xlabel("time (s)")
    axes[0].legend(ncol=2, fontsize=8, loc="upper left")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def make_representative_figure(raw: dict, cfg: BenchmarkConfig, output: Path):
    _make_stress_case_figure(
        raw,
        cfg,
        "directional_collapse",
        1,
        r"$e_y$ (m)",
        "Directional-authority stress case: same 1 kHz impedance controller",
        output,
    )


def make_point_of_no_return_figure(raw: dict, cfg: BenchmarkConfig, output: Path):
    _make_stress_case_figure(
        raw,
        cfg,
        "point_of_no_return",
        0,
        r"$e_x$ (m)",
        "Near-boundary braking stress case: same 1 kHz impedance controller",
        output,
    )


def make_scenario_summary_figure(metrics: dict, output: Path):
    scenarios = (
        "slow_saturation",
        "sudden_disturbance",
        "directional_collapse",
        "point_of_no_return",
        "model_mismatch",
        "preview_mismatch",
    )
    methods = ("clipping", "reactive_filter", "reference_governor", "proposed")
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    x = np.arange(len(scenarios))
    width = 0.19
    for j, method in enumerate(methods):
        vals_tau = [
            metrics[_case_key("scenario", s, method)][
                "peak_preclip_torque_violation_Nm"
            ]
            for s in scenarios
        ]
        vals_pos = [
            1000.0
            * metrics[_case_key("scenario", s, method)][
                "peak_position_violation_m"
            ]
            for s in scenarios
        ]
        offset = (j - 1.5) * width
        axes[0].bar(
            x + offset,
            vals_tau,
            width,
            label=METHOD_LABELS[method],
            color=COLORS[method],
        )
        axes[1].bar(x + offset, vals_pos, width, color=COLORS[method])
    labels = [SCENARIO_LABELS[s] for s in scenarios]
    axes[0].set_xticks(x, labels, fontsize=8)
    axes[1].set_xticks(x, labels, fontsize=8)
    axes[0].set_ylabel("peak pre-clip torque excess (N m)")
    axes[1].set_ylabel("peak workspace excess (mm)")
    axes[0].legend(fontsize=7, ncol=2)
    for ax in axes:
        ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def make_controller_heatmap(metrics: dict, controllers: dict, output: Path):
    names = list(controllers)
    scenarios = ("no_saturation", "slow_saturation", "sudden_disturbance")
    proposed = np.zeros((len(names), len(scenarios)))
    clipping = np.zeros_like(proposed)
    for i, controller in enumerate(names):
        for j, scenario in enumerate(scenarios):
            proposed[i, j] = metrics[
                _case_key("controller", controller, scenario, "proposed")
            ]["behavior_realization_rmse_mps2"]
            clipping[i, j] = metrics[
                _case_key("controller", controller, scenario, "clipping")
            ]["behavior_realization_rmse_mps2"]
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.6), sharey=True)
    vmax = max(float(np.max(proposed)), float(np.max(clipping)), 1.0e-6)
    for ax, data, title in zip(
        axes, (clipping, proposed), ("clipping", "predictive manager")
    ):
        image = ax.imshow(data, cmap="magma", aspect="auto", vmin=0.0, vmax=vmax)
        ax.set_xticks(range(len(scenarios)), [s.replace("_", "\n") for s in scenarios])
        ax.set_yticks(range(len(names)), [n.replace("_", " ") for n in names])
        ax.set_title(title)
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                ax.text(j, i, f"{data[i,j]:.2f}", ha="center", va="center", color="white", fontsize=8)
    fig.subplots_adjust(left=0.18, right=0.88, bottom=0.16, top=0.88, wspace=0.18)
    color_axis = fig.add_axes((0.905, 0.16, 0.018, 0.72))
    fig.colorbar(
        image,
        cax=color_axis,
        label=r"realization RMSE (m/s$^2$)",
    )
    fig.savefig(output, dpi=180)
    plt.close(fig)


def make_robot_transfer_figure(audit: dict, output: Path):
    names = list(audit)
    required = [audit[n]["required_empirical_radius_mps"] for n in names]
    shared = [
        audit[n]["shared_successor_defect_budget_mps"] for n in names
    ]
    x = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(7.3, 4.3))
    ax.bar(x, required, color="#2563eb", label="observed successor defect")
    ax.plot(x, shared, "ko--", label="common sampled-defect budget")
    ax.set_xticks(x, [n.replace("_", "\n") for n in names])
    ax.set_ylabel("velocity-successor defect (m/s)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def make_ablation_figure(
    metrics: dict, scenario_metrics: dict, output: Path
):
    fig, axes = plt.subplots(2, 3, figsize=(12.0, 7.2))

    def bars(ax, labels, values, ylabel, title, color="#2563eb"):
        x = np.arange(len(labels))
        ax.bar(x, values, color=color)
        ax.set_xticks(x, labels, fontsize=8)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.25)

    bars(
        axes[0, 0],
        ["all horizon", "first step"],
        [
            metrics["ablation__horizon_ramp__full"][
                "maximum_planned_torque_violation_Nm"
            ],
            metrics["ablation__horizon_ramp__first_step_torque"][
                "maximum_planned_torque_violation_Nm"
            ],
        ],
        "planned excess (N m)",
        "Horizon-wide torque rows",
    )
    bars(
        axes[0, 1],
        ["tightened", "not tightened"],
        [
            metrics["ablation__directional_collapse__full"][
                "peak_preclip_torque_violation_Nm"
            ],
            metrics["ablation__directional_collapse__no_tightening"][
                "peak_preclip_torque_violation_Nm"
            ],
        ],
        "pre-clip excess (N m)",
        "Uncertainty tightening",
        color="#059669",
    )
    bars(
        axes[0, 2],
        ["projection on", "projection off"],
        [
            metrics["ablation__sudden_disturbance__full"][
                "peak_applied_torque_violation_Nm"
            ],
            metrics["ablation__sudden_disturbance__no_final_projection"][
                "peak_applied_torque_violation_Nm"
            ],
        ],
        "applied excess (N m)",
        "Final 1 kHz projection",
        color="#dc2626",
    )
    preview_labels = ["rollout", "ZOH", "zero force", "oracle force"]
    preview_keys = [
        "ablation__preview_mismatch__full",
        "ablation__preview_mismatch__preview_zoh",
        "ablation__preview_mismatch__preview_zero_force",
        "ablation__preview_mismatch__preview_oracle_force",
    ]
    bars(
        axes[1, 0],
        preview_labels,
        [1000.0 * metrics[k]["peak_position_violation_m"] for k in preview_keys],
        "workspace excess (mm)",
        "Preview mismatch (outside viability)",
        color="#d97706",
    )
    bars(
        axes[1, 1],
        ["fast remap", "cached torque"],
        [
            scenario_metrics["scenario__slow_saturation__proposed"][
                "refinement_defect_peak_mps"
            ],
            metrics["ablation__slow_saturation__cached_full_torque"][
                "refinement_defect_peak_mps"
            ],
        ],
        "successor defect (m/s)",
        "Slow-to-fast implementation",
        color="#7c3aed",
    )
    bars(
        axes[1, 2],
        ["updated map", "frozen map"],
        [
            metrics["ablation__model_mismatch__updated_realization_map"][
                "refinement_defect_peak_mps"
            ],
            metrics["ablation__model_mismatch__frozen_realization_map"][
                "refinement_defect_peak_mps"
            ],
        ],
        "successor defect (m/s)",
        "Realization-map update",
        color="#0891b2",
    )
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def timing_summary(*metric_groups: dict) -> dict:
    proposed = []
    fast = []
    for group in metric_groups:
        for key, value in group.items():
            if key.endswith("__proposed") or "__full" in key:
                proposed.append(value["manager_time_ms"])
                fast.append(value["fast_time_us"])
    return {
        "manager_ms_across_cases": {
            "median_of_medians": float(np.median([v["median"] for v in proposed])),
            "worst_p95": float(max(v["p95"] for v in proposed)),
            "worst_p99": float(max(v["p99"] for v in proposed)),
            "worst_max": float(max(v["max"] for v in proposed)),
            "nominal_deadline_ms": 20.0,
        },
        "fast_path_us_across_cases": {
            "median_of_medians": float(np.median([v["median"] for v in fast])),
            "worst_p95": float(max(v["p95"] for v in fast)),
            "worst_p99": float(max(v["p99"] for v in fast)),
            "worst_max": float(max(v["max"] for v in fast)),
            "nominal_deadline_us": 1000.0,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "results",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cfg = BenchmarkConfig()
    robots = make_robots()
    controllers = make_controllers()
    scenarios = make_scenarios()

    scenario_metrics, raw = scenario_matrix(cfg, robots, controllers, scenarios)
    print(f"completed scenario matrix: {len(scenario_metrics)}", flush=True)
    controller_metrics = controller_transfer_matrix(
        cfg, robots, controllers, scenarios
    )
    print(f"completed controller transfer: {len(controller_metrics)}", flush=True)
    robot_metrics = robot_transfer_matrix(cfg, robots, controllers, scenarios)
    print(f"completed robot transfer: {len(robot_metrics)}", flush=True)
    ablation_metrics = ablation_matrix(cfg, robots, controllers, scenarios)
    print(f"completed ablations: {len(ablation_metrics)}", flush=True)
    audit = sampled_refinement_audit(cfg, robot_metrics, robots)
    timing = timing_summary(
        scenario_metrics, controller_metrics, robot_metrics, ablation_metrics
    )

    report = {
        "study_scope": (
            "deterministic reduced-order 2-D simulation; FR3/arm6 cases are "
            "actuator-geometry surrogates, not rigid-body or hardware validation"
        ),
        "configuration": serializable_config(cfg),
        "shared_audit_config_hash": cfg.audit_config_hash(),
        "robots": {
            name: {
                "n_act": robot.n_act,
                "torque_limits": robot.torque_limits.tolist(),
                "mass": robot.mass,
            }
            for name, robot in robots.items()
        },
        "controllers": {
            name: {"class": controller.__class__.__name__}
            for name, controller in controllers.items()
        },
        "scenarios": {
            name: {
                "description": scenario.description,
                "mismatch_scale": scenario.mismatch_scale,
                "initial_state": list(scenario.initial_state),
            }
            for name, scenario in scenarios.items()
        },
        "scenario_comparison": scenario_metrics,
        "controller_transfer": controller_metrics,
        "robot_transfer": robot_metrics,
        "ablations": ablation_metrics,
        "sampled_refinement_audit": audit,
        "timing": timing,
        "policy_training": {
            "rl_policy": (
                "deterministic small evolution-strategy policy search on a "
                "nominal double integrator; not a deep-RL benchmark"
            ),
            "neural_policy": (
                "fixed-hidden-layer neural policy fit to impedance-controller "
                "demonstrations"
            ),
            "ai_conditioned_proxy": (
                "scripted behavior primitive plus PD executor; software-interface "
                "case only, not an AI-performance result"
            ),
        },
    }
    with (args.output_dir / "all_experiment_metrics.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(report, handle, indent=2)

    _save_raw_npz(raw, args.output_dir / "representative_logs.npz")
    make_representative_figure(
        raw, cfg, args.output_dir / "directional_authority_results.png"
    )
    make_point_of_no_return_figure(
        raw, cfg, args.output_dir / "near_boundary_braking_results.png"
    )
    make_scenario_summary_figure(
        scenario_metrics, args.output_dir / "scenario_summary.png"
    )
    make_controller_heatmap(
        controller_metrics,
        controllers,
        args.output_dir / "controller_transfer.png",
    )
    make_robot_transfer_figure(
        audit, args.output_dir / "sampled_refinement_audit.png"
    )
    make_ablation_figure(
        ablation_metrics,
        scenario_metrics,
        args.output_dir / "ablation_summary.png",
    )

    print(
        json.dumps(
            {
                "scenario_cases": len(scenario_metrics),
                "controller_cases": len(controller_metrics),
                "robot_cases": len(robot_metrics),
                "ablation_cases": len(ablation_metrics),
                "sampled_refinement_audit": audit,
                "timing": timing,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
