"""Regression tests for the complete saturation experiment suite."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from saturation_benchmark import (
    BenchmarkConfig,
    RunOptions,
    Scenario,
    make_controllers,
    make_robots,
    make_scenarios,
    run_case,
    summarize,
)


ROOT = Path(__file__).resolve().parents[1]


def _objects():
    return make_robots(), make_controllers(), make_scenarios(), BenchmarkConfig()


def test_all_controller_interfaces_are_finite_and_deterministic():
    robots, controllers, scenarios, cfg = _objects()
    robot = robots["fr3_surrogate"]
    short_cfg = BenchmarkConfig(duration=0.25)
    for controller in controllers.values():
        log = run_case(
            robot,
            controller,
            scenarios["no_saturation"],
            short_cfg,
            RunOptions(method="proposed"),
        )
        assert np.all(np.isfinite(log["state"]))
        assert np.all(np.isfinite(log["requested_acceleration"]))


def test_no_saturation_case_leaves_nominal_behavior_unchanged():
    robots, controllers, scenarios, cfg = _objects()
    log = run_case(
        robots["fr3_surrogate"],
        controllers["impedance"],
        scenarios["no_saturation"],
        cfg,
        RunOptions(method="proposed"),
    )
    metrics = summarize(log, cfg)
    assert metrics["correction_rmse_mps2"] < 0.01
    assert metrics["peak_preclip_torque_violation_Nm"] == 0.0
    assert metrics["sampled_interface_audit_passed"]


def test_horizon_wide_rows_prevent_infeasible_future_plan():
    robots, controllers, scenarios, cfg = _objects()
    common = (
        robots["fr3_surrogate"],
        controllers["impedance"],
        scenarios["horizon_ramp"],
        cfg,
    )
    full = summarize(
        run_case(*common, RunOptions(method="proposed")), cfg
    )
    first = summarize(
        run_case(
            *common,
            RunOptions(method="proposed", constraint_steps=1),
        ),
        cfg,
    )
    assert full["maximum_planned_torque_violation_Nm"] < 1.0e-4
    assert first["maximum_planned_torque_violation_Nm"] > 1.0
    assert full["peak_position_violation_m"] < first["peak_position_violation_m"]


def test_uncertainty_tightening_removes_preclip_excess():
    robots, controllers, scenarios, cfg = _objects()
    common = (
        robots["fr3_surrogate"],
        controllers["impedance"],
        scenarios["directional_collapse"],
        cfg,
    )
    tight = summarize(
        run_case(*common, RunOptions(method="proposed", tightening=True)),
        cfg,
    )
    untight = summarize(
        run_case(*common, RunOptions(method="proposed", tightening=False)),
        cfg,
    )
    assert tight["peak_preclip_torque_violation_Nm"] < 1.0e-4
    assert untight["peak_preclip_torque_violation_Nm"] > 0.02


def test_final_projection_is_required_for_impulsive_case():
    robots, controllers, scenarios, cfg = _objects()
    common = (
        robots["fr3_surrogate"],
        controllers["impedance"],
        scenarios["sudden_disturbance"],
        cfg,
    )
    protected = summarize(
        run_case(*common, RunOptions(method="proposed")), cfg
    )
    unprotected = summarize(
        run_case(
            *common,
            RunOptions(method="proposed", final_projection=False),
        ),
        cfg,
    )
    assert protected["peak_applied_torque_violation_Nm"] < 1.0e-9
    assert unprotected["peak_applied_torque_violation_Nm"] > 1.0


def test_certified_action_set_bounds_speed_below_uncertified_case():
    robots, controllers, scenarios, cfg = _objects()
    zero = lambda _t: np.zeros(2)
    scenario = Scenario(
        "certificate_margin",
        (0.0, 0.0, 0.565, 0.0),
        lambda _t: np.array([0.5, 0.0]),
        zero,
        lambda _t: 1.0,
        description="K_cert regression check",
    )
    common = (robots["fr3_surrogate"], controllers["impedance"], scenario, cfg)
    constrained = summarize(
        run_case(*common, RunOptions(method="proposed", certificate_constrained=True)),
        cfg,
    )
    unconstrained = summarize(
        run_case(*common, RunOptions(method="proposed", certificate_constrained=False)),
        cfg,
    )
    initial_speed = scenario.initial_state[2]
    certified_limit = cfg.speed_limit - cfg.velocity_defect_radius
    assert initial_speed < certified_limit
    assert constrained["peak_speed_mps"] <= certified_limit + 1.0e-9
    assert unconstrained["peak_speed_mps"] > certified_limit + 1.0e-3
    assert unconstrained["peak_speed_mps"] < cfg.speed_limit + 1.0e-9


def test_same_sampled_interface_audit_holds_across_robots():
    robots, controllers, scenarios, cfg = _objects()
    for robot in robots.values():
        log = run_case(
            robot,
            controllers["impedance"],
            scenarios["slow_saturation"],
            cfg,
            RunOptions(method="proposed"),
        )
        metrics = summarize(log, cfg)
        assert metrics["torque_error_bound_satisfied"]
        assert metrics["sampled_interface_audit_passed"]


def test_saved_report_covers_every_required_experiment_family():
    report_path = ROOT / "results" / "all_experiment_metrics.json"
    assert report_path.exists()
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert len(report["scenario_comparison"]) == 40
    assert len(report["controller_transfer"]) == 30
    assert len(report["robot_transfer"]) == 24
    assert len(report["ablations"]) == 17
    assert len(report["sampled_interface_audit"]) == 3
    assert len(
        {
            row["shared_audit_config_hash"]
            for row in report["sampled_interface_audit"].values()
        }
    ) == 1
