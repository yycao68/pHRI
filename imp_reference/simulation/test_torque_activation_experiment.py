"""End-to-end checks for paper.md Section 6.4's torque-active ablation."""

from functools import lru_cache

from run_torque_activation_experiment import run_experiment


@lru_cache(maxsize=1)
def _report():
    _, report = run_experiment()
    return report


def test_horizon_wide_plan_respects_derated_budget():
    cases = _report()["cases"]
    assert cases["horizon_wide"]["max_planned_torque_violation_Nm"] < 0.01
    assert cases["horizon_wide"]["torque_constraint_active_steps"] > 0


def test_first_step_only_plan_is_future_infeasible():
    cases = _report()["cases"]
    assert cases["first_step_only"]["max_planned_torque_violation_Nm"] > 5.0


def test_horizon_wide_runtime_reduces_executed_mismatch_and_displacement():
    cases = _report()["cases"]
    full = cases["horizon_wide"]
    first = cases["first_step_only"]
    assert full["torque_violation_Nm"] < first["torque_violation_Nm"]
    assert full["max_abs_position_m"] < first["max_abs_position_m"]
