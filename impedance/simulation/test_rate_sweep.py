import numpy as np

from rate_sweep import representative_diagnostic
from verify_fr3_two_rate_benchmark import Config, leakage_sweep


def test_representative_diagnostic_reports_sane_activation_and_chatter_fields():
    cfg = Config(duration=1.8, manager_dt=0.02, horizon=10)
    diag = representative_diagnostic(cfg, seed=4)
    assert diag["manager_updates_total"] > 0
    assert 0.0 <= diag["fraction_active"] <= 1.0
    assert diag["total_variation_of_applied_residual_N"] >= 0.0
    assert diag["activation_events"] >= 0


def test_faster_manager_does_not_change_leakage_sweep_invariants():
    cfg = Config(duration=1.8, manager_dt=0.01, horizon=20)
    result = leakage_sweep(cfg, seeds=2)
    for row in result["rows"]:
        assert row["minimum_tank_j_min"] >= cfg.tank_minimum - 1e-12
        assert row["maximum_torque_ratio"] <= 1.0 + 1e-8
        assert row["qp_failures"] == 0
