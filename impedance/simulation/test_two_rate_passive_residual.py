import numpy as np

from verify_two_rate_passive_residual import Config, ResidualMPC, run_trial


def test_impedance_causal_residual_model_changes_with_rendered_stiffness():
    low = ResidualMPC(Config(stiffness=40.0))
    high = ResidualMPC(Config(stiffness=180.0))
    assert not np.allclose(low.a, high.a)
    assert not np.allclose(low.h, high.h)


def test_fast_projection_preserves_tank_lower_bound_and_actuator_limit():
    cfg = Config()
    result = run_trial("fast_guard", cfg, seed=4)
    assert result["metrics"]["minimum_tank_j"] >= cfg.tank_minimum - 1e-12
    assert np.max(np.abs(result["log"]["applied"])) <= cfg.force_limit + 1e-12
    assert result["metrics"]["qp_failures"] == 0


def test_manager_only_authorization_misses_inter_update_energy_violation():
    cfg = Config()
    result = run_trial("manager_guard", cfg, seed=4)
    assert result["metrics"]["tank_violation_j"] > 0.0

