import numpy as np

from verify_residual_mpc import Config, ResidualMPC, run_trial


def test_prediction_model_and_gain_do_not_depend_on_impedance_stiffness():
    low = ResidualMPC(Config(desired_stiffness=40.0))
    high = ResidualMPC(Config(desired_stiffness=240.0))
    np.testing.assert_allclose(low.a, high.a, atol=0.0, rtol=0.0)
    np.testing.assert_allclose(low.b, high.b, atol=0.0, rtol=0.0)
    np.testing.assert_allclose(low.h, high.h, atol=0.0, rtol=0.0)
    np.testing.assert_allclose(low.first_move_gain, high.first_move_gain, atol=0.0, rtol=0.0)


def test_matched_intentional_force_preserves_impedance_reference():
    result = run_trial("residual_mpc", Config(), seed=1, disturbance=False)
    assert result["metrics"]["impedance_fidelity_rms_mm"] < 1e-10
    assert result["metrics"]["qp_failures"] == 0


def test_total_input_constraint_is_respected_at_manager_samples():
    cfg = Config()
    result = run_trial("residual_mpc", cfg, seed=2, disturbance=True)
    applied = np.asarray(result["log"]["u"])
    assert np.max(np.abs(applied)) <= cfg.force_limit + 1e-12
    assert result["metrics"]["qp_failures"] == 0
