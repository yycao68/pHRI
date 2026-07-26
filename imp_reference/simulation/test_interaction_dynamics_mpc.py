import numpy as np

from interaction_dynamics_mpc import (
    AdmittanceReference,
    ImpedanceReference,
    InteractionDynamicsMPC,
    MPCConfig,
    integrate_point_mass,
)


def test_generators_have_expected_equilibria():
    impedance = ImpedanceReference(mass=2.0, stiffness=40.0, damping=10.0)
    state = np.array([0.0, 0.2, 0.0, 0.0])
    force = np.array([0.0, 8.0])
    np.testing.assert_allclose(impedance.acceleration(state, force), 0.0)

    admittance = AdmittanceReference(time_constant=0.25, force_to_velocity=0.02)
    state = np.array([0.4, -0.3, 0.0, 0.16])
    np.testing.assert_allclose(admittance.acceleration(state, force), 0.0)


def test_zero_state_zero_force_returns_zero_command():
    cfg = MPCConfig(horizon=8)
    controller = InteractionDynamicsMPC(ImpedanceReference(), cfg)
    decision = controller.control(np.zeros(4), np.zeros((cfg.horizon, 2)))
    np.testing.assert_allclose(decision.command, 0.0, atol=1.0e-8)


def test_mpc_respects_first_step_limits():
    cfg = MPCConfig(horizon=10)
    controller = InteractionDynamicsMPC(AdmittanceReference(), cfg)
    # Feasible state with enough stopping distance under the slew limit.
    state = np.array([0.0, 0.04, 0.0, 0.05])
    forecast = np.repeat(np.array([[0.0, 12.0]]), cfg.horizon, axis=0)
    decision = controller.control(state, forecast)
    next_state = integrate_point_mass(
        state, decision.command, forecast[0], cfg.robot_mass, cfg.dt
    )
    assert np.max(np.abs(decision.command)) <= cfg.force_limit + 1.0e-5
    assert np.max(np.abs(next_state[:2])) <= cfg.position_limit + 1.0e-5
    assert np.max(np.abs(next_state[2:])) <= cfg.speed_limit + 1.0e-5


def test_invalid_forecast_shape_is_rejected():
    cfg = MPCConfig(horizon=6)
    controller = InteractionDynamicsMPC(ImpedanceReference(), cfg)
    try:
        controller.control(np.zeros(4), np.zeros((cfg.horizon, 3)))
    except ValueError as error:
        assert "force_forecast" in str(error)
    else:
        raise AssertionError("invalid forecast shape was accepted")
