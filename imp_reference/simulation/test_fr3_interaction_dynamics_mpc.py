import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))
from fr3_mujoco import FR3MuJoCoEnv  # noqa: E402

from fr3_interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference3D,
    FR3MPCConfig,
    FR3RealizationMPC,
    ImpedanceReference3D,
    compute_tau_base,
    fr3_clipped_reference_command,
    make_default_impedance_params,
)


def _env():
    return FR3MuJoCoEnv(timestep=0.001)


def test_generators_have_expected_equilibria():
    impedance = ImpedanceReference3D(mass=2.0, stiffness=200.0, damping=28.0)
    force = np.array([0.0, 0.0, 10.0])
    equilibrium_e = force / impedance.stiffness  # K_d e = f_h at edot=0, a_id=0
    state = np.concatenate([equilibrium_e, np.zeros(3)])
    np.testing.assert_allclose(impedance.acceleration(state, force), 0.0, atol=1e-10)

    admittance = AdmittanceReference3D(time_constant=0.3, force_to_velocity=0.01)
    equilibrium_edot = admittance.force_to_velocity * force  # T a_id + edot = Y f_h at a_id=0
    state = np.concatenate([np.array([0.4, -0.2, 0.1]), equilibrium_edot])
    np.testing.assert_allclose(admittance.acceleration(state, force), 0.0, atol=1e-10)


def test_zero_state_zero_force_returns_zero_command():
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()

    cfg = FR3MPCConfig(horizon=8)
    controller = FR3RealizationMPC(ImpedanceReference3D(), cfg)
    step = controller.control(dyn, state, p_nominal, R_d, np.zeros((cfg.horizon, 3)))
    np.testing.assert_allclose(step.command, 0.0, atol=1e-6)


def test_mpc_respects_first_step_limits():
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()

    cfg = FR3MPCConfig(horizon=10)
    controller = FR3RealizationMPC(AdmittanceReference3D(), cfg)
    forecast = np.tile(np.array([0.0, 0.0, -15.0]), (cfg.horizon, 1))
    step = controller.control(dyn, state, p_nominal, R_d, forecast)

    assert np.max(np.abs(step.command)) <= cfg.force_limit + 1e-5
    assert np.all(np.abs(step.tau) <= cfg.tau_max + 1e-5)


def test_horizon_wide_torque_feasibility_nonbinding():
    """The property this module exists to guarantee: every predicted horizon
    step's torque -- not only the first -- respects the per-joint limits.
    A first-step-only formulation could pass this scenario's i=0 check while
    still planning infeasible later steps; this test inspects the full
    predicted sequence directly.

    With the default tau_max this scenario's peak torque utilization is only
    ~34% (checked directly), so this test alone cannot distinguish a working
    constraint from a silently disabled one -- it would pass identically
    either way. See test_horizon_wide_torque_feasibility_binding below for
    the version that actually exercises the constraint."""
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()

    cfg = FR3MPCConfig(horizon=10, position_limit=0.08, speed_limit=0.4)
    controller = FR3RealizationMPC(ImpedanceReference3D(stiffness=400.0, damping=40.0), cfg)
    forecast = np.tile(np.array([15.0, -10.0, 25.0]), (cfg.horizon, 1))
    step = controller.control(dyn, state, p_nominal, R_d, forecast)

    assert step.horizon_tau.shape == (cfg.horizon, 7)
    per_joint_ok = np.all(np.abs(step.horizon_tau) <= cfg.tau_max[None, :] + 1e-5)
    assert per_joint_ok, (
        "a horizon step's predicted torque exceeds a per-joint limit: "
        f"max abs per joint = {np.max(np.abs(step.horizon_tau), axis=0)}, "
        f"tau_max = {cfg.tau_max}"
    )


def test_horizon_wide_torque_feasibility_binding():
    """Same scenario as the nonbinding test above, but with joint 4's limit
    artificially tightened to 5 Nm -- well below what this scenario would
    otherwise command there (confirmed directly: ~26 Nm at the default
    87 Nm limit). This forces the constraint to actually activate, so the
    test can distinguish a genuinely enforced bound from one that merely
    happens to never be approached. A working constraint should (a) hold
    joint 4 at or under its tightened limit and (b) visibly redistribute
    correction effort to other joints to compensate -- both checked here,
    not just the aggregate no-violation property."""
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()

    default_tau_max = FR3MPCConfig().tau_max.copy()
    tightened_tau_max = default_tau_max.copy()
    tightened_tau_max[3] = 5.0  # well under the ~26 Nm this scenario naturally commands there

    cfg = FR3MPCConfig(horizon=10, position_limit=0.08, speed_limit=0.4, tau_max=tightened_tau_max)
    controller = FR3RealizationMPC(ImpedanceReference3D(stiffness=400.0, damping=40.0), cfg)
    forecast = np.tile(np.array([15.0, -10.0, 25.0]), (cfg.horizon, 1))
    step = controller.control(dyn, state, p_nominal, R_d, forecast)

    # 1e-3, not the tighter 1e-5 used in the nonbinding test above: with the
    # constraint genuinely active, OSQP's own KKT-residual tolerance
    # (cfg.osqp_eps=1e-4) does not tightly bound the constraint violation
    # itself -- confirmed empirically, the observed violation here is
    # ~2e-4, already larger than osqp_eps. 1e-3 Nm is still utterly
    # negligible against a 5 Nm limit; the point of this assertion is "no
    # meaningful violation," not "zero to machine precision."
    per_joint_ok = np.all(np.abs(step.horizon_tau) <= cfg.tau_max[None, :] + 1e-3)
    assert per_joint_ok, "the tightened per-joint limit is violated somewhere in the horizon"

    max_joint4 = np.max(np.abs(step.horizon_tau[:, 3]))
    assert max_joint4 >= 4.9, (
        f"joint 4's torque (max {max_joint4:.3f} Nm) never approaches its tightened "
        "5 Nm limit -- the constraint may not be binding in this scenario, which would "
        "make the nonbinding test's pass uninformative about whether enforcement works"
    )

    # Same scenario, unconstrained on joint 4, for direct comparison.
    baseline_cfg = FR3MPCConfig(horizon=10, position_limit=0.08, speed_limit=0.4)
    baseline_controller = FR3RealizationMPC(ImpedanceReference3D(stiffness=400.0, damping=40.0), baseline_cfg)
    baseline_step = baseline_controller.control(dyn, state, p_nominal, R_d, forecast)
    baseline_joint4 = np.max(np.abs(baseline_step.horizon_tau[:, 3]))
    assert baseline_joint4 > 5.0, (
        "baseline (untightened) joint 4 torque does not exceed the tightened limit -- "
        "the tightened scenario above would not actually be testing constraint activation"
    )


def test_invalid_forecast_shape_is_rejected():
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()

    cfg = FR3MPCConfig(horizon=6)
    controller = FR3RealizationMPC(ImpedanceReference3D(), cfg)
    try:
        controller.control(dyn, state, p_nominal, R_d, np.zeros((cfg.horizon, 2)))
    except ValueError as error:
        assert "force_forecast" in str(error)
    else:
        raise AssertionError("invalid forecast shape was accepted")


def test_inner_loop_recompute_uses_fresh_state():
    """Guards against the exact class of bug this session found and fixed in
    the sibling paper: tau_base must be recomputed from the CURRENT (q, qdot),
    not cached/held across ticks, or the claimed inner-loop rate is a lie."""
    env = _env()
    cfg = FR3MPCConfig()
    imp_params = make_default_impedance_params(cfg)

    dyn1, state1 = env.get_dynamics_and_state()
    R_d = state1.ee_rot.copy()
    tau_base_1, J_v_1, _ = compute_tau_base(dyn1, state1, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg)

    # Move to a visibly different configuration and recompute.
    env.reset(q=env.q + 0.3, dq=np.zeros(7))
    dyn2, state2 = env.get_dynamics_and_state()
    tau_base_2, J_v_2, _ = compute_tau_base(dyn2, state2, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg)

    assert not np.allclose(tau_base_1, tau_base_2), (
        "tau_base did not change between two different configurations -- "
        "looks cached/held rather than recomputed from the current state"
    )
    assert not np.allclose(J_v_1, J_v_2)


def test_reactive_baseline_matches_generator_instant_law():
    """The reactive comparator should realize the generator's *instantaneous*
    desired acceleration exactly when unconstrained (small force, well inside
    every box), unlike the predictive controller which may trade accuracy for
    horizon-wide feasibility even when nothing is currently violated."""
    env = _env()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    cfg = FR3MPCConfig()
    generator = ImpedanceReference3D()
    force = np.array([0.0, 0.0, 1.0])  # small: nowhere near any bound

    command = fr3_clipped_reference_command(
        generator, dyn, state, p_nominal, force, cfg, previous_command=np.zeros(3)
    )

    J_v = dyn.J[:3, :]
    M_inv = np.linalg.inv(dyn.M)
    Lam_inv = J_v @ M_inv @ J_v.T + cfg.lambda_reg * np.eye(3)
    a_realized = Lam_inv @ (command + force)
    x = np.concatenate([state.ee_pos - p_nominal, state.ee_vel[:3]])
    a_id = generator.acceleration(x, force)
    np.testing.assert_allclose(a_realized, a_id, atol=1e-6)
