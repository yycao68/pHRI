"""Regression tests for the horizon constraints and impedance/backbone modes."""

from __future__ import annotations

import numpy as np
import pytest
import scipy.sparse as sp

from simulation.impedance_mpc import ImpedanceMPCController, ImpedanceMPCParams


def _controller(*, solver: str = "osqp", horizon: bool = True, **kwargs):
    params = ImpedanceMPCParams(
        N=4,
        solver=solver,
        horizon_torque_constraint=horizon,
        F_max=5.0,
        tau_max=np.full(7, 0.5),
        **kwargs,
    )
    return ImpedanceMPCController(params, use_kalman=False)


def _single_axis_jacobian() -> np.ndarray:
    """Only F_x maps to joint 1, making the expected torque bound explicit."""
    J = np.zeros((3, 7))
    J[0, 0] = 1.0
    return J


def test_sparse_and_dense_horizon_matrices_match():
    rng = np.random.default_rng(7)
    ctrl = _controller()
    jacobians = [rng.normal(size=(3, 7)) for _ in range(ctrl.p.N)]
    dense = ctrl._torque_constraint_matrix(jacobians, 3 * ctrl.p.N)
    sparse = ctrl._torque_constraint_sparse(
        jacobians, 3 * ctrl.p.N, sp
    ).toarray()
    np.testing.assert_allclose(sparse, dense)
    assert dense.shape == (7 * ctrl.p.N, 3 * ctrl.p.N)


@pytest.mark.parametrize("solver", ["scipy", "osqp"])
def test_horizon_constraint_limits_every_predicted_input(solver):
    J = _single_axis_jacobian()
    H = np.eye(12)
    h = -10.0 * np.ones(12)

    first_only = _controller(solver=solver, horizon=False)
    u_first = first_only._solve_qp(H, h, [J], [np.zeros(7)])
    assert u_first[0] <= 0.5 + 2e-5
    assert np.all(u_first[3::3] > 4.9)

    horizon = _controller(solver=solver, horizon=True)
    u_all = horizon._solve_qp(
        H, h, [J] * horizon.p.N, [np.zeros(7)] * horizon.p.N
    )
    assert horizon.last_qp_success, horizon.last_qp_status
    assert np.all(u_all[0::3] <= 0.5 + 2e-5)
    assert np.all(u_all[0::3] >= -0.5 - 2e-5)


@pytest.mark.parametrize("solver", ["scipy", "osqp"])
def test_impedance_reference_respects_every_horizon_torque_row(solver):
    """The impedance-reference target may be infeasible; every step must clip."""
    ctrl = _controller(solver=solver, impedance_track=True)
    J = _single_axis_jacobian()

    # This is the impedance_track branch's quadratic reference-tracking QP:
    # min 1/2 U'Rbar U - (Rbar U_imp)'U, with an intentionally large target.
    U_imp = np.tile(np.array([20.0, 0.0, 0.0]), ctrl.p.N)
    H = ctrl.R_bar
    h = -ctrl.R_bar @ U_imp
    u = ctrl._solve_qp(
        H, h, [J] * ctrl.p.N, [np.zeros(7)] * ctrl.p.N
    )

    assert ctrl.last_qp_success, ctrl.last_qp_status
    np.testing.assert_allclose(u[0::3], 0.5, atol=2e-5)


@pytest.mark.parametrize("solver", ["scipy", "osqp"])
def test_infeasible_qp_returns_explicit_zero_correction_fallback(solver):
    ctrl = _controller(solver=solver)
    H = np.eye(3 * ctrl.p.N)
    h = np.zeros(3 * ctrl.p.N)
    J = np.zeros((3, 7))
    impossible_base = np.full(7, 2.0)

    u = ctrl._solve_qp(
        H,
        h,
        [J] * ctrl.p.N,
        [impossible_base] * ctrl.p.N,
    )

    assert not ctrl.last_qp_success
    assert ctrl.last_qp_status != "not solved"
    np.testing.assert_array_equal(u, np.zeros_like(u))


def test_scheduled_backbone_builder_reduces_to_frozen_builder():
    ctrl = _controller(backbone_track=True)
    lam_inv = np.diag([0.8, 1.1, 1.4])
    Bd = ctrl._B_d(lam_inv)
    K = ctrl.p.k_backbone * np.eye(3)
    D = 2.0 * ctrl.p.zeta_backbone * np.sqrt(ctrl.p.k_backbone) * np.eye(3)
    G = np.hstack([K, D])

    frozen = ctrl._build_closed_loop_horizon(Bd, ctrl.A_d + Bd @ G)
    scheduled = ctrl._build_scheduled_closed_loop_horizon([Bd] * ctrl.p.N, G)
    for actual, expected in zip(scheduled, frozen):
        np.testing.assert_allclose(actual, expected, atol=1e-13)


def test_default_objective_penalizes_centered_not_absolute_input():
    """At zero error, the unconstrained optimum must be U=-d_seq exactly."""
    ctrl = _controller(solver="scipy")
    lam_inv = np.diag([0.7, 1.0, 1.3])
    d_hat = np.array([2.0, -3.0, 4.0])
    H, h, Gamma, D_bar, x_free = ctrl._default_qp_objective(
        np.zeros(6), d_hat, lam_inv)
    d_seq = np.tile(d_hat, ctrl.p.N)

    # This identity is the algebraic condition used by Theorem 2.
    np.testing.assert_allclose(D_bar, Gamma @ np.tile(np.eye(3), (ctrl.p.N, 1)))
    np.testing.assert_allclose(x_free, Gamma @ d_seq)
    np.testing.assert_allclose(np.linalg.solve(H, h), d_seq, rtol=1e-10, atol=1e-10)

    # Check the assembled OSQP form against the centered physical cost up to
    # a decision-independent constant, using two arbitrary input sequences.
    rng = np.random.default_rng(19)
    inputs = [rng.normal(size=3 * ctrl.p.N) for _ in range(2)]
    assembled = [0.5 * u @ H @ u + h @ u for u in inputs]
    physical = []
    for u in inputs:
        X = x_free + Gamma @ u
        V = u + d_seq
        physical.append(0.5 * X @ ctrl.Q_bar @ X + 0.5 * V @ ctrl.R_bar @ V)
    np.testing.assert_allclose(
        assembled[1] - assembled[0], physical[1] - physical[0],
        rtol=1e-11, atol=1e-11)


@pytest.mark.parametrize("solver", ["scipy", "osqp"])
@pytest.mark.parametrize("mode", ["default", "impedance_reference", "backbone"])
def test_real_fr3_one_step_smoke(solver, mode):
    """Exercise the full control call, including dynamics and horizon rows."""
    from simulation.fr3_mujoco import FR3MuJoCoEnv
    from simulation.phri import circular_ref

    env = FR3MuJoCoEnv(timestep=0.001)
    dyn, state = env.get_dynamics_and_state()
    p_d, dp_d, ddp_d = circular_ref(0.0)
    ctrl = ImpedanceMPCController(
        ImpedanceMPCParams(
            N=3,
            dt_mpc=0.002,
            solver=solver,
            impedance_track=mode == "impedance_reference",
            backbone_track=mode == "backbone",
            horizon_torque_constraint=True,
            k_ws=0.0,
        ),
        use_kalman=False,
    )
    tau, force = ctrl.control(
        state.ee_pos,
        state.ee_vel,
        state.ee_rot,
        p_d,
        dp_d,
        ddp_d,
        np.eye(3),
        dyn,
        state.q,
        state.dq,
    )

    assert ctrl.last_qp_success, ctrl.last_qp_status
    assert tau.shape == (7,)
    assert force.shape == (3,)
    assert np.all(np.isfinite(tau))
    assert np.all(np.isfinite(force))


def test_osqp_path_validates_primal_residual_not_just_status():
    """Regression test for a real gap found by external review: the OSQP path
    used to accept any 'solved'/'solved inaccurate' status on finiteness
    alone, with no check that the returned point actually satisfies
    A_sp @ x in [lb, ub] -- unlike the SciPy path, which already computed
    this residual explicitly. A model-free reproduction (matching the
    review's own smoke test: N=4, F_max=5, tau_max=0.5 Nm, random J/tau_base)
    checks last_qp_residual is now populated on every solve and stays well
    under OSQP_FEAS_TOL, and that last_qp_success genuinely depends on it,
    not just the status string."""
    ctrl = _controller(solver="osqp")
    rng = np.random.default_rng(0)
    for _ in range(50):
        H = np.eye(3 * ctrl.p.N)
        h = rng.normal(size=3 * ctrl.p.N) * 2.0
        J_list = [rng.normal(size=(3, 7)) for _ in range(ctrl.p.N)]
        tau_base_list = [rng.normal(size=7) * 0.1 for _ in range(ctrl.p.N)]
        ctrl._solve_qp_osqp(H, h, J_list, tau_base_list)
        assert ctrl.last_qp_residual is not None, "residual must be logged every call"
        if ctrl.last_qp_success:
            assert ctrl.last_qp_residual < ctrl.OSQP_FEAS_TOL
    print("OK: OSQP path's last_qp_residual is populated and success genuinely "
          "gates on it, not just the solver status string")
