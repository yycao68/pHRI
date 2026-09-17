"""Tests for the merged PIR controller.

These check the structural properties Merged Lemma 1 and the four-term
residual are stated about, on short runs rather than full sweeps.
"""

from __future__ import annotations

import numpy as np
import pytest

import pir_common as pc
import pir_controller as pk
from pir_controller import PIRConfig, PIRRealizationMPC, pir_servo_step
from run_pir_closed_loop import run_variant

K0, D0 = 380.0, 29.07


@pytest.fixture(scope="module")
def plant():
    env = pc.FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    dyn, state = env.get_dynamics_and_state()
    J_v = dyn.J[:3, :]
    Lam_inv = J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3)
    cfg = pc.FR3MPCConfig()
    tau_base, _, _ = pc.compute_tau_base(
        dyn, state, state.ee_rot.copy(), pc.make_default_impedance_params(cfg),
        cfg.K_rot, cfg.D_rot, cfg.lambda_reg)
    return dyn, state, J_v, Lam_inv, tau_base


# --- the servo rule ----------------------------------------------------


def test_servo_holds_the_torque_envelope_for_any_residual(plant):
    """Lemma 1's torque half: a feasible anchor plus the scaled residual stays
    in the box, however large the residual the QP proposes."""
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    cap = pc.torque_envelope(cfg.envelope)
    rng = np.random.default_rng(0)
    for _ in range(50):
        e = rng.uniform(-0.05, 0.05, 3)
        v = rng.uniform(-0.3, 0.3, 3)
        f_r = rng.uniform(-400.0, 400.0, 3)  # far beyond anything the QP emits
        step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v, f_r,
                              tank=0.08, h=1e-3, cap=cap)
        if step.anchor_feasible:
            assert step.tau_ratio <= 1.0 + 1e-9, "torque envelope violated"
        assert 0.0 <= step.alpha_tau <= 1.0
        assert 0.0 <= step.alpha_E <= 1.0


def test_servo_holds_the_tank_floor_when_reauthorized_every_tick(plant):
    """Lemma 1's energy half: recomputing alpha_E each tick cannot take the
    ledger below E_min, whatever the proposed residual."""
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    cap = pc.torque_envelope(cfg.envelope)
    rng = np.random.default_rng(1)
    tank = cfg.tank_initial
    for _ in range(500):
        e = rng.uniform(-0.05, 0.05, 3)
        v = rng.uniform(-0.3, 0.3, 3)
        f_r = rng.uniform(-200.0, 200.0, 3)
        step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v, f_r,
                              tank=tank, h=1e-3, cap=cap)
        tank = step.tank
        assert tank >= cfg.tank_minimum - 1e-12
        assert tank <= cfg.tank_maximum + 1e-12


def test_a_stale_energy_scale_can_breach_the_floor(plant):
    """The B4 contrast: holding alpha_E across ticks removes the guarantee.
    If this ever stops failing, the manager-guard baseline has become the
    fast rule and the comparison is vacuous."""
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    cap = pc.torque_envelope(cfg.envelope)
    tank = cfg.tank_minimum + 1e-4
    v = np.array([0.0, 0.0, 0.2])
    f_r = np.array([0.0, 0.0, 60.0])  # injecting: F_r . v > 0
    step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, np.zeros(3), v, f_r,
                          tank=tank, h=1e-3, cap=cap, held_alpha_E=1.0)
    assert step.tank < cfg.tank_minimum, "a stale alpha_E of 1 should overdraw"


def test_no_energy_debit_when_the_residual_is_dissipative(plant):
    """alpha_E = 1 whenever the residual removes energy from the port."""
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    cap = pc.torque_envelope(cfg.envelope)
    v = np.array([0.0, 0.0, 0.2])
    step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, np.zeros(3), v,
                          np.array([0.0, 0.0, -10.0]), tank=cfg.tank_minimum,
                          h=1e-3, cap=cap)
    assert step.alpha_E == 1.0
    assert step.tank >= cfg.tank_minimum


def test_zero_gains_reduce_the_anchor_to_tau_base(plant):
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=0.0, d0=0.0)
    cap = pc.torque_envelope(cfg.envelope)
    step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, np.full(3, 0.03),
                          np.full(3, 0.2), np.zeros(3), tank=0.08, h=1e-3, cap=cap)
    np.testing.assert_allclose(step.anchor, tau_base)
    np.testing.assert_allclose(step.f_nom, np.zeros(3))


# --- the QP ------------------------------------------------------------


def test_qp_respects_the_horizon_wide_torque_envelope(plant):
    """The property phri2 exists to guarantee, preserved under the split:
    every predicted step, not just the first."""
    dyn, state, _, _, _ = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    mpc = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg)
    cap = pc.torque_envelope(cfg.envelope)
    out = mpc.control(dyn, state, state.ee_pos.copy(), state.ee_rot.copy(),
                      np.tile(np.array([0.0, 0.0, -20.0]), (cfg.mpc.horizon, 1)))
    assert np.all(np.abs(out["horizon_tau"]) <= cap[None, :] + 1e-3)
    assert out["residual_command"].shape == (3,)


def test_behaviour_channel_defaults_to_the_force_channel(plant):
    """Omitting behaviour_forecast must recover phri2's single-signal
    assumption exactly, not merely approximately."""
    dyn, state, _, _, _ = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    forecast = np.tile(np.array([0.0, 0.0, -20.0]), (cfg.mpc.horizon, 1))
    a = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg).control(
        dyn, state, state.ee_pos.copy(), state.ee_rot.copy(), forecast)
    b = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg).control(
        dyn, state, state.ee_pos.copy(), state.ee_rot.copy(), forecast,
        behaviour_forecast=forecast)
    np.testing.assert_allclose(a["residual_command"], b["residual_command"], atol=1e-12)


def test_separate_behaviour_channel_changes_the_residual(plant):
    """...and supplying a different behaviour channel must actually matter,
    or the split is cosmetic."""
    dyn, state, _, _, _ = plant
    cfg = PIRConfig(k0=K0, d0=D0)
    total = np.tile(np.array([0.0, 4.0, -20.0]), (cfg.mpc.horizon, 1))
    push = np.tile(np.array([0.0, 0.0, -20.0]), (cfg.mpc.horizon, 1))
    a = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg).control(
        dyn, state, state.ee_pos.copy(), state.ee_rot.copy(), total)
    b = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg).control(
        dyn, state, state.ee_pos.copy(), state.ee_rot.copy(), total,
        behaviour_forecast=push)
    assert np.linalg.norm(a["residual_command"] - b["residual_command"]) > 1e-6


# --- the four-term residual --------------------------------------------


def test_four_term_residual_closes_exactly():
    """r_reg + r_con_qp + r_con_fast + r_auth must reproduce the modelled
    deviation from a_id on every QP tick, to machine precision."""
    out = run_variant("pir", K0, D0, "derated_joint4", scenario="merged",
                      duration=1.0)
    assert out["summary"]["decomposition_closure_max_on_qp_ticks"] < 1e-10


def test_r_auth_is_zero_exactly_when_authorization_never_fires():
    """The passivity axis must be silent when it is not needed -- r_auth is a
    measurement of intervention, not a always-on correction."""
    out = run_variant("pir", K0, D0, "derated_joint4", scenario="push", duration=1.0)
    s = out["summary"]
    if s["alpha_E_min"] == 1.0:
        assert s["r_auth_rms"] == 0.0


# --- closed loop --------------------------------------------------------


def test_closed_loop_satisfies_merged_lemma_1():
    out = run_variant("pir", K0, D0, "derated_joint4", scenario="merged", duration=2.0)
    s = out["summary"]
    assert s["lemma1_precondition_holds"]
    assert s["lemma1_conclusion_holds"]
    assert s["tank_floor_holds"]


def test_closed_loop_row1_matches_the_replay_estimate():
    """The Task 1 gate measured row (1) by replay.  If the merged closed loop
    disagrees materially, the gate's 2% margin was measured on the wrong
    trajectory and the GO verdict needs revisiting."""
    out = run_variant("pir", K0, D0, "derated_joint4", scenario="push")
    closed_loop = out["summary"]["diag1_anchor_ratio_closed_loop"]
    traj = pc.load_or_generate_trajectory()
    replay = pc.replay_diagnostics(traj, K0, D0, pc.torque_envelope("derated_joint4"))
    assert abs(closed_loop - replay["anchor_ratio"]) < 0.01
