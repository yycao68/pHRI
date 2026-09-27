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


# --- the Section 7.7 fix: authority over the nominal --------------------


def test_nominal_authorization_keeps_the_anchor_legal(plant):
    """With the fix on, an anchor that would overrun is scaled until it fits.
    Without it, alpha_nom stays 1 and the overrun stands."""
    _, _, J_v, Lam_inv, tau_base = plant
    cap = pc.torque_envelope("derated_joint4")
    # A displacement large enough that K0 e alone breaks the envelope.
    e = np.array([0.0, 0.0, 0.30])
    v = np.zeros(3)

    off = pir_servo_step(PIRConfig(k0=K0, d0=D0, nominal_authorization=False),
                         tau_base, J_v, Lam_inv, e, v, np.zeros(3),
                         tank=0.08, h=1e-3, cap=cap)
    assert not off.anchor_feasible, "test needs an infeasible anchor to be meaningful"
    assert off.alpha_nom == 1.0
    assert off.tau_ratio > 1.0

    on = pir_servo_step(PIRConfig(k0=K0, d0=D0, nominal_authorization=True),
                        tau_base, J_v, Lam_inv, e, v, np.zeros(3),
                        tank=0.08, h=1e-3, cap=cap)
    assert on.alpha_nom < 1.0
    assert on.tau_ratio <= 1.0 + 1e-9, "the fix must make the applied torque legal"


def test_adopted_default_is_inert_on_the_published_closed_loop():
    """Decision 0 must not move any number Section 7 reports: at nominal load
    the anchor fits, so alpha_nom stays 1 and the controller is unchanged."""
    s = run_variant("pir", K0, D0, "derated_joint4", scenario="push")["summary"]
    assert s["alpha_nom_min"] == 1.0
    assert s["diag1_anchor_ratio_closed_loop"] == pytest.approx(0.9762, abs=5e-4)
    assert s["diag4_budget_ratio_closed_loop"] == pytest.approx(0.3474, abs=5e-4)


def test_nominal_authorization_is_inert_when_the_anchor_fits(plant):
    """It must not perturb the certified operating point in normal operation."""
    _, _, J_v, Lam_inv, tau_base = plant
    cap = pc.torque_envelope("derated_joint4")
    e, v = np.array([0.0, 0.0, 0.05]), np.array([0.0, 0.0, 0.1])
    a = pir_servo_step(PIRConfig(k0=K0, d0=D0, nominal_authorization=False),
                       tau_base, J_v, Lam_inv, e, v, np.array([0.0, 0.0, 5.0]),
                       tank=0.08, h=1e-3, cap=cap)
    b = pir_servo_step(PIRConfig(k0=K0, d0=D0, nominal_authorization=True),
                       tau_base, J_v, Lam_inv, e, v, np.array([0.0, 0.0, 5.0]),
                       tank=0.08, h=1e-3, cap=cap)
    assert a.anchor_feasible and b.alpha_nom == 1.0
    np.testing.assert_allclose(a.tau, b.tau, atol=1e-12)


def test_restiffening_charges_the_tank_and_softening_does_not(plant):
    """Only one direction of a time-varying spring gain is an injection."""
    _, _, J_v, Lam_inv, tau_base = plant
    # The mode is pinned to "free": the adopted rule meters the rise against
    # the tank, which is precisely what bounds the charge this test is about.
    # Testing that the charge exists means testing the unrestricted rule.
    cfg = PIRConfig(k0=K0, d0=D0, nominal_authorization=True,
                    nominal_reauth="free")
    cap = pc.torque_envelope("derated_joint4")
    e, v = np.array([0.0, 0.0, 0.30]), np.zeros(3)
    common = dict(tau_base=tau_base, J_v=J_v, Lam_inv=Lam_inv, e=e, v=v,
                  f_r_held=np.zeros(3), tank=0.08, h=1e-3, cap=cap)
    # alpha_nom lands below 1 here, so coming from 1.0 is a softening step and
    # coming from below it is a re-stiffening step.
    softening = pir_servo_step(cfg, previous_alpha_nom=1.0, **common)
    restiffening = pir_servo_step(cfg, previous_alpha_nom=0.0, **common)
    assert restiffening.tank < softening.tank, "re-stiffening must cost more"


def test_monotone_alpha_nom_never_rises():
    """The monotone ablation must still do what it says: never rise, and hold
    both guarantees.  It is no longer the default (see the test below), but
    Section 8.5's (P3) argument is stated through it."""
    out = run_variant("pir", K0, D0, "derated_joint4",
                      scenario="merged", disturbance_scale=12.0,
                      overrides={"nominal_reauth": "monotone"})
    log, s = out["log"], out["summary"]
    assert np.all(np.diff(log["alpha_nom"]) <= 1e-12), "alpha_nom rose"
    assert s["lemma1_conclusion_holds"], "torque envelope lost"
    assert s["tank_floor_holds"], "tank floor lost"
    assert s["alpha_nom_min"] < 1.0, "test scenario never exercised the fix"


def test_energy_authorized_reauth_recovers_without_losing_either_guarantee():
    """The adopted rule (decision 0, as repaired by Section 8.5's (C5)):
    alpha_nom recovers after a transient, and both guarantees survive it.

    If this ever fails while the monotone test above passes, the metering is
    buying recovery with the tank floor and the trade has gone the wrong way.
    """
    assert PIRConfig().nominal_authorization is True
    assert PIRConfig().nominal_reauth == "energy_authorized"
    out = run_variant("pir", K0, D0, "derated_joint4",
                      scenario="merged", disturbance_scale=12.0)
    log, s = out["log"], out["summary"]
    assert s["lemma1_conclusion_holds"], "torque envelope lost"
    assert s["tank_floor_holds"], "tank floor lost"
    assert s["alpha_nom_min"] < 1.0, "test scenario never exercised the fix"
    # The latch is gone: alpha_nom comes back up, and ends above its minimum.
    assert np.max(np.diff(log["alpha_nom"])) > 1e-9, "alpha_nom never recovered"
    assert log["alpha_nom"][-1] > s["alpha_nom_min"] + 1e-9


def test_unrestricted_reauthorization_loses_the_tank_floor():
    """The contrast the monotone rule is there to fix. If this stops failing,
    the comparison in pir_fixes.py is vacuous.

    The mode is pinned to "free" explicitly rather than relying on the
    default, so this keeps testing the pre-decision-0 rule after the default
    changed.
    """
    s = run_variant("pir", K0, D0, "derated_joint4", scenario="merged",
                    disturbance_scale=12.0,
                    overrides={"nominal_reauth": "free"})["summary"]
    assert s["lemma1_conclusion_holds"]
    assert not s["tank_floor_holds"]


def test_anisotropic_gains_are_wired_through(plant):
    """A per-axis K0 must actually reach the nominal force, not be broadcast
    from the first element."""
    _, _, J_v, Lam_inv, tau_base = plant
    cap = pc.torque_envelope("derated_joint4")
    cfg = PIRConfig(k0=(60.0, 60.0, 380.0), d0=(8.0, 8.0, 29.07))
    np.testing.assert_allclose(cfg.k0_vector(), [60.0, 60.0, 380.0])
    step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, np.full(3, 0.01),
                          np.zeros(3), np.zeros(3), tank=0.08, h=1e-3, cap=cap)
    np.testing.assert_allclose(step.f_nom, [-0.6, -0.6, -3.8], rtol=1e-9)


def test_qp_and_servo_agree_on_the_interaction_pose():
    """The QP and the 1 kHz servo each build tau_base, and both must centre the
    null-space spring on the SAME pose.  Centring the QP on Q_NEUTRAL while the
    servo sits elsewhere makes the QP plan against a tau_base the servo never
    applies -- invisible at Q_NEUTRAL, and caught away from it only by the
    four-term closure.  This pins the closure at both poses.
    """
    q = pc.Q_NEUTRAL.copy()
    q[1], q[3], q[5] = -1.30, -1.30, 1.571
    for pose, k0, d0 in ((None, K0, D0), (q, 520.0, 56.13)):
        s = run_variant("pir", k0, d0, "derated_joint4", scenario="push",
                        duration=1.0, pose=pose)["summary"]
        assert s["decomposition_closure_max_on_qp_ticks"] < 1e-10, (
            f"four-term residual does not close at pose {s['pose_tag']}")


def test_moving_the_pose_moves_the_qp_s_nominal_params():
    """The guard behind the test above: PIRConfig.pose must actually reach the
    realization QP's impedance params, not just the servo's."""
    q = pc.Q_NEUTRAL.copy()
    q[1], q[3], q[5] = -1.30, -1.30, 1.571
    at_neutral = PIRRealizationMPC(pc.ImpedanceReference3D(), PIRConfig(k0=K0, d0=D0))
    at_pose = PIRRealizationMPC(pc.ImpedanceReference3D(),
                                PIRConfig(k0=K0, d0=D0, pose=tuple(q)))
    np.testing.assert_allclose(at_neutral.imp_params.q_null, pc.Q_NEUTRAL)
    np.testing.assert_allclose(at_pose.imp_params.q_null, q)


# --- Merged Lemma 1' (Section 8.5) --------------------------------------


def test_lemma1_prime_C1_C2_hold_on_adversarial_ticks(plant):
    """(C1) and (C2) are unconditional given (P1)-(P3): sample far outside
    anything the QP emits and neither the envelope nor the floor moves."""
    _, _, J_v, Lam_inv, tau_base = plant
    cfg = PIRConfig(k0=K0, d0=D0)  # adopted default
    cap = pc.torque_envelope(cfg.envelope)
    assert np.all(np.abs(tau_base) <= cap), "(P1) must hold for the lemma to apply"
    rng = np.random.default_rng(7)
    tank, prev = cfg.tank_initial, 1.0
    for _ in range(400):
        e = rng.uniform(-0.35, 0.35, 3)
        step = pir_servo_step(cfg, tau_base, J_v, Lam_inv,
                              e, rng.uniform(-1.2, 1.2, 3),
                              rng.uniform(-600.0, 600.0, 3),
                              tank=tank, h=1e-3, cap=cap, previous_alpha_nom=prev)
        assert step.tau_ratio <= 1.0 + 1e-9, "(C1) torque envelope"
        assert step.tank >= cfg.tank_minimum - 1e-12, "(C2) tank floor"
        # (P3) bounds the RISE by what the tank can buy; it does not forbid it.
        spring = 0.5 * float(e @ (np.full(3, K0) * e))
        assert (step.alpha_nom - prev) * spring <= (tank - cfg.tank_minimum) + 1e-9, \
            "(P3) alpha_nom rose beyond the tank's budget"
        tank, prev = step.tank, step.alpha_nom


def test_lemma1_prime_P3_is_what_makes_C2_provable(plant):
    """Drop (P3) and (C2) fails. If this stops failing, the monotone rule has
    become cosmetic and Section 8.5's argument needs revisiting."""
    _, _, J_v, Lam_inv, tau_base = plant
    cap = pc.torque_envelope("derated_joint4")
    cfg = PIRConfig(k0=K0, d0=D0, nominal_reauth="free")
    rng = np.random.default_rng(7)
    tank, prev = cfg.tank_initial, 1.0
    breached = False
    for _ in range(400):
        step = pir_servo_step(cfg, tau_base, J_v, Lam_inv,
                              rng.uniform(-0.35, 0.35, 3), rng.uniform(-1.2, 1.2, 3),
                              rng.uniform(-600.0, 600.0, 3),
                              tank=tank, h=1e-3, cap=cap, previous_alpha_nom=prev)
        breached = breached or step.tank < cfg.tank_minimum - 1e-12
        tank, prev = step.tank, step.alpha_nom
    assert breached, "unrestricted alpha_nom must be able to overdraw the tank"


def test_lemma1_prime_C4_cost_is_the_latch_not_the_authorization(plant):
    """(C4): alpha_nom buys the torque envelope by spending workspace -- but
    almost all of what that looked like it cost was the LATCH, not the
    authorization.

    At 8x the adopted rule holds the envelope at essentially the ablation's own
    excursion, while the monotone rule it replaced spends ~70 points of box for
    the same envelope. Pin both halves: if the first ever regresses, decision 0
    is back to being a trade rather than a win; if the second stops holding,
    Section 8.5's account of where the cost came from is wrong.
    """
    common = dict(scenario="merged", disturbance_scale=8.0)
    on = run_variant("pir", K0, D0, "derated_joint4", **common)["summary"]
    mono = run_variant("pir", K0, D0, "derated_joint4",
                       overrides={"nominal_reauth": "monotone"},
                       **common)["summary"]
    off = run_variant("pir_no_nominal_auth", K0, D0, "derated_joint4",
                      **common)["summary"]
    # Both rules hold the envelope; the ablation does not, but only barely.
    assert on["lemma1_conclusion_max_tau_ratio"] <= 1.0 + 1e-9
    assert mono["lemma1_conclusion_max_tau_ratio"] <= 1.0 + 1e-9
    assert 1.0 < off["lemma1_conclusion_max_tau_ratio"] < 1.02
    # The monotone rule pays for it in workspace; the adopted rule does not.
    assert mono["max_abs_e_axis_m"] > off["max_abs_e_axis_m"] * 1.4
    assert on["max_abs_e_axis_m"] < off["max_abs_e_axis_m"] * 1.1


# --- Propositions 2 and 3 (Section 8.6) ---------------------------------


def test_feasibility_certificate_is_sound_on_the_assembled_qp(plant):
    """Proposition 2 is a SUFFICIENT condition, so the one thing it may never
    do is call a QP feasible that the solver rejects. Check the certificate
    against the rows the controller actually assembles, not against a
    re-derivation of them."""
    import pir_recursive_feasibility as rf

    dyn, state, _, _, _ = plant
    cfg = PIRConfig(k0=K0, d0=D0, envelope="derated_joint4")
    mpc = PIRRealizationMPC(pc.ImpedanceReference3D(), cfg)
    force = np.array([0.0, 0.0, -pc.PUSH_MAGNITUDE_N])
    forecast = np.tile(force, (mpc.cfg.horizon, 1))
    rec = rf.check_certificate(
        mpc, dyn, state, state.ee_pos.copy(), state.ee_rot.copy(),
        forecast, forecast, cfg, np.zeros(3))
    assert rec["F1_rate_admits_zero"], "zero must be reachable from zero"
    assert rec["certificate_sound"], "certificate claimed a feasible QP OSQP rejects"
    if rec["F2_holds"]:
        assert rec["candidate_feasible"], "(F1)+(F2) must imply the candidate"
        assert rec["solver_feasible"]


def test_terminal_set_is_nonempty_exactly_when_P1_is_strict():
    """Proposition 3's c* is positive iff the anchor has headroom at the
    fallback equilibrium. Pin that equivalence on a synthetic plant, so it is
    checked as algebra rather than inherited from one FR3 pose."""
    import pir_recursive_feasibility as rf

    A_cl = np.diag([0.9, 0.9, 0.9, 0.8, 0.8, 0.8])
    B = np.vstack([0.01 * np.eye(3), 0.1 * np.eye(3)])
    B_accel = np.zeros((6, 3))
    jt_g0 = np.zeros((2, 6))
    jt_g0[0, 0] = 100.0
    jt_g0[1, 3] = 50.0
    d, f = np.zeros(3), np.array([1.0, 0.0, 0.0])
    tau_base = np.array([1.0, 1.0])

    generous = rf.terminal_set(A_cl, B, B_accel, d, f, tau_base, jt_g0,
                               np.array([50.0, 50.0]))
    assert generous["schur"] and generous["nonempty"]
    assert generous["P1_at_equilibrium_worst_ratio"] < 1.0

    # Squeeze the cap onto the equilibrium anchor: headroom goes, c* goes.
    tight_cap = np.array([abs(generous["_x_eq"][0] * 100.0 - 1.0) * 0.5, 50.0])
    tight = rf.terminal_set(A_cl, B, B_accel, d, f, tau_base, jt_g0, tight_cap)
    assert tight["P1_at_equilibrium_worst_ratio"] > 1.0
    assert tight["c_star"] == 0.0 and not tight["nonempty"]
