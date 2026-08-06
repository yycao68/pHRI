import numpy as np

from verify_fr3_two_rate_benchmark import (
    Config, authorization_period_sweep, energy_budget_sweep, po_pc_regularization_check,
    run_trial, sensing_realism_sweep, torque_scale, wall_classification_check,
)


def test_joint_torque_scaling_respects_derated_fr3_envelope():
    nominal = np.array([10.0, -8.0, 4.0, 2.0, 1.0, -1.0, 0.0])
    residual = np.array([20.0, -30.0, 5.0, 0.0, 8.0, -8.0, 3.0])
    limits = np.array([21.75, 21.75, 21.75, 21.75, 3.0, 3.0, 3.0])
    alpha, feasible = torque_scale(nominal, residual, limits)
    assert feasible
    assert 0.0 <= alpha <= 1.0
    assert np.all(np.abs(nominal + alpha * residual) <= limits + 1e-12)


def test_two_rate_fr3_preserves_tank_floor_and_torque_envelope():
    cfg = Config(duration=1.8)
    result = run_trial("two_rate", cfg, seed=4)
    assert result["metrics"]["minimum_tank_j"] >= cfg.tank_minimum - 1e-12
    assert result["metrics"]["maximum_torque_ratio"] <= 1.0 + 1e-8
    assert result["metrics"]["nominal_infeasible_samples"] == 0
    assert result["metrics"]["qp_failures"] == 0


def test_manager_guard_breaches_the_tank_floor_that_two_rate_preserves():
    # B3': same MPC proposal and authorization rule as two_rate, but alpha_E
    # is computed once per manager tick and held stale across the fast ticks
    # it spans. This is the intended, regression-guarded behavior -- it
    # isolates that fast re-authorization, not periodic authorization alone,
    # is what keeps the tank floor invariant that two_rate preserves.
    # Full default duration (not the 1.8 s smoke-test length used elsewhere
    # in this file): the breach only appears once the trial reaches the
    # wall-contact segment starting at t=2.25 s, matching the actual
    # 20-seed Table 1 protocol this test guards.
    cfg = Config()
    result = run_trial("manager_guard", cfg, seed=4)
    assert result["metrics"]["tank_violation_j"] > 0.0
    assert result["metrics"]["minimum_tank_j"] < cfg.tank_minimum
    assert result["metrics"]["maximum_torque_ratio"] <= 1.0 + 1e-8
    assert result["metrics"]["nominal_infeasible_samples"] == 0
    assert result["metrics"]["qp_failures"] == 0


def test_hannaford_ryu_po_pc_keeps_observer_nonnegative():
    result = run_trial("tdpc", Config(duration=1.8), seed=4)
    assert result["metrics"]["minimum_po_energy_j"] >= -1e-12
    assert result["metrics"]["maximum_torque_ratio"] <= 1.0 + 1e-8


def test_two_rate_preserves_invariants_under_delay_colored_noise_and_velocity_bias():
    cfg = Config(duration=1.8)
    result = run_trial(
        "two_rate", cfg, seed=4, leakage=0.25,
        wall_stiffness=0.0, wall_damping=0.0, disturbance_scale=0.0, sensor_noise=0.05,
        estimate_delay_ticks=1, noise_ar1=0.9, velocity_bias=np.array([0.005, 0.0, 0.0]),
    )
    assert result["metrics"]["minimum_tank_j"] >= cfg.tank_minimum - 1e-12
    assert result["metrics"]["maximum_torque_ratio"] <= 1.0 + 1e-8
    assert result["metrics"]["nominal_infeasible_samples"] == 0
    assert result["metrics"]["qp_failures"] == 0


def test_wall_reaction_is_pushed_into_with_zero_authorization_engagement():
    # F_e (the wall's own reaction force) is folded into d_hat at full weight
    # (22 with lambda=0), so the MPC's rejection objective structurally
    # includes cancelling legitimate contact resistance. This is the
    # intended, regression-guarded finding: passive impedance never reaches
    # the wall under this protocol, the proposed controller does and pushes
    # into it, and the tank never engages because the failure is a
    # classification error, not an energy or torque violation.
    cfg = Config()
    result = wall_classification_check(cfg, seeds=3)
    assert result["rows"]["impedance"]["contact_fraction_mean"] == 0.0
    assert result["rows"]["two_rate"]["contact_fraction_mean"] > 0.0
    assert result["rows"]["two_rate"]["into_wall_impulse_ns_mean"] > 0.0
    assert result["rows"]["two_rate"]["authorization_active_during_contact_mean"] == 0.0


def test_energy_budget_sweep_trades_tracking_against_contact_aggressiveness():
    # More initial budget above the fixed floor should let the controller
    # intervene less (lower active fraction) and track better (lower RMS),
    # while doing so more aggressively into a misclassified wall contact
    # (higher penetration) -- the intended, regression-guarded tradeoff.
    cfg = Config()
    result = energy_budget_sweep(cfg, seeds=2)
    rows = result["rows"]
    assert [r["budget_above_floor_j"] for r in rows] == sorted(r["budget_above_floor_j"] for r in rows)
    assert rows[0]["residual_rms_mm_mean"] > rows[-1]["residual_rms_mm_mean"]
    assert rows[0]["projection_active_fraction_mean"] > rows[-1]["projection_active_fraction_mean"]
    assert rows[0]["max_penetration_mm_mean"] <= rows[-1]["max_penetration_mm_mean"]
    for row in rows:
        assert row["minimum_tank_j_mean"] >= cfg.tank_minimum - 1e-9


def test_po_pc_regularized_variant_still_trails_two_rate():
    # The velocity dead zone should not fully close the gap to two_rate --
    # this is the intended, regression-guarded finding that the singular
    # gain is real but not the dominant driver of PO/PC's disadvantage.
    result = po_pc_regularization_check(Config(), seeds=2)
    assert result["regularized_vs_two_rate_paired_difference_mm"] > 0.0
    assert (result["summary"]["tdpc_regularized"]["residual_rms_mm_mean"]
            <= result["summary"]["tdpc"]["residual_rms_mm_mean"])


def test_authorization_period_sweep_breaches_at_the_first_staleness_tick():
    # 1 ms (period=1) is two_rate itself and must stay safe; any longer
    # period is intended, regression-guarded to violate immediately, not
    # gracefully, matching the Section 7.1 finding.
    cfg = Config()
    result = authorization_period_sweep(cfg, seeds=2)
    rows = {row["period_ticks"]: row for row in result["rows"]}
    assert rows[1]["tank_violation_j_mean"] == 0.0
    assert rows[2]["seeds_violated"] > 0


def test_sensing_realism_sweep_reports_five_conditions_with_preserved_invariants():
    cfg = Config(duration=1.8)
    result = sensing_realism_sweep(cfg, seeds=2)
    assert result["seeds_per_level"] == 2
    names = [row["condition"] for row in result["rows"]]
    assert names == ["baseline", "delay only", "colored noise only",
                      "velocity bias only", "all combined"]
    for row in result["rows"]:
        assert row["minimum_tank_j_min"] >= cfg.tank_minimum - 1e-12
        assert row["maximum_torque_ratio"] <= 1.0 + 1e-8
        assert row["qp_failures"] == 0

