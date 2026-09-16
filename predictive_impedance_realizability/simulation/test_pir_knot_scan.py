"""Regression tests for the PIR (K0, D0) go/no-go scan.

These are cheap structural tests -- they do not re-run the 800-cell scan.
What they pin down is the machinery the scan's conclusion rests on: that the
anchor arithmetic matches what the plant is actually commanded, that the
fallback law really is the alpha -> 0 limit of the merged controller, and
that the two torque envelopes are the ones the source papers use.
"""

from __future__ import annotations

import numpy as np
import pytest

import pir_common as pc
from pir_knot_scan import K_GRID, D_GRID, run_scan


@pytest.fixture(scope="module")
def traj() -> pc.Trajectory:
    return pc.load_or_generate_trajectory()


def test_torque_envelopes_match_the_source_studies():
    """rho and the derated joint-4 limit are the papers' numbers, not fresh ones."""
    rho_cap = pc.torque_envelope("rho_0.28")
    np.testing.assert_allclose(rho_cap, 0.28 * pc.TAU_LIMIT)
    j4_cap = pc.torque_envelope("derated_joint4")
    assert j4_cap[pc.DERATED_JOINT] == pytest.approx(31.5)
    # Every other joint keeps the stock FR3 limit.
    others = [i for i in range(7) if i != pc.DERATED_JOINT]
    np.testing.assert_allclose(j4_cap[others], pc.TAU_LIMIT[others])
    with pytest.raises(ValueError):
        pc.torque_envelope("not_an_envelope")


def test_scenario_constants_are_pulled_from_the_repo():
    """The 0.06 m bound is phri2's own position_limit, not a hardcoded copy."""
    assert pc.WORKSPACE_BOUND_M == pc.FR3MPCConfig().position_limit
    # The push profile is run_fr3_experiments.human_force_at at full hold.
    peak = pc.human_force_at(2.0)
    assert np.linalg.norm(peak) == pytest.approx(pc.PUSH_MAGNITUDE_N)
    np.testing.assert_allclose(peak / np.linalg.norm(peak), pc.PUSH_AXIS)


def test_trajectory_is_the_phri2_benchmark(traj: pc.Trajectory):
    assert traj.n_ticks == pytest.approx(pc.DURATION_S * 1000, rel=1e-3)
    # phri2's predictive realization keeps the EE inside its own workspace box
    # on the constrained axis; if this drifts the replay is no longer the
    # benchmark the scan claims to be reusing.
    assert np.abs(traj.e[:, pc.PUSH_AXIS_INDEX]).max() <= pc.WORKSPACE_BOUND_M + 5e-4
    assert np.abs(traj.f_h).max() == pytest.approx(pc.PUSH_MAGNITUDE_N, rel=1e-6)


def test_anchor_is_tau_base_plus_jacobian_transpose_f_nom(traj: pc.Trajectory):
    """The vectorised einsum must agree with the per-tick matrix product."""
    k, d = 400.0, 25.0
    f_nom = pc.nominal_force(k, d, traj.e, traj.v)
    vectorised = np.einsum("tij,ti->tj", traj.J_v, f_nom)
    for t in (0, 1234, traj.n_ticks - 1):
        np.testing.assert_allclose(vectorised[t], traj.J_v[t].T @ f_nom[t], rtol=1e-12)


def test_diagnostic_1_dominates_diagnostic_4(traj: pc.Trajectory):
    """The anchor includes tau_base, so it can never be below the F_nom-only part
    by more than tau_base itself -- a sanity relation between rows 1 and 4."""
    cap = pc.torque_envelope("derated_joint4")
    cell = pc.replay_diagnostics(traj, 400.0, 25.0, cap)
    base = pc.base_only_anchor_ratio(traj, cap)["base_only_anchor_ratio"]
    assert cell["anchor_ratio"] <= cell["budget_ratio"] + base + 1e-9
    assert cell["anchor_ratio"] >= 0.0 and cell["budget_ratio"] >= 0.0


def test_zero_gains_reduce_the_anchor_to_tau_base(traj: pc.Trajectory):
    cap = pc.torque_envelope("rho_0.28")
    cell = pc.replay_diagnostics(traj, 0.0, 0.0, cap)
    base = pc.base_only_anchor_ratio(traj, cap)
    assert cell["anchor_ratio"] == pytest.approx(base["base_only_anchor_ratio"])
    assert cell["budget_ratio"] == pytest.approx(0.0)


def test_rho_envelope_cannot_even_carry_tau_base_plus_the_push(traj: pc.Trajectory):
    """The headline structural result: under rho = 0.28 the bias torque alone
    eats 82% of joint 4's cap, and phri2's own realized torque overruns it."""
    cap = pc.torque_envelope("rho_0.28")
    base = pc.base_only_anchor_ratio(traj, cap)
    assert base["base_only_worst_joint"] == pc.DERATED_JOINT
    assert base["base_only_anchor_ratio"] > 0.8
    assert np.max(np.abs(traj.tau_realized) / cap[None, :]) > 1.0


@pytest.mark.parametrize("k", [200.0, 800.0])
def test_fallback_equilibrium_balances_the_push(k: float):
    """At the alpha -> 0 equilibrium K0 e = F_h, so k|e| should recover 20 N.

    This is the relation that makes diagnostic 1b's verdict a property of the
    push rather than of the gains.
    """
    result = pc.run_fallback_equilibrium(k, 25.0)
    assert result["settled"], "fallback did not settle inside the push"
    assert k * result["e_ss_axis"] == pytest.approx(pc.PUSH_MAGNITUDE_N, rel=0.12)


def test_fallback_is_measured_after_the_transient_has_decayed():
    """The averaging window must be long enough to cover a full oscillation of
    the fallback spring, or the 'steady state' is an artifact of the window."""
    k = 344.2
    task_inertia_kg = 4.6  # Lambda_zz at the neutral pose, see the draft
    period_s = 2 * np.pi * np.sqrt(task_inertia_kg / k)
    assert pc.FALLBACK_AVERAGING_WINDOW_S > period_s
    result = pc.run_fallback_equilibrium(k, 2.0)  # the least damped case
    assert result["e_ss_spread_axis"] < 0.1 * pc.WORKSPACE_BOUND_M


def test_scan_is_reproducible_on_a_tiny_grid():
    """A 2x2 scan must produce every field the decision gate reads."""
    report = run_scan(np.array([50.0, 600.0]), np.array([10.0, 30.0]), workers=2)
    assert set(report["envelopes"]) == set(pc.ENVELOPE_NAMES)
    for block in report["envelopes"].values():
        for key in ("diag1_anchor_ratio_replay", "diag1b_anchor_ratio_fallback",
                    "diag3_e_ss_axis_m", "diag4_budget_ratio"):
            assert np.array(block[key]).shape == (2, 2)
        assert isinstance(block["common_region_nonempty"], bool)
        assert block["diag2_passive"] is True


def test_published_grid_resolves_the_feasible_band():
    """The feasible sliver is thin enough that a coarse grid misses it entirely;
    the published grid must keep points inside the band that carries it."""
    band = K_GRID[(K_GRID >= 300.0) & (K_GRID <= 520.0)]
    assert len(band) >= 10, "k grid is too coarse through the feasible band"
    assert (np.diff(band) <= 25.0).all()
    assert D_GRID.min() <= 8.0 and D_GRID.max() >= 40.0
