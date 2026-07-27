"""End-to-end verification of the FR3 benchmark's headline claims (paper.md Section 8).

The unit tests in ``test_fr3_interaction_dynamics_mpc.py`` check the QP's
mechanics in isolation (one solve, hand-picked states). None of them run the
actual 6-second, closed-loop MuJoCo benchmark scenario the paper reports
numbers from -- that scenario is only ever executed by
``run_fr3_experiments.py``, a driver script with zero assertions. Nothing
would catch it if a future change (a QP weight, a MuJoCo model file, an OSQP
version bump) silently broke the claims Table 2 in the paper reports.

This module closes that gap: it runs the real benchmark scenario (the same
``run_case`` the paper's figure and table are generated from) and asserts the
claims directly, so a regression fails a test instead of only showing up as
a quieter number in a table nobody re-checks.

Physical quantities here (position, torque, violation counts) were
deterministic across repeated runs in the development environment. Solve
times are wall-clock and are NOT reproducible run-to-run; this module only
ever bounds them, never asserts an exact value.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

from fr3_mujoco import FR3MuJoCoEnv  # noqa: E402

from fr3_interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference3D,
    FR3MPCConfig,
    ImpedanceReference3D,
)
from run_fr3_experiments import componentwise_rmse, metrics, run_case  # noqa: E402

_DURATION = 6.0  # matches paper.md Section 8.2; full scenario, not a shortened smoke test


def test_residual_rmse_uses_componentwise_convention():
    """Table 2 must pool components and samples, matching planar Table 1."""
    residual = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 0.0]])
    assert np.isclose(componentwise_rmse(residual), np.sqrt(25.0 / 6.0))


def _all_conditions():
    """Run all four (generator, controller) conditions once and return their metrics.

    Module-level cache: the benchmark is deterministic and re-running it per
    test would multiply a ~20 s MuJoCo simulation across every assertion.
    """
    if not hasattr(_all_conditions, "_cache"):
        cfg = FR3MPCConfig()
        env = FR3MuJoCoEnv(timestep=0.001)
        generators = {"impedance": ImpedanceReference3D(), "admittance": AdmittanceReference3D()}
        result = {}
        for name, generator in generators.items():
            for controller_kind in ("mpc", "clipped"):
                log = run_case(env, generator, controller_kind, cfg, duration=_DURATION)
                result[(name, controller_kind)] = metrics(log, cfg)
        _all_conditions._cache = result
    return _all_conditions._cache


def test_no_torque_violation_in_any_condition():
    """The claim actually load-bearing for the whole architecture: per-joint
    torque feasibility, enforced at every horizon step, must never be
    violated at the executed sample in any of the four conditions."""
    for key, m in _all_conditions().items():
        assert m["torque_violation_Nm"] == 0.0, f"{key}: torque limit violated ({m['torque_violation_Nm']} Nm)"
        assert m["max_torque_utilization"] <= 1.0 + 1e-8


def test_reports_empirical_and_model_predicted_residuals_separately():
    """The paper distinguishes the residual observed from MuJoCo motion from
    the residual reconstructed through the controller model. Both must be
    finite, and the legacy realization key must denote the empirical value."""
    for key, m in _all_conditions().items():
        assert np.isfinite(m["empirical_realization_rmse_mps2"]), key
        assert np.isfinite(m["predicted_realization_rmse_mps2"]), key
        assert m["realization_rmse_mps2"] == m["empirical_realization_rmse_mps2"]


def test_predictive_realization_has_no_unhandled_infeasibility():
    """paper.md Section 8.3: 'no solve is reported infeasible in this
    scenario.' If a future change makes solves infeasible, the reactive
    fallback keeps the sim running (Section 8.1) but the paper's claim would
    silently become false -- this must fail loudly instead."""
    for generator_name in ("impedance", "admittance"):
        m = _all_conditions()[(generator_name, "mpc")]
        assert m["n_infeasible_solves"] == 0, (
            f"{generator_name}: {m['n_infeasible_solves']} solves fell back to the "
            "reactive law -- either the frozen-Jacobian model drifted more than "
            "expected, or a fallback that should be rare has become common"
        )


def test_predictive_realization_respects_workspace_bound():
    """Table 2: predictive peak |e_z| is 0.0602 m / 0.0601 m against a 0.06 m
    bound -- within a fraction of a millimeter, via the slack-relaxed soft
    constraint. This asserts a 2 mm envelope around the bound (roughly 10x
    the largest observed overshoot of ~0.2 mm): tight enough to catch a real
    regression, loose enough not to be flaky against minor retuning. An
    earlier version of this test used a 1 cm envelope -- 50x looser than the
    actual margin -- which would not have caught a regression an order of
    magnitude worse than what is actually reported."""
    cfg = FR3MPCConfig()
    for generator_name in ("impedance", "admittance"):
        m = _all_conditions()[(generator_name, "mpc")]
        assert m["max_abs_position_m"] <= cfg.position_limit + 0.002, (
            f"{generator_name} predictive: peak |e_z|={m['max_abs_position_m']:.4f} m "
            f"is more than 2 mm past the {cfg.position_limit} m bound"
        )


def test_slack_usage_is_logged_small_and_controller_specific():
    """Section 8.1 reports that predictive solves genuinely use small state
    slack, while the reactive comparator has no QP slack. Protect the saved
    evidence itself, not only the resulting physical workspace envelope."""
    for generator_name in ("impedance", "admittance"):
        predictive = _all_conditions()[(generator_name, "mpc")]
        assert predictive["n_solves_with_nontrivial_slack"] > 0
        assert 0.0 < predictive["max_position_slack_m"] < 1.0e-4
        assert 0.0 <= predictive["max_speed_slack_mps"] < 1.0e-6

        reactive = _all_conditions()[(generator_name, "clipped")]
        assert reactive["n_solves_with_nontrivial_slack"] == 0
        assert reactive["max_position_slack_m"] == 0.0
        assert reactive["max_speed_slack_mps"] == 0.0


def test_reactive_clipping_overshoots_predictive_realization():
    """The paper's central empirical contrast (Table 2, Figure 4): under an
    identical command/rate box and an identical generator, reactive clipping
    -- which has no lookahead on the workspace bound -- displaces the EE
    further than predictive realization. Checked as a relative comparison
    (predictive < reactive), not against hardcoded absolute values, so the
    test survives minor retuning as long as the qualitative story holds."""
    for generator_name in ("impedance", "admittance"):
        predictive = _all_conditions()[(generator_name, "mpc")]
        reactive = _all_conditions()[(generator_name, "clipped")]
        assert reactive["max_abs_position_m"] > predictive["max_abs_position_m"], (
            f"{generator_name}: reactive clipping ({reactive['max_abs_position_m']:.4f} m) "
            f"did not overshoot predictive realization ({predictive['max_abs_position_m']:.4f} m) -- "
            "the paper's headline predictive-vs-reactive contrast does not hold"
        )
        # Table 2 shows roughly 1.7x-2.3x; require at least a 1.3x margin so the
        # test fails on a genuine erosion of the effect, not on noise.
        assert reactive["max_abs_position_m"] >= 1.3 * predictive["max_abs_position_m"], (
            f"{generator_name}: reactive overshoot margin over predictive has shrunk "
            f"to {reactive['max_abs_position_m'] / predictive['max_abs_position_m']:.2f}x, "
            "well below the reported ~1.7-2.3x -- re-check before citing Table 2's numbers"
        )


def test_admittance_never_recovers_displacement_reactively():
    """Section 8.3: the admittance generator has no position-restoring term
    (Section 4.2), so reactive clipping should retain most of its peak
    displacement after the force releases, unlike impedance which returns
    toward its equilibrium. Checked via final vs. peak displacement."""
    m = _all_conditions()[("admittance", "clipped")]
    assert abs(m["final_z_m"]) >= 0.9 * abs(m["peak_z_m"]), (
        "admittance reactive clipping recovered displacement after force release "
        "-- contradicts the generator's no-restoring-term property (Section 4.2)"
    )


def test_solve_times_are_bounded_not_reproduced_exactly():
    """Solve times are wall-clock and vary with machine load. This only
    bounds them generously against runaway regression (e.g., a conditioning
    change that makes OSQP take seconds per solve); it does not assert the
    particular values recorded by one benchmark execution."""
    for key, m in _all_conditions().items():
        if m["mean_solve_time_ms"] > 0.0:  # reactive comparator does not solve a QP
            assert m["mean_solve_time_ms"] < 500.0, f"{key}: mean solve time {m['mean_solve_time_ms']:.1f} ms"
            assert m["max_solve_time_ms"] < 2000.0, f"{key}: max solve time {m['max_solve_time_ms']:.1f} ms"
