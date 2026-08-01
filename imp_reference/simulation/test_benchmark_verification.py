"""End-to-end verification of the planar benchmark's headline claims (paper.md Section 5).

Mirrors ``test_fr3_benchmark_verification.py``'s reasoning: the unit tests in
``test_interaction_dynamics_mpc.py`` check the QP on hand-picked single
states; none of them run the actual 6-second scripted-force scenario Table 1
and Figure 2 report numbers from, and ``run_experiments.py`` itself asserts
nothing. This module runs that scenario directly and checks the claims.

The planar plant is an exact discrete-time LTI system with no randomness
anywhere in the loop, so unlike the FR3 study, results here should reproduce
bit-for-bit run to run; this was confirmed empirically before writing the
tolerances below, which are still kept as tolerances rather than exact
equality so a deliberate, disclosed retuning of the scenario does not force
an unrelated edit to this file.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from interaction_dynamics_mpc import AdmittanceReference, ImpedanceReference, MPCConfig  # noqa: E402
from run_experiments import metrics, run_case  # noqa: E402


def _all_conditions():
    if not hasattr(_all_conditions, "_cache"):
        cfg = MPCConfig()
        generators = {"impedance": ImpedanceReference(), "admittance": AdmittanceReference()}
        result = {}
        for name, generator in generators.items():
            for controller_kind in ("mpc", "clipped"):
                log = run_case(generator, controller_kind, cfg)
                result[(name, controller_kind)] = metrics(log, cfg)
        _all_conditions._cache = result
    return _all_conditions._cache


def test_predictive_realization_respects_workspace_and_speed_bounds():
    """Table 1: predictive realization's position/speed violation is within
    numerical solver tolerance (<= 1e-5 m / m/s reported), for both
    generators. A generous 1 mm / 1 mm/s envelope is used here so this test
    tracks a real regression, not solver-tolerance jitter."""
    for generator_name in ("impedance", "admittance"):
        m = _all_conditions()[(generator_name, "mpc")]
        assert m["position_violation_m"] <= 1e-3, (
            f"{generator_name} predictive: position violation {m['position_violation_m']:.2e} m"
        )
        assert m["component_speed_violation_mps"] <= 1e-3, (
            f"{generator_name} predictive: speed violation {m['component_speed_violation_mps']:.2e} m/s"
        )


def test_reactive_clipping_violates_bounds_predictive_does_not():
    """The paper's central contrast (Table 1, Figure 2): reactive clipping,
    with no lookahead on state constraints, overshoots the workspace bound
    it was configured with; predictive realization does not. Table 1 reports
    0.166 m / 0.425 m position violation for reactive impedance/admittance --
    require at least half that (0.05 m) so the test catches a real erosion
    of the effect rather than noise."""
    for generator_name in ("impedance", "admittance"):
        m = _all_conditions()[(generator_name, "clipped")]
        assert m["position_violation_m"] >= 0.05, (
            f"{generator_name} reactive: position violation only {m['position_violation_m']:.4f} m -- "
            "the paper's headline predictive-vs-reactive contrast does not hold"
        )


def test_admittance_never_recovers_displacement_reactively():
    """Section 5.2: admittance has no position-restoring term, so reactive
    clipping should retain its peak displacement after force release, unlike
    impedance which returns toward its equilibrium at the origin."""
    m = _all_conditions()[("admittance", "clipped")]
    assert abs(m["final_y_m"]) >= 0.9 * abs(m["peak_y_m"]), (
        "admittance reactive clipping recovered displacement after force release "
        "-- contradicts the generator's no-restoring-term property (Section 3.2)"
    )


def test_impedance_predictive_returns_to_equilibrium_after_release():
    """Section 5.2: after the force releases, predictive realization should
    return to the impedance equilibrium at the origin (final position near
    zero), unlike the reactive comparator (checked separately above)."""
    m = _all_conditions()[("impedance", "mpc")]
    assert abs(m["final_y_m"]) <= 5e-3, (
        f"impedance predictive: final position {m['final_y_m']:.4f} m did not "
        "return near the origin after force release"
    )
