"""Regression test for the online generator-switching demonstration.

Mirrors ``test_benchmark_verification.py``'s reasoning: a driver script with
no assertions is not a verified claim. This runs the actual switching
scenario (one controller instance, ``.generator`` reassigned at each switch)
and asserts the properties the paper's new experiment reports directly.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference,
    ImpedanceReference,
    InteractionDynamicsMPC,
    MPCConfig,
)
from run_generator_switching_experiment import (  # noqa: E402
    SWITCH_TIMES,
    metrics,
    run_switching_case,
)


def test_switching_respects_workspace_and_speed_bounds():
    cfg = MPCConfig()
    log = run_switching_case(cfg)
    m = metrics(log, cfg)
    assert m["max_abs_position_m"] < cfg.position_limit
    assert m["max_speed_mps"] < cfg.speed_limit


def test_switching_never_exceeds_the_controllers_own_rate_limit():
    """The whole point: swapping .generator on a live controller instance
    produces no discontinuity beyond what the QP's own command-rate
    constraint already permits -- no special-casing at the switch."""
    cfg = MPCConfig()
    log = run_switching_case(cfg)
    m = metrics(log, cfg)
    assert len(m["command_jump_at_switch_N"]) == len(SWITCH_TIMES)
    assert m["no_switch_jump_exceeds_rate_limit"]


def test_only_the_generator_attribute_changes_across_a_switch():
    """Construct the controller once, swap .generator, and confirm the
    plant matrices and previous_command state are untouched by the swap --
    the literal content of Theorem 1 (Affine Generator Independence)."""
    cfg = MPCConfig()
    controller = InteractionDynamicsMPC(ImpedanceReference(), cfg)
    a_before, b_before = controller.A.copy(), controller.B.copy()
    controller.control(np.zeros(4), np.zeros((cfg.horizon, 2)))
    prev_command_before = controller.previous_command.copy()

    controller.generator = AdmittanceReference()

    np.testing.assert_array_equal(controller.A, a_before)
    np.testing.assert_array_equal(controller.B, b_before)
    np.testing.assert_array_equal(controller.previous_command, prev_command_before)


def test_impedance_phase_converges_to_the_expected_equilibrium():
    """Both impedance segments (before and after the admittance excursion)
    should settle near F_h/K_d, regardless of the very different state the
    second segment starts from -- no special-casing on re-entry."""
    cfg = MPCConfig()
    log = run_switching_case(cfg)
    time = log["time"]
    position = log["state"][:, 1]
    expected_equilibrium = 1.0 / 45.0  # FORCE_MAGNITUDE / ImpedanceReference().stiffness

    end_of_first_impedance = position[np.argmin(np.abs(time - (SWITCH_TIMES[0] - 0.02)))]
    end_of_run = position[-1]
    assert abs(end_of_first_impedance - expected_equilibrium) < 5e-3
    assert abs(end_of_run - expected_equilibrium) < 5e-3


def test_admittance_phase_keeps_drifting_with_no_restoring_term():
    cfg = MPCConfig()
    log = run_switching_case(cfg)
    time = log["time"]
    position = log["state"][:, 1]
    start_idx = np.argmin(np.abs(time - SWITCH_TIMES[0]))
    end_idx = np.argmin(np.abs(time - (SWITCH_TIMES[1] - 0.02)))
    assert position[end_idx] > position[start_idx] + 0.02
