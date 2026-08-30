"""Turn RuntimeWarning into a test failure in this directory.

Added in response to an external review that found a feasible admittance
solve emitted 9 repeatable NumPy RuntimeWarnings (divide-by-zero/overflow/
invalid-value in matmul) despite returning finite, correct results -- not
caught because a warning alone doesn't fail a test. The exact source was
not reproduced here (unlike a near-identical finding in the sibling
pHRI/impedance project, this codebase's QP assembly is already all-dense
NumPy with no scipy.sparse mixing, so that specific root cause does not
apply); finiteness is now checked explicitly after QP assembly and solve
in interaction_dynamics_mpc.py and fr3_interaction_dynamics_mpc.py
regardless. This is the backstop that makes any recurrence, here or
elsewhere, loud instead of silent.
"""
import warnings

import pytest


@pytest.fixture(autouse=True)
def _fail_on_runtime_warning():
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        yield
