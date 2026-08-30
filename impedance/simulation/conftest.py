"""Turn RuntimeWarning into a test failure in this directory.

Added in response to an external review that found passing tests emitting
repeatable NumPy RuntimeWarnings (divide-by-zero/overflow/invalid-value in
matmul) at QP Hessian assembly -- not caught because a warning alone doesn't
fail a test. The root cause (dense/sparse matrix mixing in the Hessian
construction) is fixed directly in verify_residual_mpc.py,
verify_two_rate_passive_residual.py, and verify_fr3_two_rate_benchmark.py;
this is the backstop that makes any future recurrence (here or elsewhere)
loud instead of silent.
"""
import warnings

import pytest


@pytest.fixture(autouse=True)
def _fail_on_runtime_warning():
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        yield
