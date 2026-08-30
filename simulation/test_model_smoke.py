"""Clean-environment smoke test: does the FR3 model actually load and run.

Added in response to an external review that could not reproduce the paper's
FR3 benchmarks from a fresh checkout (the mesh assets are gitignored --
`**/assets/`, `cloud_verify/` -- to keep the repo small; `setup_model.py`
fetches them from mujoco_menagerie instead). This test is the fast,
minimal check CI should run to catch "the model doesn't load" before paying
for a full benchmark run: it does not check numerical correctness (see
verify_tables_3_4.py for that), only that the environment loads and a
controller can take one real step without erroring.

Skips (not fails) if the model hasn't been downloaded yet, with a message
pointing at the fix, since running `setup_model.py` is a documented
prerequisite, not something this test should do implicitly on every CI run
(network access, ~30MB download).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # so `fr3_mujoco`/`fair_offset_free_comparison`
                                # resolve regardless of pytest's invocation cwd
SCENE_XML = HERE / "models" / "franka_fr3" / "fr3_phri_scene.xml"

pytestmark = pytest.mark.skipif(
    not SCENE_XML.exists(),
    reason=f"FR3 model not downloaded (missing {SCENE_XML}); "
           f"run `python3 simulation/setup_model.py` first",
)


def test_model_loads_and_steps():
    from fr3_mujoco import FR3MuJoCoEnv

    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    assert env.nv == 7
    env.step()
    assert all(map(__import__("math").isfinite, env.q))
    print(f"OK: FR3 model loaded ({env.nv} DOF) and stepped, q={env.q}")


def test_controller_completes_one_step():
    """One real MPC-controller step through the same path
    fair_offset_free_comparison.run() uses for the paper's tables, not just
    a raw physics step -- this is what the review's smoke-test request
    actually asked for."""
    from fair_offset_free_comparison import run

    result = run("DI-MPC 100 Hz", cycles=1)
    metrics = result["metrics"]
    assert all(
        __import__("math").isfinite(v) for v in metrics.values() if v is not None
    )
    assert metrics["qp_solve_count"] > 0, "expected at least one real QP solve"
    print(f"OK: controller completed a short run ({metrics['qp_solve_count']} QP "
          f"solves, {metrics['qp_failure_count']} failures)")


if __name__ == "__main__":
    test_model_loads_and_steps()
    test_controller_completes_one_step()
