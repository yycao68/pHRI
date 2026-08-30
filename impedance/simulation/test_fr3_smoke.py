"""Clean-environment smoke test for the 7-DoF FR3 two-rate benchmark.

Added in response to an external review that could not reproduce the FR3
results from a fresh checkout (blocked at MuJoCo mesh loading -- the
Menagerie assets are gitignored in the shared pHRI/simulation/ tree this
file imports from, and are fetched by pHRI/simulation/setup_model.py
instead; see that script and its own test_model_smoke.py for the asset-
pinning fix). This is the fast, minimal check CI should run before paying
for a full 20-trial benchmark: does the shared FR3 model actually load, and
can `run_trial` complete a short but valid run (duration >= 0.3s, so the
0.25s warmup window used by every metric is non-empty -- found and guarded
directly in run_trial while building this test) without error.

Skips (not fails) if the model hasn't been downloaded yet.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
SIM = HERE.parent.parent / "simulation"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(SIM))

SCENE_XML = SIM / "models" / "franka_fr3" / "fr3_phri_scene.xml"

pytestmark = pytest.mark.skipif(
    not SCENE_XML.exists(),
    reason=f"FR3 model not downloaded (missing {SCENE_XML}); "
           f"run `python3 ../../simulation/setup_model.py` first",
)


def test_fr3_model_loads_and_steps():
    from fr3_mujoco import FR3MuJoCoEnv

    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    assert env.nv == 7
    env.step()
    assert all(map(__import__("math").isfinite, env.q))
    print(f"OK: FR3 model loaded ({env.nv} DOF) and stepped, q={env.q}")


def test_two_rate_controller_completes_a_short_valid_trial():
    from verify_fr3_two_rate_benchmark import Config, run_trial

    result = run_trial("two_rate", Config(duration=0.3), seed=0)
    metrics = result["metrics"]
    assert all(__import__("math").isfinite(v) for v in metrics.values())
    assert metrics["qp_failures"] == 0
    print(f"OK: two_rate completed a 0.3s trial, "
          f"residual_rms_mm={metrics['residual_rms_mm']:.4f}, "
          f"qp_failures={metrics['qp_failures']}")


if __name__ == "__main__":
    test_fr3_model_loads_and_steps()
    test_two_rate_controller_completes_a_short_valid_trial()
