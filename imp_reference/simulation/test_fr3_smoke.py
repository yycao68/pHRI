"""Clean-environment smoke test for the FR3/MuJoCo manipulator study.

Added in response to an external review that could not reproduce the FR3
results from a fresh checkout (blocked at MuJoCo mesh loading -- the
Menagerie assets are gitignored in the shared pHRI/simulation/ tree this
project imports from, and are fetched by pHRI/simulation/setup_model.py
instead; see that script and its own test_model_smoke.py for the asset-
pinning fix). This is the fast, minimal check CI should run before paying
for the full FR3 experiment suite: does the shared FR3 model actually load
and can FR3RealizationMPC.control complete one real solve.

Skips (not fails) if the model hasn't been downloaded yet.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
SIM = HERE.parents[1] / "simulation"
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


def test_fr3_realization_mpc_completes_one_solve():
    from fr3_interaction_dynamics_mpc import (
        AdmittanceReference3D,
        FR3MPCConfig,
        FR3RealizationMPC,
    )
    from fr3_mujoco import FR3MuJoCoEnv

    env = FR3MuJoCoEnv(timestep=0.001)
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()
    cfg = FR3MPCConfig(horizon=5)
    mpc = FR3RealizationMPC(AdmittanceReference3D(), cfg)
    force_forecast = np.zeros((cfg.horizon, 3))
    step = mpc.control(dyn, state, p_nominal, R_d, force_forecast)
    assert np.all(np.isfinite(step.command))
    print(f"OK: FR3RealizationMPC completed one solve, command={step.command}, "
          f"status={step.status}")


if __name__ == "__main__":
    test_fr3_model_loads_and_steps()
    test_fr3_realization_mpc_completes_one_solve()
