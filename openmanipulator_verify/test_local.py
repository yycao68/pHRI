#!/usr/bin/env python3
"""Dependency-light smoke test (numpy only): checks FK/Jacobian sanity and that
the control path runs without diverging on the lightweight sim plant.

This is only a coarse "does it run" check. For RIGOROUS validation against an
independent physics model (the real ROBOTIS URDF), run:

    python3 tools/verify_against_mujoco.py     # needs mujoco + robot_descriptions

which validates FK/Jacobian/gravity/mass to machine precision and confirms the
J1-J4 suite reaches offset-free steady state on MuJoCo physics.
"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "lib"))
sys.path.insert(0, str(ROOT / "verification"))

from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from run_openmanipulator_hardware import run  # noqa: E402
from analyze_log import load_csv, metrics  # noqa: E402


def main() -> None:
    kin = OpenManipulatorKinematics()
    q = np.array([0.0, -0.6, 0.3, 0.3])
    ee, J = kin.fk(q), kin.jacobian(q)
    assert ee.shape == (3,) and J.shape == (3, 4), "FK/Jacobian shape"
    assert np.linalg.norm(ee) > 0.05, f"EE too close to base: {ee}"
    print(f"[test] FK(home)={np.round(ee, 4)} m, |J|={np.linalg.norm(J):.3f}")

    # Regression check: the fast, Jacobian-formula mass_matrix() must stay
    # numerically identical to the original (slower) n-RNEA-call approach --
    # not just fast, but provably the same answer.
    dyn = OpenManipulatorDynamics()
    rng = np.random.default_rng(0)
    max_M_err = 0.0
    for _ in range(50):
        q_r = rng.uniform(-2.0, 2.0, 4)
        max_M_err = max(max_M_err, np.max(np.abs(dyn.mass_matrix(q_r) - dyn._mass_matrix_rnea(q_r))))
    assert max_M_err < 1e-9, f"fast mass_matrix() diverged from _mass_matrix_rnea by {max_M_err}"
    print(f"[test] dynamics: fast mass_matrix() matches the slow RNEA reference "
          f"(max err {max_M_err:.2e} N.m over 50 random samples)")

    out = ROOT / "results" / "hardware"
    for cfg, dur in (("hold", 6), ("payload", 12)):
        args = SimpleNamespace(config=ROOT / "configs" / f"{cfg}.yaml", backend="sim",
                               duration=dur, output=out / f"sim_{cfg}.csv", port=None, baud=1000000)
        run(args)
        m = metrics(load_csv(out / f"sim_{cfg}.csv"))
        print(f"[test] sim_{cfg}: max={m['max_error_mm']:.1f} mm  SS={m['steady_state_error_mm']:.1f} mm")
        assert np.isfinite(m["max_error_mm"]) and m["max_error_mm"] < 300.0, f"sim_{cfg} diverged"

    print("[test] smoke test passed (control path runs, no divergence).")
    print("[test] for rigorous offset-free validation: python3 tools/verify_against_mujoco.py")


if __name__ == "__main__":
    main()
