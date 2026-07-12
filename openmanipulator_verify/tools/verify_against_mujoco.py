#!/usr/bin/env python3
"""Validate the OpenManipulator-X harness against an INDEPENDENT physics model
(MuJoCo, from the ROBOTIS URDF) before trusting it on hardware:

  1. FK + Jacobian vs MuJoCo ground truth,
  2. gravity G(q) + mass matrix M(q) vs MuJoCo,
  3. the J1-J4 control suite on MuJoCo physics (hold / circle / push / payload),
     checking each reaches a small steady-state error (offset-free).

Requires `mujoco` and `robot_descriptions`. Run from the package root:
    python3 tools/verify_against_mujoco.py
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
sys.path.insert(0, str(ROOT / "verification"))


def main() -> None:
    import mujoco
    from mujoco_sim import build_omx_model, ARM_JOINTS, TOOL_BODY
    from kinematics import OpenManipulatorKinematics
    from dynamics import OpenManipulatorDynamics
    from analyze_log import load_csv, metrics

    m = build_omx_model(); d = mujoco.MjData(m)
    kin = OpenManipulatorKinematics(); dyn = OpenManipulatorDynamics()
    aq = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM_JOINTS]
    av = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM_JOINTS]
    tool = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)
    tcp = np.array(kin.links["d4e"]); M = np.zeros((m.nv, m.nv))
    rng = np.random.default_rng(0)
    fk_e = j_e = g_e = m_e = 0.0
    for _ in range(12):
        q = rng.uniform(-1.0, 1.0, 4); d.qpos[:] = 0; d.qvel[:] = 0
        for a, v in zip(aq, q): d.qpos[a] = v
        mujoco.mj_forward(m, d)
        mj_tcp = d.xpos[tool] + d.xmat[tool].reshape(3, 3) @ tcp
        fk_e = max(fk_e, np.linalg.norm(mj_tcp - kin.fk(q)))
        jp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, jp, None, mj_tcp, tool)
        j_e = max(j_e, np.max(np.abs(jp[:, av] - kin.jacobian(q))))
        g_e = max(g_e, np.max(np.abs(dyn.gravity(q) - np.array([d.qfrc_bias[a] for a in av]))))
        mujoco.mj_fullM(m, d, M)
        m_e = max(m_e, np.max(np.abs(dyn.mass_matrix(q) - M[np.ix_(av, av)])))

    print("[1] FK vs MuJoCo:        max %.2e m   %s" % (fk_e, "OK" if fk_e < 1e-3 else "FAIL"))
    print("[1] Jacobian vs MuJoCo:  max %.2e     %s" % (j_e, "OK" if j_e < 1e-3 else "FAIL"))
    print("[2] gravity vs MuJoCo:   max %.2e Nm  %s" % (g_e, "OK" if g_e < 2e-2 else "FAIL"))
    print("[2] mass M vs MuJoCo:    max %.2e     %s" % (m_e, "OK" if m_e < 2e-2 else "FAIL"))

    print("[3] control suite on MuJoCo physics:")
    ok = fk_e < 1e-3 and j_e < 1e-3 and g_e < 2e-2 and m_e < 2e-2
    for name, dur, thr in (("hold", 8, 5.0), ("circle", 18, 6.0), ("push", 18, 6.0), ("payload", 18, 6.0)):
        csv = ROOT / "results" / "hardware" / f"mjc_{name}.csv"
        subprocess.run([sys.executable, str(ROOT / "verification" / "run_hardware_verification.py"),
                        "--test-id", f"mjc_{name}", "--backend", "mjc",
                        "--config", str(ROOT / "configs" / f"{name}.yaml"), "--duration", str(dur)],
                       capture_output=True)
        mt = metrics(load_csv(csv))
        good = mt["steady_state_error_mm"] < thr
        ok = ok and good
        print("    %-8s SS=%5.2f mm  max=%6.1f mm  %s" % (name, mt["steady_state_error_mm"], mt["max_error_mm"],
                                                          "OK" if good else "FAIL"))
    print("PASS: harness validated against MuJoCo physics" if ok else "FAIL: see above")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
