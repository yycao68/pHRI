"""MuJoCo physics backend for the OpenManipulator-X, for rigorous off-hardware
validation. The controller's torques are applied to the *real* URDF rigid-body
dynamics (true inertias + gravity from ROBOTIS's model), so the controller's
approximate gravity model and kinematics are tested against ground truth -- not
against their own assumptions. External pushes/payloads are applied as
Cartesian forces on the tool link.

Requires `mujoco` and `robot_descriptions` (only for --backend mjc).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from dynamixel_backend import RobotIO

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
TOOL_BODY = "link5"


def build_omx_model():
    import mujoco
    from robot_descriptions import open_manipulator_x_description as omx
    urdf = Path(omx.URDF_PATH)
    pkg_root = urdf.parents[2]  # .../open_manipulator_description
    txt = urdf.read_text().replace("package://open_manipulator_description/", str(pkg_root) + "/")
    tmp = urdf.with_name("open_manipulator_x_mjfixed.urdf")
    tmp.write_text(txt)
    return mujoco.MjModel.from_xml_path(str(tmp))


class MujocoArmBackend:
    def __init__(self, config: dict, kin):
        import mujoco
        self.mj = mujoco
        self.kin = kin
        self.m = build_omx_model()
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = 0.002
        self.dt = float(config.get("controller", {}).get("dt", 0.01))
        self.substeps = max(1, int(round(self.dt / self.m.opt.timestep)))

        def jid(n): return mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qadr = [self.m.jnt_qposadr[jid(n)] for n in ARM_JOINTS]
        self.dadr = [self.m.jnt_dofadr[jid(n)] for n in ARM_JOINTS]
        self.grip_qadr = [self.m.jnt_qposadr[jid(n)] for n in GRIPPER_JOINTS]
        self.grip_dadr = [self.m.jnt_dofadr[jid(n)] for n in GRIPPER_JOINTS]
        self.tool = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)
        self.d4e = np.asarray(kin.links["d4e"], dtype=float)

        robot = config.get("robot", {})
        self.kt = float(robot.get("torque_constant_Nm_per_A", 1.78))
        q0 = np.asarray(robot.get("sim_home_q_rad", [0.0, -0.6, 0.3, 0.3]), dtype=float)
        for a, v in zip(self.qadr, q0):
            self.d.qpos[a] = v
        mujoco.mj_forward(self.m, self.d)
        self._tau = np.zeros(4)
        self.t = 0.0
        self.first = True

        d = config.get("disturbance", {})
        self.pushes = d.get("push", []) or []
        if isinstance(self.pushes, dict): self.pushes = [self.pushes]
        self.payloads = d.get("payload", []) or []
        if isinstance(self.payloads, dict): self.payloads = [self.payloads]

    def _f_ext(self, t: float) -> np.ndarray:
        F = np.zeros(3)
        for p in self.pushes:
            t0, t1 = float(p.get("t_start", 0.0)), float(p.get("t_end", 1.0))
            amp = np.asarray(p.get("force_N", [0.0, 0.0, 4.0]), dtype=float)
            if t0 <= t < t1 and t1 > t0:
                F = F + amp * 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / (t1 - t0)))
        for q in self.payloads:
            if t >= float(q.get("t_start", 0.0)):
                F = F + np.asarray(q.get("force_N", [0.0, 0.0, -2.0]), dtype=float)
        return F

    def enable(self) -> None:
        pass

    def read_state(self) -> RobotIO:
        if not self.first:
            for _ in range(self.substeps):
                self.d.qfrc_applied[:] = 0.0
                for k, dof in enumerate(self.dadr):
                    self.d.qfrc_applied[dof] = self._tau[k]
                # hold the (nearly massless) gripper joints closed
                for gd, gq in zip(self.grip_dadr, self.grip_qadr):
                    self.d.qfrc_applied[gd] = -2.0 * self.d.qpos[gq] - 0.05 * self.d.qvel[gd]
                # external Cartesian force at the tool
                self.d.xfrc_applied[:] = 0.0
                self.d.xfrc_applied[self.tool, :3] = self._f_ext(self.t)
                self.mj.mj_step(self.m, self.d)
                self.t += self.m.opt.timestep
        self.first = False
        q = np.array([self.d.qpos[a] for a in self.qadr])
        dq = np.array([self.d.qvel[a] for a in self.dadr])
        return RobotIO(q=q, dq=dq, current_A=self._tau / self.kt)

    def send_torque(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=float).reshape(4)

    def disable(self) -> None:
        self._tau = np.zeros(4)

    # ground-truth EE for validation
    def true_ee(self) -> np.ndarray:
        return self.d.xpos[self.tool] + self.d.xmat[self.tool].reshape(3, 3) @ self.d4e
