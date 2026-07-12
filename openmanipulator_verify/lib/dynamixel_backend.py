"""Backends for OpenManipulator-X torque-level pHRI verification.

- DynamixelCurrentBackend: real hardware via the DYNAMIXEL SDK in **Current
  Control Mode** (Operating_Mode=0). Reads present position/velocity/current with
  one GroupSyncRead and writes Goal_Current with one GroupSyncWrite per tick.
  This is the genuine torque interface the pHRI paper needs -- nothing here goes
  through LeRobot; it is the ROBOTIS Protocol 2.0 stack.
- SimArmBackend: a 4-DOF joint-space plant (diagonal inertia + gravity + damping)
  with an injectable external Cartesian force (push/payload), for validating the
  whole loop off-hardware. It exposes offset-free recovery because -- unlike a
  position-controlled arm -- the plant is genuinely torque-driven.

Interface (both): read_state() -> (q, dq, current_A); send_torque(tau_Nm);
enable(); disable().
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# XM430-W350 / Protocol 2.0 control-table (address, length) and unit scales.
ADDR = {
    "operating_mode": (11, 1),
    "current_limit": (38, 2),
    "torque_enable": (64, 1),
    "goal_current": (102, 2),
    "present_current": (126, 2),   # contiguous 126..135:
    "present_velocity": (128, 4),  #   current(2) + velocity(4) + position(4)
    "present_position": (132, 4),
}
CURRENT_MODE = 0
POS_PER_TICK = 2.0 * np.pi / 4096.0          # rad/tick
VEL_PER_TICK = 0.229 * 2.0 * np.pi / 60.0    # (rev/min) -> rad/s
CUR_PER_TICK = 0.00269                        # A/tick (2.69 mA)


def _s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def _s32(v: int) -> int:
    return v - 4294967296 if v >= 2147483648 else v


@dataclass
class RobotIO:
    q: np.ndarray
    dq: np.ndarray
    current_A: np.ndarray


class DynamixelCurrentBackend:
    def __init__(self, config: dict, port: str, baud: int):
        try:
            from dynamixel_sdk import (
                PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite,
            )
        except ImportError as exc:
            raise RuntimeError(
                "dynamixel-sdk is required for --backend dynamixel. "
                "pip install dynamixel-sdk"
            ) from exc

        robot = config.get("robot", {})
        self.ids = list(robot.get("servo_ids", [11, 12, 13, 14]))
        self.n = len(self.ids)
        self.sign = np.asarray(robot.get("joint_sign", [1] * self.n), dtype=float)
        self.q_offset = np.asarray(robot.get("joint_offset_rad", [0.0] * self.n), dtype=float)
        self.kt = float(robot.get("torque_constant_Nm_per_A", 1.78))
        self.cur_limit_ticks = int(robot.get("current_limit_ticks", 400))  # ~1.08 A default (conservative)

        self.port = PortHandler(port)
        self.ph = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f"failed to open {port}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"failed to set baud {baud}")

        a, ln = ADDR["present_current"][0], 10  # 126..135
        self.reader = GroupSyncRead(self.port, self.ph, a, ln)
        for i in self.ids:
            self.reader.addParam(i)
        self.writer = GroupSyncWrite(self.port, self.ph, *ADDR["goal_current"])
        self._enabled = False

    def _w1(self, i, addr, val):
        self.ph.write1ByteTxRx(self.port, i, addr, val)

    def _w2(self, i, addr, val):
        self.ph.write2ByteTxRx(self.port, i, addr, val & 0xFFFF)

    def enable(self) -> None:
        for i in self.ids:
            self._w1(i, ADDR["torque_enable"][0], 0)            # off to write EEPROM
            self._w1(i, ADDR["operating_mode"][0], CURRENT_MODE)
            self._w2(i, ADDR["current_limit"][0], self.cur_limit_ticks)
            self._w1(i, ADDR["torque_enable"][0], 1)
        self._enabled = True

    def read_state(self) -> RobotIO:
        self.reader.txRxPacket()
        q = np.zeros(self.n); dq = np.zeros(self.n); cur = np.zeros(self.n)
        base = ADDR["present_current"][0]
        for k, i in enumerate(self.ids):
            cur_t = _s16(self.reader.getData(i, base, 2))
            vel_t = _s32(self.reader.getData(i, base + 2, 4))
            pos_t = _s32(self.reader.getData(i, base + 6, 4))
            q[k] = self.sign[k] * (pos_t * POS_PER_TICK) - self.q_offset[k]
            dq[k] = self.sign[k] * vel_t * VEL_PER_TICK
            cur[k] = self.sign[k] * cur_t * CUR_PER_TICK
        return RobotIO(q=q, dq=dq, current_A=cur)

    def send_torque(self, tau: np.ndarray) -> None:
        tau = np.asarray(tau, dtype=float).reshape(self.n)
        ticks = np.round(self.sign * tau / self.kt / CUR_PER_TICK).astype(int)
        ticks = np.clip(ticks, -self.cur_limit_ticks, self.cur_limit_ticks)
        self.writer.clearParam()
        for k, i in enumerate(self.ids):
            v = int(ticks[k]) & 0xFFFF
            self.writer.addParam(i, [v & 0xFF, (v >> 8) & 0xFF])
        self.writer.txPacket()

    def disable(self) -> None:
        try:
            for i in self.ids:
                self._w2(i, ADDR["goal_current"][0], 0)
                self._w1(i, ADDR["torque_enable"][0], 0)
        finally:
            self.port.closePort()
        self._enabled = False


class SimArmBackend:
    """Lightweight (numpy-only) 4-DOF plant: M(q) ddq = tau - g(q) - b dq + J^T F_ext.

    Uses the validated rigid-body dynamics (dynamics.py) so it is consistent with
    the controller -- good for a fast smoke test, but for a *rigorous*,
    independent check use the MuJoCo backend (--backend mjc), whose dynamics are
    not shared with the controller.
    """

    def __init__(self, config: dict, kin):
        from dynamics import OpenManipulatorDynamics
        robot = config.get("robot", {})
        self.kin = kin
        self.dyn = OpenManipulatorDynamics()
        self.n = 4
        self.dt = float(config.get("controller", {}).get("dt", 0.01))
        self.sub = max(1, int(round(self.dt / 0.001)))   # 1 ms integration substeps
        self.sdt = self.dt / self.sub
        self.b = np.asarray(robot.get("sim_joint_damping", [0.02, 0.03, 0.02, 0.015]), dtype=float)
        self.q = np.asarray(robot.get("sim_home_q_rad", [0.0, -0.6, 0.3, 0.3]), dtype=float)
        self.dq = np.zeros(self.n)
        self.kt = float(robot.get("torque_constant_Nm_per_A", 1.78))
        self.t = 0.0
        self.first = True
        self._tau = np.zeros(self.n)
        d = config.get("disturbance", {})
        self.pushes = d.get("push", []) or []
        if isinstance(self.pushes, dict):
            self.pushes = [self.pushes]
        self.payloads = d.get("payload", []) or []
        if isinstance(self.payloads, dict):
            self.payloads = [self.payloads]

    def _f_ext(self, t: float) -> np.ndarray:
        F = np.zeros(3)
        for p in self.pushes:
            t0, t1 = float(p.get("t_start", 0.0)), float(p.get("t_end", 1.0))
            amp = np.asarray(p.get("force_N", [0.0, 0.0, 8.0]), dtype=float)
            if t0 <= t < t1 and t1 > t0:
                F = F + amp * 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / (t1 - t0)))
        for q in self.payloads:
            if t >= float(q.get("t_start", 0.0)):
                F = F + np.asarray(q.get("force_N", [0.0, 0.0, -3.0]), dtype=float)
        return F

    def enable(self) -> None:
        pass

    def read_state(self) -> RobotIO:
        if not self.first:
            for _ in range(self.sub):
                g = self.dyn.gravity(self.q)
                Mq = self.dyn.mass_matrix(self.q)
                tau_ext = self.kin.jacobian(self.q).T @ self._f_ext(self.t)
                ddq = np.linalg.solve(Mq, self._tau - g - self.b * self.dq + tau_ext)
                self.dq = self.dq + self.sdt * ddq
                self.q = self.q + self.sdt * self.dq
                self.t += self.sdt
        self.first = False
        return RobotIO(q=self.q.copy(), dq=self.dq.copy(), current_A=self._tau / self.kt)

    def send_torque(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=float).reshape(self.n)

    def disable(self) -> None:
        self._tau = np.zeros(self.n)


def create_backend(kind: str, config: dict, kin, args) -> object:
    if kind == "sim":
        return SimArmBackend(config, kin)
    if kind == "mjc":
        from mujoco_sim import MujocoArmBackend
        return MujocoArmBackend(config, kin)
    if kind == "dynamixel":
        if not getattr(args, "port", None):
            raise ValueError("--port is required for --backend dynamixel")
        return DynamixelCurrentBackend(config, port=args.port, baud=int(args.baud))
    raise ValueError(f"unknown backend: {kind}")
