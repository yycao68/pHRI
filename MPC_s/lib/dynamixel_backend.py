"""Backends for the 3-DOF (waist-removed) OpenManipulator-X, torque control only.

- DynamixelCurrentBackend: real hardware via the DYNAMIXEL SDK in **Current
  Control Mode** (Operating_Mode=0). Reads present position/velocity/current
  with one GroupSyncRead and writes Goal_Current with one GroupSyncWrite per
  tick.
- SimArmBackend: an N-DOF joint-space plant (RNEA inertia + gravity + damping)
  with an injectable external Cartesian force (push/payload), for validating
  the whole loop off-hardware before connecting servos. N is taken from the
  kinematics/dynamics model (3 for the waist-removed arm), not hardcoded.

Interface: read_state() -> RobotIO(q, dq, current_A); enable(); disable();
send_torque(tau_Nm) -- see docs/03_hardware_safety.md for what each of the
guards below protects against and why they exist.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

# XM430-W350 / Protocol 2.0 control-table (address, length) and unit scales.
ADDR = {
    "return_delay_time": (9, 1),   # EEPROM, x2us; factory default 250 (=500us) per servo
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


def usb_latency_timer_ms(port: str) -> int | None:
    """Best-effort read of the USB-serial (FTDI, e.g. U2D2) adapter's latency
    timer for `port`, in ms; None if it can't be determined (non-FTDI adapter,
    macOS, permissions). The driver holds back a partially-filled receive
    buffer for up to this long before handing it to the host, so every
    GroupSyncRead reply can wait up to this extra time. Factory default is
    16 ms on both Windows and Linux; 1 ms is the usual setting for control
    loops. This is an OS/driver setting, not a servo register:
      Linux:   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
      Windows: Device Manager -> Ports -> USB Serial Port (COMx) -> Properties ->
               Port Settings -> Advanced -> Latency Timer (msec) = 1, then replug.
    """
    import os
    import sys
    try:
        if sys.platform.startswith("linux"):
            path = f"/sys/bus/usb-serial/devices/{os.path.basename(port)}/latency_timer"
            with open(path) as f:
                return int(f.read().strip())
        if sys.platform == "win32":
            import winreg
            root = r"SYSTEM\CurrentControlSet\Enum\FTDIBUS"
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, root) as k_bus:
                for a in range(winreg.QueryInfoKey(k_bus)[0]):
                    dev = winreg.EnumKey(k_bus, a)
                    with winreg.OpenKey(k_bus, dev) as k_dev:
                        for b in range(winreg.QueryInfoKey(k_dev)[0]):
                            inst = winreg.EnumKey(k_dev, b)
                            try:
                                with winreg.OpenKey(k_dev, inst + r"\Device Parameters") as k_par:
                                    name = winreg.QueryValueEx(k_par, "PortName")[0]
                                    if str(name).upper() == port.upper():
                                        return int(winreg.QueryValueEx(k_par, "LatencyTimer")[0])
                            except OSError:
                                continue
    except (OSError, ValueError):
        return None
    return None


def assert_startup_pose_plausible(q: np.ndarray, jmin: np.ndarray, jmax: np.ndarray,
                                   tol_rad: float = 0.5) -> None:
    """Refuse to energize when the measured pose is not physically reachable.

    Why this exists: the servo's Present_Position register is a MULTI-TURN
    signed count, and read_state() converts it with
    q = sign*raw*POS_PER_TICK - offset. If the turn counter is off by one --
    which can happen after a power cycle, or after the arm is moved by hand
    with torque disabled -- every reading for that joint is shifted by
    exactly 2*pi. Forward kinematics alone cannot catch this: FK is
    2*pi-periodic in every revolute joint, so a full-turn-wrapped reading
    produces the exact same end-effector position as the true one.

    A joint-space range test catches it instantly: the true pose is inside
    [jmin, jmax] by construction, and a wrap is 2*pi = 6.28 rad away from it,
    so any reasonable tolerance separates the two cleanly. tol_rad = 0.5 is a
    generous allowance for a pose set by hand slightly past a software
    limit, and is still far smaller than the 2*pi wrap this is meant to catch.

    Deliberately NOT bypassable by --force-start: that flag means "yes, the
    arm really is sitting somewhere unusual, proceed" -- a statement about a
    pose that exists. A reading outside the joint range describes a pose the
    arm cannot physically be in, so there is nothing to consent to."""
    q = np.asarray(q, dtype=float)
    jmin = np.asarray(jmin, dtype=float)
    jmax = np.asarray(jmax, dtype=float)
    bad = np.where((q < jmin - tol_rad) | (q > jmax + tol_rad))[0]
    if bad.size == 0:
        return
    lines = []
    for k in bad:
        note = ""
        for turns in (-2, -1, 1, 2):
            q_try = q[k] + turns * 2.0 * np.pi
            if jmin[k] - tol_rad <= q_try <= jmax[k] + tol_rad:
                note = (f"  <-- {q_try:+.4f} rad after {turns:+d} turn(s), which IS in range: "
                        f"this is a full-turn wrap of the multi-turn position register, "
                        f"not a real pose")
                break
        lines.append(f"    joint {k}: q = {q[k]:+.4f} rad, allowed "
                     f"[{jmin[k]:+.4f}, {jmax[k]:+.4f}] +/- {tol_rad}{note}")
    raise RuntimeError(
        "measured startup pose is outside joint_min_rad/joint_max_rad, i.e. the arm cannot "
        "physically be where the servos say it is:\n" + "\n".join(lines) +
        "\n  Do NOT re-run and hope. Power-cycle the servos (12V and USB), then re-read the "
        "pose with torque off (tools/calibrate_joints.py) and confirm it is in range before "
        "energizing. --force-start does not bypass this check.")


@dataclass
class RobotIO:
    q: np.ndarray
    dq: np.ndarray
    current_A: np.ndarray


class DynamixelCurrentBackend:
    def __init__(self, config: dict, port: str, baud: int, return_delay_time: int | None = None):
        try:
            from dynamixel_sdk import (
                PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite, COMM_SUCCESS,
            )
        except ImportError as exc:
            raise RuntimeError(
                "dynamixel-sdk is required for --backend dynamixel. "
                "pip install dynamixel-sdk"
            ) from exc
        self._COMM_SUCCESS = COMM_SUCCESS

        robot = config.get("robot", {})
        self.ids = list(robot.get("servo_ids", [12, 13, 14]))
        self.n = len(self.ids)
        self.sign = np.asarray(robot.get("joint_sign", [1] * self.n), dtype=float)
        self.q_offset = np.asarray(robot.get("joint_offset_rad", [0.0] * self.n), dtype=float)
        self.kt = float(robot.get("torque_constant_Nm_per_A", 1.78))
        self.cur_limit_ticks = int(robot.get("current_limit_ticks", 300))  # conservative default

        self.port = PortHandler(port)
        self.ph = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f"failed to open {port}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"failed to set baud {baud}")
        # None = leave the servos' stored Return_Delay_Time alone (old behavior).
        self.return_delay_time = return_delay_time
        lat = usb_latency_timer_ms(port)
        if lat is None:
            print(f"[dynamixel] USB latency timer for {port}: unknown (not an FTDI adapter, or "
                  f"not readable on this OS) -- see usb_latency_timer_ms() for how to check/set it.")
        elif lat > 1:
            print(f"[dynamixel] WARNING: USB latency timer for {port} is {lat} ms (1 ms recommended) "
                  f"-- every read_state() can wait up to that long for the adapter to flush. "
                  f"See usb_latency_timer_ms() in lib/dynamixel_backend.py for how to lower it.")
        else:
            print(f"[dynamixel] USB latency timer for {port}: {lat} ms (ok)")

        a, ln = ADDR["present_current"][0], 10  # 126..135
        self.reader = GroupSyncRead(self.port, self.ph, a, ln)
        for i in self.ids:
            self.reader.addParam(i)
        self.writer = GroupSyncWrite(self.port, self.ph, *ADDR["goal_current"])
        self._enabled = False
        self._stale_counts: dict = {}
        self._consecutive_stale: dict = {i: 0 for i in self.ids}
        self._read_fail_count = 0
        self._last_warn_time = 0.0
        self.max_consecutive_stale = int(robot.get("max_consecutive_stale_reads", 20))
        # isAvailable() only means "new bytes arrived for this address range" -- it does NOT
        # mean the bytes decode to a sane value. A corrupted/all-zero packet that still passes
        # that freshness check will decode to q = -joint_offset_rad for every joint (raw=0 ticks).
        # At the servo's rated max speed, one 100 Hz control period can move a joint at most
        # ~0.05 rad, so any single-step jump well beyond that is physically impossible and gets
        # rejected below (last-known-good state is returned instead of the glitched one).
        self.max_step_rad = float(robot.get("max_plausible_step_rad", 0.3))
        self._last_good: RobotIO | None = None
        self._consecutive_jump = 0
        self.max_consecutive_jump = int(robot.get("max_consecutive_jump_reads", 20))

    def _w1(self, i, addr, val):
        self.ph.write1ByteTxRx(self.port, i, addr, val)

    def _w2(self, i, addr, val):
        self.ph.write2ByteTxRx(self.port, i, addr, val & 0xFFFF)

    def enable(self) -> None:
        for i in self.ids:
            self._w1(i, ADDR["torque_enable"][0], 0)            # off to write EEPROM
            if self.return_delay_time is not None:
                # EEPROM: persists across power cycles. Each servo waits this long (x2us)
                # before replying, and GroupSyncRead visits all servos in series on one
                # half-duplex bus, so the waits add up every tick.
                self._w1(i, ADDR["return_delay_time"][0], int(self.return_delay_time))
                rdt, cr, _ = self.ph.read1ByteTxRx(self.port, i, ADDR["return_delay_time"][0])
                if cr != self._COMM_SUCCESS or int(rdt) != int(self.return_delay_time):
                    raise RuntimeError(f"servo {i}: Return_Delay_Time write did not stick "
                                       f"(read back {rdt}, wanted {self.return_delay_time})")
            self._w1(i, ADDR["operating_mode"][0], CURRENT_MODE)
            self._w2(i, ADDR["current_limit"][0], self.cur_limit_ticks)
            self._w1(i, ADDR["torque_enable"][0], 1)
        self._enabled = True

    def read_state(self) -> RobotIO:
        comm_result = self.reader.txRxPacket()
        q = np.zeros(self.n); dq = np.zeros(self.n); cur = np.zeros(self.n)
        base = ADDR["present_current"][0]
        stale_ids = []
        for k, i in enumerate(self.ids):
            # isAvailable() checks whether THIS servo's data in the current sync-read
            # response is actually fresh -- without this check, getData() silently
            # returns the last cached value on a communication drop for that servo,
            # which looks identical to the joint being physically stuck.
            ok_cur = self.reader.isAvailable(i, base, 2)
            ok_vel = self.reader.isAvailable(i, base + 2, 4)
            ok_pos = self.reader.isAvailable(i, base + 6, 4)
            if not (ok_cur and ok_vel and ok_pos):
                stale_ids.append(i)
                self._stale_counts[i] = self._stale_counts.get(i, 0) + 1
                self._consecutive_stale[i] += 1
            else:
                self._consecutive_stale[i] = 0
            cur_t = _s16(self.reader.getData(i, base, 2))
            vel_t = _s32(self.reader.getData(i, base + 2, 4))
            pos_t = _s32(self.reader.getData(i, base + 6, 4))
            q[k] = self.sign[k] * (pos_t * POS_PER_TICK) - self.q_offset[k]
            dq[k] = self.sign[k] * vel_t * VEL_PER_TICK
            cur[k] = self.sign[k] * cur_t * CUR_PER_TICK
        if comm_result != self._COMM_SUCCESS or stale_ids:
            self._read_fail_count += 1
            # throttle printing so a burst of failures doesn't flood the console
            now = time.monotonic()
            if now - self._last_warn_time > 0.5:
                print(f"\n[WARN] stale/failed read on servo(s) {stale_ids or self.ids} "
                      f"(comm_result={comm_result}); total stale reads so far per id: "
                      f"{self._stale_counts}", flush=True)
                self._last_warn_time = now
        worst = max(self._consecutive_stale.values()) if self._consecutive_stale else 0
        if worst >= self.max_consecutive_stale:
            bad = [i for i, c in self._consecutive_stale.items() if c >= self.max_consecutive_stale]
            raise RuntimeError(
                f"servo(s) {bad} had {self.max_consecutive_stale}+ consecutive stale/failed "
                f"reads -- treating this as a communication failure and stopping (torque will "
                f"be disabled by the caller's finally block). Check cabling/connectors and the "
                f"COM port; do not ignore this and keep running on stale position data."
            )

        io = RobotIO(q=q, dq=dq, current_A=cur)
        if self._last_good is not None:
            jump = np.abs(q - self._last_good.q)
            if np.any(jump > self.max_step_rad):
                self._consecutive_jump += 1
                now = time.monotonic()
                if now - self._last_warn_time > 0.5:
                    print(f"\n[WARN] rejected implausible single-step joint jump "
                          f"{np.round(jump, 4)} rad (limit {self.max_step_rad}); reusing last "
                          f"known-good state instead of a likely corrupted read.", flush=True)
                    self._last_warn_time = now
                if self._consecutive_jump >= self.max_consecutive_jump:
                    raise RuntimeError(
                        f"{self.max_consecutive_jump}+ consecutive implausible joint-position "
                        f"jumps (>{self.max_step_rad} rad/step) -- treating this as a sustained "
                        f"communication failure and stopping (torque will be disabled by the "
                        f"caller's finally block). Check cabling/connectors and the COM port."
                    )
                return self._last_good
        self._consecutive_jump = 0
        self._last_good = io
        return io

    def resync(self) -> None:
        """Call this right after any intentional gap in the control loop
        during which the arm's true position may legitimately have moved by
        more than max_plausible_step_rad. Without this, read_state()'s
        implausible-jump guard compares the next (real, valid) reading
        against the stale pre-gap _last_good, rejects it, and -- since a
        rejected read never updates _last_good -- keeps rejecting every
        subsequent read forever, eventually raising a fatal RuntimeError
        even though nothing was actually wrong with communication."""
        self._last_good = None
        self._consecutive_jump = 0

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
        if self._read_fail_count:
            print(f"[Info] {self._read_fail_count} read cycles had a stale/failed servo "
                  f"read during this run; per-id counts: {self._stale_counts}")


class SimArmBackend:
    """Lightweight (numpy-only) N-DOF plant: M(q) ddq = tau - g(q) - b dq + J^T F_ext.

    N is inferred from the kinematics object (kin.n), so this works
    unmodified for any arm model with the same interface. F_ext is the
    optional injected push/payload disturbance (disturbance.push/
    disturbance.payload in config) -- read directly by this class since it
    only ever matters in sim (a real push/payload has to be applied by hand).
    """

    def __init__(self, config: dict, kin):
        from dynamics import OpenManipulatorDynamics
        robot = config.get("robot", {})
        self.kin = kin
        self.dyn = OpenManipulatorDynamics()
        self.n = getattr(kin, "n", 3)
        self.dt = float(config.get("controller", {}).get("dt", 0.01))
        self.sub = max(1, int(round(self.dt / 0.001)))   # 1 ms integration substeps
        self.sdt = self.dt / self.sub
        self.b = np.asarray(robot.get("sim_joint_damping", [0.02] * self.n), dtype=float)
        self.q = np.asarray(robot.get("sim_home_q_rad", [0.3] * self.n), dtype=float)
        self.dq = np.zeros(self.n)
        self.kt = float(robot.get("torque_constant_Nm_per_A", 1.78))
        self.t = 0.0
        self.first = True
        self._tau = np.zeros(self.n)
        # Reflected rotor/gearbox inertia not present in dyn.mass_matrix(q) (real link
        # inertia only) -- see docs/01_concepts.md for how this was identified from real
        # hardware data. Default 0.0: byte-for-byte unchanged unless a config opts in.
        self.dyn_armature = float(robot.get("dyn_armature_kg_m2", 0.0))
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
                # cosine-ramped pulse: rises smoothly to full amplitude and back down over
                # [t0, t1], rather than a hard step -- easier to attribute what happens to
                # the disturbance vs. an actuator/measurement transient.
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
                Mq = self.dyn.mass_matrix(self.q) + self.dyn_armature * np.eye(self.n)
                tau_ext = self.kin.jacobian(self.q).T @ self._f_ext(self.t)
                ddq = np.linalg.solve(Mq, self._tau - g - self.b * self.dq + tau_ext)
                self.dq = self.dq + self.sdt * ddq
                self.q = self.q + self.sdt * self.dq
                self.t += self.sdt
        self.first = False
        return RobotIO(q=self.q.copy(), dq=self.dq.copy(), current_A=self._tau / self.kt)

    def send_torque(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=float).reshape(self.n)

    def reset_time(self) -> None:
        """Zero the internal sim clock that disturbance.push/payload's
        t_start/t_end are measured against. Needed because
        lib/move_to_start.py's own read_state() calls (before the real
        trajectory-tracking loop begins) already advance this clock --
        without resetting it, a push/payload scheduled for e.g. t_start=4.0
        fires 4s into move_to_start's own settling phase instead of 4s into
        the real tracking run."""
        self.t = 0.0

    def disable(self) -> None:
        self._tau = np.zeros(self.n)


def create_backend(kind: str, config: dict, kin, args) -> object:
    if kind == "sim":
        return SimArmBackend(config, kin)
    if kind == "dynamixel":
        if not getattr(args, "port", None):
            raise ValueError("--port is required for --backend dynamixel")
        return DynamixelCurrentBackend(config, port=args.port, baud=int(args.baud),
                                       return_delay_time=getattr(args, "return_delay_time", None))
    raise ValueError(f"unknown backend: {kind!r} (expected 'sim' or 'dynamixel')")
