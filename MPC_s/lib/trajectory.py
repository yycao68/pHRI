"""Cartesian reference-trajectory generator: hold, circle, or step.

A trajectory answers two questions at any time `now`:
  sample(now)         -> (position, velocity, acceleration) in task space [m]
  sample_posture(now) -> (q_nom, dq_nom) for the redundant-DOF posture term,
                          or (None, None) if it has no opinion (the caller
                          then falls back to a fixed nominal posture from
                          config -- see run_hardware.py).
"""
from __future__ import annotations

import numpy as np


def _quintic(tau: np.ndarray) -> np.ndarray:
    """Minimum-jerk scalar profile s(tau) in [0,1] for tau in [0,1]: zero
    velocity AND acceleration at both endpoints (unlike a raised-cosine or
    linear ramp), so a step move starts and ends with no commanded jerk."""
    tau = np.clip(tau, 0.0, 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def _quintic_d(tau: np.ndarray) -> np.ndarray:
    tau = np.clip(tau, 0.0, 1.0)
    return 30 * tau**2 - 60 * tau**3 + 30 * tau**4


def _quintic_dd(tau: np.ndarray) -> np.ndarray:
    tau = np.clip(tau, 0.0, 1.0)
    return 60 * tau - 180 * tau**2 + 120 * tau**3


class CartesianTrajectory:
    def __init__(self, cfg: dict, origin: np.ndarray, start_time: float):
        self.cfg = cfg
        self.origin = np.asarray(origin, dtype=float).reshape(3)
        self.start_time = float(start_time)

    def sample(self, now: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = max(0.0, float(now) - self.start_time)
        typ = self.cfg.get("type", "hold")
        ramp = float(self.cfg.get("ramp_s", 2.0))
        a = min(1.0, t / max(ramp, 1e-6))  # linear 0->1 blend over the first `ramp_s`
        # seconds, so the controller doesn't have to react to the full reference
        # instantaneously at t=0 -- see docs/01_concepts.md.
        if typ == "hold":
            return self.origin.copy(), np.zeros(3), np.zeros(3)
        if typ == "step":
            return self._sample_step(t)
        if typ != "circle":
            raise ValueError(f"unknown trajectory type: {typ!r} (expected 'hold', 'circle', or 'step')")
        radius = float(self.cfg.get("circle_radius_m", 0.04))
        period = float(self.cfg.get("circle_period_s", 12.0))
        plane = str(self.cfg.get("circle_plane", "xz")).lower()
        w = 2.0 * np.pi / max(period, 1e-6)
        s, c = np.sin(w * t), np.cos(w * t)
        pos = np.zeros(3); vel = np.zeros(3); acc = np.zeros(3)
        axes = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}.get(plane)
        if axes is None:
            raise ValueError(f"unsupported circle_plane: {plane!r}")
        i, j = axes
        pos[i] = radius * (c - 1.0); pos[j] = radius * s
        vel[i] = -radius * w * s; vel[j] = radius * w * c
        acc[i] = -radius * w * w * c; acc[j] = -radius * w * w * s
        return self.origin + a * pos, a * vel, a * acc

    def _sample_step(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Repeated step along one axis: an initial hold, then `step_cycles`
        repeats of (quintic move out, dwell, quintic move back, dwell) --
        matching MPC_s_Hardware_results_2026-09-29/configs/step/step.yaml's
        own comment ("5s hold, then 4 cycles of a 20mm move along x (1.5s
        quintic) and back, 5s dwell after each move"). `step_initial_hold_s`
        is not a key that config actually set -- it defaults to
        `step_dwell_s` (reusing that value) rather than inventing a separate
        constant, since the data is consistent with that and no config file
        seen so far sets it explicitly."""
        axis = str(self.cfg.get("step_axis", "x")).lower()
        axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis)
        if axis_idx is None:
            raise ValueError(f"unsupported step_axis: {axis!r}")
        amp = float(self.cfg.get("step_amplitude_m", 0.02))
        move_t = float(self.cfg.get("step_move_time_s", 1.5))
        dwell_t = float(self.cfg.get("step_dwell_s", 5.0))
        cycles = int(self.cfg.get("step_cycles", 4))
        hold_t = float(self.cfg.get("step_initial_hold_s", dwell_t))

        disp = vel1 = acc1 = 0.0
        tt = t - hold_t
        cycle_t = 2.0 * move_t + 2.0 * dwell_t
        if tt > 0.0 and cycle_t > 0.0:
            cyc_idx = int(tt // cycle_t)
            if cyc_idx < cycles:  # after all cycles, back at the origin (disp=0) and done
                tp = tt - cyc_idx * cycle_t
                if tp < move_t:                              # moving out
                    tau = tp / max(move_t, 1e-9)
                    disp = amp * _quintic(tau)
                    vel1 = amp * _quintic_d(tau) / max(move_t, 1e-9)
                    acc1 = amp * _quintic_dd(tau) / max(move_t, 1e-9) ** 2
                elif tp < move_t + dwell_t:                   # dwelling at +amp
                    disp = amp
                elif tp < 2.0 * move_t + dwell_t:             # moving back
                    tau = (tp - move_t - dwell_t) / max(move_t, 1e-9)
                    disp = amp * (1.0 - _quintic(tau))
                    vel1 = -amp * _quintic_d(tau) / max(move_t, 1e-9)
                    acc1 = -amp * _quintic_dd(tau) / max(move_t, 1e-9) ** 2
                # else: dwelling at the origin (disp=0) for the rest of this cycle

        pos = self.origin.copy(); vel = np.zeros(3); acc = np.zeros(3)
        pos[axis_idx] += disp
        vel[axis_idx] = vel1
        acc[axis_idx] = acc1
        return pos, vel, acc

    def sample_posture(self, now: float) -> tuple[None, None]:
        return None, None
