"""Cartesian reference-trajectory generator: hold or circle.

A trajectory answers two questions at any time `now`:
  sample(now)         -> (position, velocity, acceleration) in task space [m]
  sample_posture(now) -> (q_nom, dq_nom) for the redundant-DOF posture term,
                          or (None, None) if it has no opinion (the caller
                          then falls back to a fixed nominal posture from
                          config -- see run_hardware.py).
"""
from __future__ import annotations

import numpy as np


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
        if typ != "circle":
            raise ValueError(f"unknown trajectory type: {typ!r} (expected 'hold' or 'circle')")
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

    def sample_posture(self, now: float) -> tuple[None, None]:
        return None, None
