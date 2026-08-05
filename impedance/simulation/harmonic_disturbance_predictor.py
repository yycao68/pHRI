"""Anticipatory rejectable-disturbance predictor for the two-rate residual MPC.

The existing residual MPC (`ResidualMPC3D` in verify_fr3_two_rate_benchmark.py)
holds its disturbance estimate constant across the whole prediction horizon
(`dseq = np.tile(disturbance_hat, cfg.horizon)`). This module instead tracks
and forecasts a *structured* disturbance -- one task-axis, one known
frequency each -- forward through the horizon, replacing the frozen hold with
a genuine prediction for that structured component.

Each task axis j is modeled as a single sinusoid of known angular frequency
omega_j and unknown, slowly-varying amplitude/phase, written in sine/cosine
form d_j(t) = a_j*sin(omega_j*t) + b_j*cos(omega_j*t). Because sin(omega_j*t)
and cos(omega_j*t) are known functions of time, the measurement model is
linear in the state [a_j, b_j] at every instant, so a small scalar Kalman
filter per axis suffices -- no discretized continuous-time exosystem is
needed for this fixed-frequency case.

Assuming the frequencies are known is a disclosed modeling choice, matched to
this benchmark's own simulator-known disturbance frequencies; it is not a
general online frequency-identification method. A genuinely non-periodic
component (e.g. contact transients) will not be captured by this model, by
construction -- that is the honest limitation the accompanying study is
designed to expose, not hide.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class HarmonicDisturbancePredictor:
    frequencies_hz: np.ndarray  # shape (3,), one known frequency per task axis
    process_noise: float = 1e-4
    measurement_noise: float = 0.05
    initial_variance: float = 1.0

    _omega: np.ndarray = field(init=False)
    _state: np.ndarray = field(init=False)  # shape (3, 2): [a_j, b_j] per axis
    _cov: np.ndarray = field(init=False)  # shape (3, 2, 2): P_j per axis

    def __post_init__(self) -> None:
        self._omega = 2.0 * np.pi * np.asarray(self.frequencies_hz, dtype=float)
        self._state = np.zeros((3, 2))
        self._cov = np.stack([np.eye(2) * self.initial_variance for _ in range(3)])

    def update(self, t: float, measurement: np.ndarray) -> None:
        """One scalar Kalman update per axis from the measured disturbance at time t."""
        q = np.eye(2) * self.process_noise
        r = self.measurement_noise ** 2
        for j in range(3):
            state_pred = self._state[j]
            cov_pred = self._cov[j] + q

            c = np.array([np.sin(self._omega[j] * t), np.cos(self._omega[j] * t)])
            innovation = measurement[j] - c @ state_pred
            s = c @ cov_pred @ c + r
            k = (cov_pred @ c) / s

            self._state[j] = state_pred + k * innovation
            self._cov[j] = cov_pred - np.outer(k, c) @ cov_pred

    def forecast(self, horizon_times: np.ndarray) -> np.ndarray:
        """Forecast the fitted harmonic model at each of the given future times.

        Returns an array of shape (len(horizon_times), 3).
        """
        horizon_times = np.asarray(horizon_times, dtype=float)
        out = np.zeros((len(horizon_times), 3))
        for j in range(3):
            a, b = self._state[j]
            out[:, j] = a * np.sin(self._omega[j] * horizon_times) + b * np.cos(self._omega[j] * horizon_times)
        return out

    @property
    def current_estimate(self) -> np.ndarray:
        """The fitted model evaluated at the state's own reference (a, b) pair, i.e. amplitude/phase."""
        amplitude = np.linalg.norm(self._state, axis=1)
        phase = np.arctan2(self._state[:, 0], self._state[:, 1])
        return np.stack([amplitude, phase], axis=1)
