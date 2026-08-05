import numpy as np

from harmonic_disturbance_predictor import HarmonicDisturbancePredictor

FREQUENCIES_HZ = np.array([0.9, 1.4, 1.9])
TRUE_A = np.array([1.1, 0.6, 0.9])
TRUE_B = np.array([0.3, -0.2, 0.5])
DT = 0.01  # 100 Hz manager tick, matching the benchmark's own manager_dt


def true_signal(t: float) -> np.ndarray:
    omega = 2.0 * np.pi * FREQUENCIES_HZ
    return TRUE_A * np.sin(omega * t) + TRUE_B * np.cos(omega * t)


def test_predictor_converges_to_true_amplitude_and_phase():
    rng = np.random.default_rng(0)
    predictor = HarmonicDisturbancePredictor(frequencies_hz=FREQUENCIES_HZ, measurement_noise=0.02)
    for k in range(400):  # 4 s of ticks, several periods of the slowest (0.9 Hz) axis
        t = k * DT
        measurement = true_signal(t) + rng.normal(0.0, 0.02, 3)
        predictor.update(t, measurement)

    true_amplitude = np.hypot(TRUE_A, TRUE_B)
    true_phase = np.arctan2(TRUE_A, TRUE_B)
    fitted = predictor.current_estimate
    assert np.allclose(fitted[:, 0], true_amplitude, atol=0.05)
    assert np.allclose(fitted[:, 1], true_phase, atol=0.05)


def test_forecast_beats_frozen_hold_on_a_known_periodic_signal():
    rng = np.random.default_rng(1)
    predictor = HarmonicDisturbancePredictor(frequencies_hz=FREQUENCIES_HZ, measurement_noise=0.02)
    t = 0.0
    last_measurement = np.zeros(3)
    for k in range(400):
        t = k * DT
        last_measurement = true_signal(t) + rng.normal(0.0, 0.02, 3)
        predictor.update(t, last_measurement)

    horizon = 20
    horizon_times = t + DT * np.arange(1, horizon + 1)
    forecast = predictor.forecast(horizon_times)
    frozen_hold = np.tile(last_measurement, (horizon, 1))
    truth = np.stack([true_signal(ht) for ht in horizon_times])

    forecast_rmse = np.sqrt(np.mean((forecast - truth) ** 2))
    frozen_rmse = np.sqrt(np.mean((frozen_hold - truth) ** 2))
    assert forecast_rmse < frozen_rmse
