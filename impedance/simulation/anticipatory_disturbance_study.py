#!/usr/bin/env python3
"""Anticipatory (harmonic-forecast) vs frozen-hold disturbance rejection.

Compares the `two_rate` controller (frozen disturbance estimate, tiled
across the MPC horizon) against `anticipatory` (same tank/torque
authorization, but the raw MPC proposal is built from a per-axis
known-frequency Kalman forecast of the rejectable disturbance -- see
harmonic_disturbance_predictor.py).

Three dedicated, wall-disabled scenarios isolate the comparison from
contact-force confounds (the harmonic model has no way to represent
non-periodic contact transients, and should not be asked to):

- pure_harmonic_ablation: the three known-frequency sinusoids only, the
  12 N pulse suppressed (pulse_scale=0) -- zero non-harmonic content, the
  cleanest case the forecaster could ask for, and the control for the
  other two scenarios below.
- periodic_only: all three known-frequency sinusoids plus the pulse active
  -- the scenario used throughout the paper; a small amount of
  non-harmonic content (the pulse) alongside the harmonic content.
- pulse_only: sinusoids silenced (disturbance_scale=0), pulse alone active
  -- entirely non-harmonic content, the negative-control extreme.

A fourth, non-headline check reruns the full matched benchmark (wall
active, everything on) to see whether any periodic-disturbance advantage
survives once contact-force content dominates the disturbance estimate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import stats

from verify_fr3_two_rate_benchmark import Config, run_trial

HERE = Path(__file__).resolve().parent


def _paired_comparison(cfg: Config, seeds: int, **trial_kwargs) -> dict:
    rows = {"two_rate": [], "anticipatory": []}
    for name in rows:
        for seed in range(seeds):
            result = run_trial(name, cfg, seed, **trial_kwargs)
            rows[name].append(result["metrics"])

    two_rate_rms = np.array([r["residual_rms_mm"] for r in rows["two_rate"]])
    anticipatory_rms = np.array([r["residual_rms_mm"] for r in rows["anticipatory"]])
    diff = anticipatory_rms - two_rate_rms
    half = stats.t.ppf(0.975, seeds - 1) * stats.sem(diff) if seeds > 1 else 0.0
    p_value = float(stats.ttest_rel(anticipatory_rms, two_rate_rms).pvalue) if seeds > 1 else 1.0

    def summarize(name: str) -> dict:
        vals = rows[name]
        return {
            "residual_rms_mm_mean": float(np.mean([v["residual_rms_mm"] for v in vals])),
            "residual_rms_mm_std": float(np.std([v["residual_rms_mm"] for v in vals], ddof=1)) if seeds > 1 else 0.0,
            "minimum_tank_j_min": float(np.min([v["minimum_tank_j"] for v in vals])),
            "maximum_torque_ratio": float(np.max([v["maximum_torque_ratio"] for v in vals])),
            "qp_failures": int(np.sum([v["qp_failures"] for v in vals])),
            "nominal_infeasible_samples": int(np.sum([v["nominal_infeasible_samples"] for v in vals])),
        }

    return {
        "two_rate": summarize("two_rate"),
        "anticipatory": summarize("anticipatory"),
        "paired_difference_mm": float(diff.mean()),
        "paired_95_percent_ci_mm": [float(diff.mean() - half), float(diff.mean() + half)],
        "paired_t_p_value": p_value,
        "relative_change_percent": float(100 * diff.mean() / two_rate_rms.mean()),
    }


def pure_harmonic_ablation(cfg: Config, seeds: int) -> dict:
    # pulse_scale=0 removes the only non-harmonic content from periodic_only,
    # leaving a disturbance the two-parameter (amplitude/phase) harmonic
    # model can represent exactly.
    return _paired_comparison(cfg, seeds, wall_stiffness=0.0, wall_damping=0.0, pulse_scale=0.0)


def periodic_only_comparison(cfg: Config, seeds: int) -> dict:
    return _paired_comparison(cfg, seeds, wall_stiffness=0.0, wall_damping=0.0)


def pulse_only_comparison(cfg: Config, seeds: int) -> dict:
    # disturbance_scale=0 silences the sinusoids, so the per-seed random
    # phases no longer reach the trial; without another noise source every
    # seed would replay the identical pulse-only trajectory. 0.05 N estimator
    # noise (the same level used by the leakage/sensing-realism sweeps) gives
    # the 20 seeds genuine, independent variation.
    return _paired_comparison(cfg, seeds, wall_stiffness=0.0, wall_damping=0.0,
                               disturbance_scale=0.0, pulse_scale=1.0, sensor_noise=0.05)


def full_matched_robustness_check(cfg: Config, seeds: int) -> dict:
    return _paired_comparison(cfg, seeds)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--output", type=Path, default=HERE / "anticipatory_disturbance_results.json")
    args = parser.parse_args()
    results = {
        "pure_harmonic_ablation": pure_harmonic_ablation(Config(), args.seeds),
        "periodic_only": periodic_only_comparison(Config(), args.seeds),
        "pulse_only": pulse_only_comparison(Config(), args.seeds),
        "full_matched_robustness_check": full_matched_robustness_check(Config(), args.seeds),
    }
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
