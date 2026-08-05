#!/usr/bin/env python3
"""Horizon-length sensitivity for the anticipatory (harmonic-forecast) controller.

`anticipatory_disturbance_study.py`'s headline comparison (Table 6) uses the
paper's own 0.20 s horizon (N=20). This sweep asks whether that specific
choice drives the result: does a longer lookahead let the anticipatory
controller recover -- or even reverse -- its disadvantage against the
frozen-hold `two_rate` baseline, for periodic-only, and does the
full-matched (wall + everything on) scenario's apparent advantage survive
past N=20?

Horizons run from 0.10 s up to 1.10 s, past a full period of the slowest
(0.9 Hz) rejectable-force sinusoid. This is a smaller, exploratory seed
count (`--seeds`, default 8) reporting mean RMS only -- not the paired
95% CI / t-test that Table 6's 20-seed comparison supports -- because its
role is to characterize a trend across six horizon settings, not to certify
a single point estimate.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from verify_fr3_two_rate_benchmark import Config, run_trial

HERE = Path(__file__).resolve().parent

HORIZONS = [10, 20, 40, 60, 80, 110]


def _sweep(seeds: int, **trial_kwargs) -> list[dict]:
    rows = []
    for n in HORIZONS:
        cfg = Config(horizon=n)
        t0 = time.time()
        two_rate = np.array([run_trial("two_rate", cfg, seed, **trial_kwargs)["metrics"]["residual_rms_mm"]
                              for seed in range(seeds)])
        anticipatory = np.array([run_trial("anticipatory", cfg, seed, **trial_kwargs)["metrics"]["residual_rms_mm"]
                                  for seed in range(seeds)])
        rows.append({
            "horizon": n,
            "lookahead_s": n * cfg.manager_dt,
            "two_rate_rms_mm_mean": float(two_rate.mean()),
            "anticipatory_rms_mm_mean": float(anticipatory.mean()),
            "relative_change_percent": float(100 * (anticipatory.mean() - two_rate.mean()) / two_rate.mean()),
            "wall_clock_s": time.time() - t0,
        })
    return rows


def periodic_only_sweep(seeds: int) -> list[dict]:
    return _sweep(seeds, wall_stiffness=0.0, wall_damping=0.0)


def full_matched_sweep(seeds: int) -> list[dict]:
    return _sweep(seeds)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--output", type=Path, default=HERE / "horizon_sweep_results.json")
    args = parser.parse_args()
    results = {
        "seeds": args.seeds,
        "periodic_only": periodic_only_sweep(args.seeds),
        "full_matched": full_matched_sweep(args.seeds),
    }
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
