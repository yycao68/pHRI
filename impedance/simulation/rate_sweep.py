#!/usr/bin/env python3
"""Manager-rate sensitivity sweep for the two-rate residual MPC (Section 7.3).

Reruns the main matched benchmark, the leakage sweep, and the sensing-realism
sweep at 20/50/100 Hz, holding the horizon duration fixed at 0.20 s (so only
the re-check interval changes, not the manager's lookahead). Also captures a
representative-trial diagnostic (correction total variation, activation-event
rate, RMS-when-active-vs-inactive) that explains the resulting ranking rather
than just reporting it.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from verify_fr3_two_rate_benchmark import (
    Config,
    leakage_sweep,
    run_benchmark,
    run_trial,
    sensing_realism_sweep,
)

HERE = Path(__file__).resolve().parent

RATES = [("20Hz", 0.05, 4), ("50Hz", 0.02, 10), ("100Hz", 0.01, 20)]


def representative_diagnostic(cfg: Config, seed: int = 4) -> dict:
    rep = run_trial("two_rate", cfg, seed=seed)
    log = rep["log"]
    applied = np.asarray(log["applied_residual"])
    active = np.asarray(log["authorization_active"]) > 0.5
    residual_norm = np.linalg.norm(np.asarray(log["residual_position"]), axis=1)
    duration = log["time"][-1] - log["time"][0]

    total_variation = float(np.sum(np.linalg.norm(np.diff(applied, axis=0), axis=1)))
    transitions = int(np.sum((~active[:-1]) & active[1:]))
    return {
        "total_variation_of_applied_residual_N": total_variation,
        "tv_per_second_N_per_s": total_variation / duration,
        "activation_events": transitions,
        "activation_events_per_second": transitions / duration,
        "manager_updates_total": int(round(duration / cfg.manager_dt)),
        "rms_position_mm_when_active": float(1e3 * np.sqrt(np.mean(residual_norm[active] ** 2))) if active.any() else 0.0,
        "rms_position_mm_when_inactive": float(1e3 * np.sqrt(np.mean(residual_norm[~active] ** 2))) if (~active).any() else 0.0,
        "fraction_active": float(active.mean()),
    }


def _paired_stat(a: np.ndarray, b: np.ndarray) -> float:
    return float(100 * (a.mean() - b.mean()) / b.mean())


def run_sweep(benchmark_seeds: int, leakage_seeds: int, realism_seeds: int) -> dict:
    results = {}
    for label, manager_dt, horizon in RATES:
        cfg = Config(manager_dt=manager_dt, horizon=horizon)
        t0 = time.time()

        benchmark = run_benchmark(cfg, seeds=benchmark_seeds)
        raw = benchmark["raw"]
        two_rate_rms = np.array([r["residual_rms_mm"] for r in raw["two_rate"]])
        unguarded_rms = np.array([r["residual_rms_mm"] for r in raw["unguarded_mpc"]])
        vs_unguarded_pct = _paired_stat(two_rate_rms, unguarded_rms)

        diagnostic = representative_diagnostic(cfg)
        leak = leakage_sweep(cfg, seeds=leakage_seeds)
        realism = sensing_realism_sweep(cfg, seeds=realism_seeds)
        results[label] = {
            "manager_dt_ms": manager_dt * 1000,
            "horizon_s": manager_dt * horizon,
            "main_benchmark_summary": benchmark["summary"]["two_rate"],
            "main_benchmark_vs_unguarded_pct": vs_unguarded_pct,
            "diagnostic": diagnostic,
            "leakage_sweep": leak,
            "sensing_realism_sweep": realism,
            "wall_clock_s": time.time() - t0,
        }
    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leakage-seeds", type=int, default=5)
    parser.add_argument("--realism-seeds", type=int, default=5)
    parser.add_argument("--output", type=Path, default=HERE / "rate_sweep_results.json")
    args = parser.parse_args()
    results = run_sweep(20, args.leakage_seeds, args.realism_seeds)
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
