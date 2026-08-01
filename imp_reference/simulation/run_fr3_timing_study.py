"""Timing study for the FR3 realization QP's solve-time gap (paper.md Section 6.3).

The main FR3 benchmark (run_fr3_experiments.py) reports a single run's mean
and max solve time per condition, without isolating why the admittance
generator's QP is slower than the impedance generator's despite both having
identical dimensions and constraints but different objective coefficients.
This script isolates it, along two independent axes:

1. Repeated full-benchmark runs (not a single run) to characterize solve-time
   variability directly with percentiles, rather than one mean/max pair.
2. Warm-starting (OSQP's own `warm_start(x=, y=)`, previously unused anywhere
   in this codebase -- FR3MPCConfig.warm_start defaults to False, so every
   existing benchmark, test, and paper number is unaffected by its
   existence) on vs. off, since a warm start only changes ADMM's initial
   iterate, not the QP being solved, so it cannot explain a *generator*-
   dependent gap if that gap is really about conditioning.

Conditioning is checked directly, not just plausibly invoked: this script
also computes the condition number of each generator's condensed QP cost
Hessian P at the same fixed state, which isolates the objective-coefficient
effect from anything about the solve trajectory.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

from fr3_mujoco import FR3MuJoCoEnv  # noqa: E402
from fr3_interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference3D,
    FR3MPCConfig,
    FR3RealizationMPC,
    ImpedanceReference3D,
)
from run_fr3_experiments import run_case  # noqa: E402

N_REPEATS = 5
DURATION_S = 6.0


def percentiles(values: np.ndarray, n_solves: int) -> dict:
    # Duplicating every value by the same held-between-solves factor leaves
    # the empirical distribution's mean/percentiles/max unchanged (see
    # solve_time_distribution's docstring), so computing them on the padded
    # per-tick array is valid; only "n" would be misleading if left as the
    # padded array's length, so the true solve count is reported separately.
    return {
        "n_solves": n_solves,
        "n_ticks_with_recorded_solve_time": int(values.size),
        "mean_ms": float(np.mean(values) * 1e3),
        "p50_ms": float(np.percentile(values, 50) * 1e3),
        "p95_ms": float(np.percentile(values, 95) * 1e3),
        "p99_ms": float(np.percentile(values, 99) * 1e3),
        "max_ms": float(np.max(values) * 1e3),
    }


def solve_time_distribution(env, generator, cfg: FR3MPCConfig) -> tuple[np.ndarray, int]:
    """Per-tick solve_time_s values, pooled across repetitions.

    run_case holds each solve's time constant across the ~mpc_every inner
    ticks until the next solve (the same convention run_fr3_experiments.py's
    own mean/max_solve_time_ms already use), so this array is the true
    per-solve value repeated a uniform number of times, not independent
    per-tick samples. That padding does not bias mean, percentiles, or max
    -- it preserves the empirical CDF -- but the true number of distinct
    solve events is also returned for an honest "n".
    """
    all_times: list[float] = []
    n_solves = 0
    for _ in range(N_REPEATS):
        log = run_case(env, generator, "mpc", cfg, duration=DURATION_S)
        times = log["solve_time_s"]
        all_times.extend(t for t in times if t > 0.0)
        # A new solve event is any tick whose held value differs from the
        # previous tick's (including the first nonzero value in the run).
        prev = 0.0
        for t in times:
            if t > 0.0 and t != prev:
                n_solves += 1
            prev = t
    return np.asarray(all_times), n_solves


def hessian_condition_number(env, generator, cfg: FR3MPCConfig) -> float:
    """Condition number of the condensed QP's cost Hessian P at the current
    (reset) state, isolating the generator's objective-coefficient effect
    from anything about how the solve trajectory evolves."""
    env.reset()
    dyn, state = env.get_dynamics_and_state()
    p_nominal = state.ee_pos.copy()
    R_d = state.ee_rot.copy()
    controller = FR3RealizationMPC(generator, cfg)
    force_forecast = np.zeros((cfg.horizon, 3))
    p, *_ = controller._condense(dyn, state, p_nominal, R_d, force_forecast)
    return float(np.linalg.cond(p))


def main() -> None:
    env = FR3MuJoCoEnv(timestep=0.001)
    generators = {"impedance": ImpedanceReference3D(), "admittance": AdmittanceReference3D()}
    base_cfg = FR3MPCConfig()

    report = {
        "purpose": "isolate the admittance-vs-impedance FR3 QP solve-time gap along the "
        "warm-start and Hessian-conditioning axes",
        "n_repeats_per_condition": N_REPEATS,
        "duration_s": DURATION_S,
        "conditions": {},
        "hessian_condition_number": {},
    }

    for gen_name, generator in generators.items():
        for warm_start in (False, True):
            cfg = replace(base_cfg, warm_start=warm_start)
            key = f"{gen_name}_warm_start={warm_start}"
            times, n_solves = solve_time_distribution(env, generator, cfg)
            report["conditions"][key] = percentiles(times, n_solves)
            print(key, report["conditions"][key])

        report["hessian_condition_number"][gen_name] = hessian_condition_number(
            env, generator, base_cfg
        )
        print(gen_name, "Hessian cond(P) =", report["hessian_condition_number"][gen_name])

    output_dir = Path(__file__).resolve().parents[1] / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "fr3_timing_study.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(f"Saved results to {output_dir / 'fr3_timing_study.json'}")


if __name__ == "__main__":
    main()
