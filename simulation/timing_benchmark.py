"""Real-time QP solve timing at 100 Hz and 500 Hz.

Wraps ImpedanceMPCController.control() with a perf_counter timer (wall-clock,
includes Kalman step + Gamma/H/h rebuild + OSQP solve + torque assembly --
everything done once per QP tick) and drives it through the same 3-cycle
circular-trajectory + step-force scenario as the headline benchmark (via
fair_offset_free_comparison.run(), unmodified) for C5 (100 Hz) and C7
(500 Hz). Reports mean, p50, p99, max solve time and deadline-miss counts
(deadline = the QP period itself, 10 ms / 2 ms).

Run:  python3 timing_benchmark.py
Writes timing_benchmark.json.
"""
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import impedance_mpc
from fair_offset_free_comparison import run

_TIMES: list[float] = []
_orig_control = impedance_mpc.ImpedanceMPCController.control


def _timed_control(self, *args, **kwargs):
    t0 = time.perf_counter()
    out = _orig_control(self, *args, **kwargs)
    _TIMES.append(time.perf_counter() - t0)
    return out


def bench(name: str, deadline_s: float, n_cycles: int = 3) -> dict:
    global _TIMES
    _TIMES = []
    impedance_mpc.ImpedanceMPCController.control = _timed_control
    try:
        run(name, n_cycles, verbose=False)
    finally:
        impedance_mpc.ImpedanceMPCController.control = _orig_control
    t = np.array(_TIMES) * 1e3  # ms
    misses = int(np.sum(t > deadline_s * 1e3))
    return {
        "n_solves": len(t),
        "mean_ms": float(np.mean(t)),
        "p50_ms": float(np.percentile(t, 50)),
        "p99_ms": float(np.percentile(t, 99)),
        "max_ms": float(np.max(t)),
        "deadline_ms": deadline_s * 1e3,
        "deadline_misses": misses,
        "deadline_miss_fraction": misses / len(t),
    }


def main():
    print(f"Platform: {platform.processor() or platform.machine()}, "
          f"{platform.system()} {platform.release()}, Python {platform.python_version()}")
    results = {}
    for label, name, deadline in (
        ("100 Hz (C5)", "DI-MPC + Kalman 100 Hz", 0.010),
        ("500 Hz (C7)", "DI-MPC + Kalman 500 Hz", 0.002),
    ):
        print(f"\nBenchmarking {label} ...")
        r = bench(name, deadline)
        results[label] = r
        print(f"  n={r['n_solves']}  mean={r['mean_ms']:.3f} ms  "
              f"p50={r['p50_ms']:.3f} ms  p99={r['p99_ms']:.3f} ms  "
              f"max={r['max_ms']:.3f} ms  deadline={r['deadline_ms']:.1f} ms  "
              f"misses={r['deadline_misses']}/{r['n_solves']}")

    out = HERE / "timing_benchmark.json"
    out.write_text(json.dumps({
        "platform": f"{platform.processor() or platform.machine()}, "
                    f"{platform.system()} {platform.release()}, "
                    f"Python {platform.python_version()}",
        "results": results,
    }, indent=2))
    print(f"\nresults -> {out}")


if __name__ == "__main__":
    main()
