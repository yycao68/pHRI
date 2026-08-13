"""Active-torque-constraint experiment.

Every headline benchmark keeps F_max=150 N as the binding limit; the
first-move applied-torque row (9b) never actually saturates there (max raw
excess ~2e-5 Nm, see fair_offset_free_comparison.py). This script tightens
the joint-torque budget via phri.TAU_MAX_SCALE until (9b) actually binds and
compares two controllers that are IDENTICAL except for whether the QP knows
about tau_max:

    PROPOSED      "DI-MPC + Kalman 500 Hz"            (9b) embedded in the QP
    UNCONSTRAINED "DI-MPC + Kalman + NoTauRow 500 Hz" no torque row at all

The plant (FR3MuJoCoEnv.apply_torque) clips to tau_max regardless of which
controller is used -- so this is NOT a test of whether the robot ever sees
an infeasible torque (it never does). It isolates whether the QP's own
optimum already accounts for the limit (PROPOSED plans within budget) or
is corrected downstream by clipping after the fact (UNCONSTRAINED), which
shows up as degraded tracking and a persistent commanded/applied mismatch.

Reuses fair_offset_free_comparison.run() for the metrics harness (contact/SS
RMS, clip_sample_fraction, max_command_excess_Nm, qp_solve_count/failures).

Run:  python3 torque_constraint_active_experiment.py --n-cycles 3
Writes torque_constraint_active_experiment.json.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import phri
from fair_offset_free_comparison import run

PROPOSED = "DI-MPC + Kalman 500 Hz"
UNCONSTRAINED = "DI-MPC + Kalman + NoTauRow 500 Hz"


def _summarize(label, m):
    print(f"  {label:14s} contact={1e3*m['rms_contact']:7.3f} mm  "
          f"peak={1e3*m['peak_defl']:7.3f} mm  ss={1e3*m['ss_err']:7.3f} mm  "
          f"clip={100*m['clip_sample_fraction']:5.1f}%  "
          f"raw_excess={m['max_command_excess_Nm']:8.3f} Nm  "
          f"qp_fail={m['qp_failure_count']}/{m['qp_solve_count']}")


def main(n_cycles=3, scales=(1.0, 0.5, 0.3)):
    out = {}
    for scale in scales:
        phri.TAU_MAX_SCALE = None if scale == 1.0 else float(scale)
        print(f"\n== tau_max scale={scale:.2f} ({n_cycles} cycle(s)) ==")
        out[scale] = {}
        for label, name in ((PROPOSED, PROPOSED), (UNCONSTRAINED, UNCONSTRAINED)):
            result = run(name, n_cycles, verbose=False)
            m = result["metrics"]
            _summarize(label, m)
            out[scale][label] = m
    phri.TAU_MAX_SCALE = None

    out_json = HERE / "torque_constraint_active_experiment.json"
    payload = {"n_cycles": n_cycles, "scales": list(scales),
               "results": {str(s): {k: v for k, v in d.items()}
                            for s, d in out.items()}}
    out_json.write_text(json.dumps(payload, indent=2))
    print(f"\nresults -> {out_json}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--n-cycles", type=int, default=3)
    args = p.parse_args()
    main(n_cycles=args.n_cycles)
