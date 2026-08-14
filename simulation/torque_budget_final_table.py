"""Regenerates the corrected Torque-Budget Sweep table (scale in {1.0, 0.32}
only -- scale 0.2 dropped after diagnostic review found its Soft slack
diverges to non-physical magnitudes, see soft_constraint_diagnostic.py).
Uses the now-fixed phri.py default (soft_torque_rho=1.0) and the corrected
budget-relative excess metric in fair_offset_free_comparison.py.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import phri
from fair_offset_free_comparison import run

CONTROLLERS = [
    ("C5", "DI-MPC + Kalman 100 Hz"),
    ("Unconstrained", "DI-MPC + Kalman + NoTauRow 100 Hz"),
    ("Observer", "DI-MPC + Kalman + Observer 100 Hz"),
    ("Soft", "DI-MPC + Kalman + Soft 100 Hz"),
]


def main(cycles=3):
    for scale in (1.0, 0.32):
        phri.TAU_MAX_SCALE = None if scale == 1.0 else scale
        print(f"\n== scale={scale} ==")
        for label, name in CONTROLLERS:
            r = run(name, cycles, verbose=False)
            m = r["metrics"]
            nz = m["soft_slack_nonzero_fraction"]
            nz_str = "None" if nz is None else f"{100*nz:.1f}%"
            print(
                f"  {label:14s} contact={1e3*m['rms_contact']:7.3f} mm  "
                f"fail={100*m['qp_failure_fraction']:5.1f}%  "
                f"clip%={100*m['clip_sample_fraction']:5.2f}%  "
                f"excess_full={m['max_command_excess_Nm']:9.3f} Nm  "
                f"excess_budget={m['max_command_excess_vs_tested_budget_Nm']:9.3f} Nm  "
                f"slack_max={m['soft_slack_max']}  "
                f"slack_rms={m['soft_slack_rms']}  "
                f"slack_nz%={nz_str}"
            )
    phri.TAU_MAX_SCALE = None


if __name__ == "__main__":
    main()
