"""Re-audit the two Kalman MPC rows after centered-input verification.

Runs the paper's three-cycle Benchmark I protocol for C5 and C7 and writes a
small machine-readable ledger.  The controllers themselves are constructed by
``phri.make_mpc_controller``; this script adds no alternate control path.
"""

from __future__ import annotations

import json
from pathlib import Path

from fr3_mujoco import FR3MuJoCoEnv
import phri


OUT_JSON = Path(__file__).with_name("centered_input_reaudit.json")
CONTROLLERS = [
    "DI-MPC + Kalman 100 Hz",
    "DI-MPC + Kalman 500 Hz",
]


def main() -> None:
    results = {}
    for name in CONTROLLERS:
        env = FR3MuJoCoEnv(timestep=0.001)
        run = phri.run_episode(
            name,
            env,
            n_cycles=3,
            hifreq_dt=phri.MPC_DT_FAST,
            verbose=True,
        )
        results[name] = {
            key: float(run[key])
            for key in ("rms_total", "rms_contact", "peak_defl", "ss_err")
        }

    payload = {
        "protocol": {
            "benchmark": "Benchmark I circular trajectory under 15 N step force",
            "cycles": 3,
            "duration_s": 24.0,
            "inner_rate_hz": 1000,
            "effort_cost": "0.5*(U+d_seq)^T*R_bar*(U+d_seq)",
        },
        "results": results,
    }
    OUT_JSON.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps(payload, indent=2))
    print(f"Wrote {OUT_JSON}")


if __name__ == "__main__":
    main()
