#!/usr/bin/env python3
"""Run and log one OpenManipulator-X torque-verification trial."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from run_openmanipulator_hardware import run  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--test-id", required=True)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--backend", choices=["sim", "mjc", "dynamixel"], default="sim")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()
    args.output = ROOT / "results" / "hardware" / f"{args.test_id}.csv"
    out = run(args)
    print(f"[omx-verify] wrote {out}")


if __name__ == "__main__":
    main()
