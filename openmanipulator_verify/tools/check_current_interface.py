#!/usr/bin/env python3
"""Confirm the OpenManipulator-X servos accept Current Control Mode.

Run this ONCE on the control computer with the arm powered and SUPPORTED before
any torque run. It (1) checks the DYNAMIXEL SDK is installed, (2) opens the port,
(3) pings each servo, (4) reads model number and Operating Mode, (5) verifies it
can set Operating_Mode=0 (current) and read Present_Current -- WITHOUT commanding
any torque (Goal_Current stays 0, torque left disabled at the end).

    python3 tools/check_current_interface.py --port /dev/ttyUSB0 --ids 11 12 13 14
"""
from __future__ import annotations

import argparse
import json
import sys


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[11, 12, 13, 14])
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    try:
        from dynamixel_sdk import PortHandler, PacketHandler
    except ImportError:
        print("FAIL: dynamixel-sdk not installed. pip install dynamixel-sdk")
        sys.exit(2)

    port = PortHandler(args.port); ph = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        print(f"FAIL: could not open {args.port} @ {args.baud}")
        sys.exit(2)

    report = {"port": args.port, "servos": [], "current_control_ok": True}
    try:
        for i in args.ids:
            model, cr, _ = ph.ping(port, i)
            entry = {"id": i, "ping_ok": cr == 0, "model_number": int(model) if cr == 0 else None}
            if cr == 0:
                # torque off, set current mode (0), read back, read present current
                ph.write1ByteTxRx(port, i, 64, 0)          # Torque_Enable = 0
                ph.write1ByteTxRx(port, i, 11, 0)          # Operating_Mode = 0 (Current)
                mode, _, _ = ph.read1ByteTxRx(port, i, 11)
                cur, _, _ = ph.read2ByteTxRx(port, i, 126)  # Present_Current
                entry["operating_mode_after_set"] = int(mode)
                entry["current_mode_accepted"] = (int(mode) == 0)
                entry["present_current_raw"] = int(cur)
                if int(mode) != 0:
                    report["current_control_ok"] = False
            else:
                report["current_control_ok"] = False
            report["servos"].append(entry)
    finally:
        for i in args.ids:
            ph.write1ByteTxRx(port, i, 64, 0)  # ensure torque OFF
        port.closePort()

    print(json.dumps(report, indent=2))
    print("PASS: all servos accept Current Control Mode" if report["current_control_ok"]
          else "FAIL: at least one servo did not accept current mode")
    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(report, f, indent=2)
    sys.exit(0 if report["current_control_ok"] else 1)


if __name__ == "__main__":
    main()
