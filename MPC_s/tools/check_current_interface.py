#!/usr/bin/env python3
"""Confirm the servos accept Current Control Mode.

Run this ONCE on the control computer, with the arm powered and SUPPORTED,
before any torque run -- of either controller. Both run_hardware.py and
run_pid.py command torque through Goal_Current, so if the servos do not accept
Operating_Mode=0 neither of them can work, and this script is the cheapest
place to find that out.

It (1) checks the DYNAMIXEL SDK is installed, (2) opens the port, (3) pings
each servo, (4) reads its model number and Operating Mode, (5) verifies it can
set Operating_Mode=0 (current) and read Present_Current -- all WITHOUT
commanding any torque. Goal_Current is never written, and torque is left
disabled on every exit path, including the failure paths.

It also reports (read-only by default) two registers relevant to per-tick
communication latency -- see "Is the ~6-8ms per tick communication-bound?"
in implementation_fix.md and tools/benchmark_io.py, which this is meant to
be run alongside:

- Return_Delay_Time (addr 9): how long the servo waits before replying to a
  read. Default is 250 (x2us = 500us) per servo; with 3 servos in one
  GroupSyncRead this adds up serially over the shared half-duplex bus. It is
  usually safe to set to 0 and is one of the most common, most overlooked
  sources of avoidable Dynamixel latency.
- Status_Return_Level (addr 68): whether the servo acks writes at all (2 =
  ack everything, 1 = ack reads only, 0 = ack nothing). GroupSyncWrite
  already targets a broadcast ID that servos never acknowledge regardless of
  this setting, so it should not affect send_torque() -- reported here so
  that fact is verified against the actual configured value, not assumed.

Pass --set-return-delay-time 0 to WRITE Return_Delay_Time on every servo (an
EEPROM write, so torque is toggled off around it exactly like Operating_Mode
above); omitted by default so this stays read-only unless asked.

Usage:
    python tools/check_current_interface.py --port COM3 --baud 1000000 --ids 12 13 14
    python tools/check_current_interface.py --port /dev/ttyUSB0 --ids 12 13 14 --json-out check.json
    python tools/check_current_interface.py --port /dev/ttyUSB0 --set-return-delay-time 0
"""
from __future__ import annotations

import argparse
import json
import sys


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[12, 13, 14])
    ap.add_argument("--json-out", default=None)
    ap.add_argument("--set-return-delay-time", type=int, default=None, metavar="N",
                     help="WRITE Return_Delay_Time=N (x2us) on every servo listed in --ids "
                          "(EEPROM write; torque toggled off around it). Omit to only report "
                          "the current value.")
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
                # torque off, set current mode (0), read it back, read present current
                ph.write1ByteTxRx(port, i, 64, 0)           # Torque_Enable = 0
                ph.write1ByteTxRx(port, i, 11, 0)           # Operating_Mode = 0 (Current)
                mode, _, _ = ph.read1ByteTxRx(port, i, 11)
                cur, _, _ = ph.read2ByteTxRx(port, i, 126)  # Present_Current
                entry["operating_mode_after_set"] = int(mode)
                entry["current_mode_accepted"] = (int(mode) == 0)
                entry["present_current_raw"] = int(cur)
                if int(mode) != 0:
                    report["current_control_ok"] = False

                if args.set_return_delay_time is not None:
                    ph.write1ByteTxRx(port, i, 9, int(args.set_return_delay_time))  # EEPROM write
                rdt, _, _ = ph.read1ByteTxRx(port, i, 9)     # Return_Delay_Time [x2us]
                srl, _, _ = ph.read1ByteTxRx(port, i, 68)    # Status_Return_Level
                entry["return_delay_time_x2us"] = int(rdt)
                entry["return_delay_time_us"] = int(rdt) * 2
                entry["status_return_level"] = int(srl)
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

    rdts = [e.get("return_delay_time_us") for e in report["servos"] if e.get("ping_ok")]
    if rdts:
        total_us = sum(rdts)
        print(f"\nReturn_Delay_Time per servo (us): {rdts}  ->  sums to ~{total_us}us "
              f"({total_us/1000.0:.3f}ms) of the shared bus's per-read_state() round trip, "
              f"since GroupSyncRead visits all servos on one half-duplex line.")
        if any(t > 0 for t in rdts):
            print("  Non-zero and usually safe to lower: "
                  "--set-return-delay-time 0 (see this file's own docstring). "
                  "This is a config change, not something --backend sim can validate -- "
                  "re-run tools/benchmark_io.py --backend dynamixel before/after to see if "
                  "it moved the needle on 'read' time.")
        print("Also check the USB-serial adapter's OWN latency timer (separate from this "
              "register, and on many systems the larger of the two): on Linux, "
              "`cat /sys/bus/usb-serial/devices/<ttyUSBx>/latency_timer` (often defaults to "
              "16ms) -- `echo 1 | sudo tee .../latency_timer` to lower it. Not something this "
              "script can read/set; it's an OS/driver setting, not a servo register.")
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
    sys.exit(0 if report["current_control_ok"] else 1)


if __name__ == "__main__":
    main()
