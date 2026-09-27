#!/usr/bin/env python3
"""Interactive joint calibration helper (README step 1).

Torque stays OFF the whole time -- this script never commands any current.
It streams the RAW joint angle (no sign/offset applied) for each servo so you
can move the arm by hand and read off the values needed to fill in
`joint_sign` / `joint_offset_rad` in the config yaml.

Workflow:
  1. Run this script. It prints live raw q (rad) for joints 12,13,14 at ~5 Hz.
  2. By hand, move the arm to your chosen ZERO reference pose (whatever pose
     you want q=[0,0,0] to mean -- e.g. matching `sim_home_q_rad` conventions,
     or fully extended/vertical, your choice as long as it's consistent with
     how you write kinematics.py's frame convention).
  3. Type a short label and press ENTER to record a checkpoint (or just press
     ENTER with no text for an auto-generated label). Do this at a few
     recognizable poses (e.g. "zero", "j1_plus30deg", etc).
  4. For each joint: joint_offset_rad[k] = raw_q[k] measured at your chosen
     zero pose. joint_sign[k] = +1 if raw_q increases when the physical joint
     rotates in the direction your kinematics.py convention calls positive,
     otherwise -1.
  5. Ctrl+C to quit. All recorded checkpoints are printed again at the end.

    python3 tools/calibrate_joints.py --port COM3 --baud 1000000 --ids 12 13 14
"""
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time

import numpy as np

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_PRESENT_POSITION = 132
POS_PER_TICK = 2.0 * np.pi / 4096.0
CURRENT_MODE = 0


def _s32(v: int) -> int:
    return v - 4294967296 if v >= 2147483648 else v


def _input_worker(line_queue: "queue.Queue[str]") -> None:
    """Runs in a background thread; blocking input() works fine on Windows,
    unlike select.select(stdin), which only supports sockets there."""
    while True:
        try:
            line = input()
        except EOFError:
            break
        line_queue.put(line)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[12, 13, 14])
    ap.add_argument("--rate-hz", type=float, default=5.0)
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

    # Ensure torque is OFF on all joints -- this script never enables it.
    for i in args.ids:
        ph.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)
        ph.write1ByteTxRx(port, i, ADDR_OPERATING_MODE, CURRENT_MODE)
        ph.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)  # re-confirm OFF

    print(f"Torque is OFF on {args.ids}. Move the arm by hand.")
    print("Type a label and press ENTER to record a checkpoint (blank = auto label). Ctrl+C to quit.\n")

    def read_raw_q():
        q = []
        for i in args.ids:
            pos_t, cr, _ = ph.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)
            q.append(_s32(pos_t) * POS_PER_TICK if cr == 0 else float("nan"))
        return np.array(q)

    checkpoints = []
    line_queue: "queue.Queue[str]" = queue.Queue()
    t = threading.Thread(target=_input_worker, args=(line_queue,), daemon=True)
    t.start()

    period = 1.0 / max(args.rate_hz, 0.5)
    try:
        while True:
            q = read_raw_q()
            qs = "  ".join(f"id{i}: {v:+.4f} rad" for i, v in zip(args.ids, q))
            print(f"\r[live] {qs}    (type label + ENTER to record, Ctrl+C to quit)   ",
                  end="", flush=True)
            try:
                label = line_queue.get(timeout=period)
            except queue.Empty:
                continue
            label = label.strip() or f"checkpoint_{len(checkpoints) + 1}"
            q_now = read_raw_q()
            checkpoints.append((label, q_now.copy()))
            print(f"\nRecorded '{label}': " + ", ".join(f"id{i}={v:+.4f}" for i, v in zip(args.ids, q_now)))
    except KeyboardInterrupt:
        pass
    finally:
        for i in args.ids:
            ph.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)  # ensure OFF
        port.closePort()

    print("\n\n=== Recorded checkpoints ===")
    if not checkpoints:
        print("(none recorded)")
    for label, q in checkpoints:
        print(f"{label:20s}: " + ", ".join(f"id{i}={v:+.4f} rad" for i, v in zip(args.ids, q)))
    print(
        "\nNext: pick your 'zero pose' checkpoint -> its raw values become "
        "joint_offset_rad in the yaml. Compare two checkpoints where you moved "
        "one joint in the kinematics-positive direction to confirm joint_sign "
        "(+1 if raw q increased, -1 if it decreased)."
    )


if __name__ == "__main__":
    main()
