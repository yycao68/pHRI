#!/usr/bin/env python3
"""Offline check: does the configured Cartesian trajectory (hold/circle) stay
inside this arm's actually reachable workspace, respect joint limits, and
avoid kinematic singularities -- BEFORE running it on real hardware.

For each sampled point on the trajectory, this does numerical inverse
kinematics (damped least-squares in the x-z plane, matching the 2D task space
the real controller uses) starting from the previous solution, and reports:
  - IK residual (did it actually reach the target, or get stuck short?)
  - whether the solution stays within joint_min/max_rad (with the same
    barrier_margin_rad used by the real controller)
  - the x-z Jacobian's smallest singular value (near 0 = near-singular pose,
    which is what caused the earlier 3D-vs-2D task-space bug -- worth
    tracking even now that the controller itself uses the 2D formulation)

This does NOT replace testing with --backend sim, but catches "the requested
circle physically doesn't fit" problems before spending time on a sim/hardware
run at all.

Usage:
    python3 tools/check_circle_workspace.py --config configs/circle.yaml
    python3 tools/check_circle_workspace.py --config configs/hold.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

from kinematics import OpenManipulatorKinematics  # noqa: E402
from interaction_mpc import CartesianTrajectory  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--samples", type=int, default=200, help="points to check over one trajectory period")
    ap.add_argument("--start-q", type=float, nargs=3, default=None,
                     help="starting joint config [q0,q1,q2] rad; default uses posture_q_rad from config")
    args = ap.parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    kin = OpenManipulatorKinematics()
    robot = config.get("robot", {})
    jmin = np.asarray(robot.get("joint_min_rad", [-1.8, -1.6, -1.8]), dtype=float)
    jmax = np.asarray(robot.get("joint_max_rad", [1.6, 1.4, 1.8]), dtype=float)
    margin = float(robot.get("joint_barrier_margin_rad", 0.15))
    q_start = np.asarray(args.start_q if args.start_q is not None
                          else robot.get("posture_q_rad", robot.get("sim_home_q_rad", [-0.185, -1.385, 0.0])),
                          dtype=float)

    p0 = kin.fk(q_start)
    traj_cfg = config.get("trajectory", {})
    traj = CartesianTrajectory(traj_cfg, p0, 0.0)

    typ = traj_cfg.get("type", "hold")
    period = float(traj_cfg.get("circle_period_s", 12.0)) if typ == "circle" else 2.0
    ts = np.linspace(0.0, period, args.samples)

    print(f"config: {args.config}  trajectory type: {typ}")
    print(f"start q = {np.round(q_start, 4)}  ->  ee0 = {np.round(p0, 4)}")
    print(f"joint_min_rad={jmin}  joint_max_rad={jmax}  barrier_margin_rad={margin}\n")

    q = q_start.copy()
    max_residual = 0.0
    min_sv_overall = np.inf
    worst_margin = np.full(3, np.inf)  # smallest (q - limit) distance seen, per joint, per direction
    bad_points = []

    for t in ts:
        p_d, _, _ = traj.sample(t)
        q, residual, min_sv = kin.solve_ik_xz(p_d[[0, 2]], q)
        max_residual = max(max_residual, residual)
        min_sv_overall = min(min_sv_overall, min_sv)
        margin_hi = jmax - q
        margin_lo = q - jmin
        worst_margin = np.minimum(worst_margin, np.minimum(margin_hi, margin_lo))
        if residual > 1e-3 or np.any(q < jmin) or np.any(q > jmax) or min_sv < 0.02:
            bad_points.append((t, q.copy(), residual, min_sv))

    print(f"IK residual over trajectory: max = {max_residual*1000:.3f} mm "
          f"({'OK, target reachable' if max_residual < 1e-3 else 'FAIL -- target NOT fully reachable'})")
    print(f"Jacobian min singular value over trajectory: {min_sv_overall:.4f} "
          f"({'OK' if min_sv_overall > 0.05 else 'WARNING -- getting close to a kinematic singularity'})")
    print(f"Closest approach to any joint limit (rad): {np.round(worst_margin, 4)} "
          f"({'OK, all >= margin' if np.all(worst_margin >= margin) else 'WARNING -- inside barrier margin at some point'})")

    if bad_points:
        print(f"\n{len(bad_points)} / {len(ts)} sampled points flagged:")
        for t, q, residual, min_sv in bad_points[:10]:
            flags = []
            if residual > 1e-3:
                flags.append(f"unreachable(err={residual*1000:.1f}mm)")
            if np.any(q < jmin) or np.any(q > jmax):
                flags.append("joint limit exceeded")
            if min_sv < 0.02:
                flags.append(f"near-singular(sv={min_sv:.4f})")
            print(f"  t={t:5.2f}s q={np.round(q,3)}  {', '.join(flags)}")
        if len(bad_points) > 10:
            print(f"  ... and {len(bad_points)-10} more")
        print("\nFAIL: this trajectory is not safely inside the workspace as configured. "
              "Consider reducing circle_radius_m, moving p0/posture_q_rad, or picking a "
              "different circle_plane before testing on hardware.")
    else:
        print("\nPASS: trajectory stays reachable, within joint limits (with margin), "
              "and away from singularities across the sampled points. Still verify with "
              "--backend sim before hardware, but this rules out a workspace/geometry problem.")


if __name__ == "__main__":
    main()
