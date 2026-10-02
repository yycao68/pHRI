#!/usr/bin/env python3
"""Rough friction estimates from EXISTING closed-loop run-log CSVs (sim or real
hardware) -- not a controller, not a new experiment: this reads logs already
produced by `run_hardware.py` and reports what they imply about friction.

Why this exists, and why it's "rough": this project's dynamics model
(`lib/dynamics.py`) has NO friction term at all (see `docs/01_concepts.md`:
"the simulator has no friction model... the friction part of the story needs
real hardware"). The closed-loop disturbance observer's `d_hat` is explicitly
designed to lump "real friction, unmodelled dynamics, payload, ..." into one
signal (`lib/interaction_mpc.py`'s own docstring) -- so `d_hat`, read off an
already-recorded log, is the cheapest available proxy for friction, with real
caveats (task-space not joint-space, filtered by the observer's own
bandwidth, conflates friction with every other unmodeled effect). A genuinely
clean, joint-space, closed-loop-free measurement needs
`tools/chirp_response.py --backend dynamixel` (open-loop, no observer, no
feedback at all) -- this script is what's possible WITHOUT that data existing
yet, not a replacement for it once it does.

Three independent, differently-flawed estimates, pick what the log supports:

  static     A `hold`-task log's steady-state d_hat -> the equilibrium
             torque needed to hold position, mapped through Lambda(q)/J^T
             into joint-torque units. This is the HOLDING residual at zero
             velocity, NOT the breakaway/stiction threshold -- a real but
             different (and typically much smaller) quantity. Needs a log
             where the arm actually settles (long enough after the last
             reference change).

  viscous    A continuously-moving log (`circle` or a `step` log's move
             phases)'s d_hat correlated against ee_vel, by task-space axis,
             converted to task-space FORCE via Lambda(q) (computed PER
             SAMPLE from the logged q, not one fixed posture). The right
             SIGN (d_hat opposing ee_vel) and a nonzero slope support a
             viscous-friction-like term, but correlation is typically weak
             (this project's own first pass: ~0.33-0.38, i.e. velocity
             explains only ~11-14% of d_hat's variance) -- most of d_hat's
             variance is something else (curvature effects, observer noise,
             other model residual), not friction specifically. NOT mapped to
             per-joint torque: this project's logs don't record joint
             velocity (dq), only task-space ee_vel, and back-projecting
             through a pseudo-inverse Jacobian would introduce a null-space
             ambiguity this controller's posture term actually uses -- so
             this mode stays in task space rather than pretending to a
             joint-space answer the data doesn't support.

  breakaway  Scans each joint's logged `q` for candidate STICTION events: a
             joint pinned within ~1 encoder tick (not moving at all, not
             sensor noise -- the tolerance matches real encoder resolution)
             for a sustained run, immediately followed by it actually
             breaking free and moving. Reports the commanded-torque swing
             during the stuck window. Needs real hardware data (sim has no
             stiction, this mode will correctly find nothing there).

             IMPORTANT, found the hard way testing this on real step-task
             logs: a joint correctly holding still because the TRAJECTORY
             commands zero motion (e.g. during `step_dwell_s`) looks
             IDENTICAL to one stuck fighting friction -- both are "q flat,
             then q moves." Long stuck windows (several seconds) that line
             up with the task's own dwell time are almost always the former,
             not stiction. Events stuck longer than `--likely-dwell-s` are
             TAGGED, not dropped -- an earlier version of this tool dropped
             them outright and silently lost the clearest real event in this
             tool's own test data, because the genuine breakaway signal sat
             at the TAIL of a window that started during a legitimate
             preceding hold, with no discontinuity in the torque trace to
             split on. Short, large-swing, untagged events are the more
             credible candidates -- cross-check against the task's own
             dwell_s in its trajectory config either way.

Usage:
    python3 tools/estimate_friction.py static \
        --csv hold/completed/hw_hold_A_ry1e-8_1.csv --config configs/hold.yaml
    python3 tools/estimate_friction.py viscous \
        --csv circle/completed/hw_circle_L024_ry1e-8_1.csv --config configs/circle.yaml
    python3 tools/estimate_friction.py breakaway \
        --csv step/completed/hw_step_A_1.csv
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
from dynamics import OpenManipulatorDynamics  # noqa: E402

XM430_TICK_RAD = 2.0 * np.pi / 4096  # one encoder tick, XM430-W350 (12-bit, 4096 ticks/rev)


def load_csv(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(f"[estimate_friction] no such file: {path}")
    try:
        d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    except ValueError as exc:
        raise SystemExit(f"[estimate_friction] could not parse {path} as a run-log CSV ({exc})") from exc
    if d.dtype.names is None:
        raise SystemExit(f"[estimate_friction] {path} has no column-name header row -- not a run log")
    if d.shape == ():
        d = d.reshape(1)
    return {name: np.asarray(d[name]) for name in d.dtype.names}


def cols(d: dict, prefix: str, n: int = 3) -> np.ndarray:
    return np.vstack([d[f"{prefix}_{i}"].astype(float) for i in range(n)]).T


def lambda_and_Jxz(kin: OpenManipulatorKinematics, dyn: OpenManipulatorDynamics,
                    q: np.ndarray, armature: float, lam_damp: float) -> tuple[np.ndarray, np.ndarray]:
    """Lambda(q) (2x2, x-z task-space mass matrix) and Jxz (2x3), at ONE q --
    exactly what run_hardware.py itself computes per tick, so this matches
    what actually drove the logged d_hat."""
    n = kin.n
    J = kin.jacobian(q)
    Jxz = J[[0, 2], :]
    Mq = dyn.mass_matrix(q) + armature * np.eye(n)
    Minv = np.linalg.inv(Mq)
    Lam = np.linalg.inv(Jxz @ Minv @ Jxz.T + lam_damp * np.eye(2))
    return Lam, Jxz


def _robot_block(config_path: Path) -> dict:
    with open(config_path, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return config.get("robot", {})


def analyze_static(csv_path: Path, config_path: Path, tail_s: float) -> None:
    d = load_csv(csv_path)
    t = d["t"].astype(float)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    n_tail = max(1, int(round(tail_s / dt)))
    if n_tail >= len(t):
        print(f"[estimate_friction] WARNING: --tail-s {tail_s}s is >= the whole run "
              f"({t[-1]:.1f}s) -- using the full log as 'steady state', which may not be settled.")
        n_tail = len(t)

    p_d = cols(d, "p_d")[-n_tail:]
    p_d_range = p_d.max(axis=0) - p_d.min(axis=0)
    if np.max(p_d_range) > 1e-4:
        print(f"[estimate_friction] WARNING: p_d moved by up to {np.max(p_d_range)*1000:.2f}mm "
              f"during the tail window -- this doesn't look like a settled hold; the result below "
              f"is NOT a clean static estimate. Use a hold-task log, or shorten --tail-s.")

    q_tail = cols(d, "q")[-n_tail:]
    q_mean = q_tail.mean(axis=0)
    d_hat_tail = cols(d, "d_hat")[-n_tail:]
    d_hat_mean = np.array([d_hat_tail[:, 0].mean(), d_hat_tail[:, 2].mean()])  # x, z only (y is always 0)

    robot = _robot_block(config_path)
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))
    lam_damp = float(robot.get("lambda_damping", 2e-3))
    kin, dyn = OpenManipulatorKinematics(), OpenManipulatorDynamics()
    Lam, Jxz = lambda_and_Jxz(kin, dyn, q_mean, armature, lam_damp)

    F = Lam @ d_hat_mean          # task-space force [N]
    tau_eq = Jxz.T @ F            # joint-space torque equivalent [Nm]

    print(f"[estimate_friction static] {csv_path.name}  (tail={tail_s:.1f}s, n={n_tail} samples)")
    print(f"  mean q over tail:      {np.round(q_mean, 4)} rad")
    print(f"  mean d_hat (x,z):      {np.round(d_hat_mean, 4)} (residual accel. units)")
    print(f"  -> task-space force:   {np.round(F, 3)} N  (|F|={np.linalg.norm(F):.3f} N)")
    print(f"  -> joint torque equiv: {np.round(tau_eq, 3)} Nm  (per joint, this is NOT breakaway")
    print(f"                          stiction -- see --mode breakaway for that)")


def analyze_viscous(csv_path: Path, config_path: Path, skip_s: float) -> None:
    d = load_csv(csv_path)
    t = d["t"].astype(float)
    mask = t > skip_s
    if mask.sum() < 20:
        raise SystemExit(f"[estimate_friction] fewer than 20 samples after --skip-s {skip_s}s "
                          f"(run is {t[-1]:.1f}s) -- lower --skip-s or use a longer log.")

    q = cols(d, "q")[mask]
    ee_vel = cols(d, "ee_vel")[mask]
    d_hat = cols(d, "d_hat")[mask]

    robot = _robot_block(config_path)
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))
    lam_damp = float(robot.get("lambda_damping", 2e-3))
    kin, dyn = OpenManipulatorKinematics(), OpenManipulatorDynamics()

    # Lambda(q) PER SAMPLE (not one fixed posture) -- the arm's task-space
    # inertia genuinely changes as it moves, so converting d_hat -> force
    # with a single posture's Lambda (as a quick-and-dirty check would) is
    # itself an approximation this avoids where it costs nothing to avoid.
    F = np.zeros((len(q), 2))
    for i in range(len(q)):
        Lam, _ = lambda_and_Jxz(kin, dyn, q[i], armature, lam_damp)
        F[i] = Lam @ np.array([d_hat[i, 0], d_hat[i, 2]])

    print(f"[estimate_friction viscous] {csv_path.name}  (skip={skip_s:.1f}s, n={mask.sum()} samples)")
    print(f"  NOTE: correlation, not a clean measurement -- see this script's own docstring for why.")
    for axis, idx in (("x", 0), ("z", 2)):
        v = ee_vel[:, idx]
        f = F[:, 0] if idx == 0 else F[:, 1]
        corr = np.corrcoef(v, f)[0, 1]
        b, c = np.polyfit(v, f, 1)
        r2 = corr ** 2
        sign_note = "opposes velocity (consistent with viscous friction)" if b < 0 else \
                    "SAME sign as velocity (not consistent with viscous friction, or noise-dominated)"
        print(f"  axis {axis}: corr(F, ee_vel)={corr:+.3f}  R^2={r2:.3f}  "
              f"F ~= {b:+.2f}*vel {c:+.3f}  N per (m/s)  [{sign_note}]")
        print(f"           vel range {v.min():+.3f}..{v.max():+.3f} m/s, "
              f"F range {f.min():+.3f}..{f.max():+.3f} N")


def _distinct_within(x: np.ndarray, tol: float) -> np.ndarray:
    """Cluster-free count of distinct values in x within +/- tol of each other
    (used only for the printed diagnostic, not the detector itself)."""
    return np.unique(np.round(x / tol) * tol)


def detect_breakaway(t: np.ndarray, q: np.ndarray, tau: np.ndarray, tick_rad: float,
                      min_stuck_samples: int, likely_dwell_s: float | None,
                      break_tol_ticks: float = 3.0) -> list[dict]:
    """Scan one joint's (t, q, tau) for stiction-breakaway CANDIDATES: q
    pinned within ~1.5 ticks of its value at the start of a run for at least
    `min_stuck_samples` in a row, immediately followed by real motion
    (>= break_tol_ticks away) within the next few samples.

    Every candidate is returned -- NOTHING is dropped here, on purpose: an
    earlier version of this function discarded windows longer than a
    duration cap to filter out commanded dwell phases, and that silently
    ate the clearest real event in this script's own test data, because the
    genuine breakaway signal sat at the TAIL of a window that started during
    a legitimate preceding hold (the torque ramp is continuous across that
    boundary, there's no discontinuity to split on). Instead each event is
    tagged `likely_dwell` (duration > `likely_dwell_s`) for the caller to
    print a warning on, not to hide -- see this module's docstring."""
    events = []
    n = len(q)
    i = 0
    while i < n - min_stuck_samples:
        q_ref = q[i]
        j = i
        while j < n and abs(q[j] - q_ref) <= 1.5 * tick_rad:
            j += 1
        run_len = j - i
        if run_len >= min_stuck_samples and j < n:
            look_end = min(n, j + 5)
            moved = any(abs(q[k] - q_ref) > break_tol_ticks * tick_rad for k in range(j, look_end))
            if moved:
                k = j
                stuck_s = float(t[j - 1] - t[i])
                events.append(dict(
                    t_start=float(t[i]), t_break=float(t[k]), duration_s=float(t[k] - t[i]),
                    tau_start=float(tau[i]), tau_break=float(tau[k]),
                    tau_swing=float(tau[k] - tau[i]), n_distinct=len(_distinct_within(q[i:j], tick_rad)),
                    likely_dwell=(likely_dwell_s is not None and stuck_s > likely_dwell_s)))
                i = k + 1
                continue
        i += 1
    return events


def analyze_breakaway(csv_path: Path, tick_rad: float, min_stuck_s: float,
                       likely_dwell_s: float | None) -> None:
    d = load_csv(csv_path)
    t = d["t"].astype(float)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    min_stuck_samples = max(2, int(round(min_stuck_s / dt)))
    q = cols(d, "q")
    tau = cols(d, "tau")
    n_joints = q.shape[1]

    tag_note = f", events stuck > {likely_dwell_s:.2f}s flagged as likely-dwell (not dropped)" \
        if likely_dwell_s is not None else ""
    print(f"[estimate_friction breakaway] {csv_path.name}  "
          f"(tick={tick_rad*1000:.4f} mrad, min_stuck={min_stuck_s:.2f}s = {min_stuck_samples} samples{tag_note})")
    total = 0
    for j in range(n_joints):
        events = detect_breakaway(t, q[:, j], tau[:, j], tick_rad, min_stuck_samples, likely_dwell_s)
        if not events:
            print(f"  joint {j}: no breakaway events found")
            continue
        total += len(events)
        print(f"  joint {j}: {len(events)} event(s)")
        for e in events:
            dwell_note = "  [likely dwell, not stiction -- cross-check task's dwell_s]" if e["likely_dwell"] else ""
            print(f"    t={e['t_start']:7.3f}-{e['t_break']:7.3f}s  "
                  f"stuck {e['duration_s']:5.2f}s ({e['n_distinct']} distinct encoder value(s))  "
                  f"tau {e['tau_start']:+.4f} -> {e['tau_break']:+.4f} Nm  "
                  f"swing={e['tau_swing']:+.4f} Nm  |swing|={abs(e['tau_swing']):.4f} Nm{dwell_note}")
    if total == 0:
        print("  (no events on any joint -- expected for --backend sim logs, which have no "
              "stiction model; a real-hardware log with zero events either means this task/gain "
              "never stressed stiction, or --min-stuck-s / --tick-rad need adjusting)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["static", "viscous", "breakaway"])
    ap.add_argument("--csv", type=Path, required=True, help="a run-log CSV from run_hardware.py")
    ap.add_argument("--config", type=Path, default=None,
                     help="the config used for this run (needed for static/viscous, to get "
                          "dyn_armature_kg_m2/lambda_damping -- not needed for breakaway)")
    ap.add_argument("--tail-s", type=float, default=2.0,
                     help="[static] how many seconds at the END of the log count as 'steady state'")
    ap.add_argument("--skip-s", type=float, default=5.0,
                     help="[viscous] skip this many seconds at the START (startup ramp/transient)")
    ap.add_argument("--tick-rad", type=float, default=XM430_TICK_RAD,
                     help="[breakaway] one encoder tick, in radians (default: XM430-W350, "
                          "2*pi/4096) -- change if using a different servo model")
    ap.add_argument("--min-stuck-s", type=float, default=0.1,
                     help="[breakaway] minimum duration a joint must stay within ~1 tick to "
                          "count as 'stuck' (filters out brief coincidental non-motion)")
    ap.add_argument("--likely-dwell-s", type=float, default=2.0,
                     help="[breakaway] flag (NOT drop -- see docstring) events stuck longer "
                          "than this as more likely a commanded dwell phase than stiction; "
                          "pass a negative number to disable the flag")
    args = ap.parse_args()
    if args.mode == "breakaway" and args.likely_dwell_s is not None and args.likely_dwell_s < 0:
        args.likely_dwell_s = None

    if args.mode in ("static", "viscous") and args.config is None:
        raise SystemExit(f"--config is required for --mode {args.mode}")

    if args.mode == "static":
        analyze_static(args.csv, args.config, args.tail_s)
    elif args.mode == "viscous":
        analyze_viscous(args.csv, args.config, args.skip_s)
    else:
        analyze_breakaway(args.csv, args.tick_rad, args.min_stuck_s, args.likely_dwell_s)


if __name__ == "__main__":
    main()
