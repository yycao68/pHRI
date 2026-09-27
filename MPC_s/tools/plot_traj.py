#!/usr/bin/env python3
"""Plot a 3-DOF OpenManipulator-X torque-control CSV log as a set of separate
figures: end-effector path (actual vs desired) in the x-z plane, x/z position
reference vs. actual over time, tracking error over time, joint angles over
time, controller output (commanded joint torque tau) over time, measured
joint current over time, achieved control-loop rate over time, and the
disturbance observer's estimate (d_hat) over time.

Use this any time you need to see the actual shape of the motion -- e.g. to
tell whether a "circle" run actually traced a circle or stalled/drifted, or
whether the observer's d_hat settled to something small and steady vs. kept
growing.

The current plot is most useful on real-hardware logs (--backend dynamixel):
there, cur_i is an independent sync-read of the servo's own Present_Current
register, so plotting it alongside tau lets you check, e.g., whether a frozen
joint's current is tracking its commanded torque (ruling out a comm/current
problem) or pinned at current_limit_ticks (saturation) -- see
lib/dynamixel_backend.py's read_state(). On --backend sim logs, cur_i is just
tau_i/torque_constant_Nm_per_A echoed back (SimArmBackend has no independent
current model), so this plot carries no extra information there.

The control-loop-rate plot is worth checking on every log, sim or hardware:
--duration measures simulated/trajectory time, not wall-clock time, so a slow
machine can silently achieve well under the configured control rate (e.g.
~50 Hz against a configured 100 Hz) without --duration reflecting that --
this plot is how you'd notice.

The perr-vs-perr_hat plot shows the Kalman disturbance observer's own
one-step-ahead prediction of the task-space position error (perr_hat) against
the actual measurement (perr) -- see docs/02_tuning_guide.md for what NIS
(shown in the title) means and why it should average out to roughly the
measurement dimension (2, for this project) in a well-tuned filter.

Usage:
    python3 tools/plot_traj.py results/hardware/sim_circle.csv
    python3 tools/plot_traj.py results/hardware/sim_circle.csv --output results/hardware/sim_circle.png
    # -> saves sim_circle_path.png, sim_circle_posref.png, sim_circle_err.png,
    #    sim_circle_q.png, sim_circle_tau.png, sim_circle_cur.png,
    #    sim_circle_hz.png, sim_circle_yhat.png, and sim_circle_dhat.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path) -> dict:
    # numpy's own failure here is just "ValueError: Some errors were detected !",
    # which says nothing about what was wrong with the file -- and the most common
    # mistake by far is handing this script a path that is not a run log at all
    # (a .py, a directory, a not-yet-created output name). Catch that up front and
    # say which file and why, rather than passing the opaque message through.
    if not path.exists():
        raise SystemExit(f"[plot_traj] no such file: {path}")
    if path.is_dir():
        raise SystemExit(f"[plot_traj] {path} is a directory -- pass a single run-log CSV")
    if path.suffix.lower() != ".csv":
        raise SystemExit(
            f"[plot_traj] {path} is not a .csv -- this script's positional argument is the "
            f"run-log CSV to plot (e.g. results/hardware/sim_circle.csv), not a script or config")
    try:
        d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    except ValueError as exc:
        raise SystemExit(
            f"[plot_traj] could not parse {path} as a run-log CSV ({exc}). Expected a comma-"
            f"separated file with a header row (t, ee_0.., q_0.., tau_0.., ...) as written by "
            f"run_hardware.py.") from exc
    if d.dtype.names is None:
        raise SystemExit(f"[plot_traj] {path} has no column-name header row -- not a run log")
    if d.shape == ():
        d = d.reshape(1)
    return {n: np.asarray(d[n]) for n in d.dtype.names}


def cols(d: dict, prefix: str, n: int) -> np.ndarray:
    return np.vstack([d[f"{prefix}_{i}"].astype(float) for i in range(n)]).T


def plot_traj(d: dict, ndof: int = 3, title: str = "") -> dict[str, plt.Figure]:
    t = d["t"].astype(float)
    ee = cols(d, "ee", 3)
    p_d = cols(d, "p_d", 3)
    err = d["err_mm"].astype(float)
    q = cols(d, "q", ndof)
    tau = cols(d, "tau", ndof)
    cur = cols(d, "cur", ndof)
    suffix = f" - {title}" if title else ""

    fig_path, ax = plt.subplots(figsize=(6, 6))
    ax.plot(p_d[:, 0], p_d[:, 2], "--", color="0.5", lw=1, label="desired")
    ax.plot(ee[:, 0], ee[:, 2], "-", color="tab:blue", lw=1, label="actual")
    # sprinkle a few arrows along the actual path, each pointing a few samples ahead,
    # so the direction of travel is visible at a glance
    n_arrows = min(12, max(1, len(ee) - 1))
    step = max(1, len(ee) // 100)
    for i in np.linspace(0, len(ee) - 1 - step, n_arrows, dtype=int):
        j = i + step
        ax.annotate("", xy=(ee[j, 0], ee[j, 2]), xytext=(ee[i, 0], ee[i, 2]),
                    arrowprops=dict(arrowstyle="-|>", color="tab:blue", alpha=0.85, mutation_scale=14))
    ax.plot(ee[0, 0], ee[0, 2], "o", color="tab:green", ms=7, label="start")
    ax.plot(ee[-1, 0], ee[-1, 2], "s", color="tab:red", ms=7, label="end")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title(f"end-effector path (x-z plane){suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_path.tight_layout()

    # Same ee/p_d data as the path plot above, but split into x and z vs. time instead of
    # x-z space -- the path plot shows the overall shape well but hides WHEN along the run
    # actual departs from desired (e.g. a stall vs. a steady lag); this makes that visible
    # per axis.
    fig_posref, (axx, axz) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
    axx.plot(t, p_d[:, 0], "--", color="0.5", lw=1, label="desired")
    axx.plot(t, ee[:, 0], "-", color="tab:blue", lw=1, label="actual")
    axx.set_ylabel("x [m]")
    axx.set_title(f"x/z position: reference vs. actual over time{suffix}")
    axx.legend(fontsize=8, loc="best")
    axz.plot(t, p_d[:, 2], "--", color="0.5", lw=1, label="desired")
    axz.plot(t, ee[:, 2], "-", color="tab:blue", lw=1, label="actual")
    axz.set_xlabel("t [s]"); axz.set_ylabel("z [m]")
    axz.legend(fontsize=8, loc="best")
    fig_posref.tight_layout()

    fig_err, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t, err, color="tab:blue")
    ax.set_xlabel("t [s]"); ax.set_ylabel("err [mm]")
    ax.set_title(f"tracking error  (rmse={np.sqrt(np.mean(err**2)):.1f}mm){suffix}")
    fig_err.tight_layout()

    fig_q, ax = plt.subplots(figsize=(6, 5))
    for i in range(ndof):
        ax.plot(t, q[:, i], label=f"q{i}")
    ax.set_xlabel("t [s]"); ax.set_ylabel("rad")
    ax.set_title(f"joint angles{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_q.tight_layout()

    fig_tau, ax = plt.subplots(figsize=(6, 5))
    for i in range(ndof):
        ax.plot(t, tau[:, i], label=f"tau{i}")
    ax.set_xlabel("t [s]"); ax.set_ylabel("tau [Nm]")
    ax.set_title(f"controller output (commanded joint torque){suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_tau.tight_layout()

    fig_cur, ax = plt.subplots(figsize=(6, 5))
    for i in range(ndof):
        ax.plot(t, cur[:, i], label=f"cur{i}")
    ax.set_xlabel("t [s]"); ax.set_ylabel("current [A]")
    ax.set_title(f"measured joint current{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_cur.tight_layout()

    figs = {"path": fig_path, "posref": fig_posref, "err": fig_err, "q": fig_q, "tau": fig_tau, "cur": fig_cur}

    # achieved control-loop rate: period_ms[0] isn't a real loop period (it's measured
    # from loop-start, before any iteration ran), so it's dropped here.
    if "period_ms" in d:
        period_ms = d["period_ms"].astype(float)
        hz = 1000.0 / np.clip(period_ms[1:], 1e-6, None)
        t_hz = t[1:]
        fig_hz, ax = plt.subplots(figsize=(6, 5))
        ax.plot(t_hz, hz, color="tab:purple")
        ax.set_xlabel("t [s]"); ax.set_ylabel("achieved rate [Hz]")
        ax.set_title(f"control-loop rate  (mean={np.mean(hz):.1f}Hz, p99={np.percentile(hz,99):.1f}Hz){suffix}")
        fig_hz.tight_layout()
        figs["hz"] = fig_hz

    # perr vs. perr_hat: the Kalman disturbance observer's own one-step-ahead prediction
    # of the task-space position error (perr_hat = C@z^-) against the actual measurement
    # (perr = e = ee_xz - p_d_xz) -- a visual check of filter consistency: a well-tuned
    # filter should show perr_hat tracking perr closely. Only present when the log has a
    # y_hat_0 column.
    if "y_hat_0" in d:
        y_x, y_z = ee[:, 0] - p_d[:, 0], ee[:, 2] - p_d[:, 2]
        y_hat_x, y_hat_z = d["y_hat_0"].astype(float), d["y_hat_2"].astype(float)
        fig_yhat, (axx, axz) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
        axx.plot(t, y_x, "-", color="tab:blue", lw=1, label="perr (measured)")
        axx.plot(t, y_hat_x, "--", color="tab:orange", lw=1, label="perr_hat (predicted)")
        axx.set_ylabel("e_x [m]")
        nis_note = f", mean nis={np.mean(d['nis'].astype(float)):.2f} (dim(y)=2)" if "nis" in d else ""
        axx.set_title(f"Kalman filter: perr vs. perr_hat{nis_note}{suffix}")
        axx.legend(fontsize=8, loc="best")
        axz.plot(t, y_z, "-", color="tab:blue", lw=1, label="perr (measured)")
        axz.plot(t, y_hat_z, "--", color="tab:orange", lw=1, label="perr_hat (predicted)")
        axz.set_xlabel("t [s]"); axz.set_ylabel("e_z [m]")
        axz.legend(fontsize=8, loc="best")
        fig_yhat.tight_layout()
        figs["yhat"] = fig_yhat

    # d_hat: the disturbance observer's estimate. Columns are padded [x, 0(unused y), z]
    # (task space is the x-z plane; the y column is always 0), so only indices 0 and 2
    # carry information.
    if "d_hat_0" in d:
        d_hat_x = d["d_hat_0"].astype(float)
        d_hat_z = d["d_hat_2"].astype(float)
        d_hat_mag = np.hypot(d_hat_x, d_hat_z)
        fig_dhat, ax = plt.subplots(figsize=(6, 5))
        ax.plot(t, d_hat_x, label="d_hat_x")
        ax.plot(t, d_hat_z, label="d_hat_z")
        ax.plot(t, d_hat_mag, "--", color="0.3", label="|d_hat|")
        ax.set_xlabel("t [s]"); ax.set_ylabel("d_hat [residual accel. units]")
        ax.set_title(f"disturbance observer estimate  (peak |d_hat|={np.max(d_hat_mag):.2f}){suffix}")
        ax.legend(fontsize=8, loc="best")
        fig_dhat.tight_layout()
        figs["dhat"] = fig_dhat

    return figs


def _show_until_enter(figs: dict[str, plt.Figure]) -> None:
    """Like plt.show(), but pressing ENTER closes every window at once instead
    of having to close each one by hand.

    Deliberately NOT a poll loop calling plt.pause() repeatedly: plt.pause()
    calls show(block=False) again on every iteration, which on some backends
    re-raises/re-flashes the window -- at a ~0.1s polling interval that shows
    up as visible flicker. There is nothing else running concurrently here, so
    a single blocking input() call is both simpler and flicker-free.
    """
    plt.show(block=False)
    plt.pause(0.001)  # let the initial draw finish once, then never touch the event loop again
    print("press ENTER to close all plot windows...")
    try:
        input()
    except EOFError:
        pass
    plt.close("all")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--ndof", type=int, default=3)
    ap.add_argument("--output", type=Path, default=None, help="save PNG here instead of opening a window")
    args = ap.parse_args()

    d = load_csv(args.csv)
    figs = plot_traj(d, ndof=args.ndof, title=args.csv.name)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        suffix = args.output.suffix or ".png"
        for name, fig in figs.items():
            out = args.output.with_name(f"{args.output.stem}_{name}{suffix}")
            fig.savefig(out, dpi=110)
            print(f"saved {out}")
    else:
        _show_until_enter(figs)


if __name__ == "__main__":
    main()
