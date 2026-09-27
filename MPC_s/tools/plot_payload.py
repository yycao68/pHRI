#!/usr/bin/env python3
"""Plot a configs/payload.yaml run as three separate figures -- end-effector
path (with an arrow showing the payload force direction), tracking error over
time, and joint angles over time -- each marking when the payload disturbance
becomes active.

Companion to tools/plot_traj.py; this one is specific to the "payload" case
(a single sustained force starting at t_start, no t_end) so it can annotate
the disturbance timing/direction that plot_traj.py knows nothing about.

Usage:
    python3 tools/plot_payload.py results/hardware/sim_payload.csv --config configs/payload.yaml
    python3 tools/plot_payload.py results/hardware/sim_payload.csv --config configs/payload.yaml --output results/hardware/sim_payload.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


def load_csv(path: Path) -> dict:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    if d.shape == ():
        d = d.reshape(1)
    return {n: np.asarray(d[n]) for n in d.dtype.names}


def cols(d: dict, prefix: str, n: int) -> np.ndarray:
    return np.vstack([d[f"{prefix}_{i}"].astype(float) for i in range(n)]).T


def steady_state_error(t: np.ndarray, err: np.ndarray) -> float:
    """Mean error over the last ~1s of the run."""
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    tail = max(1, min(len(err), int(round(1.0 / max(dt, 1e-3)))))
    return float(np.mean(err[-tail:]))


def load_payloads(config_path: Path) -> list[dict]:
    with open(config_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    payloads = (cfg.get("disturbance", {}) or {}).get("payload", []) or []
    if isinstance(payloads, dict):
        payloads = [payloads]
    return payloads


def plot_payload(d: dict, payloads: list[dict], ndof: int = 3, title: str = "") -> dict[str, plt.Figure]:
    t = d["t"].astype(float)
    ee = cols(d, "ee", 3)
    p_d = cols(d, "p_d", 3)
    err = d["err_mm"].astype(float)
    q = cols(d, "q", ndof)
    suffix = f" - {title}" if title else ""
    sse = steady_state_error(t, err)

    # --- figure 1: end-effector path + payload force-direction arrow ---
    fig_path, ax = plt.subplots(figsize=(6, 6))
    ax.plot(p_d[:, 0], p_d[:, 2], "--", color="0.5", lw=1, label="desired")
    ax.plot(ee[:, 0], ee[:, 2], "-", color="tab:blue", lw=1, label="actual")
    ax.plot(ee[0, 0], ee[0, 2], "o", color="tab:green", ms=7, label="start")
    ax.plot(ee[-1, 0], ee[-1, 2], "s", color="tab:red", ms=7, label="end")
    ax.annotate(f"steady-state err: {sse:.1f} mm", xy=(ee[-1, 0], ee[-1, 2]), xytext=(8, -10),
                textcoords="offset points", color="tab:red", fontsize=8)

    extent = max(ee[:, 0].max() - ee[:, 0].min(), ee[:, 2].max() - ee[:, 2].min(), 1e-3)
    arrow_len = 0.3 * extent
    for p in payloads:
        f_xz = np.asarray(p.get("force_N", [0.0, 0.0, 0.0]), dtype=float)[[0, 2]]
        f_norm = np.linalg.norm(f_xz)
        if f_norm < 1e-9:
            continue
        f_dir = f_xz / f_norm * arrow_len
        ax.annotate("", xy=(ee[0, 0] + f_dir[0], ee[0, 2] + f_dir[1]), xytext=(ee[0, 0], ee[0, 2]),
                    arrowprops=dict(facecolor="tab:purple", edgecolor="tab:purple", width=2, headwidth=8))
        ax.text(ee[0, 0] + f_dir[0] * 1.1, ee[0, 2] + f_dir[1] * 1.1, f"{f_norm:.2f} N",
                color="tab:purple", fontsize=9, ha="center", va="center")

    ax.margins(0.25)  # leave room for the arrow-tip label so it isn't clipped at the axes edge
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title(f"end-effector path + payload force direction{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_path.tight_layout()

    # --- figure 2: tracking error over time, marking when the payload turns on ---
    fig_err, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t, err, color="tab:blue")
    for p in payloads:
        t0 = float(p.get("t_start", 0.0))
        ax.axvline(t0, color="0.3", linestyle="--", lw=1)
        ax.axvspan(t0, t[-1], color="tab:purple", alpha=0.08)
        ax.text(t0, ax.get_ylim()[1] * 0.95, " payload on", color="0.3", fontsize=8, va="top")
    ax.axhline(sse, color="tab:red", linestyle=":", lw=1.2)
    ax.text(t[-1], sse, f" steady-state: {sse:.1f}mm", color="tab:red", fontsize=8, va="bottom", ha="right")
    ax.set_xlabel("t [s]"); ax.set_ylabel("err [mm]")
    ax.set_title(f"tracking error  (rmse={np.sqrt(np.mean(err**2)):.1f}mm){suffix}")
    fig_err.tight_layout()

    # --- figure 3: joint angles over time, same payload-onset marking ---
    fig_q, ax = plt.subplots(figsize=(6, 5))
    for i in range(ndof):
        ax.plot(t, q[:, i], label=f"q{i}")
    for p in payloads:
        t0 = float(p.get("t_start", 0.0))
        ax.axvline(t0, color="0.3", linestyle="--", lw=1)
        ax.axvspan(t0, t[-1], color="tab:purple", alpha=0.08)
    ax.set_xlabel("t [s]"); ax.set_ylabel("rad")
    ax.set_title(f"joint angles{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_q.tight_layout()

    return {"path": fig_path, "err": fig_err, "q": fig_q}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--config", type=Path, required=True,
                     help="the configs/payload.yaml (or similar) used for this run -- read for "
                          "disturbance.payload's force_N/t_start")
    ap.add_argument("--ndof", type=int, default=3)
    ap.add_argument("--output", type=Path, default=None, help="save PNG here instead of opening a window")
    args = ap.parse_args()

    d = load_csv(args.csv)
    payloads = load_payloads(args.config)
    if not payloads:
        print(f"[plot_payload] warning: no disturbance.payload entries found in {args.config}; "
              f"plotting without the force arrow/onset markers.")
    figs = plot_payload(d, payloads, ndof=args.ndof, title=args.csv.name)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        suffix = args.output.suffix or ".png"
        for name, fig in figs.items():
            out = args.output.with_name(f"{args.output.stem}_{name}{suffix}")
            fig.savefig(out, dpi=110)
            print(f"saved {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
