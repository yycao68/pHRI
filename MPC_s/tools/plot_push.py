#!/usr/bin/env python3
"""Plot a configs/push.yaml run as three separate figures -- end-effector path
(with arrows showing each push's force direction, anchored at wherever the arm
actually was when that push started), tracking error over time, and joint
angles over time -- each marking every push's [t_start, t_end] time window.

Companion to tools/plot_payload.py; pushes differ from a payload in that they
are timed pulses (t_start AND t_end, possibly several of them), not a single
sustained force from t_start onward, so the windows/arrows are per-push here
instead of "one force, in effect for the rest of the run".

Usage:
    python3 tools/plot_push.py results/hardware/sim_push.csv --config configs/push.yaml
    python3 tools/plot_push.py results/hardware/sim_push.csv --config configs/push.yaml --output results/hardware/sim_push.png
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


def load_pushes(config_path: Path) -> list[dict]:
    with open(config_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    pushes = (cfg.get("disturbance", {}) or {}).get("push", []) or []
    if isinstance(pushes, dict):
        pushes = [pushes]
    return pushes


def plot_push(d: dict, pushes: list[dict], ndof: int = 3, title: str = "") -> dict[str, plt.Figure]:
    t = d["t"].astype(float)
    ee = cols(d, "ee", 3)
    p_d = cols(d, "p_d", 3)
    err = d["err_mm"].astype(float)
    q = cols(d, "q", ndof)
    suffix = f" - {title}" if title else ""
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    sse = steady_state_error(t, err)

    # --- figure 1: end-effector path + one force-direction arrow per push,
    #     anchored where the end-effector actually was when that push started ---
    fig_path, ax = plt.subplots(figsize=(6, 6))
    ax.plot(p_d[:, 0], p_d[:, 2], "--", color="0.5", lw=1, label="desired")
    ax.plot(ee[:, 0], ee[:, 2], "-", color="tab:blue", lw=1, label="actual")
    ax.plot(ee[0, 0], ee[0, 2], "o", color="tab:green", ms=7, label="start")
    ax.plot(ee[-1, 0], ee[-1, 2], "s", color="tab:red", ms=7, label="end")
    ax.annotate(f"steady-state err: {sse:.1f} mm", xy=(ee[-1, 0], ee[-1, 2]), xytext=(8, -10),
                textcoords="offset points", color="tab:red", fontsize=8)

    extent = max(ee[:, 0].max() - ee[:, 0].min(), ee[:, 2].max() - ee[:, 2].min(), 1e-3)
    arrow_len = 0.3 * extent
    for k, p in enumerate(pushes):
        t0 = float(p.get("t_start", 0.0))
        idx = int(np.searchsorted(t, t0))
        idx = min(max(idx, 0), len(t) - 1)
        anchor = ee[idx, [0, 2]]
        f_xz = np.asarray(p.get("force_N", [0.0, 0.0, 0.0]), dtype=float)[[0, 2]]
        f_norm = np.linalg.norm(f_xz)
        if f_norm < 1e-9:
            continue
        f_dir = f_xz / f_norm * arrow_len
        c = colors[k % len(colors)]
        ax.annotate("", xy=(anchor[0] + f_dir[0], anchor[1] + f_dir[1]), xytext=(anchor[0], anchor[1]),
                    arrowprops=dict(facecolor=c, edgecolor=c, width=2, headwidth=8))
        ax.plot(anchor[0], anchor[1], "o", color=c, ms=4)
        ax.text(anchor[0] + f_dir[0] * 1.1, anchor[1] + f_dir[1] * 1.1,
                f"push{k+1}: {f_norm:.2f} N\n(t={t0:.1f}s)", color=c, fontsize=8, ha="center", va="center")

    ax.margins(0.25)  # leave room for the arrow-tip labels so they aren't clipped at the axes edge
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title(f"end-effector path + push force directions{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_path.tight_layout()

    # --- figure 2: tracking error over time, with each push's [t_start, t_end] window shaded ---
    fig_err, ax = plt.subplots(figsize=(6, 5))
    ax.plot(t, err, color="tab:blue")
    for k, p in enumerate(pushes):
        t0, t1 = float(p.get("t_start", 0.0)), float(p.get("t_end", t0))
        c = colors[k % len(colors)]
        ax.axvspan(t0, t1, color=c, alpha=0.12)
        ax.text(t0, ax.get_ylim()[1] * 0.95, f" push{k+1}", color=c, fontsize=8, va="top")
    ax.axhline(sse, color="tab:red", linestyle=":", lw=1.2)
    ax.text(t[-1], sse, f" steady-state: {sse:.1f}mm", color="tab:red", fontsize=8, va="bottom", ha="right")
    ax.set_xlabel("t [s]"); ax.set_ylabel("err [mm]")
    ax.set_title(f"tracking error  (rmse={np.sqrt(np.mean(err**2)):.1f}mm){suffix}")
    fig_err.tight_layout()

    # --- figure 3: joint angles over time, same push windows shaded ---
    fig_q, ax = plt.subplots(figsize=(6, 5))
    for i in range(ndof):
        ax.plot(t, q[:, i], label=f"q{i}")
    for k, p in enumerate(pushes):
        t0, t1 = float(p.get("t_start", 0.0)), float(p.get("t_end", t0))
        ax.axvspan(t0, t1, color=colors[k % len(colors)], alpha=0.12)
    ax.set_xlabel("t [s]"); ax.set_ylabel("rad")
    ax.set_title(f"joint angles{suffix}")
    ax.legend(fontsize=8, loc="best")
    fig_q.tight_layout()

    return {"path": fig_path, "err": fig_err, "q": fig_q}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--config", type=Path, required=True,
                     help="the configs/push.yaml (or similar) used for this run -- read for "
                          "disturbance.push's force_N/t_start/t_end")
    ap.add_argument("--ndof", type=int, default=3)
    ap.add_argument("--output", type=Path, default=None, help="save PNG here instead of opening a window")
    args = ap.parse_args()

    d = load_csv(args.csv)
    pushes = load_pushes(args.config)
    if not pushes:
        print(f"[plot_push] warning: no disturbance.push entries found in {args.config}; "
              f"plotting without the force arrows/time windows.")
    figs = plot_push(d, pushes, ndof=args.ndof, title=args.csv.name)

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
