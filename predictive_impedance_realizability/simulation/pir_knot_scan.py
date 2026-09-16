#!/usr/bin/env python3
"""Task 1 of the PIR synthesis note: the ``(K0, D0)`` go/no-go scan.

The merged controller splits phri2's command into a passive nominal plus the
residual the QP optimizes and the tank authorizes::

    F_cmd = F_nom + F_r,        F_nom = -K0 e - D0 v

Merged Lemma 1's first precondition is that the **anchor**
``a = tau_base + J_v^T F_nom`` stays inside the derated torque envelope at
every tick.  Whether a ``(K0, D0)`` exists that satisfies that precondition
*and* leaves the alpha -> 0 fallback strong enough to hold the 20 N push
inside the 0.06 m workspace bound is the question this script answers.  If no
such cell exists, the passive-nominal split is dead and the proof that
depends on it is an empty lemma.

Four diagnostics per grid cell (numbering follows the note's Section 13.2):

  1  ``max_t ||tau_base + J_v^T F_nom||_inf / cap`` on the phri2 replay  < 1
  1b the same ratio, peaked over the closed-loop alpha -> 0 fallback run  < 1
  2  ``lambda_min(K0), lambda_min(D0)``               passive             > 0
  3  settled ``|e_z|`` at alpha -> 0 under 20 N       fallback holds  <= 0.06 m
  4  ``max_t ||J_v^T F_nom||_inf / cap``              not budget-eating < beta

Diagnostic 1b is not in the note's table.  It is added because it turned out
to be the binding constraint, and because without it the scan answers the
wrong question.  Diagnostic 1 is measured on a trajectory the phri2 MPC keeps
near the nominal pose, so ``F_nom`` is small there; diagnostic 3 defines a
completely different operating point -- the displaced equilibrium the robot
must actually hold once the residual is de-authorized.  Merged Lemma 1
requires ``|a| <= cap`` at *every* tick, which includes every tick of that
fallback.  Checking row 1 in one place and row 3 in another produces cells
that look green only because no single trajectory was ever asked to satisfy
both: on this benchmark three such false-green cells appear (k = 344.2 N/m,
d in {2.0, 5.9, 9.7}), and all three exceed the joint-4 cap by 4-10% the
moment the fallback is actually simulated.

Note that 1b is peaked over the whole fallback run, transient included, not
evaluated at the equilibrium alone -- a lightly damped cell overshoots well
past its own equilibrium, and Lemma 1's precondition has no exemption for
overshoot.

Run::

    python3 pir_knot_scan.py                 # both envelopes, full grid
    python3 pir_knot_scan.py --quick         # coarse grid, for smoke testing
"""

from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pir_common as pc

#: Section 13.2 proposes k in [10, 400], d in [2, 60], and says to widen if the
#: green region sits at an edge.  It does: diagnostic 3 needs |F_h| / k <= 0.06,
#: i.e. k >= 333 N/m, which is the top edge of the proposed range.  The grid is
#: therefore widened to 1200 N/m and log-spaced, so the decade that matters is
#: resolved instead of being one column.
#: The feasible region, where it exists at all, is a thin sliver: a 24 x 16
#: grid steps straight over it and reports an empty region.  The band around
#: k in [300, 520] N/m, d in [8, 40] N.s/m is therefore refined, and
#: ``pir_verify.py``'s check C re-derives the same window independently.
K_GRID = np.unique(np.concatenate([
    np.round(np.geomspace(10.0, 1200.0, 24), 1),
    np.round(np.linspace(300.0, 520.0, 12), 1),
]))
D_GRID = np.unique(np.concatenate([
    np.round(np.linspace(2.0, 60.0, 16), 2),
    np.round(np.linspace(8.0, 40.0, 9), 2),
]))
BETA_DEFAULT = 0.5

QUICK_K = np.unique(np.round(np.geomspace(10.0, 1200.0, 6), 1))
QUICK_D = np.round(np.linspace(2.0, 60.0, 4), 2)


def score_envelope(
    k_grid: np.ndarray,
    d_grid: np.ndarray,
    d1: np.ndarray,
    d1b: np.ndarray,
    e_ss: np.ndarray,
    e_peak: np.ndarray,
    settled: np.ndarray,
    d4: np.ndarray,
    beta: float,
    cap: np.ndarray,
    base_only: dict,
    phri2_ratio: float,
) -> dict:
    """Turn one envelope's five diagnostic maps into the decision-gate block.

    Kept separate from the measurement so ``--rescore`` can re-derive every
    verdict from a stored ``pir_knot_scan.json`` without re-running 900 MuJoCo
    trials -- and so the published JSON is always exactly what the current
    scoring code produces, rather than a snapshot that can drift from it.
    """
    bound = pc.WORKSPACE_BOUND_M
    # Diagnostic 2 is trivially satisfied for every cell on this grid
    # (isotropic K0 = k I, D0 = d I with k, d > 0), so it is recorded as a
    # scalar fact rather than a map that would be uniformly green.
    d2_ok = bool(k_grid.min() > 0 and d_grid.min() > 0)
    pass1, pass1b, pass3 = d1 < 1.0, d1b < 1.0, e_ss <= bound
    common = pass1 & pass1b & pass3 & d2_ok
    worst_use = np.maximum(np.maximum(d1, d1b), e_ss / bound)

    def describe(i: int, j: int) -> dict:
        return {
            "k_N_per_m": float(k_grid[i]),
            "d_Ns_per_m": float(d_grid[j]),
            "diag1_anchor_ratio_replay": float(d1[i, j]),
            "diag1b_anchor_ratio_fallback": float(d1b[i, j]),
            "diag3_e_ss_axis_m": float(e_ss[i, j]),
            "diag3_e_peak_axis_m": float(e_peak[i, j]),
            "diag4_budget_ratio": float(d4[i, j]),
            "worst_normalized_use": float(worst_use[i, j]),
        }

    recommended = most_robust = None
    if common.any():
        # The note's literal criterion: "the cell maximizing diagnostic-4
        # margin subject to passing 1-3".
        recommended = describe(*np.unravel_index(
            int(np.argmin(np.where(common, d4, np.inf))), d4.shape))
        # ...which, when the feasible region is a thin sliver, lands a fraction
        # of a percent from violating rows 1b and 3.  Minimising row 4 is the
        # wrong objective there -- row 4 is the only diagnostic the note does
        # not hard-gate -- so the cell furthest from *every* hard constraint is
        # reported too, and that is the one an operating point should be
        # chosen at.
        most_robust = describe(*np.unravel_index(
            int(np.argmin(np.where(common, worst_use, np.inf))), worst_use.shape))

    return {
        "tau_cap_Nm": cap.tolist(),
        "base_only": base_only,
        "phri2_realized_tau_ratio": phri2_ratio,
        "diag1_anchor_ratio_replay": d1.tolist(),
        "diag1b_anchor_ratio_fallback": d1b.tolist(),
        "diag2_passive": d2_ok,
        "diag3_e_ss_axis_m": e_ss.tolist(),
        "diag3_e_peak_axis_m": e_peak.tolist(),
        "diag3_settled": settled.tolist(),
        "diag4_budget_ratio": d4.tolist(),
        "n_cells": int(d1.size),
        "n_pass_diag1": int(pass1.sum()),
        "n_pass_diag1b": int(pass1b.sum()),
        "n_pass_diag3": int(pass3.sum()),
        "n_pass_diag3_peak": int((e_peak <= bound).sum()),
        "n_unsettled": int((~settled).sum()),
        "common_region_nonempty": bool(common.any()),
        "common_region_size": int(common.sum()),
        "common_region_size_with_beta": int((common & (d4 < beta)).sum()),
        # What the note's own four-row table would have reported: row 1b is the
        # difference between a region that exists and one that is twice as big
        # and partly fictional.
        "region_size_without_diag1b": int((pass1 & pass3).sum()),
        "region_size_gated_on_peak": int((pass1 & pass1b & (e_peak <= bound)).sum()),
        "best_worst_normalized_use": float(worst_use.min()),
        "recommended_K0_D0": recommended,
        "most_robust_K0_D0": most_robust,
    }


def _fallback_worker(args: tuple[float, float]) -> tuple:
    """Closed-loop alpha -> 0 run for one cell (process-pool entry point)."""
    k, d = args
    r = pc.run_fallback_equilibrium(k, d)
    return k, d, r["e_ss_axis"], r["e_peak_axis"], bool(r["settled"]), r["anchor_max_abs"]


def run_scan(
    k_grid: np.ndarray,
    d_grid: np.ndarray,
    beta: float = BETA_DEFAULT,
    workers: int = 4,
) -> dict:
    traj = pc.load_or_generate_trajectory()
    nk, nd = len(k_grid), len(d_grid)

    # --- Diagnostic 3 / 1b: one closed-loop MuJoCo run per cell ------------
    cells = [(float(k), float(d)) for k in k_grid for d in d_grid]
    e_ss = np.zeros((nk, nd))
    e_peak = np.zeros((nk, nd))
    settled = np.zeros((nk, nd), dtype=bool)
    anchor_eq = np.zeros((nk, nd, 7))
    k_index = {float(k): i for i, k in enumerate(k_grid)}
    d_index = {float(d): j for j, d in enumerate(d_grid)}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for k, d, ss, peak, ok, anchor_abs in pool.map(_fallback_worker, cells):
            i, j = k_index[k], d_index[d]
            e_ss[i, j] = ss
            e_peak[i, j] = peak
            settled[i, j] = ok
            anchor_eq[i, j] = anchor_abs

    report: dict = {
        "scenario": {
            "source_trajectory": "phri2 FR3 20 N sustained push (run_fr3_experiments.human_force_at)",
            "push_magnitude_N": pc.PUSH_MAGNITUDE_N,
            "push_axis": pc.PUSH_AXIS.tolist(),
            "duration_s": pc.DURATION_S,
            "workspace_bound_m": pc.WORKSPACE_BOUND_M,
            "servo_rate_Hz": 1000.0,
            "fallback_hold_s": pc.FALLBACK_HOLD_S,
            "fallback_duration_s": pc.FALLBACK_DURATION_S,
            "fallback_averaging_window_s": pc.FALLBACK_AVERAGING_WINDOW_S,
            "trajectory_ticks": traj.n_ticks,
            "beta": beta,
        },
        "grid": {"k_N_per_m": k_grid.tolist(), "d_Ns_per_m": d_grid.tolist()},
        "envelopes": {},
    }

    for envelope in pc.ENVELOPE_NAMES:
        cap = pc.torque_envelope(envelope)
        d1 = np.zeros((nk, nd))
        d4 = np.zeros((nk, nd))
        for i, k in enumerate(k_grid):
            for j, d in enumerate(d_grid):
                cell = pc.replay_diagnostics(traj, float(k), float(d), cap)
                d1[i, j] = cell["anchor_ratio"]
                d4[i, j] = cell["budget_ratio"]
        d1b = (anchor_eq / cap[None, None, :]).max(axis=2)
        report["envelopes"][envelope] = score_envelope(
            k_grid, d_grid, d1, d1b, e_ss, e_peak, settled, d4, beta,
            cap=cap,
            base_only=pc.base_only_anchor_ratio(traj, cap),
            phri2_ratio=float(np.max(np.abs(traj.tau_realized) / cap[None, :])),
        )

    return report


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def _heatmap(ax, data, k_grid, d_grid, title, cmap, vmin=None, vmax=None,
             contour_at=None, contour_label=""):
    mesh = ax.pcolormesh(d_grid, k_grid, data, cmap=cmap, vmin=vmin, vmax=vmax,
                         shading="nearest")
    ax.set_yscale("log")
    ax.set_xlabel(r"$d$  [N$\cdot$s/m]")
    ax.set_ylabel(r"$k$  [N/m]")
    ax.set_title(title, fontsize=9)
    if contour_at is not None and np.nanmin(data) < contour_at < np.nanmax(data):
        cs = ax.contour(d_grid, k_grid, data, levels=[contour_at],
                        colors="k", linewidths=1.4, linestyles="--")
        if contour_label:
            ax.clabel(cs, fmt={contour_at: contour_label}, fontsize=7)
    return mesh


def make_figures(report: dict, outdir: Path) -> list[Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    k_grid = np.array(report["grid"]["k_N_per_m"])
    d_grid = np.array(report["grid"]["d_Ns_per_m"])
    bound = report["scenario"]["workspace_bound_m"]
    beta = report["scenario"]["beta"]
    written: list[Path] = []

    for envelope, block in report["envelopes"].items():
        d1 = np.array(block["diag1_anchor_ratio_replay"])
        d1b = np.array(block["diag1b_anchor_ratio_fallback"])
        d3 = np.array(block["diag3_e_ss_axis_m"])
        d4 = np.array(block["diag4_budget_ratio"])

        panels = [
            ("diag1", d1, r"(1) anchor ratio on replay  $\max_t\|\tau_b+J_v^\top F_{nom}\|_\infty/\bar\tau$",
             "RdYlGn_r", 1.0, "= 1"),
            ("diag1b", d1b, r"(1b) peak anchor ratio over the $\alpha\to0$ fallback run",
             "RdYlGn_r", 1.0, "= 1"),
            ("diag3", d3, r"(3) steady-state $|e_z|$ at $\alpha\to0$  [m]",
             "RdYlGn_r", bound, f"= {bound} m"),
            ("diag4", d4, r"(4) $F_{nom}$ budget fraction  $\max_t\|J_v^\top F_{nom}\|_\infty/\bar\tau$",
             "viridis", beta, rf"$\beta$ = {beta}"),
        ]
        for tag, data, title, cmap, level, label in panels:
            fig, ax = plt.subplots(figsize=(4.4, 3.6))
            mesh = _heatmap(ax, data, k_grid, d_grid, title, cmap,
                            contour_at=level, contour_label=label)
            fig.colorbar(mesh, ax=ax)
            fig.suptitle(f"envelope: {envelope}", fontsize=8, y=0.99)
            fig.tight_layout()
            path = outdir / f"pir_knot_scan_{tag}_{envelope}.png"
            fig.savefig(path, dpi=170)
            plt.close(fig)
            written.append(path)

    # --- composite overlay ------------------------------------------------
    # Two rows: the full swept range, and a zoom on the band that carries the
    # feasible region.  The zoom is not decoration -- the region is ~40 N/m
    # tall inside a 10-1200 N/m sweep, which is exactly why a coarse grid
    # reports it empty, and at full scale it is a single invisible line.
    envelopes = list(report["envelopes"].items())
    fig, axes = plt.subplots(2, len(envelopes), figsize=(5.0 * len(envelopes), 7.4),
                             squeeze=False)
    zoom_k = (280.0, 560.0)
    handles = None
    for col, (envelope, block) in enumerate(envelopes):
        d1 = np.array(block["diag1_anchor_ratio_replay"])
        d1b = np.array(block["diag1b_anchor_ratio_fallback"])
        d3 = np.array(block["diag3_e_ss_axis_m"])
        d4 = np.array(block["diag4_budget_ratio"])
        common = (d1 < 1.0) & (d1b < 1.0) & (d3 <= bound)
        shade = np.where(common, np.clip(d4, 0.0, 1.0), np.nan)
        best = block.get("most_robust_K0_D0")

        for row, ax in enumerate(axes[:, col]):
            # Failing cells: a visible light grey, so "infeasible" reads as a
            # filled field rather than as blank paper.
            ax.pcolormesh(d_grid, k_grid, np.where(common, np.nan, 1.0),
                          cmap="Greys", vmin=0.0, vmax=3.0, shading="nearest")
            mesh = ax.pcolormesh(d_grid, k_grid, shade, cmap="viridis",
                                 vmin=0.0, vmax=1.0, shading="nearest")
            drawn = []
            for data, level, color, label in (
                (d1, 1.0, "tab:red", r"(1) replay anchor $= 1$"),
                (d1b, 1.0, "tab:purple", r"(1b) fallback anchor $= 1$"),
                (d3, bound, "tab:blue", rf"(3) $|e_z| = {bound}$ m"),
            ):
                if np.nanmin(data) < level < np.nanmax(data):
                    cs = ax.contour(d_grid, k_grid, data, levels=[level],
                                    colors=[color], linewidths=1.6)
                    drawn.append((cs.legend_elements()[0][0], label))
            if best is not None:
                ax.plot(best["d_Ns_per_m"], best["k_N_per_m"], marker="*",
                        markersize=15, color="white", markeredgecolor="black",
                        markeredgewidth=0.8, linestyle="none",
                        label="most robust cell")
            ax.set_xlabel(r"$d$  [N$\cdot$s/m]")
            ax.set_ylabel(r"$k$  [N/m]")
            if row == 0:
                ax.set_yscale("log")
                n = int(common.sum())
                ax.set_title(
                    f"{envelope}\ncommon feasible region: "
                    + (f"{n} / {common.size} cells" if n else "EMPTY"),
                    fontsize=10,
                )
            else:
                ax.set_ylim(*zoom_k)
                ax.set_title(f"zoom: $k \\in$ [{zoom_k[0]:.0f}, {zoom_k[1]:.0f}] N/m",
                             fontsize=9)
            # Keep the richest legend: the rho panel has no (1b) contour to
            # draw, so taking the first panel's would silently drop a curve.
            if handles is None or len(drawn) > len(handles):
                handles = drawn
            if int(common.sum()):
                fig.colorbar(mesh, ax=ax, label=r"diagnostic 4 ($F_{nom}$ budget)")

    if handles:
        fig.legend([h for h, _ in handles], [t for _, t in handles],
                   loc="lower center", ncol=3, fontsize=9, frameon=False)
    fig.suptitle(
        "PIR passive-nominal design region: cells passing (1), (1b) and (3), "
        "shaded by (4).\nGrey = infeasible.",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0.055, 1, 1))
    path = outdir / "pir_knot_scan_overlay.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    written.append(path)
    return written


def rescore(path: Path, beta: float = BETA_DEFAULT) -> dict:
    """Re-derive every verdict in an existing report from its stored maps."""
    report = json.loads(path.read_text())
    k_grid = np.array(report["grid"]["k_N_per_m"])
    d_grid = np.array(report["grid"]["d_Ns_per_m"])
    report["scenario"]["beta"] = beta
    for envelope, block in report["envelopes"].items():
        report["envelopes"][envelope] = score_envelope(
            k_grid, d_grid,
            np.array(block["diag1_anchor_ratio_replay"]),
            np.array(block["diag1b_anchor_ratio_fallback"]),
            np.array(block["diag3_e_ss_axis_m"]),
            np.array(block["diag3_e_peak_axis_m"]),
            np.array(block["diag3_settled"], dtype=bool),
            np.array(block["diag4_budget_ratio"]),
            beta,
            cap=np.array(block["tau_cap_Nm"]),
            base_only=block["base_only"],
            phri2_ratio=block["phri2_realized_tau_ratio"],
        )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="coarse grid (smoke test)")
    parser.add_argument("--rescore", action="store_true",
                        help="re-derive verdicts and figures from an existing "
                             "pir_knot_scan.json instead of re-measuring")
    parser.add_argument("--beta", type=float, default=BETA_DEFAULT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    if args.rescore:
        report = rescore(args.outdir / "pir_knot_scan.json", beta=args.beta)
        k_grid = np.array(report["grid"]["k_N_per_m"])
        d_grid = np.array(report["grid"]["d_Ns_per_m"])
    else:
        k_grid = QUICK_K if args.quick else K_GRID
        d_grid = QUICK_D if args.quick else D_GRID
        report = run_scan(k_grid, d_grid, beta=args.beta, workers=args.workers)
    (args.outdir / "pir_knot_scan.json").write_text(json.dumps(report, indent=2))
    figures = make_figures(report, args.outdir)

    print(f"grid: {len(k_grid)} k x {len(d_grid)} d = {len(k_grid) * len(d_grid)} cells")
    for envelope, block in report["envelopes"].items():
        cap = block["tau_cap_Nm"]
        print(f"\n[{envelope}]  cap = {np.round(cap, 2).tolist()}")
        print(f"  tau_base alone already uses {block['base_only']['base_only_anchor_ratio']:.3f} "
              f"of the cap (worst joint {block['base_only']['base_only_worst_joint'] + 1})")
        print(f"  phri2's own realized torque peaks at {block['phri2_realized_tau_ratio']:.3f} of this cap")
        print(f"  pass (1) replay anchor      : {block['n_pass_diag1']:4d} / {block['n_cells']}")
        print(f"  pass (1b) alpha->0 anchor   : {block['n_pass_diag1b']:4d} / {block['n_cells']}")
        print(f"  pass (3) 0.06 m fallback    : {block['n_pass_diag3']:4d} / {block['n_cells']} "
              f"(on peak instead of steady state: {block['n_pass_diag3_peak']}; "
              f"unsettled cells: {block['n_unsettled']})")
        print(f"  COMMON REGION NONEMPTY      : {block['common_region_nonempty']} "
              f"({block['common_region_size']} cells; "
              f"{block['region_size_without_diag1b']} if row 1b were omitted, "
              f"{block['region_size_gated_on_peak']} if row 3 gated on peak)")
        for label, key in (("recommended (min row 4)", "recommended_K0_D0"),
                           ("most robust (max margin)", "most_robust_K0_D0")):
            cell = block.get(key)
            if cell:
                print(f"  {label:26s}: k = {cell['k_N_per_m']:.1f} N/m, d = {cell['d_Ns_per_m']:.2f} N.s/m "
                      f"| (1) {cell['diag1_anchor_ratio_replay']:.3f}  (1b) {cell['diag1b_anchor_ratio_fallback']:.3f}  "
                      f"(3) {cell['diag3_e_ss_axis_m'] * 1e3:.1f} mm  (4) {cell['diag4_budget_ratio']:.3f}  "
                      f"| worst use {cell['worst_normalized_use']:.3f}")
    print("\nwrote:")
    for path in figures:
        print(f"  {path}")
    print(f"  {args.outdir / 'pir_knot_scan.json'}")


if __name__ == "__main__":
    main()
