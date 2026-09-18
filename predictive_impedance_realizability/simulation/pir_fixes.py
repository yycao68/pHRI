#!/usr/bin/env python3
"""Candidate fixes for the three bad findings of Section 7, measured side by side.

``pir_rootcause.py`` establishes the cause: diagnostic 3 forces
``K0 >= |F_h| / 0.06 = 333`` N/m, which is above the desired impedance's own
``K_d = 200`` N/m, and all three symptoms are monotone consequences.  This
script evaluates what can be done about it.  Candidates:

``certified``        the Task 1 operating point, K0 = 380.  The baseline.
``soft_nominal``     K0 = K_d = 200, D0 = D_d = 28.  Rejects diagnostic 3's
                     premise rather than its arithmetic: the requirement asks
                     the passivity FLOOR to satisfy a workspace bound that the
                     desired BEHAVIOUR does not satisfy either (phri2's own
                     K_d = 200 gives 0.10 m static displacement against a
                     0.06 m bound, and phri2 meets the bound through its
                     predictive layer, not its impedance).  Under this reading
                     the fallback's job is to be passive and bounded, not to
                     meet the performance spec.
``anisotropic``      K0 stiff on the push axis, soft off it.  Section 13.2 of
                     the plan defers this as "a later refinement".  Included
                     because it is the obvious first lever, and because it is
                     worth knowing whether it actually helps here.
``nominal_auth``     the structural fix for symptom (iii): let the servo scale
                     the NOMINAL too, so an infeasible anchor is something it
                     can act on.  Restores an unconditional torque guarantee
                     whenever tau_base alone fits.
``nominal_auth_mono``  the same, with alpha_nom made monotone non-increasing.
                     Re-stiffening is the direction that charges the tank, so
                     forbidding it removes the charge entirely -- at the cost
                     of a floor that never recovers its stiffness within an
                     episode, which a deployed system would have to reset per
                     contact.
``soft_plus_auth``   soft_nominal and nominal_auth together.
``soft_plus_mono``   soft_nominal and nominal_auth_mono together.

Each is scored on the three symptoms plus the constraint that caused them::

    (0) diagnostic 3   the alpha -> 0 fallback displacement
    (i) headroom       anchor headroom, and the residual's share of the command
    (ii) E0 trade      does the passivity axis buy anything
    (iii) robustness   largest disturbance scale with the torque envelope intact
                       AND the tank floor intact

Run::

    python3 pir_fixes.py
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
from run_pir_closed_loop import run_variant

K_D = 200.0  # ImpedanceReference3D.stiffness
D_D = 28.0  # ImpedanceReference3D.damping
PROBE_SCALES = (1.0, 2.0, 4.0, 6.0, 8.0, 12.0, 16.0)

#: name -> (variant, k0, d0, overrides)
CANDIDATES: dict[str, tuple] = {
    "certified": ("pir", 380.0, 29.07, {}),
    "soft_nominal": ("pir", K_D, D_D, {}),
    "anisotropic": ("pir", (60.0, 60.0, 380.0), (8.0, 8.0, 29.07), {}),
    "nominal_auth": ("pir_nominal_auth", 380.0, 29.07, {}),
    "nominal_auth_mono": ("pir_nominal_auth", 380.0, 29.07, {"nominal_reauth_rate": 0.0}),
    "soft_plus_auth": ("pir_nominal_auth", K_D, D_D, {}),
    "soft_plus_mono": ("pir_nominal_auth", K_D, D_D, {"nominal_reauth_rate": 0.0}),
}


def _evaluate(name: str) -> dict:
    variant, k0, d0, overrides = CANDIDATES[name]

    # (0) The constraint that caused everything: the alpha -> 0 fallback.
    # Uses the push-axis gain, which is what the fallback equilibrium sees.
    k_axis = float(np.broadcast_to(np.asarray(k0, float), (3,))[pc.PUSH_AXIS_INDEX])
    d_axis = float(np.broadcast_to(np.asarray(d0, float), (3,))[pc.PUSH_AXIS_INDEX])
    fallback = pc.run_fallback_equilibrium(k_axis, d_axis)

    base = run_variant(variant, k0, d0, "derated_joint4", scenario="merged",
                       overrides=overrides)["summary"]

    # (iii) How far the disturbance scales before EITHER guarantee is lost.
    torque_ok_to = 0.0
    both_ok_to = 0.0
    probe_rows = []
    for scale in PROBE_SCALES:
        s = run_variant(variant, k0, d0, "derated_joint4", scenario="merged",
                        disturbance_scale=scale, overrides=overrides)["summary"]
        torque_ok = s["lemma1_conclusion_holds"]
        both_ok = torque_ok and s["tank_floor_holds"]
        if torque_ok:
            torque_ok_to = scale
        if both_ok:
            both_ok_to = scale
        probe_rows.append({
            "scale": scale,
            "max_tau_ratio": s["lemma1_conclusion_max_tau_ratio"],
            "torque_envelope_holds": torque_ok,
            "tank_min": s["tank_min"],
            "tank_floor_holds": s["tank_floor_holds"],
            "rms_realization_residual": s["rms_realization_residual"],
            "alpha_nom_min": s["alpha_nom_min"],
            "max_abs_e_axis_m": s["max_abs_e_axis_m"],
        })

    # (ii) Does the passivity axis buy anything: empty tank vs full, under load.
    empty = run_variant(variant, k0, d0, "derated_joint4", scenario="merged",
                        tank_initial=0.021, disturbance_scale=4.0,
                        overrides=overrides)["summary"]
    full = run_variant(variant, k0, d0, "derated_joint4", scenario="merged",
                       tank_initial=0.080, disturbance_scale=4.0,
                       overrides=overrides)["summary"]

    return {
        "name": name,
        "variant": variant,
        "K0": base["K0_vector"],
        "D0": base["D0_vector"],
        "nominal_authorization": base["nominal_authorization"],
        "nominal_reauth_rate": base["nominal_reauth_rate"],
        "diag3_fallback_e_ss_m": fallback["e_ss_axis"],
        "diag3_passes": bool(fallback["e_ss_axis"] <= pc.WORKSPACE_BOUND_M),
        "anchor_ratio": base["diag1_anchor_ratio_closed_loop"],
        "anchor_headroom": base["anchor_headroom"],
        "residual_share_of_command": base["residual_share_of_command"],
        "rms_realization_residual": base["rms_realization_residual"],
        "max_abs_e_axis_m": base["max_abs_e_axis_m"],
        "largest_scale_torque_ok": torque_ok_to,
        "largest_scale_both_ok": both_ok_to,
        "e0_trade_rms_spread": abs(empty["rms_realization_residual"]
                                   - full["rms_realization_residual"]),
        # How deep the worst tank breach goes, which the pass/fail scale alone
        # hides: a candidate that halves the deficit is better even when it
        # still fails at the same scale.
        "worst_tank_deficit_J": float(min(
            0.0, min(r["tank_min"] for r in probe_rows) - 0.02)),
        "alpha_nom_min_over_probe": float(min(r["alpha_nom_min"] for r in probe_rows)),
        "probe": probe_rows,
    }


def run(workers: int = 4) -> dict:
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_evaluate, list(CANDIDATES)))
    order = list(CANDIDATES)
    rows.sort(key=lambda r: order.index(r["name"]))
    return {
        "cause": (
            "diagnostic 3 requires K0 >= |F_h| / 0.06 = 333 N/m, above the "
            "desired impedance's own K_d = 200 N/m; symptoms (i), (ii) and "
            "(iii) are monotone consequences (see pir_rootcause.json)"
        ),
        "workspace_bound_m": pc.WORKSPACE_BOUND_M,
        "K_d": K_D,
        "candidates": rows,
    }


def make_figure(report: dict, outdir: Path) -> Path:
    rows = report["candidates"]
    names = [r["name"] for r in rows]
    x = np.arange(len(names))
    bound = report["workspace_bound_m"]
    fig, axes = plt.subplots(1, 5, figsize=(18.0, 4.0))

    axes[0].bar(x, [1e3 * r["diag3_fallback_e_ss_m"] for r in rows],
                color=["tab:green" if r["diag3_passes"] else "tab:red" for r in rows])
    axes[0].axhline(1e3 * bound, color="k", ls="--", lw=1)
    axes[0].set_ylabel(r"fallback $|e_z|$ [mm]")
    axes[0].set_title("(0) diagnostic 3\ngreen = passes", fontsize=9)

    axes[1].bar(x, [100 * r["anchor_headroom"] for r in rows], color="tab:blue")
    axes[1].set_ylabel("anchor headroom [% of cap]")
    axes[1].set_title("(i) headroom for $F_r$", fontsize=9)

    axes[2].bar(x - 0.2, [r["largest_scale_torque_ok"] for r in rows], 0.4,
                label="torque envelope", color="tab:purple")
    axes[2].bar(x + 0.2, [r["largest_scale_both_ok"] for r in rows], 0.4,
                label="+ tank floor", color="tab:orange")
    axes[2].set_ylabel("largest safe disturbance scale")
    axes[2].set_title("(iii) how far each guarantee survives", fontsize=9)
    axes[2].legend(fontsize=7)

    axes[3].bar(x, [r["e0_trade_rms_spread"] for r in rows], color="tab:brown")
    axes[3].set_ylabel(r"$E_0$ trade: RMS spread [m/s$^2$]")
    axes[3].set_title("(ii) does the passivity axis\nbuy anything", fontsize=9)

    # The cost that the four panels above hide: a softer nominal keeps the
    # workspace box under nominal load just as well, and much worse under a
    # large one.  Without this panel the soft candidates look free.
    def excursion(r: dict, scale: float) -> float:
        hit = [p for p in r["probe"] if p["scale"] == scale]
        return 1e3 * hit[0]["max_abs_e_axis_m"] if hit else np.nan

    axes[4].bar(x - 0.2, [excursion(r, 1.0) for r in rows], 0.4,
                label="disturbance x1", color="tab:cyan")
    axes[4].bar(x + 0.2, [excursion(r, 12.0) for r in rows], 0.4,
                label="disturbance x12", color="tab:red")
    axes[4].axhline(1e3 * bound, color="k", ls="--", lw=1)
    axes[4].set_ylabel(r"closed-loop max $|e_z|$ [mm]")
    axes[4].set_title("the cost the others hide:\nworkspace under load", fontsize=9)
    axes[4].legend(fontsize=7)

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
        ax.grid(alpha=0.25, axis="y")
    fig.suptitle("Candidate fixes for the three Section 7 findings", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_fixes.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--refigure", action="store_true",
                        help="redraw from an existing pir_fixes.json")
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    if args.refigure:
        report = json.loads((args.outdir / "pir_fixes.json").read_text())
    else:
        report = run(workers=args.workers)
        (args.outdir / "pir_fixes.json").write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir)

    print(f"{'candidate':<19} {'(0)mm':>6} {'(0)ok':>6} {'headrm':>7} "
          f"{'resid%':>7} {'tau ok':>7} {'both ok':>8} {'worstJ':>8} {'aNmin':>6} "
          f"{'E0dRMS':>8} {'RMS':>6}")
    for r in report["candidates"]:
        print(f"{r['name']:<19} {1e3 * r['diag3_fallback_e_ss_m']:>6.0f} "
              f"{'yes' if r['diag3_passes'] else 'NO':>6} "
              f"{100 * r['anchor_headroom']:>6.1f}% "
              f"{100 * r['residual_share_of_command']:>6.1f}% "
              f"{r['largest_scale_torque_ok']:>7.0f} {r['largest_scale_both_ok']:>8.0f} "
              f"{r['worst_tank_deficit_J']:>8.3f} "
              f"{r['alpha_nom_min_over_probe']:>6.3f} "
              f"{r['e0_trade_rms_spread']:>8.4f} {r['rms_realization_residual']:>6.3f}")
    print(f"\nwrote {args.outdir / 'pir_fixes.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
