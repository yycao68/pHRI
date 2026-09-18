#!/usr/bin/env python3
"""Root-cause the three bad findings of Section 7 by sweeping K0.

Section 7 reported three problems at the certified operating point:

  (i)   the nominal is 84% of the command and leaves the residual 2.4% of
        joint 4's torque cap;
  (ii)  the E0 authorization-vs-tracking trade is flat -- the passivity axis is
        cheap because it is weak;
  (iii) Merged Lemma 1's precondition fails at 4x the source disturbance, and
        the fast layer has no authority over the nominal that causes it.

The hypothesis is that all three are the *same* problem: diagnostic 3 forces
K0 above the desired impedance's own stiffness, and everything else follows.
Diagnostic 3 requires the alpha -> 0 fallback to hold the whole 20 N push
inside 0.06 m, i.e. K0 >= |F_h| / 0.06 = 333 N/m, while the behaviour the
controller is supposed to render has K_d = 200 N/m.  If that is the cause,
every symptom should be monotone in K0 and should improve as K0 falls -- with
diagnostic 3, and only diagnostic 3, getting worse.

This sweeps K0 at a fixed damping ratio (so D0 is not a free second variable)
and measures all of it.  It does not yet fix anything; ``pir_fixes.py`` does
that.  Run::

    python3 pir_rootcause.py
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
from pir_controller import PIRConfig
from run_pir_closed_loop import run_variant

#: Task-space inertia on the push axis at the nominal pose, used only to hold
#: the damping ratio fixed across the sweep so K0 is the single variable.
LAMBDA_ZZ = 4.557
ZETA = 29.07 / (2.0 * np.sqrt(380.0 * LAMBDA_ZZ))  # the certified cell's zeta

K0_GRID = (100.0, 150.0, 200.0, 250.0, 300.0, 380.0, 460.0, 600.0)
#: How far the disturbance is scaled while looking for the point at which the
#: anchor stops fitting.  1.0 is impedance_residual's own amplitude.
PROBE_SCALES = (1.0, 2.0, 4.0, 6.0, 8.0, 12.0)


def damping_for(k0: float) -> float:
    return float(2.0 * ZETA * np.sqrt(k0 * LAMBDA_ZZ))


def _probe(args: tuple) -> dict:
    k0, = args
    d0 = damping_for(k0)

    # (a) The constraint that pushes K0 up: the alpha -> 0 fallback.
    fallback = pc.run_fallback_equilibrium(k0, d0)

    # (b) Symptoms (i) and the anchor, on the merged scenario at nominal stress.
    base = run_variant("pir", k0, d0, "derated_joint4", scenario="merged")["summary"]

    # (c) Symptom (iii): how far the disturbance scales before the precondition
    #     stops holding.
    largest_ok = 0.0
    first_fail = None
    for scale in PROBE_SCALES:
        s = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                        disturbance_scale=scale)["summary"]
        if s["lemma1_precondition_holds"]:
            largest_ok = scale
        elif first_fail is None:
            first_fail = scale

    # (d) Symptom (ii): the E0 trade, as the spread between a nearly empty tank
    #     and a full one.  A flat pair means the passivity axis buys nothing.
    empty = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                        tank_initial=0.021, disturbance_scale=4.0)["summary"]
    full = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                       tank_initial=0.080, disturbance_scale=4.0)["summary"]

    return {
        "K0": k0,
        "D0": d0,
        "diag3_fallback_e_ss_m": fallback["e_ss_axis"],
        "diag3_passes": bool(fallback["e_ss_axis"] <= pc.WORKSPACE_BOUND_M),
        "anchor_ratio": base["diag1_anchor_ratio_closed_loop"],
        "anchor_headroom": base["anchor_headroom"],
        "residual_share_of_command": base["residual_share_of_command"],
        "f_nom_rms_N": base["f_nom_rms_N"],
        "f_r_rms_N": base["f_r_rms_N"],
        "largest_safe_disturbance_scale": largest_ok,
        "first_failing_scale": first_fail,
        "e0_trade_rms_spread": abs(empty["rms_realization_residual"]
                                   - full["rms_realization_residual"]),
        "e0_trade_auth_spread": abs(empty["authorization_active_fraction"]
                                    - full["authorization_active_fraction"]),
        "rms_realization_residual": base["rms_realization_residual"],
        "max_abs_e_axis_m": base["max_abs_e_axis_m"],
    }


def run(workers: int = 4) -> dict:
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(_probe, [(k,) for k in K0_GRID]))
    rows.sort(key=lambda r: r["K0"])
    return {
        "hypothesis": (
            "diagnostic 3 forces K0 above K_d = 200 N/m, and symptoms (i), (ii) "
            "and (iii) are all monotone consequences of that"
        ),
        "desired_impedance_K_d": pc.ImpedanceReference3D().stiffness,
        "diag3_required_K0": pc.PUSH_MAGNITUDE_N / pc.WORKSPACE_BOUND_M,
        "zeta": ZETA,
        "rows": rows,
    }


def make_figure(report: dict, outdir: Path) -> Path:
    rows = report["rows"]
    k = np.array([r["K0"] for r in rows])
    kd = report["desired_impedance_K_d"]
    fig, axes = plt.subplots(1, 4, figsize=(14.0, 3.5))

    axes[0].plot(k, 1e3 * np.array([r["diag3_fallback_e_ss_m"] for r in rows]), "o-")
    axes[0].axhline(1e3 * pc.WORKSPACE_BOUND_M, color="k", ls="--", lw=1)
    axes[0].set_ylabel(r"fallback $|e_z|$ [mm]")
    axes[0].set_title("(3) the constraint pushing $K_0$ UP", fontsize=9)

    axes[1].plot(k, 100 * np.array([r["anchor_headroom"] for r in rows]), "o-")
    axes[1].set_ylabel("anchor headroom [% of cap]")
    axes[1].set_title("(i) headroom left for $F_r$", fontsize=9)

    axes[2].plot(k, 100 * np.array([r["residual_share_of_command"] for r in rows]), "o-")
    axes[2].set_ylabel("residual share of command [%]")
    axes[2].set_title("(i) how much is predictive", fontsize=9)

    axes[3].plot(k, [r["largest_safe_disturbance_scale"] for r in rows], "o-")
    axes[3].set_ylabel("largest safe disturbance scale")
    axes[3].set_title("(iii) precondition robustness", fontsize=9)

    for ax in axes:
        ax.axvline(kd, color="tab:red", ls=":", lw=1.4)
        ax.axvline(report["diag3_required_K0"], color="tab:blue", ls=":", lw=1.4)
        ax.set_xlabel(r"$K_0$ [N/m]")
        ax.grid(alpha=0.25)
    axes[0].annotate(rf"$K_d$ = {kd:.0f}", xy=(kd, axes[0].get_ylim()[1]),
                     xytext=(-4, -12), textcoords="offset points", fontsize=8,
                     color="tab:red", ha="right")
    axes[0].annotate(rf"(3) needs {report['diag3_required_K0']:.0f}",
                     xy=(report["diag3_required_K0"], axes[0].get_ylim()[1]),
                     xytext=(4, -12), textcoords="offset points", fontsize=8,
                     color="tab:blue")
    fig.suptitle("Root cause: every symptom is monotone in $K_0$, and only "
                 "diagnostic (3) wants $K_0$ large", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / "pir_rootcause.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    report = run(workers=args.workers)
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_rootcause.json").write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir)

    print(f"desired impedance K_d = {report['desired_impedance_K_d']:.0f} N/m; "
          f"diagnostic (3) needs K0 >= {report['diag3_required_K0']:.0f} N/m; "
          f"zeta held at {report['zeta']:.3f}\n")
    print(f"{'K0':>6} {'D0':>6} {'(3)mm':>7} {'(3)ok':>6} {'anchor':>7} "
          f"{'headrm':>7} {'resid%':>7} {'safe x':>7} {'E0 dRMS':>8} {'RMS':>7}")
    for r in report["rows"]:
        print(f"{r['K0']:>6.0f} {r['D0']:>6.1f} "
              f"{1e3 * r['diag3_fallback_e_ss_m']:>7.1f} "
              f"{'yes' if r['diag3_passes'] else 'NO':>6} "
              f"{r['anchor_ratio']:>7.4f} "
              f"{100 * r['anchor_headroom']:>6.1f}% "
              f"{100 * r['residual_share_of_command']:>6.1f}% "
              f"{r['largest_safe_disturbance_scale']:>7.0f} "
              f"{r['e0_trade_rms_spread']:>8.4f} "
              f"{r['rms_realization_residual']:>7.3f}")
    print(f"\nwrote {args.outdir / 'pir_rootcause.json'}\nwrote {figure}")


if __name__ == "__main__":
    main()
