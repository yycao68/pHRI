#!/usr/bin/env python3
"""Task 3: the authorization-vs-tracking curve, with BOTH tightenings active.

impedance_residual measured this curve on its own nominal: sweeping the tank's
initial charge ``E0`` took authorization from 68.9% of ticks active down to 0%,
and tracking RMS from 21.49 mm to 15.98 mm.  The synthesis note predicts the
curve gets **steeper** after the merge, because phri2's horizon-wide torque
tightening and the tank's energy tightening compound in series
(``delta_tau (+) m_safe``).  This re-measures it on the merged controller.

It also answers a question the merged closed-loop run raised on its own: at the
only ``(K0, D0)`` the Task 1 gate certifies, the nominal is 83% of the command
and leaves the residual 2.4% of joint 4's cap, so neither axis activates under
the source scenarios.  Sweeping ``E0`` down and the disturbance up is how you
find out whether the passivity axis works at all here, or is merely inert.

Two sweeps, both on the merged (push + oscillatory disturbance) scenario:

* ``E0``                 -- the tank's initial charge, toward its floor.
* ``disturbance_scale``  -- impedance_residual's own disturbance amplitude knob.

Three variants per point, which is the contrast that matters:

* ``pir``               1 kHz re-authorization (the merged rule)
* ``pir_manager_guard`` alpha_E computed once per 20 ms and held (B4)
* ``pir_no_tank``       no passivity axis at all

Run::

    python3 pir_e0_sweep.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pir_common as pc
from pir_controller import PIRConfig
from run_pir_closed_loop import run_variant

SWEEP_VARIANTS = ("pir", "pir_manager_guard", "pir_no_tank")

#: E_min = 0.02 J, so 0.021 starts the tank essentially empty: any
#: energy-injecting residual must then be authorized against the nominal's
#: instantaneous dissipation alone.
E0_GRID = (0.021, 0.023, 0.026, 0.030, 0.040, 0.060, 0.080)
DISTURBANCE_GRID = (1.0, 2.0, 4.0, 8.0, 12.0)


def _point(variant: str, k0: float, d0: float, envelope: str,
           tank_initial: float, disturbance_scale: float,
           pose=None, push_axis=None) -> dict:
    out = run_variant(variant, k0, d0, envelope, scenario="merged",
                      tank_initial=tank_initial, disturbance_scale=disturbance_scale,
                      pose=pose, push_axis=push_axis)
    s = out["summary"]
    return {
        "variant": variant,
        "E0": tank_initial,
        "disturbance_scale": disturbance_scale,
        "authorization_active_fraction": s["authorization_active_fraction"],
        "alpha_E_min": s["alpha_E_min"],
        "alpha_tau_min": s["alpha_tau_min"],
        "tank_min": s["tank_min"],
        "tank_floor_holds": s["tank_floor_holds"],
        "tank_floor_breach_ticks": s["tank_floor_breach_ticks"],
        "rms_realization_residual": s["rms_realization_residual"],
        "max_abs_e_axis_m": s["max_abs_e_axis_m"],
        "r_auth_rms": s["r_auth_rms"],
        "lemma1_precondition_holds": s["lemma1_precondition_holds"],
        "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
        "lemma1_conclusion_max_tau_ratio": s["lemma1_conclusion_max_tau_ratio"],
        "residual_share_of_command": s["residual_share_of_command"],
    }


def run_sweeps(k0: float, d0: float, envelope: str, pose=None, push_axis=None) -> dict:
    default_e0 = PIRConfig().tank_initial
    e0_rows = [_point(v, k0, d0, envelope, e0, 1.0, pose, push_axis)
               for e0 in E0_GRID for v in SWEEP_VARIANTS]
    dist_rows = [_point(v, k0, d0, envelope, default_e0, scale, pose, push_axis)
                 for scale in DISTURBANCE_GRID for v in SWEEP_VARIANTS]
    return {
        "operating_point": {"K0": k0, "D0": d0, "envelope": envelope,
                            "scenario": "merged",
                            "pose": pc.nominal_pose(pose).tolist(),
                            "pose_tag": pc.scenario_tag(pose, push_axis)},
        "tank_floor": PIRConfig().tank_minimum,
        "e0_sweep": e0_rows,
        "disturbance_sweep": dist_rows,
    }


def _series(rows: list[dict], variant: str, x: str, y: str) -> tuple[np.ndarray, np.ndarray]:
    sel = [r for r in rows if r["variant"] == variant]
    sel.sort(key=lambda r: r[x])
    return np.array([r[x] for r in sel]), np.array([r[y] for r in sel])


def make_figure(report: dict, outdir: Path, suffix: str = "") -> Path:
    colors = {"pir": "tab:blue", "pir_manager_guard": "tab:orange",
              "pir_no_tank": "tab:green"}
    floor = report["tank_floor"]
    fig, axes = plt.subplots(2, 3, figsize=(12.0, 6.6))
    panels = (
        ("e0_sweep", "E0", r"tank initial charge $E_0$  [J]"),
        ("disturbance_sweep", "disturbance_scale", "disturbance scale"),
    )
    for row, (key, xkey, xlabel) in enumerate(panels):
        rows = report[key]
        spread = 0.0
        for variant in SWEEP_VARIANTS:
            c = colors[variant]
            x, active = _series(rows, variant, xkey, "authorization_active_fraction")
            axes[row, 0].plot(x, 100 * active, "o-", color=c, label=variant, ms=4)
            _, tank = _series(rows, variant, xkey, "tank_min")
            axes[row, 1].plot(x, tank, "o-", color=c, ms=4)
            _, rms = _series(rows, variant, xkey, "rms_realization_residual")
            axes[row, 2].plot(x, rms, "o-", color=c, ms=4)
            spread = max(spread, float(rms.max() / rms.min() - 1.0))
        axes[row, 1].axhline(floor, color="k", ls="--", lw=1.0)
        # Matplotlib autoscales this panel to its own range, which can make a
        # fraction of a percent look like a trend.  Say the actual spread.
        axes[row, 2].annotate(f"total spread: {100 * spread:.2f}%",
                              xy=(0.03, 0.92), xycoords="axes fraction", fontsize=8,
                              bbox=dict(boxstyle="round,pad=0.25", fc="white",
                                        ec="0.7", alpha=0.85))
        axes[row, 0].set_ylabel("authorization active [% of ticks]")
        axes[row, 1].set_ylabel(r"min tank $E$  [J]")
        axes[row, 2].set_ylabel(r"RMS realization residual  [m/s$^2$]")
        for col in range(3):
            axes[row, col].set_xlabel(xlabel)
            axes[row, col].grid(alpha=0.25)
        if key == "e0_sweep":
            for col in range(3):
                axes[row, col].set_xscale("log")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set_title(rf"dashed: floor $E_{{\min}} = {floor}$ J", fontsize=8)
    fig.suptitle(
        "PIR: authorization vs tracking with both tightenings active "
        f"($K_0$ = {report['operating_point']['K0']} N/m, "
        f"$D_0$ = {report['operating_point']['D0']} N$\\cdot$s/m, "
        f"envelope {report['operating_point']['envelope']})",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / f"pir_e0_sweep{suffix}.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--k0", type=float, default=380.0)
    parser.add_argument("--d0", type=float, default=29.07)
    parser.add_argument("--envelope", default="derated_joint4", choices=pc.ENVELOPE_NAMES)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--refigure", action="store_true",
                        help="redraw from an existing pir_e0_sweep.json")
    parser.add_argument("--pose", type=float, nargs=3, metavar=("Q2", "Q4", "Q6"))
    parser.add_argument("--push-axis", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--tag", default="")
    args = parser.parse_args()

    pose = None
    if args.pose is not None:
        pose = pc.Q_NEUTRAL.copy()
        pose[1], pose[3], pose[5] = args.pose
    push_axis = np.array(args.push_axis) if args.push_axis else None
    suffix = f"_{args.tag}" if args.tag else ""

    args.outdir.mkdir(parents=True, exist_ok=True)
    path = args.outdir / f"pir_e0_sweep{suffix}.json"
    if args.refigure:
        report = json.loads(path.read_text())
    else:
        report = run_sweeps(args.k0, args.d0, args.envelope, pose, push_axis)
        path.write_text(json.dumps(report, indent=2))
    figure = make_figure(report, args.outdir, suffix=suffix)

    for key, xkey, label in (("e0_sweep", "E0", "E0 [J]"),
                             ("disturbance_sweep", "disturbance_scale", "dist scale")):
        print(f"\n=== {key} ===")
        print(f"{label:>12} {'variant':<19} {'auth%':>7} {'min aE':>7} {'tankmin':>8} "
              f"{'floor':>6} {'breach':>7} {'RMS':>7} {'max|ez|mm':>10}")
        for r in sorted(report[key], key=lambda r: (r[xkey], r["variant"])):
            print(f"{r[xkey]:>12} {r['variant']:<19} "
                  f"{100 * r['authorization_active_fraction']:>6.1f}% "
                  f"{r['alpha_E_min']:>7.3f} {r['tank_min']:>8.4f} "
                  f"{'ok' if r['tank_floor_holds'] else 'BREACH':>6} "
                  f"{r['tank_floor_breach_ticks']:>7} "
                  f"{r['rms_realization_residual']:>7.3f} "
                  f"{r['max_abs_e_axis_m'] * 1e3:>10.1f}")
    print(f"\nwrote {path}\nwrote {figure}")


if __name__ == "__main__":
    main()
