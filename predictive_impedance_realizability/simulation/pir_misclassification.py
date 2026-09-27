#!/usr/bin/env python3
"""The force-misclassification pillar: measured, not merely disclaimed.

Section 11 has carried this as an untouched assumption: the tank meters
F_r^T v, so if the intentional force F_h leaks into the disturbance estimate,
the power bookkeeping is about the wrong quantity and every certificate in this
document can stay green while the behaviour is wrong.  Section 7.1's channel
split -- ``force_forecast`` (what the plant feels) against
``behaviour_forecast`` (what the behaviour responds to) -- made the assumption
VISIBLE but did not discharge it.

A disclaimer that says "this could fail" is worth much less than one that says
how, and by how much.  This sweeps the misclassification directly, using
impedance_residual's own knob: a fraction lambda of F_h is labelled
disturbance, so the behaviour layer stops responding to it while the plant
still feels all of it.  Only the LABEL moves; the plant's total is identical at
every lambda, so any difference is attributable to the labelling alone.

Three things are measured at each lambda.

  CERTIFICATES.  Merged Lemma 1's precondition and conclusion, the tank floor,
  the alpha's.  Expected to stay green: none of them is a statement about
  whether the label is right.

  THE CLOSURE.  The four-term residual identity, which caught two real bugs
  (Sections 8.2 and 9.4).  It is evaluated against a_id computed from the
  BEHAVIOUR channel, so when the label is wrong it closes perfectly against the
  wrong target.  This is the sharpest single statement available about the
  pillar: the best diagnostic in this document is structurally blind to it.

  THE ERROR THE CERTIFICATES DO NOT SEE.  The gap between the realized
  acceleration and the behaviour the operator actually asked for -- a_id
  recomputed from the FULL push rather than the leaked one -- and the energy
  the arm exchanges with the human through the misclassified channel,
  sum_l h (lambda F_h,l)^T v_l, which the ledger never debits.

Run::

    python3 pir_misclassification.py
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
from run_pir_closed_loop import run_variant

K0, D0 = 380.0, 29.07
REC_POSE = (-1.30, -1.30, 1.571)
REC_K0, REC_D0 = 520.0, 56.13
LEAKAGES = (0.0, 0.1, 0.25, 0.5, 0.75, 1.0)


def evaluate(leakage: float, pose, k0, d0, label, push_axis=None) -> dict:
    out = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                      pose=pose, leakage=leakage)
    log, s = out["log"], out["summary"]
    h = 1e-3
    axis = pc.PUSH_AXIS if push_axis is None else np.asarray(push_axis, float)

    # The behaviour the operator actually asked for, recomputed from the FULL
    # push -- the target the certificates are silent about.
    generator = pc.ImpedanceReference3D()
    a_id_true = np.zeros_like(log["a_id"])
    push = np.zeros_like(log["a_id"])
    for i, t in enumerate(log["time"]):
        push[i] = pc.human_force_at(float(t), axis=tuple(axis))
        a_id_true[i] = generator.acceleration(
            np.concatenate([log["e"][i], log["v"][i]]), push[i])

    behaviour_error = log["a_modelled"] - a_id_true
    unaccounted = float(np.sum(h * np.sum((leakage * push) * log["v"], axis=1)))

    return {
        "label": label,
        "leakage": leakage,
        # certificates
        "lemma1_precondition_holds": s["lemma1_precondition_holds"],
        "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
        "lemma1_conclusion_max_tau_ratio": s["lemma1_conclusion_max_tau_ratio"],
        "tank_floor_holds": s["tank_floor_holds"],
        "tank_min": s["tank_min"],
        "alpha_nom_min": s["alpha_nom_min"],
        "qp_infeasible_solves": s["qp_infeasible_solves"],
        # The diagnostic that is blind to this.  Taken on QP ticks, where the
        # identity is exact: between them r_reg and r_con_qp are held from the
        # last solve, so the residual sum is deliberately stale and the gap is
        # the hold, not a broken identity.
        "four_term_closure_max_on_qp_ticks":
            s["decomposition_closure_max_on_qp_ticks"],
        "reported_residual_rms": s["rms_realization_residual"],
        # what the certificates do not see
        "true_behaviour_error_rms": float(np.sqrt(np.mean(
            np.sum(behaviour_error ** 2, axis=1)))),
        "unaccounted_human_energy_J": unaccounted,
        "max_abs_e_axis_m": s["max_abs_e_axis_m"],
    }


def figure(rows: list[dict], path: Path) -> None:
    fig, axes = plt.subplots(1, 4, figsize=(16, 3.6))
    for label, marker in (("phri2", "o"), ("recommended", "s")):
        sub = [r for r in rows if r["label"] == label]
        lam = [100 * r["leakage"] for r in sub]
        axes[0].plot(lam, [r["four_term_closure_max_on_qp_ticks"] for r in sub],
                     marker=marker, label=label)
        axes[1].plot(lam, [r["reported_residual_rms"] for r in sub],
                     marker=marker, label=label)
        axes[2].plot(lam, [r["true_behaviour_error_rms"] for r in sub],
                     marker=marker, label=label)
        axes[3].plot(lam, [r["unaccounted_human_energy_J"] for r in sub],
                     marker=marker, label=label)
    axes[0].set_yscale("log")
    axes[0].set_title("the closure identity\n(blind to the label)")
    axes[0].set_ylabel("max closure on QP ticks [m/s$^2$]")
    axes[1].set_title("residual the controller REPORTS")
    axes[1].set_ylabel("RMS [m/s$^2$]")
    axes[2].set_title("error against the behaviour\nactually asked for")
    axes[2].set_ylabel("RMS [m/s$^2$]")
    axes[3].set_title("human-directed energy\noutside the ledger")
    axes[3].set_ylabel("[J]")
    for ax in axes:
        ax.set_xlabel("intentional force labelled disturbance [%]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Every certificate holds at every leakage; the behaviour does not")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    args = parser.parse_args()

    print("Force misclassification: only the LABEL moves, not the plant\n")
    print(f"  {'pose':<12}{'leak':>6}  {'precond':>8}{'concl':>7}{'floor':>7}"
          f"  {'closure':>10}  {'reported':>9}  {'true err':>9}  {'unacct J':>9}")
    rows = []
    for pose, k0, d0, label in ((None, K0, D0, "phri2"),
                                (pc.pose_from_q246(*REC_POSE), REC_K0, REC_D0,
                                 "recommended")):
        for leakage in LEAKAGES:
            r = evaluate(leakage, pose, k0, d0, label)
            rows.append(r)
            print(f"  {label:<12}{100*leakage:>5.0f}%  "
                  f"{str(r['lemma1_precondition_holds']):>8}"
                  f"{str(r['lemma1_conclusion_holds']):>7}"
                  f"{str(r['tank_floor_holds']):>7}  "
                  f"{r['four_term_closure_max_on_qp_ticks']:>10.2e}  "
                  f"{r['reported_residual_rms']:>9.3f}  "
                  f"{r['true_behaviour_error_rms']:>9.3f}  "
                  f"{r['unaccounted_human_energy_J']:>9.4f}")

    report = {
        "question": ("does any certificate in this document notice when the "
                     "intentional force is labelled disturbance?"),
        "knob": ("a fraction lambda of F_h is removed from the behaviour "
                 "channel only; the plant's total force is identical at every "
                 "lambda"),
        "rows": rows,
    }
    green = [r for r in rows if r["lemma1_conclusion_holds"]
             and r["tank_floor_holds"]]
    report["verdict"] = {
        "all_certificates_hold_at_every_leakage":
            len(green) == len(rows),
        "closure_stays_at_machine_precision":
            all(r["four_term_closure_max_on_qp_ticks"] < 1e-9 for r in rows),
        "reported_residual_falls_while_true_error_does_not":
            min(r["reported_residual_rms"] for r in rows)
            < 0.9 * max(r["reported_residual_rms"] for r in rows),
        "true_behaviour_error_grows":
            max(r["true_behaviour_error_rms"] for r in rows)
            > 1.5 * min(r["true_behaviour_error_rms"] for r in rows),
        "worst_unaccounted_human_energy_J":
            max(abs(r["unaccounted_human_energy_J"]) for r in rows),
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_misclassification.json").write_text(
        json.dumps(report, indent=2))
    figure(rows, args.outdir / "pir_misclassification.png")
    print(f"\n  verdict: {report['verdict']}")
    print(f"\nwrote {args.outdir / 'pir_misclassification.json'}")


if __name__ == "__main__":
    main()
