#!/usr/bin/env python3
"""Task 2: the merged statements, and a check of each one.

The synthesis note's Task 2 asks for merged Lemma 1 (per-joint alpha_tau closed
form, the alpha_tau * alpha_E on-segment lemma) and merged Proposition 1 at
submittable granularity, with the precondition stated as a STANDING RUN-TIME
HYPOTHESIS rather than a design-time check.  The pieces were built in the order
the experiments demanded them and are scattered across Sections 7.3, 8.5 and
8.6; this module collects the ones that are statements about the ALGEBRA
(rather than about one FR3 configuration) and checks each.

---------------------------------------------------------------------------
LEMMA A (per-joint closed form, and maximality).
---------------------------------------------------------------------------

Let a be the anchor, r = J_v^T F_r the residual's joint torque, and tau_bar the
per-joint envelope.  If |a_l| <= tau_bar_l for every l, then

    A(a, r) := { alpha in [0, 1] : |a + alpha r| <= tau_bar }  =  [0, alpha_tau]

with the closed form

    alpha_tau = min( 1, min_{l : r_l != 0} (tau_bar_l - sgn(r_l) a_l) / |r_l| ).

PROOF.  Joint l constrains alpha by -tau_bar_l <= a_l + alpha r_l <= tau_bar_l.
For r_l > 0 the binding side is the upper one, giving alpha <= (tau_bar_l -
a_l)/r_l; for r_l < 0 it is the lower one, giving alpha <= (-tau_bar_l -
a_l)/r_l = (tau_bar_l + a_l)/|r_l|.  Both are (tau_bar_l - sgn(r_l) a_l)/|r_l|.
Rows with r_l = 0 read |a_l| <= tau_bar_l, which is the hypothesis.  Each row's
admissible set is an interval containing 0 (again by the hypothesis), so the
intersection with [0, 1] is [0, alpha_tau].  The set is an INTERVAL FROM ZERO,
which is the only property Lemma B needs.  []

The hypothesis |a| <= tau_bar is exactly Merged Lemma 1's precondition.  When
it fails the set A is empty and no scaling of the residual repairs it -- which
is Section 7.7's finding, and why alpha_nom exists.

---------------------------------------------------------------------------
LEMMA B (composition on the segment).
---------------------------------------------------------------------------

Under Lemma A's hypothesis, for every beta in [0, alpha_tau],
|a + beta r| <= tau_bar.  In particular the applied scale alpha_E * alpha_tau
is admissible for any alpha_E in [0, 1], so composing the torque and energy
scalings -- in either order -- preserves the torque conclusion.

PROOF.  Immediate from Lemma A: A is the interval [0, alpha_tau], and
alpha_E * alpha_tau lies in it.  []

This is the whole content of "the two authorizations do not fight".  The energy
layer may only shrink the residual, and shrinking moves the command along the
segment from a + alpha_tau r back toward the anchor a, which is interior.

---------------------------------------------------------------------------
MERGED PROPOSITION 1 (what the merged controller delivers).
---------------------------------------------------------------------------

Under (P1) |tau_base| <= tau_bar, (P2) E_0 >= E_min and (P3) the
energy-authorized re-stiffening rule of Section 8.5:

  (i)   TORQUE.  |tau_l| <= tau_bar for every tick.  [Lemma B + alpha_nom]
  (ii)  ENERGY.  Over any interval, the energy the residual extracts at the
        port is bounded by the initial budget plus the damping actually
        applied:
            sum_l h F_applied,l^T v_l  <=  (E_0 - E_min)
                                           + sum_l h alpha_nom,l v_l^T D_0 v_l.
        This is the storage-function statement behind (C2), in the form that
        does not depend on the tank's upper cap (capping discards credit, which
        only strengthens it).
  (iii) REALIZATION.  The realized behaviour differs from the desired one by
        exactly the four-term residual,
            a_modelled - a_id = r_reg + r_con + r_mod + r_auth,
        an identity, not a fit (Section 7.3; it closes to 4e-16 and has caught
        two real bugs).

STANDING HYPOTHESIS, NOT A DESIGN-TIME CHECK.  (P1) is a property of the pose
and the trajectory.  Nothing in the controller defends it, it is re-evaluated
at every tick, and it can fail at run time -- Section 10.6 found it holding in
10 of 20 resampled runs at one cell.  Everything above is conditional on it,
and it appears in three separate roles: Merged Lemma 1's precondition, the
nonemptiness of Section 8.6's terminal set, and the feasibility of the QP's own
fallback.  A write-up that states it once, early, as a design-time condition
would be misrepresenting all three.

Run::

    python3 pir_theory_check.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import pir_common as pc
from pir_controller import PIRConfig, torque_scale
from run_pir_closed_loop import run_variant

N_DRAWS = 20000
SEED = 20260927
K0, D0 = 380.0, 29.07


def alpha_tau_closed_form(a: np.ndarray, r: np.ndarray,
                          cap: np.ndarray) -> float:
    """Lemma A's formula, written from the statement rather than the code."""
    if np.any(np.abs(a) > cap + 1e-10):
        return 0.0
    alpha = 1.0
    active = np.abs(r) > 0.0
    if np.any(active):
        alpha = min(alpha, float(np.min(
            (cap[active] - np.sign(r[active]) * a[active]) / np.abs(r[active]))))
    return float(np.clip(alpha, 0.0, 1.0))


def check_lemma_A(rng: np.random.Generator, cap: np.ndarray) -> dict:
    """The closed form must agree with the imported implementation, and the
    interval must be exactly [0, alpha_tau] -- tight from above."""
    worst_gap = 0.0
    n_disagree = n_infeasible_at_alpha = n_not_maximal = 0
    n_zero_rows = 0
    for _ in range(N_DRAWS):
        # Draw the anchor inside the box (Lemma A's hypothesis) and the
        # residual torque anywhere, including exact zeros on some rows.
        a = rng.uniform(-1.0, 1.0, cap.size) * cap
        r = rng.uniform(-400.0, 400.0, cap.size)
        mask = rng.random(cap.size) < 0.25
        r[mask] = 0.0
        n_zero_rows += int(mask.sum())
        mine = alpha_tau_closed_form(a, r, cap)
        theirs, ok = torque_scale(a, r, cap)
        worst_gap = max(worst_gap, abs(mine - theirs))
        if abs(mine - theirs) > 1e-9 or not ok:
            n_disagree += 1
        if np.any(np.abs(a + mine * r) > cap + 1e-9):
            n_infeasible_at_alpha += 1
        # Maximality: nudging past alpha_tau must leave the box, unless the
        # cap of 1 is what bound it.
        if mine < 1.0 - 1e-12:
            eps = 1e-6 * max(1.0, float(np.max(cap / np.maximum(np.abs(r), 1e-9))))
            if np.all(np.abs(a + (mine + eps) * r) <= cap + 1e-12):
                n_not_maximal += 1
    return {
        "n_draws": N_DRAWS,
        "n_rows_with_zero_residual": n_zero_rows,
        "closed_form_vs_implementation_worst_gap": float(worst_gap),
        "n_disagreements": n_disagree,
        "n_infeasible_at_alpha_tau": n_infeasible_at_alpha,
        "n_not_maximal": n_not_maximal,
    }


def check_lemma_B(rng: np.random.Generator, cap: np.ndarray) -> dict:
    """Every beta in [0, alpha_tau] is admissible, so alpha_E * alpha_tau is."""
    n_violations = n_composed_violations = 0
    worst_ratio = 0.0
    for _ in range(N_DRAWS):
        a = rng.uniform(-1.0, 1.0, cap.size) * cap
        r = rng.uniform(-400.0, 400.0, cap.size)
        alpha = alpha_tau_closed_form(a, r, cap)
        beta = rng.uniform(0.0, 1.0) * alpha
        if np.any(np.abs(a + beta * r) > cap + 1e-9):
            n_violations += 1
        alpha_e = rng.uniform(0.0, 1.0)
        composed = alpha_e * alpha
        ratio = float(np.max(np.abs(a + composed * r) / cap))
        worst_ratio = max(worst_ratio, ratio)
        if ratio > 1.0 + 1e-9:
            n_composed_violations += 1
    return {
        "n_draws": N_DRAWS,
        "n_interior_violations": n_violations,
        "n_composed_violations": n_composed_violations,
        "worst_composed_tau_ratio": float(worst_ratio),
    }


def check_proposition_1_energy(scales, pose=None, k0=K0, d0=D0,
                               label="phri2") -> list[dict]:
    """(ii): extracted energy <= initial budget + damping actually applied.

    Checked on the closed loop rather than on sampled ticks, because it is an
    accumulation statement -- a per-tick check cannot fail the way a long run
    can.
    """
    rows = []
    cfg = PIRConfig(k0=k0, d0=d0)
    d0v = cfg.d0_vector()
    for scale in scales:
        out = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                          disturbance_scale=scale, pose=pose)
        log, summary = out["log"], out["summary"]
        h = 1e-3
        v = log["v"]
        extracted = float(np.sum(h * np.sum(log["f_r_applied"] * v, axis=1)))
        dissipated = float(np.sum(
            h * log["alpha_nom"] * np.sum(v * (d0v * v), axis=1)))
        budget = cfg.tank_initial - cfg.tank_minimum
        rows.append({
            "pose": label,
            "disturbance_scale": scale,
            "extracted_J": extracted,
            "budget_plus_dissipated_J": budget + dissipated,
            "slack_J": budget + dissipated - extracted,
            "holds": bool(extracted <= budget + dissipated + 1e-9),
            "tank_floor_holds": summary["tank_floor_holds"],
            "max_tau_ratio": summary["lemma1_conclusion_max_tau_ratio"],
        })
        print(f"  (ii) {label:<12}{scale:>4.0f}x  extracted {extracted:8.4f} J  "
              f"<= budget+dissipated {budget + dissipated:8.4f} J  "
              f"slack {rows[-1]['slack_J']:8.4f} J  {rows[-1]['holds']}")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--scales", type=float, nargs="*", default=[1.0, 4.0, 12.0])
    args = parser.parse_args()
    rng = np.random.default_rng(SEED)
    cap = pc.torque_envelope("derated_joint4")

    print("Task 2 -- the merged statements, each checked\n")
    lemma_a = check_lemma_A(rng, cap)
    print(f"  (A) closed form vs the imported torque_scale: "
          f"{lemma_a['n_disagreements']} disagreements in {lemma_a['n_draws']} "
          f"draws, worst gap {lemma_a['closed_form_vs_implementation_worst_gap']:.2e}")
    print(f"      admissible at alpha_tau: "
          f"{lemma_a['n_infeasible_at_alpha_tau']} failures; "
          f"maximal: {lemma_a['n_not_maximal']} failures")
    lemma_b = check_lemma_B(rng, cap)
    print(f"  (B) interior of [0, alpha_tau]: {lemma_b['n_interior_violations']} "
          f"violations; composed alpha_E*alpha_tau: "
          f"{lemma_b['n_composed_violations']} violations, worst ratio "
          f"{lemma_b['worst_composed_tau_ratio']:.6f}\n")
    energy = check_proposition_1_energy(args.scales)
    energy += check_proposition_1_energy(
        args.scales, pose=pc.pose_from_q246(-1.30, -1.30, 1.571),
        k0=520.0, d0=56.13, label="recommended")

    report = {
        "lemma_A": lemma_a,
        "lemma_B": lemma_b,
        "proposition_1_energy": energy,
        "standing_hypothesis": (
            "(P1) |tau_base| <= tau_bar is a property of the pose and the "
            "trajectory, re-evaluated every tick and undefended by the "
            "controller; it appears as Merged Lemma 1's precondition, as the "
            "nonemptiness of Section 8.6's terminal set, and as the "
            "feasibility of the QP's own fallback"),
    }
    report["verdict"] = {
        "A_closed_form_matches": lemma_a["n_disagreements"] == 0,
        "A_alpha_tau_is_maximal": lemma_a["n_not_maximal"] == 0,
        "B_composition_safe": lemma_b["n_composed_violations"] == 0,
        "P1_energy_bound_holds": all(r["holds"] for r in energy),
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_theory_check.json").write_text(json.dumps(report, indent=2))
    print(f"\n  verdict: {report['verdict']}")
    print(f"\nwrote {args.outdir / 'pir_theory_check.json'}")


if __name__ == "__main__":
    main()
