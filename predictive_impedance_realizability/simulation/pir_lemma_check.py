#!/usr/bin/env python3
"""Merged Lemma 1': what the guarantee becomes once alpha_nom exists.

Section 10.6 exposed a gap between the lemma as stated and the controller as
shipped.  Across 20 resampled runs at the Section 10.5 cell, Lemma 1's
**precondition** held in 10 and its **conclusion** held in 20.  Those cannot
both be right for the lemma as written: a conditional guarantee whose condition
fails half the time should fail half the time.  What closed the gap is
``alpha_nom`` (decision 0), which is not in the lemma at all.

This module states the restated lemma and checks every clause of it, on
adversarially sampled ticks rather than only on trajectories the controller
happens to visit.

---------------------------------------------------------------------------
MERGED LEMMA 1' (nominal authorization, monotone)
---------------------------------------------------------------------------

Hypotheses

  (P1)  |tau_base,l| <= tau_bar at every tick.
        NOTE this is the whole precondition.  It is a property of the pose and
        the trajectory and does NOT involve K0, D0, e or v -- unlike Lemma 1's
        |tau_base + J^T F_nom| <= tau_bar, which does.
  (P2)  E_0 >= E_min.
  (P3)  alpha_nom is monotone non-increasing (``nominal_reauth_rate = 0``).

Conclusions

  (C1)  |tau_l| <= tau_bar for all l.            UNCONDITIONAL given (P1).
  (C2)  E_l >= E_min for all l.                  UNCONDITIONAL given (P1)-(P3).
  (C3)  UNTIL THE FIRST TICK at which the original precondition
        |tau_base + J^T F_nom| <= tau_bar fails, alpha_nom = 1 and the
        controller is bit-identical to the one without nominal authorization.
        So Lemma 1' strictly extends Lemma 1 and is inert on any run where
        Lemma 1 already applied throughout.
  (C5)  ALPHA_NOM IS A LATCH.  (P3) makes it monotone over the WHOLE run, so
        after the first firing it never recovers, and (C3)'s inertness is gone
        for the remainder of the run even at ticks where Lemma 1's precondition
        holds again.  The first draft of this module stated (C3) without the
        "until" and its own checker falsified it on 1326 of 4000 ticks --
        correctly, because the latch had already fired.  A deployed system must
        reset alpha_nom per contact episode; this benchmark is short enough
        that it does not, and nothing here characterises the ratchet over
        minutes.
  (C4)  THE COST OF (C1)-(C2).  Where alpha_nom < 1 the rendered nominal is alpha_nom*K0,
        alpha_nom*D0, so the storage is H0 = (1/2) v'Lv + (1/2) alpha_nom e'K0 e
        and the alpha -> 0 fallback displacement scales as 1/alpha_nom.  The
        design-time guarantee from diagnostic 3 -- that the fallback holds the
        push inside the workspace bound -- is NOT preserved.

Why (P3) is a hypothesis and not an implementation detail: the ledger debits
``spring_release = max(0, (1/2)(alpha_nom - alpha_prev) e'K0 e)`` when the
nominal RE-STIFFENS, and nothing bounds that debit against the available
energy.  With alpha_nom monotone the term is identically zero and (C2) goes
through; with an unrestricted rate it does not, and ``check_C2_needs_P3``
exhibits the counter-example rather than asserting the hypothesis is needed.

So the honest one-line reading of what decision 0 bought:

    alpha_nom converts a torque-envelope violation into a workspace-bound
    violation.  It preserves (C1) and (C2) at the cost of (C4), and (P3) buys
    (C2) at the cost of (C5).

Run::

    python3 pir_lemma_check.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import pir_common as pc
from pir_controller import PIRConfig, pir_servo_step
from run_pir_closed_loop import run_variant

K0, D0 = 380.0, 29.07
N_SAMPLES = 4000
SEED = 20260925


def _plant():
    env = pc.FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    dyn, state = env.get_dynamics_and_state()
    J_v = dyn.J[:3, :]
    Lam_inv = J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3)
    cfg = pc.FR3MPCConfig()
    tau_base, _, _ = pc.compute_tau_base(
        dyn, state, state.ee_rot.copy(), pc.make_default_impedance_params(cfg),
        cfg.K_rot, cfg.D_rot, cfg.lambda_reg)
    return J_v, Lam_inv, tau_base


def check_C1_C2_C3(rng: np.random.Generator) -> dict:
    """Adversarial sampling: huge residuals, wide states, ledger carried along."""
    J_v, Lam_inv, tau_base = _plant()
    cfg = PIRConfig(k0=K0, d0=D0)  # adopted default: nominal auth on, monotone
    plain = PIRConfig(k0=K0, d0=D0, nominal_authorization=False)
    cap = pc.torque_envelope(cfg.envelope)
    assert np.all(np.abs(tau_base) <= cap), "(P1) does not hold at this pose"

    tank = cfg.tank_initial
    prev_alpha_nom = 1.0
    worst_tau, worst_tank = 0.0, np.inf
    c1_fail = c2_fail = c3_fail = 0
    c3_checked = c5_latched = 0
    n_alpha_nom_fired = 0
    alpha_nom_rose = 0
    spring_release_nonzero = 0

    for _ in range(N_SAMPLES):
        e = rng.uniform(-0.35, 0.35, 3)      # well past the workspace box
        v = rng.uniform(-1.2, 1.2, 3)        # well past the speed limit
        f_r = rng.uniform(-600.0, 600.0, 3)  # far beyond anything the QP emits
        step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v, f_r,
                              tank=tank, h=1e-3, cap=cap,
                              previous_alpha_nom=prev_alpha_nom)
        worst_tau = max(worst_tau, step.tau_ratio)
        worst_tank = min(worst_tank, step.tank)
        if step.tau_ratio > 1.0 + 1e-9:
            c1_fail += 1
        if step.tank < cfg.tank_minimum - 1e-12:
            c2_fail += 1
        if step.alpha_nom < 1.0 - 1e-12:
            n_alpha_nom_fired += 1
        if step.alpha_nom > prev_alpha_nom + 1e-12:
            alpha_nom_rose += 1
        # (C3): inertness is claimed only BEFORE the latch first fires, so it
        # is checked against a fresh controller state rather than the carried
        # one.  Checking it against the carried state is what falsified the
        # first draft of this clause -- see (C5).
        if step.anchor_feasible:
            fresh = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v, f_r,
                                   tank=tank, h=1e-3, cap=cap,
                                   previous_alpha_nom=1.0)
            ref = pir_servo_step(plain, tau_base, J_v, Lam_inv, e, v, f_r,
                                 tank=tank, h=1e-3, cap=cap)
            if not np.allclose(fresh.tau, ref.tau, atol=1e-12):
                c3_fail += 1
            if not np.allclose(step.tau, ref.tau, atol=1e-12):
                c5_latched += 1
            c3_checked += 1
        tank = step.tank
        prev_alpha_nom = step.alpha_nom

    return {
        "n_samples": N_SAMPLES,
        "C1_torque_violations": c1_fail,
        "C1_worst_tau_ratio": float(worst_tau),
        "C2_floor_violations": c2_fail,
        "C2_worst_tank_J": float(worst_tank),
        "C2_floor_J": cfg.tank_minimum,
        "C3_checked_ticks": c3_checked,
        "C3_inertness_violations_from_fresh_state": c3_fail,
        "C5_ticks_where_latch_broke_inertness": c5_latched,
        "n_ticks_alpha_nom_fired": n_alpha_nom_fired,
        "alpha_nom_ever_rose": alpha_nom_rose,
        "spring_release_nonzero_ticks": spring_release_nonzero,
    }


def check_C2_needs_P3(rng: np.random.Generator) -> dict:
    """(P3) is load-bearing: drop it and (C2) fails. Exhibit, do not assert."""
    J_v, Lam_inv, tau_base = _plant()
    cap = pc.torque_envelope("derated_joint4")
    out = {}
    for label, rate in (("monotone (adopted)", 0.0), ("unrestricted", float("inf"))):
        cfg = PIRConfig(k0=K0, d0=D0, nominal_reauth_rate=rate)
        tank, prev = cfg.tank_initial, 1.0
        worst = np.inf
        violations = 0
        for _ in range(N_SAMPLES):
            e = rng.uniform(-0.35, 0.35, 3)
            v = rng.uniform(-1.2, 1.2, 3)
            f_r = rng.uniform(-600.0, 600.0, 3)
            step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v, f_r,
                                  tank=tank, h=1e-3, cap=cap,
                                  previous_alpha_nom=prev)
            worst = min(worst, step.tank)
            if step.tank < cfg.tank_minimum - 1e-12:
                violations += 1
            tank, prev = step.tank, step.alpha_nom
        out[label] = {"floor_violations": violations, "worst_tank_J": float(worst)}
    return out


def check_C4_cost() -> dict:
    """The cost clause: where alpha_nom < 1 the rendered stiffness is
    alpha_nom*K0, so the fallback bound the design certified no longer holds.

    Measured by comparing the alpha -> 0 fallback displacement at K0 against
    the same quantity at alpha*K0, and separately by the closed-loop excursion
    at a disturbance large enough to drive alpha_nom down.
    """
    rows = []
    for alpha in (1.0, 0.75, 0.5):
        r = pc.run_fallback_equilibrium(alpha * K0, alpha * D0)
        rows.append({"alpha": alpha, "effective_K0": alpha * K0,
                     "fallback_e_ss_m": r["e_ss_axis"],
                     "within_box": bool(r["e_ss_axis"] <= pc.WORKSPACE_BOUND_M)})
    closed = []
    for scale in (1.0, 8.0, 12.0):
        s = run_variant("pir", K0, D0, "derated_joint4", scenario="merged",
                        disturbance_scale=scale)["summary"]
        closed.append({"disturbance_scale": scale,
                       "alpha_nom_min": s["alpha_nom_min"],
                       "max_abs_e_axis_m": s["max_abs_e_axis_m"],
                       "box_overshoot": s["max_abs_e_axis_m"] / pc.WORKSPACE_BOUND_M,
                       "lemma1_precondition_holds": s["lemma1_precondition_holds"],
                       "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
                       "tank_floor_holds": s["tank_floor_holds"]})
    return {"fallback_vs_effective_stiffness": rows, "closed_loop": closed}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    args = parser.parse_args()
    rng = np.random.default_rng(SEED)

    report = {
        "lemma": "Merged Lemma 1' (nominal authorization, monotone)",
        "hypotheses": {
            "P1": "|tau_base| <= tau_bar at every tick (pose property; no K0, D0, e, v)",
            "P2": "E_0 >= E_min",
            "P3": "alpha_nom monotone non-increasing (nominal_reauth_rate = 0)",
        },
        "clauses": check_C1_C2_C3(rng),
        "P3_is_load_bearing": check_C2_needs_P3(rng),
        "C4_cost": check_C4_cost(),
    }
    c = report["clauses"]
    report["verdict"] = {
        "C1_holds": c["C1_torque_violations"] == 0,
        "C2_holds": c["C2_floor_violations"] == 0,
        "C3_holds": c["C3_inertness_violations_from_fresh_state"] == 0,
        "C5_latch_observed": c["C5_ticks_where_latch_broke_inertness"] > 0,
        "P3_needed": (report["P3_is_load_bearing"]["unrestricted"]["floor_violations"] > 0
                      and report["P3_is_load_bearing"]["monotone (adopted)"]["floor_violations"] == 0),
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_lemma_check.json").write_text(json.dumps(report, indent=2))

    print(f"Merged Lemma 1' -- {c['n_samples']} adversarial ticks "
          f"(|e| to 0.35 m, |v| to 1.2 m/s, |F_r| to 600 N)\n")
    print(f"  (C1) torque envelope  : {c['C1_torque_violations']} violations, "
          f"worst |tau|/cap = {c['C1_worst_tau_ratio']:.6f}")
    print(f"  (C2) tank floor       : {c['C2_floor_violations']} violations, "
          f"worst E = {c['C2_worst_tank_J']:.6f} J (floor {c['C2_floor_J']})")
    print(f"  (C3) inert before the latch fires : "
          f"{c['C3_inertness_violations_from_fresh_state']} violations over "
          f"{c['C3_checked_ticks']} applicable ticks")
    print(f"  (C5) latch breaks inertness later : "
          f"{c['C5_ticks_where_latch_broke_inertness']} / {c['C3_checked_ticks']} "
          f"of those same ticks differ once alpha_nom has latched")
    print(f"       alpha_nom fired on {c['n_ticks_alpha_nom_fired']} / {c['n_samples']} ticks; "
          f"rose on {c['alpha_nom_ever_rose']} (P3 requires 0)")
    print("\n  (P3) is load-bearing, not cosmetic:")
    for label, d in report["P3_is_load_bearing"].items():
        print(f"    {label:<20} floor violations {d['floor_violations']:>5}, "
              f"worst E = {d['worst_tank_J']:.4f} J")
    print("\n  (C4) the cost -- rendered stiffness is alpha_nom * K0:")
    for r in report["C4_cost"]["fallback_vs_effective_stiffness"]:
        print(f"    alpha = {r['alpha']:.2f} -> effective K0 = {r['effective_K0']:.0f} N/m, "
              f"fallback |e| = {1e3 * r['fallback_e_ss_m']:.1f} mm, "
              f"inside the 60 mm box: {r['within_box']}")
    print("    closed loop:")
    for r in report["C4_cost"]["closed_loop"]:
        print(f"      disturbance {r['disturbance_scale']:>4.0f}x  alpha_nom_min "
              f"{r['alpha_nom_min']:.3f}  excursion {100 * r['box_overshoot']:.0f}% of box  "
              f"(precond {r['lemma1_precondition_holds']}, "
              f"concl {r['lemma1_conclusion_holds']}, floor {r['tank_floor_holds']})")
    print(f"\n  verdict: {report['verdict']}")
    print(f"\nwrote {args.outdir / 'pir_lemma_check.json'}")


if __name__ == "__main__":
    main()
