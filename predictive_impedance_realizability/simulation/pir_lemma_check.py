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
MERGED LEMMA 1' (nominal authorization, energy-authorized re-stiffening)
---------------------------------------------------------------------------

Hypotheses

  (P1)  |tau_base,l| <= tau_bar at every tick.
        NOTE this is the whole precondition.  It is a property of the pose and
        the trajectory and does NOT involve K0, D0, e or v -- unlike Lemma 1's
        |tau_base + J^T F_nom| <= tau_bar, which does.
  (P2)  E_0 >= E_min.
  (P3)  alpha_nom FALLS freely and RISES only within the tank's budget:

            (alpha_nom,l - alpha_nom,l-1) * (1/2) e_l' K0 e_l  <=  E_{l-1} - E_min

        (``nominal_reauth = "energy_authorized"``).  The monotone rule
        ``nominal_reauth = "monotone"`` is the special case in which the left
        side is never positive, so everything proved below from (P3) holds for
        it too; it is retained as an ablation.

Conclusions

  (C1)  |tau_l| <= tau_bar for all l.            UNCONDITIONAL given (P1).
  (C2)  E_l >= E_min for all l.                  UNCONDITIONAL given (P1)-(P3).
  (C3)  UNTIL THE FIRST TICK at which the original precondition
        |tau_base + J^T F_nom| <= tau_bar fails, alpha_nom = 1 and the
        controller is bit-identical to the one without nominal authorization.
        So Lemma 1' strictly extends Lemma 1 and is inert on any run where
        Lemma 1 already applied throughout.
  (C5)  RECOVERY, AND ITS PRICE.  alpha_nom returns to 1 once the tank can pay
        for the re-stiffening -- in a single tick when
        (1 - alpha_nom)(1/2) e' K0 e <= E - E_min, and immediately when
        e' K0 e -> 0 (nothing is stored, so nothing is injected).  So (C3)'s
        inertness comes back instead of being lost for the rest of the run.

        The bound is the point, and it bites: while the robot stays far off its
        reference the spring energy (1/2) e' K0 e is two orders of magnitude
        larger than the whole tank, the budget buys almost nothing, and
        alpha_nom stays down.  ``check_C1_C2_C3`` runs in exactly that regime
        -- |e| near 0.35 m on all 4000 ticks -- and reports no full recovery,
        which is the rule working, not failing: a robot held 0.35 m off its
        reference has not earned its stiffness back.  Recovery is measured
        where it means something, by ``check_C5_recovery``, which scripts a
        transient followed by a return to a benign state.

        This clause replaces a WEAKER ONE.  Under the monotone rule that
        decision 0 originally adopted, alpha_nom was a LATCH: (P3) held over
        the whole run, so after the first firing it never recovered, and (C3)
        was empty from then on.  The first draft of this module stated (C3)
        without the "until" and its own checker falsified it on 1326 of 4000
        ticks -- correctly, because the latch had already fired.  The
        energy-authorized rule removes the latch without giving up (C2);
        ``check_C5_recovery`` measures both rules side by side rather than
        asserting the improvement.  Nothing here characterises the ratchet
        over minutes of real contact; that remains owed.
  (C4)  THE COST OF (C1)-(C2).  Where alpha_nom < 1 the rendered nominal is alpha_nom*K0,
        alpha_nom*D0, so the storage is H0 = (1/2) v'Lv + (1/2) alpha_nom e'K0 e
        and the alpha -> 0 fallback displacement scales as 1/alpha_nom.  The
        design-time guarantee from diagnostic 3 -- that the fallback holds the
        push inside the workspace bound -- is NOT preserved.

Why (P3) is a hypothesis and not an implementation detail: the ledger debits
``spring_release = max(0, (1/2)(alpha_nom - alpha_prev) e'K0 e)`` when the
nominal RE-STIFFENS, and without (P3) nothing bounds that debit against the
available energy.  (P3) bounds it by E - E_min, which is exactly what makes
the ``max(0, .)`` clamp on ``available`` in step 4 of the servo non-binding:
with the clamp inactive the step-6 ledger lands at E_min in the worst case
instead of below it, whether the residual's port power is positive (alpha_E
throttles it to what is left) or not (the ledger only gains).  With
``nominal_reauth = "free"`` the bound is absent and (C2) fails;
``check_C2_needs_P3`` exhibits the counter-example rather than asserting the
hypothesis is needed.

So the honest one-line reading of what decision 0 bought:

    alpha_nom converts a torque-envelope violation into a workspace-bound
    violation.  It preserves (C1) and (C2) at the cost of (C4).  The monotone
    rule bought (C2) at the cost of the latch; metering the recovery against
    the tank buys (C2) without it.

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


def check_C1_C2_C3(rng: np.random.Generator, mode: str = "energy_authorized") -> dict:
    """Adversarial sampling: huge residuals, wide states, ledger carried along."""
    J_v, Lam_inv, tau_base = _plant()
    cfg = PIRConfig(k0=K0, d0=D0, nominal_reauth=mode)
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
    recoveries_to_full = 0
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
            if step.alpha_nom >= 1.0 - 1e-12:
                recoveries_to_full += 1
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
        "mode": mode,
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
        "alpha_nom_rose_ticks": alpha_nom_rose,
        "alpha_nom_recoveries_to_full": recoveries_to_full,
        "spring_release_nonzero_ticks": spring_release_nonzero,
    }


def check_C2_needs_P3(rng: np.random.Generator) -> dict:
    """(P3) is load-bearing: drop it and (C2) fails. Exhibit, do not assert."""
    J_v, Lam_inv, tau_base = _plant()
    cap = pc.torque_envelope("derated_joint4")
    out = {}
    for label, mode in (("energy_authorized (adopted)", "energy_authorized"),
                        ("monotone", "monotone"),
                        ("unrestricted", "free")):
        cfg = PIRConfig(k0=K0, d0=D0, nominal_reauth=mode)
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


def check_C5_recovery() -> dict:
    """(C5): does inertness come BACK after the transient that broke it?

    The adversarial sampler above cannot answer this.  It holds |e| near 0.35 m
    on every tick, so the spring energy (1/2) e' K0 e is two orders of
    magnitude larger than the whole tank, the budget buys almost nothing, and
    alpha_nom stays down.  That is the correct behaviour (a robot held 0.35 m
    off its reference has not earned its stiffness back), not a recovery test.

    So script the episode: drive alpha_nom down with a hard transient, then let
    the state RETURN -- e decaying toward the reference with the matching
    velocity, which is what a real recovery looks like and, crucially, is what
    lets the tank recharge (the harvest is alpha_nom * v' D0 v, so a state that
    is merely benign but motionless recharges nothing).

    Two numbers characterise the rule independently of the scripted path:

      * the post-transient tank level E, and
      * the one-tick affordability threshold |e|*, the largest error at which
        the remaining budget pays for full re-stiffening outright,
            (1 - alpha) * (1/2) K0 |e|^2  <=  E - E_min.

    Below |e|* recovery is immediate; above it, it waits for the tank.
    """
    J_v, Lam_inv, tau_base = _plant()
    cap = pc.torque_envelope("derated_joint4")
    out = {}
    for label, mode in (("energy_authorized (adopted)", "energy_authorized"),
                        ("monotone", "monotone")):
        cfg = PIRConfig(k0=K0, d0=D0, nominal_reauth=mode)
        tank, prev = cfg.tank_initial, 1.0
        # Phase A -- the transient: large push-axis error, large residual.
        e_hard = np.array([0.0, 0.0, 0.30])
        for _ in range(200):
            step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e_hard,
                                  np.array([0.0, 0.0, 0.4]),
                                  np.array([0.0, 0.0, 400.0]),
                                  tank=tank, h=1e-3, cap=cap,
                                  previous_alpha_nom=prev)
            tank, prev = step.tank, step.alpha_nom
        after_transient, tank_after = prev, tank
        budget = max(0.0, tank_after - cfg.tank_minimum)
        e_star = (np.sqrt(2.0 * budget / ((1.0 - after_transient) * K0))
                  if after_transient < 1.0 - 1e-12 else np.inf)
        # Phase B -- the return: e decays to the reference, v = de/dt.
        h, tau_ret, e0 = 1e-3, 0.20, 0.05
        ticks_to_recover, e_at_recovery = None, None
        for i in range(2000):
            t = i * h
            ez = e0 * np.exp(-t / tau_ret)
            e = np.array([0.0, 0.0, ez])
            v = np.array([0.0, 0.0, -ez / tau_ret])
            step = pir_servo_step(cfg, tau_base, J_v, Lam_inv, e, v,
                                  np.array([0.0, 0.0, 5.0]),
                                  tank=tank, h=h, cap=cap,
                                  previous_alpha_nom=prev)
            tank, prev = step.tank, step.alpha_nom
            if ticks_to_recover is None and prev >= 1.0 - 1e-12:
                ticks_to_recover, e_at_recovery = i + 1, float(ez)
        out[label] = {
            "alpha_nom_after_transient": float(after_transient),
            "tank_after_transient_J": float(tank_after),
            "one_tick_affordable_below_mm": float(1e3 * e_star),
            "alpha_nom_after_return": float(prev),
            "ticks_to_full_recovery": ticks_to_recover,
            "error_at_recovery_mm": (None if e_at_recovery is None
                                     else 1e3 * e_at_recovery),
            "recovered": ticks_to_recover is not None,
            "tank_end_J": float(tank),
        }
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
        row = {"disturbance_scale": scale}
        # The adopted rule, the monotone ablation it replaced, and no alpha_nom
        # at all -- so the excursion column separates what the AUTHORIZATION
        # costs from what the LATCH cost.
        for key, variant, overrides in (
                ("adopted", "pir", None),
                ("monotone", "pir", {"nominal_reauth": "monotone"}),
                ("no_nominal_auth", "pir_no_nominal_auth", None)):
            s = run_variant(variant, K0, D0, "derated_joint4", scenario="merged",
                            disturbance_scale=scale, overrides=overrides)["summary"]
            row[key] = {
                "alpha_nom_min": s["alpha_nom_min"],
                "max_abs_e_axis_m": s["max_abs_e_axis_m"],
                "box_overshoot": s["max_abs_e_axis_m"] / pc.WORKSPACE_BOUND_M,
                "max_tau_ratio": s["lemma1_conclusion_max_tau_ratio"],
                "lemma1_precondition_holds": s["lemma1_precondition_holds"],
                "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
                "tank_floor_holds": s["tank_floor_holds"],
            }
        closed.append(row)
    return {"fallback_vs_effective_stiffness": rows, "closed_loop": closed}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    args = parser.parse_args()
    rng = np.random.default_rng(SEED)

    report = {
        "lemma": ("Merged Lemma 1' (nominal authorization, "
                  "energy-authorized re-stiffening)"),
        "hypotheses": {
            "P1": "|tau_base| <= tau_bar at every tick (pose property; no K0, D0, e, v)",
            "P2": "E_0 >= E_min",
            "P3": ("alpha_nom falls freely, rises only within E - E_min "
                   "(nominal_reauth = 'energy_authorized'; 'monotone' is the "
                   "special case)"),
        },
        "clauses": check_C1_C2_C3(rng, "energy_authorized"),
        "clauses_monotone_ablation": check_C1_C2_C3(
            np.random.default_rng(SEED), "monotone"),
        "P3_is_load_bearing": check_C2_needs_P3(rng),
        "C5_recovery": check_C5_recovery(),
        "C4_cost": check_C4_cost(),
    }
    c = report["clauses"]
    report["verdict"] = {
        "C1_holds": c["C1_torque_violations"] == 0,
        "C2_holds": c["C2_floor_violations"] == 0,
        "C3_holds": c["C3_inertness_violations_from_fresh_state"] == 0,
        "C5_recovery_observed":
            report["C5_recovery"]["energy_authorized (adopted)"]["recovered"],
        "C5_monotone_never_recovers":
            not report["C5_recovery"]["monotone"]["recovered"],
        "C5_latch_ticks_adopted": c["C5_ticks_where_latch_broke_inertness"],
        "C5_latch_ticks_monotone_ablation":
            report["clauses_monotone_ablation"]["C5_ticks_where_latch_broke_inertness"],
        "P3_needed": (
            report["P3_is_load_bearing"]["unrestricted"]["floor_violations"] > 0
            and report["P3_is_load_bearing"]["energy_authorized (adopted)"]["floor_violations"] == 0),
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_lemma_check.json").write_text(json.dumps(report, indent=2))

    print(f"Merged Lemma 1' -- {c['n_samples']} adversarial ticks "
          f"(|e| to 0.35 m, |v| to 1.2 m/s, |F_r| to 600 N)\n")
    print(f"  (C1) torque envelope  : {c['C1_torque_violations']} violations, "
          f"worst |tau|/cap = {c['C1_worst_tau_ratio']:.6f}")
    print(f"  (C2) tank floor       : {c['C2_floor_violations']} violations, "
          f"worst E = {c['C2_worst_tank_J']:.6f} J (floor {c['C2_floor_J']})")
    print(f"  (C3) inert from a fresh state : "
          f"{c['C3_inertness_violations_from_fresh_state']} violations over "
          f"{c['C3_checked_ticks']} applicable ticks")
    m = report["clauses_monotone_ablation"]
    print(f"  (C5) inertness NOT recovered  : "
          f"{c['C5_ticks_where_latch_broke_inertness']} / {c['C3_checked_ticks']} ticks "
          f"adopted, vs {m['C5_ticks_where_latch_broke_inertness']} / "
          f"{m['C3_checked_ticks']} under the monotone ablation (the latch)")
    print(f"       alpha_nom fired on {c['n_ticks_alpha_nom_fired']} / {c['n_samples']} ticks; "
          f"rose on {c['alpha_nom_rose_ticks']}, of which "
          f"{c['alpha_nom_recoveries_to_full']} all the way back to 1 "
          f"(monotone: {m['alpha_nom_rose_ticks']} rises, by construction 0)")
    print("\n  (C5) scripted episode -- transient, then back to a benign state:")
    for label, d in report["C5_recovery"].items():
        rec = ("never" if d["ticks_to_full_recovery"] is None
               else f"{d['ticks_to_full_recovery']} ticks, at "
                    f"|e| = {d['error_at_recovery_mm']:.1f} mm")
        print(f"    {label:<28} alpha_nom {d['alpha_nom_after_transient']:.3f} "
              f"after the transient (tank {d['tank_after_transient_J']:.4f} J, "
              f"affordable below {d['one_tick_affordable_below_mm']:.1f} mm) "
              f"-> {d['alpha_nom_after_return']:.3f}; recovery: {rec}")
    print("\n  (P3) is load-bearing, not cosmetic:")
    for label, d in report["P3_is_load_bearing"].items():
        print(f"    {label:<28} floor violations {d['floor_violations']:>5}, "
              f"worst E = {d['worst_tank_J']:.4f} J")
    print("\n  (C4) the cost -- rendered stiffness is alpha_nom * K0:")
    for r in report["C4_cost"]["fallback_vs_effective_stiffness"]:
        print(f"    alpha = {r['alpha']:.2f} -> effective K0 = {r['effective_K0']:.0f} N/m, "
              f"fallback |e| = {1e3 * r['fallback_e_ss_m']:.1f} mm, "
              f"inside the 60 mm box: {r['within_box']}")
    print("    closed loop -- excursion as % of the 60 mm box, "
          "max |tau|/cap in brackets:")
    print(f"      {'scale':>6}  {'adopted':>22}  {'monotone':>22}  {'no alpha_nom':>22}")
    for r in report["C4_cost"]["closed_loop"]:
        cells = []
        for key in ("adopted", "monotone", "no_nominal_auth"):
            d = r[key]
            cells.append(f"{100 * d['box_overshoot']:5.0f}%  "
                         f"[{d['max_tau_ratio']:.4f}]  a={d['alpha_nom_min']:.3f}")
        print(f"      {r['disturbance_scale']:>5.0f}x  " + "  ".join(cells))
    print(f"\n  verdict: {report['verdict']}")
    print(f"\nwrote {args.outdir / 'pir_lemma_check.json'}")


if __name__ == "__main__":
    main()
