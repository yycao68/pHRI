#!/usr/bin/env python3
"""Which of this document's claims survive a pose sweep?

Section 11.1's table is the reason this exists.  Nine conclusions here were
later overturned, five of them because a carefully measured property of ONE FR3
configuration was stated as a property of the architecture.  The rule that came
out of it -- "nothing in this line should be claimed without a pose sweep" --
was stated and then immediately broken in the next paragraph, which is the best
available evidence for how strong the pull toward the tidy generalisation is.

So rather than trusting the rule, apply it: take the claims this document still
makes, evaluate each one at every pose in Section 10.5's family, and report
which are pose-invariant and which are not.  A claim that holds at every pose
is a claim about the architecture.  A claim that moves is a claim about a
configuration, and has to say so.

ONE VARIABLE.  The gain rule is held fixed across the sweep -- K0 = 380 with D0
from the pose's own task inertia (``damping_for``) -- so the only thing varying
is the pose.  That is deliberately NOT the recommended operating point, whose
K0 = 520 was chosen for its pose by Section 9.3: mixing the two would conflate
"the pose changed" with "the design changed", which is the exact error this
module exists to catch.

Run::

    python3 pir_pose_sweep.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import pir_common as pc
import pir_recursive_feasibility as rf
from pir_controller import PIRConfig
from pir_joint_scan import damping_for, pose_at, _task_inertia_on_push_axis
from run_pir_closed_loop import run_variant

K0 = 380.0
LAMBDAS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
SCALES = (1.0, 12.0)
LEAKAGE = 0.5


def certificate_stats(pose, k0, d0, scale) -> dict:
    """Section 8.6's Propositions 2 and 3, on this pose."""
    collected: list[dict] = []
    cfg = PIRConfig(k0=k0, d0=d0, envelope="derated_joint4", pose=tuple(pose))

    def probe(mpc, dyn, state, p_nominal, R_d, force_forecast,
              behaviour_forecast, step, time):
        collected.append(rf.check_certificate(
            mpc, dyn, state, p_nominal, R_d, force_forecast,
            behaviour_forecast, cfg, mpc.previous_residual,
            keep_qp=(len(collected) % 20 == 0)))

    out = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                      disturbance_scale=scale, pose=pose, qp_probe=probe)
    term = [rf.terminal_set(r["_A_cl"], r["_B"], r["_B_accel"], r["_d"],
                            r["_f"], r["_tau_base"], r["_jt_g0"], r["_cap"])
            for r in collected]
    hard = [rf.hard_box_solves(r) for r in collected if "_A" in r]
    return {
        "summary": out["summary"],
        "cert_frac": float(np.mean([r["candidate_feasible"] or r["ramp_feasible"]
                                    for r in collected])),
        "terminal_nonempty_frac": float(np.mean([t["nonempty"] for t in term])),
        "hard_box_frac": float(np.mean(hard)),
    }


def misclassification_gap(pose, k0, d0) -> dict:
    """Section 11.0's failure, re-measured here: does the reported residual
    fall while the true error rises, at THIS pose too?"""
    generator = pc.ImpedanceReference3D()
    out = {}
    for tag, leak in (("clean", 0.0), ("leaked", LEAKAGE)):
        run = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                          pose=pose, leakage=leak)
        log, s = run["log"], run["summary"]
        a_true = np.zeros_like(log["a_id"])
        for i, t in enumerate(log["time"]):
            push = pc.human_force_at(float(t), axis=tuple(pc.PUSH_AXIS))
            a_true[i] = generator.acceleration(
                np.concatenate([log["e"][i], log["v"][i]]), push)
        out[tag] = {
            "reported": s["rms_realization_residual"],
            "true": float(np.sqrt(np.mean(
                np.sum((log["a_modelled"] - a_true) ** 2, axis=1)))),
            "certificates_hold": bool(s["lemma1_conclusion_holds"]
                                      and s["tank_floor_holds"]),
        }
    return out


def evaluate(lam: float) -> dict:
    pose = pose_at(lam)
    d0 = damping_for(K0, _task_inertia_on_push_axis(pose))
    fallback = pc.run_fallback_equilibrium(K0, d0, pose=pose)
    row = {"lambda": lam, "K0": K0, "D0": d0,
           "diag3_e_ss_m": fallback["e_ss_axis"]}
    for scale in SCALES:
        st = certificate_stats(pose, K0, d0, scale)
        s = st["summary"]
        row[f"s{scale:.0f}"] = {
            "lemma1_precondition_holds": s["lemma1_precondition_holds"],
            "lemma1_conclusion_holds": s["lemma1_conclusion_holds"],
            "tank_floor_holds": s["tank_floor_holds"],
            "alpha_nom_inert": bool(s["alpha_nom_min"] >= 1.0 - 1e-12),
            "anchor_headroom": s["anchor_headroom"],
            "cert_frac": st["cert_frac"],
            "terminal_nonempty_frac": st["terminal_nonempty_frac"],
            "hard_box_frac": st["hard_box_frac"],
            "qp_infeasible_solves": s["qp_infeasible_solves"],
        }
    row["misclassification"] = misclassification_gap(pose, K0, d0)
    return row


#: claim -> (how to read it off a row, what "holds" means)
CLAIMS = {
    "Lemma 1 precondition holds (1x)":
        lambda r: r["s1"]["lemma1_precondition_holds"],
    "Lemma 1 precondition holds (12x)":
        lambda r: r["s12"]["lemma1_precondition_holds"],
    "Lemma 1 conclusion holds (1x)":
        lambda r: r["s1"]["lemma1_conclusion_holds"],
    "Lemma 1 conclusion holds (12x)":
        lambda r: r["s12"]["lemma1_conclusion_holds"],
    "tank floor holds (12x)":
        lambda r: r["s12"]["tank_floor_holds"],
    "alpha_nom inert (1x)":
        lambda r: r["s1"]["alpha_nom_inert"],
    "alpha_nom inert (12x)":
        lambda r: r["s12"]["alpha_nom_inert"],
    "diagnostic 3: fallback inside the box":
        lambda r: r["diag3_e_ss_m"] <= pc.WORKSPACE_BOUND_M,
    "anchor headroom positive (1x)":
        lambda r: r["s1"]["anchor_headroom"] > 0.0,
    "anchor headroom positive (12x)":
        lambda r: r["s12"]["anchor_headroom"] > 0.0,
    "anchor headroom above 10% (1x)":
        lambda r: r["s1"]["anchor_headroom"] > 0.10,
    "Prop 2 certificate at every tick (12x)":
        lambda r: r["s12"]["cert_frac"] >= 1.0,
    "Prop 3 terminal set nonempty (12x)":
        lambda r: r["s12"]["terminal_nonempty_frac"] >= 1.0,
    "a hard box would still solve (12x)":
        lambda r: r["s12"]["hard_box_frac"] >= 1.0,
    "no infeasible solve (12x)":
        lambda r: r["s12"]["qp_infeasible_solves"] == 0,
    "misclassification is invisible":
        lambda r: (r["misclassification"]["leaked"]["certificates_hold"]
                   and r["misclassification"]["leaked"]["reported"]
                   < r["misclassification"]["clean"]["reported"]
                   and r["misclassification"]["leaked"]["true"]
                   > r["misclassification"]["clean"]["true"]),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--rescore", action="store_true",
                        help="re-derive the claim table from the stored JSON "
                             "without re-running the 24 closed loops, so "
                             "adding a claim does not cost a sweep")
    args = parser.parse_args()
    path = args.outdir / "pir_pose_sweep.json"

    print("Applying Section 11.1's own rule: which claims survive a pose "
          f"sweep?\n  (K0 = {K0:.0f} held fixed, D0 from each pose's task "
          "inertia -- the pose is the only variable)\n")
    if args.rescore:
        rows = json.loads(path.read_text())["rows"]
    else:
        rows = [evaluate(lam) for lam in LAMBDAS]

    width = max(len(c) for c in CLAIMS)
    header = "  ".join(f"l={lam:.1f}" for lam in LAMBDAS)
    print(f"  {'claim':<{width}}  {header}   verdict")
    verdicts = {}
    for claim, read in CLAIMS.items():
        vals = [bool(read(r)) for r in rows]
        invariant = all(vals) or not any(vals)
        verdicts[claim] = {
            "per_pose": dict(zip([f"{l:.1f}" for l in LAMBDAS], vals)),
            "pose_invariant": invariant,
            "holds_everywhere": all(vals),
            "holds_nowhere": not any(vals),
        }
        marks = "  ".join(f"{'  ok ' if v else ' FAIL'}" for v in vals)
        tag = ("invariant" if invariant else "POSE-DEPENDENT")
        print(f"  {claim:<{width}}  {marks}   {tag}")

    report = {
        "question": "which claims are about the architecture, not the pose?",
        "design_held_fixed": {"K0": K0, "D0_rule": "2 * zeta * sqrt(K0 * Lambda_axis)"},
        "lambda_grid": list(LAMBDAS),
        "rows": rows,
        "claims": verdicts,
    }
    dependent = [c for c, v in verdicts.items() if not v["pose_invariant"]]
    report["verdict"] = {
        "n_claims": len(CLAIMS),
        "n_pose_invariant": sum(v["pose_invariant"] for v in verdicts.values()),
        "pose_dependent_claims": dependent,
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2))
    print(f"\n  {report['verdict']['n_pose_invariant']} of "
          f"{len(CLAIMS)} claims are pose-invariant")
    for c in dependent:
        print(f"    POSE-DEPENDENT: {c}")
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
