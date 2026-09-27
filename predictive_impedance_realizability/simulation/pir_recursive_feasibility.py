#!/usr/bin/env python3
"""Recursive feasibility of the merged QP: the missing theorem, scoped.

Section 11 has carried "recursive feasibility unproven" since the start, and
Section 12.0 showed it is load-bearing in a second place: the workspace box is
slack-relaxed *because* a hard box has no recursive-feasibility guarantee, and
was confirmed empirically to make the QP infeasible.  So the same missing
theorem decides both whether the box can be hard and whether the controller can
promise a solve at the next tick.

This module states what can actually be proved about the merged QP, and checks
every step of it numerically rather than asserting it.

---------------------------------------------------------------------------
PROPOSITION 2 (feasibility certificate).
---------------------------------------------------------------------------

The merged QP's hard constraints are the per-step torque rows, the residual
magnitude box |F_r| <= F_max, and the residual rate rows; the Cartesian
workspace/speed rows are slack-relaxed and the slacks are free above zero, so
they cannot obstruct feasibility.  Let x^free be the ALPHA -> 0 rollout from the
current state -- the closed loop under the passive nominal alone,

    x^free_{k+1} = A_cl x^free_k + B f_k + B_accel d,     A_cl = A - B G0,

and let Delta_F = force_rate_limit * dt be one step of the rate box.  If

  (F1)  |F_{r,prev}|_inf <= Delta_F, and
  (F2)  |tau_base - J_v^T G0 x^free_k|_inf <= tau_bar for every constrained
        step k,

then F_r ≡ 0, with the slacks set to the box violations of x^free, is a
feasible point of the QP.  The QP is therefore feasible.

(F1) is not free, and dropping it is the interesting half.  When the held
residual exceeds one rate step the zero sequence is not reachable in one tick,
so the certificate is the rate-limited RAMP to zero instead, checked here as
certificate B.  That matters beyond bookkeeping: Section 7's documented
fallback on an infeasible solve -- "F_r = 0 is exactly the alpha -> 0 passive
nominal the Task 1 gate certified" -- sets the held residual to zero at the
servo, which the QP's own rate row would not have allowed in one step.  The
ramp is the reachable version of that fallback.

(F2) is not a new object.  It is exactly the Task 1 gate's **diagnostic row 1b**
-- "the precondition holds on the fallback itself" -- which was added in Section
4.3 for an apparently unrelated reason and cut the apparent region from 70 cells
to 33.  Row 1b turns out to be the QP's feasibility certificate, which is why
dropping it doubled the region: half those cells could not certify their own
fallback.

---------------------------------------------------------------------------
PROPOSITION 3 (recursive feasibility, frozen model).
---------------------------------------------------------------------------

Proposition 2 certifies one solve.  Recursion needs the certificate to survive
the shift, and for that the horizon has to end somewhere it can stay.  Under a
constant force forecast the alpha -> 0 rollout has the equilibrium

    x_eq = (I - A_cl)^{-1} (B f + B_accel d),

which is diagnostic 3's fallback displacement -- the same point, arrived at from
the other direction.  Let P solve the discrete Lyapunov equation
A_cl^T P A_cl - P = -I (A_cl is Schur for K0, D0 > 0 at this dt, checked below),
and let a_j be the j-th row of J_v^T G0.  Then the terminal set

    X_f = { x : (x - x_eq)^T P (x - x_eq) <= c* },
    c*  = min_j [ (tau_bar_j - |tau_base_j - a_j^T x_eq|) / sqrt(a_j^T P^-1 a_j) ]^2

is positively invariant under the alpha -> 0 rollout and contained in the torque
polytope.  If the frozen model is carried across two solves and the horizon
terminal state lies in X_f, the previous solution shifted by one step and padded
with F_r = 0 satisfies every hard row at the next tick.

Two things to read off the formula for c*.  It is positive **iff (P1) holds
strictly at the fallback equilibrium** -- the same hypothesis as Merged
Lemma 1', now doing a second job -- and it shrinks to zero exactly as the
anchor headroom does, which is the quantity Section 8.1 found all three
Section 7 findings were monotone in.

WHAT THIS DOES NOT PROVE.  Proposition 3 is about the QP's own frozen model.
tau_base, J_v and Lambda^-1 are re-frozen at every solve from the true
nonlinear MuJoCo state, so the state the next QP starts from is not the state
this one predicted.  That gap is the same one Section 7.1's four-term residual
measures and Section 11 lists as unproven; nothing here closes it.  What
changes is its size: the missing step is now "the re-frozen model stays inside
X_f", not "feasibility, somehow".

Run::

    python3 pir_recursive_feasibility.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_discrete_lyapunov

import pir_common as pc
from pir_controller import PIRConfig
from run_pir_closed_loop import run_variant

K0, D0 = 380.0, 29.07
REC_POSE = (-1.30, -1.30, 1.571)
REC_K0, REC_D0 = 520.0, 56.13


# ---------------------------------------------------------------------------
# Proposition 2: build the candidate and check it against the real QP
# ---------------------------------------------------------------------------


def candidate_point(f_r_seq, a_con, lower, upper, n_i, n_s) -> np.ndarray:
    """z = (the given F_r sequence, slacks big enough for the box rows).

    The slacks are solved for rather than derived in closed form: with the F_r
    part fixed, each row's demand on its slack column is a scalar inequality.
    Deriving it by hand is exactly the kind of step that silently stops
    matching the assembly, so it is read off the matrix instead.
    """
    z = np.zeros(n_i + n_s)
    z[:n_i] = np.asarray(f_r_seq, dtype=float).reshape(n_i)
    value = a_con @ z
    for col in range(n_i, n_i + n_s):
        column = a_con[:, col]
        need = 0.0
        for row in np.nonzero(column)[0]:
            c = column[row]
            if np.isfinite(lower[row]) and value[row] < lower[row]:
                need = max(need, (lower[row] - value[row]) / c) if c > 0 else need
            if np.isfinite(upper[row]) and value[row] > upper[row]:
                need = max(need, (upper[row] - value[row]) / c) if c < 0 else need
        z[col] = max(0.0, need)
    return z


def check_certificate(mpc, dyn, state, p_nominal, R_d, force_forecast,
                      behaviour_forecast, pir_cfg, previous_residual,
                      keep_qp: bool = False) -> dict:
    """One tick: does (F1)+(F2) hold, and is the candidate actually feasible?"""
    cfg = mpc.cfg
    H = cfg.horizon
    n_i, n_s = 3 * H, 2 * H
    (p, q, a_con, lower, upper, labels, Lam_inv, tau_base, J_v, d_known,
     A_cl, B, B_accel, c_eff, g_id, x0) = mpc._condense(
        dyn, state, p_nominal, R_d, force_forecast, behaviour_forecast)

    # (F1): can the rate box reach zero in one step?
    delta_f = cfg.force_rate_limit * cfg.dt
    f1 = bool(np.max(np.abs(previous_residual)) <= delta_f + 1e-12)

    # (F2): row 1b along the alpha -> 0 rollout.
    jt_g0 = J_v.T @ pir_cfg.gain_matrix()
    cap = cfg.tau_max
    steps = H if cfg.torque_constraint_steps is None else cfg.torque_constraint_steps
    x = x0.copy()
    worst_1b = 0.0
    for k in range(H):
        if k < steps:
            worst_1b = max(worst_1b,
                           float(np.max(np.abs(tau_base - jt_g0 @ x) / cap)))
        x = A_cl @ x + B @ force_forecast[k] + B_accel @ d_known
    f2 = bool(worst_1b <= 1.0 + 1e-9)
    x_terminal = x.copy()

    def margin_of(f_r_seq) -> float:
        z = candidate_point(f_r_seq, a_con, lower, upper, n_i, n_s)
        v = a_con @ z
        return float(min(
            np.min(np.where(np.isfinite(lower), v - lower, np.inf)),
            np.min(np.where(np.isfinite(upper), upper - v, np.inf))))

    # Certificate A: F_r == 0 outright.  Needs (F1).
    worst = margin_of(np.zeros((H, 3)))
    feasible = bool(worst >= -1e-6)

    # Certificate B: the rate-limited RAMP to zero, which is what the runtime
    # can actually reach when the held residual is larger than one rate step.
    # Section 7's documented fallback -- "F_r = 0 is the alpha -> 0 nominal the
    # gate certified" -- is only reachable in one step when (F1) holds; the
    # ramp is the version of it the QP's own rate row admits.
    ramp = np.zeros((H, 3))
    current = np.asarray(previous_residual, dtype=float).copy()
    for k in range(H):
        step_to_zero = np.clip(-current, -delta_f, delta_f)
        current = current + step_to_zero
        ramp[k] = current
    worst_ramp = margin_of(ramp)
    feasible_ramp = bool(worst_ramp >= -1e-6)
    ramp_length = int(np.ceil(np.max(np.abs(previous_residual)) / delta_f)) if delta_f > 0 else 0

    # Solve the same QP here so the certificate can be paired with the solver's
    # own verdict tick by tick.  A sufficient condition must never say
    # "feasible" where the solver says "infeasible"; if it does, the candidate
    # construction has drifted from the assembly and the proposition is being
    # checked against the wrong matrix.
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(p), q=q, A=sparse.csc_matrix(a_con),
                 l=lower, u=upper, verbose=False, polishing=False,
                 eps_abs=cfg.osqp_eps, eps_rel=cfg.osqp_eps,
                 max_iter=cfg.osqp_max_iter)
    solver_ok = solver.solve(raise_error=False).info.status_val in (1, 2)

    out = {
        "F1_rate_admits_zero": f1,
        "F2_row1b_worst_ratio": worst_1b,
        "F2_holds": f2,
        "candidate_feasible": feasible,
        "candidate_worst_margin": worst,
        "ramp_feasible": feasible_ramp,
        "ramp_worst_margin": worst_ramp,
        "ramp_steps_to_zero": ramp_length,
        "solver_feasible": solver_ok,
        "certificate_sound": bool(solver_ok or not (feasible or feasible_ramp)),
        # Small arrays only: a 6 s run is 300 QP ticks and the assembled rows
        # are ~400 x 105, so holding every tick's matrices is 100 MB per run
        # for no reason.  The hard-box contrast keeps them on a subsample.
        "_A_cl": A_cl, "_B": B, "_B_accel": B_accel, "_d": d_known,
        "_tau_base": tau_base, "_jt_g0": jt_g0, "_cap": cap,
        "_x_terminal": x_terminal, "_f": force_forecast[0],
    }
    if keep_qp:
        out.update({"_A": a_con, "_l": lower, "_u": upper, "_p": p, "_q": q,
                    "_n_i": n_i, "_n_s": n_s})
    return out


# ---------------------------------------------------------------------------
# Proposition 3: the terminal ellipsoid
# ---------------------------------------------------------------------------


def terminal_set(A_cl, B, B_accel, d, f, tau_base, jt_g0, cap) -> dict:
    """X_f = {(x - x_eq)' P (x - x_eq) <= c*}, and whether it is nonempty."""
    eig = np.max(np.abs(np.linalg.eigvals(A_cl)))
    schur = bool(eig < 1.0)
    if not schur:
        return {"A_cl_spectral_radius": float(eig), "schur": False,
                "c_star": 0.0, "nonempty": False}
    P = solve_discrete_lyapunov(A_cl.T, np.eye(A_cl.shape[0]))
    P_inv = np.linalg.inv(P)
    x_eq = np.linalg.solve(np.eye(A_cl.shape[0]) - A_cl, B @ f + B_accel @ d)
    residual_at_eq = tau_base - jt_g0 @ x_eq
    c_each = []
    for j in range(jt_g0.shape[0]):
        a = jt_g0[j]
        headroom = cap[j] - abs(residual_at_eq[j])
        norm = np.sqrt(max(a @ P_inv @ a, 1e-30))
        c_each.append((headroom / norm) ** 2 if headroom > 0 else 0.0)
    c_star = float(min(c_each))
    return {
        "A_cl_spectral_radius": float(eig),
        "schur": True,
        "x_eq": x_eq.tolist(),
        "x_eq_push_axis_mm": float(1e3 * abs(x_eq[pc.PUSH_AXIS_INDEX])),
        "P1_at_equilibrium_worst_ratio":
            float(np.max(np.abs(residual_at_eq) / cap)),
        "c_star": c_star,
        "nonempty": bool(c_star > 0.0),
        "binding_joint": int(np.argmin(c_each)),
        "_P": P, "_x_eq": x_eq,
    }


# ---------------------------------------------------------------------------
# The hard-box counter-example, in our own QP rather than on the source's word
# ---------------------------------------------------------------------------


def hard_box_solves(rec: dict) -> bool:
    """Same QP with the slacks pinned to zero.  Does OSQP still solve it?"""
    n_i, n_s = rec["_n_i"], rec["_n_s"]
    a_con, lower, upper = rec["_A"], rec["_l"].copy(), rec["_u"].copy()
    pin = np.zeros((n_s, n_i + n_s))
    pin[:, n_i:] = np.eye(n_s)
    a_hard = np.vstack([a_con, pin])
    l_hard = np.concatenate([lower, np.zeros(n_s)])
    u_hard = np.concatenate([upper, np.zeros(n_s)])
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(rec["_p"]), q=rec["_q"],
                 A=sparse.csc_matrix(a_hard), l=l_hard, u=u_hard,
                 verbose=False, polishing=False, eps_abs=1e-6, eps_rel=1e-6,
                 max_iter=20000)
    return solver.solve(raise_error=False).info.status_val in (1, 2)


# ---------------------------------------------------------------------------
# Driving it over real closed loops
# ---------------------------------------------------------------------------


def study(pose, k0, d0, scales, label) -> dict:
    rows = []
    for scale in scales:
        collected: list[dict] = []
        pir_cfg = PIRConfig(k0=k0, d0=d0, envelope="derated_joint4",
                            pose=None if pose is None else tuple(pose))

        def probe(mpc, dyn, state, p_nominal, R_d, force_forecast,
                  behaviour_forecast, step, time):
            keep = (len(collected) % 12 == 0)
            collected.append(check_certificate(
                mpc, dyn, state, p_nominal, R_d, force_forecast,
                behaviour_forecast, pir_cfg, mpc.previous_residual,
                keep_qp=keep))

        out = run_variant("pir", k0, d0, "derated_joint4", scenario="merged",
                          disturbance_scale=scale, pose=pose, qp_probe=probe)
        n = len(collected)
        term = [terminal_set(r["_A_cl"], r["_B"], r["_B_accel"], r["_d"],
                             r["_f"], r["_tau_base"], r["_jt_g0"], r["_cap"])
                for r in collected]
        inside = []
        for r, t in zip(collected, term):
            if not t["nonempty"]:
                inside.append(False)
                continue
            dx = r["_x_terminal"] - t["_x_eq"]
            inside.append(bool(dx @ t["_P"] @ dx <= t["c_star"]))
        # The hard-box contrast is expensive, so probe the kept subsample.
        hard_ok = [hard_box_solves(r) for r in collected if "_A" in r]
        rows.append({
            "pose": label,
            "disturbance_scale": scale,
            "n_qp_ticks": n,
            "F1_holds_frac": float(np.mean([r["F1_rate_admits_zero"] for r in collected])),
            "F2_holds_frac": float(np.mean([r["F2_holds"] for r in collected])),
            "F2_worst_ratio": float(max(r["F2_row1b_worst_ratio"] for r in collected)),
            "candidate_feasible_frac":
                float(np.mean([r["candidate_feasible"] for r in collected])),
            "candidate_worst_margin":
                float(min(r["candidate_worst_margin"] for r in collected)),
            "ramp_feasible_frac":
                float(np.mean([r["ramp_feasible"] for r in collected])),
            "either_certificate_frac":
                float(np.mean([r["candidate_feasible"] or r["ramp_feasible"]
                               for r in collected])),
            "max_ramp_steps_to_zero":
                int(max(r["ramp_steps_to_zero"] for r in collected)),
            "solver_feasible_frac":
                float(np.mean([r["solver_feasible"] for r in collected])),
            "certificate_unsound_ticks":
                int(sum(not r["certificate_sound"] for r in collected)),
            "certificate_conservatism_ticks":
                int(sum(r["solver_feasible"] and not (r["candidate_feasible"]
                                                      or r["ramp_feasible"])
                        for r in collected)),
            "qp_infeasible_solves": out["summary"]["qp_infeasible_solves"],
            "terminal_nonempty_frac": float(np.mean([t["nonempty"] for t in term])),
            "c_star_min": float(min(t["c_star"] for t in term)),
            "c_star_median": float(np.median([t["c_star"] for t in term])),
            "P1_at_eq_worst_ratio":
                float(max(t.get("P1_at_equilibrium_worst_ratio", np.inf) for t in term)),
            "fallback_eq_mm":
                float(max(t.get("x_eq_push_axis_mm", 0.0) for t in term)),
            "terminal_state_inside_Xf_frac": float(np.mean(inside)),
            "A_cl_spectral_radius": float(max(t["A_cl_spectral_radius"] for t in term)),
            "hard_box_solves_frac": float(np.mean(hard_ok)),
            "hard_box_probed": int(len(hard_ok)),
        })
        print(f"  {label:<12} {scale:>4.0f}x  n={n:<4d} "
              f"F2 {100*rows[-1]['F2_holds_frac']:5.1f}%  "
              f"cand {100*rows[-1]['candidate_feasible_frac']:5.1f}%  "
              f"ramp {100*rows[-1]['ramp_feasible_frac']:5.1f}%  "
              f"Xf {100*rows[-1]['terminal_state_inside_Xf_frac']:5.1f}%  "
              f"c* {rows[-1]['c_star_median']:.3e}  "
              f"solver {100*rows[-1]['solver_feasible_frac']:5.1f}%  "
              f"unsound {rows[-1]['certificate_unsound_ticks']}  "
              f"hard-box solves {100*rows[-1]['hard_box_solves_frac']:5.1f}%")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--scales", type=float, nargs="*",
                        default=[1.0, 4.0, 12.0])
    args = parser.parse_args()

    print("Propositions 2 and 3, checked on the QP the controller actually "
          "assembles\n")
    rows = study(None, K0, D0, args.scales, "phri2")
    rows += study(pc.pose_from_q246(*REC_POSE), REC_K0, REC_D0, args.scales,
                  "recommended")

    report = {
        "proposition_2": (
            "F_r = 0 plus box slacks is feasible when (F1) |F_r,prev| <= "
            "force_rate_limit*dt and (F2) |tau_base - J^T G0 x^free_k| <= "
            "tau_bar for every constrained step; (F2) is the Task 1 gate's "
            "diagnostic row 1b"),
        "proposition_3": (
            "X_f = {(x-x_eq)' P (x-x_eq) <= c*} with A_cl' P A_cl - P = -I is "
            "invariant and inside the torque polytope; c* > 0 iff (P1) holds "
            "strictly at the fallback equilibrium x_eq"),
        "scope": (
            "frozen model only: tau_base, J_v, Lambda^-1 are re-frozen every "
            "solve from the nonlinear state, so recursion still assumes the "
            "re-frozen model lands in X_f"),
        "rows": rows,
    }
    report["verdict"] = {
        "certificate_never_failed":
            all(r["candidate_feasible_frac"] == 1.0 for r in rows),
        "ramp_certificate_never_failed":
            all(r["ramp_feasible_frac"] == 1.0 for r in rows),
        "either_certificate_never_failed":
            all(r["either_certificate_frac"] == 1.0 for r in rows),
        "row1b_is_the_certificate":
            all(r["F2_holds_frac"] == 1.0 for r in rows
                if r["candidate_feasible_frac"] == 1.0),
        "terminal_set_always_nonempty":
            all(r["terminal_nonempty_frac"] == 1.0 for r in rows),
        "terminal_constraint_would_ever_bind":
            any(r["terminal_state_inside_Xf_frac"] < 1.0 for r in rows),
        "hard_box_loses_feasibility":
            any(r["hard_box_solves_frac"] < 1.0 for r in rows),
        "certificate_is_sound_everywhere":
            all(r["certificate_unsound_ticks"] == 0 for r in rows),
        "certificate_is_conservative":
            any(r["certificate_conservatism_ticks"] > 0 for r in rows),
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "pir_recursive_feasibility.json").write_text(
        json.dumps(report, indent=2))
    print(f"\n  verdict: {report['verdict']}")
    print(f"\nwrote {args.outdir / 'pir_recursive_feasibility.json'}")


if __name__ == "__main__":
    main()
