#!/usr/bin/env python3
"""Run the merged PIR controller on phri2's FR3 20 N benchmark.

This exists to pay the debt Section 7 of the Task 1 draft recorded: rows (1)
and (4) of the ``(K0, D0)`` gate were measured by **replaying** the trajectory
phri2's own MPC produced, which is exact only if the merged controller's total
command tracks phri2's.  It does not, exactly -- the merged QP optimizes the
residual under a different feasible set -- so the 2 % margin row (1) reported
at the recommended cell has to be re-measured in the merged controller's own
closed loop before anything is claimed from it.

Four variants, all sharing the same plant, nominal, QP and torque envelope:

``pir``               the merged rule: 1 kHz torque + energy re-authorization.
``pir_manager_guard`` identical, but alpha_E is recomputed only once per 20 ms
                      manager update and held across the fast ticks -- the
                      transplant of impedance_residual's B4 guard onto the
                      merged port.  Isolates whether *fast* re-authorization
                      is load-bearing here too.
``pir_no_tank``       the merged rule with the tank disabled.  This is the
                      feasibility axis alone: phri2's story with a passive
                      nominal, and no passivity axis.
``zero_nominal``      the same merged controller with K0 = D0 = 0 -- i.e. no
                      passive nominal, which is phri2's structure.  Its anchor
                      is tau_base alone, so it is the reference row (1) and (4)
                      are compared against.
``pir_no_nominal_auth``  ``pir`` with the servo's authority over the nominal
                      taken away again.  This is what Section 7 measured before
                      decision 0, and it is kept as the ablation that shows why
                      the adopted default is there: it is the variant whose
                      torque envelope fails at 8x disturbance.

Every variant differs from ``pir`` in exactly one knob, so a difference between
two rows is attributable.

Two scenarios, because the two source papers stress **different axes** and
neither benchmark exercises both:

``push``    phri2's own benchmark: a smooth sustained 20 N push.  This loads
            the feasibility axis (the torque envelope) and essentially does
            not load the passivity axis at all -- the motion is monotone and
            largely dissipative, so the tank harvests faster than the residual
            drains it and ``alpha_E`` never leaves 1.
``merged``  that push plus impedance_residual's own rejectable disturbance:
            three sinusoids at 0.9/1.4/1.9 Hz and a 12 N pulse placed
            deliberately between two 50 Hz manager ticks.  This is the
            scenario that loads both axes, and it is the one any claim about
            the passivity axis has to be made on.

Run::

    python3 run_pir_closed_loop.py --scenario merged
    python3 run_pir_closed_loop.py --envelope rho_0.28   # the infeasible case
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
import pir_controller as pk
from pir_controller import PIRConfig, PIRRealizationMPC, pir_servo_step

from verify_fr3_two_rate_benchmark import rejectable_force  # noqa: E402

VARIANTS = ("pir", "pir_manager_guard", "pir_no_tank", "zero_nominal")
#: Ablations kept for the studies rather than the headline comparison.
#: ``pir_nominal_auth`` is retained as an alias of ``pir`` so the Section 7/8
#: scripts and their stored JSON keep resolving after decision 0 made nominal
#: authorization the default.
FIX_VARIANTS = ("pir_no_nominal_auth", "pir_nominal_auth")
ALL_VARIANTS = VARIANTS + FIX_VARIANTS
SCENARIOS = ("push", "merged")

#: impedance_residual seeds the disturbance phases from the trial seed; fixed
#: here so every variant in a comparison sees the identical force history.
DISTURBANCE_SEED = 0


def external_force(t: float, scenario: str, phases: np.ndarray,
                   disturbance_scale: float = 1.0,
                   push_axis: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Return (total external force, the part the behaviour layer is about).

    The push is what the impedance behaviour is defined against; the
    oscillatory part is a disturbance.  Both are applied to the plant, and the
    QP forecasts their sum by zero-order hold -- phri2 freezes the human force,
    impedance_residual freezes ``disturbance_hat``, and this does both at once.
    """
    axis = pc.PUSH_AXIS if push_axis is None else np.asarray(push_axis, float)
    push = pc.human_force_at(t, axis=tuple(axis))
    if scenario == "push":
        return push, push
    if scenario == "merged":
        return push + rejectable_force(t, phases, disturbance_scale), push
    raise ValueError(scenario)


def run_variant(
    variant: str,
    k0: float,
    d0: float,
    envelope: str,
    scenario: str = "push",
    duration: float = pc.DURATION_S,
    tank_initial: float | None = None,
    disturbance_scale: float = 1.0,
    overrides: dict | None = None,
    pose: np.ndarray | None = None,
    push_axis: np.ndarray | None = None,
) -> dict:
    if variant not in ALL_VARIANTS:
        raise ValueError(variant)
    phases = np.random.default_rng(DISTURBANCE_SEED).uniform(-np.pi, np.pi, 3)
    cap = pc.torque_envelope(envelope)
    generator = pc.ImpedanceReference3D()

    env = pc.FR3MuJoCoEnv(timestep=0.001)
    env.reset(q=pc.nominal_pose(pose))
    _, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()

    # Each variant flips exactly one knob away from the adopted default.
    per_variant: dict[str, dict] = {
        "pir": {},
        "pir_manager_guard": {"auth_period_ticks": 20},
        "pir_no_tank": {"energy_authorization": False},
        "zero_nominal": {"k0": 0.0, "d0": 0.0},
        "pir_no_nominal_auth": {"nominal_authorization": False},
        "pir_nominal_auth": {},  # alias of the adopted default, kept for callers
    }
    # Precedence, low to high: the call's gains, then the variant's one knob,
    # then the study's explicit overrides.  Built as a dict rather than keyword
    # splats because a variant may legitimately override k0/d0 (zero_nominal
    # does), which duplicate keyword arguments cannot express.
    settings: dict = {"k0": k0, "d0": d0, "envelope": envelope,
                      "pose": None if pose is None else tuple(pc.nominal_pose(pose))}
    if tank_initial is not None:
        settings["tank_initial"] = tank_initial
    settings.update(per_variant[variant])
    settings.update(overrides or {})
    pir_cfg = PIRConfig(**settings)
    cfg = pir_cfg.resolved_mpc()
    mpc = PIRRealizationMPC(generator, pir_cfg)

    # The posture spring is centred on the interaction pose, not on Q_NEUTRAL:
    # otherwise a pose change is really a pose change plus a fight with the
    # null-space term.
    imp_params = pc.params_at(cfg, pose)
    axis_index = int(np.argmax(np.abs(
        pc.PUSH_AXIS if push_axis is None else np.asarray(push_axis, float))))
    mpc_every = max(1, round(cfg.dt / env.dt))
    n_steps = int(round(duration / env.dt))
    h = env.dt

    f_r_held = np.zeros(3)
    tank = pir_cfg.tank_initial
    held_alpha_E = 1.0
    previous_alpha_nom = 1.0
    n_infeasible = 0
    qp_ticks = 0

    keys3 = ("e", "v", "f_nom", "f_r_applied", "f_h", "a_id", "a_modelled",
             "r_reg", "r_con_qp", "r_con_fast", "r_auth", "closure")
    keys1 = ("time", "anchor_ratio", "tau_ratio", "budget_ratio", "alpha_tau",
             "alpha_E", "alpha_nom", "tank", "anchor_feasible", "tank_floor_ok")
    log: dict = {k: np.zeros((n_steps, 3)) for k in keys3}
    log.update({k: np.zeros(n_steps) for k in keys1})
    log["tau"] = np.zeros((n_steps, 7))

    last = {"r_reg": np.zeros(3), "r_con_qp": np.zeros(3)}

    for i in range(n_steps):
        t = env.time
        force, behaviour_force = external_force(t, scenario, phases,
                                               disturbance_scale, push_axis)
        dyn, state = env.get_dynamics_and_state(
            f_ext_override=np.concatenate([force, np.zeros(3)])
        )
        tau_base, J_v, d_known = pc.compute_tau_base(
            dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
        )
        Lam_inv = J_v @ np.linalg.inv(dyn.M) @ J_v.T + cfg.lambda_reg * np.eye(3)
        e = state.ee_pos - p_nominal
        v = state.ee_vel[:3]

        if i % mpc_every == 0:
            qp_ticks += 1
            try:
                out = mpc.control(
                    dyn, state, p_nominal, R_d,
                    force_forecast=np.tile(force, (cfg.horizon, 1)),
                    behaviour_forecast=np.tile(behaviour_force, (cfg.horizon, 1)),
                )
                f_r_held = out["residual_command"]
                last = {"r_reg": out["r_reg"], "r_con_qp": out["r_con_qp"]}
            except RuntimeError:
                # phri2 has no principled fallback here and drops to its
                # clipped reactive law (its own run_case documents the
                # cascading-runaway alternative).  The merged design does have
                # one: F_r = 0 is exactly the alpha -> 0 passive nominal the
                # Task 1 gate certified, so an infeasible solve degrades to a
                # certified behaviour rather than an improvised one.
                f_r_held = np.zeros(3)
                last = {"r_reg": np.zeros(3), "r_con_qp": np.zeros(3)}
                n_infeasible += 1

        servo = pir_servo_step(
            pir_cfg, tau_base, J_v, Lam_inv, e, v, f_r_held, tank, h, cap,
            held_alpha_E=(held_alpha_E if variant == "pir_manager_guard"
                          and i % pir_cfg.auth_period_ticks != 0 else None),
            previous_alpha_nom=previous_alpha_nom,
        )
        previous_alpha_nom = servo.alpha_nom
        if variant == "pir_manager_guard" and i % pir_cfg.auth_period_ticks == 0:
            held_alpha_E = servo.alpha_E
        tank = servo.tank

        # The behaviour is defined against the interaction force, not the
        # disturbance: a_id uses the push alone, while the plant and the QP
        # both see the total.  Under `push` the two coincide.
        a_id = generator.acceleration(np.concatenate([e, v]), behaviour_force)
        a_modelled = Lam_inv @ (servo.f_nom + servo.f_r_applied + force) + d_known

        log["time"][i] = t
        log["e"][i], log["v"][i], log["f_h"][i] = e, v, force
        log["f_nom"][i], log["f_r_applied"][i] = servo.f_nom, servo.f_r_applied
        log["a_id"][i], log["a_modelled"][i] = a_id, a_modelled
        log["r_reg"][i], log["r_con_qp"][i] = last["r_reg"], last["r_con_qp"]
        log["r_con_fast"][i], log["r_auth"][i] = servo.r_con_fast, servo.r_auth
        log["closure"][i] = (a_modelled - a_id - last["r_reg"] - last["r_con_qp"]
                             - servo.r_con_fast - servo.r_auth)
        log["tau"][i] = servo.tau
        log["anchor_ratio"][i] = servo.anchor_ratio
        log["tau_ratio"][i] = servo.tau_ratio
        log["budget_ratio"][i] = float(np.max(np.abs(J_v.T @ servo.f_nom) / cap))
        log["alpha_tau"][i], log["alpha_E"][i] = servo.alpha_tau, servo.alpha_E
        log["alpha_nom"][i] = servo.alpha_nom
        log["tank"][i] = servo.tank
        log["anchor_feasible"][i] = float(servo.anchor_feasible)
        log["tank_floor_ok"][i] = float(servo.tank_floor_ok)

        env.apply_torque(servo.tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        env.step()

    empirical = np.full_like(log["a_modelled"], np.nan)
    empirical[:-1] = np.diff(log["v"], axis=0) / h
    log["r_mod"] = empirical - log["a_modelled"]

    # Closure is only meaningful on QP ticks, where every term is evaluated at
    # the same state; between them r_reg/r_con_qp are held from the last solve.
    qp_mask = np.zeros(n_steps, dtype=bool)
    qp_mask[::mpc_every] = True

    summary = {
        "variant": variant,
        "envelope": envelope,
        "scenario": scenario,
        "pose": pc.nominal_pose(pose).tolist(),
        "pose_tag": pc.scenario_tag(pose, push_axis),
        # Which task-space row the workspace diagnostics are read off, so the
        # figure does not have to re-derive it from the push axis.
        "axis_index": axis_index,
        "K0_vector": pir_cfg.k0_vector().tolist(),
        "D0_vector": pir_cfg.d0_vector().tolist(),
        "nominal_authorization": pir_cfg.nominal_authorization,
        "nominal_reauth_rate": pir_cfg.nominal_reauth_rate,
        "tank_initial": pir_cfg.tank_initial,
        "disturbance_scale": disturbance_scale,
        "K0": k0 if variant != "phri2" else None,
        "D0": d0 if variant != "phri2" else None,
        # --- what Section 7 owed: rows (1) and (4), re-measured in closed loop
        "diag1_anchor_ratio_closed_loop": float(log["anchor_ratio"].max()),
        "diag4_budget_ratio_closed_loop": float(log["budget_ratio"].max()),
        # --- Merged Lemma 1: precondition in, conclusion out
        # With nominal_authorization on, the precondition is no longer a
        # hypothesis the runtime must be handed: alpha_nom enforces it.  Report
        # both the raw anchor (what the precondition WOULD have been) and
        # whether the servo actually kept the applied torque legal.
        "lemma1_precondition_holds": bool(log["anchor_feasible"].all()),
        "alpha_nom_min": float(log["alpha_nom"].min()),
        "alpha_nom_active_fraction": float(np.mean(log["alpha_nom"] < 1 - 1e-10)),
        "lemma1_conclusion_max_tau_ratio": float(log["tau_ratio"].max()),
        "lemma1_conclusion_holds": bool(log["tau_ratio"].max() <= 1.0 + 1e-9),
        "tank_min": float(log["tank"].min()),
        "tank_floor": pir_cfg.tank_minimum,
        "tank_floor_holds": bool(log["tank_floor_ok"].all()),
        "tank_floor_breach_ticks": int((~log["tank_floor_ok"].astype(bool)).sum()),
        # --- behaviour
        "max_abs_e_axis_m": float(np.abs(log["e"][:, axis_index]).max()),
        "workspace_bound_m": pc.WORKSPACE_BOUND_M,
        "rms_realization_residual": float(
            np.sqrt(np.mean(np.sum((log["a_modelled"] - log["a_id"]) ** 2, axis=1)))),
        # The behaviour the controller is trying to render.  This is the right
        # denominator for "how much of the intended behaviour did this axis
        # remove": the four residual terms sum to the NET deviation but can
        # oppose one another, so an individual term normalised by the net can
        # exceed 1 and is not a share of anything.
        "a_id_rms": float(np.sqrt(np.mean(np.sum(log["a_id"] ** 2, axis=1)))),
        "authorization_active_fraction": float(
            np.mean((log["alpha_tau"] < 1 - 1e-10) | (log["alpha_E"] < 1 - 1e-10)
                    | (log["alpha_nom"] < 1 - 1e-10))),
        "alpha_E_min": float(log["alpha_E"].min()),
        "alpha_tau_min": float(log["alpha_tau"].min()),
        # --- how much of the command is the nominal rather than the residual.
        # This is the note's Section 7 boundary ("K0/D0 large => floor eats
        # budget") turned into a number: if the nominal is most of the command
        # and most of the anchor budget, the merged controller is a stiff PD
        # with a small predictive correction, not a predictive controller.
        "f_nom_rms_N": float(np.sqrt(np.mean(np.sum(log["f_nom"] ** 2, axis=1)))),
        "f_r_rms_N": float(np.sqrt(np.mean(np.sum(log["f_r_applied"] ** 2, axis=1)))),
        "residual_share_of_command": float(
            np.sqrt(np.mean(np.sum(log["f_r_applied"] ** 2, axis=1)))
            / max(1e-12, np.sqrt(np.mean(np.sum(
                (log["f_nom"] + log["f_r_applied"]) ** 2, axis=1))))),
        # Budget left for the residual once the anchor is paid for.
        "anchor_headroom": float(1.0 - log["anchor_ratio"].max()),
        # --- the four-term residual
        "r_auth_rms": float(np.sqrt(np.mean(np.sum(log["r_auth"] ** 2, axis=1)))),
        "r_con_fast_rms": float(np.sqrt(np.mean(np.sum(log["r_con_fast"] ** 2, axis=1)))),
        "decomposition_closure_max_on_qp_ticks": float(
            np.abs(log["closure"][qp_mask]).max()),
        "qp_infeasible_solves": n_infeasible,
        "qp_solves": qp_ticks,
    }
    return {"summary": summary, "log": log}


def make_figure(results: dict, outdir: Path, envelope: str, scenario: str,
                suffix: str = "") -> Path:
    order = [v for v in VARIANTS if v in results]
    axis_index = results[order[0]]["summary"]["axis_index"]
    axis_label = "xyz"[axis_index]
    colors = {"pir": "tab:blue", "pir_manager_guard": "tab:orange",
              "pir_no_tank": "tab:green", "zero_nominal": "0.45"}
    fig, axes = plt.subplots(4, 1, figsize=(7.2, 9.4), sharex=True)
    for name in order:
        log = results[name]["log"]
        t = log["time"]
        c = colors[name]
        axes[0].plot(t, log["anchor_ratio"], color=c, lw=1.2, label=name)
        axes[1].plot(t, log["tau_ratio"], color=c, lw=1.2)
        axes[2].plot(t, log["tank"], color=c, lw=1.2)
        axes[3].plot(t, log["e"][:, axis_index], color=c, lw=1.2)

    axes[0].axhline(1.0, color="k", ls="--", lw=1.0)
    axes[0].set_ylabel(r"anchor $\|a\|_\infty/\bar\tau$")
    axes[0].set_title("Merged Lemma 1's precondition (row 1), measured in closed loop",
                      fontsize=9)
    axes[1].axhline(1.0, color="k", ls="--", lw=1.0)
    axes[1].set_ylabel(r"applied $\|\tau\|_\infty/\bar\tau$")
    axes[1].set_title("Merged Lemma 1's conclusion", fontsize=9)
    floor = PIRConfig().tank_minimum
    axes[2].axhline(floor, color="k", ls="--", lw=1.0)
    axes[2].set_ylabel("tank $E$  [J]")
    axes[2].set_title(rf"energy tank vs its floor $E_{{\min}} = {floor}$ J", fontsize=9)
    bound = pc.WORKSPACE_BOUND_M
    for sign in (1, -1):
        axes[3].axhline(sign * bound, color="k", ls="--", lw=1.0)
    axes[3].set_ylabel(rf"$e_{axis_label}$  [m]")
    axes[3].set_xlabel("time [s]")
    axes[3].set_title(f"workspace excursion on the push axis vs the {bound} m bound",
                      fontsize=9)
    for ax in axes:
        ax.grid(alpha=0.25)
    axes[0].legend(fontsize=8, ncol=2)
    fig.suptitle(f"PIR merged controller, scenario '{scenario}', "
                 f"envelope '{envelope}'", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / f"pir_closed_loop{suffix}_{scenario}_{envelope}.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envelope", default="derated_joint4", choices=pc.ENVELOPE_NAMES)
    parser.add_argument("--scenario", default="push", choices=SCENARIOS)
    parser.add_argument("--k0", type=float, default=380.0)
    parser.add_argument("--d0", type=float, default=29.07)
    parser.add_argument("--outdir", type=Path, default=pc.RESULTS)
    parser.add_argument("--pose", type=float, nargs=3, metavar=("Q2", "Q4", "Q6"),
                        help="interaction pose as (q2, q4, q6); other joints stay "
                             "at Q_NEUTRAL. Default: phri2's own pose.")
    parser.add_argument("--push-axis", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="push direction. Default: phri2's own -z.")
    parser.add_argument("--tag", default="",
                        help="suffix for output filenames, so a pose study does "
                             "not overwrite the neutral-pose result")
    args = parser.parse_args()

    pose = None
    if args.pose is not None:
        pose = pc.Q_NEUTRAL.copy()
        pose[1], pose[3], pose[5] = args.pose
    push_axis = np.array(args.push_axis) if args.push_axis else None
    suffix = f"_{args.tag}" if args.tag else ""

    results = {v: run_variant(v, args.k0, args.d0, args.envelope, args.scenario,
                              pose=pose, push_axis=push_axis)
               for v in VARIANTS}
    figure = make_figure(results, args.outdir, args.envelope, args.scenario,
                         suffix=suffix)

    # The replay estimates the Task 1 gate used, for the comparison.  Must be
    # the trajectory for THIS scenario, or the comparison is against a
    # different robot configuration entirely.
    traj = pc.load_or_generate_trajectory(pose=pose, push_axis=push_axis)
    cap = pc.torque_envelope(args.envelope)
    replay = pc.replay_diagnostics(traj, args.k0, args.d0, cap)

    out = {
        "operating_point": {"K0": args.k0, "D0": args.d0, "envelope": args.envelope,
                            "scenario": args.scenario,
                            "pose": pc.nominal_pose(pose).tolist(),
                            "pose_tag": pc.scenario_tag(pose, push_axis)},
        "replay_estimate": {"diag1_anchor_ratio": replay["anchor_ratio"],
                            "diag4_budget_ratio": replay["budget_ratio"]},
        "variants": {v: results[v]["summary"] for v in VARIANTS},
    }
    path = args.outdir / f"pir_closed_loop{suffix}_{args.scenario}_{args.envelope}.json"
    path.write_text(json.dumps(out, indent=2))

    print(f"operating point: K0 = {args.k0} N/m, D0 = {args.d0} N.s/m, "
          f"envelope = {args.envelope}, scenario = {args.scenario}")
    print(f"pose: {np.round(pc.nominal_pose(pose), 3).tolist()}  "
          f"({pc.scenario_tag(pose, push_axis)})")
    print(f"\nrow (1) anchor ratio  -- replay estimate (Task 1): {replay['anchor_ratio']:.4f}")
    print(f"row (4) budget ratio  -- replay estimate (Task 1): {replay['budget_ratio']:.4f}")
    for v in VARIANTS:
        s = results[v]["summary"]
        print(f"\n[{v}]")
        print(f"  row (1) anchor ratio, closed loop : {s['diag1_anchor_ratio_closed_loop']:.4f}"
              f"   (precondition holds: {s['lemma1_precondition_holds']})")
        print(f"  row (4) budget ratio, closed loop : {s['diag4_budget_ratio_closed_loop']:.4f}")
        print(f"  Lemma 1 conclusion  max |tau|/cap : {s['lemma1_conclusion_max_tau_ratio']:.4f}"
              f"   (holds: {s['lemma1_conclusion_holds']})")
        print(f"  tank min / floor                  : {s['tank_min']:.4f} / {s['tank_floor']}"
              f"   (floor holds: {s['tank_floor_holds']}, breaches: {s['tank_floor_breach_ticks']} ticks)")
        print(f"  max |e_z| / bound                 : {s['max_abs_e_axis_m'] * 1e3:.1f} mm / "
              f"{s['workspace_bound_m'] * 1e3:.0f} mm")
        print(f"  RMS realization residual          : {s['rms_realization_residual']:.4f} m/s^2")
        print(f"  authorization active              : {s['authorization_active_fraction'] * 100:.1f}% of ticks"
              f"  (min alpha_tau {s['alpha_tau_min']:.3f}, min alpha_E {s['alpha_E_min']:.3f})")
        print(f"  F_nom RMS / F_r RMS               : {s['f_nom_rms_N']:.2f} / {s['f_r_rms_N']:.2f} N"
              f"   (residual is {s['residual_share_of_command'] * 100:.1f}% of the command)")
        print(f"  anchor headroom left for F_r      : {s['anchor_headroom'] * 100:.1f}% of the cap")
        print(f"  r_auth RMS / r_con_fast RMS       : {s['r_auth_rms']:.4f} / {s['r_con_fast_rms']:.4f} m/s^2")
        print(f"  4-term closure on QP ticks        : {s['decomposition_closure_max_on_qp_ticks']:.2e}")
        print(f"  QP infeasible solves              : {s['qp_infeasible_solves']} / {s['qp_solves']}")
    print(f"\nwrote {path}\nwrote {figure}")


if __name__ == "__main__":
    main()
