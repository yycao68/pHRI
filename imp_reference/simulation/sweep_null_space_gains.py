"""Null-space centering gain sweep backing the claim in paper.md Section 6.1.

Section 6.1 states that the original k_null=10, d_null=2 gains let joint
configuration drift more than a radian from Q_NEUTRAL over a full 6 s
benchmark run -- in every one of the four conditions, not three -- and that
k_null=40, d_null=8 substantially controls it. That claim was previously
backed only by ad hoc interactive sessions with no saved script or
artifact; this script is the reproducible version, run over the same four
(generator, controller) conditions as run_fr3_experiments.py's main
benchmark.

For each (k_null, d_null) pair and each condition, this script tracks:
  - max_q_dev: the largest ||q - Q_NEUTRAL|| observed over the full run,
    the quantity Section 6.1's claim is directly about;
  - max_abs_position_m: a coarse proxy for whether a stronger gain
    suppresses the generator's own requested displacement as a side
    effect (it does, visibly, at k_null=100/d_null=20 for admittance).
    This is only a proxy, not a clean measurement of generator damping in
    isolation: for the predictive conditions, position is independently
    constrained by the workspace box, so its peak cannot isolate the
    gain's effect from the constraint's; even for the reactive
    conditions, the qualitative decoupling failure this sweep is
    diagnosing (approximate, regularization-limited, not exact) is itself
    what couples null-space gain to task-space displacement, so the two
    effects are not separable from this metric alone.
  - max_tau_over_limit_Nm and n_infeasible_solves: reported for every
    condition, not assumed zero. The reactive (clipped) controller has no
    QP and therefore no hard torque constraint at all -- unlike the
    predictive conditions, which never violate at any gain in this sweep,
    the reactive impedance condition at the original k_null=10, d_null=2
    gains is actuator-infeasible, exceeding tau_max by about 1.6 Nm. This
    sweep does not soften or hide that: it is direct evidence for why a
    hard, QP-enforced torque constraint (Section 6.1) is not a
    formality -- an under-tuned auxiliary term can push an unconstrained
    reactive law past the actuator limit even when the interaction task
    itself is unremarkable.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from dataclasses import asdict

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_imp_reference_mpl")

import numpy as np

import sys

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation"))

from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL  # noqa: E402

from fr3_interaction_dynamics_mpc import (  # noqa: E402
    AdmittanceReference3D,
    FR3MPCConfig,
    FR3RealizationMPC,
    ImpedanceReference3D,
    combine_full_torque,
    compute_tau_base,
    fr3_clipped_reference_command,
    make_default_impedance_params,
)
from run_fr3_experiments import human_force_at  # noqa: E402

GAIN_SWEEP = [(10.0, 2.0), (20.0, 4.0), (40.0, 8.0), (60.0, 12.0), (100.0, 20.0)]
DURATION = 6.0


def run_condition(generator, controller_kind: str, k_null: float, d_null: float) -> dict:
    env = FR3MuJoCoEnv(timestep=0.001)
    cfg = FR3MPCConfig(k_null=k_null, d_null=d_null)
    imp_params = make_default_impedance_params(cfg)
    dyn0, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()
    mpc = FR3RealizationMPC(generator, cfg) if controller_kind == "mpc" else None
    mpc_every = max(1, round(cfg.dt / env.dt))

    f_cmd = np.zeros(3)
    max_q_dev = 0.0
    max_abs_position = 0.0
    max_tau_over_limit = 0.0
    n_infeasible = 0

    for i in range(int(DURATION / env.dt)):
        t = env.time
        force = human_force_at(t)
        dyn, state = env.get_dynamics_and_state(f_ext_override=np.concatenate([force, np.zeros(3)]))
        tau_base, J_v, d_known = compute_tau_base(dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg)

        if i % mpc_every == 0:
            if controller_kind == "mpc":
                forecast = np.tile(force, (cfg.horizon, 1))
                try:
                    step = mpc.control(dyn, state, p_nominal, R_d, forecast)
                    f_cmd = step.command
                except RuntimeError:
                    n_infeasible += 1
                    f_cmd = fr3_clipped_reference_command(
                        generator, dyn, state, p_nominal, force, cfg, f_cmd, d_known
                    )
            else:
                f_cmd = fr3_clipped_reference_command(
                    generator, dyn, state, p_nominal, force, cfg, f_cmd, d_known
                )

        q_dev = float(np.linalg.norm(state.q - np.asarray(Q_NEUTRAL)))
        max_q_dev = max(max_q_dev, q_dev)
        max_abs_position = max(max_abs_position, float(np.max(np.abs(state.ee_pos - p_nominal))))

        tau = combine_full_torque(tau_base, J_v, f_cmd)
        max_tau_over_limit = max(max_tau_over_limit, float(np.max(np.abs(tau) - cfg.tau_max)))

        env.apply_torque(tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        env.step()

    return {
        "max_q_dev_rad": max_q_dev,
        "max_abs_position_m": max_abs_position,
        "max_tau_over_limit_Nm": max_tau_over_limit,
        "n_infeasible_solves": n_infeasible,
    }


def main() -> None:
    generators = {"impedance": ImpedanceReference3D(), "admittance": AdmittanceReference3D()}
    base_cfg = FR3MPCConfig()
    report = {
        "metadata": {
            "schema_version": 1,
            "duration_s": DURATION,
            "simulation_timestep_s": 0.001,
            "gain_pairs": [
                {"k_null": k_null, "d_null": d_null}
                for k_null, d_null in GAIN_SWEEP
            ],
            "controller_configuration": {
                key: (value.tolist() if isinstance(value, np.ndarray) else value)
                for key, value in asdict(base_cfg).items()
                if key not in {"k_null", "d_null"}
            },
            "force_profile": {
                "magnitude_N": 20.0,
                "axis": [0.0, 0.0, -1.0],
                "onset_s": 1.0,
                "ramp_s": 0.25,
                "hold_s": 2.0,
                "forecast": "measured force held constant over the horizon",
            },
            "generators": {
                name: asdict(generator) for name, generator in generators.items()
            },
            "metric_definitions": {
                "max_q_dev_rad": "max over the run of ||q - Q_NEUTRAL||_2",
                "max_abs_position_m": "max over time and Cartesian components of |p - p_nominal|",
                "max_tau_over_limit_Nm": "max(0, max over time and joints of |tau|-tau_max)",
                "n_infeasible_solves": "number of predictive solves that invoked reactive fallback",
            },
        }
    }
    for k_null, d_null in GAIN_SWEEP:
        key = f"k_null={k_null:g}_d_null={d_null:g}"
        report[key] = {}
        for gen_name, generator in generators.items():
            for controller_kind in ("mpc", "clipped"):
                condition = f"{gen_name}_{controller_kind}"
                result = run_condition(generator, controller_kind, k_null, d_null)
                report[key][condition] = result
                print(f"{key} {condition}: {result}")

    output_dir = Path(__file__).resolve().parents[1] / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "null_space_gain_sweep.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    print(f"Saved results to {output_dir / 'null_space_gain_sweep.json'}")


if __name__ == "__main__":
    main()
