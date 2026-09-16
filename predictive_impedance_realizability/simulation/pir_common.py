"""Shared machinery for the Predictive Interaction Realizability (PIR) scan.

This module deliberately owns **no** FR3 dynamics of its own.  Every mass
matrix, Jacobian, task-space inertia, bias torque and null-space projector it
uses is imported from the two existing studies in this repository:

* ``simulation/fr3_mujoco.py``      -- plant, ``TAU_LIMIT``, ``Q_NEUTRAL``
* ``simulation/fr3_impedance.py``   -- ``FrankaDynamics`` / ``RobotState``
* ``imp_reference/simulation/fr3_interaction_dynamics_mpc.py``
                                    -- ``compute_tau_base`` (the phri2
                                       feedforward + orientation + null-space
                                       torque), ``FR3MPCConfig``,
                                       ``FR3RealizationMPC``
* ``imp_reference/simulation/run_fr3_experiments.py``
                                    -- ``human_force_at`` (the 20 N push)

A second, independent FR3 model would make the scan inconsistent with the
papers it is meant to gate, so there is exactly one source for each quantity.

Terminology follows the synthesis note:

    F_cmd = F_nom + F_r,        F_nom = -K0 e - D0 v

``F_nom`` is a small behavior-agnostic passive spring-damper (the *passivity
floor*, not the behavior); ``F_r`` is the residual the realization QP
optimizes and the energy tank authorizes.  The **anchor** is the torque that
remains when the residual is fully de-authorized (alpha -> 0):

    a = tau_base + J_v^T F_nom

Merged Lemma 1's first precondition is ``|a| <= tau_cap`` at every tick.
Whether that precondition is satisfiable *is* the ``(K0, D0)`` design problem.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/phri_pir_mpl")

HERE = Path(__file__).resolve().parent
PIR_DIR = HERE.parent
REPO = PIR_DIR.parent
SHARED_SIM = REPO / "simulation"
PHRI2_SIM = REPO / "imp_reference" / "simulation"

for _p in (str(SHARED_SIM), str(PHRI2_SIM)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fr3_mujoco import FR3MuJoCoEnv, TAU_LIMIT  # noqa: E402
from fr3_interaction_dynamics_mpc import (  # noqa: E402
    FR3MPCConfig,
    FR3RealizationMPC,
    ImpedanceReference3D,
    combine_full_torque,
    compute_tau_base,
    make_default_impedance_params,
)
from run_fr3_experiments import human_force_at  # noqa: E402

RESULTS = PIR_DIR / "results"


# ---------------------------------------------------------------------------
# Scenario constants -- all pulled from the two source studies, none invented
# ---------------------------------------------------------------------------

#: phri2's torque-activation study derates joint 4 (zero-based index 3) from
#: the FR3's 87 N.m to 31.5 N.m; impedance_residual instead derates *every*
#: joint to a rho = 0.28 continuous envelope.  The merged Lemma 1 precondition
#: is stated against "rho tau_max", so rho_0.28 is the primary envelope and
#: the phri2 envelope is carried as the sensitivity case.  They are genuinely
#: different stress cases and the scan reports both rather than silently
#: picking one.
DERATED_JOINT = 3  # run_torque_activation_experiment.DERATED_JOINT
DERATED_LIMIT_NM = 31.5  # run_torque_activation_experiment.DERATED_LIMIT_NM
RHO = 0.28  # verify_fr3_two_rate_benchmark.Config.torque_margin


def torque_envelope(name: str) -> np.ndarray:
    """Return the per-joint torque cap (N.m) for a named stress case."""
    if name == "rho_0.28":
        return RHO * TAU_LIMIT
    if name == "derated_joint4":
        cap = TAU_LIMIT.copy()
        cap[DERATED_JOINT] = DERATED_LIMIT_NM
        return cap
    raise ValueError(f"unknown torque envelope {name!r}")


ENVELOPE_NAMES = ("rho_0.28", "derated_joint4")

#: The 20 N sustained push: magnitude, axis, ramp and hold all come from
#: run_fr3_experiments.human_force_at's defaults.
PUSH_MAGNITUDE_N = 20.0
PUSH_AXIS = np.array([0.0, 0.0, -1.0])
PUSH_AXIS_INDEX = 2  # the z row, i.e. phri2's |e_z| bound

#: Workspace bound: FR3MPCConfig.position_limit.
WORKSPACE_BOUND_M = FR3MPCConfig().position_limit

#: phri2's own trajectory length for the FR3 study.
DURATION_S = 6.0


# ---------------------------------------------------------------------------
# Reference trajectory (generated once, replayed by every grid cell)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Trajectory:
    """Per-tick replay of the phri2 FR3 20 N benchmark.

    Everything the anchor needs is stored, so a grid cell costs a handful of
    matrix products and no simulation at all.
    """

    time: np.ndarray  # (T,)
    q: np.ndarray  # (T, 7)
    dq: np.ndarray  # (T, 7)
    e: np.ndarray  # (T, 3) ee_pos - p_nominal
    v: np.ndarray  # (T, 3) translational ee velocity
    tau_base: np.ndarray  # (T, 7) phri2's feedforward + orientation + null-space
    J_v: np.ndarray  # (T, 3, 7) translational Jacobian
    f_h: np.ndarray  # (T, 3) applied human force
    tau_realized: np.ndarray  # (T, 7) torque phri2's own MPC actually applied

    @property
    def n_ticks(self) -> int:
        return self.time.shape[0]

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            time=self.time,
            q=self.q,
            dq=self.dq,
            e=self.e,
            v=self.v,
            tau_base=self.tau_base,
            J_v=self.J_v,
            f_h=self.f_h,
            tau_realized=self.tau_realized,
        )

    @classmethod
    def load(cls, path: Path) -> "Trajectory":
        with np.load(path) as z:
            return cls(**{k: z[k] for k in z.files})


def generate_reference_trajectory(
    duration: float = DURATION_S,
    envelope: str = "derated_joint4",
) -> Trajectory:
    """Run phri2's own predictive realization controller and log the replay.

    The trajectory is generated by the **phri2 MPC** (impedance generator,
    horizon-wide torque constraint), not by any PIR controller: the scan asks
    whether a passive nominal would have fitted inside the budget *along the
    operating trajectory the existing paper already reports*.  That is a
    replay diagnostic, and its one real limitation is stated plainly in the
    draft: the closed loop under a PIR controller visits a slightly different
    ``(q, qdot)``.  ``pir_closed_loop_check.py`` quantifies that gap.
    """
    cap = torque_envelope(envelope)
    cfg = FR3MPCConfig(tau_max=cap)
    generator = ImpedanceReference3D()
    imp_params = make_default_impedance_params(cfg)

    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    _, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()

    mpc = FR3RealizationMPC(generator, cfg)
    mpc_every = max(1, round(cfg.dt / env.dt))
    n_steps = int(round(duration / env.dt))

    cols: dict[str, list] = {k: [] for k in
                             ("time", "q", "dq", "e", "v", "tau_base", "J_v", "f_h", "tau_realized")}
    f_cmd_held = np.zeros(3)

    for i in range(n_steps):
        t = env.time
        force = human_force_at(t)
        dyn, state = env.get_dynamics_and_state(
            f_ext_override=np.concatenate([force, np.zeros(3)])
        )
        tau_base, J_v, d_known = compute_tau_base(
            dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
        )
        if i % mpc_every == 0:
            try:
                f_cmd_held = mpc.control(dyn, state, p_nominal, R_d,
                                         np.tile(force, (cfg.horizon, 1))).command
            except RuntimeError:
                # phri2's own recursive-feasibility caveat; hold the last
                # command rather than dropping to zero (see its run_case).
                pass
        tau = combine_full_torque(tau_base, J_v, f_cmd_held)

        cols["time"].append(t)
        cols["q"].append(state.q.copy())
        cols["dq"].append(state.dq.copy())
        cols["e"].append(state.ee_pos - p_nominal)
        cols["v"].append(state.ee_vel[:3].copy())
        cols["tau_base"].append(tau_base.copy())
        cols["J_v"].append(J_v.copy())
        cols["f_h"].append(force.copy())
        cols["tau_realized"].append(tau.copy())

        env.apply_torque(tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        env.step()

    return Trajectory(**{k: np.asarray(v) for k, v in cols.items()})


def load_or_generate_trajectory(
    cache: Path | None = None,
    duration: float = DURATION_S,
    envelope: str = "derated_joint4",
    refresh: bool = False,
) -> Trajectory:
    cache = cache or (RESULTS / "pir_reference_trajectory.npz")
    if cache.exists() and not refresh:
        return Trajectory.load(cache)
    traj = generate_reference_trajectory(duration=duration, envelope=envelope)
    traj.save(cache)
    return traj


# ---------------------------------------------------------------------------
# The nominal law and the anchor
# ---------------------------------------------------------------------------


def nominal_force(k: float, d: float, e: np.ndarray, v: np.ndarray) -> np.ndarray:
    """F_nom = -K0 e - D0 v, isotropic K0 = k I, D0 = d I."""
    return -k * np.asarray(e) - d * np.asarray(v)


def replay_diagnostics(traj: Trajectory, k: float, d: float, cap: np.ndarray) -> dict:
    """Diagnostics 1 and 4 of the synthesis note, along the replayed trajectory.

    Returns the two per-joint-normalized infinity-norm maxima:

    * ``anchor_ratio`` = max_t ||tau_base + J_v^T F_nom||_inf / cap  -- (a)
    * ``budget_ratio`` = max_t ||J_v^T F_nom||_inf / cap             -- (d)
    """
    f_nom = nominal_force(k, d, traj.e, traj.v)  # (T, 3)
    tau_nom = np.einsum("tij,ti->tj", traj.J_v, f_nom)  # J_v^T F_nom, (T, 7)
    anchor = traj.tau_base + tau_nom
    anchor_ratio_t = np.max(np.abs(anchor) / cap[None, :], axis=1)
    budget_ratio_t = np.max(np.abs(tau_nom) / cap[None, :], axis=1)
    worst = int(np.argmax(anchor_ratio_t))
    return {
        "anchor_ratio": float(anchor_ratio_t.max()),
        "budget_ratio": float(budget_ratio_t.max()),
        "anchor_worst_joint": int(np.argmax(np.abs(anchor[worst]) / cap)),
        "anchor_worst_time_s": float(traj.time[worst]),
    }


def base_only_anchor_ratio(traj: Trajectory, cap: np.ndarray) -> dict:
    """``max_t ||tau_base||_inf / cap`` -- the k = d = 0 floor of diagnostic 1.

    This is the part of the anchor budget that no ``(K0, D0)`` choice can
    reduce; if it already exceeds 1 the design problem is infeasible for
    *every* cell and the scan's answer is structural, not a tuning failure.
    """
    ratio_t = np.max(np.abs(traj.tau_base) / cap[None, :], axis=1)
    worst = int(np.argmax(ratio_t))
    return {
        "base_only_anchor_ratio": float(ratio_t.max()),
        "base_only_worst_joint": int(np.argmax(np.abs(traj.tau_base[worst]) / cap)),
    }


# ---------------------------------------------------------------------------
# Diagnostic 3 -- the alpha -> 0 fallback equilibrium
# ---------------------------------------------------------------------------


def analytic_fallback_displacement(k: float, magnitude: float = PUSH_MAGNITUDE_N) -> float:
    """|e| at the F_nom-only equilibrium: K0 e = F_h, so |e| = |F_h| / k.

    Exact for the isotropic case, and independent of ``d`` (damping does no
    work at rest).  ``run_fallback_equilibrium`` confirms it on the plant.
    """
    return float(magnitude / k)


#: The alpha -> 0 fallback is a lightly damped task-space spring: at k = 344
#: N/m against a task inertia of ~4.6 kg the natural period is ~0.7 s, and at
#: d = 2 N.s/m the damping ratio is ~0.03, so the decay time constant is
#: several seconds.  phri2's own 2.0 s hold is far too short to read an
#: equilibrium off, and averaging over a sub-period window biases the estimate
#: by a visible fraction of the 0.06 m bound (it made the measured
#: displacement look *smaller* at low damping, which is backwards).  The push
#: shape and magnitude are phri2's ``human_force_at``; only the hold is
#: extended, and the average is taken over an integer-multiple-of-period
#: window so the residual oscillation cancels rather than biasing.
FALLBACK_HOLD_S = 6.0
FALLBACK_DURATION_S = 9.0
FALLBACK_AVERAGING_WINDOW_S = 1.5
SETTLED_SPEED_M_PER_S = 5.0e-3


def run_fallback_equilibrium(
    k: float,
    d: float,
    duration: float = FALLBACK_DURATION_S,
    hold: float = FALLBACK_HOLD_S,
    window: float = FALLBACK_AVERAGING_WINDOW_S,
) -> dict:
    """Closed-loop MuJoCo run of the alpha -> 0 law: ``tau = tau_base + J_v^T F_nom``.

    This is *literally* what the merged controller degenerates to when the
    energy authorization fully de-authorizes the residual, so it is the right
    way to measure diagnostic 3 rather than trusting the static algebra alone.

    Returns both the plateau-averaged displacement (diagnostic 3) and a
    ``settled`` flag, so a cell whose fallback never actually converges inside
    the push cannot be silently reported as an equilibrium.
    """
    cfg = FR3MPCConfig()
    imp_params = make_default_impedance_params(cfg)
    env = FR3MuJoCoEnv(timestep=0.001)
    env.reset()
    _, state0 = env.get_dynamics_and_state()
    p_nominal = state0.ee_pos.copy()
    R_d = state0.ee_rot.copy()

    n_steps = int(round(duration / env.dt))
    e_log = np.zeros((n_steps, 3))
    v_log = np.zeros((n_steps, 3))
    anchor_log = np.zeros((n_steps, 7))
    t_log = np.zeros(n_steps)

    for i in range(n_steps):
        t = env.time
        force = human_force_at(t, hold=hold)
        dyn, state = env.get_dynamics_and_state(
            f_ext_override=np.concatenate([force, np.zeros(3)])
        )
        tau_base, J_v, _ = compute_tau_base(
            dyn, state, R_d, imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
        )
        e = state.ee_pos - p_nominal
        v = state.ee_vel[:3]
        tau = tau_base + J_v.T @ nominal_force(k, d, e, v)

        t_log[i] = t
        e_log[i] = e
        v_log[i] = v
        anchor_log[i] = tau

        env.apply_torque(tau)
        env.apply_ee_wrench(np.concatenate([force, np.zeros(3)]))
        env.step()

    # Average over the tail of the force plateau, before the release ramp.
    t_on, ramp = 1.0, 0.25
    t_off = t_on + ramp + hold
    plateau = (t_log >= t_off - window) & (t_log < t_off)
    if not plateau.any():  # pragma: no cover - only if hold is made tiny
        plateau = t_log >= t_log[-1] - window
    e_ss = e_log[plateau].mean(axis=0)
    speed_in_window = float(np.abs(v_log[plateau, PUSH_AXIS_INDEX]).max())
    return {
        "e_ss": e_ss,
        "e_ss_axis": float(abs(e_ss[PUSH_AXIS_INDEX])),
        "e_ss_norm": float(np.linalg.norm(e_ss)),
        "e_ss_spread_axis": float(
            e_log[plateau, PUSH_AXIS_INDEX].max() - e_log[plateau, PUSH_AXIS_INDEX].min()
        ),
        "e_peak_axis": float(np.abs(e_log[:, PUSH_AXIS_INDEX]).max()),
        "settled": bool(speed_in_window < SETTLED_SPEED_M_PER_S),
        "window_peak_speed": speed_in_window,
        "anchor_max_abs": np.abs(anchor_log).max(axis=0),
        "time": t_log,
        "e": e_log,
        "v": v_log,
        "anchor": anchor_log,
    }
