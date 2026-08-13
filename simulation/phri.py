"""
FR3 pHRI Benchmark — Unified Driver
===================================

Combines the three former scripts into one file with subcommands:

    python3 phri.py compare         # paper D1/D2/D3/D7 comparison plot
                                    #   (was fr3_mpc_comparison.py)
    python3 phri.py focused         # focused 5-controller + frequency plots
                                    #   (was fr3_focused_comparison.py)
    python3 phri.py video           # rendered MP4 walkthrough
                                    #   (was fr3_phri_video.py)

Scenario (shared by all modes):
    Reference  : 3-D circular trajectory in the xz-plane,
                 p_d(t) = CENTER + R·[cos ωt, 0, sin ωt],  ω = 2π/8  (8 s period)
    Human force: step wrench F_h = [0, 0, -15] N applied t = 3–6 s of every cycle
    Inner loop : 1 kHz feedforward for every controller; QP rate per controller

Controllers (canonical names; detection is substring-based):
    Impedance, Admittance, PI Impedance,
    DI-MPC 100 Hz, DI-MPC + Kalman 100 Hz,
    DI-MPC 500 Hz, DI-MPC + Kalman 500 Hz

Examples:
    python3 phri.py compare --no-viewer --cycles 3
    python3 phri.py focused
    python3 phri.py video --cycles 2 --fps 30
    python3 phri.py video --controllers "Impedance" "DI-MPC + Kalman 500 Hz"
"""

from __future__ import annotations
import sys
import argparse
import time
from collections import deque
from pathlib import Path

import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')   # non-interactive backend — required off the main thread
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

SIM_DIR = Path(__file__).parent.parent / "simulation"
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(Path(__file__).parent))   # for impedance_mpc (same dir)

from fr3_impedance import (
    make_impedance_params, cartesian_impedance_control, RobotState,
    AdmittanceController, make_admittance_params,
    build_operational_space_model,
)
from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL
from impedance_mpc import ImpedanceMPCController, ImpedanceMPCParams
from so3_utils import rotation_error_matrix


# ===========================================================================
#  Shared scenario: trajectory + human force
# ===========================================================================

CENTER  = np.array([0.45, 0.0, 0.45])   # circle centre (world frame)
RADIUS  = 0.12                          # m — overridden per mode / via --radius
OMEGA   = 2.0 * np.pi / 8.0             # rad/s  (8-second period)
PERIOD  = 2.0 * np.pi / OMEGA           # s      (= 8 s)
T_RAMP  = 1.5                           # s — smooth ramp-up from rest

T_FORCE_ON  = 3.0                       # s
T_FORCE_OFF = 6.0                       # s
F_HUMAN     = np.array([0.0, 0.0, -15.0])   # N (downward z)


def circular_ref(t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (p_d, dp_d, ddp_d) at time t. Uses the module-global RADIUS,
    which each mode's main sets before running."""
    ramp   = min(1.0, t / T_RAMP) if T_RAMP > 0 else 1.0
    dramp  = (1.0 / T_RAMP) if t < T_RAMP else 0.0
    ddramp = 0.0
    cos_t, sin_t = np.cos(OMEGA * t), np.sin(OMEGA * t)

    r_vec   = RADIUS * np.array([cos_t, 0.0, sin_t])
    dr_vec  = RADIUS * OMEGA * np.array([-sin_t, 0.0, cos_t])
    ddr_vec = -RADIUS * OMEGA**2 * np.array([cos_t, 0.0, sin_t])

    p_d   = CENTER + ramp * r_vec
    dp_d  = dramp * r_vec + ramp * dr_vec
    ddp_d = ddramp * r_vec + 2.0 * dramp * dr_vec + ramp * ddr_vec
    return p_d, dp_d, ddp_d


def human_wrench(t: float) -> np.ndarray:
    """6-D wrench [force; torque] at the EE; repeats every cycle."""
    t_cyc = t % PERIOD
    if T_FORCE_ON <= t_cyc <= T_FORCE_OFF:
        return np.concatenate([F_HUMAN, np.zeros(3)])
    return np.zeros(6)


# ===========================================================================
#  Joint-space reference trajectory q_d(t)/q̇_d(t)/q̈_d(t) — for
#  ImpedanceMPCParams.horizon_torque_schedule's reference-scheduled horizon
#  constraint. circular_ref() only gives a CARTESIAN reference; the
#  operational-space controller has no joint-space reference of its own
#  (redundancy is resolved online, in the null-space term, not offline), so
#  one is built here via closed-loop resolved-rate IK sharing the same
#  null-space attractor (Q_NEUTRAL) the controller already uses.
# ===========================================================================

class JointTrajFn:
    """Interpolating lookup over a precomputed (q_d, q̇_d, q̈_d) grid.
    Piecewise-linear on q_d/q̇_d (consistent with the finite-difference
    q̈_d already being piecewise-constant between grid samples at this
    resolution); called as joint_traj_fn(t) -> (q_d, dq_d, ddq_d)."""

    def __init__(self, t_grid: np.ndarray, q_d: np.ndarray,
                 dq_d: np.ndarray, ddq_d: np.ndarray):
        self.t_grid = t_grid
        self.q_d, self.dq_d, self.ddq_d = q_d, dq_d, ddq_d

    def __call__(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t_c = min(max(t, self.t_grid[0]), self.t_grid[-1])
        q_d  = np.array([np.interp(t_c, self.t_grid, self.q_d[:, j])  for j in range(7)])
        dq_d = np.array([np.interp(t_c, self.t_grid, self.dq_d[:, j]) for j in range(7)])
        idx  = min(int(np.searchsorted(self.t_grid, t_c)), len(self.t_grid) - 1)
        return q_d, dq_d, self.ddq_d[idx]


def precompute_joint_reference(
    env: "FR3MuJoCoEnv", q0: np.ndarray, duration: float, *,
    dt: float = 0.001, k_p: float = 5.0, k_null: float = 2.0,
) -> JointTrajFn:
    """Closed-loop resolved-rate IK: integrates a redundancy-resolved joint
    trajectory q_d(t) that tracks circular_ref()'s Cartesian reference,
    with a null-space attractor toward Q_NEUTRAL — the SAME redundancy-
    resolution objective (rest-pose centering) the online controller's
    null_torque() already uses, so q_d(t) is a plausible "what the
    controller would do absent any disturbance" reference, not an
    arbitrary IK branch:

        q̇_d = J_v^+(q_d) [ṗ_d + k_p(p_d − FK(q_d))]  +  N̄(q_d)[−k_null(q_d − q_null)]

    Integrated once, offline, at a fine fixed dt via shadow_kinematics (no
    physics stepping, ~0.004 ms/call — a 24 s episode costs ~0.1 s to
    precompute). q̇_d, q̈_d are then read off by finite-differencing the
    resulting q_d(t) array (accurate at this resolution) rather than
    differentiating the CLIK law itself.

    NOTE (limitation): q_d(t) is the UNDISTURBED reference — during the
    human push the actual robot deliberately deflects away from p_d(t),
    so q(t) can diverge from q_d(t) exactly when the horizon constraint
    matters most. Reference scheduling assumes this gap is small enough
    over one MPC horizon to still be a better local model than freezing
    at q_k; see stable_backbone_mpc.md §7 for the empirical check.
    """
    n_steps = int(round(duration / dt)) + 1
    t_grid  = np.arange(n_steps) * dt
    q_d     = np.zeros((n_steps, 7))
    q       = q0.copy()
    q_d[0]  = q
    for i in range(1, n_steps):
        t = t_grid[i - 1]
        p_d, dp_d, _ = circular_ref(t)
        ee_pos, J_v  = env.shadow_kinematics(q)
        JJT     = J_v @ J_v.T + 1e-6 * np.eye(3)
        J_pinv  = J_v.T @ np.linalg.inv(JJT)
        N_bar   = np.eye(7) - J_pinv @ J_v
        qdot    = (J_pinv @ (dp_d + k_p * (p_d - ee_pos))
                  + N_bar @ (-k_null * (q - Q_NEUTRAL)))
        q       = q + dt * qdot
        q_d[i]  = q
    dq_d  = np.gradient(q_d, dt, axis=0)
    ddq_d = np.gradient(dq_d, dt, axis=0)
    return JointTrajFn(t_grid, q_d, dq_d, ddq_d)


# ===========================================================================
#  Controller identity, colours, styles
# ===========================================================================

ALL_CONTROLLERS = [
    "Impedance",
    "Admittance",
    "PI Impedance",
    "Variable-Impedance MPC 100 Hz",
    "DI-MPC 100 Hz",
    "DI-MPC + Kalman 100 Hz",
    "DI-MPC 500 Hz",
    "DI-MPC + Kalman 500 Hz",
]

# Paper figures use only four representative curves for readability:
# D1/D2/D3 are the reactive baselines and D7 is the final proposed controller.
PAPER_CONTROLLERS = [
    "Impedance",
    "Admittance",
    "PI Impedance",
    "Variable-Impedance MPC 100 Hz",
    "DI-MPC + Kalman 500 Hz",
]
PAPER_LABELS = {
    "Impedance": "D1 Impedance",
    "Admittance": "D2 Admittance",
    "PI Impedance": "D3 PI Impedance",
    "Variable-Impedance MPC 100 Hz": "MPVIC Var.-Imp. MPC",
    "DI-MPC + Kalman 500 Hz": "D7 DI-MPC+K 500 Hz",
}
VIDEO_CONTROLLERS = [
    "Impedance",
    "DI-MPC + Kalman 500 Hz",
]

COLORS = {
    "Impedance":                     "#2196F3",   # blue
    "Admittance":                    "#9C27B0",   # purple
    "PI Impedance":                  "#F44336",   # red
    "Variable-Impedance MPC 100 Hz":         "#795548",   # brown
    "DI-MPC 100 Hz":          "#FF9800",   # orange
    "DI-MPC + Kalman 100 Hz": "#4CAF50",   # green
    "DI-MPC 500 Hz":          "#00BCD4",   # teal
    "DI-MPC + Kalman 500 Hz": "#E91E63",   # pink
}
LINESTYLES = {
    "Impedance":                     "--",
    "Admittance":                    ":",
    "PI Impedance":                  "-.",
    "Variable-Impedance MPC 100 Hz":         (0, (4, 2)),
    "DI-MPC 100 Hz":          (0, (3, 1, 1, 1)),
    "DI-MPC + Kalman 100 Hz": "-",
    "DI-MPC 500 Hz":          (0, (5, 1)),
    "DI-MPC + Kalman 500 Hz": (0, (1, 1)),
}
# Single source of truth for line weights so every curve plot and the video
# overlay emphasise the 4 MPC variants identically (500 Hz thicker than 100 Hz,
# baselines thin).  Use _lw(name) everywhere instead of ad-hoc per-plot rules.
LINE_WIDTHS = {
    "Impedance":                     1.6,
    "Admittance":                    1.6,
    "PI Impedance":                  1.6,
    "Variable-Impedance MPC 100 Hz":         1.8,
    "DI-MPC 100 Hz":          1.9,
    "DI-MPC + Kalman 100 Hz": 1.9,
    "DI-MPC 500 Hz":          2.4,
    "DI-MPC + Kalman 500 Hz": 2.4,
}


def _lw(name: str) -> float:
    return LINE_WIDTHS.get(name, 1.6)


def _hex_to_rgb_int(h: str) -> tuple[int, int, int]:
    h = h.lstrip("#")
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))


def _hex_to_rgb(h: str) -> list[float]:
    return [v / 255.0 for v in _hex_to_rgb_int(h)]


COLORS_RGB = {n: _hex_to_rgb(c)     for n, c in COLORS.items()}
COLORS_INT = {n: _hex_to_rgb_int(c) for n, c in COLORS.items()}


# ===========================================================================
#  MPC sample-rate — SINGLE SOURCE OF TRUTH
# ===========================================================================
# Both MPC QP rates live here so the magic numbers can't drift between the
# compare / focused / video paths.  Every MPC controller is built through
# make_mpc_controller(), which resolves the rate from the controller name:
#
#   • "500 Hz" controllers run the QP at MPC_DT_FAST  (500 Hz)
#   • all other ("100 Hz") MPC controllers at MPC_DT_SLOW (100 Hz)
#
# Note the fast rate is an explicit constant rather than "every physics step":
# the env may run faster than the QP (e.g. 1 kHz inner loop in `compare`), so
# tying the QP to the physics step would silently change the rate with the env
# timestep.  Pin it instead, and decimate via mpc_every.
MPC_DT_SLOW = 0.01    # s — 100 Hz QP for the non-"500" MPC variants
MPC_DT_FAST = 0.002   # s — 500 Hz QP for the "500 Hz" MPC variants


# Monkeypatchable override for the MPC corrective-force bound F_max, in the
# style of F_HUMAN/human_wrench above. None -> the normal 150 N default.
# Used by stable_backbone_comparison.py to stress-test saturation without
# duplicating make_mpc_controller.
F_MAX_OVERRIDE: float | None = None

# FR3 joint torque limits (Nm), libfranka values — same numbers as
# ImpedanceMPCParams.tau_max's default, duplicated here so TAU_MAX_SCALE
# below can scale it without instantiating a throwaway params object.
BASE_TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])

# Monkeypatchable multiplicative scale on tau_max (same pattern as
# F_MAX_OVERRIDE). None -> full BASE_TAU_MAX. Used by
# horizon_schedule_comparison.py to force the horizon-wide torque
# constraint to actually bind — under the paper's normal 15 N push, F_max
# is the constraint that saturates first; BASE_TAU_MAX has enough headroom
# that frozen-vs-scheduled J_v/τ_ff scheduling never shows a difference
# unless tau_max is tightened toward where the constraint is active.
TAU_MAX_SCALE: float | None = None

# Monkeypatchable override for ImpedanceMPCParams.schedule_rho (same
# pattern). None -> the dataclass default (0.0, pure reference scheduling).
# Used to sweep the error-decay correction blend without duplicating
# make_mpc_controller.
SCHEDULE_RHO_OVERRIDE: float | None = None


def make_mpc_controller(name: str, dt_sim: float, *,
                        dt_slow: float = MPC_DT_SLOW,
                        dt_fast: float = MPC_DT_FAST):
    """Build the double-integrator predictive controller and its decimation factor for a
    controller name.  The ONE place the QP rate is resolved.

    Returns (mpc_ctrl, mpc_every): the controller is already reset, and the
    caller should call mpc_ctrl.control(...) once every `mpc_every` physics
    steps.  dt_slow / dt_fast are the single knobs for the two QP rates.
    """
    high_freq  = "500" in name
    variable   = "Variable" in name
    # "Backbone" selects the exploratory impedance-backbone architecture
    # (stable_backbone_mpc.md): a fixed restoring/damped impedance law is
    # commanded independently of the QP, which only shapes a bounded ADDITIONAL
    # correction on top, with the torque-realizability constraint extended
    # to the whole horizon (frozen-Jacobian approximation) instead of only
    # the first step.
    backbone   = "Backbone" in name
    # "Frozen" selects the horizon-wide torque-realizability constraint
    # STANDALONE (no backbone): the same frozen-at-q_k affine row used
    # inside Backbone, but on top of the default LQ-MPC branch instead of
    # the backbone+additive-correction one — isolates the constraint's
    # effect from the backbone architecture change.
    frozen_horizon   = "Frozen" in name
    # "Schedule" selects the reference-scheduled horizon-wide constraint
    # instead of freezing at q_k: J_v,i/τ_ff,i are recomputed along the
    # redundancy-resolved joint reference q_d(t) precomputed by
    # precompute_joint_reference (falls back to the same frozen-at-q_k
    # behavior as "Frozen" if run_episode didn't wire in a joint_traj_fn),
    # see ImpedanceMPCParams.horizon_torque_schedule/schedule_rho.
    schedule_horizon = "Schedule" in name
    # MPVIC always runs the Kalman observer (it schedules stiffness on d̂ but
    # does not cancel it); the DI-MPC variants use Kalman only when named.
    use_kal    = variable or ("Kalman" in name)
    dt_mpc_eff = dt_fast if high_freq else dt_slow
    mpc_every  = max(1, round(dt_mpc_eff / dt_sim))
    # Q_proc_d is a per-Kalman-tick process-noise variance, and the filter
    # ticks once per control() call, i.e. once every dt_mpc_eff seconds (see
    # ImpedanceMPCController._kalman_step). Holding it at the same absolute
    # value for both the 100 Hz and 500 Hz configs (as an earlier version of
    # this code did) injects 5x the process noise per second at 500 Hz,
    # confounding "faster rate" with "implicitly more aggressive estimator".
    # Scale by dt_mpc_eff/MPC_DT_SLOW so the continuous-time-equivalent
    # density is held constant instead; this reproduces the original
    # Q_proc_d=10.0 exactly at the 100 Hz reference rate.
    # This applies to every "500 Hz"-named controller built through this
    # function, i.e. every experiment in phri_ICRA.tex that reuses it. As of
    # 2026-08-12: the core ablation (Table II), broad screen (Table III), and
    # waypoint generalization (Table tab:waypoints, via guidance.py's OWN
    # separate make_mpc_controller -- see the identical fix and comment
    # there) have been rerun and updated in the paper to match. The
    # four-plane generalization table (tab:planes) only ever used C5 (100 Hz)
    # so it was never affected by this confound -- checked, not rerun.
    # The force-magnitude/shape sweep and the C8 corrective-authority stress
    # test still use the old, unscaled Q_w=10 at 500 Hz and have NOT been
    # re-verified against this fix -- re-run them before trusting those
    # specific numbers.
    Q_proc_d_eff = 10.0 * (dt_mpc_eff / MPC_DT_SLOW)
    mpc_params = ImpedanceMPCParams(
        N=10, dt_mpc=dt_mpc_eff,
        Q_pos=2e4 * np.eye(3), Q_vel=50.0 * np.eye(3),
        Q_f_scale=5.0, R_u=1e-6 * np.eye(3),
        Q_proc_d=Q_proc_d_eff,
        variable_impedance=variable,
        backbone_track=backbone,
        horizon_torque_constraint=backbone or frozen_horizon,
        horizon_torque_schedule=schedule_horizon,
        schedule_rho=SCHEDULE_RHO_OVERRIDE if SCHEDULE_RHO_OVERRIDE is not None else 0.0,
        F_max=F_MAX_OVERRIDE if F_MAX_OVERRIDE is not None else 150.0,
        tau_max=BASE_TAU_MAX * TAU_MAX_SCALE if TAU_MAX_SCALE is not None else BASE_TAU_MAX,
        K_rot=20.0, D_rot=6.0,
        k_null=10.0, d_null=2.0, q_null=Q_NEUTRAL,
        # Disable Cartesian workspace projection: at the demo radius it offsets
        # p_d up to max_ws_corr near joint limits, and since that offset is
        # hidden from the Kalman observer it shows up as an uncorrectable
        # ~30 mm tracking error (apparent "static error"). The null-space
        # barrier in null_torque still guards the limits.
        k_ws=0.0,
    )
    mpc_ctrl = ImpedanceMPCController(mpc_params, use_kalman=use_kal)
    mpc_ctrl.reset()
    return mpc_ctrl, mpc_every


# ===========================================================================
#  EpisodeController — one object per controller, encapsulates torque law
# ===========================================================================

class EpisodeController:
    """Builds and runs the per-controller torque law.

    The controller kind is inferred from substrings of `name`, so all three
    naming conventions used by the former scripts resolve identically:
        'Admittance'      -> admittance
        'PI'              -> PI impedance
        'MPC'             -> impedance MPC (+Kalman if 'Kalman', 500 Hz if '500')
        else              -> classical Cartesian impedance

    MPC QP timing (resolved by make_mpc_controller; see MPC_DT_SLOW/FAST):
        dt_mpc    -> slow ("100 Hz") QP period           (default MPC_DT_SLOW)
        hifreq_dt -> overrides the fast ("500 Hz") rate  (default MPC_DT_FAST)
    """

    def __init__(self, name: str, env: "FR3MuJoCoEnv", *,
                 dt_mpc: float = MPC_DT_SLOW, hifreq_dt: float | None = None):
        self.name = name
        self.env  = env
        dt_sim    = env.dt

        self.imp_params = make_impedance_params(
            k_pos=300.0, k_rot=20.0, damping_ratio=1.0, q_null=Q_NEUTRAL)

        self.adm_ctrl = None
        self.mpc_ctrl = None
        self.pi_mode  = False
        self.pi_integral_e = np.zeros(3)
        self.PI_K_INT  = 80.0     # task-space integral gain  [N/(m·s)]
        self.PI_WINDUP = 0.15     # anti-windup magnitude limit [m·s]
        self.mpc_every = 1

        if "Admittance" in name:
            # M_a ẍ_r + D_a ẋ_r + K_a x_r = f_ext  ->  x_cmd = p_d + x_r
            self.adm_ctrl = AdmittanceController(
                make_admittance_params(m_pos=0.5, d_pos=15.0, k_pos=100.0),
                dt=dt_sim,
            )
        elif "PI" in name:
            self.pi_mode = True
        elif "MPC" in name:
            # hifreq_dt overrides the 500 Hz rate when provided (e.g. `compare`
            # runs a 1 kHz inner loop); otherwise the constant MPC_DT_FAST.
            dt_fast = hifreq_dt if hifreq_dt is not None else MPC_DT_FAST
            self.mpc_ctrl, self.mpc_every = make_mpc_controller(
                name, dt_sim, dt_slow=dt_mpc, dt_fast=dt_fast)
        # else: classical impedance — uses imp_params as-is

        self.tau_cached   = np.zeros(7)
        self.F_mpc_cached = np.zeros(3)

    @property
    def mpc_rate_hz(self) -> float | None:
        return 1.0 / (self.mpc_every * self.env.dt) if self.mpc_ctrl else None

    def compute(self, state, dyn, p_d, dp_d, ddp_d, R_d, wrench, i, t=None,
                joint_traj_fn=None):
        """Return (tau, F_mpc) for inner-loop step index i.

        `t`/`joint_traj_fn` are only used by the MPC branch, and only
        matter when horizon_torque_schedule=True (t, traj_fn/dyn_query_fn/
        joint_traj_fn are otherwise ignored by control()); joint_traj_fn
        None falls back to the frozen-at-q_k horizon constraint."""
        dt_sim   = self.env.dt
        dx_d_6d  = np.concatenate([dp_d,  np.zeros(3)])
        ddx_d_6d = np.concatenate([ddp_d, np.zeros(3)])

        if self.adm_ctrl is not None:
            x_r, v_r = self.adm_ctrl.step(wrench)
            x_cmd  = p_d + x_r[:3]
            dx_cmd = np.concatenate([dp_d + v_r[:3], np.zeros(3)])
            tau = cartesian_impedance_control(
                state, dyn, x_cmd, R_d, dx_cmd, ddx_d_6d, self.imp_params)
            tau += self.env.null_space_gravity_comp(dyn)
            self.F_mpc_cached = np.zeros(3)

        elif self.pi_mode:
            e_pi = p_d - state.ee_pos
            self.pi_integral_e += dt_sim * e_pi
            nm = float(np.linalg.norm(self.pi_integral_e))
            if nm > self.PI_WINDUP:
                self.pi_integral_e *= self.PI_WINDUP / nm
            J_v = dyn.J[:3, :]
            Lam = np.linalg.inv(
                J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3))
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, self.imp_params)
            tau += J_v.T @ (Lam @ (self.PI_K_INT * self.pi_integral_e))
            tau += self.env.null_space_gravity_comp(dyn)
            self.F_mpc_cached = np.zeros(3)

        elif self.mpc_ctrl is not None:
            if i % self.mpc_every == 0:
                self.tau_cached, self.F_mpc_cached = self.mpc_ctrl.control(
                    state.ee_pos, state.ee_vel, state.ee_rot,
                    p_d, dp_d, ddp_d, R_d, dyn, state.q, state.dq,
                    t=t, traj_fn=circular_ref, dyn_query_fn=self.env.shadow_dynamics,
                    joint_traj_fn=joint_traj_fn)
                tau = self.tau_cached
            else:
                # 1 kHz inner loop: feedforward, orientation, and
                # null-space torques are recomputed every tick from
                # fresh (q, dq); only the QP correction F_mpc is held
                # from the last solve.
                J_v, J_w = dyn.J[:3, :], dyn.J[3:, :]
                Lam_pos = np.linalg.inv(
                    J_v @ np.linalg.inv(dyn.M) @ J_v.T + 1e-6 * np.eye(3))
                tau_ff = dyn.Cq_dot + J_v.T @ (Lam_pos @ ddp_d)
                e_R    = rotation_error_matrix(R_d, state.ee_rot)
                p_mpc  = self.mpc_ctrl.p
                tau_or = J_w.T @ (-p_mpc.K_rot * e_R - p_mpc.D_rot * state.ee_vel[3:])
                N_bar  = build_operational_space_model(dyn, state.ee_vel).N_bar
                tau    = (tau_ff + J_v.T @ self.F_mpc_cached + tau_or
                          + self.mpc_ctrl.null_torque(state.q, state.dq, N_bar))

        else:  # classical impedance
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, self.imp_params)
            tau += self.env.null_space_gravity_comp(dyn)
            self.F_mpc_cached = np.zeros(3)

        return tau, self.F_mpc_cached


def _episode_metrics(t_log: np.ndarray, err_log: np.ndarray) -> dict:
    """RMS / contact / peak / steady-state metrics, per-cycle (mod PERIOD)."""
    t_cyc       = t_log % PERIOD
    mask_cont   = (t_cyc >= T_FORCE_ON) & (t_cyc <= T_FORCE_OFF)
    mask_ss     = (t_cyc >= T_FORCE_OFF - 0.2) & (t_cyc <= T_FORCE_OFF)
    return dict(
        rms_total   = float(np.sqrt(np.mean(err_log**2))),
        rms_contact = float(np.sqrt(np.mean(err_log[mask_cont]**2))) if mask_cont.any() else float('nan'),
        peak_defl   = float(np.max(err_log[mask_cont]))              if mask_cont.any() else float('nan'),
        ss_err      = float(np.mean(err_log[mask_ss]))               if mask_ss.any()   else float('nan'),
    )


# ===========================================================================
#  Live-viewer scene helpers (compare / focused)
# ===========================================================================

def _add_line(scn, a, b, rgba, width=0.003):
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    mujoco.mjv_connector(
        scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE, width,
        np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64))
    scn.geoms[n].rgba[:] = rgba
    scn.ngeom += 1


def _ref_circle_pts(n_seg: int = 80) -> np.ndarray:
    theta = np.linspace(0, 2 * np.pi, n_seg + 1)
    return np.column_stack([
        CENTER[0] + RADIUS * np.cos(theta),
        np.full(n_seg + 1, CENTER[1]),
        CENTER[2] + RADIUS * np.sin(theta),
    ])


# ===========================================================================
#  run_episode — shared by `compare` and `focused`
# ===========================================================================

def run_episode(controller_name: str, env: "FR3MuJoCoEnv", *,
                n_cycles: int = 1, dt_mpc: float = MPC_DT_SLOW,
                hifreq_dt: float | None = None,
                verbose: bool = True, viewer=None) -> dict:
    env.reset()
    R_d = np.eye(3)

    def _sid(name):
        return mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, name)
    _target_sid = _sid("target_pos")
    _wrist_sid  = _sid("human_wrist")

    dt_sim   = env.dt
    duration = n_cycles * PERIOD
    n_steps  = int(duration / dt_sim)

    ctrl = EpisodeController(controller_name, env,
                             dt_mpc=dt_mpc, hifreq_dt=hifreq_dt)

    # Reference-scheduled horizon torque constraint (ImpedanceMPCParams.
    # horizon_torque_schedule) needs a joint-space reference trajectory to
    # schedule against; precompute it once per episode (cheap, ~0.1 s for a
    # full episode via shadow_kinematics) rather than inside the control
    # loop. None for every other controller — control() falls back to the
    # frozen-at-q_k horizon constraint when joint_traj_fn is None.
    joint_traj_fn = None
    if ctrl.mpc_ctrl is not None and ctrl.mpc_ctrl.p.horizon_torque_schedule:
        joint_traj_fn = precompute_joint_reference(env, env.data.qpos[:env.nv].copy(), duration)

    t_log      = np.zeros(n_steps)
    err_log    = np.zeros(n_steps)
    ee_pos_log = np.zeros((n_steps, 3))
    pd_log     = np.zeros((n_steps, 3))
    fhum_log   = np.zeros(n_steps)
    tau_log    = np.zeros((n_steps, 7))
    Fmpc_log   = np.zeros((n_steps, 3))

    if verbose:
        rate = f"{ctrl.mpc_rate_hz:.0f} Hz" if ctrl.mpc_ctrl else "N/A"
        print(f"\n[{controller_name}] {n_cycles} cycle(s) = {duration:.1f} s "
              f"(dt={dt_sim*1e3:.1f} ms, MPC {rate})…")

    if viewer is not None:
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,
                          mujoco.mjtGridPos.mjGRID_TOP, controller_name, ""))
    _ctrl_rgb   = COLORS_RGB.get(controller_name, [0.5, 0.5, 0.5])
    _ee_trail   = deque(maxlen=600)
    _ref_circle = _ref_circle_pts(80)

    t_wall_start = time.perf_counter()

    for i in range(n_steps):
        t = env.time
        p_d, dp_d, ddp_d = circular_ref(t)
        wrench = human_wrench(t)
        dyn, state = env.get_dynamics_and_state()

        if np.any(wrench[:3] != 0):
            env.apply_ee_wrench(wrench)

        tau, F_mpc = ctrl.compute(state, dyn, p_d, dp_d, ddp_d, R_d, wrench, i, t=t,
                                  joint_traj_fn=joint_traj_fn)
        env.apply_torque(tau)
        env.step()

        err = float(np.linalg.norm(state.ee_pos - p_d))
        t_log[i]      = t
        err_log[i]    = err
        ee_pos_log[i] = state.ee_pos
        pd_log[i]     = p_d
        fhum_log[i]   = float(np.linalg.norm(wrench[:3]))
        tau_log[i]    = tau
        Fmpc_log[i]   = F_mpc

        if verbose and i % 2000 == 0:
            print(f"  t={t:.2f}s  |e|={err:.4f} m")

        if viewer is not None:
            if _target_sid >= 0:
                env.model.site_pos[_target_sid] = p_d
            if _wrist_sid >= 0:
                t_cyc = t % PERIOD
                if T_FORCE_ON <= t_cyc <= T_FORCE_OFF:
                    env.model.site_pos[_wrist_sid]     = state.ee_pos
                    env.model.site_rgba[_wrist_sid, 3] = 0.7
                else:
                    env.model.site_rgba[_wrist_sid, 3] = 0.0
            _ee_trail.append(state.ee_pos.copy())
            scn = viewer.user_scn
            scn.ngeom = 0
            for k in range(len(_ref_circle) - 1):
                _add_line(scn, _ref_circle[k], _ref_circle[k+1],
                          [0.0, 0.85, 0.0, 0.35], width=0.002)
            trail = list(_ee_trail)
            n_seg = len(trail) - 1
            for k in range(0, n_seg, 3):
                alpha = 0.15 + 0.75 * (k / max(n_seg, 1))
                _add_line(scn, trail[k], trail[k+1],
                          [*_ctrl_rgb, alpha], width=0.004)
            if err > 0.002:
                _add_line(scn, state.ee_pos, p_d, [1.0, 0.15, 0.0, 0.9], width=0.002)
            t_sim  = (i + 1) * dt_sim
            t_wall = time.perf_counter() - t_wall_start
            if t_sim > t_wall:
                time.sleep(t_sim - t_wall)
            viewer.sync()

    metrics = _episode_metrics(t_log, err_log)
    if verbose:
        print(f"  → RMS total:   {metrics['rms_total']*1e3:.1f} mm")
        print(f"  → RMS contact: {metrics['rms_contact']*1e3:.1f} mm")
        print(f"  → Peak defl:   {metrics['peak_defl']*1e3:.1f} mm")
        print(f"  → SS error:    {metrics['ss_err']*1e3:.1f} mm")

    return dict(t=t_log, pos_err=err_log, ee_pos=ee_pos_log, p_d=pd_log,
                f_human=fhum_log, tau=tau_log, F_mpc=Fmpc_log, **metrics)


def _run_all_episodes(controllers, env, *, n_cycles, hifreq_dt,
                      viewer=None, pause_secs=3.0):
    results = {}
    for k, name in enumerate(controllers):
        results[name] = run_episode(name, env, n_cycles=n_cycles,
                                    hifreq_dt=hifreq_dt, verbose=True,
                                    viewer=viewer)
        if viewer is not None and k < len(controllers) - 1:
            next_name = controllers[k + 1]
            print(f"\n  ── Holding {pause_secs:.0f} s — next: {next_name} ──")
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,
                              mujoco.mjtGridPos.mjGRID_TOP,
                              f"Next: {next_name}", ""))
            t_end = time.perf_counter() + pause_secs
            while time.perf_counter() < t_end:
                viewer.sync(); time.sleep(0.02)
    return results


def _print_summary(results: dict, width: int = 32):
    print("\n" + "=" * 68)
    print(f"{'Controller':<{width}} {'RMS(mm)':>8} {'Contact(mm)':>12} "
          f"{'Peak(mm)':>10} {'SS(mm)':>8}")
    print("-" * 68)
    for name, d in results.items():
        print(f"{name:<{width}} {d['rms_total']*1e3:>8.1f} "
              f"{d['rms_contact']*1e3:>12.1f} {d['peak_defl']*1e3:>10.1f} "
              f"{d['ss_err']*1e3:>8.1f}")
    print("=" * 68)


def _launch_with_viewer(env, run_fn):
    import mujoco.viewer as mjviewer
    with mjviewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.azimuth   = 135.0
        viewer.cam.elevation = -20.0
        viewer.cam.distance  =  2.5
        viewer.cam.lookat[:] = [0.45, 0.0, 0.45]
        return run_fn(viewer)


# ===========================================================================
#  Mode: compare  — paper-readable 4-controller summary plot
# ===========================================================================

def _paper_subset(results: dict) -> dict:
    return {k: results[k] for k in PAPER_CONTROLLERS if k in results}


def plot_results(results: dict, save_path: str | None = None, paper_only: bool = True):
    plot_data = _paper_subset(results) if paper_only else results
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("FR3 pHRI Benchmark: Circular Trajectory + Step Human Force",
                 fontsize=13, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)
    ax_err  = fig.add_subplot(gs[0, :])
    ax_x    = fig.add_subplot(gs[1, 0])
    ax_z    = fig.add_subplot(gs[1, 1])
    ax_traj = fig.add_subplot(gs[2, 0])
    ax_bar  = fig.add_subplot(gs[2, 1])

    for name, data in plot_data.items():
        t  = data["t"]; c = COLORS[name]; ls = LINESTYLES[name]; lw = _lw(name)
        label = PAPER_LABELS.get(name, name)
        ax_err.plot(t, data["pos_err"] * 1e3, color=c, ls=ls, lw=lw, label=label)
        ax_x.plot(t, data["ee_pos"][:, 0] * 1e2, color=c, ls=ls, lw=lw, label=label)
        ax_z.plot(t, data["ee_pos"][:, 2] * 1e2, color=c, ls=ls, lw=lw, label=label)
        ax_traj.plot(data["ee_pos"][:, 0] * 1e2, data["ee_pos"][:, 2] * 1e2,
                     color=c, ls=ls, lw=lw, label=label)

    first = next(iter(plot_data.values()))
    t_ref, p_d = first["t"], first["p_d"]
    ax_x.plot(t_ref, p_d[:, 0] * 1e2, 'k:', lw=1, label="Reference")
    ax_z.plot(t_ref, p_d[:, 2] * 1e2, 'k:', lw=1, label="Reference")
    theta = np.linspace(0, 2*np.pi, 200)
    ax_traj.plot((CENTER[0] + RADIUS*np.cos(theta)) * 1e2,
                 (CENTER[2] + RADIUS*np.sin(theta)) * 1e2, 'k:', lw=1, label="Reference")

    for ax in [ax_err, ax_x, ax_z]:
        ax.axvspan(T_FORCE_ON, T_FORCE_OFF, alpha=0.08, color='red', label="Human force")

    ax_err.set_xlabel("Time (s)"); ax_err.set_ylabel("Position error (mm)")
    ax_err.set_title("End-effector position error ‖p − p_d‖")
    ax_err.legend(fontsize=8, ncol=2); ax_err.grid(True, alpha=0.3)
    for ax, ylabel, title in [(ax_x, "x (cm)", "EE x-position"),
                              (ax_z, "z (cm)", "EE z-position")]:
        ax.set_xlabel("Time (s)"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax_traj.set_xlabel("x (cm)"); ax_traj.set_ylabel("z (cm)")
    ax_traj.set_title("xz-plane trajectory"); ax_traj.set_aspect("equal")
    # No legend here: the square equal-aspect panel is too small to hold an
    # 8-entry legend without covering the circle; ax_err already carries the
    # full legend with identical colors/linestyles.
    ax_traj.grid(True, alpha=0.3)

    _bar_summary(
        ax_bar, plot_data, list(plot_data.keys()), w=0.18,
        label_fn=lambda n: PAPER_LABELS.get(n, n),
    )

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n[plot] Saved → {save_path}")
    return fig


def _bar_summary(ax, results, names, w=0.15, label_fn=None):
    metrics = ["rms_total", "rms_contact", "peak_defl", "ss_err"]
    labels  = ["RMS total\n(mm)", "RMS contact\n(mm)", "Peak defl.\n(mm)", "SS error\n(mm)"]
    x = np.arange(len(metrics))
    offsets = np.linspace(-(len(names)-1)/2, (len(names)-1)/2, len(names)) * w
    for k, name in enumerate(names):
        vals = [results[name][m] * 1e3 for m in metrics]
        lbl  = label_fn(name) if label_fn else name
        ax.bar(x + offsets[k], vals, w, color=COLORS[name], label=lbl, alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("Error (mm)"); ax.set_title("Performance summary")
    ax.legend(fontsize=7); ax.grid(True, axis='y', alpha=0.3)


def main_compare(args):
    global RADIUS
    RADIUS = args.radius
    save_dir = Path(__file__).parent.parent / "simulation_results"
    save_dir.mkdir(parents=True, exist_ok=True)
    save_path = str(save_dir / "mpc_comparison_results.png")

    print("=" * 60)
    print("FR3 pHRI Benchmark: Circular trajectory + step human force")
    print(f"  Reference : radius={RADIUS} m, period={PERIOD:.1f} s")
    print(f"  Human force: {F_HUMAN} N  from t={T_FORCE_ON}–{T_FORCE_OFF} s")
    print(f"  Cycles    : {args.cycles} × {PERIOD:.1f} s = {args.cycles*PERIOD:.1f} s")
    print("=" * 60)

    env = FR3MuJoCoEnv(timestep=0.001)   # 1 kHz inner loop; 500 Hz QP via hifreq_dt
    run = lambda viewer=None: _run_all_episodes(
        PAPER_CONTROLLERS, env, n_cycles=args.cycles, hifreq_dt=MPC_DT_FAST, viewer=viewer)
    results = _launch_with_viewer(env, run) if not args.no_viewer else run()

    _print_summary(results, width=28)
    plt.close(plot_results(results, save_path=save_path, paper_only=True))


# ===========================================================================
#  Mode: focused — paper-readable comparison plot
# ===========================================================================

def plot_controller_comparison(results: dict, save_path: str | None = None, paper_only: bool = True):
    subset = _paper_subset(results) if paper_only else {k: results[k] for k in ALL_CONTROLLERS if k in results}
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("FR3 pHRI Benchmark — Controller Comparison\n"
                 "(Circular Trajectory + Step Human Force — D1/D2/D3/D7 shown for readability)",
                 fontsize=13, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.48, wspace=0.35)
    ax_err  = fig.add_subplot(gs[0, :])
    ax_x    = fig.add_subplot(gs[1, 0])
    ax_z    = fig.add_subplot(gs[1, 1])
    ax_traj = fig.add_subplot(gs[2, 0])
    ax_bar  = fig.add_subplot(gs[2, 1])

    for name, data in subset.items():
        t  = data["t"]; c = COLORS[name]; ls = LINESTYLES[name]; lw = _lw(name)
        label = PAPER_LABELS.get(name, name)
        ax_err.plot(t, data["pos_err"] * 1e3, color=c, ls=ls, lw=lw, label=label)
        ax_x.plot(t, data["ee_pos"][:, 0] * 1e2, color=c, ls=ls, lw=lw, label=label)
        ax_z.plot(t, data["ee_pos"][:, 2] * 1e2, color=c, ls=ls, lw=lw, label=label)
        ax_traj.plot(data["ee_pos"][:, 0] * 1e2, data["ee_pos"][:, 2] * 1e2,
                     color=c, ls=ls, lw=lw, label=label)

    first = next(iter(subset.values()))
    t_ref, p_d = first["t"], first["p_d"]
    ax_x.plot(t_ref, p_d[:, 0] * 1e2, 'k:', lw=1.2, label="Reference")
    ax_z.plot(t_ref, p_d[:, 2] * 1e2, 'k:', lw=1.2, label="Reference")
    theta = np.linspace(0, 2*np.pi, 200)
    ax_traj.plot((CENTER[0] + RADIUS*np.cos(theta)) * 1e2,
                 (CENTER[2] + RADIUS*np.sin(theta)) * 1e2, 'k:', lw=1.2, label="Reference")

    t_end = t_ref[-1]
    for cyc in range(int(round(t_end / PERIOD))):
        t_on, t_off = cyc*PERIOD + T_FORCE_ON, cyc*PERIOD + T_FORCE_OFF
        for ax in [ax_err, ax_x, ax_z]:
            ax.axvspan(t_on, min(t_off, t_end), alpha=0.08, color='red',
                       label="Human force" if cyc == 0 else None)

    ax_err.set_xlabel("Time (s)"); ax_err.set_ylabel("Position error (mm)")
    ax_err.set_title("End-effector position error ‖p − p_d‖")
    ax_err.legend(fontsize=8, ncol=2, loc="upper right"); ax_err.grid(True, alpha=0.3)
    for ax, ylabel, title in [(ax_x, "x (cm)", "EE x-position"),
                              (ax_z, "z (cm)", "EE z-position")]:
        ax.set_xlabel("Time (s)"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax_traj.set_xlabel("x (cm)"); ax_traj.set_ylabel("z (cm)")
    ax_traj.set_title("xz-plane trajectory"); ax_traj.set_aspect("equal")
    # No legend here: the square equal-aspect panel is too small to hold an
    # 8-entry legend without covering the circle; ax_err already carries the
    # full legend with identical colors/linestyles.
    ax_traj.grid(True, alpha=0.3)

    _bar_summary(
        ax_bar, subset, list(subset.keys()), w=0.18,
        label_fn=lambda n: PAPER_LABELS.get(n, n),
    )

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[plot1] Saved → {save_path}")
    return fig


def main_focused(args):
    global RADIUS
    RADIUS = args.radius
    save_dir = Path(__file__).parent.parent / "simulation_results"
    save_dir.mkdir(parents=True, exist_ok=True)
    path1 = str(save_dir / "focused_controller_comparison.png")

    print("=" * 60)
    print("FR3 pHRI Benchmark — Focused Comparisons")
    print(f"  Radius={RADIUS} m, period={PERIOD:.1f} s, cycles={args.cycles}, "
          f"duration={args.cycles*PERIOD:.1f} s")
    print(f"  Human force: {F_HUMAN} N  t=[{T_FORCE_ON}, {T_FORCE_OFF}] s per cycle")
    print("=" * 60)

    env = FR3MuJoCoEnv()
    run = lambda viewer=None: _run_all_episodes(
        PAPER_CONTROLLERS, env, n_cycles=args.cycles, hifreq_dt=None, viewer=viewer)
    results = _launch_with_viewer(env, run) if not args.no_viewer else run()

    _print_summary(results, width=32)
    plt.close(plot_controller_comparison(results, save_path=path1, paper_only=True))
    print("\nDone. Output file:")
    print(f"  {path1}")


# ===========================================================================
#  Mode: video — rendered MP4 walkthrough
# ===========================================================================

try:
    from PIL import Image, ImageDraw, ImageFont
    _PIL_OK = True
except ImportError:
    _PIL_OK = False

_FONT_LG = _FONT_MD = _FONT_SM = None

PLOT_W, PLOT_H, PLOT_DPI = 390, 200, 100
_ZERO3 = np.zeros(3, dtype=np.float64)
_EYE9  = np.eye(3, dtype=np.float64).flatten()
_RGBA0 = np.zeros(4, dtype=np.float32)

DESCRIPTIONS = {
    "Impedance": (
        "Spring-damper in task space  |  SS error = Fh / Kd = 50 mm",
        ["Classical Cartesian impedance:  tau = J^T [ Lambda(ddx_d - Kd*e - Dd*de) + mu + p ]",
         "Human force Fh is an unmodelled disturbance — the controller has no knowledge of it.",
         "At steady state the spring deflection balances the force:  e_ss = Kd^-1 * Fh = 15/300 = 50 mm.",
         "No integral action, no prediction, no estimator.  Deflection is permanent while force persists."],
    ),
    "Admittance": (
        "Yields to force by design  |  SS deflection = Fh / Ka = 150 mm",
        ["Admittance model:  Ma * x_r'' + Da * x_r' + Ka * x_r = Fh",
         "The virtual spring Ka (100 N/m) filters human intent into a reference offset x_r.",
         "Impedance then tracks  x_cmd = p_d + x_r  — the arm intentionally moves with the operator.",
         "Steady-state offset = Fh / Ka = 15/100 = 150 mm.  Large deflection is the goal, not a failure."],
    ),
    "PI Impedance": (
        "Integral action reduces SS error  |  gain-limited by stability: Kint < Dd*Kd",
        ["Adds a task-space integral term:  tau_int = J^T * Lambda * Kint * integral(e dt)",
         "The integrator accumulates position error and injects a correction torque.",
         "Stability condition limits gain:  Kint < Dd * Kd  (here: 80 < 30*300 = 9000 N/(m*s)).",
         "Result: SS error falls from 50 mm → ~22 mm, but slow convergence — never reaches zero."],
    ),
    "DI-MPC 100 Hz": (
        "QP lookahead @ 100 Hz  |  predicts disturbance within horizon",
        ["Two-layer architecture:  feedforward nonlinear inversion  +  receding-horizon QP on residual.",
         "QP state: [e, de, d_hat].  Horizon N=10 steps × 10 ms = 100 ms lookahead.",
         "QP update rate: 100 Hz (every 10 physics steps).  Torque held constant between updates.",
         "No Kalman estimator: d_hat is not updated between QP solves → persistent SS error ~3.7 mm."],
    ),
    "DI-MPC + Kalman 100 Hz": (
        "MPC @ 100 Hz + Kalman disturbance estimator  |  drives SS error → 0",
        ["Augmented state:  x_aug = [e; de; d_hat]  where d_hat in R^3 is the estimated constant input-channel disturbance.",
         "Kalman update:  d_hat(k+1) = d_hat(k) + Kf * (y(k) - C * x_aug(k))",
         "As d_hat converges, the centered QP drives F_mpc ≈ -d_hat, cancelling constant deflection.",
         "Convergence lag: ~1 QP interval (10 ms) before d_hat tracks the matched disturbance.  After that: SS error < 0.1 mm."],
    ),
    "DI-MPC 500 Hz": (
        "QP lookahead @ 500 Hz  |  faster update reduces peak deflection",
        ["Same two-layer MPC as 100 Hz variant — only the QP update rate changes: every 2 ms.",
         "Shorter zero-order-hold:  torque refreshed every 2 ms instead of 10 ms.",
         "Peak deflection falls vs. 100 Hz because force onset is corrected faster.",
         "No Kalman: persistent SS error ~1.1 mm remains.  Frequency affects transient, NOT steady state."],
    ),
    "DI-MPC + Kalman 500 Hz": (
        "MPC @ 500 Hz + Kalman  |  peak < 1 mm, SS error < 0.1 mm",
        ["Combines both improvements: 500 Hz QP (fast transient) + Kalman estimator (zero SS error).",
         "The two axes are ORTHOGONAL: update rate governs peak deflection; Kalman governs SS error.",
         "500 Hz QP: corrects within 2 ms of force onset → small peak.",
         "Kalman: d_hat -> Fh in ~1 update (2 ms) → SS error < 0.1 mm.  220x improvement vs. classical."],
    ),
}


def _make_camera() -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth   = 135.0
    cam.elevation = -22.0
    cam.distance  =   1.7
    cam.lookat[:] = [0.45, 0.0, 0.45]
    return cam


def _load_font(size: int):
    if not _PIL_OK:
        return None
    try:
        return ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", size)
    except Exception:
        try:
            return ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
        except Exception:
            return ImageFont.load_default()


def _wrap_text(text: str, max_chars: int = 80) -> list[str]:
    words = text.split()
    lines, cur = [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > max_chars:
            lines.append(cur); cur = w
        else:
            cur = (cur + " " + w).lstrip()
    if cur:
        lines.append(cur)
    return lines


def _add_capsule(scn, a, b, rgba, width: float = 0.003) -> None:
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    # mjv_initGeom zero-initializes all struct fields first; mjv_connector
    # alone leaves pointer fields uninitialized -> SIGBUS in the renderer.
    mujoco.mjv_initGeom(scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE,
                        _ZERO3, _ZERO3, _EYE9, _RGBA0)
    mujoco.mjv_connector(scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(a, dtype=np.float64),
                         np.asarray(b, dtype=np.float64))
    scn.geoms[n].rgba[:] = rgba
    scn.ngeom += 1


def _draw_3d_overlays(scn, ee_pos, p_d, trail, ctrl_rgb, ref_circle) -> None:
    for k in range(len(ref_circle) - 1):
        _add_capsule(scn, ref_circle[k], ref_circle[k+1],
                     [0.0, 0.9, 0.1, 0.5], width=0.002)
    pts = list(trail); n = len(pts) - 1
    for k in range(0, n, 3):
        alpha = 0.15 + 0.75 * (k / max(n, 1))
        _add_capsule(scn, pts[k], pts[k+1], [*ctrl_rgb, alpha], width=0.004)
    if float(np.linalg.norm(ee_pos - p_d)) > 0.002:
        _add_capsule(scn, ee_pos, p_d, [1.0, 0.15, 0.0, 0.9], width=0.002)


def _create_error_fig(ctrl_name, n_cycles, ctrl_color_hex):
    duration = n_cycles * PERIOD
    fig, ax = plt.subplots(figsize=(PLOT_W/PLOT_DPI, PLOT_H/PLOT_DPI), dpi=PLOT_DPI)
    bg = (0.06, 0.06, 0.10)
    ax.set_facecolor(bg); fig.patch.set_facecolor(bg)
    for c in range(n_cycles):
        ax.axvspan(c*PERIOD + T_FORCE_ON, c*PERIOD + T_FORCE_OFF,
                   alpha=0.18, color='#FF5555', zorder=0, lw=0)
    (line,) = ax.plot([], [], lw=1.4, color=ctrl_color_hex, zorder=3)
    cursor = ax.axvline(x=0.0, color='#FFFF80', lw=0.9, alpha=0.85, zorder=4)
    ax.set_xlim(0.0, duration); ax.set_ylim(0.0, 10.0)
    ax.set_xlabel('t  (s)', color='#999999', fontsize=6.5, labelpad=1)
    ax.set_ylabel('|e|  (mm)', color='#999999', fontsize=6.5, labelpad=1)
    ax.set_title('Position error', color='#cccccc', fontsize=7.5, pad=3)
    ax.tick_params(colors='#888888', labelsize=5.5, length=2, pad=1)
    for sp in ax.spines.values():
        sp.set_edgecolor('#333333')
    ax.grid(True, color='#1e1e2e', lw=0.6, zorder=1)
    fig.tight_layout(pad=0.55); fig.canvas.draw()
    return fig, ax, line, cursor


def _render_error_inset(fig, ax, line, cursor, t_arr, err_arr, t_now, y_max_state):
    line.set_data(t_arr, err_arr)
    cursor.set_xdata([t_now, t_now])
    cur_max = float(err_arr.max()) if len(err_arr) > 0 else 0.0
    if cur_max * 1.25 > y_max_state[0]:
        y_max_state[0] = max(cur_max * 1.25, 5.0)
        ax.set_ylim(0.0, y_max_state[0])
    fig.canvas.draw()
    rgba = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(PLOT_H, PLOT_W, 4)
    return rgba[:, :, :3].copy()


def _overlay_text(frame, ctrl_name, t, err_m, force_on, ep_idx, n_ep,
                  ctrl_rgb_int, plot_inset=None):
    if not _PIL_OK:
        return frame
    img = Image.fromarray(frame)
    if plot_inset is not None:
        ph, pw = plot_inset.shape[:2]
        ix, iy = frame.shape[1] - pw - 18, 18
        ImageDraw.Draw(img).rectangle([ix-3, iy-3, ix+pw+3, iy+ph+3],
                                      outline=(80, 80, 80), width=1)
        img.paste(Image.fromarray(plot_inset), (ix, iy))
    draw = ImageDraw.Draw(img)
    x0, y, lh = 22, 20, 32
    r, g, b = ctrl_rgb_int
    draw.rectangle([x0-6, y-6, x0+560, y+lh-2], fill=(r//4, g//4, b//4, 200))
    draw.text((x0, y), ctrl_name, font=_FONT_LG, fill=(r, g, b))
    y += lh + 2
    short_desc, _ = DESCRIPTIONS.get(ctrl_name, ("", []))
    if short_desc:
        draw.text((x0, y), short_desc, font=_FONT_SM, fill=(180, 180, 100)); y += 26
    y += 4
    draw.text((x0, y), f"t = {t:5.2f} s", font=_FONT_MD, fill=(220, 220, 220)); y += lh-4
    err_mm = err_m * 1e3
    ecol = (255, 80, 80) if err_mm > 20 else (255, 200, 60) if err_mm > 5 else (120, 255, 120)
    draw.text((x0, y), f"|e| = {err_mm:6.1f} mm", font=_FONT_MD, fill=ecol); y += lh-4
    if force_on:
        draw.text((x0, y), "●  Human force  ON  (Fh = 15 N, -z)", font=_FONT_MD, fill=(255, 90, 90))
    else:
        draw.text((x0, y), "○  Free motion", font=_FONT_MD, fill=(160, 160, 160))
    draw.text((frame.shape[1]-210, 18+PLOT_H+12), f"Controller  {ep_idx} / {n_ep}",
              font=_FONT_SM, fill=(180, 180, 180))
    return np.array(img)


def _intro_card(renderer, cam, data, ctrl_name, ctrl_rgb_int, fps, duration):
    renderer.update_scene(data, camera=cam)
    bg = (renderer.render().copy() * 0.25).astype(np.uint8)
    if not _PIL_OK:
        return [bg.copy() for _ in range(int(fps * duration))]
    img = Image.fromarray(bg); draw = ImageDraw.Draw(img)
    W, H = img.size; r, g, b = ctrl_rgb_int
    x0, y = 60, 60
    draw.text((x0, y), ctrl_name, font=_FONT_LG, fill=(r, g, b)); y += 44
    draw.line([(x0, y), (W-x0, y)], fill=(r//2, g//2, b//2, 180), width=1); y += 14
    _, detail_lines = DESCRIPTIONS.get(ctrl_name, ("", []))
    for line in detail_lines:
        for wrapped in _wrap_text(line, max_chars=90):
            draw.text((x0, y), wrapped, font=_FONT_SM, fill=(220, 220, 200)); y += 26
        y += 4
    if "Kalman" in ctrl_name:
        y += 10
        draw.line([(x0, y), (W-x0, y)], fill=(80, 200, 80, 160), width=1); y += 12
        draw.text((x0, y), "Kalman Disturbance Estimator — how it achieves zero SS error",
                  font=_FONT_MD, fill=(100, 240, 100)); y += 34
        for line in [
            "The MPC state is augmented with a force-form disturbance variable d_hat (3-vector, units: N).",
            "  x_aug = [ e(k);  de(k);  d_hat(k) ]   (e = position error, de = velocity error)",
            "",
            "Prediction model:  x_aug(k+1) = A_aug * x_aug(k) + B_aug * u(k)",
            "  A_aug includes an identity block for d_hat — the model assumes disturbance is constant.",
            "",
            "Kalman measurement update (runs every QP interval):",
            "  innovation  v(k)  =  y(k)  -  C_aug * x_aug(k|k-1)      (y = measured [e; de])",
            "  d_hat(k|k)  =  d_hat(k|k-1)  +  Kf * v(k)               (Kf = Kalman gain, 3x6)",
            "",
            "Interpretation:  if the arm is pushed down (Fh = -15 N in z), the measured position",
            "  error grows larger than the model predicts (v(k) != 0).  The Kalman gain Kf maps",
            "  this residual into d_hat, which ramps toward Fh over ~1 QP interval.",
            "",
            "Once d_hat converges, the centered QP adds correction force F_mpc ~= -d_hat, cancelling the",
            "  external load.  The steady-state error is zero for constant matched disturbances.",
        ]:
            if line == "":
                y += 8; continue
            draw.text((x0, y), line, font=_FONT_SM, fill=(190, 230, 190)); y += 24
    card = np.array(img)
    return [card.copy() for _ in range(int(fps * duration))]


def _transition_card(renderer, cam, data, next_name, next_rgb_int, fps, duration):
    renderer.update_scene(data, camera=cam)
    bg = (renderer.render().copy() * 0.35).astype(np.uint8)
    if not _PIL_OK:
        return [bg.copy() for _ in range(int(fps * duration))]
    img = Image.fromarray(bg); draw = ImageDraw.Draw(img)
    W, H = img.size; r, g, b = next_rgb_int
    short_desc, _ = DESCRIPTIONS.get(next_name, ("", []))
    cx = W // 2
    draw.text((cx-90, H//2-48), "Up next", font=_FONT_SM, fill=(160, 160, 160))
    try:
        bbox = _FONT_LG.getbbox(next_name); tw = bbox[2] - bbox[0]
    except Exception:
        tw = len(next_name) * 16
    draw.text((cx - tw//2, H//2-18), next_name, font=_FONT_LG, fill=(r, g, b))
    if short_desc:
        try:
            bbox2 = _FONT_SM.getbbox(short_desc); tw2 = bbox2[2] - bbox2[0]
        except Exception:
            tw2 = len(short_desc) * 10
        draw.text((cx - tw2//2, H//2+28), short_desc, font=_FONT_SM, fill=(180, 180, 120))
    card = np.array(img)
    return [card.copy() for _ in range(int(fps * duration))]


def run_episode_video(ctrl_name, env, renderer, cam, ref_circle, *,
                      n_cycles, fps, ep_idx, n_ep, intro_duration=3.0):
    env.reset()
    ctrl_rgb_int = COLORS_INT.get(ctrl_name, (128, 128, 128))
    frames = _intro_card(renderer, cam, env.data, ctrl_name,
                         ctrl_rgb_int, fps, intro_duration)
    R_d      = np.eye(3)
    dt_sim   = env.dt
    n_steps  = int(n_cycles * PERIOD / dt_sim)
    render_every = max(1, int(round(1.0 / (fps * dt_sim))))

    ctrl = EpisodeController(ctrl_name, env, dt_mpc=MPC_DT_SLOW, hifreq_dt=None)
    ctrl_rgb_float = COLORS_RGB.get(ctrl_name, [0.5, 0.5, 0.5])

    joint_traj_fn = None
    if ctrl.mpc_ctrl is not None and ctrl.mpc_ctrl.p.horizon_torque_schedule:
        joint_traj_fn = precompute_joint_reference(
            env, env.data.qpos[:env.nv].copy(), n_cycles * PERIOD)

    def _sid(name):
        return mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, name)
    target_sid, wrist_sid = _sid("target_pos"), _sid("human_wrist")

    trail = deque(maxlen=400)
    err_fig, err_ax, err_line, err_cursor = _create_error_fig(
        ctrl_name, n_cycles, COLORS.get(ctrl_name, "#ffffff"))
    t_buf   = np.empty(n_steps)
    err_buf = np.empty(n_steps)
    y_max_state = [5.0]
    log_idx = 0

    print(f"  [{ctrl_name}] rendering {n_cycles} cycle(s) ({n_steps} steps)…")

    for i in range(n_steps):
        t = env.time
        p_d, dp_d, ddp_d = circular_ref(t)
        wrench = human_wrench(t)
        dyn, state = env.get_dynamics_and_state()
        if np.any(wrench[:3] != 0):
            env.apply_ee_wrench(wrench)

        tau, _ = ctrl.compute(state, dyn, p_d, dp_d, ddp_d, R_d, wrench, i, t=t,
                              joint_traj_fn=joint_traj_fn)
        env.apply_torque(tau)
        env.step()

        trail.append(state.ee_pos.copy())
        err_now = float(np.linalg.norm(state.ee_pos - p_d)) * 1e3  # mm
        t_buf[log_idx], err_buf[log_idx] = t, err_now
        log_idx += 1

        if target_sid >= 0:
            env.model.site_pos[target_sid] = p_d
        if wrist_sid >= 0:
            t_cyc = t % PERIOD
            if T_FORCE_ON <= t_cyc <= T_FORCE_OFF:
                env.model.site_pos[wrist_sid]     = state.ee_pos
                env.model.site_rgba[wrist_sid, 3] = 0.7
            else:
                env.model.site_rgba[wrist_sid, 3] = 0.0

        if i % render_every != 0:
            continue

        renderer.update_scene(env.data, camera=cam)
        try:
            _draw_3d_overlays(renderer.scene, state.ee_pos, p_d, trail,
                              ctrl_rgb_float, ref_circle)
        except AttributeError:
            pass
        raw = renderer.render().copy()
        plot_inset = _render_error_inset(err_fig, err_ax, err_line, err_cursor,
                                         t_buf[:log_idx], err_buf[:log_idx],
                                         t, y_max_state)
        t_cyc = t % PERIOD
        force_on = T_FORCE_ON <= t_cyc <= T_FORCE_OFF
        frames.append(_overlay_text(raw, ctrl_name, t, err_now/1e3, force_on,
                                    ep_idx, n_ep, ctrl_rgb_int, plot_inset))

    err_log_m = err_buf[:log_idx] / 1e3
    metrics = _episode_metrics(t_buf[:log_idx], err_log_m)
    plt.close(err_fig)
    return frames, metrics


def _generate_comparison_table(controller_names, all_metrics,
                               video_w, video_h, fps, duration=8.0):
    cols = ["RMS total\n(mm)", "RMS contact\n(mm)", "Peak defl.\n(mm)", "SS error\n(mm)"]
    keys = ["rms_total", "rms_contact", "peak_defl", "ss_err"]
    n_ctrl = len(controller_names)

    cell_vals, float_vals = [], {k: [] for k in keys}
    for name in controller_names:
        m = all_metrics[name]; row = [name]
        for k in keys:
            v = m[k] * 1e3; row.append(f"{v:.1f}"); float_vals[k].append(v)
        cell_vals.append(row)

    def _col_colors(vals):
        lo, hi = min(vals), max(vals); out = []
        for v in vals:
            if hi == lo:
                out.append("#2a2a3a")
            else:
                frac = (v - lo) / (hi - lo)
                r2 = int(30 + frac*140); g2 = int(120 - frac*90); b2 = int(40 - frac*20)
                out.append(f"#{r2:02x}{g2:02x}{b2:02x}")
        return out
    col_colors = {k: _col_colors(float_vals[k]) for k in keys}

    cell_colors = []
    for i in range(n_ctrl):
        row_c = ["#1a1a2e"]
        for k in keys:
            row_c.append(col_colors[k][i])
        cell_colors.append(row_c)

    fig, ax = plt.subplots(figsize=(video_w/100.0, video_h/100.0), dpi=100)
    fig.patch.set_facecolor("#0d0d1a"); ax.set_facecolor("#0d0d1a"); ax.axis("off")
    ax.text(0.5, 0.95,
            "Performance Summary — Circular Trajectory + Step Human Force (Fh = 15 N, −z)",
            ha="center", va="top", transform=ax.transAxes,
            fontsize=13, color="#e0e0ff", fontweight="bold")
    ax.text(0.5, 0.91,
            f"Reference radius {RADIUS*100:.0f} cm  |  3 cycles × 8 s  |  MuJoCo simulation @ 1 kHz",
            ha="center", va="top", transform=ax.transAxes, fontsize=9, color="#888899")

    tbl = ax.table(
        cellText=[[row[j] for j in range(5)] for row in cell_vals],
        colLabels=["Controller"] + cols, cellColours=cell_colors,
        colWidths=[0.30, 0.175, 0.175, 0.175, 0.175],
        loc="center", bbox=[0.02, 0.04, 0.96, 0.82])
    tbl.auto_set_font_size(False); tbl.set_fontsize(10)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#333355"); cell.set_linewidth(0.6)
        if r == 0:
            cell.set_facecolor("#1a1a40")
            cell.set_text_props(color="#aaaaff", fontweight="bold", fontsize=9)
        else:
            rgb = COLORS_INT.get(controller_names[r-1], (180, 180, 180))
            if c == 0:
                cell.set_text_props(color=f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}",
                                    fontweight="bold")
            else:
                cell.set_text_props(color="#e8e8e8")
    ax.text(0.5, 0.01, "Cell colour: green = best  →  red = worst  (per column)",
            ha="center", va="bottom", transform=ax.transAxes,
            fontsize=8, color="#666677", style="italic")
    fig.tight_layout(pad=0.3); fig.canvas.draw()
    rgb = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(video_h, video_w, 4)[:, :, :3].copy()
    plt.close(fig)
    return [rgb.copy() for _ in range(int(fps * duration))]


def main_video(args):
    global _FONT_LG, _FONT_MD, _FONT_SM, RADIUS
    RADIUS = args.radius

    save_dir = Path(__file__).parent.parent / "simulation_results"
    save_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.output or str(save_dir / "fr3_phri_simulation.mp4")

    if _PIL_OK:
        _FONT_LG, _FONT_MD, _FONT_SM = _load_font(28), _load_font(22), _load_font(18)
    else:
        print("[WARNING] PIL not found — text overlay disabled")

    print("=" * 60)
    print("FR3 pHRI Simulation Video")
    print(f"  Controllers  : {len(args.controllers)}")
    print(f"  Cycles each  : {args.cycles}  ({args.cycles * PERIOD:.0f} s)")
    print(f"  Resolution   : {args.width} × {args.height}  @ {args.fps} fps")
    print(f"  Radius       : {RADIUS} m")
    print(f"  Output       : {out_path}")
    print("=" * 60)

    env = FR3MuJoCoEnv()
    cam = _make_camera()
    renderer = mujoco.Renderer(env.model, height=args.height, width=args.width,
                               max_geom=10000)
    ref_circle = _ref_circle_pts(80)

    all_frames, all_metrics = [], {}
    n_ep = len(args.controllers)
    for ep_idx, ctrl_name in enumerate(args.controllers, start=1):
        ep_frames, metrics = run_episode_video(
            ctrl_name, env, renderer, cam, ref_circle,
            n_cycles=args.cycles, fps=args.fps, ep_idx=ep_idx, n_ep=n_ep)
        all_frames.extend(ep_frames)
        all_metrics[ctrl_name] = metrics
        if ep_idx < n_ep:
            next_name = args.controllers[ep_idx]
            all_frames.extend(_transition_card(
                renderer, cam, env.data, next_name,
                COLORS_INT.get(next_name, (128, 128, 128)), args.fps, args.pause))
    renderer.close()

    print("  Generating comparison table…")
    all_frames.extend(_generate_comparison_table(
        args.controllers, all_metrics, args.width, args.height, args.fps, duration=8.0))

    print(f"\n  Total frames : {len(all_frames)}")
    print(f"  Video length : {len(all_frames)/args.fps:.1f} s")
    print(f"  Saving → {out_path} …")
    try:
        import imageio.v2 as imageio
    except ImportError:
        import imageio  # type: ignore
    imageio.mimwrite(out_path, all_frames, fps=args.fps, quality=8)
    print(f"  Done. Saved {out_path}")


# ===========================================================================
#  CLI
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="FR3 pHRI benchmark — compare / focused / video")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_cmp = sub.add_parser("compare", help="paper summary plot: D1/D2/D3/D7")
    p_cmp.add_argument("--no-viewer", action="store_true")
    p_cmp.add_argument("--cycles", type=int, default=3)
    p_cmp.add_argument("--radius", type=float, default=0.12)
    p_cmp.set_defaults(func=main_compare)

    p_foc = sub.add_parser("focused", help="paper focused plot: D1/D2/D3/D7")
    p_foc.add_argument("--no-viewer", action="store_true")
    p_foc.add_argument("--cycles", type=int, default=3)
    p_foc.add_argument("--radius", type=float, default=0.12)
    p_foc.set_defaults(func=main_focused)

    p_vid = sub.add_parser("video", help="rendered MP4 walkthrough: D1 vs D7 by default")
    p_vid.add_argument("--cycles", type=int, default=3)
    p_vid.add_argument("--fps", type=int, default=30)
    p_vid.add_argument("--height", type=int, default=720)
    p_vid.add_argument("--width", type=int, default=1280)
    p_vid.add_argument("--pause", type=float, default=2.0)
    p_vid.add_argument("--radius", type=float, default=0.18)
    p_vid.add_argument("--controllers", nargs="+", default=VIDEO_CONTROLLERS)
    p_vid.add_argument("--output", type=str, default="")
    p_vid.set_defaults(func=main_video)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
