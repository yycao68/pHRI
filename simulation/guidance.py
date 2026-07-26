"""
FR3 pHRI Guidance Scenario
==========================

Contrasts three control strategies on a reach-and-hold task.

Push timing is **waypoint-relative**: the push fires PUSH_DELAY seconds
after the robot first enters each waypoint's radius, so every controller
sees exactly one push per waypoint regardless of how fast it arrives.
The robot must then recover and dwell in the radius for HOLD_AFTER seconds
before advancing to the next waypoint.

  1. Stiff Impedance      — K_d = 300 N/m, rejects push as disturbance
  2. Pure Admittance      — yields to push, recovers via virtual spring
  3. Variable Compliance  — softens on contact (K_d: 300→80 N/m),
                            snaps back to goal immediately after release

Viewer cues
-----------
  Green ball  : current target waypoint (jumps on advance)
  Red sphere  : simulated human hand (EE + 18 cm in push direction)
  Yellow flag : proportional to compliance level α
  EE trail    : controller colour when stiff → orange when compliant
  Gold stars  : waypoints A, B, C
  Green arc   : dwell countdown (fills as robot holds at waypoint)
  White lines : planned A→B→C→A path

Usage
-----
    mjpython guidance_scenario.py
    mjpython guidance_scenario.py --no-viewer
"""

from __future__ import annotations
import sys
import argparse
import time
from collections import deque
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer as mjviewer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

try:
    from PIL import Image, ImageDraw, ImageFont
    _PIL_OK = True
except ImportError:
    _PIL_OK = False

try:
    import imageio
    _IMAGEIO_OK = True
except ImportError:
    _IMAGEIO_OK = False


SIM_DIR = Path(__file__).parent.parent / "simulation"
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(Path(__file__).parent))

from fr3_impedance import (
    make_impedance_params, cartesian_impedance_control,
    AdmittanceController, make_admittance_params,
    build_operational_space_model,
)
from fr3_mujoco    import FR3MuJoCoEnv, Q_NEUTRAL
from impedance_mpc import ImpedanceMPCController, ImpedanceMPCParams
from so3_utils import rotation_error_matrix


# ---------------------------------------------------------------------------
# Waypoints — triangle in reachable workspace
# ---------------------------------------------------------------------------

WAYPOINTS = np.array([
    [0.55,  0.00, 0.50],   # A — forward, high
    [0.45,  0.22, 0.35],   # B — left, low
    [0.45, -0.22, 0.35],   # C — right, low
])
WAYPOINT_NAMES  = ["A", "B", "C"]
WAYPOINT_RADIUS = 0.035   # m — "in radius" threshold
N_LAPS          = 1

# Push fired at each waypoint after arrival
WAYPOINT_PUSH_FORCES = np.array([
    [ 0.0,  0.0, -15.0],   # A: push down   (-z)
    [ 0.0, 15.0,   0.0],   # B: push lateral (+y)
    [ 0.0,  0.0,  15.0],   # C: push up     (+z)
])
PUSH_DELAY    = 0.8   # s — wait after entering radius before push starts
PUSH_DURATION = 2.0   # s — push duration
HOLD_AFTER    = 1.0   # s — continuous in-radius hold required after push ends

EPISODE_DURATION = 22.0   # s — enough for all controllers to complete 1 lap

# ---------------------------------------------------------------------------
# Variable compliance parameters
# ---------------------------------------------------------------------------

F_CONTACT_THRESH = 5.0    # N — enter compliance mode
K_STIFF          = 300.0  # N/m — free-motion stiffness
K_SOFT           =  80.0  # N/m — compliance stiffness (15 N → ~188 mm deflection)
ALPHA_TC         =  0.08  # s — compliance transition time constant

# ---------------------------------------------------------------------------
# Colours / style
# ---------------------------------------------------------------------------

COLORS = {
    "Stiff Impedance":                  "#2196F3",
    "Pure Admittance":                  "#9C27B0",
    "Variable Compliance":              "#4CAF50",
    "Variable-Impedance MPC 100Hz":     "#795548",
    "DI-MPC 100Hz":                    "#FF9800",
    "DI-MPC + Kalman 100Hz":           "#F44336",
    "DI-MPC 500Hz":              "#00BCD4",
    "DI-MPC + Kalman 500Hz":     "#E91E63",
}
LINESTYLES = {
    "Stiff Impedance":                  "--",
    "Pure Admittance":                  ":",
    "Variable Compliance":              "-",
    "Variable-Impedance MPC 100Hz":     (0, (4, 2)),
    "DI-MPC 100Hz":                    (0, (3, 1, 1, 1)),
    "DI-MPC + Kalman 100Hz":           "-.",
    "DI-MPC 500Hz":              (0, (5, 1)),
    "DI-MPC + Kalman 500Hz":     (0, (1, 1)),
}
# Single source of truth for line weights so every curve plot emphasises the
# 4 MPC variants identically (500 Hz thicker than the slow variant; baselines
# thin; Variable Compliance kept prominent as the guidance hero).  Use
# _lw(name) everywhere instead of ad-hoc per-plot rules.
LINE_WIDTHS = {
    "Stiff Impedance":                  1.6,
    "Pure Admittance":                  1.6,
    "Variable Compliance":              2.0,
    "Variable-Impedance MPC 100Hz":     1.8,
    "DI-MPC 100Hz":              1.9,
    "DI-MPC + Kalman 100Hz":     1.9,
    "DI-MPC 500Hz":              2.4,
    "DI-MPC + Kalman 500Hz":     2.4,
}


def _lw(name: str) -> float:
    return LINE_WIDTHS.get(name, 1.6)

def _hex_to_rgb(h: str) -> list[float]:
    h = h.lstrip("#")
    return [int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4)]

COLORS_RGB = {name: _hex_to_rgb(col) for name, col in COLORS.items()}
PAUSE_SECS = 3.0


# ---------------------------------------------------------------------------
# MPC sample-rate — SINGLE SOURCE OF TRUTH
# ---------------------------------------------------------------------------
# Every run path (headless `run_episode`, plot `main_compare`, and the rendered
# `run_episode_video`) builds its DI-MPC through make_mpc_controller(),
# so the QP sample rate is defined in exactly one place and the code paths can
# never disagree.  A 50 Hz vs 100 Hz mismatch between two of these paths once
# caused the Kalman variant to diverge only in the rendered video.
#
#   • "500Hz" controllers run the QP every physics step      → dt_mpc = dt_sim
#   • all other ("100Hz") MPC controllers run it at this rate → dt_mpc = MPC_DT_SLOW
#
# To change the slow rate, change MPC_DT_SLOW here (or pass dt_slow through the
# entry points); it propagates to every run path automatically.
MPC_DT_SLOW = 0.01   # s — 100 Hz QP for the non-500Hz MPC variants

# The four DI-MPC controller names (single definition, shared by all
# run paths' "is this an MPC controller?" checks).
MPC_NAMES = (
    "DI-MPC 100Hz",
    "DI-MPC + Kalman 100Hz",
    "DI-MPC 500Hz",
    "DI-MPC + Kalman 500Hz",
    # Predictive variable-impedance baseline: routes through the same MPC path
    # (builds an ImpedanceMPCController in variable_impedance mode).
    "Variable-Impedance MPC 100Hz",
)


def make_mpc_controller(ctrl_name: str, dt_sim: float,
                        dt_slow: float = MPC_DT_SLOW):
    """Build the DI-MPC controller and its decimation factor for a
    controller name.  The ONE place the QP rate is resolved.

    Returns (mpc_ctrl, mpc_every) where the controller is already reset and the
    caller should invoke mpc_ctrl.control(...) once every `mpc_every` physics
    steps.  `dt_slow` is the single knob for the slow ("100Hz") QP period.
    """
    high_freq  = "500Hz" in ctrl_name
    variable   = "Variable-Impedance" in ctrl_name
    # MPVIC always runs the Kalman observer (it schedules stiffness on d̂ but
    # does not cancel it); the DI-MPC variants use Kalman only when named.
    use_kal    = variable or ("Kalman" in ctrl_name)
    dt_mpc_eff = dt_sim if high_freq else dt_slow
    mpc_every  = 1 if high_freq else max(1, round(dt_slow / dt_sim))
    mpc_params = ImpedanceMPCParams(
        N=10, dt_mpc=dt_mpc_eff,
        Q_pos=2e4 * np.eye(3), Q_vel=50.0 * np.eye(3),
        Q_f_scale=5.0, R_u=1e-6 * np.eye(3),
        variable_impedance=variable,
        F_max=150.0, K_rot=20.0, D_rot=6.0,
        k_null=10.0, d_null=2.0, q_null=Q_NEUTRAL,
        # Disable Cartesian workspace projection (see phri.py): it offsets p_d
        # near joint limits, and since that offset is hidden from the Kalman
        # observer it surfaces as uncorrectable "static" tracking error. The
        # null-space barrier in null_torque still guards the limits.
        k_ws=0.0,
    )
    mpc_ctrl = ImpedanceMPCController(mpc_params, use_kalman=use_kal)
    mpc_ctrl.reset()
    return mpc_ctrl, mpc_every


# ---------------------------------------------------------------------------
# user_scn helpers
# ---------------------------------------------------------------------------

def _add_line(scn, a, b, rgba, width: float = 0.003):
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    mujoco.mjv_connector(
        scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE, width,
        np.asarray(a, dtype=np.float64),
        np.asarray(b, dtype=np.float64),
    )
    scn.geoms[n].rgba[:] = rgba
    scn.ngeom += 1


def _add_sphere(scn, pos, rgba, radius: float = 0.02):
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    g = scn.geoms[n]
    g.type    = mujoco.mjtGeom.mjGEOM_SPHERE
    g.size[:] = [radius, radius, radius]
    g.pos[:]  = np.asarray(pos, dtype=np.float64)
    g.mat[:]  = np.eye(3)
    g.rgba[:] = [float(v) for v in rgba]
    scn.ngeom += 1


# ---------------------------------------------------------------------------
# Episode runner
# ---------------------------------------------------------------------------

def run_episode(
    controller_name: str,
    env:             "FR3MuJoCoEnv",
    viewer=None,
    verbose:         bool  = True,
    dt_mpc_slow:     float = MPC_DT_SLOW,   # slow-QP period (s); 500Hz vars ignore
) -> dict:
    env.reset()

    dt_sim  = env.dt
    n_steps = int(EPISODE_DURATION / dt_sim)

    def _sid(name):
        return mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, name)
    _target_sid  = _sid("target_pos")
    _wrist_sid   = _sid("human_wrist")
    _contact_sid = _sid("contact_flag")

    # Waypoint / push state
    wp_idx            = 0
    waypoints_reached = 0
    max_wp            = N_LAPS * len(WAYPOINTS)

    # entry_time[k] = sim time when robot first entered radius at waypoint k
    entry_time  = [None] * len(WAYPOINTS)
    _dwell_timer = 0.0   # counts continuous in-radius time after push ends

    # Compliance state
    alpha = 0.0

    # Controllers
    R_d      = np.eye(3)
    imp_base = make_impedance_params(
        k_pos=K_STIFF, k_rot=20.0, damping_ratio=1.0, q_null=Q_NEUTRAL
    )
    adm_ctrl = None
    mpc_ctrl = None
    tau_cached   = np.zeros(7)
    F_mpc_cached = np.zeros(3)

    if controller_name == "Pure Admittance":
        adm_ctrl = AdmittanceController(
            make_admittance_params(m_pos=0.5, d_pos=15.0, k_pos=100.0),
            dt=dt_sim,
        )
    elif controller_name in MPC_NAMES:
        mpc_ctrl, mpc_every = make_mpc_controller(
            controller_name, dt_sim, dt_mpc_slow)

    if verbose:
        mpc_rate = ""
        if mpc_ctrl is not None:
            rate_hz = 1.0 / (mpc_every * dt_sim)
            mpc_rate = f", MPC {rate_hz:.0f} Hz"
        print(f"\n[{controller_name}]  {EPISODE_DURATION:.0f} s, "
              f"{N_LAPS} lap × {len(WAYPOINTS)} waypoints  "
              f"(push fires {PUSH_DELAY} s after arrival{mpc_rate}) …")

    if viewer is not None:
        viewer.set_texts((
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOP,
            controller_name, "",
        ))

    TRAIL_LEN    = 1200
    TRAIL_STEP   = 4
    _ctrl_rgb    = COLORS_RGB.get(controller_name, [0.5, 0.5, 0.5])
    _ee_trail    = deque(maxlen=TRAIL_LEN)
    _alpha_trail = deque(maxlen=TRAIL_LEN)

    t_log       = np.zeros(n_steps)
    err_log     = np.zeros(n_steps)
    contact_log = np.zeros(n_steps, dtype=bool)
    alpha_log   = np.zeros(n_steps)
    ee_pos_log  = np.zeros((n_steps, 3))
    wp_pos_log  = np.zeros((n_steps, 3))

    t_wall_start = time.perf_counter()

    for i in range(n_steps):
        t   = env.time
        wpk = wp_idx % len(WAYPOINTS)  # current waypoint index 0-2
        p_d = WAYPOINTS[wpk]

        dyn, state = env.get_dynamics_and_state()
        dist = float(np.linalg.norm(state.ee_pos - p_d))

        # Record first entry into radius
        if dist < WAYPOINT_RADIUS and entry_time[wpk] is None:
            entry_time[wpk] = t

        # Compute push window for current waypoint
        t_entry    = entry_time[wpk]
        t_push_on  = (t_entry + PUSH_DELAY)         if t_entry is not None else np.inf
        t_push_off = (t_entry + PUSH_DELAY + PUSH_DURATION) if t_entry is not None else np.inf
        push_active = (t_push_on <= t <= t_push_off) and (waypoints_reached < max_wp)

        wrench = np.zeros(6)
        if push_active:
            wrench[:3] = WAYPOINT_PUSH_FORCES[wpk]

        # Apply simulated human force
        if wrench.any():
            env.apply_ee_wrench(wrench)

        # Dwell counter: only accumulates after push ends and robot is in radius
        if dist < WAYPOINT_RADIUS and t > t_push_off:
            _dwell_timer += dt_sim
        else:
            _dwell_timer = 0.0

        # Advance waypoint
        if _dwell_timer >= HOLD_AFTER and waypoints_reached < max_wp:
            _dwell_timer  = 0.0
            entry_time[wpk] = None
            wp_idx            += 1
            waypoints_reached += 1
            wpk_new = wp_idx % len(WAYPOINTS)
            if verbose:
                wp_nm = WAYPOINT_NAMES[wpk_new] if waypoints_reached < max_wp else "done"
                print(f"  t={t:.1f}s  → waypoint {waypoints_reached}/{max_wp} "
                      f"reached, next: {wp_nm}")
            p_d = WAYPOINTS[wp_idx % len(WAYPOINTS)]
            # Clear stale disturbance estimate so Kalman doesn't inject last
            # waypoint's push force as a prior when approaching the next target.
            if mpc_ctrl is not None:
                mpc_ctrl.reset()

        # Compliance alpha
        f_mag = float(np.linalg.norm(wrench[:3]))
        if controller_name == "Stiff Impedance":
            target_alpha = 0.0
        elif controller_name == "Pure Admittance":
            target_alpha = 1.0
        else:
            target_alpha = 1.0 if f_mag > F_CONTACT_THRESH else 0.0
        alpha += (target_alpha - alpha) * dt_sim / ALPHA_TC
        alpha  = float(np.clip(alpha, 0.0, 1.0))

        # ── Compute torque ────────────────────────────────────────────────
        dx_d_6d  = np.zeros(6)
        ddx_d_6d = np.zeros(6)

        if controller_name == "Stiff Impedance":
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, imp_base
            )
            tau += env.null_space_gravity_comp(dyn)

        elif controller_name == "Pure Admittance":
            x_r, v_r = adm_ctrl.step(wrench)
            x_cmd    = p_d + x_r[:3]
            dx_cmd   = np.concatenate([v_r[:3], np.zeros(3)])
            tau = cartesian_impedance_control(
                state, dyn, x_cmd, R_d, dx_cmd, ddx_d_6d, imp_base
            )
            tau += env.null_space_gravity_comp(dyn)

        elif mpc_ctrl is not None:
            # DI-MPC: target is current waypoint (static hold)
            if i % mpc_every == 0:
                tau_cached, F_mpc_cached = mpc_ctrl.control(
                    state.ee_pos, state.ee_vel, state.ee_rot,
                    p_d, np.zeros(3), np.zeros(3), R_d,
                    dyn, state.q, state.dq,
                )
                tau = tau_cached
            else:
                # 1 kHz inner loop: feedforward, orientation, and
                # null-space torques are recomputed every tick from
                # fresh (q, dq); only the QP correction F_mpc is held
                # from the last solve.
                J_v, J_w = dyn.J[:3, :], dyn.J[3:, :]
                tau_ff = dyn.Cq_dot  # static hold reference: ddp_d = 0
                e_R    = rotation_error_matrix(R_d, state.ee_rot)
                p_mpc  = mpc_ctrl.p
                tau_or = J_w.T @ (-p_mpc.K_rot * e_R - p_mpc.D_rot * state.ee_vel[3:])
                N_bar  = build_operational_space_model(dyn, state.ee_vel).N_bar
                tau    = (tau_ff + J_v.T @ F_mpc_cached + tau_or
                          + mpc_ctrl.null_torque(state.q, state.dq, N_bar))

        else:  # Variable Compliance
            k_eff  = K_STIFF * (1.0 - alpha) + K_SOFT * alpha
            imp_vc = make_impedance_params(
                k_pos=k_eff, k_rot=20.0, damping_ratio=1.0, q_null=Q_NEUTRAL
            )
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, imp_vc
            )
            tau += env.null_space_gravity_comp(dyn)

        env.apply_torque(tau)
        env.step()

        err            = float(np.linalg.norm(state.ee_pos - p_d))
        t_log[i]       = t
        err_log[i]     = err
        contact_log[i] = push_active
        alpha_log[i]   = alpha
        ee_pos_log[i]  = state.ee_pos
        wp_pos_log[i]  = p_d

        if verbose and i % 2000 == 0:
            entry_str = f"{t - t_entry:.1f}s" if t_entry is not None else "—"
            print(f"  t={t:.1f}s  wp={WAYPOINT_NAMES[wp_idx%3]}"
                  f"  |e|={err*1e3:.0f}mm  α={alpha:.2f}"
                  f"  push={'ON' if push_active else 'off'}"
                  f"  dwell={_dwell_timer:.1f}s"
                  f"  in_wp={entry_str}")

        # ── Viewer sync ───────────────────────────────────────────────────
        if viewer is not None:
            if _target_sid >= 0:
                env.model.site_pos[_target_sid] = p_d

            if _wrist_sid >= 0:
                f_now = wrench[:3]
                f_norm = np.linalg.norm(f_now)
                if f_norm > 0:
                    env.model.site_pos[_wrist_sid]     = (
                        state.ee_pos + 0.18 * f_now / f_norm
                    )
                    env.model.site_rgba[_wrist_sid, 3] = 0.85
                else:
                    env.model.site_rgba[_wrist_sid, 3] = 0.0

            if _contact_sid >= 0:
                env.model.site_rgba[_contact_sid, 3] = float(0.8 * alpha)

            _ee_trail.append(state.ee_pos.copy())
            _alpha_trail.append(alpha)
            scn = viewer.user_scn
            scn.ngeom = 0

            # Planned path (white, faint)
            for k in range(len(WAYPOINTS)):
                _add_line(scn,
                          WAYPOINTS[k], WAYPOINTS[(k + 1) % len(WAYPOINTS)],
                          [0.85, 0.85, 0.85, 0.28], width=0.002)

            # Waypoint markers — current is brighter and larger
            current_wpk = wp_idx % len(WAYPOINTS)
            for k, wp in enumerate(WAYPOINTS):
                is_cur = (k == current_wpk)
                rgba   = [1.0, 0.85, 0.1, 0.9 if is_cur else 0.4]
                _add_sphere(scn, wp, rgba, radius=0.030 if is_cur else 0.018)

            # Dwell progress arc (green, fills clockwise around current waypoint)
            dwell_frac = min(1.0, _dwell_timer / HOLD_AFTER)
            if dwell_frac > 0:
                n_arc = max(1, int(dwell_frac * 24))
                theta = np.linspace(0, 2 * np.pi * dwell_frac, n_arc + 1)
                arc   = np.column_stack([
                    p_d[0] + 0.05 * np.cos(theta),
                    np.full(n_arc + 1, p_d[1]),
                    p_d[2] + 0.05 * np.sin(theta),
                ])
                for k in range(n_arc):
                    _add_line(scn, arc[k], arc[k + 1],
                              [0.2, 1.0, 0.2, 0.85], width=0.003)

            # EE trail — blends controller colour → orange as alpha rises
            trail = list(_ee_trail)
            al_t  = list(_alpha_trail)
            n_seg = len(trail) - 1
            for k in range(0, n_seg, TRAIL_STEP):
                a_k  = al_t[k]
                rgba = [
                    _ctrl_rgb[0] * (1 - a_k) + 1.0 * a_k,
                    _ctrl_rgb[1] * (1 - a_k) + 0.5 * a_k,
                    _ctrl_rgb[2] * (1 - a_k) + 0.0 * a_k,
                    0.85,
                ]
                _add_line(scn, trail[k], trail[k + 1], rgba, width=0.004)

            # Error line EE → waypoint (red, only when large)
            if err > 0.06:
                _add_line(scn, state.ee_pos, p_d,
                          [1.0, 0.1, 0.1, 0.65], width=0.002)

            # Real-time pacing
            t_sim  = (i + 1) * dt_sim
            t_wall = time.perf_counter() - t_wall_start
            if t_sim > t_wall:
                time.sleep(t_sim - t_wall)
            viewer.sync()

    # ── Metrics ──────────────────────────────────────────────────────────
    mask_free = ~contact_log
    mask_cont =  contact_log
    rms_free    = float(np.sqrt(np.mean(err_log[mask_free]**2))) if mask_free.any() else float('nan')
    rms_contact = float(np.sqrt(np.mean(err_log[mask_cont]**2))) if mask_cont.any() else float('nan')
    peak_defl   = float(np.max(err_log[mask_cont]))              if mask_cont.any() else float('nan')

    if verbose:
        print(f"  → Waypoints reached : {waypoints_reached}/{max_wp}")
        print(f"  → RMS free-motion   : {rms_free*1e3:.1f} mm")
        print(f"  → RMS contact       : {rms_contact*1e3:.1f} mm")
        print(f"  → Peak deflection   : {peak_defl*1e3:.1f} mm")

    return dict(
        t=t_log, pos_err=err_log, contact=contact_log, alpha=alpha_log,
        ee_pos=ee_pos_log, wp_pos=wp_pos_log,
        waypoints_reached=waypoints_reached,
        rms_free=rms_free, rms_contact=rms_contact, peak_defl=peak_defl,
    )


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_results(results: dict, save_path: str | None = None):
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(
        "FR3 pHRI Guidance: Reach-and-Hold under Human Push\n"
        "(trail: controller colour = stiff  →  orange = compliant; "
        "push fires per waypoint after arrival)",
        fontsize=11, fontweight='bold',
    )
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.48, wspace=0.38)

    ax_err  = fig.add_subplot(gs[0, :2])
    ax_alph = fig.add_subplot(gs[0, 2])
    ax_xz   = fig.add_subplot(gs[1, 0])
    ax_xy   = fig.add_subplot(gs[1, 1])
    ax_bar  = fig.add_subplot(gs[1, 2])

    for name, data in results.items():
        t  = data["t"]
        c  = COLORS[name]
        ls = LINESTYLES[name]
        lw = _lw(name)

        ax_err.plot(t, data["pos_err"] * 1e3, color=c, ls=ls, lw=lw, label=name)
        ax_alph.plot(t, data["alpha"],          color=c, ls=ls, lw=lw, label=name)

        ee = data["ee_pos"]
        ax_xz.plot(ee[:, 0] * 1e2, ee[:, 2] * 1e2, color=c, ls=ls, lw=lw, label=name)
        ax_xy.plot(ee[:, 0] * 1e2, ee[:, 1] * 1e2, color=c, ls=ls, lw=lw, label=name)

        # Shade push windows per controller
        contact = data["contact"]
        t_arr   = t
        in_push = False
        t_start = None
        for j, active in enumerate(contact):
            if active and not in_push:
                in_push = True
                t_start = t_arr[j]
            elif not active and in_push:
                in_push = False
                ax_err.axvspan(t_start, t_arr[j], alpha=0.06, color=c)
        if in_push:
            ax_err.axvspan(t_start, t_arr[-1], alpha=0.06, color=c)

    wp_x = WAYPOINTS[:, 0] * 1e2
    wp_y = WAYPOINTS[:, 1] * 1e2
    wp_z = WAYPOINTS[:, 2] * 1e2
    for ax, ys, ylbl, title in [
        (ax_xz, wp_z, "z (cm)", "xz-plane trajectory"),
        (ax_xy, wp_y, "y (cm)", "xy-plane trajectory"),
    ]:
        ax.scatter(wp_x, ys, c='gold', s=140, zorder=6, marker='*',
                   edgecolors='k', linewidths=0.5, label="Waypoints")
        labels = ["A", "B", "C"]
        for k in range(len(WAYPOINTS)):
            ax.annotate(labels[k], (wp_x[k]+0.5, ys[k]+0.5), fontsize=9)
            nk = (k + 1) % len(WAYPOINTS)
            ax.plot([wp_x[k], wp_x[nk]], [ys[k], ys[nk]], 'k:', lw=1, alpha=0.3)
        ax.set_xlabel("x (cm)")
        ax.set_ylabel(ylbl)
        ax.set_title(title)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")

    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Error to current waypoint (mm)")
    ax_err.set_title("Waypoint tracking error  (shaded = each controller's push)")
    ax_err.legend(fontsize=8, ncol=2)
    ax_err.grid(True, alpha=0.3)

    ax_alph.set_xlabel("Time (s)")
    ax_alph.set_ylabel("Compliance α  (0=stiff, 1=soft)")
    ax_alph.set_title("Compliance level")
    ax_alph.set_ylim(-0.05, 1.05)
    ax_alph.legend(fontsize=8)
    ax_alph.grid(True, alpha=0.3)

    metrics = ["rms_free", "rms_contact", "peak_defl"]
    labels  = ["RMS free\n(mm)", "RMS contact\n(mm)", "Peak defl.\n(mm)"]
    x       = np.arange(len(metrics))
    n_ctrl  = len(results)
    w       = min(0.22, 0.9 / n_ctrl)
    offsets = np.linspace(-(n_ctrl - 1) / 2, (n_ctrl - 1) / 2, n_ctrl) * w
    for k, (name, data) in enumerate(results.items()):
        vals = [data[m] * 1e3 for m in metrics]
        ax_bar.bar(x + offsets[k], vals, w,
                   color=COLORS[name], label=name, alpha=0.85)
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(labels, fontsize=8)
    ax_bar.set_ylabel("Error (mm)")
    ax_bar.set_title("Performance summary")
    ax_bar.legend(fontsize=7)
    ax_bar.grid(True, axis='y', alpha=0.3)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n[plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main_demo():
    parser = argparse.ArgumentParser(
        description="FR3 pHRI guidance: reach-and-hold with waypoint-relative pushes"
    )
    parser.add_argument("--no-viewer", action="store_true",
                        help="Run headless (no MuJoCo viewer)")
    args = parser.parse_args()
    show_viewer = not args.no_viewer

    SAVE_DIR = Path(__file__).parent.parent / "simulation_results"
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    save_path = str(SAVE_DIR / "guidance_results.png")

    controllers = [
        "Stiff Impedance",
        "Pure Admittance",
        "Variable Compliance",
        "DI-MPC 100Hz",
        "DI-MPC + Kalman 100Hz",
        "DI-MPC 500Hz",
        "DI-MPC + Kalman 500Hz",
    ]

    print("=" * 64)
    print("FR3 pHRI Guidance: Reach-and-Hold under Waypoint-Relative Pushes")
    print(f"  Waypoints : A{WAYPOINTS[0]}  B{WAYPOINTS[1]}  C{WAYPOINTS[2]}")
    print(f"  Push      : 15 N, fires {PUSH_DELAY} s after arrival, "
          f"lasts {PUSH_DURATION} s")
    print(f"  Advance   : {HOLD_AFTER} s continuous hold in radius after push ends")
    print(f"  Duration  : {EPISODE_DURATION:.0f} s")
    print("=" * 64)

    env = FR3MuJoCoEnv()

    def _run_all(viewer=None):
        results = {}
        for k, name in enumerate(controllers):
            results[name] = run_episode(name, env=env, viewer=viewer, verbose=True)
            if viewer is not None and k < len(controllers) - 1:
                next_name = controllers[k + 1]
                print(f"\n  ── Holding {PAUSE_SECS:.0f} s — next: {next_name} ──")
                viewer.set_texts((
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOP,
                    f"Next: {next_name}", "",
                ))
                t_end = time.perf_counter() + PAUSE_SECS
                while time.perf_counter() < t_end:
                    viewer.sync()
                    time.sleep(0.02)
        return results

    if show_viewer:
        with mjviewer.launch_passive(env.model, env.data) as viewer:
            viewer.cam.azimuth   = 145.0
            viewer.cam.elevation = -18.0
            viewer.cam.distance  =  2.0
            viewer.cam.lookat[:] = [0.47, 0.0, 0.42]
            results = _run_all(viewer)
    else:
        results = _run_all()

    print("\n" + "=" * 72)
    print(f"{'Controller':<32} {'Reached':>9} {'RMS free':>10} "
          f"{'RMS contact':>13} {'Peak':>8}")
    print("-" * 72)
    for name, data in results.items():
        print(f"{name:<32} "
              f"{data['waypoints_reached']:>5}/{N_LAPS*len(WAYPOINTS):<4}"
              f"{data['rms_free']*1e3:>10.1f} "
              f"{data['rms_contact']*1e3:>13.1f} "
              f"{data['peak_defl']*1e3:>8.1f}")
    print("=" * 72)

    fig = plot_results(results, save_path=save_path)
    plt.close(fig)



#==========================================================================
# Benchmark comparison (Table IV) -- from guidance_focused_comparison.py
#==========================================================================

ALL_CONTROLLERS = [
    "Stiff Impedance",
    "Pure Admittance",
    "Variable Compliance",
    "DI-MPC 100Hz",
    "DI-MPC + Kalman 100Hz",
    "DI-MPC 500Hz",
    "DI-MPC + Kalman 500Hz",
]

PAPER_CONTROLLERS = [
    "Stiff Impedance",
    "Pure Admittance",
    "Variable Compliance",
    "Variable-Impedance MPC 100Hz",
    "DI-MPC + Kalman 500Hz",
]
PAPER_LABELS = {
    "Stiff Impedance": "D1 Stiff Imp.",
    "Pure Admittance": "D2 Pure Adm.",
    "Variable Compliance": "D3 Var. Compl.",
    "Variable-Impedance MPC 100Hz": "MPVIC Var.-Imp. MPC",
    "DI-MPC + Kalman 500Hz": "D7 DI-MPC+K 500Hz",
}
VIDEO_CONTROLLERS = [
    "Stiff Impedance",
    "DI-MPC + Kalman 500Hz",
]

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _shade_pushes(ax, data, color, alpha_fill=0.10):
    """Shade every push window for one controller's data."""
    contact = data["contact"]
    t_arr   = data["t"]
    in_push = False
    t_start = None
    for j, active in enumerate(contact):
        if active and not in_push:
            in_push = True;  t_start = t_arr[j]
        elif not active and in_push:
            in_push = False
            ax.axvspan(t_start, t_arr[j], alpha=alpha_fill, color=color)
    if in_push:
        ax.axvspan(t_start, t_arr[-1], alpha=alpha_fill, color=color)


def _draw_waypoints(ax, wp_x, wp_y, annotate=True):
    ax.scatter(wp_x, wp_y, c='gold', s=140, zorder=6, marker='*',
               edgecolors='k', linewidths=0.5, label="Waypoints")
    for k in range(len(WAYPOINTS)):
        if annotate:
            ax.annotate(WAYPOINT_NAMES[k], (wp_x[k] + 0.5, wp_y[k] + 0.5), fontsize=9)
        nk = (k + 1) % len(WAYPOINTS)
        ax.plot([wp_x[k], wp_x[nk]], [wp_y[k], wp_y[nk]], 'k:', lw=1, alpha=0.3)


# ---------------------------------------------------------------------------
# Plot 1: Five-controller paradigm comparison
# ---------------------------------------------------------------------------

def _paper_subset(results: dict) -> dict:
    return {k: results[k] for k in PAPER_CONTROLLERS if k in results}


def plot_controller_comparison(results: dict, save_path: str | None = None, paper_only: bool = True):
    """Paper-readable comparison plot."""
    subset = _paper_subset(results) if paper_only else {k: results[k] for k in ALL_CONTROLLERS if k in results}

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(
        "FR3 pHRI Guidance Benchmark — Controller Comparison\n"
        "(Reach-and-Hold under Waypoint-Relative Human Push — D1/D2/D3/D7 + MPVIC baseline shown)",
        fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.52, wspace=0.38)

    ax_err  = fig.add_subplot(gs[0, :2])   # error time series (wide)
    ax_alph = fig.add_subplot(gs[0, 2])    # compliance α
    ax_xz   = fig.add_subplot(gs[1, 0])    # xz trajectory
    ax_xy   = fig.add_subplot(gs[1, 1])    # xy trajectory
    ax_bar  = fig.add_subplot(gs[1, 2])    # bar chart
    ax_err2 = fig.add_subplot(gs[2, :2])   # zoomed error (MPC only, no admittance)
    ax_info = fig.add_subplot(gs[2, 2])    # waypoint advance table

    wp_x = WAYPOINTS[:, 0] * 1e2
    wp_z = WAYPOINTS[:, 2] * 1e2
    wp_y = WAYPOINTS[:, 1] * 1e2

    for name, data in subset.items():
        t  = data["t"]
        c  = COLORS[name]
        ls = LINESTYLES[name]
        lw = _lw(name)

        label = PAPER_LABELS.get(name, name)
        ax_err.plot(t, data["pos_err"] * 1e3, color=c, ls=ls, lw=lw, label=label)
        ax_alph.plot(t, data["alpha"],          color=c, ls=ls, lw=lw, label=label)
        ax_xz.plot(data["ee_pos"][:, 0] * 1e2,
                   data["ee_pos"][:, 2] * 1e2,
                   color=c, ls=ls, lw=lw, label=label)
        ax_xy.plot(data["ee_pos"][:, 0] * 1e2,
                   data["ee_pos"][:, 1] * 1e2,
                   color=c, ls=ls, lw=lw, label=label)

        _shade_pushes(ax_err, data, c, alpha_fill=0.07)

        # Zoomed panel — only show MPC variants (skip reactive for scale)
        if "MPC" in name:
            ax_err2.plot(t, data["pos_err"] * 1e3, color=c, ls=ls, lw=lw, label=name)
            _shade_pushes(ax_err2, data, c, alpha_fill=0.10)

    # Waypoint markers (no in-panel legend: the equal-aspect trajectory panels
    # are too small to hold it without covering the curves — a single shared
    # figure legend is drawn at the bottom instead).
    for ax, ys in [(ax_xz, wp_z), (ax_xy, wp_y)]:
        _draw_waypoints(ax, wp_x, ys)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    ax_xz.set_xlabel("x (cm)"); ax_xz.set_ylabel("z (cm)")
    ax_xz.set_title("xz-plane trajectory")
    ax_xy.set_xlabel("x (cm)"); ax_xy.set_ylabel("y (cm)")
    ax_xy.set_title("xy-plane trajectory")

    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Error to waypoint (mm)")
    ax_err.set_title("Waypoint tracking error (shaded = per-controller push window)")
    ax_err.grid(True, alpha=0.3)

    ax_err2.set_xlabel("Time (s)")
    ax_err2.set_ylabel("Error (mm)")
    ax_err2.set_title("D7 zoomed view (reactive controllers excluded for scale)")
    ax_err2.grid(True, alpha=0.3)
    ax_err2.set_ylim(bottom=0)

    ax_alph.set_xlabel("Time (s)")
    ax_alph.set_ylabel("Compliance α")
    ax_alph.set_title("Compliance level\n(0 = stiff, 1 = fully soft)")
    ax_alph.set_ylim(-0.05, 1.05)
    ax_alph.grid(True, alpha=0.3)

    # Bar chart: performance summary
    metrics = ["rms_free", "rms_contact", "peak_defl"]
    xlabels = ["RMS free\n(mm)", "RMS contact\n(mm)", "Peak defl.\n(mm)"]
    x       = np.arange(len(metrics))
    n_ctrl  = len(subset)
    w       = 0.15
    offsets = np.linspace(-(n_ctrl-1)/2, (n_ctrl-1)/2, n_ctrl) * w

    for k, (name, data) in enumerate(subset.items()):
        vals = [data[m] * 1e3 for m in metrics]
        ax_bar.bar(x + offsets[k], vals, w, color=COLORS[name], label=name, alpha=0.85)

    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(xlabels, fontsize=8)
    ax_bar.set_ylabel("Error (mm)")
    ax_bar.set_title("Performance summary")
    ax_bar.grid(True, axis='y', alpha=0.3)

    # Info table: waypoints reached + metrics.
    # Short controller labels (matching the paper's Table III) keep the first
    # column readable; the bottom legend maps full names to colors.
    _ABBR = {
        "Stiff Impedance":              "Stiff Imp.",
        "Pure Admittance":              "Pure Adm.",
        "Variable Compliance":          "Var. Compl.",
        "DI-MPC 100Hz":          "MPC 100Hz",
        "DI-MPC + Kalman 100Hz": "MPC+K 100Hz",
        "DI-MPC 500Hz":          "MPC 500Hz",
        "DI-MPC + Kalman 500Hz": "MPC+K 500Hz",
    }
    max_wp = N_LAPS * len(WAYPOINTS)
    col_labels = ["Controller", "Reached", "RMS free\n(mm)",
                  "RMS contact\n(mm)", "Peak\n(mm)"]
    cell_vals  = []
    for name, data in subset.items():
        cell_vals.append([
            PAPER_LABELS.get(name, _ABBR.get(name, name)),
            f"{data['waypoints_reached']}/{max_wp}",
            f"{data['rms_free']*1e3:.1f}",
            f"{data['rms_contact']*1e3:.1f}",
            f"{data['peak_defl']*1e3:.1f}",
        ])
    ax_info.axis('off')
    tbl = ax_info.table(
        cellText=cell_vals,
        colLabels=col_labels,
        colWidths=[0.34, 0.16, 0.17, 0.18, 0.15],
        loc='center', cellLoc='center',
    )
    # Left-align and color the controller-name column to match the curves.
    names = list(subset.keys())
    for r, nm in enumerate(names):
        cell = tbl[r + 1, 0]          # +1 to skip the header row
        cell.set_text_props(ha='left', color=COLORS[nm])
        cell.PAD = 0.04
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7)
    tbl.scale(1.0, 1.4)
    ax_info.set_title("Metric summary", fontsize=9, pad=6)

    # Single shared legend for the whole figure, placed separately along the
    # bottom so it never overlaps any panel (controllers from ax_err, plus the
    # waypoint marker from the trajectory panels).
    handles, labels = ax_err.get_legend_handles_labels()
    h_xz, l_xz = ax_xz.get_legend_handles_labels()
    for h, l in zip(h_xz, l_xz):
        if l == "Waypoints" and l not in labels:
            handles.append(h); labels.append(l)
    fig.legend(handles, labels, loc="lower center", ncol=len(labels),
               fontsize=8, frameon=True, bbox_to_anchor=(0.5, 0.0))

    fig.tight_layout(rect=[0, 0.05, 1, 0.94])
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[plot1] Saved → {save_path}")
    return fig


# ---------------------------------------------------------------------------
# Plot 2: Frequency effect
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main_compare():
    parser = argparse.ArgumentParser(
        description="FR3 pHRI guidance focused comparison: paradigms + frequency effect")
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()
    show_viewer = not args.no_viewer

    SAVE_DIR = Path(__file__).parent.parent / "simulation_results"
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    path_plot1 = str(SAVE_DIR / "guidance_controller_comparison.png")

    print("=" * 68)
    print("FR3 pHRI Guidance Benchmark — Focused Comparisons")
    print(f"  Waypoints  : A{WAYPOINTS[0]}  B{WAYPOINTS[1]}  C{WAYPOINTS[2]}")
    print(f"  Push       : 15 N, fires {PUSH_DELAY} s after arrival, "
          f"lasts {PUSH_DURATION} s")
    print(f"  Advance    : {HOLD_AFTER} s continuous hold after push ends")
    print(f"  Duration   : {EPISODE_DURATION:.0f} s per controller")
    print("=" * 68)

    env = FR3MuJoCoEnv()

    def _run_all(viewer=None):
        results = {}
        for k, name in enumerate(PAPER_CONTROLLERS):
            if viewer is not None:
                viewer.set_texts((
                    mujoco.mjtFontScale.mjFONTSCALE_150,
                    mujoco.mjtGridPos.mjGRID_TOP,
                    name, "",
                ))
            results[name] = run_episode(
                name, env=env, viewer=viewer, verbose=True)
            if viewer is not None and k < len(PAPER_CONTROLLERS) - 1:
                print(f"\n  ── Pausing {PAUSE_SECS:.0f} s before next controller ──")
                t_end = time.perf_counter() + PAUSE_SECS
                while time.perf_counter() < t_end:
                    viewer.sync(); time.sleep(0.02)
        return results

    if show_viewer:
        with mjviewer.launch_passive(env.model, env.data) as viewer:
            viewer.cam.azimuth   = 145.0
            viewer.cam.elevation = -18.0
            viewer.cam.distance  =  2.0
            viewer.cam.lookat[:] = [0.47, 0.0, 0.42]
            results = _run_all(viewer)
    else:
        results = _run_all()

    # Summary table
    max_wp = N_LAPS * len(WAYPOINTS)
    print("\n" + "=" * 72)
    print(f"{'Controller':<32} {'Reached':>9} {'RMS free':>10} "
          f"{'RMS contact':>13} {'Peak':>8}")
    print("-" * 72)
    for name, data in results.items():
        print(f"{name:<32} "
              f"{data['waypoints_reached']:>5}/{max_wp:<4}"
              f"{data['rms_free']*1e3:>10.1f} "
              f"{data['rms_contact']*1e3:>13.1f} "
              f"{data['peak_defl']*1e3:>8.1f}")
    print("=" * 72)

    fig1 = plot_controller_comparison(results, save_path=path_plot1, paper_only=True)
    plt.close(fig1)

    print("\nDone. Output file:")
    print(f"  {path_plot1}")



#==========================================================================
# Narrated video renderer -- from guidance_video.py
#==========================================================================

WAYPOINTS = np.array([
    [0.55,  0.00, 0.50],
    [0.45,  0.22, 0.35],
    [0.45, -0.22, 0.35],
])
WAYPOINT_NAMES = ["A", "B", "C"]
WAYPOINT_RADIUS = 0.035

WAYPOINT_PUSH_FORCES = np.array([
    [ 0.0,  0.0, -15.0],
    [ 0.0, 15.0,   0.0],
    [ 0.0,  0.0,  15.0],
])
PUSH_DELAY    = 0.8
PUSH_DURATION = 2.0
HOLD_AFTER    = 1.0

VIDEO_N_LAPS           = 2
VIDEO_EPISODE_DURATION = 44.0   # 22 s baseline × 2 laps

F_CONTACT_THRESH = 5.0
K_STIFF          = 300.0
K_SOFT           =  80.0
ALPHA_TC         =  0.08
# MPC sample rate lives in MPC_DT_SLOW (single source of truth) near the top.

# ---------------------------------------------------------------------------
# Controllers & colours
# ---------------------------------------------------------------------------

ALL_CONTROLLERS = [
    "Stiff Impedance",
    "Pure Admittance",
    "Variable Compliance",
    "DI-MPC 100Hz",
    "DI-MPC + Kalman 100Hz",
    "DI-MPC 500Hz",
    "DI-MPC + Kalman 500Hz",
]

COLORS_HEX = {
    "Stiff Impedance":                  "#2196F3",
    "Pure Admittance":                  "#9C27B0",
    "Variable Compliance":              "#4CAF50",
    "DI-MPC 100Hz":                    "#FF9800",
    "DI-MPC + Kalman 100Hz":           "#F44336",
    "DI-MPC 500Hz":              "#00BCD4",
    "DI-MPC + Kalman 500Hz":     "#E91E63",
}


def _hex_to_rgb_int(h: str) -> tuple[int, int, int]:
    h = h.lstrip("#")
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))


def _hex_to_rgb_float(h: str) -> list[float]:
    return [v / 255.0 for v in _hex_to_rgb_int(h)]


COLORS_INT   = {n: _hex_to_rgb_int(c)   for n, c in COLORS_HEX.items()}
COLORS_FLOAT = {n: _hex_to_rgb_float(c) for n, c in COLORS_HEX.items()}

# ---------------------------------------------------------------------------
# Per-controller descriptions
# ---------------------------------------------------------------------------

DESCRIPTIONS = {
    "Stiff Impedance": (
        "K=300 N/m, rejects push — SS deflection ≈ 50 mm",
        [
            "Classical Cartesian impedance:  τ = J^T[ Λ(ẍ_d − K·e − D·ė) + μ + p ]",
            "Spring stiffness K = 300 N/m resists the 15 N push.",
            "Steady-state deflection = F / K = 15 / 300 ≈ 50 mm  (permanent while force persists).",
            "No integral term, no estimator — deflection cannot be driven to zero.",
            "Safe and deterministic, but accuracy degrades proportionally to applied force.",
        ],
    ),
    "Pure Admittance": (
        "Yields to push by design — SS deflection ≈ 150 mm",
        [
            "Admittance model:  M_a ẍ_r + D_a ẋ_r + K_a x_r = F_h",
            "Virtual parameters: M=0.5 kg, D=15 N·s/m, K=100 N/m.",
            "Human force becomes a reference velocity input — arm intentionally moves with operator.",
            "Steady-state deflection = F / K_a = 15/100 = 150 mm.  Large yield is the design intent.",
            "Very safe for physical interaction; poor position hold under unexpected disturbances.",
        ],
    ),
    "Variable Compliance": (
        "K: 300→80 N/m on contact, snaps back after release",
        [
            "Detects contact when |F_ext| > 5 N; ramps compliance α smoothly: 0 → 1 (τ_c = 0.08 s).",
            "Effective stiffness:  K_eff = K_stiff·(1−α) + K_soft·α  →  range 300 to 80 N/m.",
            "Under 15 N push: yields ~188 mm at K=80 N/m — safe, compliant, human-friendly.",
            "After force release: α decays back to 0, K snaps to 300 N/m, arm returns to goal.",
            "Best of both modes: compliant during contact, accurate in free motion.",
        ],
    ),
    "DI-MPC 100Hz": (
        "MPC @ 100 Hz — predictive rejection, lower SS error than stiff impedance",
        [
            "Two-layer: feedforward cancels nominal dynamics (500 Hz) + QP outer loop (100 Hz).",
            "QP state: x_e = [e, ė]  (4-vector, xz only).  Horizon N=10 steps × 10 ms = 0.1 s.",
            "LPV scheduling: B(ρ) updated from current Λ⁻¹(q) each MPC step.",
            "No Kalman estimator: reacts to accumulated error only — SS deflection ~ 15–20 mm.",
        ],
    ),
    "DI-MPC + Kalman 100Hz": (
        "MPC @ 100 Hz + Kalman disturbance estimator — drives SS error → 0",
        [
            "Augmented state:  x_aug = [ e(k);  ė(k);  d̂(k) ]   where d̂ ∈ ℝ³ is the force-form input-channel disturbance.",
            "Kalman update:  d̂(k+1) = d̂(k) + K_f · (y(k) − C · x_aug(k|k−1))",
            "Once d̂ converges, the centered QP adds F_mpc ≈ −d̂, cancelling the matched constant load.",
            "Steady-state error → 0 for constant matched disturbances via the augmented model.",
            "Convergence: ~1 QP interval (10 ms) after push onset. Each waypoint gets its own reset.",
        ],
    ),
    "DI-MPC 500Hz": (
        "MPC @ 500 Hz (every step) — fastest correction, lower peak deflection",
        [
            "Same QP structure as 100 Hz variant — only the solve rate changes: every 2 ms physics step.",
            "Zero-order-hold duration drops from 10 ms → 2 ms; torque refreshed 5× more often.",
            "Peak deflection falls ~3× vs 100 Hz (faster correction at force onset).",
            "No Kalman: SS error still ~ 5–10 mm. Rate governs transient; estimator governs steady state.",
        ],
    ),
    "DI-MPC + Kalman 500Hz": (
        "MPC @ 500 Hz + Kalman — fastest transient AND near-zero SS error",
        [
            "Combines highest QP rate (500 Hz) with Kalman disturbance estimation.",
            "Two independent improvements: rate → peak deflection, Kalman → SS error.",
            "Kalman drives d̂ to the matched input-channel disturbance within 1 step (2 ms) — SS error < 1 mm.",
            "500 Hz QP corrects within 1 physics step of force onset — minimum peak deflection.",
            "Performance ceiling of this architecture.  Both axes optimised simultaneously.",
        ],
    ),
}

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


def _make_camera() -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth   = 145.0
    cam.elevation = -20.0
    cam.distance  =  1.75
    cam.lookat[:] = [0.48, 0.0, 0.42]
    return cam


# ---------------------------------------------------------------------------
# Geometry helpers  (SIGBUS-safe: mjv_initGeom before mjv_connector)
# ---------------------------------------------------------------------------

_ZERO3 = np.zeros(3, dtype=np.float64)
_EYE9  = np.eye(3, dtype=np.float64).flatten()
_RGBA0 = np.zeros(4, dtype=np.float32)


def _add_capsule(scn, a, b, rgba, width: float = 0.003) -> None:
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE,
                        _ZERO3, _ZERO3, _EYE9, _RGBA0)
    mujoco.mjv_connector(scn.geoms[n], mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(a, dtype=np.float64),
                         np.asarray(b, dtype=np.float64))
    scn.geoms[n].rgba[:] = rgba
    scn.ngeom += 1


def _add_sphere_v(scn, pos, rgba, radius: float = 0.02) -> None:
    n = scn.ngeom
    if n >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(scn.geoms[n], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, radius, radius], np.float64),
                        np.asarray(pos, np.float64),
                        _EYE9, np.array(rgba, np.float32))
    scn.ngeom += 1


def _draw_guidance_scene(scn, wp_idx: int, dwell_frac: float,
                          ee_trail: deque, alpha_trail: deque,
                          ctrl_rgb_hex: str,
                          push_active: bool, ee_pos: np.ndarray,
                          wrench: np.ndarray) -> None:
    """Draw triangle path, waypoints, dwell arc, EE trail, push indicator."""
    ctrl_rgb = _hex_to_rgb_float(ctrl_rgb_hex)
    current_wpk = wp_idx % len(WAYPOINTS)
    p_d = WAYPOINTS[current_wpk]

    # Planned triangle path (white, faint)
    for k in range(len(WAYPOINTS)):
        _add_capsule(scn, WAYPOINTS[k], WAYPOINTS[(k + 1) % len(WAYPOINTS)],
                     [0.9, 0.9, 0.9, 0.35], width=0.005)

    # Waypoint spheres (gold; current larger & brighter)
    for k, wp in enumerate(WAYPOINTS):
        is_cur = (k == current_wpk)
        _add_sphere_v(scn, wp,
                    [1.0, 0.85, 0.1, 0.95 if is_cur else 0.45],
                    radius=0.038 if is_cur else 0.022)

    # Dwell progress arc (green ring around current waypoint)
    if dwell_frac > 0.01:
        n_arc = max(2, int(dwell_frac * 32))
        theta = np.linspace(0, 2 * np.pi * dwell_frac, n_arc + 1)
        arc_r = 0.065
        arc = np.column_stack([
            p_d[0] + arc_r * np.cos(theta),
            np.full(n_arc + 1, p_d[1]),
            p_d[2] + arc_r * np.sin(theta),
        ])
        for k in range(n_arc):
            _add_capsule(scn, arc[k], arc[k + 1],
                         [0.2, 1.0, 0.2, 0.90], width=0.005)

    # EE trail — blends controller colour → orange as compliance rises
    pts  = list(ee_trail)
    al_t = list(alpha_trail)
    r, g, b = ctrl_rgb
    skip = 2
    for k in range(0, len(pts) - 1, skip):
        a_k = al_t[k]
        col = [r * (1 - a_k) + 1.0 * a_k,
               g * (1 - a_k) + 0.5 * a_k,
               b * (1 - a_k) + 0.0 * a_k,
               0.85]
        _add_capsule(scn, pts[k], pts[k + 1], col, width=0.005)

    # Error line EE → waypoint when large
    err = float(np.linalg.norm(ee_pos - p_d))
    if err > 0.04:
        _add_capsule(scn, ee_pos, p_d, [1.0, 0.15, 0.1, 0.65], width=0.003)

    # Push indicator: red sphere (human hand) + arrow to EE
    if push_active:
        f = wrench[:3]
        f_norm = float(np.linalg.norm(f))
        if f_norm > 0:
            hand_pos = ee_pos + 0.16 * f / f_norm
            _add_sphere_v(scn, hand_pos, [1.0, 0.1, 0.1, 0.90], radius=0.028)
            _add_capsule(scn, hand_pos, ee_pos, [1.0, 0.2, 0.2, 0.70], width=0.006)


# ---------------------------------------------------------------------------
# Real-time error-plot inset
# ---------------------------------------------------------------------------

PLOT_W, PLOT_H, PLOT_DPI = 390, 200, 100


def _create_error_fig(ctrl_name: str, duration: float,
                       ctrl_color_hex: str) -> tuple:
    fig, ax = plt.subplots(figsize=(PLOT_W / PLOT_DPI, PLOT_H / PLOT_DPI),
                            dpi=PLOT_DPI)
    bg = (0.06, 0.06, 0.10)
    ax.set_facecolor(bg)
    fig.patch.set_facecolor(bg)
    (line,) = ax.plot([], [], lw=1.4, color=ctrl_color_hex, zorder=3)
    cursor  = ax.axvline(x=0.0, color='#FFFF80', lw=0.9, alpha=0.85, zorder=4)
    ax.set_xlim(0.0, duration)
    ax.set_ylim(0.0, 20.0)
    ax.set_xlabel('t  (s)', color='#999999', fontsize=6.5, labelpad=1)
    ax.set_ylabel('|e| to waypoint  (mm)', color='#999999', fontsize=6.5, labelpad=1)
    ax.set_title('Waypoint tracking error', color='#cccccc', fontsize=7.5, pad=3)
    ax.tick_params(colors='#888888', labelsize=5.5, length=2, pad=1)
    for sp in ax.spines.values():
        sp.set_edgecolor('#333333')
    ax.grid(True, color='#1e1e2e', lw=0.6, zorder=1)
    fig.tight_layout(pad=0.55)
    fig.canvas.draw()
    return fig, ax, line, cursor


def _render_error_inset(fig, ax, line, cursor,
                         t_arr, err_arr, t_now: float,
                         y_max_state: list,
                         push_spans: list) -> np.ndarray:
    """Update plot; push_spans is list of (t_on, t_off) already added."""
    line.set_data(t_arr, err_arr)
    cursor.set_xdata([t_now, t_now])
    cur_max = float(err_arr.max()) if len(err_arr) > 0 else 0.0
    if cur_max * 1.25 > y_max_state[0]:
        y_max_state[0] = max(cur_max * 1.25, 10.0)
        ax.set_ylim(0.0, y_max_state[0])
    fig.canvas.draw()
    buf  = fig.canvas.buffer_rgba()
    rgba = np.frombuffer(buf, dtype=np.uint8).reshape(PLOT_H, PLOT_W, 4)
    return rgba[:, :, :3].copy()


# ---------------------------------------------------------------------------
# Font helpers
# ---------------------------------------------------------------------------

def _load_font(size: int):
    if not _PIL_OK:
        return None
    for path in ["/System/Library/Fonts/Helvetica.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


_FONT_LG = None
_FONT_MD = None
_FONT_SM = None


def _init_fonts():
    global _FONT_LG, _FONT_MD, _FONT_SM
    if _PIL_OK and _FONT_LG is None:
        _FONT_LG = _load_font(26)
        _FONT_MD = _load_font(20)
        _FONT_SM = _load_font(15)


def _wrap(text: str, max_chars: int = 88) -> list[str]:
    words, lines, cur = text.split(), [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > max_chars:
            lines.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).lstrip()
    if cur:
        lines.append(cur)
    return lines


# ---------------------------------------------------------------------------
# Overlay helpers
# ---------------------------------------------------------------------------

def _overlay_text(frame: np.ndarray, ctrl_name: str, t: float,
                  err_m: float, wp_name: str,
                  wp_reached: int, max_wp: int,
                  push_active: bool, alpha: float,
                  ctrl_rgb_int: tuple,
                  plot_inset: np.ndarray | None = None) -> np.ndarray:
    if not _PIL_OK:
        return frame
    img  = Image.fromarray(frame)

    if plot_inset is not None:
        ph, pw = plot_inset.shape[:2]
        ix = frame.shape[1] - pw - 18
        iy = 18
        ImageDraw.Draw(img).rectangle(
            [ix - 3, iy - 3, ix + pw + 3, iy + ph + 3],
            outline=(80, 80, 80), width=1)
        img.paste(Image.fromarray(plot_inset), (ix, iy))

    draw = ImageDraw.Draw(img)
    x0, y, lh = 22, 20, 32
    r, g, b = ctrl_rgb_int

    draw.rectangle([x0 - 6, y - 6, x0 + 560, y + lh - 2],
                   fill=(r // 4, g // 4, b // 4, 200))
    draw.text((x0, y), ctrl_name, font=_FONT_LG, fill=(r, g, b))
    y += lh + 2

    short_desc, _ = DESCRIPTIONS.get(ctrl_name, ("", []))
    if short_desc:
        draw.text((x0, y), short_desc, font=_FONT_SM, fill=(180, 180, 100))
        y += 26
    y += 4

    draw.text((x0, y), f"t = {t:5.2f} s", font=_FONT_MD, fill=(220, 220, 220))
    y += lh - 4

    err_mm = err_m * 1e3
    ecol = (255, 80, 80) if err_mm > 40 else (255, 200, 60) if err_mm > 15 else (120, 255, 120)
    draw.text((x0, y), f"|e| = {err_mm:6.1f} mm  →  WP {wp_name}",
              font=_FONT_MD, fill=ecol)
    y += lh - 4

    if push_active:
        draw.text((x0, y), "●  Human push  ON  (15 N)",
                  font=_FONT_MD, fill=(255, 90, 90))
    else:
        draw.text((x0, y), "○  Free motion",
                  font=_FONT_MD, fill=(160, 160, 160))
    y += lh - 4

    if alpha > 0.01:
        draw.text((x0, y), f"α = {alpha:.2f}  (compliance active)",
                  font=_FONT_SM, fill=(255, 165, 0))

    counter = f"Waypoints  {wp_reached} / {max_wp}"
    draw.text((frame.shape[1] - 210, 18 + PLOT_H + 12),
              counter, font=_FONT_SM, fill=(180, 180, 180))

    return np.array(img)


def _make_opening_card(renderer, cam, data, fps: int,
                        duration: float = 4.0) -> list:
    """Scenario-overview card shown before the first controller."""
    renderer.update_scene(data, camera=cam)
    bg = (renderer.render().copy() * 0.20).astype(np.uint8)
    if not _PIL_OK:
        return [bg] * int(fps * duration)

    img  = Image.fromarray(bg)
    draw = ImageDraw.Draw(img)
    W, H = img.size

    x0, y = 60, 45
    draw.text((x0, y), "FR3 pHRI  —  Guidance Scenario",
              font=_FONT_LG, fill=(255, 220, 60))
    y += 50
    draw.line([(x0, y), (W - x0, y)], fill=(180, 150, 0, 160), width=1)
    y += 16

    lines = [
        "Task:   Reach-and-hold at 3 waypoints in a triangle (A → B → C → A)",
        "Push:   15 N force fires 0.8 s after the robot enters each waypoint radius",
        "        Waypoint A: −z  |  B: +y  |  C: +z",
        "Advance: robot must hold in radius for 1.0 s continuously after push ends",
        "Laps:   2 laps per controller  (6 push events total)",
        "",
        "Expected results:",
        "  Stiff Impedance         — resists push, ~50 mm steady-state deflection",
        "  Pure Admittance         — large compliant yield (~150 mm), slow recovery",
        "  Variable Compliance     — yields during contact, snaps back to goal after release",
        "  DI-MPC (100 Hz)  — predictive rejection, lower SS error",
        "  Imp. MPC + Kalman 100Hz — Kalman drives d̂ → F_h, near-zero SS error",
        "  DI-MPC 500 Hz    — fastest correction, lower peak deflection",
        "  Imp. MPC + Kalman 500Hz — best overall: fast transient AND zero SS error",
    ]
    for ln in lines:
        if ln == "":
            y += 8
            continue
        draw.text((x0, y), ln, font=_FONT_SM, fill=(210, 210, 200))
        y += 23

    card = np.array(img)
    return [card.copy() for _ in range(int(fps * duration))]


def _intro_card(renderer, cam, data, ctrl_name: str,
                ctrl_rgb_int: tuple, fps: int, duration: float) -> list:
    renderer.update_scene(data, camera=cam)
    bg = (renderer.render().copy() * 0.25).astype(np.uint8)
    if not _PIL_OK:
        return [bg.copy() for _ in range(int(fps * duration))]

    img  = Image.fromarray(bg)
    draw = ImageDraw.Draw(img)
    W, H = img.size
    r, g, b = ctrl_rgb_int
    x0, y = 60, 60

    draw.text((x0, y), ctrl_name, font=_FONT_LG, fill=(r, g, b))
    y += 44
    draw.line([(x0, y), (W - x0, y)], fill=(r // 2, g // 2, b // 2, 180), width=1)
    y += 14

    _, detail_lines = DESCRIPTIONS.get(ctrl_name, ("", []))
    for dl in detail_lines:
        for wrapped in _wrap(dl, max_chars=90):
            draw.text((x0, y), wrapped, font=_FONT_SM, fill=(220, 220, 200))
            y += 26
        y += 4

    if "Kalman" in ctrl_name:
        y += 10
        draw.line([(x0, y), (W - x0, y)], fill=(80, 200, 80, 160), width=1)
        y += 12
        draw.text((x0, y), "Kalman Disturbance Estimator — how it achieves zero SS error",
                  font=_FONT_MD, fill=(100, 240, 100))
        y += 34
        for ln in [
            "Augmented state:  x_aug = [ e(k);  ė(k);  d̂(k) ]   (d̂ ∈ ℝ³ estimates F_human)",
            "Prediction:  x_aug(k+1) = A_aug·x_aug(k) + B_aug·u(k)",
            "  A_aug has an identity block for d̂ — assumes disturbance is constant across horizon.",
            "",
            "Kalman measurement update (every QP interval):",
            "  innovation  v(k) = y(k) − C_aug·x_aug(k|k−1)      [y = measured (e, ė)]",
            "  d̂(k|k)     = d̂(k|k−1) + K_f · v(k)              [K_f = Kalman gain, 3×6]",
            "",
            "Interpretation: if the arm is pushed, measured error grows beyond prediction.",
            "  K_f maps residual into d̂, which ramps toward F_h in ~1 QP interval.",
            "Once d̂ ≈ F_h, QP sets F_mpc = −d̂ → cancels external load → SS error = 0.",
        ]:
            if ln == "":
                y += 8
                continue
            draw.text((x0, y), ln, font=_FONT_SM, fill=(190, 230, 190))
            y += 24

    card = np.array(img)
    return [card.copy() for _ in range(int(fps * duration))]


def _transition_card(renderer, cam, data, next_name: str,
                      next_rgb_int: tuple, fps: int, duration: float) -> list:
    renderer.update_scene(data, camera=cam)
    bg = (renderer.render().copy() * 0.35).astype(np.uint8)
    if not _PIL_OK:
        return [bg.copy() for _ in range(int(fps * duration))]

    img  = Image.fromarray(bg)
    draw = ImageDraw.Draw(img)
    W, H = img.size
    r, g, b = next_rgb_int
    cx  = W // 2
    short_desc, _ = DESCRIPTIONS.get(next_name, ("", []))

    draw.text((cx - 90, H // 2 - 48), "Up next", font=_FONT_SM, fill=(160, 160, 160))
    try:
        tw = _FONT_LG.getbbox(next_name)[2]
    except Exception:
        tw = len(next_name) * 16
    draw.text((cx - tw // 2, H // 2 - 18), next_name, font=_FONT_LG, fill=(r, g, b))
    if short_desc:
        try:
            tw2 = _FONT_SM.getbbox(short_desc)[2]
        except Exception:
            tw2 = len(short_desc) * 10
        draw.text((cx - tw2 // 2, H // 2 + 28), short_desc,
                  font=_FONT_SM, fill=(180, 180, 120))

    card = np.array(img)
    return [card.copy() for _ in range(int(fps * duration))]


# ---------------------------------------------------------------------------
# Comparison table
# ---------------------------------------------------------------------------

def _generate_comparison_table(all_metrics: dict, fps: int,
                                 W: int, H: int,
                                 duration: float = 8.0) -> list:
    fig, ax = plt.subplots(figsize=(W / 100, H / 100), dpi=100)
    bg = (0.06, 0.07, 0.12)
    fig.patch.set_facecolor(bg)
    ax.set_facecolor(bg)
    ax.axis('off')

    ctrl_names = list(all_metrics.keys())
    cols  = ["RMS free\n(mm)", "RMS contact\n(mm)", "Peak defl.\n(mm)",
             "WPs reached"]
    rows  = []
    raw   = {c: [] for c in cols}

    for name in ctrl_names:
        m = all_metrics[name]
        rms_f = m["rms_free"]   * 1e3
        rms_c = m["rms_contact"] * 1e3
        peak  = m["peak_defl"]   * 1e3
        nwp   = m["waypoints_reached"]
        rows.append([f"{rms_f:.1f}", f"{rms_c:.1f}", f"{peak:.1f}",
                     f"{nwp}/{VIDEO_N_LAPS*len(WAYPOINTS)}"])
        raw[cols[0]].append(rms_f)
        raw[cols[1]].append(rms_c)
        raw[cols[2]].append(peak)
        raw[cols[3]].append(nwp)

    # Cell colours: green (best) → red (worst) per numeric column
    n_ctrl = len(ctrl_names)
    cell_colors = [["#1a1a2e"] * len(cols) for _ in range(n_ctrl)]
    for ci, col in enumerate(cols[:3]):
        vals = raw[col]
        lo, hi = min(vals), max(vals)
        for ri, v in enumerate(vals):
            t = 0.0 if hi == lo else (v - lo) / (hi - lo)
            gr = int(180 * (1 - t))
            rd = int(180 * t)
            cell_colors[ri][ci] = f"#{rd:02x}{gr:02x}28"
    # WPs reached: more is better
    for ri, nwp in enumerate(raw[cols[3]]):
        max_wp = VIDEO_N_LAPS * len(WAYPOINTS)
        t = nwp / max_wp
        gr = int(180 * t)
        rd = int(180 * (1 - t))
        cell_colors[ri][3] = f"#{rd:02x}{gr:02x}28"

    row_labels = [f"  {n}  " for n in ctrl_names]
    row_colors = [[COLORS_HEX.get(n, "#888888")] for n in ctrl_names]

    tbl = ax.table(
        cellText=rows,
        rowLabels=row_labels,
        colLabels=cols,
        cellLoc='center',
        rowLoc='right',
        loc='center',
        cellColours=cell_colors,
        rowColours=[c[0] for c in row_colors],
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(11)
    tbl.scale(1.0, 2.2)

    # Style header & row labels
    for (ri, ci), cell in tbl.get_celld().items():
        cell.set_edgecolor('#333355')
        cell.set_linewidth(0.5)
        if ri == 0:
            cell.set_text_props(color='#dddddd', fontweight='bold')
            cell.set_facecolor('#0d1133')
        elif ci == -1:
            name = ctrl_names[ri - 1]
            rgb  = COLORS_HEX.get(name, "#888888").lstrip("#")
            cell.set_text_props(
                color=f"#{rgb}",
                fontweight='bold',
                ha='right',
            )
            cell.set_facecolor('#0a0a1a')

    ax.set_title("Guidance Scenario — Controller Comparison",
                 color='#eeeeee', fontsize=14, fontweight='bold', pad=18)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.canvas.draw()
    buf  = fig.canvas.buffer_rgba()
    rgba = np.frombuffer(buf, dtype=np.uint8).reshape(H, W, 4)
    frame = rgba[:, :, :3].copy()
    plt.close(fig)
    return [frame.copy() for _ in range(int(fps * duration))]


# ---------------------------------------------------------------------------
# Single-controller episode with video capture
# ---------------------------------------------------------------------------

def run_episode_video(ctrl_name: str, env: FR3MuJoCoEnv,
                       renderer: mujoco.Renderer, cam: mujoco.MjvCamera,
                       fps: int, ep_idx: int, n_ep: int,
                       intro_duration: float = 3.0,
                       dt_mpc_slow: float = MPC_DT_SLOW) -> tuple[list, dict]:
    env.reset()

    ctrl_rgb_int   = COLORS_INT.get(ctrl_name, (128, 128, 128))
    ctrl_rgb_float = COLORS_FLOAT.get(ctrl_name, [0.5, 0.5, 0.5])
    ctrl_rgb_hex   = COLORS_HEX.get(ctrl_name, "#888888")

    frames = _intro_card(renderer, cam, env.data, ctrl_name,
                          ctrl_rgb_int, fps, intro_duration)

    dt_sim   = env.dt
    n_steps  = int(VIDEO_EPISODE_DURATION / dt_sim)
    render_every = max(1, int(round(1.0 / (fps * dt_sim))))
    max_wp   = VIDEO_N_LAPS * len(WAYPOINTS)

    # ── Controller initialisation ──────────────────────────────────────────
    R_d       = np.eye(3)
    imp_base  = make_impedance_params(k_pos=K_STIFF, k_rot=20.0,
                                      damping_ratio=1.0, q_null=Q_NEUTRAL)
    adm_ctrl  = None
    mpc_ctrl  = None
    tau_cached   = np.zeros(7)
    F_mpc_cached = np.zeros(3)
    mpc_every  = 1

    if ctrl_name == "Pure Admittance":
        adm_ctrl = AdmittanceController(
            make_admittance_params(m_pos=0.5, d_pos=15.0, k_pos=100.0),
            dt=dt_sim,
        )
    elif ctrl_name in MPC_NAMES:
        mpc_ctrl, mpc_every = make_mpc_controller(
            ctrl_name, dt_sim, dt_mpc_slow)

    # ── Waypoint / push state ──────────────────────────────────────────────
    wp_idx            = 0
    waypoints_reached = 0
    entry_time        = [None] * len(WAYPOINTS)
    dwell_timer       = 0.0
    alpha             = 0.0

    TRAIL_LEN  = 1000
    ee_trail   = deque(maxlen=TRAIL_LEN)
    alpha_trail = deque(maxlen=TRAIL_LEN)

    # ── Logging ───────────────────────────────────────────────────────────
    t_buf    = np.zeros(n_steps)
    err_buf  = np.zeros(n_steps)
    push_log = np.zeros(n_steps, dtype=bool)

    err_fig, err_ax, err_line, err_cursor = _create_error_fig(
        ctrl_name, VIDEO_EPISODE_DURATION, ctrl_rgb_hex)
    y_max_state  = [20.0]
    push_spans   = []   # list of (t_on, t_off) for shading (informational)

    print(f"  [{ctrl_name}] rendering {n_steps} steps …")

    for i in range(n_steps):
        t    = env.time
        wpk  = wp_idx % len(WAYPOINTS)
        p_d  = WAYPOINTS[wpk]

        dyn, state = env.get_dynamics_and_state()
        dist = float(np.linalg.norm(state.ee_pos - p_d))

        # Record first entry into waypoint radius
        if dist < WAYPOINT_RADIUS and entry_time[wpk] is None:
            entry_time[wpk] = t

        # Push window
        t_entry    = entry_time[wpk]
        t_push_on  = (t_entry + PUSH_DELAY)                  if t_entry is not None else np.inf
        t_push_off = (t_entry + PUSH_DELAY + PUSH_DURATION)  if t_entry is not None else np.inf
        push_active = (t_push_on <= t <= t_push_off) and (waypoints_reached < max_wp)

        wrench = np.zeros(6)
        if push_active:
            wrench[:3] = WAYPOINT_PUSH_FORCES[wpk]
            env.apply_ee_wrench(wrench)

        # Dwell counter (only after push ends)
        if dist < WAYPOINT_RADIUS and t > t_push_off:
            dwell_timer += dt_sim
        else:
            dwell_timer = 0.0

        # Advance waypoint
        if dwell_timer >= HOLD_AFTER and waypoints_reached < max_wp:
            dwell_timer       = 0.0
            entry_time[wpk]   = None
            wp_idx            += 1
            waypoints_reached += 1
            p_d = WAYPOINTS[wp_idx % len(WAYPOINTS)]
            if mpc_ctrl is not None:
                mpc_ctrl.reset()

        # Compliance alpha
        f_mag = float(np.linalg.norm(wrench[:3]))
        if ctrl_name == "Stiff Impedance":
            target_alpha = 0.0
        elif ctrl_name == "Pure Admittance":
            target_alpha = 1.0
        else:
            target_alpha = 1.0 if f_mag > F_CONTACT_THRESH else 0.0
        alpha += (target_alpha - alpha) * dt_sim / ALPHA_TC
        alpha  = float(np.clip(alpha, 0.0, 1.0))

        # ── Torque ────────────────────────────────────────────────────────
        dx_d_6d = ddx_d_6d = np.zeros(6)

        if ctrl_name == "Stiff Impedance":
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, imp_base)
            tau += env.null_space_gravity_comp(dyn)

        elif ctrl_name == "Pure Admittance":
            x_r, v_r = adm_ctrl.step(wrench)
            tau = cartesian_impedance_control(
                state, dyn, p_d + x_r[:3], R_d,
                np.concatenate([v_r[:3], np.zeros(3)]), ddx_d_6d, imp_base)
            tau += env.null_space_gravity_comp(dyn)

        elif ctrl_name == "Variable Compliance":
            k_eff = K_STIFF * (1 - alpha) + K_SOFT * alpha
            imp_vc = make_impedance_params(k_pos=k_eff, k_rot=20.0,
                                           damping_ratio=1.0, q_null=Q_NEUTRAL)
            tau = cartesian_impedance_control(
                state, dyn, p_d, R_d, dx_d_6d, ddx_d_6d, imp_vc)
            tau += env.null_space_gravity_comp(dyn)

        else:  # MPC variants
            if i % mpc_every == 0:
                tau_cached, F_mpc_cached = mpc_ctrl.control(
                    state.ee_pos, state.ee_vel, state.ee_rot,
                    p_d, np.zeros(3), np.zeros(3), R_d,
                    dyn, state.q, state.dq,
                )
                tau = tau_cached
            else:
                # 1 kHz inner loop: feedforward, orientation, and
                # null-space torques are recomputed every tick from
                # fresh (q, dq); only the QP correction F_mpc is held
                # from the last solve.
                J_v, J_w = dyn.J[:3, :], dyn.J[3:, :]
                tau_ff = dyn.Cq_dot  # static hold reference: ddp_d = 0
                e_R    = rotation_error_matrix(R_d, state.ee_rot)
                p_mpc  = mpc_ctrl.p
                tau_or = J_w.T @ (-p_mpc.K_rot * e_R - p_mpc.D_rot * state.ee_vel[3:])
                N_bar  = build_operational_space_model(dyn, state.ee_vel).N_bar
                tau    = (tau_ff + J_v.T @ F_mpc_cached + tau_or
                          + mpc_ctrl.null_torque(state.q, state.dq, N_bar))

        env.apply_torque(tau)
        env.step()

        err = float(np.linalg.norm(state.ee_pos - p_d))
        t_buf[i]    = t
        err_buf[i]  = err * 1e3   # mm
        push_log[i] = push_active

        ee_trail.append(state.ee_pos.copy())
        alpha_trail.append(alpha)

        if i % render_every != 0:
            continue

        # ── Render frame ──────────────────────────────────────────────────
        renderer.update_scene(env.data, camera=cam)
        try:
            scn = renderer.scene
            _draw_guidance_scene(
                scn, wp_idx, min(1.0, dwell_timer / HOLD_AFTER),
                ee_trail, alpha_trail, ctrl_rgb_hex,
                push_active, state.ee_pos, wrench,
            )
        except AttributeError:
            pass
        raw = renderer.render().copy()

        plot_inset = _render_error_inset(
            err_fig, err_ax, err_line, err_cursor,
            t_buf[:i + 1], err_buf[:i + 1], t, y_max_state, push_spans)

        frame = _overlay_text(
            raw, ctrl_name, t, err,
            WAYPOINT_NAMES[wp_idx % len(WAYPOINTS)],
            waypoints_reached, max_wp,
            push_active, alpha,
            ctrl_rgb_int, plot_inset,
        )
        frames.append(frame)

    plt.close(err_fig)

    # ── Metrics ───────────────────────────────────────────────────────────
    err_m = err_buf / 1e3   # back to metres
    contact_mask = push_log  # exact push windows logged during simulation

    rms_free    = float(np.sqrt(np.mean(err_m[~contact_mask]**2))) if (~contact_mask).any() else float('nan')
    rms_contact = float(np.sqrt(np.mean(err_m[contact_mask]**2)))  if contact_mask.any()   else float('nan')
    peak_defl   = float(np.max(err_m[contact_mask])) if contact_mask.any() else float(np.max(err_m))

    print(f"      WP reached={waypoints_reached}/{max_wp}"
          f"  RMS free={rms_free*1e3:.1f} mm"
          f"  RMS contact={rms_contact*1e3:.1f} mm"
          f"  Peak={peak_defl*1e3:.1f} mm")

    metrics = dict(rms_free=rms_free, rms_contact=rms_contact,
                   peak_defl=peak_defl, waypoints_reached=waypoints_reached)
    return frames, metrics


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main_video():
    parser = argparse.ArgumentParser(description="FR3 pHRI Guidance Scenario Video")
    parser.add_argument("--fps",    type=int, default=30)
    parser.add_argument("--output", default="simulation/guidance_video.mp4")
    parser.add_argument("--controllers", nargs="+", default=None,
                        help="subset of controllers to render; default is D1 and D7")
    args = parser.parse_args()

    _init_fonts()

    output_path = Path(__file__).parent / args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)

    W, H = 1280, 720
    env  = FR3MuJoCoEnv()
    renderer = mujoco.Renderer(env.model, height=H, width=W)
    cam  = _make_camera()

    ctrl_list = args.controllers if args.controllers else VIDEO_CONTROLLERS

    print("=" * 64)
    print("FR3 pHRI Guidance Scenario Video")
    print(f"  Controllers : {len(ctrl_list)}")
    print(f"  Laps        : {VIDEO_N_LAPS}  ({VIDEO_EPISODE_DURATION:.0f} s per controller)")
    print(f"  Resolution  : {W}×{H} @ {args.fps} fps")
    print(f"  Output      : {output_path}")
    print("=" * 64)

    all_frames  = []
    all_metrics = {}

    # Opening scenario card
    env.reset()
    all_frames += _make_opening_card(renderer, cam, env.data,
                                      args.fps, duration=5.0)

    n_ep = len(ctrl_list)
    for ep_idx, ctrl_name in enumerate(ctrl_list, 1):
        ep_frames, metrics = run_episode_video(
            ctrl_name, env, renderer, cam,
            args.fps, ep_idx, n_ep,
            intro_duration=3.0,
        )
        all_frames  += ep_frames
        all_metrics[ctrl_name] = metrics

        # Transition card (not after last)
        if ep_idx < n_ep:
            next_name = ctrl_list[ep_idx]
            all_frames += _transition_card(
                renderer, cam, env.data, next_name,
                COLORS_INT.get(next_name, (128, 128, 128)),
                args.fps, duration=2.0,
            )

    # Comparison table
    print("  Generating comparison table …")
    all_frames += _generate_comparison_table(all_metrics, args.fps, W, H, duration=8.0)

    renderer.close()

    total_s = len(all_frames) / args.fps
    print(f"\n  Total frames : {len(all_frames)}")
    print(f"  Video length : {total_s:.1f} s  ({total_s/60:.1f} min)")
    print(f"  Saving → {output_path} …")

    if _IMAGEIO_OK:
        imageio.mimwrite(str(output_path), all_frames,
                         fps=args.fps, quality=8)
    else:
        import cv2
        out = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            args.fps, (W, H),
        )
        for f in all_frames:
            out.write(f[:, :, ::-1])
        out.release()

    print(f"  Done. Saved {output_path}")



#==========================================================================
# Unified CLI:  python3 guidance.py {demo|compare|video} [options]
#==========================================================================
if __name__ == "__main__":
    import sys as _sys
    _modes = {"demo": main_demo, "compare": main_compare, "video": main_video}
    if len(_sys.argv) < 2 or _sys.argv[1] not in _modes:
        print("usage: python3 guidance.py {demo|compare|video} [options]")
        _sys.exit(1)
    _modes[_sys.argv.pop(1)]()
