"""
FR3 MuJoCo simulation demo runner.

Usage (run from fr3_impedance/ directory):
    python3 simulation/run_simulation.py --mode impedance
    python3 simulation/run_simulation.py --mode phri
    python3 simulation/run_simulation.py --mode rl_random
    python3 simulation/run_simulation.py --mode data_check

    # headless (no MuJoCo 3-D window):
    python3 simulation/run_simulation.py --mode impedance --no-viewer

    # headless + no live plot (useful on servers):
    python3 simulation/run_simulation.py --mode impedance --no-viewer --no-plot

Modes
-----
impedance   : FR3 drives to a fixed Cartesian target under critically-damped
              impedance control.  Live dashboard shows convergence in real time.
phri        : pHRI demo: virtual human arm pushes the EE in +Y for 2 s; the
              admittance model lets the robot yield compliantly.
rl_random   : Random RL actions through PHRIEnv — exercises the full interface.
data_check  : Print a dynamics snapshot at t=0.  No viewer or plot.
"""

from __future__ import annotations
import sys
import argparse
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from fr3_impedance import (
    make_impedance_params, make_admittance_params,
    cartesian_impedance_control, AdmittanceController,
)
from fr3_mujoco  import FR3MuJoCoEnv, Q_NEUTRAL
from phri_env    import PHRIEnv, HumanArmParams, VirtualHumanArm
from visualizer  import SimVisualizer


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _try_viewer(model, data):
    """Return a MuJoCo passive viewer context, or None if unavailable."""
    try:
        import mujoco.viewer
        return mujoco.viewer.launch_passive(model, data)
    except Exception:
        print("[run] MuJoCo viewer not available — running headless.")
        return None


def _make_viz(mode: str, dt: float, x_d: np.ndarray, enabled: bool) -> SimVisualizer | None:
    if not enabled:
        return None
    try:
        return SimVisualizer(mode=mode, dt=dt, x_d=x_d)
    except Exception as exc:
        print(f"[run] Live dashboard unavailable ({exc}) — continuing without plot.")
        return None


# ---------------------------------------------------------------------------
# Mode 1: Impedance control
# ---------------------------------------------------------------------------

def run_impedance(args):
    """
    FR3 drives from the neutral configuration to x_d = [0.5, 0.0, 0.4] m
    under critically-damped Cartesian impedance (k_pos = 300 N/m).

    Dashboard panels
    ----------------
    • Position error |x − x_d| converging to < 1 cm
    • EE position components x, y, z with target dashed lines
    • External force (zero — no contact in this mode)
    • All 7 joint torques
    • EE speed
    • All 7 joint velocities
    """
    env    = FR3MuJoCoEnv()
    x_d    = np.array([0.5, 0.0, 0.4])
    R_d    = np.eye(3)
    dx_d   = np.zeros(6)
    ddx_d  = np.zeros(6)
    params = make_impedance_params(k_pos=300.0, k_rot=20.0, damping_ratio=1.0,
                                   q_null=Q_NEUTRAL)

    # print(f"[impedance] params: {params} m")

    env.reset()
    env.update_target_site(x_d)

    duration = 5.0
    n_steps  = int(duration / env.dt)
    #print(f"[impedance] Running for {duration:.1f} s with dt={env.dt*1000:.1f} ms (n_steps={n_steps})")

    viewer = None if args.no_viewer else _try_viewer(env.model, env.data)
    viz    = _make_viz("impedance", env.dt, x_d, not args.no_plot)

    print(f"[impedance] Running {duration:.1f} s  (dt={env.dt*1000:.1f} ms)…")
    if viz:
        print("[impedance] Live dashboard open.  Close the plot window to exit early.")

    def _run():
        for i in range(n_steps):
            dyn, state = env.get_dynamics_and_state()

            tau  = cartesian_impedance_control(state, dyn, x_d, R_d, dx_d, ddx_d, params)
            tau += env.null_space_gravity_comp(dyn)
            env.apply_torque(tau)
            env.step()

            err = float(np.linalg.norm(state.ee_pos - x_d))

            if viz is not None:
                viz.push(
                    env.time,
                    pos_err = err,
                    ee_pos  = state.ee_pos,
                    tau     = tau,
                    dq      = state.dq,
                    f_ext   = 0.0,
                    ee_vel  = float(np.linalg.norm(state.ee_vel[:3])),
                )
                if i % viz.update_every == 0:
                    viz.refresh()

            if i % 500 == 0:
                print(f"  t={env.time:.2f}s  |x−x_d|={err:.4f} m  EE={np.round(state.ee_pos, 3)}")

            if viewer is not None:
                viewer.sync()

    try:
        if viewer is not None:
            with viewer:
                _run()
        else:
            _run()
    finally:
        if viz is not None:
            viz.refresh()
            viz.save("impedance_dashboard.png")
            import matplotlib.pyplot as plt
            plt.show(block=True)
            viz.close()

    print(f"[impedance] Done.")


# ---------------------------------------------------------------------------
# Mode 2: pHRI with virtual human
# ---------------------------------------------------------------------------

def run_phri(args):
    """
    Physical human-robot interaction demo.

    A virtual spring-damper arm pushes the EE 10 cm in +Y from t=1–3 s.
    The admittance model (K_a=0) makes the robot comply; after the push it
    holds the displaced reference.

    Dashboard panels
    ----------------
    • Position error |x − x_d|
    • EE trajectory (x, y, z) — y rises during the push
    • Human force magnitude |f_human|
    • Joint torques
    • EE speed
    • CUSUM contact score with contact-active shading
    """
    env    = FR3MuJoCoEnv()
    adm    = AdmittanceController(
        make_admittance_params(m_pos=1.0, d_pos=80.0, k_pos=0.0), dt=env.dt
    )
    x_d    = np.array([0.45, 0.0, 0.45])
    R_d    = np.eye(3)
    params = make_impedance_params(k_pos=250.0, k_rot=20.0, damping_ratio=1.0,
                                   q_null=Q_NEUTRAL)
    human  = VirtualHumanArm(HumanArmParams(
        K_h=np.diag([80., 80., 80.]),
        D_h=np.diag([15., 15., 15.]),
    ))

    env.reset()
    env.update_target_site(x_d)

    duration = 6.0
    n_steps  = int(duration / env.dt)

    viewer = None if args.no_viewer else _try_viewer(env.model, env.data)
    viz    = _make_viz("phri", env.dt, x_d, not args.no_plot)

    # We track the CUSUM state manually for the dashboard
    from phri_env import CUSUMDetector
    cusum = CUSUMDetector()

    print(f"[phri] Running {duration:.1f} s  (dt={env.dt*1000:.1f} ms)…")
    print(f"       Human push: t=1–3 s, +Y direction, 10 cm offset")

    def _run():
        for i in range(n_steps):
            t = env.time
            dyn, state = env.get_dynamics_and_state()

            f_human_vec = np.zeros(6)
            if 1.0 < t < 3.0:
                anchor = x_d + np.array([0., 0.10, 0.])
                human.set_anchor(anchor)
                f_human_vec = human.compute_wrench(state.ee_pos, state.ee_vel[:3])
                env.apply_ee_wrench(f_human_vec)

            f_norm = float(np.linalg.norm(f_human_vec[:3]))
            cusum.update(f_norm)

            x_r, v_r = adm.step(f_human_vec)
            x_cmd    = x_d + x_r[:3]
            dx_cmd   = np.concatenate([v_r[:3], np.zeros(3)])
            state.f_ext[:] = f_human_vec

            tau  = cartesian_impedance_control(
                state, dyn, x_cmd, R_d, dx_cmd, np.zeros(6), params, use_f_ext=True,
            )
            tau += env.null_space_gravity_comp(dyn)
            env.apply_torque(tau)
            env.step()

            err = float(np.linalg.norm(state.ee_pos - x_d))

            if viz is not None:
                viz.push(
                    t,
                    pos_err = err,
                    ee_pos  = state.ee_pos,
                    tau     = tau,
                    dq      = state.dq,
                    f_ext   = f_norm,
                    ee_vel  = float(np.linalg.norm(state.ee_vel[:3])),
                    contact = cusum.contact,
                    cusum   = cusum.S,
                )
                if i % viz.update_every == 0:
                    viz.refresh()

            if i % 500 == 0:
                print(f"  t={t:.2f}s  |x−x_d|={err:.4f} m  |f_human|={f_norm:.1f} N")

            if viewer is not None:
                viewer.sync()

    try:
        if viewer is not None:
            with viewer:
                _run()
        else:
            _run()
    finally:
        if viz is not None:
            viz.refresh()
            viz.save("phri_dashboard.png")
            import matplotlib.pyplot as plt
            plt.show(block=True)
            viz.close()

    print(f"[phri] Done.")


# ---------------------------------------------------------------------------
# Mode 3: Random RL actions through PHRIEnv
# ---------------------------------------------------------------------------

def run_rl_random(args):
    """
    Exercises the full PHRIEnv RL interface with random actions.

    Dashboard panels
    ----------------
    • Position error per RL step (aggregated across episodes)
    • External force |f_ext|
    • Impedance parameters Kd, Dd, kn over time
    """
    rng = np.random.default_rng(42)
    env = PHRIEnv()

    obs = env.reset(x_d=np.array([0.45, 0.05, 0.45]),
                    randomize_human=True, rng=rng)

    print(f"[rl_random] obs_dim={env.obs_dim}  act_dim={env.act_dim}")
    print(f"            obs[0:6] (f_ext) = {obs[:6]}")

    viz = _make_viz("rl_random", env.dt * env.N, np.array([0.45, 0.05, 0.45]),
                    not args.no_plot)

    n_episodes, max_steps = 2, 50
    t_global = 0.0

    for ep in range(n_episodes):
        obs = env.reset(randomize_human=True, rng=rng)
        total_reward = 0.0

        for step in range(max_steps):
            action = rng.uniform(*env.act_bounds)
            obs, reward, done, info = env.step(action)
            total_reward += reward
            t_global += env.dt * env.N

            if viz is not None:
                viz.push(
                    t_global,
                    pos_err = info["pos_error_m"],
                    f_ext   = info["f_human_N"],
                    ee_vel  = info["ee_speed_m_s"],
                    contact = info["contact"],
                    params  = np.array([info["Kd"], info["Dd"], info["kn"]]),
                    reward  = reward,
                )
                viz.refresh()

            if done:
                break

        print(f"  episode {ep+1}: steps={step+1}  total_reward={total_reward:.3f}  "
              f"final_err={info['pos_error_m']:.4f} m  contact={info['contact']}")

    if viz is not None:
        viz.save("rl_random_dashboard.png")
        import matplotlib.pyplot as plt
        plt.show(block=True)
        viz.close()


# ---------------------------------------------------------------------------
# Mode 4: Dynamics snapshot (no viewer / plot)
# ---------------------------------------------------------------------------

def run_data_check(args):
    """Print a dynamics snapshot at t=0."""
    env = FR3MuJoCoEnv()
    env.reset()
    dyn, state = env.get_dynamics_and_state()

    print("[data_check] FR3 MuJoCo environment loaded successfully.")
    print(f"  nv          = {env.nv}")
    print(f"  dt          = {env.dt*1000:.1f} ms")
    print(f"  EE site id  = {env.ee_site_id}")
    print(f"  EE pos      = {state.ee_pos}")
    print(f"  q (neutral) = {np.round(state.q, 3)}")
    print(f"  M diag      = {np.round(np.diag(dyn.M), 4)}")
    print(f"  J shape     = {dyn.J.shape}  (6×{env.nv})")
    print(f"  bias[:7]    = {np.round(dyn.Cq_dot, 4)}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FR3 MuJoCo simulation runner")
    parser.add_argument(
        "--mode",
        choices=["impedance", "phri", "rl_random", "data_check"],
        default="data_check",
        help="Simulation mode (default: data_check)",
    )
    parser.add_argument(
        "--no-viewer", action="store_true",
        help="Disable the MuJoCo 3-D GUI viewer",
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable the live matplotlib dashboard",
    )
    args = parser.parse_args()

    modes = {
        "impedance":  run_impedance,
        "phri":       run_phri,
        "rl_random":  run_rl_random,
        "data_check": run_data_check,
    }
    modes[args.mode](args)
