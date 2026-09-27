#!/usr/bin/env python3
"""Torque-controlled task-space tracking on a 3-DOF (waist-removed)
OpenManipulator-X, driven by a finite-horizon LQR controller + Kalman
disturbance observer. See docs/01_concepts.md for the full derivation.

Control law (operational-space, Current Control Mode):
    x_ee = FK(q),  J = Jacobian(q),  ee_vel = J dq
    u      = -K0 (x_ee_error, ee_vel_error) - d_hat    # residual task-space acceleration
    d_hat  = KalmanObserver(x_ee_error, u)              # applied u fed back in
    F_task = Lambda(q) (ddp_d + u)                      # operational-space force
    tau    = J^T F_task + N tau_post + gravity_scale*g(q) + C(q,dq)  # joint torque
    Goal_Current = tau / K_t                             # per XM430 servo

`Lambda(q) = (J M(q)^-1 J^T)^-1` is the operational-space (task-space) mass
matrix -- it makes `u` behave like a genuine Cartesian acceleration command
regardless of the arm's current configuration. `N = I - J^T(Lambda J M^-1)`
is the dynamically-consistent null-space projector: this arm has 3 joints
but only a 2D (x-z) task, so one degree of freedom is redundant, and
`tau_post` (a soft P-D pull toward a fixed nominal posture) uses it without
disturbing the task-space motion.

SAFETY: start with the arm supported and a low current_limit_ticks; torque
is disabled on any exit (including Ctrl+C and exceptions). Use --backend sim
to validate a config before ever touching real hardware -- see
docs/03_hardware_safety.md before running --backend dynamixel for the
first time.

Usage:
  python run_hardware.py --backend sim --config configs/hold.yaml --duration 10
  python run_hardware.py --backend sim --config configs/push.yaml --duration 20 --output results/hardware/sim_push.csv
  python run_hardware.py --backend sim --config configs/circle.yaml --duration 60 --live-plot
  python run_hardware.py --backend dynamixel --port COM3 --baud 1000000 --config configs/hold.yaml --duration 20 --output results/hardware/hw_hold.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import yaml

if sys.platform == "win32":
    # Windows defaults to a ~15.6 ms system timer resolution, which makes
    # time.sleep() round up any short sleep to a multiple of that -- this alone
    # can cap the achievable loop rate well below 100 Hz (dt=0.01) regardless of
    # how fast the actual computation is. Raising the resolution to 1 ms fixes
    # this; it's restored on exit. No-op on non-Windows.
    import ctypes
    _winmm = ctypes.WinDLL("winmm")
    _winmm.timeBeginPeriod(1)
    import atexit
    atexit.register(lambda: _winmm.timeEndPeriod(1))

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "lib"))

from interaction_mpc import ControllerConfig, NormalizedInteractionMPC, RandomWalkDisturbanceObserver  # noqa: E402
from trajectory import CartesianTrajectory  # noqa: E402
from move_to_start import move_to_start  # noqa: E402
from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402
from dynamixel_backend import assert_startup_pose_plausible, create_backend  # noqa: E402


def load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def controller_config(config: dict) -> ControllerConfig:
    c = config.get("controller", {})
    return ControllerConfig(
        dt=float(c.get("dt", 0.01)), horizon=int(c.get("horizon", 25)),
        q_pos=float(c.get("q_pos", 60.0)), q_vel=float(c.get("q_vel", 12.0)),
        r=float(c.get("r", 0.05)), u_max=np.asarray(c.get("u_max", [4.0, 4.0, 4.0]), dtype=float),
        observer_q_d=float(c.get("observer_q_d", 0.02)), observer_r_y=float(c.get("observer_r_y", 0.0004)),
        observer_d_hat_max=float(c.get("observer_d_hat_max", 15.0)),
        qp_iters=int(c.get("qp_iters", 200)),
    )


def build_kinematics(config: dict) -> OpenManipulatorKinematics:
    k = config.get("kinematics", {})
    kin = OpenManipulatorKinematics()
    if "links" in k:
        kin.links.update(k["links"])
    if "link_masses" in k:
        kin.link_masses = list(k["link_masses"])
    return kin


def _init_live_plot(kin: OpenManipulatorKinematics, traj: CartesianTrajectory, traj_cfg: dict,
                     p0: np.ndarray) -> dict:
    """Interactive matplotlib window showing the arm (as connected links in
    the x-z plane), the end-effector path traced so far, and the target."""
    import matplotlib.pyplot as plt

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
    ax.set_title("3-DOF arm -- live view (x-z plane)")

    if traj_cfg.get("type") == "circle":
        period = float(traj_cfg.get("circle_period_s", 12.0))
        ts = np.linspace(0.0, period, 200)
        ref = np.array([traj.sample(tt)[0] for tt in ts])
        ax.plot(ref[:, 0], ref[:, 2], "--", color="0.6", lw=1, label="target circle")

    (arm_line,) = ax.plot([], [], "o-", color="tab:blue", lw=3, ms=6, label="arm")
    (trace_line,) = ax.plot([], [], "-", color="tab:orange", lw=1, alpha=0.8, label="ee path")
    (target_pt,) = ax.plot([], [], "x", color="tab:red", ms=9, mew=2, label="target")
    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9, family="monospace")
    ax.legend(loc="lower right", fontsize=8)

    reach = sum(np.linalg.norm(kin.links[k]) for k in ("d0e_base", "d01", "d12", "d1e"))
    ax.set_xlim(p0[0] - reach * 0.7, p0[0] + reach * 0.7)
    ax.set_ylim(p0[2] - reach * 0.7, p0[2] + reach * 0.7)

    fig.canvas.draw()
    plt.pause(0.001)
    return {"fig": fig, "arm": arm_line, "trace": trace_line, "target": target_pt,
            "txt": txt, "trace_x": [], "trace_z": []}


def _update_live_plot(live: dict, kin: OpenManipulatorKinematics, q: np.ndarray, p_d: np.ndarray,
                       t: float, err_mm: float, hz: float) -> None:
    import matplotlib.pyplot as plt

    pts = [np.zeros(3)] + kin.frames(q)  # base origin + [j1, j2, j3, ee]
    xs = [p[0] for p in pts]; zs = [p[2] for p in pts]
    live["arm"].set_data(xs, zs)
    live["trace_x"].append(xs[-1]); live["trace_z"].append(zs[-1])
    live["trace"].set_data(live["trace_x"], live["trace_z"])
    live["target"].set_data([p_d[0]], [p_d[2]])
    live["txt"].set_text(f"t={t:6.2f}s  err={err_mm:6.2f}mm  {hz:5.1f}Hz")
    live["fig"].canvas.draw_idle()
    live["fig"].canvas.flush_events()
    plt.pause(0.001)


def run(args: argparse.Namespace) -> Path | None:
    config = load_yaml(args.config)
    kin = build_kinematics(config)
    dyn = OpenManipulatorDynamics(use_jit=args.use_jit)
    n = kin.n  # 3 for the waist-removed arm
    backend = create_backend(args.backend, config, kin, args)

    cfg = controller_config(config)
    # This arm's 3 joints all rotate about the same (horizontal) axis, so the
    # end-effector's y coordinate never changes -- the real task space is the
    # 2D x-z plane, not full 3D (using 3D would make Lambda(q) structurally
    # singular in the y direction).
    cfg.dim = 2
    cfg.u_max = cfg.u_max[:2]
    cfg.use_jit = args.use_jit
    mpc = NormalizedInteractionMPC(cfg)
    obs = RandomWalkDisturbanceObserver(2, cfg.dt, cfg.observer_q_d, cfg.observer_r_y, cfg.observer_d_hat_max)

    if args.use_jit:
        t_warm = dyn.warmup() + mpc.warmup()
        print(f"[jit] Numba warm-up compile done in {t_warm:.2f}s (one-time cost, before the "
              f"real-time loop below)")

    robot = config.get("robot", {})
    g_scale = float(robot.get("gravity_scale", 1.0))
    # Optional reflected-rotor/gearbox inertia term, ADDED ON TOP of dyn.mass_matrix(q)'s
    # link-only inertia -- see docs/01_concepts.md for how this was identified from real
    # hardware data. Default 0.0: no effect unless a config explicitly sets it (and, for
    # --backend sim, must match SimArmBackend's own copy of this same key so the plant
    # the controller is tested against has the inertia the controller believes it has).
    armature = float(robot.get("dyn_armature_kg_m2", 0.0))
    tau_max = np.asarray(robot.get("tau_max_Nm", [3.5, 3.0, 2.0]), dtype=float)
    jmin = np.asarray(robot.get("joint_min_rad", [-1.8, -1.6, -1.8]), dtype=float)
    jmax = np.asarray(robot.get("joint_max_rad", [1.6, 1.4, 1.8]), dtype=float)
    barrier_k = float(robot.get("joint_barrier_gain", 4.0))
    barrier_margin = float(robot.get("joint_barrier_margin_rad", 0.15))
    startup_ramp = float(robot.get("startup_ramp_s", 2.0))
    q_nom = np.asarray(robot.get("posture_q_rad", robot.get("sim_home_q_rad", [-0.6, 0.3, 0.3])), dtype=float)
    post_kp = float(robot.get("posture_kp", 0.6))
    post_kd = float(robot.get("posture_kd", 0.12))
    lam_damp = float(robot.get("lambda_damping", 2e-3))

    traj = None
    expected_p0 = kin.fk(q_nom)
    q_target_for_move = q_nom

    if args.backend == "dynamixel":
        # Peek at the pose with torque still OFF (read_state() doesn't need enable()
        # first), so a wildly-wrong starting pose can be refused before ever energizing
        # the arm. move_to_start (below) handles the "normal" amount of mismatch from
        # placing the arm by hand automatically -- this check only needs to catch
        # genuinely alarming cases (wrong config loaded, arm in an unexpected pose).
        io_check = backend.read_state()
        assert_startup_pose_plausible(
            io_check.q, jmin, jmax,
            tol_rad=float(robot.get("startup_pose_max_overshoot_rad", 0.5)))
        p0_check = kin.fk(io_check.q)
        mismatch_m = float(np.linalg.norm((p0_check - expected_p0)[[0, 2]]))
        threshold_m = float(robot.get("start_pose_mismatch_max_m", 0.4))
        if mismatch_m > threshold_m and not args.force_start:
            raise RuntimeError(
                f"actual starting pose (q={np.round(io_check.q, 4)} -> ee={np.round(p0_check, 4)}) is "
                f"{mismatch_m * 1000:.0f}mm away from configs's posture_q_rad pose (ee={np.round(expected_p0, 4)}). "
                f"This is well beyond what move_to_start is meant to correct automatically and may indicate "
                f"a wrong config or the arm being in a genuinely unexpected configuration -- verify by hand, "
                f"or pass --force-start to proceed anyway.")

    backend.enable()
    # fh/live/sample declared before the try block so the finally block (and any code
    # after the loop) can safely reference them even if an exception happens early --
    # including during move_to_start, BEFORE the main control loop even starts.
    fh = None; writer = None; live = None; sample = 0
    J_xz_prev = None  # for Jdot_xz (task-space Coriolis/centrifugal term)
    try:
        # Slow, smooth joint-space move from wherever the arm currently is to
        # posture_q_rad, BEFORE the real controller starts -- see lib/move_to_start.py's
        # module docstring for why this exists. --move-to-start-timeout-s <=0 skips it.
        if args.move_to_start_timeout_s > 0:
            move_pos_eps = float(robot.get("move_to_start_pos_eps_rad", 0.1))
            move_vel_eps = float(robot.get("move_to_start_vel_eps_rad_s", 0.05))
            move_settle_s = float(robot.get("move_to_start_settle_time_s", 2.0))
            move_kp = float(robot.get("move_to_start_kp", 0.1))
            move_ki = float(robot.get("move_to_start_ki", 0.05))
            move_kd = float(robot.get("move_to_start_kd", 0.03))
            move_to_start(backend, dyn, q_target_for_move, dt=cfg.dt,
                           tau_max=tau_max, kp=move_kp, ki=move_ki, kd=move_kd,
                           gravity_scale=g_scale, timeout_s=args.move_to_start_timeout_s,
                           pos_eps=move_pos_eps, vel_eps=move_vel_eps, settle_time_s=move_settle_s,
                           jmin=jmin, jmax=jmax, barrier_k=barrier_k, barrier_margin=barrier_margin)
            # --backend sim's disturbance.push/payload timing (t_start/t_end) is meant to be
            # relative to when the REAL trajectory tracking starts, but SimArmBackend's
            # internal clock has already been advanced by move_to_start's own read_state()
            # calls above -- reset it so a disturbance scheduled for e.g. t_start=4.0
            # fires 4s into the real run, not 4s minus however long move_to_start took.
            if hasattr(backend, "reset_time"):
                backend.reset_time()

        io = backend.read_state()
        p0 = kin.fk(io.q)
        traj = CartesianTrajectory(config.get("trajectory", {}), p0, 0.0)
        d_hat = np.zeros(2)
        post_integral = np.zeros(n)  # unused (no integral term below) -- reserved for symmetry with move_to_start

        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            fh = open(args.output, "w", newline="")

        if args.live_plot:
            if args.backend == "dynamixel":
                print("[omx-3dof] WARNING: --live-plot on real hardware adds rendering jitter "
                      "to the control loop; consider --backend sim for visualization instead.")
            live = _init_live_plot(kin, traj, config.get("trajectory", {}), p0)
        plot_every = max(1, int(round(0.04 / cfg.dt)))  # throttle redraws to ~25 fps regardless of dt

        dur_str = f"{args.duration}s" if args.duration is not None else "unlimited (Ctrl+C to stop)"
        print(f"[omx-3dof] backend={args.backend} ee0={np.round(p0, 4)} dt={cfg.dt} duration={dur_str}")
        start = time.monotonic(); next_tick = start; t = 0.0
        prev_wall = start
        last_print = start
        while True:
            wall = time.monotonic()
            # t is simulated/trajectory time under --backend sim (sample*dt), not
            # wall-clock -- the duration check must use it, or --duration silently means
            # "N real seconds" instead of "N seconds of trajectory time" whenever this
            # machine's own per-tick compute can't keep up with the configured dt.
            t = wall - start if args.backend == "dynamixel" else sample * cfg.dt
            if args.duration is not None and t >= args.duration:
                break
            period_ms = 1000.0 * (wall - prev_wall)   # true wall-clock loop period
            prev_wall = wall
            c0 = time.monotonic()

            io = backend.read_state()
            ee = kin.fk(io.q)
            J = kin.jacobian(io.q)
            ee_vel = J @ io.dq

            xz = [0, 2]  # only x,z: the y-direction Jacobian is structurally zero (see cfg.dim comment above)
            J_xz = J[xz, :]          # 2x3
            # Jdot_xz via backward finite difference on J_xz itself (consistent with
            # kinematics.py's own jacobian() already being numerical). Zero on the very
            # first tick (no previous sample yet); harmless since dq~=0 at t=0 anyway.
            Jdot_xz = (J_xz - J_xz_prev) / cfg.dt if J_xz_prev is not None else np.zeros_like(J_xz)
            J_xz_prev = J_xz.copy()
            ee_xz = ee[xz]
            ee_vel_xz = ee_vel[xz]

            p_d, dp_d, ddp_d = traj.sample(t)
            y_err = ee_xz - p_d[xz]
            x_state = np.r_[y_err, ee_vel_xz - dp_d[xz]]
            err_mm = 1000.0 * float(np.linalg.norm(y_err))

            # Operational-space realization (2D, x-z plane) with null-space posture control:
            #   F = Lambda(q)(xdd_d + u) + gravity;  N = I - J^T (Lambda J M^-1)
            ramp = min(1.0, t / max(startup_ramp, 1e-6))  # linear gain ramp-up over the first
            # startup_ramp_s seconds, so a large initial error doesn't produce a large
            # instantaneous torque command the moment the loop starts.
            Mq = dyn.mass_matrix(io.q) + armature * np.eye(n)
            Minv = np.linalg.inv(Mq)
            Lam = np.linalg.inv(J_xz @ Minv @ J_xz.T + lam_damp * np.eye(2))
            N = np.eye(n) - J_xz.T @ (Lam @ J_xz @ Minv)        # dyn-consistent null-space
            post_err = io.q - q_nom
            tau_post = -post_kp * post_err - post_kd * io.dq
            # Task-space Coriolis/centrifugal decoupling term mu = Lam(Jxz Minv C(q,dq)dq -
            # Jdot_xz dq), matching Cao & Tang's classical operational-space impedance law
            # (tau = C(q,dq)dq + G(q) + Jv^T(Lambda*pdd_d + mu + Kp*e + Kd*edot)) -- distinct
            # from the joint-space C(q,dq)dq term below (this one accounts for how the
            # task-space mapping itself changes with q/dq). Computed ONCE and reused below
            # (this used to be two separate RNEA calls for the identical quantity).
            cor = dyn.coriolis(io.q, io.dq)
            mu_xz = Lam @ (J_xz @ Minv @ cor - Jdot_xz @ io.dq)
            tau_base = N @ tau_post + g_scale * dyn.gravity(io.q) + cor + J_xz.T @ mu_xz

            u = mpc.solve(x_state, d_hat)
            d_hat, innovation, nis = obs.step(y_err, u)
            # y_hat = C@z^- = y - innovation (the Kalman filter's own one-step-ahead
            # prediction of the task-space error) -- logged below so it can be checked
            # against the actual measurement (tools/plot_traj.py's perr/perr_hat plot).
            y_hat = y_err - innovation

            F_task = ramp * (Lam @ (ddp_d[xz] + u))
            tau_task = J_xz.T @ F_task
            tau = tau_task + tau_base

            # joint-limit barrier (a soft repulsive term near the joint limits, not a hard
            # constraint): pushes away from the limits, growing linearly once a joint is
            # within barrier_margin_rad of jmin/jmax.
            over_hi = np.maximum(0.0, io.q - (jmax - barrier_margin))
            over_lo = np.maximum(0.0, (jmin + barrier_margin) - io.q)
            tau = tau - barrier_k * over_hi + barrier_k * over_lo
            tau = np.clip(tau, -tau_max, tau_max)
            backend.send_torque(tau)

            compute_ms = 1000.0 * (time.monotonic() - c0)
            if live is not None and sample % plot_every == 0:
                _update_live_plot(live, kin, io.q, p_d, t, err_mm, 1000.0 / max(period_ms, 1e-6))
            if fh is not None:
                # u/d_hat/F_task are 2D (x,z); pad to 3 elements (0 in the unused y slot)
                # so every CSV always has the same, tool-compatible column layout.
                u3 = np.array([u[0], 0.0, u[1]])
                d_hat3 = np.array([d_hat[0], 0.0, d_hat[1]])
                F3 = np.array([F_task[0], 0.0, F_task[1]])
                innovation3 = np.array([innovation[0], 0.0, innovation[1]])
                y_hat3 = np.array([y_hat[0], 0.0, y_hat[1]])
                row = {"sample": sample, "t": t, "mode": config.get("trajectory", {}).get("type", "?"),
                       "err_mm": err_mm, "nis": nis, "compute_ms": compute_ms, "period_ms": period_ms,
                       "q_pos": cfg.q_pos, "q_vel": cfg.q_vel, "r": cfg.r,
                       "observer_q_d": cfg.observer_q_d, "observer_r_y": cfg.observer_r_y,
                       "observer_d_hat_max": cfg.observer_d_hat_max,
                       "dyn_armature_kg_m2": armature}
                for name, vec in (("ee", ee), ("p_d", p_d), ("ee_vel", ee_vel), ("u", u3),
                                  ("d_hat", d_hat3), ("F", F3), ("q", io.q), ("tau", tau),
                                  ("cur", io.current_A), ("innovation", innovation3), ("y_hat", y_hat3)):
                    for i, v in enumerate(np.asarray(vec).reshape(-1)):
                        row[f"{name}_{i}"] = float(v)
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row.keys())); writer.writeheader()
                writer.writerow(row)

            sample += 1; next_tick += cfg.dt
            now = time.monotonic()
            if now - last_print >= 0.5:
                hz_inst = 1000.0 / max(period_ms, 1e-6)
                print(f"\r[omx-3dof] t={t:6.2f}s  err={err_mm:7.2f}mm  "
                      f"tau={np.round(tau, 2)}  ~{hz_inst:5.1f}Hz  n={sample:6d}   ",
                      end="", flush=True)
                last_print = now
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        backend.disable()
        if fh is not None:
            fh.close()
        print(f"\n[omx-3dof] torque disabled; {sample} samples")
    if live is not None:
        import matplotlib.pyplot as plt
        print("[omx-3dof] close the plot window to exit")
        plt.ioff(); plt.show()
    return args.output


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--backend", choices=["sim", "dynamixel"], default="sim")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--live-plot", action="store_true",
                     help="show a live matplotlib view of the arm links + ee trace while running")
    ap.add_argument("--force-start", action="store_true",
                     help="skip the starting-pose-vs-posture_q_rad safety check on --backend dynamixel")
    ap.add_argument("--move-to-start-timeout-s", type=float, default=20.0,
                     help="safety cap (seconds) on the pure joint-space PID move (not the controller "
                          "under test) from wherever the arm currently is to posture_q_rad: runs until it "
                          "actually settles there, however long that takes, then starts the real trajectory "
                          "tracking. <=0 disables this and starts tracking immediately from the current pose.")
    ap.add_argument("--use-jit", action="store_true",
                     help="Numba-JIT-compile the RNEA (lib/dynamics.py) and FISTA (lib/interaction_mpc.py) "
                          "hot paths instead of plain numpy -- measured ~65x/~5x speedups, see "
                          "implementation_fix.md's \"would C++ help\" section. Requires `pip install numba` "
                          "(optional dependency, off by default). Pays a one-time JIT compile cost "
                          "(~1-3s) at startup, before the real-time loop -- not on its first tick.")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
