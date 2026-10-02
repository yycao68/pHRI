#!/usr/bin/env python3
"""Solve for `q_pos` (LQR cost weight) that reproduces a TARGET task-space
stiffness `Kp` [N/m] at a config's own operating posture -- instead of
guessing a q_pos number and hoping the resulting force-domain gain is
reasonable.

Why this exists: `q_pos` is a dimensionless cost weight; the physically
meaningful quantity is the resulting closed-loop task-space stiffness/damping
`Kp = Lambda(q) @ K_pos`, `Kd = Lambda(q) @ K_vel` (`docs/02_tuning_guide.md`
section 4 already derives this mapping for matching MPC-vs-PID stiffness --
this tool solves it in the other direction, target-stiffness -> q_pos). The
mapping is strongly nonlinear and saturating (raising q_pos gives
diminishing returns in Kp), so it has to be solved numerically, not guessed
or copied from a different posture's Lambda(q).

IMPORTANT CAVEAT this tool does NOT resolve: the loop-gain delay stability
criterion from the original stage report (|L| = sqrt(Kp^2+(w*Kd)^2) /
(Lambda_ii * w^2), evaluated near the observed oscillation frequency) is
ALSO reported here for context, using that report's own w=50.27 rad/s
(f_crit=8Hz) -- but that frequency was derived from a ~30ms round-trip loop
delay dominated by an ~8ms QP solve that no longer exists in this codebase
(see docs/implementation_fix.md: FISTA now costs ~1ms). The TRUE stability
boundary for THIS loop is very likely higher (more permissive) than what
this old frequency implies, but confirming that requires an actual hardware
sweep (mirroring the original report's own Section 6.1), which this tool
cannot do. Treat the printed |L| as a rough, conservative sanity check
against the OLD boundary, not a certificate of stability -- always verify a
new gain in --backend sim first, then sweep up cautiously on real hardware
per docs/02_tuning_guide.md section 3, step 5.

Usage:
    python tools/solve_task_space_gain.py --config configs/hold.yaml --target-kp-zz 12
    python tools/solve_task_space_gain.py --config configs/hold.yaml --sweep
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

from interaction_mpc import ControllerConfig, NormalizedInteractionMPC  # noqa: E402
from kinematics import OpenManipulatorKinematics  # noqa: E402
from dynamics import OpenManipulatorDynamics  # noqa: E402

OMEGA_REPORT = 50.27  # original report's observed oscillation frequency (f_crit=8Hz); see caveat above


def lambda_at_posture(config: dict) -> np.ndarray:
    kin = OpenManipulatorKinematics()
    dyn = OpenManipulatorDynamics()
    q_nom = np.asarray(config["robot"]["posture_q_rad"], dtype=float)
    n = kin.n
    J = kin.jacobian(q_nom)
    Jxz = J[[0, 2], :]
    # Must match run_hardware.py's own Mq exactly, including the reflected-
    # rotor/gearbox armature term -- omitting it (as an earlier version of
    # this tool did) understates M(q) and overstates Lambda's anisotropy;
    # see docs/original_implementation.md's armature-correction finding.
    armature = float(config["robot"].get("dyn_armature_kg_m2", 0.0))
    Mq = dyn.mass_matrix(q_nom) + armature * np.eye(n)
    Minv = np.linalg.inv(Mq)
    lam_damp = float(config["robot"].get("lambda_damping", 2e-3))
    return np.linalg.inv(Jxz @ Minv @ Jxz.T + lam_damp * np.eye(2))


def kp_kd(q_pos: float, q_vel: float, r: float, horizon: int, dt: float, Lam: np.ndarray):
    """Closed-form UNCONSTRAINED first-step gain of the box-QP (u_max=inf) --
    matches the report's own Eq. 20 derivation (valid whenever the box
    constraint is not active, which the original report found to always be
    the case in its own study)."""
    cfg = ControllerConfig(dim=2, dt=dt, horizon=horizon, q_pos=q_pos, q_vel=q_vel,
                            r=r, u_max=np.array([1e9, 1e9]))
    mpc = NormalizedInteractionMPC(cfg)
    K_eff = mpc.effective_gain()
    return Lam @ K_eff[:, :2], Lam @ K_eff[:, 2:]


def l_metric(Kp: np.ndarray, Kd: np.ndarray, Lam: np.ndarray, w: float = OMEGA_REPORT):
    Lx = np.sqrt(Kp[0, 0] ** 2 + (w * Kd[0, 0]) ** 2) / (Lam[0, 0] * w * w)
    Lz = np.sqrt(Kp[1, 1] ** 2 + (w * Kd[1, 1]) ** 2) / (Lam[1, 1] * w * w)
    return Lx, Lz


def _log_bisect(f, lo: float, hi: float, iters: int = 60) -> float:
    """Log-scale bisection (the q_pos -> gain mapping saturates, so linear
    bisection converges far more slowly / can miss the root's scale)."""
    flo, fhi = f(lo), f(hi)
    if not (flo < 0 < fhi):
        raise ValueError(f"root not bracketed by q_pos in [{lo}, {hi}]; "
                          f"f(lo)={flo:.3g}, f(hi)={fhi:.3g}")
    for _ in range(iters):
        mid = np.sqrt(lo * hi)
        if f(mid) > 0:
            hi = mid
        else:
            lo = mid
    return np.sqrt(lo * hi)


def solve_qpos_for_kp_zz(target: float, q_vel: float, r: float, horizon: int, dt: float,
                          Lam: np.ndarray, lo: float = 1.0, hi: float = 1e8) -> float:
    def f(qpos):
        Kp, _ = kp_kd(qpos, q_vel, r, horizon, dt, Lam)
        return Kp[1, 1] - target
    return _log_bisect(f, lo, hi)


def solve_qpos_for_L(target_L: float, q_vel: float, r: float, horizon: int, dt: float,
                      Lam: np.ndarray, w: float = OMEGA_REPORT,
                      lo: float = 1.0, hi: float = 1e8) -> float:
    """Solve for q_pos matching a target |L| (the report's own stability
    metric) rather than a raw Kp value -- see this module's docstring caveat
    for why matching |L| is the more defensible target across postures with
    different Lambda(q), and why even |L| itself is not a guarantee."""
    def f(qpos):
        Kp, Kd = kp_kd(qpos, q_vel, r, horizon, dt, Lam)
        Lx, Lz = l_metric(Kp, Kd, Lam, w)
        return max(Lx, Lz) - target_L
    return _log_bisect(f, lo, hi)


def main() -> None:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--config", type=Path, default=ROOT / "configs" / "hold.yaml")
    ap.add_argument("--target-kp-zz", type=float, default=None,
                     help="target task-space stiffness [N/m] on the softer (z) axis; "
                          "solves for q_pos (with q_vel=0) that reproduces it")
    ap.add_argument("--target-l", type=float, default=None,
                     help="target |L| (the report's own loop-gain-delay stability metric, "
                          "see module docstring caveat) instead of a raw Kp value; solves "
                          "for q_pos that reproduces it")
    ap.add_argument("--q-vel", type=float, default=0.0,
                     help="q_vel to hold fixed while solving (0 decouples stiffness from "
                          "damping -- see the original report's own Section 4.1 Property 2)")
    ap.add_argument("--sweep", action="store_true",
                     help="print a table of target Kp_zz -> (q_pos, Kp, Kd, |L|) instead of "
                          "solving for one value, so you can pick a point with the |L| "
                          "caveat above in view rather than trusting a single number")
    args = ap.parse_args()

    with open(args.config, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    c = config["controller"]
    r, horizon, dt = float(c["r"]), int(c["horizon"]), float(c["dt"])
    Lam = lambda_at_posture(config)
    print(f"config={args.config.name}  Lam={np.round(Lam, 4).tolist()}  r={r} horizon={horizon}")
    print(f"current: q_pos={c['q_pos']} q_vel={c['q_vel']}")
    Kp0, Kd0 = kp_kd(float(c["q_pos"]), float(c["q_vel"]), r, horizon, dt, Lam)
    Lx0, Lz0 = l_metric(Kp0, Kd0, Lam)
    print(f"  -> Kp diag={np.round(np.diag(Kp0), 3)} N/m  Kd diag={np.round(np.diag(Kd0), 3)} "
          f"Ns/m  |L|={max(Lx0, Lz0):.3f}\n")

    if args.sweep:
        print(f"{'target Kp_zz':>13s} {'q_pos':>10s} {'Kp_xx':>8s} {'Kd_xx':>8s} "
              f"{'Kd_zz':>8s} {'|L|':>7s}")
        for target in [3, 5, 8, 12, 20, 30, 50, 80, 101.39]:
            qpos = solve_qpos_for_kp_zz(target, args.q_vel, r, horizon, dt, Lam)
            Kp, Kd = kp_kd(qpos, args.q_vel, r, horizon, dt, Lam)
            Lx, Lz = l_metric(Kp, Kd, Lam)
            print(f"{target:13.2f} {qpos:10.1f} {Kp[0, 0]:8.2f} {Kd[0, 0]:8.3f} "
                  f"{Kd[1, 1]:8.3f} {max(Lx, Lz):7.3f}")
        return

    if args.target_l is not None:
        qpos = solve_qpos_for_L(args.target_l, args.q_vel, r, horizon, dt, Lam)
        Kp, Kd = kp_kd(qpos, args.q_vel, r, horizon, dt, Lam)
        Lx, Lz = l_metric(Kp, Kd, Lam)
        print(f"solved: q_pos={qpos:.1f} (q_vel={args.q_vel}) for target |L|={args.target_l}")
        print(f"  -> Kp diag={np.round(np.diag(Kp), 3)} N/m  Kd diag={np.round(np.diag(Kd), 3)} "
              f"Ns/m  |L|={max(Lx, Lz):.3f}")
    elif args.target_kp_zz is not None:
        qpos = solve_qpos_for_kp_zz(args.target_kp_zz, args.q_vel, r, horizon, dt, Lam)
        Kp, Kd = kp_kd(qpos, args.q_vel, r, horizon, dt, Lam)
        Lx, Lz = l_metric(Kp, Kd, Lam)
        print(f"solved: q_pos={qpos:.1f} (q_vel={args.q_vel})")
        print(f"  -> Kp diag={np.round(np.diag(Kp), 3)} N/m  Kd diag={np.round(np.diag(Kd), 3)} "
              f"Ns/m  |L|={max(Lx, Lz):.3f}")
        print(f"\n(report's own validated |L| range: 0.156-0.261, at its own posture's Lambda "
              f"and the now-obsolete ~30ms loop delay -- see this file's module docstring caveat)")
    else:
        print("pass --target-kp-zz <N/m> or --target-l <value> to solve, or --sweep for a table")


if __name__ == "__main__":
    main()
