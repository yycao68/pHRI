"""The merged PIR controller: phri2's realization QP on a passive nominal,
with impedance_residual's fast energy authorization on the realized port.

Structure (synthesis note Sections 3-4)::

    F_cmd = F_nom + F_r,        F_nom = -K0 e - D0 v = -G0 x

* the **behaviour layer** still emits ``a_id = C x + G F_h`` (unchanged);
* the **realization QP** now optimizes only the residual ``F_r``, at 50 Hz,
  under a horizon-wide joint-torque constraint;
* the **1 kHz servo** re-derives the anchor from the current ``(q, qdot)``,
  scales ``F_r`` for torque feasibility (``alpha_tau``) and then for energy
  authorization against a tank that harvests ``v^T D0 v`` (``alpha_E``).

Two things are genuinely different from grafting the tank onto phri2, and both
live in ``PIRRealizationMPC._condense``:

1. **The nominal is closed-loop inside the horizon, not a constant offset.**
   ``F_nom`` depends on the predicted state, so the horizon dynamics become
   ``x_{k+1} = (A - B G0) x_k + B F_r_k + ...``.  Predicting with a frozen
   ``F_nom`` would let the QP plan against a nominal that the nominal itself
   invalidates.
2. **The behaviour input and the disturbance model are separate channels.**
   phri2 has one force signal: the same ``F_h`` drives the behaviour law
   ``a_id = C x + G F_h`` and the plant prediction.  That is only sound when
   every external force is interaction force.  impedance_residual, which runs
   under an explicit disturbance, keeps the two apart -- its reference is the
   origin and ``disturbance_hat`` enters only the prediction.  The merged QP
   needs both, so ``control`` takes ``force_forecast`` (everything the plant
   will feel) and ``behaviour_forecast`` (the part the behaviour is defined
   against), defaulting to the same array, which recovers phri2 exactly.
   Splitting them does **not** solve force misclassification -- deciding
   which measured force goes in which channel is precisely that unsolved
   problem -- but it puts the assumption in the signature instead of hiding
   it in an equality.

3. **The torque rows become state-dependent.**  In phri2,
   ``tau_k = tau_base + J_v^T F_cmd_k`` is affine in the decision variable
   alone.  Here ``tau_k = tau_base + J_v^T(-G0 x_k + F_r_k)`` and ``x_k`` is
   itself affine in the earlier residuals, so every torque row picks up the
   accumulated nominal reaction.  This is what makes the anchor budget a
   horizon-wide constraint rather than a per-tick check.

Everything else -- ``compute_tau_base``, the FR3 dynamics, the slack-relaxed
workspace box, the OSQP settings, ``torque_scale`` -- is imported from the two
source studies rather than reimplemented.
"""

from __future__ import annotations

import time as _time
from dataclasses import dataclass, field, replace
from typing import Protocol

import numpy as np
import osqp
from scipy import sparse

import pir_common as pc
from pir_common import (  # noqa: F401  (re-exported for callers)
    FR3MPCConfig,
    combine_full_torque,
    compute_tau_base,
    make_default_impedance_params,
)

# impedance_residual's own per-joint box-margin ratio.  Imported, not
# reimplemented: it is the exact closed form Merged Lemma 1's torque half is
# stated about, and a second copy could drift from the one the paper proves.
from verify_fr3_two_rate_benchmark import torque_scale  # noqa: E402


class InteractionGenerator(Protocol):
    name: str

    def affine_law(self) -> tuple[np.ndarray, np.ndarray]: ...

    def acceleration(self, state: np.ndarray, human_force: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class PIRConfig:
    """Merged configuration: phri2's QP settings plus the nominal and the tank."""

    #: The passive nominal, from the Task 1 scan's most-robust cell.  Scalars
    #: give the isotropic gains the scan swept; a 3-vector gives per-axis
    #: gains, which Section 13.2 of the plan defers as "a later refinement".
    k0: float | tuple[float, float, float] = 380.0
    d0: float | tuple[float, float, float] = 29.07

    #: Torque envelope by name; see pir_common.torque_envelope.
    envelope: str = "derated_joint4"

    #: Energy tank, from impedance_residual's Config.  E_min is the floor the
    #: fast authorization defends; E_max caps harvesting so a long dissipative
    #: stretch cannot bank unlimited authority.
    tank_initial: float = 0.08
    tank_minimum: float = 0.02
    tank_maximum: float = 0.30

    #: How often alpha_E is recomputed, in 1 kHz servo ticks.  1 = every tick
    #: (the merged rule).  20 = once per manager update, which is
    #: impedance_residual's B4 manager-rate guard -- the baseline that tracks
    #: like the fast guard but breaches the floor.
    auth_period_ticks: int = 1

    #: Set False to run the same controller with the tank disabled, isolating
    #: what the passivity axis costs and buys.
    energy_authorization: bool = True

    #: Give the fast layer authority over the NOMINAL as well as the residual.
    #:
    #: Section 7.7 found the split's worst cost: alpha_tau scales only F_r, so
    #: once |tau_base + J^T F_nom| > cap nothing in the servo can prevent the
    #: overrun, and phri2's unconditional torque guarantee becomes conditional
    #: on a precondition the runtime cannot defend.  With this enabled the
    #: servo also computes alpha_nom, the largest scale keeping the anchor
    #: itself inside the box.  Since alpha_nom = 0 recovers tau_base, the
    #: guarantee becomes unconditional whenever tau_base alone fits -- a much
    #: weaker and checkable condition.
    #:
    #: It is not free.  Scaling the nominal down softens the passivity floor
    #: exactly when it is most needed, and a time-varying spring gain can
    #: release stored energy.  Both are metered: the tank harvests only the
    #: APPLIED damping alpha_nom * D0 |v|^2, and any drop in alpha_nom debits
    #: the spring energy it releases.  Whether the floor still holds under that
    #: accounting is measured, not assumed (see pir_fixes.py).
    nominal_authorization: bool = False

    #: Ceiling on how fast alpha_nom may RISE, in units of alpha per second.
    #: Re-stiffening is the direction that charges the tank (see the servo), so
    #: an unlimited rise lets a transient repeatedly re-buy stiffness the port
    #: has not earned.  ``inf`` leaves the rise unrestricted; ``0.0`` makes
    #: alpha_nom monotone non-increasing, which removes the charge entirely at
    #: the cost of never recovering the floor's stiffness after a transient.
    nominal_reauth_rate: float = float("inf")

    mpc: FR3MPCConfig = field(default_factory=FR3MPCConfig)

    def resolved_mpc(self) -> FR3MPCConfig:
        """phri2's QP config with this envelope's cap substituted in."""
        return replace(self.mpc, tau_max=pc.torque_envelope(self.envelope))

    def k0_vector(self) -> np.ndarray:
        return np.broadcast_to(np.asarray(self.k0, dtype=float), (3,)).copy()

    def d0_vector(self) -> np.ndarray:
        return np.broadcast_to(np.asarray(self.d0, dtype=float), (3,)).copy()

    def gain_matrix(self) -> np.ndarray:
        """G0 = [K0  D0] in F_nom = -G0 x, x = [e; v]."""
        return np.hstack([np.diag(self.k0_vector()), np.diag(self.d0_vector())])


@dataclass(frozen=True)
class PIRServoStep:
    """One 1 kHz tick of the merged re-authorization rule."""

    tau: np.ndarray  # (7,) applied joint torque
    f_nom: np.ndarray  # (3,) passive nominal force, AFTER alpha_nom
    f_r_applied: np.ndarray  # (3,) residual after both scalings
    anchor: np.ndarray  # (7,) tau_base + J_v^T F_nom
    anchor_ratio: float  # ||anchor||_inf / cap   -- Lemma 1's precondition
    tau_ratio: float  # ||tau||_inf / cap        -- Lemma 1's conclusion
    alpha_tau: float
    alpha_E: float
    alpha_nom: float  # 1.0 unless nominal_authorization is on and the anchor bit
    anchor_feasible: bool
    tank: float
    tank_floor_ok: bool
    r_con_fast: np.ndarray  # (3,) acceleration lost to the fast torque scaling
    r_auth: np.ndarray  # (3,) acceleration lost to the energy authorization


# ---------------------------------------------------------------------------
# 50 Hz layer: the realization QP, now optimizing the residual
# ---------------------------------------------------------------------------


class PIRRealizationMPC:
    """phri2's condensed realization QP, re-posed around a passive nominal."""

    def __init__(self, generator: InteractionGenerator, config: PIRConfig | None = None) -> None:
        self.generator = generator
        self.pir = config or PIRConfig()
        self.cfg = self.pir.resolved_mpc()
        self.imp_params = make_default_impedance_params(self.cfg)
        self.previous_residual = np.zeros(3)
        self._warm_x: np.ndarray | None = None
        self._warm_y: np.ndarray | None = None

    def reset(self) -> None:
        self.previous_residual = np.zeros(3)
        self._warm_x = None
        self._warm_y = None

    # -- assembly ---------------------------------------------------------

    def _condense(self, dyn, state, p_nominal, R_d, force_forecast, behaviour_forecast):
        cfg = self.cfg
        dt, H = cfg.dt, cfg.horizon
        n_i, n_s = 3 * H, 2 * H
        n = n_i + n_s
        g0 = self.pir.gain_matrix()

        def pos_slack_col(k: int) -> int:
            return n_i + k

        def speed_slack_col(k: int) -> int:
            return n_i + H + k

        J_v = dyn.J[:3, :]
        M_inv = np.linalg.inv(dyn.M)
        Lam_inv = J_v @ M_inv @ J_v.T + cfg.lambda_reg * np.eye(3)
        tau_base, _, d_known = compute_tau_base(
            dyn, state, R_d, self.imp_params, cfg.K_rot, cfg.D_rot, cfg.lambda_reg
        )

        A = np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        B = np.vstack((0.5 * dt**2 * Lam_inv, dt * Lam_inv))
        B_accel = np.vstack((0.5 * dt**2 * np.eye(3), dt * np.eye(3)))
        # The passive nominal is state feedback, so it closes the loop inside
        # the prediction.  Freezing F_nom at its current value instead would
        # let the QP plan against a nominal its own plan invalidates.
        A_cl = A - B @ g0

        x0 = np.concatenate([state.ee_pos - p_nominal, state.ee_vel[:3]])
        c_id, g_id = self.generator.affine_law()
        # The nominal's own acceleration contribution folds into the behaviour
        # matrix: the residual the QP must cancel is measured against
        # a_id, and F_nom already supplies -Lam^-1 G0 x of it.
        c_eff = c_id + Lam_inv @ g0
        jt_g0 = J_v.T @ g0  # the nominal's joint-torque reaction, per state

        state_map = np.zeros((6, n_i))
        state_offset = x0.copy()

        residual_maps: list[np.ndarray] = []
        residual_offsets: list[np.ndarray] = []
        constraint_maps: list[np.ndarray] = []
        lowers: list[np.ndarray] = []
        uppers: list[np.ndarray] = []
        labels: list[str] = []

        for k in range(H):
            selector = np.zeros((3, n_i))
            selector[:, 3 * k : 3 * k + 3] = np.eye(3)
            f_k = force_forecast[k]  # what the plant feels
            b_k = behaviour_forecast[k]  # what the behaviour responds to

            # --- realization residual at step k, evaluated at x_k ----------
            residual_maps.append(Lam_inv @ selector - c_eff @ state_map)
            residual_offsets.append(
                Lam_inv @ f_k + d_known - c_eff @ state_offset - g_id @ b_k
            )

            # --- torque at step k, ALSO at x_k -----------------------------
            # tau_k = tau_base + J_v^T (-G0 x_k + F_r_k).  Unlike phri2 this
            # is not a function of the decision variable alone: the nominal's
            # reaction at step k depends on every residual before it.
            torque_map = np.zeros((7, n))
            torque_map[:, :n_i] = J_v.T @ selector - jt_g0 @ state_map
            torque_offset = tau_base - jt_g0 @ state_offset
            constraint_maps.append(torque_map)
            torque_steps = H if cfg.torque_constraint_steps is None else cfg.torque_constraint_steps
            if k < torque_steps:
                lowers.append(-cfg.tau_max - torque_offset)
                uppers.append(cfg.tau_max - torque_offset)
            else:
                lowers.append(-np.inf * np.ones(7))
                uppers.append(np.inf * np.ones(7))
            labels.append(f"torque[{k}]")

            # --- propagate through the CLOSED-loop dynamics ----------------
            state_map = A_cl @ state_map + B @ selector
            state_offset = A_cl @ state_offset + B @ f_k + B_accel @ d_known

            # --- slack-relaxed workspace / speed box on x_{k+1} ------------
            # Same soft treatment and the same slack scaling as phri2: these
            # are preferences under model mismatch, not actuator limits.
            inv_slack = 1.0 / cfg.slack_variable_scale
            for rows, limit, col, name in (
                (slice(0, 3), cfg.position_limit, pos_slack_col(k), "position"),
                (slice(3, 6), cfg.speed_limit, speed_slack_col(k), "speed"),
            ):
                box = np.zeros((6, n))
                box[:, :n_i] = np.vstack([state_map[rows], state_map[rows]])
                box[:3, col] = -inv_slack
                box[3:, col] = inv_slack
                constraint_maps.append(box)
                lowers.append(np.concatenate(
                    [-np.inf * np.ones(3), -limit * np.ones(3) - state_offset[rows]]))
                uppers.append(np.concatenate(
                    [limit * np.ones(3) - state_offset[rows], np.inf * np.ones(3)]))
                labels.append(f"{name}[{k + 1}]")

        slack_bounds = np.zeros((n_s, n))
        slack_bounds[:, n_i:] = np.eye(n_s)
        constraint_maps.append(slack_bounds)
        lowers.append(np.zeros(n_s))
        uppers.append(np.inf * np.ones(n_s))
        labels.append("slack_nonneg")

        r_map = np.hstack([np.vstack(residual_maps), np.zeros((3 * H, n_s))])
        r_offset = np.concatenate(residual_offsets)

        difference = np.zeros((n_i, n_i))
        difference[:3, :3] = np.eye(3)
        for k in range(1, H):
            difference[3 * k : 3 * k + 3, 3 * k : 3 * k + 3] = np.eye(3)
            difference[3 * k : 3 * k + 3, 3 * (k - 1) : 3 * k] = -np.eye(3)
        difference_offset = np.zeros(n_i)
        difference_offset[:3] = -self.previous_residual
        difference_full = np.zeros((n_i, n))
        difference_full[:, :n_i] = difference

        p = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_map)
            + cfg.force_rate_weight * (difference_full.T @ difference_full)
        )
        p[:n_i, :n_i] += 2.0 * cfg.force_weight * np.eye(n_i) + 1.0e-9 * np.eye(n_i)
        p[n_i:, n_i:] += (2.0 * cfg.slack_weight + 1.0e-9) / cfg.slack_variable_scale**2 * np.eye(n_s)
        q = 2.0 * (
            cfg.realization_weight * (r_map.T @ r_offset)
            + cfg.force_rate_weight * (difference_full.T @ difference_offset)
        )

        # Magnitude and rate limits now bound the RESIDUAL, not the whole
        # command: the nominal is not the QP's to trade away.
        identity = np.zeros((n_i, n))
        identity[:, :n_i] = np.eye(n_i)
        constraint_maps.append(identity)
        lowers.append(-cfg.force_limit * np.ones(n_i))
        uppers.append(cfg.force_limit * np.ones(n_i))
        labels.append("residual")

        constraint_maps.append(difference_full)
        rate_step = cfg.force_rate_limit * cfg.dt
        lowers.append(-rate_step * np.ones(n_i) - difference_offset)
        uppers.append(rate_step * np.ones(n_i) - difference_offset)
        labels.append("residual_rate")

        return (p, q, np.vstack(constraint_maps), np.concatenate(lowers),
                np.concatenate(uppers), labels, Lam_inv, tau_base, J_v, d_known, A_cl, B,
                B_accel, c_eff, g_id, x0)

    # -- solve ------------------------------------------------------------

    def control(self, dyn, state, p_nominal, R_d, force_forecast,
                behaviour_forecast=None) -> dict:
        force_forecast = np.asarray(force_forecast, dtype=float)
        if force_forecast.shape != (self.cfg.horizon, 3):
            raise ValueError(f"force_forecast must be {(self.cfg.horizon, 3)}")
        # Default: every external force is interaction force, which is phri2's
        # single-channel assumption.
        behaviour_forecast = (force_forecast if behaviour_forecast is None
                              else np.asarray(behaviour_forecast, dtype=float))
        if behaviour_forecast.shape != force_forecast.shape:
            raise ValueError("behaviour_forecast must match force_forecast's shape")

        t0 = _time.perf_counter()
        (p, q, a_con, lower, upper, labels, Lam_inv, tau_base, J_v, d_known,
         A_cl, B, B_accel, c_eff, g_id, x0) = self._condense(
            dyn, state, p_nominal, R_d, force_forecast, behaviour_forecast)
        if (not all(np.all(np.isfinite(m)) for m in (p, q, a_con))
                or np.any(np.isnan(lower)) or np.any(np.isnan(upper))):
            raise RuntimeError("PIRRealizationMPC: non-finite value in QP assembly")

        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(p), q=q, A=sparse.csc_matrix(a_con),
                     l=lower, u=upper, verbose=False, polishing=False,
                     eps_abs=self.cfg.osqp_eps, eps_rel=self.cfg.osqp_eps,
                     max_iter=self.cfg.osqp_max_iter)
        if (self.cfg.warm_start and self._warm_x is not None
                and self._warm_x.shape == q.shape and self._warm_y is not None
                and self._warm_y.shape == lower.shape):
            solver.warm_start(x=self._warm_x, y=self._warm_y)
        result = solver.solve(raise_error=False)
        solve_time = _time.perf_counter() - t0
        if result.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP failed: {result.info.status}")
        sequence = np.asarray(result.x)
        if not np.all(np.isfinite(sequence)):
            raise RuntimeError(f"OSQP status={result.info.status} but result.x is non-finite")
        feas_tol = 100 * self.cfg.osqp_eps
        primal = float(np.maximum(np.maximum(lower - a_con @ sequence, 0),
                                  np.maximum(a_con @ sequence - upper, 0)).max())
        if primal > feas_tol:
            raise RuntimeError(
                f"OSQP status={result.info.status} but primal residual {primal:.3e} "
                f"exceeds tolerance {feas_tol:.3e}")
        if self.cfg.warm_start:
            self._warm_x, self._warm_y = np.asarray(result.x), np.asarray(result.y)

        H, n_i = self.cfg.horizon, 3 * self.cfg.horizon
        n_s = 2 * H
        f_r = sequence[:3].copy()

        # --- residual decomposition, rows r_reg and r_con (QP level) -------
        # Same construction as phri2's: the unconstrained optimum of the SAME
        # objective isolates what regularization costs from what the torque
        # and box constraints cost.
        unconstrained = np.linalg.solve(p, -q)
        f_h0 = force_forecast[0]
        a_id_0 = self.generator.acceleration(x0, behaviour_forecast[0])
        f_nom_0 = -self.pir.gain_matrix() @ x0

        def modelled_acceleration(residual: np.ndarray) -> np.ndarray:
            return Lam_inv @ (f_nom_0 + residual + f_h0) + d_known

        a_unc = modelled_acceleration(unconstrained[:3])
        a_con_0 = modelled_acceleration(f_r)
        r_reg = a_unc - a_id_0
        r_con_qp = a_con_0 - a_unc
        closure = a_con_0 - a_id_0 - r_reg - r_con_qp

        self.previous_residual = f_r

        # --- horizon diagnostics, under the closed-loop prediction ---------
        predicted_x = x0.copy()
        horizon_tau = np.zeros((H, 7))
        residuals = []
        g0 = self.pir.gain_matrix()
        for k in range(H):
            f_k = sequence[3 * k : 3 * k + 3]
            fh_k = force_forecast[k]
            f_nom_k = -g0 @ predicted_x
            residuals.append(Lam_inv @ (f_nom_k + f_k + fh_k) + d_known
                             - self.generator.acceleration(predicted_x,
                                                           behaviour_forecast[k]))
            horizon_tau[k] = combine_full_torque(tau_base, J_v, f_nom_k + f_k)
            predicted_x = A_cl @ predicted_x + B @ (f_k + fh_k) + B_accel @ d_known

        value = a_con @ sequence
        tol = 2.0e-4
        active: list[str] = []
        row = 0
        for label, block in zip(labels, ([7, 6, 6] * H) + [n_s, n_i, n_i]):
            sl = slice(row, row + block)
            if np.any(value[sl] <= lower[sl] + tol) or np.any(value[sl] >= upper[sl] - tol):
                active.append(label)
            row += block

        slack_pos = sequence[n_i : n_i + H] / self.cfg.slack_variable_scale
        slack_spd = sequence[n_i + H : n_i + n_s] / self.cfg.slack_variable_scale
        return {
            "residual_command": f_r,
            "r_reg": r_reg,
            "r_con_qp": r_con_qp,
            "decomposition_closure_error": closure,
            "horizon_tau": horizon_tau,
            "predicted_residual_rms": float(np.sqrt(np.mean(np.square(residuals)))),
            "active_constraints": tuple(active),
            "status": result.info.status,
            "solve_time_s": solve_time,
            "slack_max_position": float(np.max(slack_pos)) if H else 0.0,
            "slack_max_speed": float(np.max(slack_spd)) if H else 0.0,
        }


# ---------------------------------------------------------------------------
# 1 kHz layer: the merged fast re-authorization rule (note Section 4)
# ---------------------------------------------------------------------------


def pir_servo_step(
    cfg: PIRConfig,
    tau_base: np.ndarray,
    J_v: np.ndarray,
    Lam_inv: np.ndarray,
    e: np.ndarray,
    v: np.ndarray,
    f_r_held: np.ndarray,
    tank: float,
    h: float,
    cap: np.ndarray,
    held_alpha_E: float | None = None,
    previous_alpha_nom: float = 1.0,
) -> PIRServoStep:
    """Steps 1-6 of the merged rule, for one 1 kHz tick.

    ``held_alpha_E`` is the manager-rate baseline's stale energy scale: pass
    it to reuse an ``alpha_E`` computed at an earlier tick (impedance_residual's
    B4 guard), or ``None`` to recompute from this tick's own velocity and tank
    (the merged rule, B5).
    """
    # 1-2. Anchor from the CURRENT state, and Lemma 1's precondition.
    k0v, d0v = cfg.k0_vector(), cfg.d0_vector()
    f_nom_full = -k0v * e - d0v * v
    anchor_full = tau_base + J_v.T @ f_nom_full
    anchor_ratio = float(np.max(np.abs(anchor_full) / cap))
    anchor_feasible = anchor_ratio <= 1.0 + 1e-12

    # 2b. Optionally give the servo authority over the nominal too, so that an
    # infeasible anchor is something it can act on rather than merely report.
    if cfg.nominal_authorization:
        alpha_nom = 1.0
        if not anchor_feasible:
            alpha_nom, _ = torque_scale(tau_base, J_v.T @ f_nom_full, cap)
        if np.isfinite(cfg.nominal_reauth_rate):
            alpha_nom = min(alpha_nom,
                            previous_alpha_nom + cfg.nominal_reauth_rate * h)
    else:
        alpha_nom = 1.0
    f_nom = alpha_nom * f_nom_full
    anchor = tau_base + J_v.T @ f_nom

    # 3. Torque scale: largest alpha keeping anchor + alpha J^T F_r in the box.
    alpha_tau, _ = torque_scale(anchor, J_v.T @ f_r_held, cap)
    f_r_bar = alpha_tau * f_r_held

    # 4. Energy scale against the tank, harvesting the nominal's dissipation.
    # A time-varying spring gain is an energy term, and only one direction is
    # dangerous.  SOFTENING (alpha_nom falling) releases stored energy: the
    # storage function's ((1/2) d(alpha)/dt e^T K0 e) term goes negative, which
    # helps passivity, so it is neither charged nor credited -- crediting it
    # would let a softening transient bank authority it has not earned.
    # RE-STIFFENING (alpha_nom rising) is the injection: at fixed e the robot
    # suddenly pushes back harder without the human having done the work to
    # store it.  That energy is charged to the tank.
    spring_release = max(0.0, 0.5 * float((alpha_nom - previous_alpha_nom)
                                          * (e @ (k0v * e))))

    if not cfg.energy_authorization:
        alpha_E = 1.0
    elif held_alpha_E is not None:
        alpha_E = held_alpha_E
    else:
        power = float(f_r_bar @ v)
        if power <= 0.0:
            alpha_E = 1.0
        else:
            # Harvest only the damping actually APPLIED: with the nominal
            # de-authorized, alpha_nom * D0 is the damper the port really has.
            dissipation = alpha_nom * float(v @ (d0v * v))
            available = max(0.0, tank - cfg.tank_minimum + h * dissipation
                            - spring_release)
            alpha_E = min(1.0, available / (h * power + 1e-15))

    # 5. Apply.
    f_r_applied = alpha_E * f_r_bar
    tau = tau_base + J_v.T @ (f_nom + f_r_applied)

    # 6. Ledger: harvest the applied damping, debit the residual's own port
    # power, and debit any spring energy released by softening the nominal.
    next_tank = min(cfg.tank_maximum,
                    tank + h * (alpha_nom * float(v @ (d0v * v))
                                - float(f_r_applied @ v))
                    - spring_release)

    return PIRServoStep(
        tau=tau,
        f_nom=f_nom,
        f_r_applied=f_r_applied,
        anchor=anchor,
        anchor_ratio=anchor_ratio,
        tau_ratio=float(np.max(np.abs(tau) / cap)),
        alpha_tau=float(alpha_tau),
        alpha_E=float(alpha_E),
        alpha_nom=float(alpha_nom),
        anchor_feasible=anchor_feasible,
        tank=next_tank,
        tank_floor_ok=bool(next_tank >= cfg.tank_minimum - 1e-12),
        # The four-term residual's two new members, both in acceleration units
        # so they add to phri2's r_reg / r_con / r_mod directly.
        r_con_fast=Lam_inv @ ((alpha_tau - 1.0) * f_r_held),
        r_auth=Lam_inv @ ((alpha_E - 1.0) * f_r_bar),
    )
