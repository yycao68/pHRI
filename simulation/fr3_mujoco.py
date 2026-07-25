"""
FR3 MuJoCo environment.

Bridges MuJoCo physics to the FrankaDynamics / RobotState interface used
by fr3_impedance.py, so every controller in that module works unmodified.

Torque control is applied via data.qfrc_applied (bypasses MuJoCo actuator
dynamics), which lets us run at the desired impedance-control bandwidth
without worrying about actuator gain tuning in the simulator.
"""

from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).parent))
from fr3_impedance import RobotState, FrankaDynamics

MODELS_DIR    = Path(__file__).parent / "models"
# Scene lives inside franka_fr3/ to keep meshdir="assets" resolution correct
DEFAULT_SCENE = MODELS_DIR / "franka_fr3" / "fr3_phri_scene.xml"
EE_SITE_NAME  = "attachment_site"    # end-effector site in mujoco_menagerie FR3

# FR3 torque limits per joint [1–7] (Nm)
TAU_LIMIT = np.array([87., 87., 87., 87., 12., 12., 12.])

# FR3 neutral/home configuration (rad)
Q_NEUTRAL = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


class FR3MuJoCoEnv:
    """
    MuJoCo simulation environment for the Franka FR3.

    Key design choices
    ------------------
    * Torques via qfrc_applied:  we bypass MuJoCo actuators so we can
      implement our own impedance law at any rate without re-tuning gains.
    * Bias decomposition: MuJoCo's qfrc_bias = C(q,dq)*dq + g(q).
      We expose the combined bias as dyn.Cq_dot and set dyn.g = 0.
      This is mathematically equivalent in the impedance law because the
      controller only uses  mu + p = Λ J M⁻¹ (C dq + g).
    * dJ: computed by finite differences between consecutive steps.
      The existing build_operational_space_model ignores dJ (uses np.zeros),
      so this value is provided for completeness but doesn't affect results.
    """

    def __init__(
        self,
        scene_xml: str | Path | None = None,
        ee_site:   str = EE_SITE_NAME,
        timestep:  float | None = None,
    ):
        """
        Parameters
        ----------
        scene_xml : path to the MJCF scene XML.
                    Defaults to simulation/models/fr3_scene.xml.
                    Run `python simulation/setup_model.py` first.
        ee_site   : name of the MuJoCo site used as end-effector frame.
        timestep  : override the XML timestep (seconds). None = use XML value.
        """
        xml = Path(scene_xml) if scene_xml else DEFAULT_SCENE
        if not xml.exists():
            raise FileNotFoundError(
                f"Scene XML not found: {xml}\n"
                f"Run first:  python simulation/setup_model.py"
            )

        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data  = mujoco.MjData(self.model)
        self.nv    = self.model.nv   # 7 for FR3

        if timestep is not None:
            self.model.opt.timestep = timestep

        # Resolve EE site
        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, ee_site
        )
        if self.ee_site_id == -1:
            avail = self._list_names(mujoco.mjtObj.mjOBJ_SITE, self.model.nsite)
            raise ValueError(
                f"Site '{ee_site}' not found in model.\n"
                f"Available sites: {avail}\n"
                f"Pass the correct name as ee_site=..."
            )
        self.ee_body_id = self.model.site_bodyid[self.ee_site_id]

        # Disable the model's built-in PD actuators so qfrc_applied is the
        # sole actuation path (matching the docstring's "bypass" claim).
        # The mujoco_menagerie FR3 has high-gain PD controllers (kp ≈ 4500)
        # that saturate at ±87 Nm and overwhelm any external torque control.
        self.model.actuator_gainprm[:, 0] = 0.0   # zero proportional gain
        self.model.actuator_biasprm[:, 1] = 0.0   # zero position feedback
        self.model.actuator_biasprm[:, 2] = 0.0   # zero velocity feedback

        # Pre-allocated scratch buffers
        self._jacp   = np.zeros((3, self.nv))
        self._jacr   = np.zeros((3, self.nv))
        self._M      = np.zeros((self.nv, self.nv))    # output of mj_fullM
        self._J_prev = np.zeros((6, self.nv))

        # Shadow MjData for hypothetical-state dynamics queries (used by the
        # reference-scheduled horizon torque constraint, ImpedanceMPCParams.
        # horizon_torque_schedule). Created lazily so environments that never
        # use it pay nothing extra; kept separate from self.data so querying
        # dynamics at a predicted future (q, dq) never touches the live
        # episode state.
        self._shadow_data = None
        self._shadow_M    = np.zeros((self.nv, self.nv))
        self._shadow_jacp = np.zeros((3, self.nv))
        self._shadow_jacr = np.zeros((3, self.nv))

        self.reset()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _list_names(self, obj_type, n: int) -> list[str]:
        return [mujoco.mj_id2name(self.model, obj_type, i) or f"<id={i}>"
                for i in range(n)]

    def _compute_jacobian(self) -> np.ndarray:
        """(6, nv) Jacobian [J_v; J_w] at the EE site, world frame."""
        self._jacp[:] = 0.0
        self._jacr[:] = 0.0
        mujoco.mj_jacSite(self.model, self.data,
                          self._jacp, self._jacr, self.ee_site_id)
        return np.vstack([self._jacp, self._jacr])

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self, q: np.ndarray | None = None, dq: np.ndarray | None = None):
        """Reset to neutral configuration (or provided q, dq)."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:self.nv] = q  if q  is not None else Q_NEUTRAL
        self.data.qvel[:self.nv] = dq if dq is not None else np.zeros(self.nv)
        mujoco.mj_forward(self.model, self.data)
        self._J_prev[:] = self._compute_jacobian()

    # ------------------------------------------------------------------
    # Dynamics extraction  →  FrankaDynamics + RobotState
    # ------------------------------------------------------------------

    def get_dynamics_and_state(
        self,
        f_ext_override: np.ndarray | None = None,
    ) -> tuple[FrankaDynamics, RobotState]:
        """
        Compute FrankaDynamics and RobotState at the current simulation state.

        Call after mj_forward / mj_step has been executed.

        Parameters
        ----------
        f_ext_override : (6,) [force; torque] to report in RobotState.f_ext.
                         Pass the known applied wrench here if you apply one
                         externally (avoids reading noisy contact forces).

        Returns
        -------
        (FrankaDynamics, RobotState) matching the fr3_impedance.py API.
        """
        # --- Full joint-space inertia matrix M (nv × nv) ---
        # MuJoCo <3.10: mj_fullM(model, dst, data.qM);  >=3.10: (model, data, dst)
        try:
            mujoco.mj_fullM(self.model, self._M, self.data.qM)
        except TypeError:
            mujoco.mj_fullM(self.model, self.data, self._M)
        M = self._M.copy()

        # --- Bias forces  C(q,dq)*dq + g(q)  ---
        # Exposed as Cq_dot (g set to zero so mu+p = Λ J M⁻¹ bias, correct).
        bias = self.data.qfrc_bias[:self.nv].copy()

        # --- Jacobian and finite-difference J̇ ---
        J  = self._compute_jacobian()
        dJ = (J - self._J_prev) / self.model.opt.timestep
        self._J_prev[:] = J

        dyn = FrankaDynamics(
            M=M,
            Cq_dot=bias,
            g=np.zeros(self.nv),
            J=J,
            dJ=dJ,
        )

        # --- EE kinematics ---
        ee_pos = self.data.site_xpos[self.ee_site_id].copy()
        # site_xmat is stored row-major (3×3 flattened)
        ee_rot = self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()
        q  = self.data.qpos[:self.nv].copy()
        dq = self.data.qvel[:self.nv].copy()
        ee_vel = (J @ dq).copy()   # [v; ω] twist, world frame

        # --- External wrench ---
        if f_ext_override is not None:
            f_ext = f_ext_override.copy()
        else:
            # cfrc_ext: MuJoCo contact forces on body, format [torque(3), force(3)]
            raw = self.data.cfrc_ext[self.ee_body_id]
            f_ext = np.concatenate([raw[3:], raw[:3]])   # reorder to [force, torque]

        state = RobotState(
            q=q, dq=dq,
            ee_pos=ee_pos, ee_rot=ee_rot,
            ee_vel=ee_vel,
            f_ext=f_ext,
        )
        return dyn, state

    # ------------------------------------------------------------------
    # Shadow dynamics query (hypothetical-state, does not touch live state)
    # ------------------------------------------------------------------

    def shadow_dynamics(
        self, q: np.ndarray, dq: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Kinematics/dynamics quantities at a HYPOTHETICAL (q, dq), without
        perturbing the live episode's self.data. Used by the
        reference-scheduled horizon torque constraint (impedance_mpc.py,
        horizon_torque_schedule) to evaluate J_v, τ_ff at scheduled future
        joint states — nominally along the redundancy-resolved reference
        trajectory q_d(t)/q̇_d(t) (see phri.precompute_joint_reference) —
        without stepping physics or corrupting the real trajectory being
        simulated.

        Returns
        -------
        J_v     : (3, nv) translational Jacobian at (q, dq)
        Lam_inv : (3, 3)  Λ⁻¹ = J_v M⁻¹ J_vᵀ (+ regularisation)
        Lam_pos : (3, 3)  Λ = Lam_inv⁻¹
        Cq_dot  : (nv,)   MuJoCo qfrc_bias = C(q,dq)q̇ + g(q)
        """
        if self._shadow_data is None:
            self._shadow_data = mujoco.MjData(self.model)
        d = self._shadow_data
        d.qpos[:self.nv] = q
        d.qvel[:self.nv] = dq
        mujoco.mj_forward(self.model, d)

        try:
            mujoco.mj_fullM(self.model, self._shadow_M, d.qM)
        except TypeError:
            mujoco.mj_fullM(self.model, d, self._shadow_M)
        M = self._shadow_M.copy()

        Cq_dot = d.qfrc_bias[:self.nv].copy()

        self._shadow_jacp[:] = 0.0
        self._shadow_jacr[:] = 0.0
        mujoco.mj_jacSite(self.model, d,
                          self._shadow_jacp, self._shadow_jacr, self.ee_site_id)
        J_v = self._shadow_jacp.copy()

        Lam_inv = J_v @ np.linalg.inv(M) @ J_v.T + 1e-6 * np.eye(3)
        Lam_pos = np.linalg.inv(Lam_inv)
        return J_v, Lam_inv, Lam_pos, Cq_dot

    def shadow_kinematics(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        EE position and translational Jacobian at a HYPOTHETICAL q, without
        the mass-matrix/bias/constraint work shadow_dynamics does. Uses
        mj_kinematics (position-level FK only) instead of mj_forward, so
        it's cheap enough to call at every step of the fine-resolution
        offline integration in phri.precompute_joint_reference (thousands
        of calls per episode) without materially slowing the precompute.

        Returns
        -------
        ee_pos : (3,)    EE position, world frame
        J_v    : (3, nv) translational Jacobian at q
        """
        if self._shadow_data is None:
            self._shadow_data = mujoco.MjData(self.model)
        d = self._shadow_data
        d.qpos[:self.nv] = q
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_comPos(self.model, d)   # cdof/subtree_com — mj_jacSite needs these
        ee_pos = d.site_xpos[self.ee_site_id].copy()

        self._shadow_jacp[:] = 0.0
        self._shadow_jacr[:] = 0.0
        mujoco.mj_jacSite(self.model, d,
                          self._shadow_jacp, self._shadow_jacr, self.ee_site_id)
        J_v = self._shadow_jacp.copy()
        return ee_pos, J_v

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------

    def null_space_gravity_comp(self, dyn: "FrankaDynamics") -> np.ndarray:
        """
        Return the null-space bias compensation torque: N̄ᵀ @ bias.

        The task-space impedance law handles the task-space projection of bias:
            J^T F_c (bias part) = (I − N̄ᵀ) @ bias

        The remaining null-space gravity/Coriolis component N̄ᵀ @ bias is not
        handled by the controller and must be added explicitly.  Without it the
        robot drifts in the null-space direction under gravity.

        Add this to the torque output of cartesian_impedance_control():
            tau_total = tau_ctrl + env.null_space_gravity_comp(dyn)
        """
        from fr3_impedance import build_operational_space_model
        ee_vel = dyn.J @ self.data.qvel[:self.nv]
        os = build_operational_space_model(dyn, ee_vel)
        return os.N_bar.T @ dyn.Cq_dot   # N̄ᵀ @ (C dq + g)

    def apply_torque(self, tau: np.ndarray):
        """
        Apply joint torques [Nm] for the next simulation step.

        Torques are written to qfrc_applied.  The built-in PD actuators are
        disabled at init, so qfrc_applied is the sole actuation path.
        Clipped to FR3 joint torque limits before application.
        """
        clipped = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        self.data.qfrc_applied[:self.nv] += clipped
        # Mirror into data.ctrl so the viewer's sensor panel shows live torques.
        # The actuator gains are zeroed at init so ctrl has no effect on physics.
        self.data.ctrl[:self.nv] = clipped

    def apply_ee_wrench(self, wrench: np.ndarray):
        """
        Apply a 6D wrench [force(3); torque(3)] at the EE site, world frame.

        Converted to an equivalent force+torque at the EE body COM via
        the lever arm from the body COM to the site:
            τ_com += (p_site − p_com) × force

        Parameters
        ----------
        wrench : (6,) [fx, fy, fz, tx, ty, tz] in world frame
        """
        force  = wrench[:3]
        torque = wrench[3:]

        body_pos = self.data.xpos[self.ee_body_id]
        site_pos = self.data.site_xpos[self.ee_site_id]
        r = site_pos - body_pos                          # lever arm

        # xfrc_applied: [force(3), torque(3)] in world frame, applied at body COM
        self.data.xfrc_applied[self.ee_body_id, :3] += force
        self.data.xfrc_applied[self.ee_body_id, 3:] += torque + np.cross(r, force)

    def clear_applied_forces(self):
        """Clear all qfrc_applied and xfrc_applied for the next step."""
        self.data.qfrc_applied[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

    # ------------------------------------------------------------------
    # Simulation step
    # ------------------------------------------------------------------

    def step(self):
        """
        Advance the simulation by one timestep (model.opt.timestep).

        Call apply_torque() and/or apply_ee_wrench() before this.
        Applied forces are cleared automatically after the step.
        """
        mujoco.mj_step(self.model, self.data)
        self.clear_applied_forces()

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def dt(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def q(self) -> np.ndarray:
        return self.data.qpos[:self.nv].copy()

    @property
    def dq(self) -> np.ndarray:
        return self.data.qvel[:self.nv].copy()

    def update_target_site(self, pos: np.ndarray, site_name: str = "target_pos"):
        """Move a visual marker site to track the current target position."""
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            self.model.site_pos[sid] = pos
