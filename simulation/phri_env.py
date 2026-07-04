"""
pHRI simulation environment.

Wraps FR3MuJoCoEnv with three additions:
  1. Virtual human arm  — spring-damper force applied at the EE site
  2. CUSUM detector     — optimal contact-onset detection (§10.2)
  3. RL interface       — state / action / reward matching §10.3 of pHRI_control.md

State  s_t ∈ ℝ³⁶ = [f_ext(6), ẋ(6), x−x_d(6), q−q_mid(7), q̇(7), c(1), params(3)]
Action a_t ∈ ℝ³  = [ΔK_d, ΔD_d, Δk_n]
Reward             = ergonomic + task + phantom-force + smooth-transition terms
"""

from __future__ import annotations
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from fr3_impedance import (
    make_impedance_params, cartesian_impedance_control,
    RobotState,
)
from fr3_mujoco import FR3MuJoCoEnv, Q_NEUTRAL

# FR3 joint midpoints  (used to centre the q−q_mid term in the RL state)
Q_MID = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 3.05, 0.785])

# Safety limits (ISO/TS 15066)
F_LIMIT = 140.0   # N  — hand/finger
V_LIMIT = 0.05    # m/s — human reaction time boundary


# ---------------------------------------------------------------------------
# 1. Virtual Human Arm
# ---------------------------------------------------------------------------

@dataclass
class HumanArmParams:
    """Biomechanical parameters for the virtual human arm (§2, §10.6)."""
    K_h: np.ndarray = field(
        default_factory=lambda: np.diag([200., 200., 200.]))   # N/m
    D_h: np.ndarray = field(
        default_factory=lambda: np.diag([30.,  30.,  30.]))    # N·s/m
    M_h: np.ndarray = field(
        default_factory=lambda: np.diag([2.,   2.,   2.]))     # kg (unused — pure spring-damper)
    m_tool: float   = 1.0     # kg — tool mass for gravity compensation
    # Forearm length for wrist position estimate (§9.8, option 1)
    L_forearm: float = 0.30   # m


class VirtualHumanArm:
    """
    Second-order spring-damper model of the human arm.

    Computes the 6D wrench the human exerts on the shared tool:

        f_human = K_h (p_anchor − p_ee) + D_h (v_anchor − v_ee)

    where p_anchor is the human's intended grip point (set via set_anchor).
    The wrench is applied to the EE site by PHRIEnv before each step.
    """

    def __init__(self, params: HumanArmParams):
        self.p = params
        self.anchor   = np.zeros(3)    # desired grip point (world frame)
        self.v_anchor = np.zeros(3)    # anchor velocity

    def set_anchor(self, pos: np.ndarray, vel: np.ndarray | None = None):
        self.anchor[:]   = pos
        self.v_anchor[:] = vel if vel is not None else np.zeros(3)

    def compute_wrench(
        self, ee_pos: np.ndarray, ee_vel_lin: np.ndarray
    ) -> np.ndarray:
        """
        Return the 6D wrench [force(3); torque(3)] the human applies.

        Torque component is zero (pure translational spring-damper).
        """
        f = (self.p.K_h @ (self.anchor - ee_pos)
             + self.p.D_h @ (self.v_anchor - ee_vel_lin))
        # Saturate to ISO/TS 15066 hand force limit
        f_norm = np.linalg.norm(f)
        if f_norm > F_LIMIT:
            f = f * (F_LIMIT / f_norm)
        return np.concatenate([f, np.zeros(3)])

    def randomize(self, rng: np.random.Generator):
        """Domain-randomise arm parameters (§10.6 table)."""
        self.p.K_h    = np.diag(rng.uniform(50,  500, 3))
        self.p.D_h    = np.diag(rng.uniform(5,   50,  3))
        self.p.m_tool = float(rng.uniform(0.2, 2.0))


# ---------------------------------------------------------------------------
# 2. CUSUM Contact Detector
# ---------------------------------------------------------------------------

class CUSUMDetector:
    """
    Cumulative Sum change-point detector (§10.2 of pHRI_control.md).

        S_k = max(0, S_{k−1} + ||f_ext,k|| − μ₀ − κ)

    Raises contact flag when S_k > h.
    """

    def __init__(self, mu0: float = 0.5, kappa: float = 0.3, h: float = 2.0,
                 S_max: float | None = None):
        """
        Parameters
        ----------
        mu0   : baseline force norm in free space (N)
        kappa : slack — sensitivity vs. false-alarm tradeoff (N)
        h     : alarm threshold (N)
        S_max : cap on the cumulative statistic (defaults to 3·h).  Bounds how
                long the contact flag takes to clear after a strong/long push.
        """
        self.mu0   = mu0
        self.kappa = kappa
        self.h     = h
        self.S_max = S_max if S_max is not None else 3.0 * h
        self.S       = 0.0
        self.contact = False

    def update(self, f_norm: float) -> bool:
        # Proper one-sided CUSUM: accumulate the excess of the force norm over
        # baseline+slack.  The max(0, …) floor lets S decay back down once the
        # force returns to baseline.  (Previously S was zeroed every step below
        # the alarm threshold, which destroyed the accumulation and reduced the
        # detector to instantaneous thresholding at f_norm > mu0 + kappa + h.)
        self.S = max(0.0, self.S + f_norm - self.mu0 - self.kappa)
        # Cap the statistic so the flag clears promptly after a strong/long
        # contact instead of taking thousands of steps to decay.
        self.S = min(self.S, self.S_max)
        self.contact = self.S > self.h
        return self.contact

    def reset(self):
        self.S = 0.0
        self.contact = False


# ---------------------------------------------------------------------------
# 3. pHRI Environment (RL interface)
# ---------------------------------------------------------------------------

class PHRIEnv:
    """
    pHRI MuJoCo environment with RL-compatible step / reset interface.

    Architecture (§10.1):
        Outer loop  — RL agent (this class), updates impedance parameters.
        Inner loop  — Classical Cartesian impedance (cartesian_impedance_control),
                      runs every simulation timestep at 1/dt Hz.

    The RL agent outputs parameter *deltas* [ΔK_d, ΔD_d, Δk_n] per call to
    step(); the inner-loop controller always runs with the most recent params.
    """

    # Impedance parameter bounds
    KD_BOUNDS = (0.0,   600.0)   # N/m
    DD_BOUNDS = (0.0,   200.0)   # N·s/m
    KN_BOUNDS = (0.0,    10.0)   # Nm/rad

    # Reward weights (§10.3)
    W_ERGO   = 0.10
    W_TASK   = 1.00
    W_GHOST  = 0.50
    W_SMOOTH = 0.01

    def __init__(
        self,
        scene_xml:       str | Path | None = None,
        human_params:    HumanArmParams | None = None,
        cusum_mu0:       float = 0.5,
        cusum_kappa:     float = 0.3,
        cusum_h:         float = 2.0,
        steps_per_action: int  = 10,    # inner-loop steps per RL action
    ):
        """
        Parameters
        ----------
        scene_xml        : passed to FR3MuJoCoEnv
        human_params     : virtual arm parameters (randomised in reset if None)
        steps_per_action : how many 1-ms simulation steps per RL decision
        """
        self.env = FR3MuJoCoEnv(scene_xml=scene_xml)
        self.dt_sim = self.env.dt
        self.dt     = self.env.dt
        self.N      = steps_per_action

        self.human  = VirtualHumanArm(human_params or HumanArmParams())
        self.cusum  = CUSUMDetector(mu0=cusum_mu0, kappa=cusum_kappa, h=cusum_h)

        # Current impedance state (maintained across steps)
        self.Kd: float = 200.0
        self.Dd: float = 2.0 * np.sqrt(self.Kd)
        self.kn: float = 2.0

        # Task target
        self.x_d  = np.array([0.4, 0.0, 0.5])
        self.R_d  = np.eye(3)
        self.dx_d  = np.zeros(6)
        self.ddx_d = np.zeros(6)

        self._prev_action = np.zeros(3)
        self._last_f_ext  = np.zeros(6)

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(
        self,
        x_d:              np.ndarray | None = None,
        randomize_human:  bool = True,
        rng:              np.random.Generator | None = None,
    ) -> np.ndarray:
        """
        Reset the environment and return the initial observation (36-dim).

        Parameters
        ----------
        x_d             : desired EE target position (world frame)
        randomize_human : domain-randomise human arm parameters each episode
        rng             : random generator for reproducibility
        """
        if rng is None:
            rng = np.random.default_rng()

        self.env.reset()
        self.cusum.reset()
        self._prev_action[:] = 0.0
        self._last_f_ext[:]  = 0.0

        self.Kd = 200.0
        self.Dd = 2.0 * np.sqrt(self.Kd)
        self.kn = 2.0

        if x_d is not None:
            self.x_d = np.asarray(x_d, dtype=float)

        if randomize_human:
            self.human.randomize(rng)

        # Human grips near the target with a small random offset
        anchor = self.x_d + rng.uniform(-0.05, 0.05, 3)
        self.human.set_anchor(anchor)

        self.env.update_target_site(self.x_d)

        _, state = self.env.get_dynamics_and_state()
        return self._build_obs(state, np.zeros(6))

    # ------------------------------------------------------------------
    # RL step
    # ------------------------------------------------------------------

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, dict]:
        """
        Execute one RL decision step.

        The RL action updates the impedance parameters once; the inner-loop
        impedance controller then runs for `steps_per_action` simulation
        steps at 1/dt Hz.

        Parameters
        ----------
        action : (3,) [ΔK_d, ΔD_d, Δk_n]

        Returns
        -------
        obs    : (36,) next observation
        reward : scalar reward
        done   : True on large position error (episode failure)
        info   : dict with diagnostics
        """
        # --- Update impedance params from RL action ---
        dKd, dDd, dkn = action
        self.Kd = float(np.clip(self.Kd + dKd, *self.KD_BOUNDS))
        self.Dd = float(np.clip(self.Dd + dDd, *self.DD_BOUNDS))
        self.kn = float(np.clip(self.kn + dkn, *self.KN_BOUNDS))

        # Build impedance params
        params = make_impedance_params(
            k_pos=self.Kd, k_rot=20.0,
            k_null=self.kn,
            d_null=float(np.clip(2.0 * np.sqrt(self.kn), 0.1, 5.0)),
            q_null=Q_NEUTRAL,
        )
        # Override positional damping with RL-controlled Dd
        params.D[:3, :3] = np.eye(3) * self.Dd

        # --- Inner loop: N simulation steps ---
        cumulative_f_ext = np.zeros(6)

        for _ in range(self.N):
            dyn, state = self.env.get_dynamics_and_state()

            # Human wrench at EE
            f_human = self.human.compute_wrench(state.ee_pos, state.ee_vel[:3])
            self.env.apply_ee_wrench(f_human)

            # CUSUM update with human force magnitude
            f_norm = float(np.linalg.norm(f_human[:3]))
            self.cusum.update(f_norm)
            cumulative_f_ext += f_human

            # Impedance controller (uses f_human as f_ext for feed-forward)
            state_with_ext = RobotState(
                q=state.q, dq=state.dq,
                ee_pos=state.ee_pos, ee_rot=state.ee_rot,
                ee_vel=state.ee_vel,
                f_ext=f_human,
            )
            tau = cartesian_impedance_control(
                state_with_ext, dyn,
                self.x_d, self.R_d, self.dx_d, self.ddx_d,
                params, use_f_ext=True,
            )
            tau += self.env.null_space_gravity_comp(dyn)

            # EE velocity safety: scale torques if EE moves too fast
            ee_speed = float(np.linalg.norm(state.ee_vel[:3]))
            if ee_speed > V_LIMIT:
                tau = tau * (V_LIMIT / ee_speed)

            self.env.apply_torque(tau)
            self.env.step()

        # Average external force over the N inner steps
        avg_f_ext = cumulative_f_ext / self.N
        self._last_f_ext[:] = avg_f_ext

        # --- Observation and reward ---
        _, state_next = self.env.get_dynamics_and_state(f_ext_override=avg_f_ext)
        obs    = self._build_obs(state_next, avg_f_ext)
        reward = self._compute_reward(state_next, avg_f_ext, action)

        # Termination: large position error
        pos_err = float(np.linalg.norm(state_next.ee_pos - self.x_d))
        done    = pos_err > 0.5   # 50 cm — clearly diverged

        info = {
            "pos_error_m":  pos_err,
            "contact":      self.cusum.contact,
            "f_human_N":    float(np.linalg.norm(avg_f_ext[:3])),
            "ee_speed_m_s": float(np.linalg.norm(state_next.ee_vel[:3])),
            "Kd": self.Kd, "Dd": self.Dd, "kn": self.kn,
        }

        self._prev_action[:] = action
        return obs, reward, done, info

    # ------------------------------------------------------------------
    # RL state construction (§10.3)
    # ------------------------------------------------------------------

    def _build_obs(self, state: RobotState, f_ext: np.ndarray) -> np.ndarray:
        """
        36-dim observation:
            [f_ext(6), ẋ(6), x−x_d(6), q−q_mid(7), q̇(7), c(1), params(3)]
        """
        pos_error = np.concatenate([
            state.ee_pos - self.x_d,    # (3,) translational
            np.zeros(3),                 # (3,) rotational (simplified)
        ])
        contact = float(self.cusum.contact)
        params  = np.array([self.Kd, self.Dd, self.kn])

        return np.concatenate([
            f_ext,                   # 6
            state.ee_vel,            # 6
            pos_error,               # 6
            state.q  - Q_MID,        # 7
            state.dq,                # 7
            [contact],               # 1
            params,                  # 3
        ])                           # = 36

    # ------------------------------------------------------------------
    # Reward (§10.3)
    # ------------------------------------------------------------------

    def _compute_reward(
        self,
        state:  RobotState,
        f_ext:  np.ndarray,
        action: np.ndarray,
    ) -> float:
        """
        r = −w1 ||τ_wrist||² − w2 ||x−x_d||² − w3 ||f_ext||² 1[c=0] − w4 ||Δa||²
        """
        # Ergonomic cost: predicted wrist torque (lever arm from wrist to EE).
        # Wrist ≈ EE minus the forearm vector along the EE's local x-axis, so
        # the lever arm rotates with the tool rather than being pinned to world
        # +x (the previous fixed [L,0,0] made this term a state-independent
        # constant that contributed nothing to the reward gradient).
        forearm_dir = state.ee_rot[:, 0]
        lever       = forearm_dir * self.human.p.L_forearm   # EE → wrist
        F_g         = np.array([0., 0., -self.human.p.m_tool * 9.81])
        tau_wrist   = np.cross(lever, F_g)
        ergo      = -self.W_ERGO * float(np.dot(tau_wrist, tau_wrist))

        # Task accuracy
        pos_err = float(np.linalg.norm(state.ee_pos - self.x_d))
        task    = -self.W_TASK * pos_err ** 2

        # Phantom force: penalise large f_ext when not in contact
        ghost = (-self.W_GHOST * float(np.dot(f_ext, f_ext))
                 * float(not self.cusum.contact))

        # Smooth transitions
        da     = action - self._prev_action
        smooth = -self.W_SMOOTH * float(np.dot(da, da))

        return ergo + task + ghost + smooth

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def obs_dim(self) -> int:
        return 36

    @property
    def act_dim(self) -> int:
        return 3

    @property
    def act_bounds(self) -> np.ndarray:
        """(2, 3) lower/upper bounds for [ΔK_d, ΔD_d, Δk_n]."""
        step = np.array([20., 10., 1.0])   # max delta per RL step
        return np.array([-step, step])
