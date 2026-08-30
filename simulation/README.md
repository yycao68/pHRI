# FR3 MuJoCo Simulation

MuJoCo-based simulation of the Franka FR3 robot arm. Bridges the Cartesian impedance/admittance controller in `fr3_impedance.py` to full rigid-body physics, with a live 7-panel visualization dashboard (including a 3D EE trajectory view), a virtual human interaction model, and an RL-compatible environment.

These scripts are the paper's simulation/demo layer. The predictive-controller
scripts use the current linear double-integrator residual backbone with the
corrected offset-free input centering. They are useful for benchmark figures,
controller comparison videos, and pHRI demonstrations.

Certified sampled joint-limit safety and energy-tank passivity are verified in
the hardware-path MuJoCo suite under `../cloud_verify/verification/`, which
runs the same `FR3ImpedanceMPCHardwareInterface` used for real FR3 tests. Use
that suite for claims involving CBF/passivity certificates; use this folder for
algorithmic MuJoCo demos and comparison plots.

---

## Prerequisites

```bash
pip install -r simulation/requirements.txt
```

Installs mujoco, numpy, scipy, osqp, matplotlib, and pytest -- the default
solver path (`impedance_mpc.py`) uses OSQP with a SciPy fallback, and the
`test_*.py` files need pytest, none of which the previous `pip install
mujoco numpy matplotlib` instruction here actually installed. See
`requirements.txt` for exact versions this environment last verified
against, and the Python/MuJoCo versions used.

Git must be available on `PATH` (used by `setup_model.py` for sparse checkout of the robot model).

---

## Quick Start

Run all commands from the `fr3_impedance/` parent directory.

**1 — Download the FR3 model (once):**

```bash
python3 simulation/setup_model.py
```

Clones only `franka_fr3/` from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) (~30 MB) via git sparse checkout and writes `simulation/models/franka_fr3/fr3_phri_scene.xml`.

**2 — Verify the setup:**

```bash
python3 simulation/run_simulation.py --mode data_check
```

Expected output:
```
[data_check] FR3 MuJoCo environment loaded successfully.
  nv          = 7
  dt          = 2.0 ms
  EE pos      = [0.307  0.     0.59 ]
  M diag      = [0.6936 1.7207 1.1185 1.029  0.1015 0.1045 0.0741]
  J shape     = (6, 7)
  bias[:7]    = [ 0.  -1.721 -0.639 18.959  0.792  1.588  0. ]
```

**3 — Run a demo:**

```bash
python3 simulation/run_simulation.py --mode impedance
python3 simulation/run_simulation.py --mode phri
python3 simulation/run_simulation.py --mode rl_random
```

### Command-line flags

| Flag | Effect |
|------|--------|
| `--mode` | `impedance` / `phri` / `rl_random` / `data_check` |
| `--no-viewer` | Disable the MuJoCo 3-D robot viewer |
| `--no-plot` | Disable the live matplotlib dashboard |

Each run auto-saves a dashboard PNG to the working directory:

| Mode | Saved file |
|------|-----------|
| `impedance` | `impedance_dashboard.png` |
| `phri` | `phri_dashboard.png` |
| `rl_random` | `rl_random_dashboard.png` |

---

## Simulation Modes

### `impedance`

The robot starts at the neutral joint configuration `Q_NEUTRAL = [0, −π/4, 0, −3π/4, 0, π/2, π/4]` rad, which places the EE at `[0.307, 0.0, 0.590]` m. The Cartesian impedance controller then drives it to the fixed target `x_d = [0.5, 0.0, 0.4]` m (a displacement of ~0.27 m).

**Closed-loop impedance parameters** (`make_impedance_params(k_pos=300, k_rot=20, damping_ratio=1.0)`):

| | Translational | Rotational |
|---|---|---|
| **K** | `300 · I₃` N/m | `20 · I₃` Nm/rad |
| **D** | `2√300 · I₃ ≈ 34.6 · I₃` Ns/m | `2√20 · I₃ ≈ 8.9 · I₃` Ns/rad |
| **M_d** | `Λ` (operational-space inertia) | same |

`M_d = Λ = (J M⁻¹ Jᵀ)⁻¹` (inertia shaping) reduces the closed-loop task-space dynamics to `Λ ẍ_e + D ẋ_e + K x_e = f_ext` with critical damping (ζ = 1) on all axes. The null-space spring is anchored at `Q_NEUTRAL`.

> **Why k_pos = 300 N/m?**
> 300 N/m is mid-range for the FR3 in Cartesian impedance mode (typical operating range: 200–600 N/m). With task-space inertia Λ ≈ 0.7–1.7 kg at the neutral configuration, this gives a natural frequency of ~13–21 rad/s and a settling time of ~0.3–0.5 s per axis — fast enough to converge within the 5 s demo without being aggressive. At dt = 2 ms the stability margin is comfortable (`K · dt² / m_eff ≈ 0.001 ≪ 1`). A deliberately distinct value from the pHRI mode (250 N/m) so the two modes' responses are visually distinguishable.

**Observed behaviour:**
- Position error: 0.271 m → 0.026 m at t = 1 s → ~0.010 m steady-state
- EE trajectory visible as a smooth 3-D arc in the dashboard
- Large torque transient at t = 0 settles within ~1 s

### `phri`

Same starting configuration as `impedance` — `Q_NEUTRAL`, EE at `[0.307, 0.0, 0.590]` m — but the nominal target is `x_d = [0.45, 0.0, 0.45]` m. Two controllers run in series:

**1 — Inner: Cartesian impedance** (`make_impedance_params(k_pos=250, k_rot=20, damping_ratio=1.0)`):

| | Translational | Rotational |
|---|---|---|
| **K** | `250 · I₃` N/m | `20 · I₃` Nm/rad |
| **D** | `2√250 · I₃ ≈ 31.6 · I₃` Ns/m | `2√20 · I₃ ≈ 8.9 · I₃` Ns/rad |
| **M_d** | `Λ` (operational-space inertia) | same |

**2 — Outer: admittance filter** (`make_admittance_params(m_pos=1.0, d_pos=80.0, k_pos=0.0)`):

`M_a ẍ_r + D_a ẋ_r = f_human` → modifies the tracking reference to `x_cmd = x_d + x_r`

| M_a | D_a | K_a |
|---|---|---|
| `1.0 · I₃` kg | `80 · I₃` Ns/m | `0` (no restoring spring) |

`K_a = 0` means the robot holds any displaced position after the human releases — it does not spring back. Setting `K_a > 0` would instead cause the robot to return to `x_d` after contact ends.

**Virtual human arm** (active t = 1–3 s, anchor at `[0.45, 0.10, 0.45]` m):

| K_h | D_h | Push |
|---|---|---|
| `80 · I₃` N/m | `15 · I₃` Ns/m | +Y, 10 cm offset from `x_d` |

> **Why K_h = 80 N/m, D_h = 15 Ns/m?**
> These values are grounded in biomechanics measurements of human arm endpoint impedance (Mussa-Ivaldi et al. 1985; Tsuji et al. 1995). 80 N/m represents a relaxed, low-activation push (measured range: ~40–200 N/m); with the 10 cm anchor offset it produces a peak contact force of `K_h × 0.10 = 8 N` — realistic for a light touch and clearly detectable by CUSUM. D_h = 15 Ns/m is mid-range for viscous arm damping (~5–30 Ns/m), giving a slightly underdamped interaction (ζ ≈ 0.8 at ~1 kg effective arm mass) consistent with real contact dynamics. The isotropic diagonal is a simplification; real arm stiffness is anisotropic but only the Y-axis matters for this demo.

**Observed behaviour:**
- Contact force peaks at ~8 N at t = 1 s, decays as the robot yields
- EE Y-displacement: ~7.7 cm at steady state during push
- CUSUM detector flags contact within one step; score rises through the push window
- 3-D trajectory shows the Y-kink clearly during the interaction

### `rl_random`

Exercises the full `PHRIEnv` RL interface with random actions over two short episodes. Confirms that the 36-dim observation, 3-dim action, scalar reward, and done flag are all returned correctly.

**Task setup:**
- Target: `x_d = [0.45, 0.05, 0.45]` m; same `Q_NEUTRAL` start as other modes
- 2 episodes × up to 50 RL steps; fixed seed (42) for reproducibility
- Each RL step spans `N = 10` inner simulation steps → **20 ms per RL decision** at dt = 2 ms

**RL interface:**

| | Detail |
|---|---|
| Observation `s_t` | 36-dim: `[f_ext(6), ẋ(6), x−x_d(6), q−q_mid(7), q̇(7), c(1), params(3)]` |
| Action `a_t` | 3-dim: `[ΔK_d, ΔD_d, Δk_n]` — impedance parameter deltas |
| Action bounds | `±[20 N/m, 10 Ns/m, 1.0 Nm/rad]` per step |
| Parameter bounds | K_d ∈ [0, 600] N/m · D_d ∈ [0, 200] Ns/m · k_n ∈ [0, 10] Nm/rad |
| Reward | `−0.10 ‖τ_wrist‖² − 1.00 ‖x−x_d‖² − 0.50 ‖f_ext‖² 1[c=0] − 0.01 ‖Δa‖²` |
| Termination | `‖x − x_d‖ > 0.5 m` (diverged) |

Impedance parameters are reset to `(K_d, D_d, k_n) = (200, 28.3, 2.0)` (critically damped) at the start of each episode.

**Domain randomisation (per episode):**

Human arm parameters are re-sampled each `reset()` call:

| Parameter | Range |
|---|---|
| K_h (stiffness) | `diag(U[50, 500])` N/m |
| D_h (damping) | `diag(U[5, 50])` Ns/m |
| m_tool (tool mass) | `U[0.2, 2.0]` kg |
| anchor offset | `U[−0.05, 0.05]³` m from `x_d` |

This tests whether the RL interface handles varying human dynamics without crashing — a prerequisite before training a real policy.

**Safety limits active during inner loop:**
- EE speed: torques scaled by `V_limit / ‖ẋ‖` if `‖ẋ‖ > 0.05 m/s` (ISO/TS 15066)
- Human force: saturated at `F_limit` (ISO/TS 15066 hand-contact limit) inside `VirtualHumanArm`

### `data_check`

Prints a one-shot dynamics snapshot at t = 0 (no viewer, no plot). Useful for confirming the model loaded correctly and all dynamics quantities are finite before running a full simulation.

**What each field means:**

| Field | Value | Interpretation |
|---|---|---|
| `nv` | 7 | 7-DOF robot confirmed |
| `dt` | 2.0 ms | Simulation timestep (`model.opt.timestep`) |
| `EE site id` | ≥ 0 | MuJoCo site ID found; −1 would mean the XML site name is wrong |
| `EE pos` | `[0.307, 0., 0.590]` | Forward kinematics at `Q_NEUTRAL` |
| `q (neutral)` | `[0, −0.785, 0, −2.356, 0, 1.571, 0.785]` | `Q_NEUTRAL` in radians = `[0, −π/4, 0, −3π/4, 0, π/2, π/4]` |
| `M diag` | `[0.69, 1.72, 1.12, 1.03, 0.10, 0.10, 0.07]` | Joint-space inertia diagonal at `Q_NEUTRAL`; decreases distally as expected |
| `J shape` | `(6, 7)` | Geometric Jacobian: 3 translational + 3 rotational rows, 7 joint columns |
| `bias[:7]` | `[0, −1.72, −0.64, 18.96, 0.79, 1.59, 0]` | `C(q,q̇)q̇ + g(q)` at `q̇ = 0`, so this is pure gravity torque `g(q)` |

> **Why is bias[3] so large (≈ 19 Nm)?**
> Joint 4 is the elbow. At `Q_NEUTRAL` (`q₄ = −3π/4`) the forearm hangs nearly horizontal, placing the largest gravitational moment on this joint. This is precisely why the null-space spring must target `Q_NEUTRAL` rather than the zero configuration — at `q = 0` the gravity load at joint 4 exceeds controller authority and destabilises the arm (see Design Notes below).

---

## Live Dashboard

`SimVisualizer` (`visualizer.py`) opens a dark-themed dashboard that updates in real time. It has a 3-D EE trajectory panel on the left and six time-series panels on the right.

```
┌─────────────────┬──────────────────────┬──────────────────────┐
│                 │  Position error      │  EE position         │
│  EE trajectory  │  |x − x_d|  (m)      │  x / y / z  (m)      │
│  3-D view       │                      │  dashed = target     │
│                 ├──────────────────────┼──────────────────────┤
│  ○ start        │  Contact force       │  Joint torques       │
│  ● EE now       │  |f_ext|  (N)        │  τ₁ … τ₇  (Nm)       │
│  ★ target       │                      │                      │
│                 ├──────────────────────┼──────────────────────┤
│  faded trail    │  EE speed            │ Mode-specific        │
│  bright recent  │  |ẋ|  (m/s)          │ impedance → q̇        │
│                 │                      │ phri   → CUSUM Sₖ    │
│                 │                      │ rl_random → Kd/Dd/kn │
└─────────────────┴──────────────────────┴──────────────────────┘
```

**3-D trajectory features:**
- Full history drawn as a faded blue trail; the most recent 200 steps are bright
- Hollow circle = start position, filled dot = current EE, yellow star = target
- Dashed vertical reference line dropped from target to the floor plane
- Axes auto-scale to always contain both trajectory and target

The matplotlib dashboard and the MuJoCo 3-D robot viewer run concurrently and are independent.

---

## File Overview

| File | Purpose |
|------|---------|
| `setup_model.py` | Downloads the MJCF model; writes the scene XML |
| `fr3_impedance.py` | Core controller: Cartesian impedance, admittance, operational-space model |
| `impedance_mpc.py` | Double-integrator predictive controller; filename retained for backward compatibility |
| `fr3_benchmark_verification.py` | Lightweight analytical FR3 benchmark with offset-free disturbance centering |
| `so3_utils.py` | SO(3) utilities: rotation error, quaternion conversion |
| `fr3_mujoco.py` | `FR3MuJoCoEnv` — MuJoCo wrapper exposing `FrankaDynamics` / `RobotState` |
| `phri_env.py` | `PHRIEnv` — RL env with virtual human arm and CUSUM contact detector |
| `visualizer.py` | `SimVisualizer` — live 3-D + time-series dashboard |
| `run_simulation.py` | Entry point for all four simulation modes |
| `models/franka_fr3/` | MJCF model files (created by `setup_model.py`) |

---

## Architecture

```
run_simulation.py
    │
    ├── FR3MuJoCoEnv  (fr3_mujoco.py)
    │       ├── MuJoCo physics  mj_step / qfrc_applied
    │       ├── get_dynamics_and_state()  →  FrankaDynamics, RobotState
    │       └── null_space_gravity_comp()  →  N̄ᵀ (C q̇ + g)
    │
    ├── cartesian_impedance_control()  (fr3_impedance.py)
    │       └── τ = Jᵀ Fc + N̄ᵀ τ_null
    │
    ├── PHRIEnv  (phri_env.py)
    │       ├── VirtualHumanArm   spring-damper wrench at EE
    │       ├── CUSUMDetector     contact-onset detection
    │       └── RL interface      obs (36-dim) · action (3-dim) · reward
    │
    └── SimVisualizer  (visualizer.py)
            ├── Axes3D  EE trajectory (3-D, left column)
            └── 6 × time-series panels (right 2 columns) → PNG on exit
```

---

## Design Notes

### Actuator bypass

The mujoco_menagerie FR3 model ships with high-gain PD position actuators (kp ≈ 4500 Nm/rad, kd ≈ 450 Nm·s/rad) that saturate at ±87 Nm when `ctrl = 0`. At the neutral configuration these actuators generate forces large enough to overwhelm any external torque controller. `FR3MuJoCoEnv.__init__` zeros all actuator gains so that `qfrc_applied` is the sole actuation path:

```python
self.model.actuator_gainprm[:, 0] = 0.0   # zero position gain
self.model.actuator_biasprm[:, 1] = 0.0   # zero position feedback
self.model.actuator_biasprm[:, 2] = 0.0   # zero velocity feedback
```

### Bias decomposition

`data.qfrc_bias = C(q, q̇) q̇ + g(q)`. The wrapper exposes the combined bias as `FrankaDynamics.Cq_dot` with `g = 0`. The impedance law uses `μ + p = Λ J M⁻¹ bias`, which is mathematically equivalent to the standard decomposition.

### Null-space gravity compensation

`cartesian_impedance_control()` compensates gravity only in the task-space direction via `Jᵀ Fc`. The orthogonal null-space component `N̄ᵀ bias` is not covered and must be added explicitly, otherwise the robot drifts under gravity in the redundant DOF:

```python
tau  = cartesian_impedance_control(state, dyn, x_d, R_d, dx_d, ddx_d, params)
tau += env.null_space_gravity_comp(dyn)   # N̄ᵀ (C q̇ + g)
env.apply_torque(tau)
```

### Null-space posture target

The null-space spring (`k_null`, `q_null` in `ImpedanceParams`) must target the neutral configuration, not the default zero configuration. At zero, the spring generates saturated destabilising torques (up to 23 Nm at joint 4):

```python
params = make_impedance_params(..., q_null=Q_NEUTRAL)
```

---

## pHRI RL Environment

| Property | Value |
|----------|-------|
| Observation `s_t` | 36-dim: `[f_ext(6), ẋ(6), x−x_d(6), q−q_mid(7), q̇(7), c(1), params(3)]` |
| Action `a_t` | 3-dim: `[ΔK_d, ΔD_d, Δk_n]` — impedance parameter deltas per RL step |
| Parameter bounds | K_d ∈ [0, 600] N/m · D_d ∈ [0, 200] N·s/m · k_n ∈ [0, 10] Nm/rad |
| Reward | −w₁‖τ_wrist‖² − w₂‖x−x_d‖² − w₃‖f_ext‖²1[c=0] − w₄‖Δa‖² |
| Reward weights | w₁ = 0.10 · w₂ = 1.00 · w₃ = 0.50 · w₄ = 0.01 |
| Contact detector | CUSUM: S_k = max(0, S_{k−1} + ‖f_ext‖ − μ₀ − κ), flag when S_k > h |
| Inner loop | 10 impedance-control steps per RL decision (20 ms per action at 2 ms dt) |
| Safety | EE torques scaled down if `|ẋ| > 0.05 m/s` (ISO/TS 15066) |
