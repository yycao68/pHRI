# OpenManipulator-X pHRI Torque-Level Verification

Real-robot verification of the pHRI interaction-dynamics controller on a ROBOTIS
**OpenManipulator-X (RM-X52-TNM)** — a 4-DOF arm built from **XM430-W350**
servos, every joint of which supports **Current Control Mode** (Operating_Mode 0,
`Goal_Current`). This arm lets us validate the paper's **torque-level** claims
(offset-free regulation, passivity, CBF safety).

Torque control here goes through the **DYNAMIXEL SDK / Protocol 2.0**, not
LeRobot. LeRobot's high-level API is position-only and is not used.

## What this validates vs. not

Supported:
- exact-ZOH double-integrator MPC + Kalman observer at torque level (`Goal_Current`);
- Cartesian tracking (hold, small circle) with `tau = J^T F_mpc + gravity`;
- **offset-free** disturbance rejection under a persistent load (needs a real
  torque interface);
- transient hand-push rejection with observer `d_hat` rise/decay;
- compute-time distribution (mean/p99/max) on the control computer.

Honest limits (state in the paper):
- Dynamixel current control is **modest fidelity** (quantized ~2.69 mA, torque
  ripple), not Franka-FR3-grade 1 kHz torque;
- 4-DOF, **translational** task only (matches the paper's `dim=3` benchmark);
- kinematics and gravity/mass are model-based and validated against MuJoCo; the
  current↔torque scale (`torque_constant`) and joint sign/offset still need a
  quick **hardware calibration** (below).

## Layout
```
openmanipulator_verify/
├── lib/
│   ├── interaction_mpc.py     # exact-ZOH DI-MPC + Kalman observer
│   ├── kinematics.py          # FK + numerical Jacobian (verified vs MuJoCo)
│   ├── dynamics.py            # RNEA gravity G(q) + mass M(q) (verified vs MuJoCo)
│   ├── dynamixel_backend.py   # DYNAMIXEL SDK current-mode backend + numpy sim
│   └── mujoco_sim.py          # MuJoCo physics backend (real URDF)
├── configs/                   # hold / circle / push / payload profiles
├── verification/              # run_hardware_verification.py, analyze_log.py
├── tools/
│   ├── check_current_interface.py   # confirms servos accept current mode
│   └── verify_against_mujoco.py      # FK/dynamics/suite vs MuJoCo (run first)
├── run_openmanipulator_hardware.py   # torque control loop
├── test_local.py              # numpy-only smoke test
└── results/hardware/
```

## Control (operational-space, torque level)

The loop uses proper rigid-body dynamics (`lib/dynamics.py`, RNEA, validated
against MuJoCo): `F = Λ(q)(ẍ_d + u) + G(q)`, `τ = Jᵀ F + N·τ_posture`, with
`Λ(q)=(J M⁻¹ Jᵀ)⁻¹` the task inertia, `G(q)` exact gravity, and `N` a
dynamically-consistent null-space projector for the redundant 4th DOF. The MPC
residual `u` and the Kalman disturbance observer are unchanged from the paper.

## Rigorous validation against MuJoCo (do this first)
```bash
python3 -m pip install numpy pyyaml mujoco robot_descriptions
python3 tools/verify_against_mujoco.py
```
This loads the **real ROBOTIS URDF** into MuJoCo (independent physics) and checks:
1. FK + Jacobian vs MuJoCo — machine precision;
2. gravity `G(q)` + mass matrix `M(q)` vs MuJoCo — to ~1%;
3. the J1–J4 control suite on MuJoCo physics reaches offset-free steady state.

Latest result (MuJoCo physics): **hold SS 1.3 mm, circle SS 2.0 mm, push
(2 N) recovers 103→1.9 mm, payload (1 N / 100 g) recovers 51→2.3 mm — offset-free**.

## Quick numpy-only smoke test (no MuJoCo)
```bash
python3 test_local.py
```
Coarse "does it run / no divergence" check on a lightweight plant; the MuJoCo
script above is the validation of record.

## On the real robot

### 0. Install + interface check (arm powered and SUPPORTED)
```bash
python3 -m pip install dynamixel-sdk
python3 tools/check_current_interface.py --port /dev/ttyUSB0 --ids 11 12 13 14
```
Must print `PASS: all servos accept Current Control Mode`. This never commands
torque; it leaves torque disabled.

### 1. Calibrate before any torque hold
- **Joint sign/offset:** with torque OFF, move the arm by hand, read joint
  angles, and set `joint_sign` / `joint_offset_rad` so `FK(q)` matches reality
  (gravity/mass come from the validated model, but they assume correct signs).
- **Torque constant:** `torque_constant_Nm_per_A` (default 1.78) sets the
  current↔torque scale; with `gravity_scale=1` the arm should nearly hold
  itself in Current mode — nudge `torque_constant`/`gravity_scale` if it sags or
  pushes. **Start with a low `current_limit_ticks`** (~300 ≈ 0.8 A) and the arm
  supported; raise only after the hold is stable.

### 2. Verification sequence (log everything to CSV)
| ID | Config | Test | Evidence |
|---|---|---|---|
| J1 | hold | torque hold, no contact | tracking, tau, compute time |
| J2 | circle | small Cartesian circle | command vs measured, RMSE |
| J3 | push (hand) | gentle hand push/release | `d_hat` rise/decay, recovery time |
| J4 | payload | hang a known mass | **offset-free** SS error, `d_hat` |

```bash
python3 verification/run_hardware_verification.py --test-id J1_hold \
    --backend dynamixel --port /dev/ttyUSB0 --config configs/hold.yaml --duration 20
python3 verification/analyze_log.py results/hardware/J1_hold.csv \
    --json-out results/hardware/J1_hold_metrics.json
```
For J3/J4 on hardware you apply the push / hang the mass by hand (the
`disturbance:` block in the config is for the sim backend only).

## SAFETY
- Keep the arm **supported** and the **power switch reachable** on first torque runs.
- Start with **low `current_limit_ticks`** and low `tau_max_Nm`; raise gradually.
- Torque is **disabled on any exit** (normal or exception).
- The loop includes a joint-limit barrier and torque saturation, but these are
  software guards — do not rely on them alone.
