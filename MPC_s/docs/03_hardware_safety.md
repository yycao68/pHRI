# Hardware safety guide

Everything in this project runs against `--backend sim` by default and requires no real hardware. This document is for the point where you want to run `--backend dynamixel` against a real OpenManipulator-X-family arm. **Read this whole document before your first hardware run.**

## Why each safety mechanism exists

| Mechanism | Config key(s) | What it protects against |
|---|---|---|
| Current limit | `robot.current_limit_ticks` | Caps the maximum current (and therefore torque) the servo firmware will ever apply, regardless of what the software commands. This is the last line of defense if the control loop computes something wrong -- keep it LOW (the shipped configs use 200-300, out of a much larger hardware maximum) until you've verified a config is stable. |
| Torque clamp | `robot.tau_max_Nm` | A software-side clamp on commanded joint torque, checked every tick before sending. Independent of current_limit_ticks (which is enforced by the servo firmware itself) -- two independent layers. |
| Joint limit barrier | `robot.joint_barrier_gain`, `robot.joint_barrier_margin_rad` | A soft repulsive torque that grows as a joint approaches `joint_min_rad`/`joint_max_rad`, so the controller is pushed away from the mechanical limits before it reaches them, rather than relying on the limits alone. |
| Startup pose plausibility check | (automatic, `robot.startup_pose_max_overshoot_rad`) | Refuses to energize the arm if the measured starting pose is outside the joint limits -- this specifically catches a multi-turn position-register wraparound (see `lib/dynamixel_backend.py`'s `assert_startup_pose_plausible` docstring for the failure mode this catches: forward kinematics alone CANNOT detect a full-turn wrap, because it's periodic in every joint angle). |
| Starting-pose mismatch check | `robot.start_pose_mismatch_max_m`, `--force-start` | Refuses to start if the arm's actual pose is implausibly far from where the config expects it to be (wrong config loaded, arm in an unexpected configuration). `move_to_start` (below) handles the NORMAL amount of mismatch from placing the arm by hand; this check is for genuinely alarming cases. |
| Slow homing move (`move_to_start`) | `robot.move_to_start_*` | Moves the arm smoothly from wherever it currently is to the config's `posture_q_rad` BEFORE the real controller starts, using a separate, simple, well-tested joint-space PID -- not the controller under test. Without this, the null-space posture term can yank the arm through an uncontrolled motion the instant torque is enabled. |
| Torque-disable on exit | (automatic) | `run_hardware.py` disables torque in a `finally` block, covering normal completion, `--duration` expiry, Ctrl+C, AND any exception -- there is no code path that leaves torque silently enabled after the script stops running. |
| Divergence auto-stop | `--max-err-mm` / `--max-err-consecutive` (on by default: 25mm / 5 samples) | Stops the loop itself if `err_mm` stays above the threshold for that many consecutive ticks, instead of relying on `--duration` expiring or a human operator noticing and hitting Ctrl+C. Added after real hardware showed a genuine ~7.4-7.9Hz self-excited oscillation above certain gains (see `divergence_analysis.md`) -- every one of those runs was otherwise stopped by an operator 1-2s after visible onset. `--max-err-mm 0` disables it. |
| Chirp diagnostic deviation abort | `tools/chirp_response.py`'s `--max-dev-rad` (default 0.3) and `--amplitude-nm` refusal | `tools/chirp_response.py` commands pure open-loop torque (gravity compensation + a swept-sine chirp, NO position feedback at all) to one joint -- same no-feedback risk category as the gravity-compensation check below, just with a known torque added on top. Aborts if the excited joint strays past `--max-dev-rad` from its start angle, and refuses to even start at an `--amplitude-nm` that isn't comfortably under that joint's `tau_max_Nm`. See `docs/friction_chirp_analysis.md` §2.0. |

## Pre-flight checklist for a NEW arm/setup

Both controllers -- `run_hardware.py` and `run_pid.py` -- use the identical hardware backend, the identical safety layer and the identical bring-up procedure. Nothing in this document is specific to one of them.

Do these once, in order, before your first real trajectory-tracking run:

0. **Confirm the servos accept Current Control Mode at all**:
   `python tools/check_current_interface.py --port COM3 --baud 1000000 --ids 12 13 14`.
   It pings each servo, sets `Operating_Mode=0`, reads it back and reads `Present_Current`. `Goal_Current` is never written and torque is left disabled on every exit path, including the failure paths. Both controllers command torque through `Goal_Current`, so if this fails neither of them can work, and this is the cheapest place to find out.

1. **Calibrate joint sign/offset**: `python tools/calibrate_joints.py --port COM3 --baud 1000000`.
   Torque stays OFF the whole time. Move the arm by hand, record checkpoints, and work out `joint_sign`/`joint_offset_rad` for your config (the script's own docstring walks through exactly how). This has to match YOUR arm's physical mounting -- the values in the shipped configs are specific to the arm this project was developed on and are very unlikely to be correct for a different physical unit without re-calibration.

2. **Verify gravity compensation ONLY**, with no closed-loop feedback at all: `python tools/test_gravity_compensation.py --port COM3 --config configs/hold.yaml --duration 15`. This commands ONLY `tau = gravity_scale * dyn.gravity(q) - damping * dq` -- no position error feedback, no Jacobian, no MPC -- so there is no possibility of the closed-loop instability a wrong `joint_sign` can cause in the full controller. The arm should feel roughly weightless (easy to move by hand, not sagging, not fighting you) if `joint_sign`/`joint_offset_rad` are correct. Fix calibration before proceeding if it doesn't. (`tools/chirp_response.py --backend dynamixel` is in this same no-feedback-at-all category, just with a known swept-sine torque added on top of the same gravity compensation -- treat a first real run of it with the same care as this check, see the table above.)

3. **Check the planned trajectory fits the workspace**, offline, before ever touching hardware: `python tools/check_circle_workspace.py --config configs/circle.yaml`. This reports whether the trajectory stays reachable, inside joint limits (with margin), and away from kinematic singularities.

3b. **Check this machine can compute a tick inside `dt`**:
   `python tools/benchmark_compute.py --config configs/hold.yaml --controller mpc`,
   and again with `--controller pid` if you intend to run that one. It times the control law alone, without the simulator's physics integration, so the number transfers to hardware. Add your serial round-trip time on top. Judge on the p99 rather than the mean, and note what happens if you get this wrong: asking for a `dt` the machine or the bus cannot serve does not fail loudly, the loop just runs slower than the config says -- and then every gain in that file is wrong for the rate it is really running at.

4. **Validate the config in `--backend sim` first, always.** Every config should run cleanly in sim (see the README's Quickstart) before you ever pass `--backend dynamixel`. Sim cannot catch everything (it has no friction model, no sensor noise, no communication delay), but it does catch sign errors, gross instabilities, and configuration mistakes cheaply and safely.

5. **First real-hardware run**: arm physically supported (not hanging freely) and a low `current_limit_ticks` (the shipped configs' 200-300 is a reasonable starting point). Keep a hand near the power switch. Run a SHORT duration first (`--duration 10`).

## Moving from sim to hardware

- Change gains in small increments once on hardware -- a gain that's perfectly stable in sim can oscillate on real hardware (see `docs/02_tuning_guide.md`'s note on why).
- Watch the console's live `err_mm`/`tau` printout. A `tau` pinned at `tau_max_Nm` for an extended time, or `err_mm` growing rather than settling, means stop (Ctrl+C) and investigate rather than waiting to see if it recovers.
- If using `--output`, `tools/plot_traj.py` after the run gives you the full picture (including the achieved control-loop rate, which can be lower on real hardware due to communication latency).
