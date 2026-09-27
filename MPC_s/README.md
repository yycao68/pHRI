# MPC: Task-Space MPC + Disturbance Observer for a 3-DOF OPENManipulator-X

## 1. Install

Requires Python 3.10+ (developed and tested on 3.13).

```bash
cd MPC
python -m venv venv
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

pip install -r requirements.txt
```

`requirements.txt` installs `numpy`, `PyYAML`, `matplotlib`, and
`dynamixel-sdk`. The first three are all you need for simulation
(`--backend sim`, the default everywhere in this README); `dynamixel-sdk`
is only imported when you pass `--backend dynamixel` to talk to real
hardware -- see `docs/03_hardware_safety.md` before you do that.

## 2. Quickstart: your first run

From the `MPC/` directory:

```bash
python run_hardware.py --backend sim --config configs/hold.yaml --duration 10
```

You should see the arm move to its starting pose (`[move-to-start]` log
lines), then a running printout like:

```
[omx-3dof] backend=sim ee0=[-0.0317  0.      0.2168] dt=0.01 duration=10.0s
[omx-3dof] t=  0.24s  err=   0.18mm  tau=[-0.23  0.1   0.05]  ~ 55.5Hz  n=    25
...
```

Your own numbers will differ run to run; what matters is the shape.
`err` (millimetres) should shrink toward ~0 and stay there -- this is the
controller holding a fixed target position. If you see this, everything is
installed correctly and working.

## 3. The four example tasks

Each config below demonstrates a different aspect of the controller. Run
each one, then look at the plots -- seeing the shape of `d_hat` rise and
fall is much more informative than the numbers alone. All commands assume
you're in the `MPC/` directory and your virtual environment is active.

### hold: track a fixed point

The simplest possible test -- track one fixed target position and reject
whatever small disturbances show up.

```bash
python run_hardware.py --backend sim --config configs/hold.yaml --duration 10 --output results/hardware/sim_hold.csv
python tools/plot_traj.py results/hardware/sim_hold.csv --output results/hardware/sim_hold.png
```

This saves several PNGs (`sim_hold_path.png`, `sim_hold_err.png`,
`sim_hold_dhat.png`, etc.) instead of opening interactive windows. Drop
`--output` from the `plot_traj.py` call to open them interactively instead.

### push: watch the observer reject a brief disturbance

The simulator injects two brief external force pulses (a push in +z at
t=4-6s, then a push in -x at t=13-15s) while the controller holds a fixed
target. Watch the tracking error spike and recover, and `d_hat` rise to
estimate each push.

```bash
python run_hardware.py --backend sim --config configs/push.yaml --duration 20 --output results/hardware/sim_push.csv
python tools/plot_push.py results/hardware/sim_push.csv --config configs/push.yaml --output results/hardware/sim_push.png
python tools/plot_traj.py results/hardware/sim_push.csv --output results/hardware/sim_push_traj.png
```

`plot_push.py` additionally draws each push's force direction as an arrow
on the end-effector path plot, and shades each push's `[t_start, t_end]`
window on the error/joint-angle plots -- `plot_traj.py`'s generic `d_hat`
plot is still the one to look at for the observer's own estimate over time.

### payload: watch offset-free tracking after a sustained disturbance

The simulator attaches a constant force at t=5s and never removes it (like
a payload suddenly grasped). This is the config to run if you want to see
**why an observer beats plain PID**: watch the error recover to near-zero
after the disturbance, not settle at a permanent offset (see
`docs/01_concepts.md` section 5 for the numbers behind this).

```bash
python run_hardware.py --backend sim --config configs/payload.yaml --duration 20 --output results/hardware/sim_payload.csv
python tools/plot_payload.py results/hardware/sim_payload.csv --config configs/payload.yaml --output results/hardware/sim_payload.png
```

### circle: track a continuously moving reference

A 40mm-radius circle in the x-z plane, one revolution every 60 seconds.
Unlike the three tasks above, the reference itself is always moving, so
this exercises tracking bandwidth rather than disturbance rejection at a
fixed point.

```bash
python run_hardware.py --backend sim --config configs/circle.yaml --duration 60 --output results/hardware/sim_circle.csv
python tools/plot_traj.py results/hardware/sim_circle.csv --output results/hardware/sim_circle.png
```

Want to watch it live instead of plotting afterward?

```bash
python run_hardware.py --backend sim --config configs/circle.yaml --duration 60 --live-plot
```

## 4. The second controller: task-space PID

`run_pid.py` is the same program as `run_hardware.py` with one line changed.
Everything else -- the kinematics, the gravity and Coriolis model, the
operational-space mass matrix `Lambda(q)`, the null-space posture projector,
the startup ramp, the joint-limit barrier, the torque clip, the backend -- is
identical. The single difference is the feedback law that turns tracking error
into a task-space force:

```
run_hardware.py:   F_task = Lambda(q) (ddp_d + u),  u = -K0 x - d_hat
run_pid.py:        F_task = -(Kp e + Ki integral(e) + Kd edot)
```

So anything you observe differing between two runs of the same config is
attributable to the control law, and not to a hundred other things.

### Run it

Exactly like `run_hardware.py`, with the same flags:

```bash
python run_pid.py --backend sim --config configs/hold.yaml --duration 10
python run_pid.py --backend sim --config configs/payload.yaml --duration 20 --output results/pid_payload.csv
python tools/plot_traj.py results/pid_payload.csv --output results/pid_payload.png
```

The PID gains live in each config's own `pid:` block, separate from the
`controller:` block the MPC reads. Neither runner reads the other's numbers,
and `pid.dt` falls back to `controller.dt` so both controllers run at the same
rate on the same config -- running them at different rates would confound any
comparison you then make.

`tools/plot_traj.py` works on a PID log unmodified. It detects that the CSV has
no `d_hat_0` column and skips the observer figures, since there is no observer.

### Run the comparison

Same config, same duration, both controllers, then plot both:

```bash
python run_hardware.py --backend sim --config configs/payload.yaml --duration 20 --output results/mpc_payload.csv
python run_pid.py      --backend sim --config configs/payload.yaml --duration 20 --output results/pid_payload.csv
python tools/plot_payload.py results/mpc_payload.csv --config configs/payload.yaml --output results/mpc_payload.png
python tools/plot_payload.py results/pid_payload.csv --config configs/payload.yaml --output results/pid_payload.png
```

`payload` **is the task to start with**, because it is the one the two laws
genuinely disagree about. It applies a constant force and holds it. The MPC's
observer estimates that force and subtracts it, so the error decays back toward
zero. The PID can only work it off through its integral term, whose gain is
bounded by stability and whose accumulator is bounded by `i_max` -- so how well
it recovers, and whether it recovers at all, depends on gains that cannot be
raised freely. `docs/01_concepts.md` section 5 works through why.

## 5. Before you touch real hardware

**Every config above should run cleanly in** `--backend sim` **first.** When
you're ready to try real hardware, read `docs/03_hardware_safety.md` in
full -- it explains every safety mechanism in `run_hardware.py`/`run_pid.py`/
`lib/dynamixel_backend.py`, gives a pre-flight checklist, and walks through
moving from sim to hardware safely.

Both controllers use the identical hardware path, identical safety layer and
identical bring-up procedure. Nothing below is specific to one of them.

Bring-up order, once per arm:

| step | tool                                 | what it settles                                                                                                                            |
|------|--------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 1    | `tools/check_current_interface.py`   | do the servos accept Current Control Mode at all? No torque is commanded.                                                                  |
| 2    | `tools/calibrate_joints.py`          | `joint_sign` and `joint_offset_rad` for **your** arm                                                                                       |
| 3    | `tools/test_gravity_compensation.py` | is the calibration right? Gravity only, no error feedback, so a wrong sign cannot destabilize it -- the arm should feel roughly weightless |
| 4    | `tools/benchmark_compute.py`         | can this PC compute a tick inside `controller.dt`? Run it for both `--controller mpc` and `--controller pid`, and with/without `--use-jit` (optional, `pip install numba` -- ~65x/~5x measured speedup on the RNEA/FISTA hot paths, see `implementation_fix.md`) if the plain-numpy numbers are tight |
| 5    | `tools/check_circle_workspace.py`    | is the trajectory reachable and clear of singularities?                                                                                    |

Step 1 comes first because it is the cheapest possible failure, and step 3
before any closed-loop run because it is the only test that cannot be
destabilized by a calibration error.

A real hardware run then looks like:

```bash
python run_hardware.py --backend dynamixel --port COM3 --baud 1000000 \
    --config configs/hold.yaml --duration 20 --output results/hw_hold.csv

python run_pid.py --backend dynamixel --port COM3 --baud 1000000 \
    --config configs/hold.yaml --duration 20 --output results/hw_hold_pid.csv
```

## 6. Project layout

```
MPC_s/
  README.md                 <- you are here
  requirements.txt
  docs/                      the three documents listed above
  configs/                   hold.yaml, push.yaml, payload.yaml, circle.yaml
                             (each carries both a controller: and a pid: block)
  lib/
    kinematics.py             forward kinematics + Jacobian
    dynamics.py                mass matrix / gravity / Coriolis (RNEA)
    trajectory.py               reference generator (hold / circle)
    interaction_mpc.py           the MPC (NormalizedInteractionMPC) and the
                                  observer (RandomWalkDisturbanceObserver)
    pid_controller.py             the task-space PID (TaskSpacePID)
    dynamixel_backend.py           real-hardware (Current Control Mode) and
                                    simulation backends
    move_to_start.py                slow homing move run once before tracking starts
  tools/
    plot_traj.py               generic result viewer (path/error/joints/tau/d_hat/...)
                               works on either controller's log
    plot_push.py, plot_payload.py   disturbance-specific plots
    check_circle_workspace.py    offline reachability/singularity check
    benchmark_compute.py          per-tick compute cost of either control law
    calibrate_joints.py            interactive joint sign/offset calibration (hardware)
    test_gravity_compensation.py   gravity-compensation-only hardware sanity check
    check_current_interface.py     confirm the servos accept Current Control Mode (hardware)
  run_hardware.py             control loop for the MPC + observer
  run_pid.py                  control loop for the task-space PID -- the same file
                              with one line changed, see section 4
  test_local.py               no-hardware smoke test for both controllers
                              (any --output path you pass is created for you)
```