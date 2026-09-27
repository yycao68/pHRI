# Concepts: task-space MPC + disturbance observer

This document explains the control architecture in `run_hardware.py` from
first principles. It assumes you know standard control theory (state
feedback, LQR, Kalman filtering) but nothing about this specific arm or
about tuning a real robot controller.

## 1. The problem

We want the end-effector of a 3-DOF robot arm to track a reference position
`p_d(t)` in a 2D task space (the x-z plane -- see "why 2D, not 3D" below).
The arm is controlled in **torque** (current) mode: at every 10ms tick, we
choose a joint torque `tau` and send it to the servos.

If the robot's dynamics model were perfect, this would be a solved problem:
compute the torque that produces exactly the acceleration you want via
inverse dynamics, and track the reference with a plain PD loop. In reality,
the dynamics model is never perfect -- and worse, this arm has real
**Coulomb friction** (static "stiction" that resists starting to move at
all) that isn't captured by the smooth, deterministic terms of a standard
rigid-body dynamics model. A real measurement showed a joint frozen solid
against half a Newton-meter of commanded torque before breaking free.

A control law that has to be stiff enough to *fight* an unknown, possibly
large disturbance to a small tracking error is inherently a trade-off: too
soft and you get a large steady-state error, too stiff and you risk
instability against the real (unmodelled) plant. The architecture here takes
a different approach.

## 2. The plant model

At the level the controller reasons about, the task-space tracking error
behaves like a double integrator with an unknown additive disturbance:

```
e_ddot = u + d(t)
```

- `e = ee_position - p_d` is the task-space tracking error.
- `u` is the residual acceleration we command (on top of the reference's own
  acceleration `p_d_ddot`).
- `d(t)` is everything the double-integrator model does NOT capture:
  friction, an added payload, model mismatch, unmodelled coupling -- lumped
  into one term. `d` is assumed to vary slowly compared to the control loop
  (a "random walk"), which is a much weaker and more realistic assumption
  than "d is exactly zero" or "d is exactly known."

## 3. The control law: active disturbance rejection

Given an ESTIMATE `d_hat` of the disturbance, the controller does two things
every tick:

```
u = -K0 [e; e_dot] - d_hat        # cancel the tracking error AND the disturbance
```

`K0` is a state-feedback gain designed for the disturbance-free system
(`e_ddot = u`) -- see `lib/interaction_mpc.py`'s `NormalizedInteractionMPC`,
which computes it as the first-step gain of a finite-horizon LQR solve
(`q_pos`/`q_vel` penalize position/velocity error, `r` penalizes control
effort). Because `d_hat` is subtracted separately, `K0` never has to be
"stiff enough to fight friction" -- it only has to handle the clean,
disturbance-free part of the problem, while `-d_hat` handles the rest. This
separation of concerns -- estimate the disturbance, cancel it explicitly,
keep the feedback gain modest -- is the core idea of active disturbance
rejection control (ADRC).

## 4. The observer: where `d_hat` comes from

`lib/interaction_mpc.py`'s `RandomWalkDisturbanceObserver` is a Kalman
filter over the augmented state `[e, e_dot, d]`, with `d` modelled as a
random walk (`d_(k+1) = d_k + noise`). Every tick it:

1. **Predicts** the next state using the plant model above and the `u` that
   was actually applied.
2. **Corrects** the prediction using the newly measured `e` (position error
   is directly measurable via forward kinematics; velocity and `d` are not
   measured directly and are inferred).

Two tuning knobs control its behavior:
- `observer_q_d`: how much the filter trusts new evidence about `d`
  (process noise on the disturbance state). Higher = faster-reacting,
  noisier estimate. This is the primary "observer bandwidth" knob.
- `observer_r_y`: how much the filter trusts the position measurement
  itself. Lower = trusts measurements more, corrects faster.

See `docs/02_tuning_guide.md` for how to choose these from a target
bandwidth rather than by trial and error, and what **NIS** (Normalized
Innovation Squared, returned by `step()`) tells you about whether the filter
is well-tuned while it's running: for a 2D measurement, a well-tuned,
consistent filter has NIS averaging around 2. Values in the hundreds mean
the observer has stopped tracking reality -- usually because the controller
(or the observer itself) is being pushed faster than the real system or
measurement can support.

## 5. Why this beats plain PID: a worked example

The argument is pure arithmetic, and it is worth doing once with numbers.
Take a task-space stiffness `Kp` in newtons per metre, and a stiction
breakaway force `F_s` in newtons -- the force the end effector must exert
before the joints move at all. Both are order-of-magnitude quantities you
can estimate for your own arm: `Kp` is whatever your controller's
first-step gain works out to at its operating point, and `F_s` you find by
pushing on the end effector with a force gauge until it breaks loose.

Suppose `Kp` is a few newtons per metre and `F_s` is around a newton --
entirely ordinary for a small, geared, direct-driven arm like this one.

- A **plain PD** controller (no integral action) settles at a
  **permanent** steady-state deadband of `F_s / Kp`. At the numbers above
  that is on the order of half a metre. A proportional term generates
  force in proportion to error, so it simply stops generating any motion
  once the error is small enough that `Kp * e` drops below `F_s`. It has
  no mechanism at all for steady-state authority against a constant
  disturbance, and raising `Kp` until the deadband is acceptable takes you
  far outside the range where the loop is still stable at this bandwidth.
- **PD + integral action** can in principle close that deadband, since an
  integrator's output keeps growing as long as any error remains. But the
  integral gain is tightly bounded by stability -- for this system
  Routh-Hurwitz gives `ki < kd*kp/Lambda` -- and even at a stable integral
  gain, the time constant to wind up to breakaway force can exceed the
  time available between reference changes. The integrator never catches
  up before the next move starts.
- **Matched stiffness plus a disturbance observer** breaks the trade-off.
  `d_hat` is subtracted from the command directly, not accumulated through
  a gain that stability limits. The observer supplies steady-state
  authority an integrator structurally cannot match at the same bandwidth,
  because a random-walk disturbance model gives far better phase margin
  than an integrator does for the same steady-state authority. The
  deadband collapses without touching `Kp` at all.

This is why `configs/payload.yaml` is worth running and watching closely
(`tools/plot_payload.py`): you'll see the tracking error spike when the
payload is added, then decay back toward zero as `d_hat` converges to the
new disturbance value -- not settle at a permanent offset the way a
plain-PD controller would.

### You can now run this argument instead of only reading it

`run_pid.py` is the plain-PID side of the comparison above, with the same arm,
the same trajectory, the same gravity model and the same null-space projector --
only the feedback law differs. Run both on `configs/payload.yaml`, which applies
a constant force and holds it, and look at how each one's error behaves after the
force appears:

```bash
python run_hardware.py --backend sim --config configs/payload.yaml --duration 20 --output results/mpc_payload.csv
python run_pid.py      --backend sim --config configs/payload.yaml --duration 20 --output results/pid_payload.csv
python tools/plot_payload.py results/mpc_payload.csv --config configs/payload.yaml --output results/mpc_payload.png
python tools/plot_payload.py results/pid_payload.csv --config configs/payload.yaml --output results/pid_payload.png
```

One caveat that matters for reading the result honestly: **the simulator has no
friction model.** `F_s` above -- the term that makes the arithmetic dramatic -- is
essentially zero in simulation. So sim understates the case for the observer, and
on some tasks the PID will look at least as good. What sim does show is the
structural part of the argument: a constant disturbance, and the fact that the PID
has only its bounded integral to work it off with while the observer estimates it
directly. The friction part of the story needs real hardware.

## 6. Two things the main loop does beyond the control law itself

**Operational-space force mapping.** `u` is a task-space (Cartesian)
acceleration; the arm is actuated in joint torque. The mapping is
`F_task = Lambda(q) (p_d_ddot + u)`, `tau_task = J^T F_task`, where
`Lambda(q) = (J M(q)^-1 J^T)^-1` is the **operational-space mass matrix** --
it's what makes `u` behave like a genuine Cartesian acceleration command
regardless of the arm's current configuration (without it, the same `u`
would produce different real accelerations at different poses, since the
arm's effective inertia as seen from the end-effector changes with pose).

**Null-space posture control.** This arm has 3 joints but the task is only
2D (x-z position) -- one degree of freedom is redundant. `N = I -
J^T(Lambda J M^-1)` is the dynamically-consistent null-space projector: it
lets a secondary objective (`tau_post`, a soft pull toward a fixed nominal
posture `posture_q_rad`) act on the redundant direction WITHOUT disturbing
the task-space tracking at all -- any `tau_post` you'd add gets projected
through `N` so its effect on `e_ddot` is exactly zero.

## Why the task space is 2D (x-z), not 3D

All 3 joints of this arm rotate about the same (horizontal) axis, so the
end-effector's y-coordinate never changes -- the Jacobian's y-row is
identically zero. Using the full 3D task space would make `Lambda(q)`
structurally singular in the y direction; the code always works in the
x-z plane instead (`xz = [0, 2]` throughout `run_hardware.py`).
