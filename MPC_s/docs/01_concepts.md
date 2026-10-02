# Concepts: task-space MPC + disturbance observer

This document explains the control architecture in `run_hardware.py` from first principles. It assumes you know standard control theory (state feedback, LQR, Kalman filtering) but nothing about this specific arm or about tuning a real robot controller.

## 1. The problem

We want the end-effector of a 3-DOF robot arm to track a reference position `p_d(t)` in a 2D task space (the x-z plane -- see "why 2D, not 3D" below). The arm is controlled in **torque** (current) mode: at every 10ms tick, we choose a joint torque `tau` and send it to the servos.

If the robot's dynamics model were perfect, this would be a solved problem: compute the torque that produces exactly the acceleration you want via inverse dynamics, and track the reference with a plain PD loop. In reality, the dynamics model is never perfect -- and worse, this arm has real
**Coulomb friction** (static "stiction" that resists starting to move at all) that isn't captured by the smooth, deterministic terms of a standard rigid-body dynamics model. A real measurement showed a joint frozen solid against half a Newton-meter of commanded torque before breaking free.

A control law that has to be stiff enough to *fight* an unknown, possibly large disturbance to a small tracking error is inherently a trade-off: too soft and you get a large steady-state error, too stiff and you risk instability against the real (unmodelled) plant. The architecture here takes a different approach.

## 2. The plant model

At the level the controller reasons about, the task-space tracking error behaves like a double integrator with an unknown additive disturbance:

$$
\ddot e = u + d(t)
$$

- `e = ee_position - p_d` is the task-space tracking error.
- `u` is the residual acceleration we command (on top of the reference's own acceleration `p_d_ddot`).
- `d(t)` is everything the double-integrator model does NOT capture: friction, an added payload, model mismatch, unmodelled coupling -- lumped into one term. `d` is assumed to vary slowly compared to the control loop (a "random walk"), which is a much weaker and more realistic assumption than "d is exactly zero" or "d is exactly known."

## 3. The control law: active disturbance rejection, solved as a constrained QP

Given an ESTIMATE `d_hat` of the disturbance, the controller re-solves a finite-horizon, box-constrained quadratic program every tick -- NOT a fixed gain -- over the residual-acceleration sequence $u_0,\dots,u_{N-1}$ (`horizon` steps):

$$
\min_{u_0,\ldots,u_{N-1}} \ \sum_{k=0}^{N-1}\Big(x_k^T Q\,x_k + (u_k+\hat d)^T R\,(u_k+\hat d)\Big) + x_N^T Q_f\,x_N \\
\qquad\text{s.t.}\qquad x_{k+1}=Ax_k+B(u_k+\hat d),\quad -u_{max}\le u_k\le u_{max}
$$

with $x_k=[e;\dot e]$, $Q=\mathrm{diag}(q_{pos}I,\,q_{vel}I)$, $R=rI$ (the same `q_pos`/`q_vel`/`r` cost weights from the config), and $Q_f$ the converged infinite-horizon Riccati solution for $(A,B,Q,R)$, so the finite horizon approximates the infinite-horizon LQR tail while the box constraint $-u_{max}\le u_k\le u_{max}$ stays exact over the whole planning window. Only the first step $u_0$ is applied; the QP is re-solved next tick (receding horizon), warm-started from the previous solution and solved by FISTA (see `lib/interaction_mpc.py`'s `NormalizedInteractionMPC`). Penalizing $(u_k+\hat d)$ rather than $u_k$ alone is the offset-free trick: it makes the steady balancing input $u=-\hat d$ cost-free rather than $R$-penalised, instead of fighting its own disturbance cancellation.

When the box constraint does not bind, this collapses to a simple linear state-feedback law,

$$
u = -K_{eff}\begin{bmatrix} e \\ \dot e \end{bmatrix} - \hat d
$$

for a fixed gain $K_{eff}$ (`NormalizedInteractionMPC.effective_gain()`) -- the quantity `docs/02_tuning_guide.md`'s tuning procedure and `tools/solve_task_space_gain.py` actually read off to turn `q_pos`/`q_vel`/`r` into a physically meaningful stiffness/damping. It is NOT the general control law: whenever a horizon step's unconstrained $u_k$ would exceed `u_max`, the QP clips it and the applied $u\ne -K_{eff}x-\hat d$ exactly (this genuinely happens -- see `implementation_fix.md`'s Finding 2/3 entries for a real case where it broke the offset-free property until `u_max` was re-sized).

**Historical note**: an earlier version of this controller WAS exactly $u=-K_0 x-\hat d$ for a static, precomputed LQR gain $K_0$ with no per-tick re-solve and no box constraint over the horizon at all -- see `implementation_fix.md`'s "Findings 1 & 2" entry for why that was found to not match this project's own report (or the cited paper) and was replaced by the QP above.

Either way, because $\hat d$ is cancelled directly rather than accumulated through a gain that stability limits, the feedback term never has to be "stiff enough to fight friction" on its own -- this separation of concerns, estimate the disturbance and cancel it explicitly while keeping the feedback term modest, is the core idea of active disturbance rejection control (ADRC); only the specific optimization solved every tick to realize it has changed.

## 4. The observer: where `d_hat` comes from

`lib/interaction_mpc.py`'s `RandomWalkDisturbanceObserver` is a Kalman filter over the augmented state `[e, e_dot, d]`, with `d` modelled as a random walk (`d_(k+1) = d_k + noise`). Every tick it:

1. **Predicts** the next state using the plant model above and the `u` that was actually applied.
2. **Corrects** the prediction using the newly measured `e` (position error is directly measurable via forward kinematics; velocity and `d` are not measured directly and are inferred).

Two tuning knobs control its behavior:
- `observer_q_d`: how much the filter trusts new evidence about `d` (process noise on the disturbance state). Higher = faster-reacting, noisier estimate. This is the primary "observer bandwidth" knob.
- `observer_r_y`: how much the filter trusts the position measurement itself. Lower = trusts measurements more, corrects faster.

See `docs/02_tuning_guide.md` for how to choose these from a target bandwidth rather than by trial and error, and what **NIS** (Normalized Innovation Squared, returned by `step()`) tells you about whether the filter is well-tuned while it's running: for a 2D measurement, a well-tuned, consistent filter has NIS averaging around 2. Values in the hundreds mean the observer has stopped tracking reality -- usually because the controller (or the observer itself) is being pushed faster than the real system or measurement can support.

## 5. Why this beats plain PID: a worked example

The argument is pure arithmetic, and it is worth doing once with numbers. Take a task-space stiffness `Kp` in newtons per metre, and a stiction breakaway force `F_s` in newtons -- the force the end effector must exert before the joints move at all. Both are order-of-magnitude quantities you can estimate for your own arm: `Kp` is whatever your controller's first-step gain works out to at its operating point, and `F_s` you find by pushing on the end effector with a force gauge until it breaks loose.

Suppose `Kp` is a few newtons per metre and `F_s` is around a newton -- entirely ordinary for a small, geared, direct-driven arm like this one.

- A **plain PD** controller (no integral action) settles at a **permanent** steady-state deadband of $F_s / K_p$. At the numbers above that is on the order of half a metre. A proportional term generates force in proportion to error, so it simply stops generating any motion once the error is small enough that $K_p e$ drops below $F_s$. It has no mechanism at all for steady-state authority against a constant disturbance, and raising `Kp` until the deadband is acceptable takes you far outside the range where the loop is still stable at this bandwidth.
- **PD + integral action** can in principle close that deadband, since an integrator's output keeps growing as long as any error remains. But the integral gain is tightly bounded by stability -- for this system Routh-Hurwitz gives $k_i < k_d k_p / \Lambda$ -- and even at a stable integral gain, the time constant to wind up to breakaway force can exceed the time available between reference changes. The integrator never catches up before the next move starts.
- **Matched stiffness plus a disturbance observer** breaks the trade-off. `d_hat` is subtracted from the command directly, not accumulated through a gain that stability limits. The observer supplies steady-state authority an integrator structurally cannot match at the same bandwidth, because a random-walk disturbance model gives far better phase margin than an integrator does for the same steady-state authority. The deadband collapses without touching `Kp` at all.

This is why `configs/payload.yaml` is worth running and watching closely (`tools/plot_payload.py`): you'll see the tracking error spike when the payload is added, then decay back toward zero as `d_hat` converges to the new disturbance value -- not settle at a permanent offset the way a plain-PD controller would.

### You can now run this argument instead of only reading it

`run_pid.py` is the plain-PID side of the comparison above, with the same arm, the same trajectory, the same gravity model and the same null-space projector -- only the feedback law differs. Run both on `configs/payload.yaml`, which applies a constant force and holds it, and look at how each one's error behaves after the force appears:

```bash
python run_hardware.py --backend sim --config configs/payload.yaml --duration 20 --output results/mpc_payload.csv
python run_pid.py      --backend sim --config configs/payload.yaml --duration 20 --output results/pid_payload.csv
python tools/plot_payload.py results/mpc_payload.csv --config configs/payload.yaml --output results/mpc_payload.png
python tools/plot_payload.py results/pid_payload.csv --config configs/payload.yaml --output results/pid_payload.png
```

One caveat that matters for reading the result honestly: **the simulator has no friction model.** `F_s` above -- the term that makes the arithmetic dramatic -- is essentially zero in simulation. So sim understates the case for the observer, and on some tasks the PID will look at least as good. What sim does show is the structural part of the argument: a constant disturbance, and the fact that the PID has only its bounded integral to work it off with while the observer estimates it directly. The friction part of the story needs real hardware.

## 6. Three things the main loop does beyond the control law itself

**Operational-space force mapping.** `u` is a task-space (Cartesian) acceleration; the arm is actuated in joint torque. The mapping is

$$
F_{task} = \Lambda(q)\,(\ddot p_d + u), \qquad \tau_{task} = J^T F_{task}, \qquad \Lambda(q) = \left(J\,M(q)^{-1} J^T + \lambda_{damp} I\right)^{-1}
$$

where $\Lambda(q)$ is the **operational-space mass matrix** -- it's what makes `u` behave like a genuine Cartesian acceleration command regardless of the arm's current configuration (without it, the same `u` would produce different real accelerations at different poses, since the arm's effective inertia as seen from the end-effector changes with pose). $\lambda_{damp}$ (config key `robot.lambda_damping`) is a small Tikhonov regularizer -- without it, $J\,M(q)^{-1}J^T$ alone is only invertible away from a kinematic singularity, and this project's own `tools/check_circle_workspace.py` exists specifically to check a trajectory stays away from one.

**Null-space posture control.** This arm has 3 joints but the task is only 2D (x-z position) -- one degree of freedom is redundant.

$$
N = I - J^T\left(\Lambda\,J\,M^{-1}\right)
$$

is the dynamically-consistent null-space projector: it lets a secondary objective (`tau_post`, a soft pull toward a fixed nominal posture `posture_q_rad`) act on the redundant direction WITHOUT disturbing the task-space tracking at all -- any `tau_post` you'd add gets projected through $N$ so its effect on $\ddot e$ is exactly zero.

**Gravity and task-space Coriolis/centrifugal compensation.** The force mapping above only covers the commanded task-space acceleration; every tick also adds joint-space gravity compensation and a task-space Coriolis/centrifugal decoupling term, so the closed loop really does see the simple double integrator of Section 2 regardless of the arm's current velocity and gravity load:

$$
\tau = J^T F_{task} + N\,\tau_{post} + g_{scale}\,G(q) + C(q,\dot q)\dot q + J^T\mu(q,\dot q), \\
\qquad \mu(q,\dot q) = \Lambda(q)\Big(J\,M(q)^{-1}\,C(q,\dot q)\dot q - \dot J\,\dot q\Big)
$$

$\mu$ accounts for how the task-space mapping itself changes with $q,\dot q$ -- distinct from, and in addition to, the joint-space $C(q,\dot q)\dot q$ term next to it -- matching Cao & Tang's classical operational-space impedance law. $g_{scale}$ (config key `robot.gravity_scale`, default 1.0) is a partial-gravity-compensation knob, mostly useful for `tools/test_gravity_compensation.py`'s own diagnostic.

## Why the task space is 2D (x-z), not 3D

All 3 joints of this arm rotate about the same (horizontal) axis, so the end-effector's y-coordinate never changes -- the Jacobian's y-row is identically zero. Using the full 3D task space would make `Lambda(q)` structurally singular in the y direction; the code always works in the x-z plane instead (`xz = [0, 2]` throughout `run_hardware.py`).
