# Tuning guide: from a target bandwidth to config numbers

There are two, mostly independent, tuning problems: the **controller**
(`q_pos`, `q_vel`, `r`) and the **observer** (`observer_q_d`,
`observer_r_y`). This guide gives you a principled starting point for both,
instead of pure trial and error.

## 1. The controller: Gao's bandwidth parameterization

A classical result (Gao, 2003 -- "Scaling and bandwidth-parameterization
based controller tuning") reduces a PD-like control law to a single
parameter, the closed-loop bandwidth `wc` (rad/s):

```
u0 = -(wc^2 * e + 2*wc * e_dot)
```

This is a critically-damped, second-order closed loop with natural
frequency `wc`. Comparing to `u0 = -(K1*e + K2*e_dot)` gives the mapping:

```
K1 = wc^2
K2 = 2*wc
```

**This project's controller is discrete** (100 Hz, `dt = 0.01s`), and the
exact discrete-time gains that place both closed-loop poles at
`p = exp(-wc*dt)` (repeated) are:

```
K1(wc) = (1 - p)^2 / dt^2
K2(wc) = (1 - p)*(3 + p) / (2*dt)
p = exp(-wc * dt)
```

These recover the continuous formula above exactly as `dt -> 0`, and are
the right ones to use for picking an actual discrete gain at `dt=0.01s`
(the continuous formula alone can be noticeably off at this sample rate for
larger `wc`).

### From a target (K1, K2) to (q_pos, q_vel, r)

`NormalizedInteractionMPC`'s actual free parameters are the LQR cost
weights `q_pos`, `q_vel`, `r` -- not `K1`/`K2` directly. There is no
closed-form inverse from a target `(K1, K2)` to `(q_pos, q_vel, r)`; the
practical approach is a small root-finding loop:

1. Fix `q_pos` (or `r`) and solve for the other two numerically
   (`scipy.optimize.fsolve`) so that the resulting gain (read off
   `NormalizedInteractionMPC(cfg).effective_gain()`, the box-QP's closed-form
   *unconstrained* first-step gain -- see its docstring) matches your target
   `K1`, `K2`.
2. A handful of Newton iterations converges quickly in practice -- e.g.
   starting from `q_vel=r=1` and iterating toward a target `(K1, K2) =
   (84.2, 17.9)` (an example target bandwidth) typically converges to
   residuals under `1e-6` within 5-10 iterations.

This is genuinely a numerical step, not something to eyeball -- but you
only need to do it once per target bandwidth, and the four configs already
in `configs/` give you good starting points to interpolate from by trial
and error if you'd rather not write the root-finder yourself.

## 2. The observer: bandwidth relative to the controller

The observer needs to be FASTER than the controller it feeds, or its
estimate lags the very transient it's supposed to help reject. MathWorks'
ADRC documentation (which directly implements Gao's LADRC framework)
recommends the observer bandwidth be **5 to 10 times** the controller
bandwidth as a starting point (The MathWorks, Inc., n.d., "Extended
State Observer with Nonlinear Functions").

`observer_q_d` is the primary bandwidth knob (higher = faster); the
observer's actual bandwidth for a given `observer_q_d`/`observer_r_y` pair
isn't a closed-form function of those two numbers alone, so the practical
approach is: pick a small `observer_q_d` (this project's own configs start
around 1.0-3.0), run `--backend sim`, and watch two things using
`tools/plot_traj.py`:
- Does `d_hat` converge to a sensible, bounded value in a reasonable
  fraction of a second (roughly 1/wc_observer), or does it lag visibly
  behind a disturbance you know you injected (`configs/push.yaml`/
  `configs/payload.yaml`)?
- Is the reported `nis` (mean, printed in the plot title) close to 2
  (the measurement dimension)? Much larger means the filter has stopped
  being consistent with reality; much smaller means it's overly
  conservative (not reacting to real evidence fast enough).

## 3. A practical workflow

1. Start from `configs/hold.yaml`'s gains -- a deliberately conservative
   starting point, not the tightest possible tuning.
2. Run `--backend sim` first, always. Watch `err_mm` in the console and/or
   `tools/plot_traj.py`'s error figure.
3. To tighten tracking: raise `q_pos` (and usually `q_vel` alongside it,
   roughly proportionally) and/or lower `r`. Re-run sim after every change.
4. To make the observer react faster: raise `observer_q_d`. Check NIS
   doesn't blow up.
5. Only after a config looks good in sim, move to `--backend dynamixel`
   (see `docs/03_hardware_safety.md` first) -- and when you do, change ONE
   gain at a time, in SMALL increments. Real hardware has failure modes sim
   cannot show at all (sensor noise, communication delay, real stiction) --
   a gain that looks perfectly stable in sim can oscillate on real hardware,
   and a large jump in gain makes it much harder to tell which change
   caused a problem.
6. `configs/payload.yaml` and `configs/circle.yaml` use noticeably
   different, stiffer gains than `configs/hold.yaml`/`configs/push.yaml` --
   this is not an oversight. Gains that work well for a fixed-point
   hold/disturbance-rejection task are not automatically right for a
   continuously-moving reference, and vice versa. Re-tune per task rather
   than assuming one gain set is universal.
7. Check what a tick actually costs before choosing `dt`:
   `python tools/benchmark_compute.py --config configs/hold.yaml --controller mpc`.
   It times only the control-law work, without the simulator's own physics
   integration, and compares the p99 against the `dt` budget. Judge on p99
   rather than the mean: a loop that misses its deadline one tick in a hundred
   is a loop that misses its deadline.

## 4. The PID gains, and what "the same gain" means across the two controllers

`configs/*.yaml` also carry a `pid:` block, which `run_pid.py` reads and
`run_hardware.py` ignores completely. Those are `kp`, `ki`, `kd` in force units
(newtons per metre, newton-seconds per metre), plus `i_max` and `f_max`.

**They are not on the same scale as `q_pos`/`q_vel`/`r` above.** The LQR weights
are costs that a Riccati solve turns into a gain; the PID numbers are the gain
itself. Writing `kp: 60` because `q_pos: 60` compares nothing.

If you want a genuinely matched comparison rather than two separately tuned
controllers, match the **closed-loop stiffness** at one operating point:

1. Pick the posture you care about, `q0`, and evaluate the operational-space
   mass matrix there: `Lam = inv(J_xz @ inv(M(q0)) @ J_xz.T)`.
2. Read the MPC's own first-step feedback gain out of the solved controller:
   `K0 = NormalizedInteractionMPC(cfg).effective_gain()` (the box-QP's
   closed-form *unconstrained* gain -- exact whenever the box constraint
   isn't active, which `tools/solve_task_space_gain.py` also relies on).
   Its position and velocity blocks are in acceleration units.
3. The MPC's command becomes a force through `F = Lam @ u`, so the force-domain
   stiffness and damping it is actually applying at `q0` are `Lam @ K0_pos` and
   `Lam @ K0_vel`. Those are the numbers to put into `pid.kp` and `pid.kd`.
4. Set `pid.ki = 0` for that run. With `ki` at zero the ONLY difference left
   between the two controllers is the observer channel, which turns a vague
   "MPC beats PID" into the specific and answerable "does the observer beat an
   integrator here".

Note step 1 is posture-dependent, and that is the point: the MPC's effective
stiffness moves with `Lam(q)` as the arm reconfigures, while the PID's stays put.
A match computed at one posture is a match at that posture only. If a comparison
sweeps the arm across a large part of its workspace, say so rather than quoting
one matched number.

`i_max` deserves its own check when `ki` is not zero. It caps the integral
before `ki` multiplies it, so `ki * i_max` is a hard ceiling on the steady-state
force the integral term can ever produce. If that ceiling sits below the force
needed to break the joints loose against stiction, the arm stops short and stays
there no matter how long the run is -- and the log will look like a controller
that converged, not like one that ran out of authority. Compute `ki * i_max`
and compare it against a measured breakaway force before reporting anything.

## 5. Reference

The MathWorks, Inc. (n.d.). *Extended State Observer with Nonlinear
Functions*. MATLAB & Simulink documentation.

Gao, Z. (2003). Scaling and bandwidth-parameterization based controller
tuning. *Proceedings of the 2003 American Control Conference*, 4989-4996.
