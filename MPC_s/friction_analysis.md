# Friction analysis (real hardware data)

Produced by `tools/estimate_friction.py` (see its own docstring for the full method and
caveats behind each number here -- this file is the write-up, not a restatement of it).
This project's dynamics model has no friction term at all (`docs/01_concepts.md`), so
everything below is a PROXY read off existing closed-loop logs, not a clean measurement --
the one thing that could give a clean, joint-space, closed-loop-free measurement is
`tools/chirp_response.py --backend dynamixel`, which hasn't been run on real hardware yet.
Treat this as "here's what the existing data already hints at," not a friction model ready
to drop into `lib/dynamics.py`.

## 0. Method: the friction model these estimates target, and how `d_hat` connects to it

The standard robotics friction model this project's dynamics has no term for (Coulomb +
viscous, with a separate static/breakaway threshold $\tau_s$ while the joint isn't moving):

$$
\tau_{friction}(\dot q) = \tau_c \cdot \mathrm{sign}(\dot q) + b\,\dot q \quad \text{for } |\dot q| > 0
$$

$$
|\tau_{friction}| \le \tau_s \quad \text{for } \dot q = 0 \ \text{(stiction)}
$$

- $\tau_c$ -- Coulomb (kinetic) friction torque, opposes motion, roughly constant once moving.
- $b$ -- viscous friction coefficient, friction grows linearly with speed.
- $\tau_s$ -- static friction (stiction): while at rest, friction exactly cancels whatever
  torque is applied, UP TO this limit -- it's a constraint on what the joint can resist, not
  a fixed value. Motion begins only once the applied torque exceeds it. Usually $\tau_s \ge
  \tau_c$ (breakaway takes more torque than keeping something already moving).

None of $\tau_c$, $b$, $\tau_s$ appear anywhere in `lib/dynamics.py`. What this controller DOES
have is a disturbance observer that estimates everything its own rigid-body model doesn't
capture -- `lib/interaction_mpc.py`'s own docstring names friction explicitly as one of the
things $\hat d$ is designed to lump together, with the controller's equilibrium condition
being $u = -\hat d$ (`docs/01_concepts.md` section 3). $\hat d$ is a task-space RESIDUAL
ACCELERATION (2D, x-z), not a torque, so every estimate below first converts it:

$$
F = \Lambda(q)\,\hat d \qquad \text{(task-space force [N])}
$$

$$
\tau_{eq} = J_{xz}(q)^T F \qquad \text{(joint-space torque equivalent [Nm])}
$$

$$
\Lambda(q) = \left(J_{xz}(q)\,M(q)^{-1} J_{xz}(q)^T + \lambda_{damp} I\right)^{-1}
$$

- $\Lambda(q)$ -- the operational-space (task-space) mass matrix, same quantity
  `run_hardware.py` computes every tick.
- $M(q)$ is the joint-space mass matrix INCLUDING the reflected-rotor armature correction
  (`dyn_armature_kg_m2`, see `implementation_fix.md`'s Finding-4 entry) -- using the same
  $M(q)$ the controller itself uses is what makes $\tau_{eq}$ a fair read of what the
  controller's own observer believed, not a recomputation with a different model.
- $J_{xz}(q)$ is the 2x3 task-space (x-z rows only) Jacobian.

Each section below is a different way of asking what $\tau_{eq}$ (or, for breakaway, the raw
logged $\tau$) says about $\tau_c$/$b$/$\tau_s$ -- none of them isolate friction cleanly from
"everything else $\hat d$ is also carrying," which is the recurring caveat throughout.

## 1. Static (holding-residual) estimate

Hold-task steady-state $\hat d$, mapped through $\Lambda(q)$/$J^T$ into joint-torque units
(§0's $F$/$\tau_{eq}$ equations). At $\dot q = 0$ and settled ($\ddot q = 0$ too), the
friction model's stiction regime applies -- the controller's own residual IS (approximately)
the friction torque it had to supply to stay put:

$$
\bar{\hat d} = \mathrm{mean}\big(\hat d \text{ over the last } t_{tail} \text{ seconds}\big)
\qquad\Longrightarrow\qquad
\tau_{eq} \approx \tau_{friction}(\dot q = 0)
$$

This is the torque the controller needed on top of its own gravity/Coriolis/mass-matrix model
to hold position at zero velocity -- NOT the breakaway/stiction threshold $\tau_s$ itself (that
needs motion to actually be attempted against it, see §3), and only "approximately" $\tau_c$
since $\hat d$ also carries any small gravity-model residual error, not friction alone.

| run | q_pos | mean d_hat (x, z) | \|F\| [N] | joint torque equiv [Nm] |
|---|---|---|---|---|
| `hw_hold_A_ry1e-8_1` | 124371 | (-0.26, -0.26) | 0.197 | [-0.032, -0.024, -0.001] |
| `hw_hold_A_ry1e-8_2` | 124371 | (-0.77, -0.40) | 0.537 | [-0.090, -0.054, +0.011] |
| `hw_hold_A_ry1e-8_3` | 124371 | (-0.91, -0.16) | 0.616 | [-0.100, -0.043, +0.033] |
| `hw_hold_re_1` | 11293.6 | (-0.35, -1.78) | 0.796 | [-0.083, -0.115, -0.054] |
| `hw_hold_re_2` | 11293.6 | (-0.56, +0.32) | 0.381 | [-0.049, -0.003, +0.038] |

All five land in the same **0.01-0.1 Nm per joint** range -- noisy and run-to-run variable
(nearly 3x spread across repeats of the *same* config, `hw_hold_A_ry1e-8_1` vs `_3`), not a
single clean number. That variability is itself informative: whatever this residual is
capturing (friction, small gravity-model error, or both) is NOT simply "the same stiction
constant every time" -- it depends on exactly where the arm settled and along what path.

![static mode plot](figures/friction/static_hold.png)

`hw_hold_A_ry1e-8_1` plotted above: `d_hat_x` has clearly NOT reached a flat equilibrium even
by the end of this 20s run -- it's still drifting through the shaded 2s tail window used for
the estimate. This run's own number (and by extension, how much to trust any of the five
above) is somewhat tail-window-dependent; a longer hold would likely give a cleaner read.

## 2. Viscous (velocity-correlated) estimate

Moving-task $\hat d$ converted to task-space force via $\Lambda(q)$ (computed per sample, not
one fixed posture, via §0's $F$ equation) and correlated against $\dot x_{ee}$ (`ee_vel`), by
axis. Kept in task space on purpose -- these logs don't record joint velocity, only
$\dot x_{ee}$, and a pseudo-inverse projection into joint space would introduce a null-space
ambiguity this controller's own posture term uses. Tests the viscous term of §0's model
directly, in task space rather than joint space:

$$
F(t) \approx -b_{visc}\,\dot x_{ee}(t) + c
$$

$$
(b, c) = \mathrm{polyfit}\big(\dot x_{ee},\, F,\ \text{degree}=1\big)
\qquad
R^2 = \mathrm{corr}\big(F,\, \dot x_{ee}\big)^2
$$

$c$ absorbs everything velocity-independent (bias, stiction residue, model error -- it is
NOT part of the viscous-friction model itself); $b$ should come out negative if this is
really viscous friction ($b_{visc} = -b > 0$); $R^2$ is how much of $F$'s variance velocity
actually explains.

| axis | corr(F, ee_vel) | R² | fit |
|---|---|---|---|
| x | -0.467 | 0.218 | F ≈ -21.8·vel - 0.34 N |
| z | -0.352 | 0.124 | F ≈ -22.3·vel - 0.34 N |

![viscous mode plot](figures/friction/viscous_circle.png)

Both axes: the right **sign** (force opposes velocity, consistent with viscous friction) and
a clearly nonzero slope -- but R² of 0.12-0.22 means velocity explains at most ~22% of `F`'s
variance. The scatter plot shows why directly: there's a real downward trend, but also a
wide, nearly-vertical band of points right around `vel≈0` with a large spread in `F` --
plausibly the circle's own curvature/direction-reversal points, not a friction effect at all
(see the script's docstring). **This supports "there's probably a viscous-like term," not
"here is its coefficient."**

## 3. Breakaway (stiction) events

Scans logged $q$ for a joint pinned within ~1 encoder tick (XM430-W350: $2\pi/4096$ rad, i.e.
genuinely not moving, not sensor noise) for a sustained run, immediately followed by real
motion. This is the most direct, least-filtered evidence of the three -- a real physical
event (the actual $\tau$ commanded to the servo, not a controller-internal residual), and the
only one of the three that directly tests §0's stiction regime rather than approximating it:

$$
\text{stuck:}\quad |q(t) - q(t_{start})| \le 1.5\,\delta_{tick}
\quad \text{for } t \in [t_{start}, t_{break})
$$

$$
\text{breakaway:}\quad |q(t_{break}) - q(t_{start})| > 3\,\delta_{tick}
\quad \text{(real motion resumes)}
$$

$$
\Delta\tau = \tau(t_{break}) - \tau(t_{start})
$$

$\Delta\tau$ is a proxy for $\tau_s$, NOT $\tau_s$ itself -- it only equals it if
$\tau(t_{start}) = 0$, which it generally isn't (gravity compensation and other joints'
coupling are already baked into $\tau(t_{start})$).

![breakaway mode plot](figures/friction/breakaway_step.png)

The largest event found in `hw_step_A_1.csv`: **joint 0 stuck for 2.16s (t=3.39-5.55s) while
commanded torque ramped smoothly from -0.19 Nm to -0.01 Nm (a 0.179 Nm swing) before the
joint broke free** and moved 0.1 rad in well under a second. The torque trace (bottom panel)
shows no discontinuity anywhere in this window -- it's one continuous ramp from before the
window starts to after it ends, which is exactly why this event is flagged
`likely_dwell` (its total stuck duration exceeds `--likely-dwell-s`, matching the task's own
`step_dwell_s=5.0`): the data genuinely cannot distinguish "the joint was commanded to hold
still and then commanded to move, and happened to need 0.18 Nm of torque change to actually
start moving" from "friction was holding it, and 0.18 Nm is roughly the breakaway threshold."
Both descriptions fit the same trace.

Across the whole log (all 3 joints, `--likely-dwell-s 2.0`): 13 events on joint 0, 12 on
joint 1, 17 on joint 2 -- most flagged `likely_dwell` (duration exceeds the task's own 5s
dwell, so almost certainly just commanded holds, not stiction) or very short with a small
swing (most likely measurement/control noise rather than a real stuck-then-broke-free event).
A handful of short (<1s), larger-swing (0.02-0.04 Nm), *un*flagged events are the more
credible stiction candidates, e.g. joint 2 at t=39.58-40.38s (swing +0.0154 Nm) -- an order of
magnitude smaller than the flagged joint-0 event above, consistent with that one being mostly
(or entirely) a commanded-dwell artifact rather than real stiction.

Running the same scan against `--backend sim` data (`configs/step.yaml`, 15s) finds **zero
events on all three joints** -- expected, since sim has no stiction model at all, and a
useful sanity check that this detector isn't just pattern-matching ordinary noise.

## What this does and does not establish

**Does establish**: real friction-like effects are present and roughly the sizes these three
methods report (holding residual ~0.01-0.1 Nm/joint, a weak-but-real velocity-opposing term,
and stiction-consistent torque swings up to ~0.15-0.18 Nm on short, un-flagged events) --
broadly consistent with the ~0.5 Nm breakaway figure `docs/01_concepts.md` already cites from
an earlier, dedicated measurement (different joint, different test, same order of magnitude).

**Does NOT establish**: a trustworthy per-joint Coulomb/viscous friction MODEL. Every number
above is a proxy filtered through this controller's own observer and its own assumptions --
none of it isolates friction from "everything else the model doesn't capture." The genuinely
clean version of this analysis needs `tools/chirp_response.py --backend dynamixel` run on
real hardware (open-loop, no observer, no feedback at all) -- not done yet, see
`next_steps_test_plan.md` item 3 and `implementation_fix_20261001.md`.
