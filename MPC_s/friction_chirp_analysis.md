# Friction and chirp analysis

Two related, independently-run diagnostics on the real OpenManipulator-X, written up
together because both exist to characterize what this project's dynamics model does NOT
capture (`docs/01_concepts.md`: "the simulator has no friction model... the friction part
of the story needs real hardware"):

- **§0-3 (friction, `tools/estimate_friction.py`)**: three PROXY friction estimates read off
  EXISTING closed-loop logs -- cheap, available now, but each filtered through the
  controller's own disturbance observer, not a clean measurement.
- **§4 (chirp, `tools/chirp_response.py`)**: an open-loop (no feedback at all) swept-sine
  torque diagnostic -- the one approach that COULD give a clean, joint-space,
  closed-loop-free measurement, including of friction, but whose real purpose here is
  distinguishing a genuine mechanical resonance from a closed-loop-delay effect (see §4).
  Only run in sim so far; the real-hardware run is what would actually answer that question.

Treat everything below as "here's what's been learned so far," not a friction model ready to
drop into `lib/dynamics.py`, and not yet an answer to the resonance question in §4.

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

## What §1-3 do and do not establish

**Does establish**: real friction-like effects are present and roughly the sizes these three
methods report (holding residual ~0.01-0.1 Nm/joint, a weak-but-real velocity-opposing term,
and stiction-consistent torque swings up to ~0.15-0.18 Nm on short, un-flagged events) --
broadly consistent with the ~0.5 Nm breakaway figure `docs/01_concepts.md` already cites from
an earlier, dedicated measurement (different joint, different test, same order of magnitude).

**Does NOT establish**: a trustworthy per-joint Coulomb/viscous friction MODEL. Every number
above is a proxy filtered through this controller's own observer and its own assumptions --
none of it isolates friction from "everything else the model doesn't capture." The genuinely
clean version needs §4's tool run for real, which is where this picks up.

## 4. Open-loop chirp diagnostic (`tools/chirp_response.py`)

Different purpose from §1-3, and a different design specifically so it can answer what they
can't: `tools/chirp_response.py` commands ONLY `tau = gravity_scale*dyn.gravity(q) -
damping*dq + chirp(t)` on one joint at a time -- NO disturbance observer, no feedback law, no
controller of any kind. Whatever shows up in the response is a property of the physical arm,
not of this project's control code. Primary motivation is the real-hardware question from
`next_steps_test_plan.md` item 3: is the ~7.4-7.9Hz self-excited closed-loop oscillation
(`divergence_analysis.md`) a genuine mechanical resonance, or a closed-loop critical frequency
from total loop delay? A real amplitude peak in that band on the open-loop plant would support
the former; a flat response there, with the closed-loop oscillation still real, would support
the latter. (It would also, incidentally, be the only clean way to fit §0's $\tau_c$/$b$/$\tau_s$
-- a joint-space, friction-isolated measurement, unlike anything in §1-3 -- but that fit hasn't
been done, and needs the real-hardware CSVs below to exist first.)

**Status: run on all 3 joints in `--backend sim` only so far (2026-10-02). The real-hardware
run -- the one that actually answers the resonance question -- has not been done; this
session has no hardware access (confirmed: no USB-serial/Dynamixel adapter present).**

```bash
python3 tools/chirp_response.py --backend sim --config configs/hold.yaml \
    --joint {0,1,2} --duration 30 --output results/chirp/sim_j{0,1,2}.csv
python3 tools/chirp_response.py --plot results/chirp/sim_j{0,1,2}.csv \
    --plot-output figures/chirp/sim_j{0,1,2}.png
```

All three joints: 3000/3000 samples, no auto-stop, deviation from the starting angle stayed
under ~0.14 rad throughout (well under the 0.3 rad default `--max-dev-rad`).

![joint 0 response](figures/chirp/sim_j0_response.png)
![joint 1 response](figures/chirp/sim_j1_response.png)
![joint 2 response](figures/chirp/sim_j2_response.png)

All three: a smooth, monotonically decreasing response with no peak anywhere, including
inside the red 7.4-7.9Hz reference band. Exactly what's expected -- sim's rigid-body model
has no mechanical resonance to find, so this is NOT evidence against the resonance
hypothesis, only confirmation that the collection/analysis pipeline itself works correctly
end to end (chirp injection, the safety abort, logging, and `--plot`'s response-vs-frequency
extraction) before ever risking it on the real arm:

![joint 1 trace](figures/chirp/sim_j1_trace.png)

The joint-1 raw trace above (q/dq/tau) shows the expected shape independent of any resonance
question: a clean 3-15Hz sweep, response amplitude rolling off smoothly as frequency rises
(ordinary inertia, not a resonance), torque staying well inside `tau_max_Nm`.

**Next step, unchanged from `next_steps_test_plan.md` item 3**: the same three commands with
`--backend dynamixel --port <port>` on the real arm, then the same `--plot` comparison against
the 7.4-7.9Hz band -- this is the one piece of data in this whole file that can actually
distinguish the two hypotheses, and it doesn't exist yet.
