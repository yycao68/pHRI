# Friction analysis (real hardware data)

Produced by `tools/estimate_friction.py` (see its own docstring for the full method and
caveats behind each number here -- this file is the write-up, not a restatement of it).
This project's dynamics model has no friction term at all (`docs/01_concepts.md`), so
everything below is a PROXY read off existing closed-loop logs, not a clean measurement --
the one thing that could give a clean, joint-space, closed-loop-free measurement is
`tools/chirp_response.py --backend dynamixel`, which hasn't been run on real hardware yet.
Treat this as "here's what the existing data already hints at," not a friction model ready
to drop into `lib/dynamics.py`.

## 1. Static (holding-residual) estimate

Hold-task steady-state `d_hat`, mapped through `Λ(q)`/`J^T` into joint-torque units. This is
the torque the controller needed on top of its own gravity/Coriolis/mass-matrix model to
hold position at zero velocity -- NOT the breakaway/stiction threshold (see §3 for that).

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

Moving-task `d_hat` converted to task-space force via `Λ(q)` (computed per sample, not one
fixed posture) and correlated against `ee_vel`, by axis. Kept in task space on purpose --
these logs don't record joint velocity, only `ee_vel`, and a pseudo-inverse projection into
joint space would introduce a null-space ambiguity this controller's own posture term uses.

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

Scans logged `q` for a joint pinned within ~1 encoder tick (XM430-W350: `2π/4096` rad, i.e.
genuinely not moving, not sensor noise) for a sustained run, immediately followed by real
motion. This is the most direct, least-filtered evidence of the three -- a real physical
event, not a controller-internal residual.

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
