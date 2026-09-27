# openmanipulator_verify Review

Review date: 2026-09-27
Triggered by: `pHRI/MPC_s` (a 3-DOF, waist-removed fork of this project's own
design) surfaced a long chain of real bugs this session -- once that work
was done, it was worth checking whether the same classes of problem exist
in *this*, the original 4-DOF reference it was forked from. See
`pHRI/MPC_s/original_implementation.md` and `implementation_fix.md` for the
full history this review draws on.

**One fix already applied** (low-risk, directly portable, independently
verified against the project's own MuJoCo validation of record -- see
Finding 1). **Everything else below is a finding, not yet applied** --
several need your input on scope or real-hardware data this environment
doesn't have.

## Finding 1 (applied): `mass_matrix()`'s RNEA-per-column inefficiency

**What was wrong.** Exactly the same issue fixed in `MPC_s` this session:
`mass_matrix(q)` built `M(q)` via `n` separate `_rnea()` calls (one per
unit-acceleration column) instead of a closed-form pass. Measured on this
4-DOF arm: **2.015 ms/call** for `mass_matrix()` alone (worse than `MPC_s`'s
pre-fix number, since this is a 4x4 system with a 4th RNEA call). Added to
the also-per-tick `gravity()` call (~0.49 ms), the shared per-tick cost for
this loop is **~2.49 ms** before the QP/observer even runs -- on hardware
comparable to `MPC_s`'s original PC, this alone could plausibly explain a
similar `~6-8ms`-class per-tick cost.

**What changed.** Replaced with the same standard closed-form Jacobian
formula (Siciliano et al.) already proven in `MPC_s`: one forward-kinematics
pass instead of 4 RNEA passes. The original RNEA-per-column method is kept
as `_mass_matrix_rnea` purely for regression testing.

**Verification (more rigorous than what was possible for `MPC_s`, since
this project already has real independent ground truth available):**

- Bit-identical check against random `q`: max abs diff `1.39e-17` over 300
  samples before adopting the change.
- New permanent regression check in `test_local.py` (mirrors `MPC_s`'s own):
  `mass_matrix()` vs `_mass_matrix_rnea()`, max diff `6.94e-18` over 50
  random samples.
- **Ran the project's own "validation of record"**, `tools/
  verify_against_mujoco.py` (loads the real ROBOTIS URDF into independent
  MuJoCo physics) -- this is a check `MPC_s` never had access to. Full pass
  after the fix:
  ```
  [1] FK vs MuJoCo:        max 8.33e-17 m   OK
  [1] Jacobian vs MuJoCo:  max 4.53e-11     OK
  [2] gravity vs MuJoCo:   max 5.87e-03 Nm  OK
  [2] mass M vs MuJoCo:    max 1.97e-04     OK
  [3] control suite on MuJoCo physics:
      hold     SS= 2.87 mm  max=   8.9 mm  OK
      circle   SS= 1.75 mm  max=  14.2 mm  OK
      push     SS= 3.02 mm  max=  80.3 mm  OK
      payload  SS= 3.23 mm  max=  44.3 mm  OK
  PASS: harness validated against MuJoCo physics
  ```
- Speedup: `mass_matrix()` alone, 2.015 ms -> 0.171 ms (**11.8x**).

**Minor, separate observation (not investigated further):** the above SS
numbers (2.87/1.75/3.02/3.23 mm) are all "OK" but don't exactly match the
specific figures quoted in this project's own README ("hold SS 1.3 mm,
circle SS 2.0 mm..."). Both pass the tool's own threshold, and this fix
doesn't touch anything the README numbers would depend on differently than
before, so this is very likely pre-existing run-to-run variance (unseeded
randomness in the sim disturbance/observer init, or the README being from
an earlier parameter set) -- flagged in case you want the README's specific
numbers refreshed, not something this fix changed.

## Finding 2: no Coriolis/centrifugal compensation at all

`lib/dynamics.py` has no `coriolis()` method, and `run_openmanipulator_
hardware.py`'s feedforward torque is `tau = tau_task + N@tau_post +
g_scale*dyn.gravity(io.q)` -- no `C(q,dq)dq` term anywhere, and no
task-space Coriolis/centrifugal decoupling term (`mu_xz` in `MPC_s`'s
design) either.

`MPC_s` (the later fork) explicitly added both, with a documented reason:
"Without it, C(q,dq)dq would be left inside the disturbance estimate d_hat
for the observer to absorb." Whether that omission actually matters here
depends on how fast this arm moves in the verification suite (`hold`/
`circle`/`push`/`payload` all look like slow, small-motion tasks per the
MuJoCo run above, where Coriolis/centrifugal terms are genuinely small) --
so this may be an intentional, reasonable simplification for this task set
rather than a bug, matching the report's own framing elsewhere in this
project's family: "d(t) is not an independent physical quantity -- it is
defined as whatever the model's own cancellation fails to remove."

**Recommendation, not applied:** port the same `coriolis()` method and the
`mu_xz` task-space decoupling term from `MPC_s`'s `lib/dynamics.py`/
`run_hardware.py` if this project's own task set is ever extended to faster
motions (its current `circle.yaml`, if it uses a much shorter period than a
gentle verification circle, would be the first place this would start to
matter) -- low effort given the `MPC_s` implementation to copy from, but a
real design decision either way, so left for you.

## Finding 3: dead code and a stale docstring

`run_openmanipulator_hardware.py` line 83: `m_eff = float(robot.get(
"task_mass_kg", 0.6))` is assigned and **never used anywhere else in the
file** -- a leftover from an earlier, simpler control law. The module's own
docstring (line 8) still describes that earlier law: `F_task = m_eff
(ddp_d + u)` -- but the actual code (line 134) uses `F_task = ramp * (Lam @
(ddp_d + u))`, the physically-correct `Λ(q)`-based version. The docstring
is stale by exactly the amount the dead variable suggests.

**Recommendation:** delete the unused `m_eff` line, and fix the docstring's
control-law description to match the actual `Λ(q)`-based line. The
`task_mass_kg` config key itself is a separate matter -- `grep -rn
task_mass_kg .` shows it's still set (to `0.6`) in **all four**
`configs/*.yaml` files, just never read by anything after this line change
would remove its only reader. So this is two dead things, not one: a dead
Python variable now, and a dead config key across four files once that
variable is removed. Low-risk either way, but not applied here since it
touches the file's own top-of-file contract statement and a config key that
exists in every shipped config -- worth your own confirmation before
removing something that visible.

## Finding 4: the same reflected-rotor/gearbox inertia gap `MPC_s`'s own
report found -- and it's shared with this project's MuJoCo ground truth too

This is the most consequential finding. `MPC_s`'s own underlying stage
report found that the XM430-W350 servo's reflected rotor/gearbox inertia
(missing from a link-geometry-only mass matrix) causes a 137.0{deg} ->
2.87{deg} error in a real-hardware open-loop torque-replay test, fixed by
adding a constant `c=0.01 kg m^2` to `M(q)`'s diagonal.

This project's `lib/dynamics.py` `LINKS` masses (`0.09841, 0.13851,
0.13275, 0.14328`) are **byte-identical** to `MPC_s`'s own (after it dropped
the waist link) -- both come from the same upstream ROBOTIS URDF, confirmed
directly:

```
$ grep -n "<mass value=" .../open_manipulator_x.urdf
9.8406837e-02, 1.3850917e-01, 1.3274562e-01, 1.4327573e-01
```

**And that URDF has no `armature` attribute on any joint** (checked
directly in the cached file) -- meaning `lib/mujoco_sim.py`'s MuJoCo
"ground truth" physics is *also* built from a model with no reflected-rotor
inertia. So the README's claim "gravity G(q) + mass matrix M(q) vs MuJoCo --
to ~1%" is accurate, but doesn't actually check against the real physical
hardware response -- it checks the controller's model against a simulation
that shares the exact same omission. The 137{deg} error the sibling
project's real-hardware test caught would not show up in this project's own
MuJoCo-based "validation of record" at all, because MuJoCo is missing the
same term.

**Not fixed here, and can't be from this environment:** the correction
this needs is a *different* constant than `MPC_s`'s `c=0.01` -- reflected
rotor inertia does scale with the same servo family, but the right value
for THIS arm's own dynamics has to come from the same kind of real-hardware
open-loop identification test `MPC_s`'s report ran (Section 2.5 there: jog
each joint with a torque sine, replay the logged torque through the model,
sweep a candidate constant against the 0.1s-horizon prediction RMSE). I
don't have real hardware access in this environment to run that test.
**Recommendation:** run that identification test on this arm before trusting
either the MuJoCo-based validation or a real-hardware run at anything but
very low speed/stiffness -- and consider adding an `armature` attribute to
a local copy of the URDF used for the MuJoCo backend too, so the sim
ground truth stops sharing the blind spot.

## Finding 5 (safety-relevant): missing hardware-communication hardening present in `MPC_s`

`lib/dynamixel_backend.py`'s `DynamixelCurrentBackend.read_state()` here is
much simpler than `MPC_s`'s own (which grew substantial hardening,
apparently from real hardware debugging experience, that was never
backported here):

- **No `isAvailable()` freshness check** on the `GroupSyncRead` response --
  `MPC_s`'s own comment explains why this matters: "`isAvailable()` only
  means new bytes arrived... it does NOT mean the bytes decode to a sane
  value. A corrupted/all-zero packet... will decode to q = -joint_offset_rad
  for every joint." This backend has no equivalent check at all.
- **No implausible-single-step-jump rejection** -- `MPC_s` rejects any
  joint reading that jumps more than a physically-plausible amount in one
  tick (a corrupted read symptom), reusing the last known-good state
  instead and raising after too many consecutive rejections. Absent here.
- **No startup multi-turn-wrap sanity check** (`assert_startup_pose_
  plausible` in `MPC_s`) -- the Present_Position register is a multi-turn
  signed count; a wrap after a power cycle reads as a physically impossible
  pose that FK alone cannot catch (FK is 2{pi}-periodic). `MPC_s` refuses to
  energize the arm if the read pose is outside the joint range before
  torque is ever enabled. This project enables torque without that check.

**This is the highest-priority item in this review**, since it's the one
that's actually safety-relevant on real hardware rather than a performance
or code-quality concern, and the fix already exists, verified, in `MPC_s`'s
own `lib/dynamixel_backend.py` -- it would need adapting from 3-DOF to
4-DOF (should be mechanical, the logic is dimension-generic) but not
redesigning. Not applied here without your go-ahead, since it's a
non-trivial port and I'd want you to confirm before I touch the real-
hardware code path.

## Finding 6: no gain-tuning tooling, no tuning guide

There's no equivalent of `MPC_s`'s `NormalizedInteractionMPC.effective_gain(
)` (the closed-form unconstrained gain, needed to know what task-space
stiffness a given `q_pos`/`q_vel`/`r` actually delivers -- see `MPC_s`'s
Finding 2/its fix for why this isn't obvious from the weights alone), no
`tools/solve_task_space_gain.py`-equivalent, and no `docs/`
folder/tuning-guide at all. Tuning this controller currently means the same
kind of ad hoc numerical work `MPC_s` needed a dedicated tool to avoid
guessing at.

**Recommendation, not applied:** port `effective_gain()` and
`solve_task_space_gain.py` if/when this project's own gains need retuning --
both are dimension-generic (built from `cfg.dim`, not hardcoded), so the
port should be closer to a copy than a rewrite.

## Finding 7: no `--use-jit` option

Given this arm's `mass_matrix()`/`_rnea()` cost is *higher* than `MPC_s`'s
(4-DOF vs 3-DOF), the same Numba-JIT option (measured ~65-79x on the RNEA
path, ~4-5x whole-tick, in `MPC_s`) would likely help proportionally more
here. Not applied -- same reasoning as `MPC_s`'s own `--use-jit` addition:
it's a new optional dependency and a real "is this needed" decision, not
something to add silently.

## What's already fine -- verified, not just assumed

- **`lib/interaction_mpc.py`'s box-constrained receding-horizon QP (FISTA,
  warm-started)** is already the *correct*, faithful design -- this is
  exactly what `MPC_s` had regressed away from and had to have restored
  *from this file*. No changes needed here; if anything, this file is the
  one to keep treating as the reference.
- **FK/Jacobian**, independently re-verified above via
  `tools/verify_against_mujoco.py`: machine-precision agreement with MuJoCo.
- **`gravity()`** likewise verified to `5.87e-03 Nm` against MuJoCo (a
  looser but still tight tolerance, appropriate for a nonlinear trig
  quantity vs the mass matrix's linear one).

## Summary: suggested priority if you want to act on these

1. **Finding 5** (hardware-communication safety hardening) -- highest
   priority, safety-relevant, fix already exists and is proven in `MPC_s`.
2. **Finding 4** (reflected-rotor inertia) -- highest *impact* on result
   validity, but needs a real-hardware identification test this environment
   cannot run; flag as a known gap in the meantime.
3. **Finding 3** (dead code / stale docstring) -- trivial, low-risk, do
   whenever convenient.
4. **Finding 6** (tuning tooling) -- worth porting before the next time
   gains need changing, not urgent otherwise.
5. **Finding 2** (Coriolis) and **Finding 7** (`--use-jit`) -- genuine
   design/dependency decisions, only worth it if this project's own task
   set or performance requirements actually need them.
