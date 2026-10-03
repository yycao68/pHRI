# MPC_s Implementation Fixes

This file tracks fixes for the problems recorded in `docs/original_implementation.md` 
-- both applied and merely recommended, clearly labeled as such. Do not add new problem reports here; add them to `docs/original_implementation.md` instead.

**This file covers 2026-09-27 only -- all sim/analysis-only fixes made before any real hardware data existed.** Fixes from 2026-09-29 onward (everything that came out of the real-hardware validation data in `MPC_s_Hardware_results_2026-09-29/`, and everything downstream of it) are tracked separately in `docs/implementation_fix_20261001.md`, to keep the pre-hardware and post-hardware stories from blurring together. Add new fixes to whichever file matches when they happened -- most likely `docs/implementation_fix_20261001.md` now.

## Background: what RNEA is (read first -- the fixes below refer to it constantly)

**RNEA** is the **Recursive Newton-Euler Algorithm**, the standard efficient way to compute *inverse dynamics*: given joint positions, velocities and accelerations (`q`, `dq`, `ddq`), it returns the joint torques needed to produce that motion,

$$\tau = M(q)\,\ddot q + C(q,\dot q)\,\dot q + g(q)$$

without ever forming `M`, `C` or `g` as separate matrices. It does this in two sweeps along the kinematic chain (hence "recursive"). In `lib/dynamics.py` it is `_rnea()`, with a Numba-compiled twin `_rnea_jit()`.

1. **Forward pass (base -> tip).** For each link `i`, the parent's motion is rotated into the current link frame (`iR = R[i].T`) and the joint's own contribution is added: angular velocity `w[i+1] = iR @ w[i] + dq[i]*axis`, angular acceleration `wd`, and linear acceleration `a` at the joint, then `a_ci` at the link's centre of mass. From these, each link's required force and moment: `F = m * a_ci` (Newton) and `N = I @ wd + w x (I @ w)` (Euler).
2. **Backward pass (tip -> base).** Forces and moments are accumulated inward: each link must supply its own `F`, `N` plus everything the links beyond it need. Projecting each joint's moment onto its axis gives `tau[i]`.

Gravity is handled by a standard trick: the base is given an upward acceleration, `a[0] = -g_vec * gravity_scale`, which is equivalent to gravity pulling down on every link.

**One routine, every dynamics term.** By zeroing some inputs, the same RNEA call isolates each term:

| call | result |
|---|---|
| `_rnea(q, 0, 0, gravity=True)` | gravity torque `g(q)` -- `dyn.gravity()` |
| `_rnea(q, dq, 0, gravity=False)` | Coriolis/centrifugal `C(q,dq)dq` -- `dyn.coriolis()` |
| `_rnea(q, 0, e_j, gravity=False)` | column `j` of the mass matrix `M(q)` -- `_mass_matrix_rnea()`, one call per column |

That last row is why the original code paid 3 RNEA calls per tick just for `M(q)` on this 3-DOF arm (`n = 3`, OpenManipulator) -- the target of the first fix below.

**`_rnea_jit()`** is the same algorithm step for step, compiled with Numba (`@njit`) on plain arrays, with the rotation-matrix construction inlined (a njit function can only call other njit'd functions). The pure-Python `_rnea()` is bound by Python/numpy per-call overhead on many tiny 3x3 operations, not by the arithmetic itself, so compiling it removes most of the cost. It is verified against `_rnea()` to ~1e-14 (floating-point noise). See the `--use-jit` section below.

## Applied: the ~6.8ms per-tick compute cost (2026-09-27)

**Status: APPLIED and verified.**

The user reported that even the **PID** controller was taking ~6.8 ms/tick on the real control PC -- close to the 10 ms (100 Hz) budget. This was tackled first since it affects every controller equally (per the README, `run_hardware.py` and `run_pid.py` share everything except the feedback law) and directly explains the original report's own "loop-gain delay" concerns (§4.2).

**Root cause.** `benchmark_compute.py --controller pid` and `--controller mpc`
measured near-identical cost on this dev machine, confirming the bottleneck is not the feedback law at all but the *shared* per-tick rigid-body computation both runners do regardless of controller. Profiling that shared path (`kin.jacobian`, `dyn.mass_matrix`, `dyn.gravity`, `dyn.coriolis`) isolated the cost almost entirely to `_rnea()`, the RNEA (Newton-Euler) routine: each call costs ~0.37 ms on this machine, dominated by Python/numpy per-call overhead rather than actual FLOPs (this is a 3-DOF arm; the arithmetic itself is tiny). The old code issued **6 RNEA calls per tick**:

- `mass_matrix(q)`: 3 calls (one per unit-acceleration column -- the classic but wasteful "RNEA-per-column" way to get `M(q)`);
- `dyn.coriolis(io.q, io.dq)`: called **twice** -- once for the task-space Coriolis/centrifugal decoupling term `mu_xz`, and again (redundantly, for the identical quantity) building `tau_base` right below it;
- `dyn.gravity(io.q)`: 1 call.

**What changed (all changes preserve exact numerical behavior, verified below):**

1. `lib/dynamics.py`: `mass_matrix(q)` now uses the standard closed-form Jacobian formula (Siciliano et al., one forward-kinematics pass) instead of 3 RNEA calls -- eliminates those 3 calls entirely. The original RNEA-per-column approach is kept as `_mass_matrix_rnea` purely as a regression reference.
2. `run_hardware.py` and `run_pid.py`: the duplicate `dyn.coriolis()` call is now computed once (`cor = dyn.coriolis(io.q, io.dq)`) and reused for both `mu_xz` and `tau_base` -- removes 1 of the remaining 3 calls.
3. `lib/dynamics.py`: added `bias_force(q, dq, gravity_scale)`, which gets `gravity_scale*G(q) + C(q,dq)dq` in a single RNEA pass (exploiting that RNEA's forward recursion is exactly linear in the base acceleration term). Not wired into `run_hardware.py`/`run_pid.py` since both need the *raw* `C(q,dq)dq` separately for `mu_xz` anyway, but available for any future caller that only wants the combined bias torque.
4. `tools/benchmark_compute.py`: updated to include the `mu_xz`/`Jdot_xz` terms it was previously missing, so its reported number now matches what `run_hardware.py` / `run_pid.py` actually do per tick (previously it under-counted the real cost).
5. `test_local.py`: added a regression check asserting `mass_matrix()` matches `_mass_matrix_rnea()` and `bias_force()` matches `gravity_scale*gravity()+coriolis()`, over random samples.

Net: 6 RNEA-equivalent calls/tick down to 2 (`coriolis` once, `gravity` once) plus one cheap Jacobian-formula mass-matrix pass.

**Verification.**

- `test_local.py`'s new regression check: max error `~1e-16`--`~1e-18` between the fast and slow paths over 50 random `(q, dq, gravity_scale)` samples -- floating-point noise, not approximation.
- `test_local.py`'s existing sim hold-loop checks (MPC and PID) still pass with unchanged steady-state error.
- `python3 -m py_compile` on every touched file: clean.
- Direct before/after timing of the exact per-tick sequence:

  | | mean/call |
  |---|---:|
  | old (as shipped: 3+2+1 = 6 RNEA calls) | 2.348 ms |
  | new (this fix: 2 RNEA calls + fast `mass_matrix`) | 0.960 ms |
  | **speedup** | **2.45x** |

Post-fix, both controllers benchmark at ~1.0 ms mean / ~1.2 ms p99 on this dev machine (`configs/hold.yaml`, 100 Hz budget) -- comfortable margin. Extrapolating the measured 2.45x from the user's reported ~6.8 ms real-hardware PID figure suggests roughly **~2.8 ms** post-fix, before adding real serial I/O -- worth reconfirming with `tools/benchmark_compute.py` and a real `--backend dynamixel` run on that machine.

**Not addressed by this pass:** `kin.jacobian()` is still a numerical (finite-difference, 6 `fk()` calls) Jacobian, costing ~0.08 ms here -- a much smaller remaining share now that the RNEA calls are down to 2, but a further target (an analytic Jacobian) if more margin is ever needed.

## Applied: Findings 1 & 2 -- restored the receding-horizon box-QP (fix A) (2026-09-27)

**Status: APPLIED (structural fix). Gain re-derivation is a separate, still-open follow-up -- see below.**

Addresses `docs/original_implementation.md`'s Finding 1 (the shipped "MPC" was a static LQR gain, not the report/paper's receding-horizon box-constrained QP). Chose option (A) from the two listed below the fold: restore the real QP, rather than (B) re-deriving gains to match the static-LQR shortcut.

**What changed.** `lib/interaction_mpc.py`'s `NormalizedInteractionMPC` was replaced with the already-validated box-QP implementation from `pHRI/openmanipulator_verify/lib/interaction_mpc.py` (same robot family, same controller, previously MuJoCo-validated), adapted to this project's own conventions rather than copied verbatim:

- Kept this project's `dim=2` (x-z task space only) and its `ControllerConfig`/`RandomWalkDisturbanceObserver` as-is (the observer's `d_hat_max` anti-windup clamp and configurable `q_vel` are improvements over the reference, not regressed).
- Added `qp_iters: int = 200` to `ControllerConfig` (FISTA iteration count, matching the reference's default) and wired it through `run_hardware.py`'s `controller_config()` and `tools/benchmark_compute.py` (optional, `configs/*.yaml` need not set it).
- The QP is solved every tick by warm-started FISTA (accelerated projected gradient) rather than OSQP -- matrix-free, no new dependency, and specifically chosen because it is cheaper than the OSQP-based version the original report measured at ~8.2 ms mean / 99th-percentile over budget (see below for the actual number here).
- The static Riccati-recursion gain `K0` and the plain `u = -K0 x - d_hat` feedback law are gone; every tick now re-solves a box-constrained QP over the full horizon (`-u_max <= u_k <= u_max` at every predicted step, not just the first), with the same offset-free input-centering trick (`V = U + d_seq`) documented in `phri_combined.tex`.

**Verification.**

- `python3 -m py_compile` on every touched file: clean.
- `test_local.py`: all checks pass, including the dynamics regression check from the timing fix above (unaffected by this change) and both sim hold-loops (MPC steady-state error 0.02 mm, PID 0.08 mm -- MPC actually *improved* slightly versus the static-LQR shortcut's 1.02 mm on this same smoke test, not just "still works").
- Full `run_hardware.py --backend sim` runs (not just `test_local.py`'s simplified harness) for all four configs:
  - `hold.yaml` (8 s): converges to ~0.00 mm, holds ~100 Hz throughout.
  - `push.yaml` (20 s): both pushes recover to sub-mm error afterward (though the first push's ~268 mm peak reflects the still-unfixed Finding 2 gain softness, not this change -- see follow-up below).
  - `payload.yaml` (20 s): recovers to genuinely offset-free (0.00 mm by the end), confirming the QP's offset-free property survived the port.
  - `circle.yaml` (15 s): stable moving-reference tracking, settles to ~0.02--0.03 mm, no divergence, ~100 Hz maintained throughout.
- Compute cost (the reason OSQP was apparently dropped in the first place):
  `tools/benchmark_compute.py --controller mpc` across all four configs, after the timing fix above, now reads:

  | config | mean | p99 | budget headroom at p99 |
  |---|---:|---:|---:|
  | `hold.yaml` | 1.98 ms | 2.34 ms | 7.7 ms |
  | `push.yaml` | 1.98 ms | 2.22 ms | 7.8 ms |
  | `payload.yaml` (horizon=50) | 2.19 ms | 2.46 ms | 7.5 ms |
  | `circle.yaml` | 1.96 ms | 2.34 ms | 7.7 ms |

  All comfortably inside the 10 ms (100 Hz) budget -- FISTA at 200 iterations costs roughly ~1 ms on top of the PID baseline's ~1.0 ms (from the timing fix above), nowhere near the original OSQP-based report's ~8.2 ms.

**Then still open, now fixed below: the gains themselves.** Restoring the QP *structure* did not by itself fix Finding 2 (the 15--250x-too-soft effective stiffness) -- the `configs/*.yaml` `q_pos`/`q_vel`/`r` values at this point were still never derived for either the old static-LQR shortcut or this restored QP; they were simply what happened to be left in the files. `push.yaml`'s ~268 mm peak excursion above was consistent with this -- not a new problem, but Finding 2 showing through unchanged. See "Applied: Finding 2 -- gain retune, and Finding 3 -- the `u_max` fix" below for how this was resolved (with a real complication: the report's raw `Kp≈101.39 N/m` target turned out not to transfer directly to this posture's own, quite different task inertia).

## Applied: Finding 2 -- gain retune, and Finding 3 -- the `u_max` fix (2026-09-27)

**Status: APPLIED and sim-verified. Real-hardware validation is explicitly NOT part of this fix -- see the caveat below; treat this as a well-evidenced starting point, not a final number.**

### Why this needed a new tool, not just a number

Fix (A) restored the QP *structure* but left `configs/*.yaml`'s `q_pos`/`q_vel`/`r` untouched, so Finding 2's softness was expected to persist (as that entry said). Re-deriving them turned out to be less simple than "target the report's Kp=101.39 N/m": that number was validated at the *report's own* posture, whose task inertia `Λ(q)` (Λxx≈0.67, Λzz≈0.43) is quite different from this project's own shared posture (`Λ=[[0.201, 0.076], [0.076, 0.052]]` -- notably smaller and more anisotropic, `Λzz` alone ~8x smaller). Both a raw-Kp match and a stability-margin match were tried directly against the report's own `|L|` metric,

$$
|L|_{ii} = \frac{\sqrt{K_{p,ii}^2 + (\omega K_{d,ii})^2}}{\Lambda_{ii}\,\omega^2}, \qquad \omega = 50.27\ \mathrm{rad/s}\ \ (f_{crit}=8\,\mathrm{Hz})
$$

evaluated at the report's own observed oscillation frequency $\omega$ (full derivation, caveats, and why $\omega$ is pinned at this old value rather than re-measured: `hardware_results_review.md` §3):

| target | resulting q_pos (hold.yaml) | Kp_zz | stability metric `\|L\|` |
|---|---:|---:|---:|
| match report's Kp=101.39 N/m directly | 1,206,539 | 101.4 | **1.463** (5.6x the report's own validated max, 0.261) |
| match report's own `\|L\|`=0.21 at its `w` | 508 | 2.9 | 0.210 (barely above the pre-retune value) |

Neither extreme is usable as-is: the first is very likely unstable by the report's own criterion; the second is barely an improvement over the 15-250x-too-soft gains Finding 2 documented. **This is itself a real refinement to Finding 2**, not just an implementation detail: "match the report's raw Kp number" was never the right target once the posture's own `Λ(q)` differs this much -- neither is now recorded as a discovered problem in `docs/original_implementation.md` (which stays as a record of the code as it stood at review time); it is recorded here because it is specific to *how* Finding 2 gets fixed.

### What changed

1. `lib/interaction_mpc.py`: added `NormalizedInteractionMPC.effective_gain()`, the box-QP's closed-form *unconstrained* first-step gain (`u = -K_eff @ x` whenever the box constraint doesn't bind) -- the QP-era replacement for the old static-LQR's `.K0` attribute, needed both for the new tool below and because `docs/02_tuning_guide.md` section 1/4 already prescribed reading `.K0` off the controller and had gone stale after Fix (A) without anyone noticing (both references there now point to `.effective_gain()`).
2. New `tools/solve_task_space_gain.py`: computes `Lambda(q)` at a config's own posture, then either reports the *current* gain's `Kp`/`Kd`/`|L|`, solves for `q_pos` reproducing a target `Kp_zz` or a target `|L|`, or prints a sweep table across candidate targets -- operationalizing Finding 2's "must be solved for, not guessed" into actual reusable code, and carrying the `|L|`-caveat below directly in its own module docstring so it can't be read in isolation from that caveat.
3. `configs/hold.yaml`, `push.yaml`, `payload.yaml`, `circle.yaml`: `q_vel` set to `0` in all four (the report's own Section 4.1 "Property 2": decouples stiffness from damping, previously tied together by each config's own `q_vel = q_pos/const` convention). `q_pos` solved (via the new tool) for a **uniform target `|L| = 0.45`** across all four -- chosen because it is not a new, untested number: it is almost exactly `payload.yaml`'s own *pre-retune* `|L|` (0.434), which restoring the QP (Fix A) had already pushed there "for free" via the QP's Riccati terminal-cost term, and which ran cleanly in sim with no sign of oscillation. `hold.yaml`/`push.yaml` were previously the clear outliers (`|L|=0.149`, 3-4x softer than `payload.yaml`/`circle.yaml` were already running at); this brings all four to the same, already-exercised level rather than inventing a separate new one. Resulting `Kp`/`Kd` at this posture were reported at the time as `Kp≈[49.2, 12.7] N/m`, `Kd≈[4.45, 1.15] Ns/m` -- **since corrected, see the armature-correction fix below**: that `Λ(q)` was computed without the reflected-rotor inertia correction (Finding 4), understating it by 240-639%. The `q_pos` values chosen here did not need to change (the `|L|=0.45` target turns out to be exactly invariant to `Λ(q)` -- see below), but the true `Kp`/`Kd` these `q_pos` values actually deliver are substantially higher than reported at the time; see the corrected numbers in that section.
4. Finding 3 (`u_max` unit mismatch), fixed in the same pass: all four configs now use `u_max: [100, 100, 100]`, replacing `hold.yaml`/ `push.yaml`'s old `[8,8,8]` and `payload.yaml`/`circle.yaml`'s unconverted-force-domain `[300,300,300]`. 100 was not guessed either -- see verification below.

### Verification

- `python3 -m py_compile` on every touched file: clean.
- `test_local.py`: all checks pass unchanged.
- Full `run_hardware.py --backend sim` runs, all four configs, **twice** (once exposed a real problem the second run then verified fixed):
  - **First pass** (gains retuned, `u_max` left at each config's prior value) found a genuine regression: `payload.yaml`'s sustained-force disturbance no longer recovered to zero -- it plateaued at a persistent ~14 mm error. Inspecting the logged `u` showed the QP's own box constraint pinned at exactly its bound (`u_2 = 40.0`, unmoving) from the transient onward while `d_hat` kept drifting -- the box constraint was now genuinely binding (unlike the original report's own regime, where it never did), breaking the offset-free property. `push.yaml` was found to bind too (`u` pinned at exactly `8.0`, its old value) during its transient, though it still fully recovered there (briefly bound, not persistently).
  - Measured the natural (unconstrained) peak correction needed across all four configs' own disturbance profiles: ~51 m/s^2 (`payload.yaml`) and ~41 m/s^2 (`push.yaml`, at a temporarily-widened bound). Set `u_max=[100,100,100]` uniformly (~2x margin over the largest of these).
  - **Second pass**, same four configs, same disturbances, with the fixed `u_max`: all four now fully recover to 0.00 mm. `push.yaml`'s peak transient error also dropped from 303 mm to 103 mm (the same disturbance, just no longer artificially amplified by an under-sized box constraint). `circle.yaml`'s tracking is unaffected (its natural peak, ~0.5 m/s^2, was never close to either bound).
- Compute cost re-checked after the gain changes (`tools/benchmark_compute.py --config configs/payload.yaml --controller mpc`): 2.15 ms mean / 2.47 ms p99, unchanged from Fix A's numbers, as expected -- gain values don't affect FISTA's iteration cost.

### What this fix does NOT establish -- read before running on real hardware

This is a sim-verified, evidence-based, uniform starting point across all four configs -- **not a hardware-validated final operating point**, and it should not be treated as one:

- The `|L|=0.45` target is a deliberate, informed choice (matching an already-exercised level in this exact codebase), not a re-derivation of the true stability boundary at this posture. The original report's own `w=50.27 rad/s` oscillation frequency was itself a property of its ~30 ms round-trip loop delay, dominated by an ~8 ms OSQP solve that no longer exists in this codebase (FISTA now costs ~1 ms, from the timing fix above) -- the TRUE critical frequency and stability boundary for the current, much-faster loop have not been re-measured, and could plausibly be higher (more permissive) OR could behave differently for reasons this offline analysis cannot see (real motor bandwidth, communication jitter, real stiction dynamics sim does not model).
- Resolving this properly needs an actual hardware stability sweep at this posture, mirroring the original report's own Section 6.1 methodology -- out of scope here (no hardware access in this review environment).
- Follow `docs/02_tuning_guide.md` section 3 step 5 before trusting this at full authority: verify in `--backend sim` first (done above), then move to `--backend dynamixel` changing one gain at a time in small increments, watching specifically for the ~8 Hz self-excited oscillation symptom the original report describes -- not just for a large error, which sim can already rule out but real hardware cannot.

## Prepared, awaiting real hardware: read/compute/send timing split (2026-09-27)

**Status: tool written and sim-validated; the actual measurement it exists for still requires `--backend dynamixel` on the real robot.**

The original report's own Table 9 (Section 6.6) explicitly flags an unresolved question next to the timing fix above: "The PID's 6.73 ms is most likely communication-bound rather than computation-bound, since the control law itself is nearly free; that hypothesis has not been tested." The timing fix earlier in this file targeted the *compute* path specifically (redundant RNEA calls) -- its "~2.45x speedup" is only worth what it says if compute was actually the majority of that 6.8ms. If serial I/O (GroupSyncRead/ GroupSyncWrite round trips) dominates instead, the compute fix caps out at a smaller real improvement than the earlier extrapolation implied.

New `tools/benchmark_io.py` splits a real control tick into three separately timed phases -- `read_state()`, the full compute block (kinematics, dynamics, controller, observer -- an exact copy of `run_hardware.py`/`run_pid.py`'s actual per-tick sequence, not a simplified stand-in), and `send_torque()` -- over many ticks, for either controller, on either backend. It prints a mean/p50/p99/max breakdown per phase and states outright, based on which phase actually dominates, whether the report's "communication-bound" hypothesis is confirmed or rejected on that run.

**Sim-validated, with an honest complication found in the process:** running it on `--backend sim` first (to check the script's own logic before hardware access) surfaced that `SimArmBackend.read_state()` is not a free numpy op -- it integrates the plant's own physics (RNEA substeps) before returning, the exact "faking physics" cost `tools/benchmark_compute.py`'s own docstring already warns about. An earlier draft of this tool's docstring incorrectly claimed sim's read/send were "near-free by construction"; that claim was wrong and has been corrected in the file -- on `--backend sim`, "read" time here measures sim-substep integration, not communication, so a sim run validates the script's plumbing only, never the actual question. Confirmed working for both `--controller mpc` and `--controller pid`; `test_local.py` still passes unchanged.

**Next step, not done here:** run `python tools/benchmark_io.py --config configs/hold.yaml --controller pid --backend dynamixel --port <port>` (and the `mpc` variant) on the real hardware, and read off whether `read`+`send` or `compute` dominates the printed breakdown -- that is the actual answer to the report's own open question, and it is not obtainable without real hardware access.

## Applied: Finding 4 -- activated the missing reflected-rotor inertia correction (2026-09-27)

**Status: APPLIED and sim-verified.**

Found while independently fact-checking an external review's citation of the original report's Section 2.5 (a constant `c=0.01 kg m²` added to `M(q)`'s diagonal, dropping joint 3's open-loop prediction RMSE from 137.0° to 2.87°, described in the report as "what every controller in this report actually uses"). The citation was accurate -- but checking it against the current codebase found the correction was never actually active here.

### What was wrong

`run_hardware.py`, `run_pid.py`, and `SimArmBackend` all already read a `dyn_armature_kg_m2` config key (default `0.0`) and add it to `dyn.mass_matrix(q)`'s diagonal before every use -- this is the exact machinery for the report's own correction, complete with a comment ("identified from real hardware data") pointing at `docs/01_concepts.md`. But:

1. **No `configs/*.yaml` file ever set it.** All four configs -- including through this session's earlier gain retune (Finding 2's fix, above) -- silently ran with `dyn_armature_kg_m2=0.0`, i.e. with the report's own decisive correction switched off.
2. `docs/01_concepts.md` does not contain the promised derivation -- the cross-reference was itself stale.
3. `test_local.py`'s own hand-rolled control loop computed `Mq = dyn.mass_matrix(io.q)` directly, never reading `dyn_armature_kg_m2` at all -- so even setting the config key would not have been honored by this test harness; it would have kept testing the controller against a plant model it disagreed with `SimArmBackend` about.

### What changed

- All four `configs/*.yaml`: added `robot.dyn_armature_kg_m2: 0.01` (`hold.yaml` carries the full reasoning; the other three cross-reference it), matching the report's own hardware-identified value -- physically justified here too, since reflected rotor/gearbox inertia is a property of the XM430-W350 servo itself, not of this project's own link geometry, and the same servo is used in both.
- `test_local.py`: `check_sim_hold()` now reads `dyn_armature_kg_m2` from the config and adds it to its own `Mq`, matching `SimArmBackend`'s plant.
- `tools/solve_task_space_gain.py`: `lambda_at_posture()` now adds the same armature term before inverting `M(q)`, matching `run_hardware.py`'s actual `Λ(q)` exactly (it previously used bare `dyn.mass_matrix(q)`, silently computing an incorrect `Λ` -- see impact below).

### Measured impact -- larger than expected, and it retroactively explains part of Finding 2's fix

At this project's shared posture, adding the correct armature term changes `Λ(q)` from `[[0.201, 0.076], [0.076, 0.052]]` to `[[0.686, -0.022], [-0.022, 0.384]]` -- diagonal entries up **240%/639%**, off-diagonal coupling dropping from >100% of the smaller diagonal entry to a small, genuinely secondary term. The corrected `Λ` is now close to the *original report's own* validated-posture `Λ` (`Λxx≈0.67, Λzz≈0.43`) -- meaning the "this posture's task inertia is much smaller and more anisotropic than the report's" reasoning used earlier to justify the `|L|`-based (rather than raw-`Kp`-based) gain target in Finding 2's fix was itself largely an artifact of this missing correction, not a genuine posture difference.

One reassuring, verified fact limits the damage: `|L|` (the metric Finding 2's fix actually targeted) is **exactly invariant to `Λ(q)`** for this controller whenever `q_vel=0` and `Q`/`R` are isotropic (both true for all four configs). Algebraically, matching `q_vel=0`/isotropic `Q`,`R` makes the closed-loop gains a scalar multiple of $\Lambda$ on each diagonal entry, $K_{p,ii} = \Lambda_{ii}\,k_p$ and $K_{d,ii} = \Lambda_{ii}\,k_d$ for the same scalars $k_p, k_d$ on every axis, so $\Lambda_{ii}$ factors straight out of the `|L|` definition (`hardware_results_review.md` §3) and cancels:

$$
\begin{aligned}
|L|_{ii} &= \frac{\sqrt{(\Lambda_{ii}k_p)^2 + (\omega\,\Lambda_{ii}k_d)^2}}{\Lambda_{ii}\,\omega^2} \\
&= \frac{\Lambda_{ii}\sqrt{k_p^2 + (\omega k_d)^2}}{\Lambda_{ii}\,\omega^2} \\
&= \frac{\sqrt{k_p^2 + (\omega k_d)^2}}{\omega^2}
\end{aligned}
$$

-- the right-hand side has no $\Lambda_{ii}$ left in it at all. So the `q_pos` values chosen in Finding 2's fix remain the right ones under the same `|L|=0.45` target -- **only the `Kp`/`Kd` numbers reported at the time were wrong**, computed with the uncorrected `Λ`. Recomputed with the fix:

| | reported at the time (wrong `Λ`) | actual (corrected `Λ`) |
|---|---:|---:|
| `Kp` diag (N/m) | [49.2, 12.7] | **[167.3, 93.8]** |
| `Kd` diag (Ns/m) | [4.45, 1.15] | **[15.1, 8.5]** |
| `\|L\|` | 0.45 | 0.45 (unchanged, as expected) |

The corrected `Kp` now *exceeds* the original report's own validated `Kp≈101.39/61.49 N/m` -- the posture-transfer problem documented at length in Finding 2's fix (needing a separate, lower `|L|`-based target instead of matching the report's raw `Kp`) turns out to have been substantially caused by this bug, not by a genuine mismatch between postures.

### Verification

- `python3 -m py_compile` on every touched file: clean.
- `test_local.py`: passes; MPC sim hold-loop max transient error *improved* (12.11mm -> 4.79mm) now that the controller and the simulated plant agree on `M(q)`.
- Full `run_hardware.py --backend sim` reruns, all four configs: all converge to 0.000mm final error, and every one improved on its own earlier (post-Finding-2-fix, pre-armature-fix) transient response -- `push.yaml`'s peak error dropped from ~103mm to ~1.7mm, `payload.yaml`'s from ~30mm to ~6.4mm -- consistent with the controller now being correctly ~3-4x stiffer than it believed it was. `u_max=100` still never binds (largest observed peak `|u|` across all four: 3.72, far below 100).
- Compute cost re-checked (`tools/benchmark_compute.py`): unchanged, ~2.0ms mean, as expected (adding a constant to a matrix diagonal is free).

### Residual documentation debt (not fixed, low priority)

`docs/01_concepts.md` still does not contain the armature-correction derivation that `run_hardware.py`'s own comment promises. Not fixed here since it's cosmetic (the correct value and its provenance are now recorded in `configs/hold.yaml` and this file); worth closing out if `docs/01_concepts.md` is revised for other reasons.

## Extended, awaiting real hardware: Return_Delay_Time / Status_Return_Level reporting (2026-09-27)

**Status: tool extended; cannot be verified without real hardware, since these are servo EEPROM registers with no `--backend sim` equivalent.**

Prompted by the user asking whether multithreading could help the communication-time question `tools/benchmark_io.py` exists to answer. 
Answer: no -- see the reasoning given directly to the user (not duplicated here), in short: `DynamixelCurrentBackend` puts all three servos on one shared half-duplex serial bus via a single `PortHandler`/`GroupSyncRead`, which is a sequential hardware protocol no amount of Python threading can parallelize, and Python's GIL limits threading's benefit to overlapping *independent* I/O-wait with *other* compute -- of which this control loop, whose every step genuinely depends on the just-read state, has very little to offer.

The two levers that actually are worth checking are both servo/OS configuration, not software architecture:

- **Return_Delay_Time** (Dynamixel register, addr 9): each servo's own built-in wait before replying to a read; default 250 (x2us = 500us); summed across 3 servos on one shared bus this is a real, avoidable contributor to `read_state()`'s cost, and is usually safe to set to 0.
- **The USB-serial adapter's own latency timer** (an OS/driver setting, not a servo register -- e.g. `/sys/bus/usb-serial/devices/<port>/latency_timer` on Linux, commonly defaulting to 16ms) -- a well-known, often-overlooked Dynamixel/ROS gotcha, and on many setups larger than Return_Delay_Time.

`tools/check_current_interface.py` (already run once per bring-up per the README's own checklist, step 1) now also reports both `Return_Delay_Time` and `Status_Return_Level` for every servo, read-only by default, plus a `--set-return-delay-time N` flag to write it (an EEPROM write, torque toggled off around it exactly like the existing `Operating_Mode` write) -- and a printed reminder about the USB latency timer, which this script cannot read or set since it isn't a servo register at all.

**Not verified here**: whether either lever measurably moves `tools/benchmark_io.py`'s `read` number on real hardware. That is the actual next step once hardware is available -- run `check_current_interface.py` first to see the current values, then `benchmark_io.py --backend dynamixel` before and after trying `--set-return-delay-time 0` and/or lowering the OS latency timer.

## Applied: opt-in `--use-jit` switch (Numba-compiled RNEA/FISTA) (2026-09-27)

**Status: APPLIED and verified. Off by default -- every existing invocation of every script behaves exactly as before unless `--use-jit` is passed.**

Follows directly from measuring whether C++ (or JIT-compiling, as a much lower-effort proxy for the same effect) would help, in response to the user asking that as a follow-up to the multithreading question. Short answer, unchanged from the initial measurement: yes, substantially, for compute -- no, for communication, for the same reason threading doesn't help it (Return_Delay_Time/bus arbitration/USB latency timer are protocol- and OS-level, not language-level; see the section above).

The user then asked for this to be actually implemented, with a switch. It now is:

| function | numpy | JIT-compiled | speedup |
|---|---:|---:|---:|
| `_rnea()` (one call, isolated) | 394.6 us | 4.97 us | **79.4x** |
| `dyn.coriolis()` (real call, incl. array overhead) | 374.0 us | 5.75 us | **65.0x** |
| FISTA solve (200 iters, isolated) | 925.7 us | 170.5 us | **5.4x** |
| `mpc.solve()` (real call, incl. array overhead) | 912.4 us | 185.3 us | **4.9x** |
| **whole-tick MPC compute** (`benchmark_compute.py`) | 1.925 ms | 0.473 ms | **4.07x** |
| **whole-tick PID compute** (`benchmark_compute.py`) | ~1.0 ms | 0.257 ms | **~3.9x** |

The whole-tick numbers matter more than the isolated-function ones: they are what `benchmark_compute.py --use-jit` actually measures end to end, not an extrapolation. All numpy-vs-JIT outputs were verified bit-identical (within 1e-9 to 1e-13, floating-point noise, not approximation) before this was trusted -- both in isolated tests and now as permanent regression checks in `test_local.py` (`check_dynamics()`'s use_jit block, `check_mpc_jit()`).

### What changed

- `lib/dynamics.py`: `OpenManipulatorDynamics(use_jit=False)` -- a new constructor flag. `_rnea()` dispatches to a module-level `_rnea_jit()` (Numba `@njit`, the rotation-matrix construction inlined rather than calling out to `_rot()`, since a njit function can only call other njit'd functions) when `use_jit=True`, otherwise the existing pure-Python path, byte-for-byte unchanged. Raises immediately if `use_jit=True` and numba isn't installed (matching how `DynamixelCurrentBackend` fails fast on missing `dynamixel-sdk`). New `warmup()` method triggers Numba's one-time compile (measured ~0.24-2.4s on this machine, varying with Numba's on-disk cache) -- must be called before a real-time loop starts, not during its first tick.
- `lib/interaction_mpc.py`: `ControllerConfig.use_jit: bool = False`; `NormalizedInteractionMPC._solve_box_qp()` dispatches to a module-level `_fista_jit()` the same way. Same `warmup()` pattern.
- `run_hardware.py`, `run_pid.py`: new `--use-jit` CLI flag, threaded into both `OpenManipulatorDynamics` and (for `run_hardware.py`) `ControllerConfig`; calls both `warmup()`s and prints the compile time before `move_to_start`/the real loop.
- `tools/benchmark_compute.py`, `tools/benchmark_io.py`: same `--use-jit` flag, so both can be run with and without it to measure the real before/after on any given machine rather than trusting a number measured elsewhere.
- `test_local.py`: `check_dynamics()` gained a `use_jit=True` regression block; new `check_mpc_jit()` does the same for the FISTA path. Both are skipped (not failed) if numba isn't installed, since it's optional.
- `requirements.txt`: `numba` added, commented as optional (mirrors the existing `dynamixel-sdk` comment) -- the project remains numpy-only by default; nothing requires installing it.
- `README.md`: bring-up step 4 now mentions running `benchmark_compute.py` with/without `--use-jit`.

### Verification

- `python3 -m py_compile` on every touched file: clean.
- `test_local.py`: all checks pass, including the two new JIT regression blocks (max diff 1.11e-16 N·m for dynamics, 1.60e-13 for the MPC solve -- floating-point noise).
- `tools/benchmark_compute.py --use-jit`, both controllers: whole-tick compute drops 1.925ms->0.473ms (MPC) and ~1.0ms->0.257ms (PID) -- see table above.
- Full `run_hardware.py --backend sim --use-jit` and `run_pid.py --backend sim --use-jit` reruns on `configs/hold.yaml`: warm-up message prints before `move_to_start`, loop holds ~100Hz, converges to the same ~0.00mm as the non-JIT run (identical behavior, not just identical numbers in isolation).
- `tools/benchmark_io.py --use-jit` (on `--backend sim`, so this only validates wiring, not the actual comm-vs-compute answer): compute share of the tick drops from ~2ms to ~0.49ms as expected, `read` (sim-substep cost, not comm -- see the section above) unaffected, confirming `--use-jit` is fully independent of the backend choice.

### What this does and does not fix

Confirms, with real end-to-end numbers rather than isolated-function estimates, that the entire per-tick compute (RNEA + FISTA + the rest) can plausibly go from ~2ms to ~0.2-0.5ms -- useful for headroom at 200-500Hz (the original report's own Section 6.7 ran TDC at 500Hz) and, separately, because Numba's compiled code doesn't hit Python's garbage collector, plausibly tightening worst-case latency too (a candidate explanation for the original report's own PID "max=37.57ms" outlier against its 6.73ms mean, though this was not directly measured here).

**Does not touch communication.** Return_Delay_Time, bus arbitration, and the USB-serial adapter's own latency timer are unaffected by `--use-jit` -- see the section above for those levers, and `tools/benchmark_io.py --backend dynamixel` (with or without `--use-jit`) for measuring the actual real-hardware split once available.
