# MPC_s Implementation Fixes -- Real-Hardware Results (2026-09-29 onward)

Continuation of `implementation_fix.md`, split out specifically for the fixes that came out of the real-hardware validation data (`MPC_s_Hardware_results_2026-09-29/`, first received 2026-09-29) and everything downstream of it, as opposed to `implementation_fix.md`'s own fixes (2026-09-27), which were all sim/analysis-only at the time they were applied. Same rules as that file: this records fixes -- applied or merely prepared, clearly labeled as such -- not new problem reports (those still go in `original_implementation.md`).

## Applied: real-hardware validation, divergence auto-stop, and a `circle.yaml` gain pullback (2026-09-29)

**Status: APPLIED and verified.**

The student ran the (code-unchanged, params-only) controller on the real 3-DOF OpenManipulator-X across `hold`/`step`/`circle` tasks (19 runs, 11 completed / 8 diverged, multiple gain sweeps). Full analysis in `hardware_results_review.md` and `divergence_analysis.md` (+ Chinese translations); summarized here only as far as it produced actual code/config changes.

**Headline results**: every fix in `implementation_fix.md` so far is confirmed working on real hardware -- box-QP runs, `u_max` no longer clips, the armature correction is active and logged, timing is compute-bound (~4.2ms/tick, comfortable 100Hz headroom) and matches the `--use-jit` speedup measured there. The one thing real hardware could show that sim/analysis couldn't: **the `~8Hz self-excited oscillation` the original report warned about is real** -- all 8 diverged runs show a tightly clustered 7.4-7.9Hz sign-alternating, amplitude-growing oscillation regardless of task or gain, and it is genuinely task-dependent (the same `|L|` that's stable for `hold` is past the boundary for `circle`).

### What changed

- **`run_hardware.py`**: new `--max-err-mm` (default 25.0) / `--max-err-consecutive` (default 5) flags. The control loop auto-stops if `err_mm` stays above `--max-err-mm` for that many consecutive samples -- every one of the 8 diverged hardware runs was instead stopped by a human operator 1-2s after visible onset, which makes the tail of those logs operator-reaction-time-dependent; this makes it deterministic and stops commanding torque into a run that's already lost. `--max-err-mm 0` disables it (old behavior). Verified in `--backend sim`: a normal `hold` run completes untouched (500/500 samples, max err 0.25mm, well under the 15.4mm highest transient peak seen across every real completed hardware run); a deliberately tiny threshold (`--max-err-mm 0.01 --max-err-consecutive 3`) stops exactly at sample 3 as designed; `--max-err-mm 0` runs the full duration.
- **`configs/circle.yaml`**: `q_pos: 75290.6 -> 27086.8`, i.e. `|L|: 0.450 -> 0.350`. The hardware sweep bracketed the true boundary between `q_pos=51101` (`|L|=0.409`, completed) and `q_pos=65405` (`|L|=0.435`, diverged) -- the previously-shipped `|L|=0.45` was already just past that boundary. `|L|=0.35` sits comfortably below the `0.409` point that stayed stable across the whole sweep. This is the only config that needed pulling back: `hold`'s and `step`'s own gains, even at the much higher `|L|~=0.51` escalated retune, stayed stable on hardware in most trials -- the margin is task-specific, not a property of `|L|` in general (see caveat below).

### Verification

- `python3 -m py_compile run_hardware.py`: clean.
- `--max-err-mm`/`--max-err-consecutive`: the three `--backend sim` cases above (normal run untouched, tiny threshold triggers at exactly the configured consecutive-sample count, `0` disables) all behaved as designed.
- `configs/circle.yaml`'s new gain re-run via `run_hardware.py --backend sim --duration 65`: completes the full 65s (one revolution) without triggering the new auto-stop, max err 0.79mm, last-2s mean ~0.0004mm -- clean tracking, no regression from the pullback.
- Every `|L|` value above was computed directly via `tools/solve_task_space_gain.py --config configs/circle.yaml --target-l ...` for each `q_pos` in the hardware sweep (51101/65405/75290.6/82547/124371), not derived by hand.

### What this does and does not fix

Fixes: `circle.yaml` now ships a gain with real hardware margin below the measured instability boundary, and any future hardware run (on any config, any task) stops itself deterministically instead of relying on an operator's reaction time.

**Does NOT fix**: `step.yaml` has no config-level fix yet. The hardware data shows the committed default (`q_pos=11293.6`) diverges on `step`, and even the escalated `q_pos=124371` retune is only marginally stable there (2/3 trials). Unlike `circle`, there is no dedicated gain sweep for `step` yet (no `step_L...` equivalent of `circle_L024/L0255/L027`) to bracket a safe value from -- picking one now would be guessing, not measuring. This needs a real sweep before `step.yaml` gets its own pullback.

**Also NOT fixed**: the anisotropic-gain code drift (`step_Az.yaml`/`step_Az55.yaml` use list-valued `q_pos`/`q_vel`, which this repo's `ControllerConfig`/`controller_config()` cannot parse -- confirmed by direct `grep`, zero matches for `q_pos_z`/list-handling anywhere in the repo). Pending clarification from the student on whether that's a small controller-config extension worth porting back, or a separate untracked fork.

## Prepared: `step` trajectory type + a controlled step-task gain sweep (2026-10-01)

**Status: trajectory support APPLIED and verified in sim. The sweep itself is only sim-verified -- the real-hardware runs it needs still have to be done on the physical arm, which this session has no access to.**

Follows from `next_steps_test_plan.md` item 1 (the `step.yaml` gap flagged in the previous fix above). Two things were needed before any sweep could even be attempted:

### What changed

- **`lib/trajectory.py` gained a `step` trajectory type.** It did not exist at all before this -- `CartesianTrajectory.sample()` only handled `hold`/`circle` and raised `ValueError` on `type: step`, which `MPC_s_Hardware_results_2026-09-29/configs/step/*.yaml` all use. Whatever generated those real hardware runs has (or had) a `step`-capable trajectory generator that was never in this repo -- the same "ran on hardware, not in the tracked code" pattern as the anisotropic-gain finding above, just for a different file. Implemented from the only two things available to reconstruct it from: the config schema itself (`step_axis`, `step_amplitude_m`, `step_move_time_s`, `step_dwell_s`, `step_cycles`) and `MPC_s_Hardware_results_2026-09-29/configs/step/step.yaml`'s own comment ("5s hold, then 4 cycles of a 20mm move along x (1.5s quintic) and back, 5s dwell after each move"). Minimum-jerk (quintic) move profile, zero velocity/acceleration at both ends of each move. `step_initial_hold_s` is accepted but not required -- no real config sets it, so it defaults to `step_dwell_s` (reusing that value rather than inventing an unobserved constant).
- **`configs/step.yaml`** added to the repo (did not exist before -- `configs/` only had `hold`/`push`/`payload`/`circle`). Trajectory/robot/pid blocks copied verbatim from the hardware-results file (same physical task, same arm); `q_pos=124371` kept as-is but the file's own comment marks it PROVISIONAL, not validated the way the other four configs are -- 5/7 real runs at this exact config completed, 2 diverged.
- **`configs/step_L15.yaml` / `_L20.yaml` / `_L25.yaml` / `_L30.yaml` / `_L35.yaml`**: five sweep points, `q_pos` solved via `tools/solve_task_space_gain.py --config configs/step.yaml --target-l <0.15..0.35>` so each targets a specific `|L|` with everything else (`r=8.77`, observer tuning, robot/trajectory blocks) held fixed at `step.yaml`'s own values -- a controlled, single-variable sweep, unlike the existing hardware data (see below for why that distinction matters here).

### Why a controlled sweep, specifically

Computing `|L|` properly for `step.yaml`'s own `r`/`lambda_damping` (not circle's) gives a result worth flagging on its own: the one hardware run that diverged at low gain (`hw_step_re_1`, `q_pos=11293.6`) sits at `|L|=0.166`, while the "A" config that was MOSTLY stable (`q_pos=124371`, 5/7 real runs) sits at `|L|=0.299` -- higher gain, better outcome, the opposite of circle's pattern (where higher `|L|` is less stable). That is not evidence `|L|` works backwards for `step` -- it is evidence those two hardware runs are not a clean `q_pos`-only comparison: `hw_step_re_1` used the committed `hold`/`push` defaults' `r`/observer values, not `step.yaml`'s own, so `q_pos` was not the only thing that changed between the two. The existing hardware data cannot actually establish the `step`-task `q_pos`-vs-stability relationship at all. The 5-point sweep above fixes exactly that by holding everything but `q_pos` constant.

### Verification

- `python3 -m py_compile lib/trajectory.py`: clean.
- `test_local.py`: unchanged, still passes (it does not exercise `step`).
- All 6 files (`step.yaml` + 5 sweep points) run via `run_hardware.py --backend sim --duration 60`: all complete the full 60s/6000 samples with no traceback and no auto-stop, max transient error 0.85-0.97mm during the quintic moves, settling back to ~0.000mm by the final dwell, peak `|u|` 0.05-0.16 (nowhere near `u_max=100`) -- the sweep is safe to try on real hardware, which is the one thing sim can actually tell us before doing so.
- Each sweep file's `|L|` was independently cross-checked by running `tools/solve_task_space_gain.py` directly on it afterward (not just trusted from the generation script): 0.150/0.200/0.250/0.300/0.350 as intended, `q_pos`=7568.0/24339.4/60339.0/126787.2/237551.6.

### What this does and does not fix

Fixes: `step` can now be run via `run_hardware.py` at all (it could not before), and there is now a safe-in-sim, controlled sweep ready to run on real hardware.

**Does NOT fix**: `step.yaml`'s own `q_pos` is still provisional. Nobody has run `step_L15.yaml` through `step_L35.yaml` on the real arm yet -- that is real-hardware work this session cannot do. Per `next_steps_test_plan.md` item 1: each point needs at least 2 real-hardware repeats (the existing "A" data shows a single run is not enough to call a marginal config "stable"), and `--max-err-mm`'s new auto-stop (above) should be on for all of them.

## Prepared: `tools/chirp_response.py`, an open-loop resonance-vs-loop-delay diagnostic (2026-10-01)

**Status: written and sim-verified clean on all 3 joints. Not yet run on real hardware -- that is the whole point of the tool, and this session cannot do it.**

Follows `next_steps_test_plan.md` item 3: the 7.4-7.9Hz self-excited oscillation's frequency being essentially gain-independent (`divergence_analysis.md`) is consistent with two different root causes needing two different fixes -- a genuine mechanical/structural resonance (fix: a notch filter, or hardware work) vs. a closed-loop critical frequency from total loop delay (fix: reduce that delay). Telling them apart needs measuring the arm with the control loop entirely removed.

### What changed

- **`tools/chirp_response.py`** (new): commands ONLY `tau = gravity_scale*dyn.gravity(q) - damping*dq + chirp(t)` on one chosen joint -- the same no-feedback-at-all design as the existing `tools/test_gravity_compensation.py`, so there is no possibility of closed-loop instability contaminating the measurement. The chirp is a logarithmic (exponential) swept sine (`--f0`/`--f1`/`--duration`/`--amplitude-nm`), analyzed via `--plot` as a sliding-window RMS of the excited joint's velocity mapped to the chirp's (analytically known, not estimated) instantaneous frequency -- a response-amplitude-vs-frequency curve, with the 7.4-7.9Hz band marked for reference. Safety: a hard abort (`--max-dev-rad`, same pattern as `run_hardware.py`'s `--max-err-mm`) if the excited joint strays too far from its starting angle, `move_to_start` first for a reproducible operating point, and the script refuses an `--amplitude-nm` that isn't comfortably under the joint's `tau_max_Nm`.

### A real design problem found and fixed during this, not just written blind

The first version defaulted to `--f0 0.5` (per `next_steps_test_plan.md`'s own draft wording). Running it in `--backend sim` tripped the safety abort in well under half a second. Not a bug in the abort logic -- a correct consequence of removing ALL position feedback: a joint's response to a torque at frequency `f` scales as `1/f^2` for a plain double integrator (`tau -> M*qddot`), so a low-frequency chirp component acts like a slowly-varying bias torque with nothing to center the joint against it. Fixed by moving the default sweep to `--f0 3 --f1 15` (still a ~2.3-octave band straddling 7.4-7.9Hz on both sides) -- verified clean afterward. Documented in the script's own docstring so this isn't rediscovered the hard way on real hardware.

### Verification

- `python3 -m py_compile tools/chirp_response.py`: clean.
- `--amplitude-nm` safety refusal: confirmed it refuses an amplitude at or above half the excited joint's `tau_max_Nm`.
- `--backend sim --config configs/hold.yaml`, all 3 joints, `--duration 10`: all complete the full 1000 samples with no auto-stop, deviation stayed under ~0.15 rad (well under the 0.3 rad default threshold) throughout.
- `--plot` on the resulting CSVs: both output plots (response-vs-frequency, raw time trace) inspected visually -- frequency sweeps smoothly from ~3.25Hz to ~14Hz as commanded, response amplitude decays smoothly with no spurious peak (expected: sim's rigid-body model has no mechanical resonance to find; this only confirms the script's own plumbing, exactly like `tools/benchmark_io.py`'s sim backend only validates wiring, not the real answer).

### What this does and does not establish

Fixes/adds: a ready-to-use, safety-checked tool for the one measurement that can actually distinguish the two candidate explanations for the resonance.

**Does NOT establish**: which explanation is correct. That needs `tools/chirp_response.py --backend dynamixel` run on all 3 joints on the real arm, then `--plot` compared against the 7.4-7.9Hz band -- not done here, no hardware access from this session. (2026-10-02: `--backend sim` on all 3 joints done and written up in `friction_chirp_analysis.md` Part 2, including the open-loop plant/resonance equations behind the method -- confirms the pipeline works, not the physics; the real run is still open.)

## Applied: `qp_iters` reduced from 200 to 50 in every shipped config (2026-10-01)

**Status: APPLIED and verified. All 10 configs (`hold`, `push`, `payload`, `circle`, `step`, `step_L15`-`step_L35`) updated and re-run clean in `--backend sim`.**

In response to the user asking whether anything else could shorten the per-tick time delay or improve efficiency, beyond what's already in `implementation_fix.md` (`--use-jit`, the still-unverified `Return_Delay_Time`/USB latency timer levers). `qp_iters` (FISTA iterations per `solve()` call) defaults to 200 and no shipped config had ever overridden it -- but `NormalizedInteractionMPC._solve_box_qp()` warm-starts FISTA from the PREVIOUS tick's solution every tick (`self._u_warm`), so 200 fresh iterations re-solves a problem that is already nearly solved.

### Evidence (verified three independent ways before touching any config)

1. **Isolated `solve()` calls**, warm-started across a realistic sequence of 300 ticks (random `x_state`/`d_hat` perturbations): `qp_iters=200` costs 916.8us/solve; `qp_iters=50` costs 242.8us/solve (**3.8x faster**), with the resulting `u` deviating at most ~3% from the 200-iteration answer.
2. **`tools/benchmark_compute.py`** (deliberately excludes sim-physics overhead, times only the real control-law cost) on `configs/circle.yaml` with `--use-jit`: p99 compute time 1.729ms -> 0.424ms at `qp_iters=50` (**4.1x**).
3. **Closed-loop `--backend sim` on `circle.yaml`** (the hardest tracking case of the four original tasks): max error 0.787mm -> 0.787mm, steady-state 0.0004mm -> 0.0004mm -- identical, not just "close."

One confound found and worth recording: an earlier closed-loop comparison via `run_hardware.py`'s own `compute_ms` column showed almost NO change across `qp_iters` values, which looked like the whole effort was pointless -- until remembering (same lesson as `tools/benchmark_io.py`'s own docstring) that `SimArmBackend.read_state()` does real physics-substep integration, which dominates that column and has nothing to do with `qp_iters`. `benchmark_compute.py` exists specifically to exclude that confound, which is where the real 4.1x number above comes from. Logged here so this isn't rediscovered the hard way later.

### What changed

- `configs/hold.yaml`, `push.yaml`, `payload.yaml`, `circle.yaml`, `step.yaml`: added `qp_iters: 50` to the `controller:` block (previously absent, meaning the 200 default was silently in effect everywhere).
- `configs/step_L15.yaml` through `step_L35.yaml`: same addition, keeping them in sync with `step.yaml`'s body (they were generated as full copies of it with only `q_pos` varied -- `qp_iters` is unrelated to the gain being swept, so adding it doesn't compromise that sweep's single-variable design).

### Verification

- `test_local.py`: unchanged, still passes.
- `--backend sim --duration 20`, `hold`/`push`/`payload`: max error 0.252mm / 1.663mm / 6.696mm -- matches the pre-existing baselines (push ~1.7mm, payload ~6.4mm) within normal run-to-run variance.
- `--backend sim --duration 65`, `circle`: max error 0.787mm, steady-state 0.0004mm -- identical to the pre-change number (see the gain-pullback entry above).
- `--backend sim --duration 60`, `step` + all 5 `step_L*` sweep points: max error 0.853-0.974mm across all 6, matching the pre-change sweep-verification numbers in the `step` trajectory entry above to 3 decimal places. No auto-stop, no traceback, in any of the 6.

### What this does and does not fix

Fixes: real, verified compute headroom (~4x on the FISTA share specifically) with no measurable cost, available immediately -- does not require real hardware to adopt, unlike most of the other items in `next_steps_test_plan.md`.

**Does NOT fix**: communication-side delay (still needs `Return_Delay_Time`/`Status_Return_Level`/USB latency timer verified on real hardware, per `implementation_fix.md`'s earlier entry) or any of the other efficiency ideas raised alongside this one but not yet tried: a higher baud rate (XM430-W350 supports well above the 1Mbps currently used in every example, exact ceiling not confirmed against the firmware here), `gc.disable()` during the real-time loop (Python's cyclic GC is a plausible but unconfirmed explanation for the PID "max=37.57ms" outlier flagged in `implementation_fix.md`), and a shorter `horizon` (would help further but changes the MPC's actual behavior, not just its speed -- needs the same full re-verification treatment as a gain change, not done here).

## Applied: `tools/estimate_friction.py`, three proxy friction estimates from existing logs (2026-10-02)

**Status: written and verified against real hardware data (not just sim). All three modes checked for sensible, non-trivial output; a real methodological bug found and fixed during that check, not just written blind.**

Follows directly from the user asking "can we estimate the friction [from existing data]?" -- answered inline in chat with ad-hoc analysis at the time, never saved as code. This is that analysis turned into a reusable tool, plus a third mode (`breakaway`) that automates a manual finding from the same conversation rather than leaving it as a one-off eyeballed observation.

### What changed

- **`tools/estimate_friction.py`** (new), three independent `--mode`s, none a substitute for `tools/chirp_response.py --backend dynamixel` (open-loop, joint-space, not filtered by the observer) once that data exists -- see the script's own docstring for the full caveats on each:
  - `static`: a `hold`-task log's steady-state `d_hat`, mapped through `Lambda(q)`/`J^T` into joint-torque units -- the holding residual at zero velocity, not breakaway stiction.
  - `viscous`: a moving-task log's `d_hat` (converted to task-space force via `Lambda(q)` computed PER SAMPLE, not one fixed posture) correlated against `ee_vel`, by axis. Stays in task space on purpose -- this project's logs don't record joint velocity, only `ee_vel`, and a pseudo-inverse projection into joint space would introduce a null-space ambiguity the controller's own posture term actually uses.
  - `breakaway`: scans logged `q` for a joint pinned within ~1 encoder tick (XM430-W350: `2*pi/4096` rad) for a sustained run, immediately followed by real motion -- reports the commanded-torque swing during the stuck window, automating the manual find from the earlier chat analysis (joint 0, `hw_step_A_1.csv`, ~0.12-0.15 Nm) rather than leaving it as one eyeballed window.

### A real methodological bug found and fixed during testing, not written blind

First version of `breakaway` added a `--max-stuck-s` cap that DROPPED windows longer than it, meant to filter out ordinary commanded dwell phases (`step_dwell_s=5.0` in the real configs) that look identical to stiction (both are "q flat, then q moves"). Running it against `hw_step_A_1.csv` silently ate the exact event this script exists to find: the real breakaway's torque ramp is CONTINUOUS across the boundary between a preceding legitimate hold and the actual stiction window (confirmed by printing the raw `tau_0` trace: a smooth, unbroken ramp from well before the move starts to well after), so there is no discontinuity to split the window on, and the duration-from-start cap discarded the whole thing. Fixed by never dropping events -- `--likely-dwell-s` now TAGS long events in the printed output instead, so a human (or future caller) sees everything and can judge, rather than trusting a cap that already proved it hides real signal.

### Verification

- `python3 -m py_compile tools/estimate_friction.py`: clean.
- `static` on all 5 real `hold/completed/*.csv` logs: joint-torque-equivalent estimates all land in the same 0.01-0.1 Nm range found in the original ad-hoc chat analysis (not identical -- this version uses each run's own mean `q` over the tail window rather than one fixed nominal posture, a real accuracy improvement, not a regression).
- `viscous` on `hw_circle_L024_ry1e-8_1.csv`: same sign (opposing velocity, consistent with viscous friction) and same rough magnitude (R^2 0.12-0.22) as the original chat analysis; now reported in physical force units (N) via per-sample `Lambda(q)` rather than raw, harder-to-interpret `d_hat` units.
- `breakaway` on `hw_step_A_1.csv`: after the fix above, correctly re-surfaces the joint-0 event (now reported as `t=3.390-5.550s`, `swing=+0.1794 Nm`, tagged `likely_dwell` since the window exceeds 2s -- an honest flag, not a hidden one, given the continuous-ramp finding above).
- `--backend sim --config configs/step.yaml` (15s) piped straight into `breakaway`: zero events on all 3 joints, as expected -- sim has no stiction model, confirming the detector isn't just pattern-matching noise.
- `test_local.py`: unchanged, still passes.

### What this does and does not establish

Adds: a reusable tool instead of one-off chat analysis, for three different (still rough) friction proxies against data that already exists.

**Does NOT establish**: a trustworthy, joint-space friction MODEL (Coulomb coefficient + viscous coefficient per joint) -- all three modes here are proxies through the closed-loop controller and its observer, exactly as caveated in the script's own docstring. That still needs `tools/chirp_response.py --backend dynamixel` run on real hardware, which doesn't exist yet.
