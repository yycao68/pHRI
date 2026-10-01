# MPC_s Original Implementation Review

Review date: 2026-09-27 (Finding 4 added same day, second pass, triggered by
fact-checking an external review's citation of the report's Section 2.5)
Reviewed against: `ReportOPENManipulatorX.pdf` (stage report, 28 Aug 2026, 39pp) and `MPC PID and TDC Manipulator.pdf` (slide deck, 02 Sep 2026) -- both now kept alongside the real-hardware data in `../MPC_s_Hardware_results_2026-09-29/`, not in this folder -- the cited paper (Cao & Tang, arXiv:2606.08281 -- `pHRI/arXiv/phri_combined.tex`), and the already-validated reference port of the same controller to the same robot family, `pHRI/openmanipulator_verify/`.

This file records **problems found** in the code as it stood at review time. Fixes -- applied or recommended -- are tracked separately in `implementation_fix.md`, not here.

**Status: all four findings below now have a corresponding fix recorded in `implementation_fix.md`** (Findings 1 & 2 structurally, via the restored QP; Finding 2's gains, via a retune; Finding 3, via a `u_max` correction; Finding 4, via activating the missing config key) -- this file is kept as the historical record of what was wrong, not a live list of open issues. Cross-check `implementation_fix.md`'s own section headers (each names which finding it addresses) before treating anything below as still outstanding.

## Summary

The student's own report is not reproducible from the code currently in `MPC_s/`. Four independent problems were found:

1. **(Finding 1)** The `NormalizedInteractionMPC` in `lib/interaction_mpc.py` is not the receding-horizon, box-constrained QP the report's own math (Eq. 12--14) and the cited paper both describe -- it is a completely different design (a static, precomputed full-state-feedback gain).
2. **(Finding 2)** The gains shipped in `configs/*.yaml` are 15--250x softer than the task-space stiffness the report validated as its final operating point.
3. **(Finding 3)** `circle.yaml`/`payload.yaml`'s `u_max` is a leftover, unconverted force-domain value, not a real acceleration-domain bound.
4. **(Finding 4)** A model correction the report calls decisive (reflected-rotor/gearbox inertia, `c=0.01 kg m²`) exists as working code but was never actually turned on in any config.

This is almost certainly why "the controller is not correctly realized": whatever produced the report's numbers is not what runs today. Findings 1/2/4 compound each other -- Finding 4 alone was later found to explain much of why Finding 2's original fix needed a lower, `Λ`-invariant target instead of the report's raw `Kp` number (see Finding 2's table below and Finding 4).

`lib/dynamics.py` and `lib/kinematics.py` were also checked and are **not** implicated -- see "What was checked and found correct" below.

## Finding 1 (primary): the shipped "MPC" is a static LQR gain, not the QP the report/paper describe

**What the report and paper specify.** §3.1 (Eq. 12--14) derives a receding-horizon QP: the plant is discretized with a *configuration-scheduled* input matrix `Bd(q)` (task inertia `Λ(q)` rebuilt every tick and held fixed across a 10-step horizon -- the report's own "quasi-LPV" framing), the decision variable is a sequence of *centered task-space forces* `v_k = F_k + Λd̂`, box-constrained at `F_max = 300 N` and re-solved by OSQP every tick. `phri_combined.tex` confirms the same structure: "the optimization variable is a corrective task force... a small convex QP whose state-transition block can be precomputed," with a separate Cartesian force bound `F_max` (`eq:qp`/`eq:QP_torque`).

**What `lib/interaction_mpc.py` actually implements.** `NormalizedInteractionMPC` computes a single gain `K0` **once**, offline, via a fixed-`horizon` backward Riccati recursion (`_finite_horizon_first_gain`), and every tick just evaluates `u = -K0 @ x - d_hat` followed by a plain `np.clip` on that one vector. There is no re-solved QP, no per-step box constraint over the horizon, and no `Λ(q)` anywhere inside the controller -- `Λ(q)` is applied *outside* it, as a post-hoc multiplication in `run_hardware.py` (`F_task = Lam @ (ddp_d + u)`). A repo-wide search confirms there is no QP-solving code at all:

```
$ grep -rln "osqp\|OSQP\|FISTA\|solve_box_qp\|scipy" --include="*.py" . (no matches)
```

**Evidence this is a regression, not a documented simplification.** The report's own numbers only make sense if it was produced by a real, re-solved QP:

- §6.6 attributes **~8 ms of the loop delay to "QP solve"**, matching the abstract's "MPC 8.2 ms mean" compute cost. A precomputed `K0` gain (a single matrix-vector product) cannot cost anything close to 8 ms.
- §4.1's Table 4 lists the report's **final validated gains** as `q_pos = 120000` (posture 1) and `q_pos = 60000` (posture 2). Neither value, nor anything near them, appears anywhere in `configs/*.yaml` today (which top out at `q_pos = 1810` in `circle.yaml`).

So the report documents a controller/gain combination that no longer exists in this repo -- at some point the OSQP-based box-QP was replaced with the static-gain shortcut (plausibly *to fix* the 8 ms compute-budget problem the report itself flags as a limitation), but the replacement gains were never re-derived to match the validated task-space stiffness.

## Finding 2: quantified impact -- current gains are 15--250x too soft

The report's own §4.1 shows the `(q_pos, q_vel, r) -> (Kp, Kd)` mapping is strongly nonlinear and saturating ("tripling the weight buys only 1.9x the stiffness"), so the replacement gains cannot have been carried over correctly by inspection. I ran the *current* `NormalizedInteractionMPC` against the *current* `dynamics.py`/`kinematics.py` at each config's own `posture_q_rad`, and computed the actual resulting task-space stiffness `Kp = Λ(q) @ K0[:, :2]`, `Kd = Λ(q) @ K0[:, 2:]`:

| config | `q_pos` (current) | **actual `Kp` (N/m)** | **actual `Kd` (Ns/m)** | report's validated `Kp`/`Kd` (N/m / Ns/m) |
|---|---:|---:|---:|---:|
| `hold.yaml` / `push.yaml` | 15 | 0.41 / 0.11 | 1.18 / 0.31 | 101.39 / 8.27 (posture 1) |
| `payload.yaml` | 900 | 6.34 / 1.64 | 4.30 / 1.11 | 101.39 / 8.27 |
| `circle.yaml` | 1810 | 2.03 / 0.52 | 5.64 / 1.46 | 61.49 / 5.08 (posture 2) |

(diagonal entries shown; off-diagonal coupling is small, as in the report's own Table 4/Eq. 21.)

**Note added after Finding 4 was found (same review session):** the `Λ(q)` used for this table was itself computed without the reflected-rotor inertia correction Finding 4 describes -- see there for the corrected `Λ` and how much of this table's apparent softness/anisotropy that alone accounts for. The qualitative conclusion below (the shipped `q_pos` values are far too soft) still holds regardless -- Finding 4's fix does not touch `q_pos` at all -- but these specific N/m numbers should not be read as the current, corrected picture.

Given the report's own headline finding -- *this arm is friction-dominated, and the entire three-way comparison is about how much authority each controller can bring to bear against stiction* -- a task-space stiffness of ~0.1--2 N/m is very unlikely to break stiction at all. Running any of the current configs today will almost certainly **not** reproduce the report's "MPC 0.504/0.864 mm" settled-error numbers.

Notably, `circle.yaml`'s own comment says it needs "much stiffer gains than hold.yaml/push.yaml" to get adequate tracking bandwidth for a moving reference -- yet its computed `Kp` (2.03/0.52 N/m) is not meaningfully stiffer than `payload.yaml`'s (6.34/1.64 N/m), and both are far below the report's validated 61--101 N/m. The intended stiffness ordering across tasks does not survive into the actual delivered gains.

## Finding 3 (secondary, lower confidence): a likely unconverted unit copy

`circle.yaml` and `payload.yaml` set `u_max: [300.0, 300.0, 300.0]`, labeled in `hold.yaml`/`push.yaml` as a "residual-acceleration clamp [m/s^2]." 300 m/s² is about 30x gravity -- implausible as an intentional acceleration limit for this arm, but numerically identical to the paper/report's **Cartesian force** bound `F_max = 300 N`. This looks like a leftover from the old QP-based version, carried into the new acceleration-domain code without converting through `Λ(q)`. It is currently inert in practice (the joint-torque clip `tau = np.clip(tau, -tau_max, tau_max)` always binds first, since `tau_max_Nm` is only a few Nm), so it has probably not corrupted any reported number, but it means the software `u_max` safety limit is not doing anything meaningful for these two tasks and should not be trusted as a real bound.

## Finding 4: the reflected-rotor/gearbox inertia correction exists in code but was never activated in any config

The report's own Section 2.5 identifies a decisive model correction: the link-geometry-only mass matrix `M(q)` (Section 2.2) omits the XM430-W350 servos' own reflected rotor/gearbox inertia, and adding a constant `c = 0.01 kg m²` to `M(q)`'s diagonal (`M_used(q) = M(q) + c*I₃ₓ₃`) drops joint 3's 0.1s open-loop prediction RMSE from **137.0° to 2.87°** (a 47.7x reduction) -- and "this is what every controller in this report actually uses" (report, verbatim).

`run_hardware.py`, `run_pid.py`, and `SimArmBackend` already have the exact matching machinery for this: `dyn_armature_kg_m2` (default `0.0`), added to `dyn.mass_matrix(q)` before every use. Its own comment even says it was "identified from real hardware data" and to see `docs/01_concepts.md` for the derivation. But:

- **No `configs/*.yaml` file ever sets `dyn_armature_kg_m2`** -- all four silently ran (and, before this session's fixes, were gain-tuned) with the default `0.0`, i.e. with the exact correction the report calls decisive turned off.
- `docs/01_concepts.md` **does not contain the promised derivation** -- the cross-reference in `run_hardware.py`'s own comment is itself stale/dangling.
- `test_local.py`'s own simplified control loop computed `Mq = dyn.mass_matrix(io.q)` directly, never reading `dyn_armature_kg_m2` at all -- even if a config set it, this test harness would still silently run the controller against the wrong plant model.

**Measured impact.** At this project's shared posture (`[0.6, -2.3493, -1.86]`), adding the correct `c=0.01` changes the operational-space mass matrix `Λ(q)` from `[[0.201, 0.076], [0.076, 0.052]]` to `[[0.686, -0.022], [-0.022, 0.384]]` -- diagonal entries up 240%/639%, and the off-diagonal coupling drops from being over 100% of the smaller diagonal entry to a small, genuinely secondary term. Notably, the corrected `Λ` is now close to the report's own validated posture's `Λxx≈0.67, Λzz≈0.43` -- suggesting the earlier apparent "this posture's task inertia is much smaller/more anisotropic than the report's" finding (used in fixing Finding 2, see `implementation_fix.md`) was itself largely an artifact of this missing correction, not a genuine posture difference.

This was found while independently fact-checking an external review's citation of the report's Section 2.5 -- the citation was accurate, and checking it against the current codebase surfaced this gap. See `implementation_fix.md` for the fix and its full verification.

## What was checked and found correct

- **`lib/dynamics.py` (RNEA gravity/mass matrix).** Diffed line-by-line against the already-validated `pHRI/openmanipulator_verify/lib/dynamics.py`. The `_rnea`/`_rot`/`mass_matrix`/`gravity` algorithm is byte-identical; the only differences are the expected consequences of physically removing the waist joint (dropped `LINKS[0]`, `n=3`, `D[0]` folded to the combined `d01+d12` offset, axes reduced to all-pitch) -- a careful, documented adaptation, not a divergence.
- **`lib/kinematics.py` (FK/Jacobian).** Same comparison: the link-offset re-derivation (`d0e_base = d01+d12`, joint relabeling) is consistent with the physical waist removal and is explicitly commented as such. The new `solve_ik_xz` method (absent from the reference) is an addition used by `trajectory.py`, not a modification of the validated FK/Jacobian code.
- **The Kalman disturbance observer's predict/correct tick ordering.** `RandomWalkDisturbanceObserver.step()` predicts using the *current* tick's `u` before correcting against the current measurement -- structurally identical to the reference implementation, which is itself validated against MuJoCo (README: "hold SS 1.3 mm, circle SS 2.0 mm... offset-free"). Not a bug; it is how the validated design already works.
- **The `u = -K0 x - d_hat` feedback structure itself.** This is not wrong in isolation -- it is exactly the paper's own unconstrained-limit reduction of the QP to a static LQR-plus-feedforward law (matching the general "MPC reduces to impedance in the unconstrained, disturbance-free limit" result used throughout this paper series), and the report's own §3.1 notes the box constraint was never active during the actual study. The problem is not this structure; it is that (a) it silently replaced a genuinely re-solved QP without saying so, and (b) its gains were never re-derived to match what was validated.

## Not yet checked

- `lib/dynamixel_backend.py`'s real-hardware current-mode I/O specifically (as opposed to `SimArmBackend`, now exercised via Finding 4's fix) was not audited in this pass.
- `lib/pid_controller.py` and the PID-vs-MPC comparison protocol (§4.3--4.5) were not independently re-verified.
- ~~The "corrected inertia model" referenced in the report's §2.5 was not compared against the `LINKS`/`D` constants currently in `dynamics.py`.~~ 
-- done, see Finding 4 above (the correction itself was never activated, not a mismatch in the `LINKS`/`D` constants).
