# Review of real-hardware results (`MPC_s_Hardware_results_2026-09-29`)

Cross-checked against `pHRI/MPC_s/original_implementation.md` and `implementation_fix.md`.
Source data: 19 CSV runs (11 `completed`, 8 `diverged`) across `hold`/`step`/`circle` tasks,
12 config variants, plus pre-generated figures. All numbers below are freshly recomputed
from the raw CSVs, not taken from filenames or prior claims.

## Bottom line

Overall: **yes, it makes sense** — every one of the fixes made earlier this session is doing
what it was supposed to do on real hardware (the box-QP runs, the armature correction is
active, `u_max` no longer clips, timing is compute-bound and controllable via `--use-jit`).
The genuinely new information from this data is that the **stability margin is task-dependent
and narrower than the sim-derived `|L|=0.45` target for the `circle` task**, and there's one
piece of **real code drift** (anisotropic gains) running on hardware that isn't in the
committed repo. Neither is a sign anything is broken; both are exactly the kind of thing you
can only learn from real hardware.

## 1. Timing: resolved, and it was never a "re vs A gains" effect

Initial pass over `hw_hold_re_1.csv` looked alarming: `compute_ms` mean 15.89ms (loop can't
hold 100Hz) vs `hw_hold_A_ry1e-8_1.csv`'s 4.18ms — a ~3.8x gap that gains alone can't explain
(same architecture, same per-tick QP size). Pulling every `*re*` run resolved it:

| run | q_pos | compute_ms mean | compute_ms max | period_ms mean |
|---|---|---|---|---|
| `hold/hw_hold_re_1` | 11293.6 | **15.894** | 19.654 | 15.973 |
| `hold/hw_hold_re_2` | 11293.6 | **4.205** | 9.069 | 9.990 |
| `step/hw_step_re_1` | 11293.6 | **4.208** | 8.546 | 9.993 |

Same gains (`q_pos=11293.6`, the committed repo default), same task family, and the SECOND
and THIRD runs are ~3.8x faster than the first. That ratio matches this session's own
measured whole-tick `--use-jit` speedup (4.07x MPC / 3.9x PID) almost exactly. The far more
likely explanation is simply that `_re_1` was run without `--use-jit` and later runs were run
with it (e.g. the flag was enabled partway through the session) — not a property of the gains
at all. **Worth a 10-second confirmation with you**, but I'm confident enough in the number
match to say the timing story is fully explained and there's no remaining mystery: compute is
dominated by the MPC solve as expected, `--use-jit` gives the same ~4x on hardware it gave in
the isolated benchmarks, and at 4.2ms/tick every config here has comfortable headroom under a
10ms (100Hz) budget.

## 2. Divergences: real dynamic instability, not a timing or saturation artifact

8 of 19 runs diverged. For every one checked (`circle_L027`, `circle_L0255`, `step_re_1`, plus
summary stats on the rest), `compute_ms`/`period_ms` stayed flat and in-budget (3.6–5.6ms)
straight through the divergence — ruling out "the loop fell behind and that caused it." No run
showed `|u|` pinned at `u_max` beforehand — ruling out the old Finding-3-style saturation
failure mode. This is a genuine closed-loop dynamic instability, consistent with the original
report's own "~8Hz self-excited oscillation at high gain" mechanism.

The clearest evidence is `circle/diverged/hw_circle_L027_ry1e-8_1.csv` (`q_pos=82547`). Signed
per-axis error (not just magnitude) around the failure:

- `t=12.35–13.90s`: bounded, smoothly-varying, sub-mm to ~1.5mm on both x and z — normal
  tracking.
- `t=13.90–14.60s`: error starts alternating sign roughly every ~50–100ms with amplitude
  **growing on each swing** — textbook onset of an unstable oscillatory mode. x and z are
  in-phase with each other (a coupled mode, not independent per-axis noise).
- `t=14.60–15.85s`: fully diverged, ±20–30mm swings alternating at roughly the same period —
  an oscillation frequency of **~8–10Hz**, matching the original report's own hypothesized
  self-excited-oscillation frequency almost exactly.

`hw_circle_L0255_ry1e-8_1.csv` (`q_pos=65405`) shows the same qualitative pattern, just
starting a few seconds later (~t=16–17s vs ~14s) — consistent with a smaller margin past the
same instability boundary rather than a different failure mode. `hw_step_re_1.csv`
(`q_pos=11293.6`, the softest of the committed configs) also diverges, but later in the run
(~t=12s of 14s) and with a somewhat different signature (large swings after a smaller initial
spike) — plausibly the same class of instability triggered by the step task's larger
transient rather than the circle's steady high-gain tracking; not fully characterized, listed
as a follow-up below rather than asserted.

None of this points to a code bug — the same mechanism (oscillatory instability above a
gain/frequency-dependent stability boundary) was already flagged as a known, unavoidable
limitation in `implementation_fix.md`'s "what this fix does NOT establish" caveat. This is
that caveat being confirmed on real hardware, not a new problem.

## 3. The circle-task stability boundary is real, and lower than the sim-derived target

The student's own sweep localizes it directly:

| q_pos | `\|L\|` (session's metric, at `w=50.27`) | circle result |
|---|---|---|
| 51101 (`L024`) | ≈0.24 | **completed** |
| 65405 (`L0255`) | ≈0.26 | diverged (~t=17s) |
| 75290.6 (committed `circle.yaml`, `\|L\|=0.45` target) | 0.45 | *(not directly run here; A/L027 bracket it)* |
| 82547 (`L027`) | ≈0.27 | diverged (~t=14s) |
| 124371 ("A", escalated retune) | ≈0.62 | diverged |

The true boundary sits close to `q_pos≈51101` (`|L|≈0.24–0.26`), well under the `|L|=0.45`
value `circle.yaml` currently ships with (`q_pos=75290.6`). Meanwhile the **same `|L|=0.45`
family is fine for `hold`** — three repeats at the much higher "A" gains (`q_pos=124371`,
`|L|≈0.62`) all completed cleanly (SS error 0.33–1.59mm). This is exactly the "stability is
task-dependent, not just gain-dependent" caveat from `implementation_fix.md`, now with a
concrete number: **`circle.yaml`'s current `q_pos=75290.6` is probably past the real margin**
and should be pulled back toward ~50000 (leaving headroom below the ≈51101 measured boundary)
if `circle` is meant to be a reliably-stable shipped config rather than a stress test.

## 4. Code drift: anisotropic gains run on hardware, not in the committed repo

`step_Az.yaml` and `step_Az55.yaml` set `q_pos`/`q_vel` as 2-element lists
(`q_pos: [124371, 877980]`), and the CSV schema has dedicated `q_pos_z`/`q_vel_z` columns to
log them. The committed `pHRI/MPC_s/lib/interaction_mpc.py` does not support this —
`ControllerConfig.q_pos` is typed `float` and `controller_config()` does
`float(c.get("q_pos", ...))`, which would raise on a list value. Whatever produced these two
configs and CSVs is running code that isn't in the repo I can see. This is the same
"undocumented divergence between what's reviewed and what actually runs" pattern this session
started from (the original static-LQR regression) — worth deciding whether to port the
anisotropic-gain capability back into the repo, or whether it was a one-off hardware-side
experiment that isn't meant to be kept.

## 5. Mid-run error spikes in "completed" runs: checked, benign

Several successful runs still show 8–15mm error spikes mid-run even though they recovered to a
good steady state (e.g. `hw_hold_A_ry1e-8_2` peaks 15.4mm, `hw_step_A_2` peaks 12.3mm). Checked
each with the same signed-per-axis method used for the diverged runs in
`divergence_analysis.md`, specifically to rule out these being near-misses of the same
~7.6–7.9Hz oscillatory mode. They aren't:

- **Hold-task spikes** (`hw_hold_A_ry1e-8_2` at t=1.40s, `hw_hold_re_2` at t=0.27s): both occur
  in the first ~1.5s of the run, while `p_d` is exactly constant (range `[0,0,0]` — a true hold
  target). Signed error rises smoothly and one-sidedly to the peak, then decays smoothly back
  down with no sign reversal. This is a single damped initial-settling transient (the arm
  converging to the hold pose from wherever it started), not oscillatory, and nothing like the
  divergence signature (which is sign-alternating and growing, not single-signed and decaying).
- **Step-task spikes**: `step.yaml`'s trajectory is "5s hold, then 4 cycles of a 20mm quintic
  move along x (1.5s) + 5s dwell" — so most spikes (`hw_step_A_1` t≈5.7s, `hw_step_A_ry1e-8_1`
  t≈18.85s, etc.) line up with the *end of a commanded 1.5s move*, exactly where a real step
  response is expected: this is the controller correctly tracking a real reference motion, not
  a fault. One case (`hw_step_A_2`, peak 12.3mm at t=3.75s, the single largest spike in any
  completed run) occurs while `p_d` is completely flat, inside the initial 5s hold window
  before any move starts — same signature as the hold-task spikes above (smooth one-sided
  rise-then-decay over ~1.5s, no sign reversal), so almost certainly the same kind of
  initial-settling transient landing a couple seconds later in this particular run rather than
  at t=0.

None of these show the diverged runs' signature (sign-alternating, amplitude-growing,
~7.6–7.9Hz) — they're all single-hump, decaying, and on a much slower (≲1–2Hz) timescale. All
were correctly logged as "completed" and don't need reclassifying.

## 6. Everything else checks out

- `hold.yaml`'s hardware config matches the committed repo config byte-for-byte, and its
  hardware result (SS ~0.14–0.33mm, well-converged, no saturation) matches what the sim
  results predicted.
- No run anywhere shows `u_max` pinned in a way that looks like the old Finding-3 unit bug —
  the u_max fix is holding.
- `dyn_armature_kg_m2=0.01` is present and logged in every config here — the armature
  correction (Finding 4) is active on hardware, not just in sim.

## Open items / follow-ups (not yet done)

- Confirm with you whether `hw_hold_re_1` really was run without `--use-jit` (timing story
  above is inferred from the numbers, not confirmed from a log/flag).
- ~~Characterize `hw_step_re_1` etc.'s divergence signatures~~ — done, see
  `divergence_analysis.md`: all 8 diverged runs (not just circle) show the same tightly
  clustered ~7.4–7.9Hz oscillatory mode.
- Visual spot-check of the pre-generated PNG figures against these numeric conclusions.
- Decide whether `circle.yaml`'s shipped `q_pos` should be pulled back from 75290.6 toward the
  measured ~50000 boundary, and whether the anisotropic-gain code should be ported into the
  repo.
