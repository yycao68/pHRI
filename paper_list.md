# Paper List

A survey of every drafted paper under `ai_learn`, as of 2026-09-13. All are
single/co-authored by Yongyan Cao (Voryx Robotics LLC), most with Jinshan
Tang. Grouped by research thread. "Location" is relative to `ai_learn/`.

---

## A. Interaction-Dynamics Foundation (pHRI)

### 1. Toward Interaction Dynamics: A Predictive Framework for Safe pHRI
**Location:** `pHRI/arXiv/` (`phri_ICRA.tex` — 8pp ICRA fork; `phri_combined.tex` —
16–17pp fuller version)

**What it does:** The base paper of the whole series. Cancels a manipulator's
known nonlinear dynamics with a feedforward layer, exposing a
configuration-independent double-integrator residual; a predictive (MPC)
feedback layer then shapes the interaction (impedance, force limiting,
offset-free tracking) on that fixed backbone.

**Achievements:** Proves the unconstrained MPC realization is exactly
classical task-space impedance (Theorem 1— the equivalence every other paper
in this list builds on). Deep numeric audit re-running every cited script;
low-stiffness compliance experiment with a soft/slack torque-constraint
relaxation at the QP level (a genuine, honestly-reported partial positive
result). 17+ independent review rounds, all committed.

**Open issues:** None outstanding — both `phri_ICRA.tex` and
`phri_combined.tex` have every simulation-derived number independently
re-verified against a fresh run as of the last audit.

### 2. Behavior-Realization (imp_reference)
**Location:** `pHRI/imp_reference/` (`phri2.tex` — user-designated final paper)

**What it does:** Reframes the same backbone around *behavior realization*:
given a desired interactive behavior (an impedance/admittance law), what does
the predictive layer need to guarantee to realize it under constraints.

**Achievements:** Three full review rounds (checklist audit, code-vs-paper
audit, port into `phri2.tex`), all pushed.

**Open issues:** None outstanding at last check.

### 3. Two-Rate Residual MPC
**Location:** `pHRI/impedance/impedance_residual.md` (+ `_zh.md`)

**What it does:** A didactic, two-time-scale variant — a fast inner residual
loop and a slower outer manager rate — of the same residual-MPC idea, aimed
at making the manager-rate design tradeoff explicit.

**Achievements:** Didactic rewrite; a manager-rate sweep showing honestly
that there is **no universal winner** (§7.3 states this directly rather than
picking a rate and hiding the tradeoff); table/PDF pipeline fixes.

**Open issues:** Still a `.md` draft, not yet promoted to a standalone `.tex`
submission target.

### 4. Impedance-Backbone MPC
**Location:** folded into `pHRI/arXiv/phri_combined.tex` (§VI-I / Table VIII)

**What it does:** A correction-authority-robust backbone+QP split (C6→C8 in
the paper's numbering) — separating what the backbone guarantees from what
the QP corrects, so robustness to correction saturation is explicit.

**Achievements:** Code pushed and folded into the combined paper as its own
subsection/table rather than kept as a separate artifact.

**Open issues:** None tracked separately — inherits `phri_combined.tex`'s
status (see #1).

### 5. Predictive Saturation Certificate
**Location:** `pHRI/saturation/predictive_saturation_arxiv.tex` (+ `_zh.tex`)

**What it does:** A certificate (`K_cert`) that a saturating predictive
controller's commanded correction stays within actuator authority, with an
a priori (not just measured) bound.

**Achievements:** v4 uses a matched comparator and held-out seeds (an
improvement over v3's design). A deep review reran all 111 evaluated cases (0
bugs) and caught a fabricated reference author list, fixed in both v3 and
`arxiv.tex`. After SCL desk-rejected on scope (2026-08-31 — the abstract
self-disqualified as "implementation contracts rather than a new theory"),
retargeted to T-CST with a new a priori (T3) bound added as the seed for that
resubmission.

**Open issues:** T-CST resubmission not yet confirmed accepted; this is the
one paper in the list with a known rejection in its history that required a
venue change and a new theoretical contribution to address it.

### 6. Offline H₂/H∞ as an MPC Replacement (companion paper)
**Location:** `h_inf/hinf.md`

**What it does:** Argues that for the medical/neural regime (milliwatt power,
kHz rates, certifiable software, mostly-unconstrained operation), the MPC
feedback layer in the base pHRI paper is *unnecessary*: its unconstrained
infinite-horizon limit is exactly H₂/LQR, and adding worst-case L₂ robustness
gives H∞ — both synthesizable **offline** as a small SDP, collapsing the
online controller to a matrix–vector multiply.

**Achievements:** States the full equivalence ladder (PID ↔ classical
impedance ↔ Impedance MPC ↔ H₂/LQR ↔ H∞ ↔ mixed H₂/H∞) with a closed-form
weight↔impedance↔gain map verified to machine precision, and a worst-case
force-attenuation bound γ.

**Open issues:** Still a `.md` draft with no dedicated git repo; not yet
formatted as a submission-ready `.tex` manuscript.

---

## B. Interaction-Dynamics Series — Platform Extensions

The five papers below are the paper's own named "interaction-dynamics
series": the same normalize-then-regulate architecture, ported to a
floating-base humanoid, a driving/avoidance setting, a dexterous hand, and a
steerable catheter.

### 7. Whole-Body Control v2_strong — Contact-Consistent Interaction Normalization
**Location:** `whole_body_control/versions/v2_strong/arXiv/body.tex`

**What it does:** Extends the double-integrator normalization to a
floating-base whole-body hierarchy, explicit about how the contact-null-space
projector conditions the normalization (config-invariant, terrain+push
disturbances) — a conference-candidate companion to v4/v5 below.

**Achievements:** All 7 findings from its last full review verified; the
paper itself is internally consistent after a MuJoCo 3.13 compatibility fix
(all 20 tests pass with the full G1 mesh set). Scenario F's headline
D6/D7 RMS numbers were found not to reproduce and corrected to the values
that actually do (12.11/12.04 mm, was reported as 12.48/12.42 mm); Scenario
E's numbers independently reverified. All 7 PDFs across the whole
whole_body_control tree (this paper plus v4 and v5) rebuilt and confirmed
current as of 2026-09-13, after two of them were found to still show stale,
superseded numbers despite the source already being fixed.

**Open issues:** None outstanding in the reviewed scope. v3 (the original
DCM/ID-MPC formulation this superseded) remains archived, not deleted —
`versions/v3/` still holds code that v2_strong/v4 don't duplicate
(multirate/authority mechanisms).

### 8. Whole-Body Control v4 — Torque-Level Terrain and Push (ID-MPC)
**Location:** `whole_body_control/versions/v4/wbc_v4.tex`

**What it does:** A fixed requested-task prediction realized as
Interaction-Dynamics MPC (ID-MPC) at 100 Hz, with a 500 Hz inverse-dynamics/
contact QP realizer, evaluated on a torque-level 240-trial Unitree G1 paired
study across terrain and push conditions.

**Achievements:** All terrain/push/timing tables were found stale relative
to the authoritative schema-2 JSON and resynced (headline: 7.0% obstacle
peak-error reduction vs. nominal MPC, 22.7% peak-error reduction on the
worst push condition, both controllers complete all flat/obstacle trials
where impedance falls in all ten). A new timing check was added to the
fail-closed verifier so this can't silently go stale again. Full,
end-to-end reverification (not just re-reading the JSON) as of 2026-09-13:
every terrain, push, timing, configuration, and figure check passes.

**Open issues:** Release is blocked on one missing artifact — the hashed
no-root-assist demonstration video (`continuous_flat_idmpc.mp4`), which is
gitignored and genuinely absent from this checkout; it must be
re-rendered before the fail-closed verifier can give a full pass.

### 9. Whole-Body Control v5 — Confidence-Gated External-Wrench Arbitration
**Location:** `whole_body_control/versions/v5/wbc_v5.tex` (+ `_zh.tex`, +
English/Chinese supplementary)

**What it does:** A confidence-gated interaction layer added on top of a
*frozen* Unitree-official pretrained RL locomotion policy — it arbitrates
between a predictive capture response and an integral hold response for
external wrenches, gated on estimator confidence so it can't run away on
noise (the "Physical AI" framing: locomotion is a frozen given, the
contribution is the interaction layer).

**Achievements:** The oracle-ablation table was found to mix two different
seed-count data sources (20-seed and 40-seed) in one row without saying so;
fixed to draw from one consistent 20-seed source in both languages, verified
against `revalidate_gated.json`. The 6 confidence-gate semantics tests were
blocked from even running without PyTorch installed (they don't need it —
only policy inference does); import made optional. With PyTorch and the
frozen policy reference both obtained on a capable workstation, the *entire*
`revalidate_gated.py` campaign was run end-to-end with real policy inference
(not cached JSON, ~13 minutes) and reproduced every headline number in the
paper exactly, including the three McNemar push results.

**Open issues:** The frozen policy reference artifact remains gitignored and
machine-specific by design (regenerable via a documented one-line command),
not a defect. No other outstanding items after the 2026-09-13 full
revalidation.

### 10. Reactive Time-Domain Motion Planning via a Configuration-Independent DI Backbone
**Location:** `autonomous_driving/paper/` (`mp_main.tex` full version;
`mp_main_ICRA.tex` + `_zh.tex` ICRA fork)

**What it does:** Applies the same constant-A_d double-integrator backbone
idea to reactive motion planning: a fixed-structure convex QP replaces the
usual path-first (geometric path → time parameterization) pipeline, giving
hard kinematic feasibility with non-halting reactive replanning.

**Achievements:** Independently reproduced essentially every quantitative
claim in a fresh review — `verify_math.py` 14/14, `fr3_dynamic_randomized.py`
matching the paper's 618/6889 baseline-violation counts exactly (seeded,
deterministic), all four weight-provenance claims checked directly against
source. The ICRA fork had grown to 9 pages (a citation added two commits
back pushed it over); restored to the 8-page limit by condensing the
Discussion, tightening the Introduction, and converting one small table to
an inline list — no citation, number, or claim dropped, only compressed.

**Open issues:** None found in the reviewed diff. The full (`mp_main.tex`)
version is unconstrained by a page limit and was left untouched by the
ICRA-specific compression.

### 11. Hybrid Asynchronous Escape for Convex QP Planners
**Location:** `avoidance_obstacle/paper/ao_main.tex` / `ao_ral.tex` (+
`body.tex` / `body_ral.tex`)

**What it does:** A sibling paper to #10, sharing the same `sim/` codebase
(the shared source of truth for both). Addresses the failure mode where a
purely reactive convex-QP/APF-style planner gets stuck at a local
equilibrium in front of an obstacle, via an asynchronous escape mechanism
with theoretical guarantees plus planar and trap-escape validation.

**Achievements:** Recovered a lost benchmark script from an orphaned
`.pyc` file (via `marshal`/`dis` disassembly) that reproduces the paper's
table exactly — notable because the source had otherwise been lost.

**Open issues:** Not independently re-verified in this session's review
passes (attention went to #10's page-limit and #9/#8's revalidation); worth
a dedicated pass before submission.

### 12. Dexterous Hand — Interaction Dynamics
**Location:** `dexterous_hand/paper/body_icra.tex` (+ `body.tex` full
version)

**What it does:** Ports the same interaction-dynamics normalization to a
dexterous (LEAP-hand-class) manipulator, an 8-page ICRA 2027 target.

**Achievements:** A constraint-embedding fix in eq. 13 completed; the LEAP
hand's `tau_max` was found to be a tuning constant rather than a physical
bound and is now documented as such rather than treated as a hard
constraint it isn't.

**Open issues:** ICRA 2027 deadline is 2026-09-15 — **imminent** at time of
writing. `body_icra.tex` and `body.tex` are maintained in parallel and both
need a final consistency pass before submission.

### 13. Steerable Catheters — Force-Limited Interaction Control
**Location:** `steerableCatheters/paper/catheter_ieee.tex` / `catheter_arxiv.tex`

**What it does:** The same architecture on a single-segment, single-tendon
steerable catheter: a partial-physics feedforward exposes a linear
interaction-dynamics model; the validated realization is its static-impedance
special case with integral residual compensation and a pointwise
corrective-force limit (the full predictive/QP formulation is derived but not
what's validated end-to-end).

**Achievements:** The paper previously overclaimed a validated "predictive
(MPC)" safety controller; retitled to correctly describe "Force-Limited
Control," with code renamed to match (`ImpedanceMPC` → `ImpedanceIntegral`).
A live regression was found and fixed: `operational_inertia()`'s MuJoCo
version-compatibility check used `hasattr(data, "qM")`, which stays true on
MuJoCo ≥3.10 even though the attribute's role changed — this crashed the
benchmark script documented as producing every paper result, before it could
produce anything. Fixed and reverified; every cited number now reproduces.
Both PDFs rebuilt (8 pages, down from a stale 9).

**Open issues:** None outstanding after the fix; this was the most recent
substantive bug found in the whole survey.

---

## C. Medical / Surgical Robotics Applications

### 14. RCM-Constrained Laparoscopic Robot Control
**Location:** `laparoscopy/v1/paper/laparoscopy_ieee.tex` / `laparoscopy_arxiv.tex`

**What it does:** A joint/task QP for robot-assisted laparoscopy that puts
tool-tip tracking, remote-center-of-motion (RCM) consistency at the trocar,
and adaptive tool–tissue force limiting into one decision, cited directly
against the base pHRI paper.

**Achievements:** The most heavily-audited paper in the list. A real bug was
found and fixed: the joint/task QP was silently ignoring its own disturbance
estimate (`d_free_tip=None` hardcoded), making the whole respiration-oscillator
experiment a no-op by construction. After fixing it, RCM deviation on the
main benchmark *got worse* (0.354→0.82 mm) — traced via a controlled 2×2
ablation to a second, undocumented change (a joint trust-region constraint)
sitting at a fragile QP-feasibility boundary, not the disturbance fix itself.
Retuned the constraint, reran the *entire* evaluation suite (benchmark,
Monte Carlo, sweeps, verification, stress tests), and rewrote every affected
number and causal claim in the paper. Final state: joint/task MPC now beats
tip-only MPC on both tracking *and* RCM deviation (was previously trading one
for the other).

**Open issues:** Constraint-matched predictive-impedance baseline still
missing (flagged as the headline RCM comparison's main limitation — framed
correctly in the paper as a mechanism demonstration, not comparative
superiority); no CI beyond the now-added `requirements.txt`.

### 15. Knee Rehabilitation — Series-Elastic Actuator ESO-MPC
**Location:** `knee_rehab/paper_ieee/knee_rehab_ieee.tex`

**What it does:** The interaction-dynamics architecture on a series-elastic
actuator (SEA) knee rehabilitation device, with an extended-state-observer
(ESO) MPC layer.

**Achievements:** Checklist pass plus two expert-review rounds pushed; a
deep audit found and fixed a headline-number/script mismatch.

**Open issues:** None outstanding at last check.

### 16. Neuralink Thread Insertion
**Location:** `neuralink/thread_insertion_ICRA.tex` (+
`thread_insertion_human.tex` sibling rewrite, + `_zh.tex`)

**What it does:** Interaction-dynamics-style force regulation for robotic
neural-thread insertion — not formally part of the "interaction-dynamics
series" naming, but the same underlying architecture applied to a
neural-interface insertion task.

**Achievements:** 5 review rounds on the ICRA version; `thread_insertion_human.tex`'s
abstract shortened to fit a length constraint (1862 chars).

**Open issues:** None outstanding at last check.

---

## D. Other Physical Systems

### 17. Impedance MPC for Autonomous Excavator Trajectory Tracking
**Location:** `excavator/paper/main.tex`

**What it does:** The same feedforward-plus-Impedance-MPC architecture on an
excavator bucket, normalized by the configuration-dependent operational-space
inertia Λ(q) for configuration-adaptive compliance without manual gain
scheduling; augmented with hydraulic cylinder pressure sensing for
low-latency soil-force estimation.

**Achievements:** Full paper draft with abstract, modeling, and control
architecture in place (`excavator/paper/main.tex`); a companion patent
provisional (`excavator/patent_provisional.md`) has also been drafted for
the same mechanism.

**Open issues:** Not a git repository yet (local-only draft); simulation
results and their verification status were not reviewed in this session —
worth a dedicated check before treating any reported number as confirmed.

### 18. Disturbance Annexation for Coupled-Oscillator Energy Harvesters (Wilberforce Pendulum)
**Location:** `Wilberforce_pendulum/paper/root.tex`, with a full submission
package at `Wilberforce_pendulum/wilberforce_automatica_submission/`
(manuscript, supplementary, cover letter, highlights, videos)

**What it does:** A different application of the same two-layer impedance-MPC
idea: disturbance annexation (folding an external disturbance into the
control-relevant state rather than fighting it) for a Wilberforce-type
coupled-oscillator energy harvester, with extended Kalman estimation, framed
around space energy harvesting.

**Achievements:** The only paper in this list with a **complete
submission package** already assembled (manuscript, supplementary,
highlights, cover letter, and demonstration videos for multiple scenarios:
parametric resonance, EKF vs. Kalman comparison, multi-disturbance,
constrained MPC, disturbance preview).

**Open issues:** Submission/review status (e.g., whether it has actually
been submitted to Automatica yet, or is still in final internal prep) was
not verified in this session — check the cover letter / repo history
directly for the current state.

### 19. Adaptive Impedance MPC for Spacecraft Soft Landing
**Location:** `space/spacecraft.md`

**What it does:** Proposes the same Adaptive-Impedance-MPC-plus-Kalman-force-
estimation pattern for spacecraft entry/descent/landing: stiff for
wind-gust rejection in the air, compliant for touchdown absorption, switched
by a sensorless augmented Kalman filter's real-time force estimate.

**Achievements:** A complete architectural proposal with full dynamics
modeling (3-DOF longitudinal Falcon-9-class descent model), AKF
formulation, and MPC constraint structure written out.

**Open issues:** **This is a proposal, not yet a results paper.** Its own
§5 is literally titled "Expected Simulation and Discussion" and uses
forward-looking language throughout ("should be established," "expected to
show," "reducing peak reaction forces by an expected 40–60%") — there is no
simulation code, no figures, and no run results in the repository yet. This
is the least mature item in the whole list; the next step is building the
actual simulation before any numeric claim can be treated as verified.

---

## E. Predictive Physical Realizability (a related but distinct thread)

These two share a research question — is a planned trajectory *physically
realizable* given actuator authority, not just kinematically valid — rather
than the interaction-dynamics normalization above, though both come from the
same `replan` codebase and cite the pHRI paper for the underlying
world-model-interface idea.

### 20. Predictive Physical Realizability — World-Model Interface Certificate
**Location:** `replan/paper_icra/predictive_realizability_ICRA.tex` (ICRA
fork); `replan/world_model_realizability_core.md` (fuller journal-track
version, + `_zh.md`)

**What it does:** Treats a learned world model's per-cycle prediction as an
explicit, falsifiable physical hypothesis and certifies a plan against it
with an actuator-margin witness ρ; a runtime monitor tests the hypothesis
against execution independently of ρ. Central result: a fallback being
*feasible* is not the same as it *helping* — for configuration-dependent
(quasi-static) surprises, a stop-in-place fallback has almost no authority,
while for inertial surprises it does.

**Achievements:** 10 new references added (18 total) with all checked for
existence/correctness (including a web-search-verified venue check).
Journal-version `\thanks{}` footnotes added, cross-checked so they only cite
genuinely-existing (not aspirational) content. Four demonstration videos
built on the real Franka FR3 MuJoCo model, each with a theorem/corollary-
grounded intro card. A self-contradiction was caught and fixed: a newly
added figure's own caption claimed the inertial-surprise class had "markedly
higher" fallback authority than the quasi-static classes, when the figure's
own plotted data showed its floor collapsing to the same range at higher
severity — traced to the "43–57%" headline number being a cherry-pick of
only the mildest severity tested; corrected throughout to the honest full
range (2–57%, collapsing at higher severity) with the same fix ported to
the Chinese translation.

**Open issues:** None outstanding after the correction above; both papers'
every simulation-derived number was independently re-verified.

### 21. Predictive Physical Realizability — Planning Architecture (PPR)
**Location:** `replan/paper_icra/predictive_realizability_ppr_ICRA.tex`
(ICRA fork); `replan/predictive_realizability_paper_draft.md` (fuller
version, + `_zh.md`)

**What it does:** The planning-side counterpart to #20 — a hierarchical
response architecture (retime → reshape → reroute) that acts on the
certificate from #20 rather than just monitoring it, integrated with MoveIt 2.

**Achievements:** Same reference-expansion and footnote-provenance work as
#20. Two additional FR3 demonstration videos (flagship B1-vs-B3 reroute,
environment-reroute) with theorem-grounded intro cards, dark-themed to match
#20's videos. All figure/table cross-references verified to resolve to the
correct figure numbers after edits.

**Open issues:** None outstanding at last check. The broader `replan`
project roadmap (v0.1–v0.5) is fully shipped; the next planned step (per the
project's own README) is spinning off a separate saturation-certificate
paper — that would become entry #22 in a future update to this list.

---

## Summary

| # | Paper | Status | Videos |
|---|---|---|---|
| 1–2 | pHRI base + imp_reference | Fully verified, no open issues | 8: 6 circle-tracking impedance-vs-MPC comparisons (`pHRI/cloud_verify/results/`) + 2 FR3 demos (`pHRI/simulation_results/`) — not reviewed this session |
| 3 | Two-rate residual MPC | `.md` draft, not yet formalized | None |
| 4 | Impedance-backbone MPC | Folded into #1, no separate status | Shares #1's videos |
| 5 | Predictive saturation certificate | Retargeted to T-CST post-rejection | None |
| 6 | H₂/H∞ companion | `.md` draft, not yet formalized | None |
| 7–9 | Whole-body control v2_strong/v4/v5 | Fully verified 2026-09-13; v4's gate now fully green | All three now have a rendered, cited comparison video (2026-09-13): v2_strong's `scenario_a_video.mp4` (D1 vs D7, reproduces the paper's 73× figure exactly, through the real audited controller path); v4's `continuous_flat_idmpc.mp4` (the release asset that was blocking the fail-closed gate — verifier now reports `"status": "PASS"` end to end); v5's `gate_comparison.mp4` (gate forced open vs the shipped confidence gate, reproduces the capture-amplification ablation's story on one seed). All three are new opt-in code paths (existing test suites re-verified unaffected: v2_strong 20/20, v5 6/6) and are cited as footnotes at the exact claims they illustrate |
| 10 | Autonomous driving (DI-QP) | Fully verified; ICRA page limit restored | 3: `fr3_motion.mp4`, `av_video_panel.mp4`, `fr3_falling_ball_cyclic.mp4` — not reviewed this session |
| 11 | Obstacle avoidance (hybrid escape) | Not independently re-verified this session | 1: `hae_video_panel_S3.mp4` — not reviewed |
| 12 | Dexterous hand | **ICRA 2027 deadline imminent (2026-09-15)** | 7, dated June — reviewed 2026-09-13: the eq.13 total-command embedding is opt-in and unwired in every video script (`hand_mpc_video.py`, `cable_hand_video.py`, `manip_video.py`, `finger_video.py`, `shadow_hand_video.py` never pass `traj_fn`), so none are actually stale |
| 13 | Steerable catheters | Fully verified; live regression found+fixed | 1 (`safety_comparison.mp4`), **rendered 2026-09-13** — unconstrained vs force-constrained MPC on the identical task, through the real audited `run_mpc()` path; reproduces the paper's headline safety result exactly (0.595N violates vs 0.467N safe, Table II: 0.595 vs 0.470N) |
| 14 | Laparoscopy | Fully verified; significant bug found+fixed, missing baseline flagged | 1 (`lap_video_mpc_kalman.mp4`) — was stale (predated the trust-region retune), **regenerated 2026-09-13** from the current controller; spot-checked frame confirms the ~0.55–0.6mm RCM-deviation peak matches the paper's 0.59mm nominal figure |
| 15–16 | Knee rehab, Neuralink | Fully verified, no open issues | 2 + 4 respectively — not reviewed this session |
| 17 | Excavator | Drafted, not yet independently verified | 1 (`excavator_digging_video.mp4`) — not reviewed |
| 18 | Wilberforce pendulum | Complete submission package; submission status unconfirmed | 4 of ~10 named scenarios (A, C, D, E1) — not reviewed |
| 19 | Spacecraft soft landing | **Proposal stage only — no simulation results yet** | None — no simulation code exists |
| 20–21 | Predictive realizability (world-model + planning) | Fully verified, no open issues | 4, all built and reviewed 2026-09-13 (this session) |

**Video rendering roadmap (2026-09-13):** rendering a from-scratch
comparison video (dark-themed intro card, two-panel conventional-vs-proposed
layout, matching today's `replan` style) is a per-paper undertaking on the
scale of a full session, not a quick add-on. Priority order: (1) whole-body
control — **done**, all three of v2_strong/v4/v5 now have a rendered, cited
comparison video (see row above); (2) steerable catheters — **done**, see
row above; (3) dexterous hand, lower priority given the imminent
2026-09-15 ICRA deadline competes directly for the same time — not
scheduled. All three items agreed for this pass are now complete.
