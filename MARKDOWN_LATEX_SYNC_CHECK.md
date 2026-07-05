# Markdown-LaTeX Sentence Sync Check

Compared:

- `pHRI/double_integrator_phri_ieee.md`
- `pHRI/arXiv/phri_main.tex`

## Canonical Choice

The LaTeX file is the compact submission version. The Markdown file is the
expanded readable version. When the two differed in a technical claim, I used
the corrected backbone-centered/theory-consistent wording and synchronized both
files. When the Markdown had extra explanatory sentences but no different claim,
I left the LaTeX compact for page length.

## Sentence-Level Corrections Applied

| Location | Mismatch found | Correct synchronized wording |
|---|---|---|
| Abstract | Markdown said `linear predictive-control form`; LaTeX said `linear constrained-control problem`. | Both now use the constrained-control framing, because the backbone is valuable for constraints, offset-free tracking, stability, and invariance. |
| Abstract | Both still over-emphasized `Impedance MPC` in the contribution sentence. | Both now say `As one finite-horizon realization...`; the main contribution remains the linear double-integrator backbone. |
| Abstract results | Both described the result as `backbone-based Impedance MPC`. | Both now use `backbone-based predictive controller with Kalman augmentation`. |
| Section III title | Both used `Linear Double-Integrator Backbone and Impedance MPC`. | Both now use `Linear Double-Integrator Backbone and Finite-Horizon Realization`. |
| Contribution 3 | Old sync report said `Impedance MPC realization third`. | Both texts use `Finite-horizon realization of the backbone`. |
| QP linear term | Real-time implementation in Markdown omitted the `+\bar R d_N` term. | Both now state `h = Gamma^T Qbar x_free + Rbar d_N`; this matches the offset-free input-centering proof and code. |
| Corollary 1 | Markdown called it `infinite-horizon Impedance MPC`. | Markdown now says `infinite-horizon predictive realization`; LaTeX already used `infinite-horizon MPC` compactly. |
| Conclusion | Both said `Impedance MPC is one realization`. | Both now say `A finite-horizon predictive controller is one realization of the backbone`. |
| Future work / passivity | Markdown said `across multiple platforms and extend the energy-tank passivity augmentation`; LaTeX said `extend energy-tank passivity augmentation`. | The energy-tank item is no longer future work: both files now include it in the method section with force scaling, tank update, and a sampled passivity proposition. |

## Technical Claims Confirmed Synced

| Area | Status |
|---|---|
| Theorem 2 | Both use the centered-input proof with `V=U+d_N` and `D_bar=Gamma(1_N \\otimes I_3)`. |
| QP equation | Both define `h=Gamma^T Qbar x_free + Rbar d_N` and the input-centered penalty `||U+d_N||_Rbar^2`. |
| Torque constraint | Both constrain the first applied total torque `tau_base + J_v^T F_mpc(0)` rather than only the Cartesian correction. |
| Prop. 1 pHRI force | Both use the acceleration-form simplification `-Lambda^{-1} F_h`, not `-F_h`. |
| Orientation coupling | Both say the additive orientation channel creates a structural cross-mobility term absorbed by `d_hat`. |
| Theorem 3 | Both include external generalized force `hat_tau_ext,k = J^T hat F_h,k` or a robust tightening assumption. |
| Certified safety | Both scope joint-limit certification to feasible/enforced one-step CBF constraints and CBF residual/slack logging. |
| Passivity certificate | Both state the task-force energy tank and the logged certificate fields `passivity_certified`, `passivity_energy`, and `passivity_scale`. |
| MuJoCo verification path | Both now distinguish direct benchmark scripts from the hardware-path MuJoCo suite. | Certified CBF/passivity claims use `run_sim_verification_suite.py`, which runs through `FR3ImpedanceMPCHardwareInterface`; direct scripts remain figure/algorithm benchmarks. |

## Intentional Differences Left

- The LaTeX file is anonymized; the Markdown file keeps author information.
- The Markdown file contains longer explanatory prose in the introduction,
  barrier smoothness discussion, experiment analysis, and platform-general
  discussion. The LaTeX version keeps the same claims but compresses those
  paragraphs for IEEE page length.
- Citation style is intentionally different: Markdown uses numbered references;
  LaTeX uses BibTeX keys.
- Tables and figures are equivalent in claim content but not sentence-identical,
  because LaTeX captions are shorter.

## Reviewer-Response Pass (interaction-dynamics framing)

Synced to both files in response to RA-L/T-RO-style review feedback:

| Location | Change (both files) |
|---|---|
| §II Problem Formulation | Added **Definition 1 (Interaction Dynamics)** — formalizes the concept as the augmented error system $x_{k+1}=Ax_k+Bu_k+Ed_k$ vs. the configuration dynamics $M\ddot q+C\dot q+G=\tau+J^\top\mathcal F_h$. New `definition` theorem environment added to the LaTeX preamble; label `def:id`. |
| §III-B after Eq. (LPV) | Added a sentence explicitly naming Eq. (7)+(9) as "the interaction dynamics of Definition 1 in concrete form." |
| Introduction | "We call this perspective interaction dynamics" now references Definition 1 and adds the vision line "we advocate predicting the interaction evolution directly." |
| Contributions | C1/C2 de-duplicated: C1 = "A predictive interaction-dynamics formulation," C2 = "A configuration-independent interaction model." C3 retitled "Offset-free predictive impedance" (states impedance is a special case); C4 retitled "Safety and passivity integration." |
| Theorem 1 | Added a motivating lead-in: classical impedance is a special case, not a competitor. |
| Conclusion | Opens with the "classical control predicts robot dynamics; this work predicts interaction dynamics" framing. |
| Abstract | Lightly trimmed the safety-filter enumeration per the "abstract too full" note. |

### Predictive variable-impedance baseline (MPVIC) — implemented and run

Addressed review point #6 by implementing a predictive variable-impedance
comparator and running it through the real MuJoCo benchmark pipeline.

- **Code:** new `variable_impedance` mode in `simulation/impedance_mpc.py`
  (`_select_variable_stiffness` + a `control()` branch) and dispatch/roster
  wiring in `simulation/phri.py` (`make_mpc_controller`, `ALL_CONTROLLERS`,
  colour/style/width maps). The baseline differs from the proposed controller
  *only* in the task-force law: it selects the apparent stiffness $K^\star$ by
  horizon rollout using the same Kalman $\hat d$, but does **not** cancel it
  (no offset-free term), so it retains $e_\infty=-K^{\star-1}\hat d$.
- **Numbers (Benchmark I, 3 cycles, real FR3 MuJoCo):** the pipeline reproduces
  the published C1–C7 rows exactly (e.g. C5 = 0.52/2.53/0.034 mm contact/peak/SS;
  C7 = 0.16/0.77/0.023). New MPVIC row: **RMS 12.8, contact 4.5, peak 7.5,
  SS 4.8 mm** — beats all reactive baselines but its finite stiffness leaves a
  4.8 mm residual vs. C5's 0.034 mm ($\sim$140×).
- **Both files updated:** Section VI-A baseline table + description, Table I row,
  a new analysis finding ("adapting the stiffness is not enough"), and the
  "full ablation" caption wording. LaTeX cites `liu2025model`/`roveda2019optimal`.
- **Table III (Benchmark II) also done.** Wired the same baseline into the
  reach-and-hold runner `simulation/guidance.py` (`make_mpc_controller` +
  `MPC_NAMES`). Runner reproduces the published G1/G5/G7 rows exactly
  (G1 63.6/41.4/47.1; G5 47.7/0.58/2.47; G7 52.1/0.18/0.75). New MPVIC row:
  **3/3 waypoints, free 51.1, contact 5.0, peak 5.9 mm** — reaches every
  waypoint, ~8× better contact than stiff impedance via predictive stiffening,
  but ~9× worse than the offset-free G5. Row + analysis added to both files
  (MPVIC stiffens predictively, the opposite of the reactive G3 that softens;
  neither substitutes for offset-free cancellation).

Both benchmark tables (I and III) now carry the MPVIC baseline.

**Comparison figures regenerated with the MPVIC curve.** Added the MPVIC name to
`PAPER_CONTROLLERS`/`PAPER_LABELS` (and colour/style/width maps) in both
`simulation/phri.py` (Benchmark I) and `simulation/guidance.py` (Benchmark II),
then regenerated headless:
- `python phri.py compare --no-viewer` → `simulation_results/mpc_comparison_results.png`
- `python guidance.py compare --no-viewer` → `simulation_results/guidance_controller_comparison.png`

Both PNGs copied into `arXiv/figures/`. MPVIC renders as a brown dashed curve
(legend "MPVIC Var.-Imp. MPC") in the error time series, trajectories, and
performance-summary bars. Figure captions (fig:bm1, fig:bm2) and the markdown
"paper plot shows…" sentences updated to state the plots now include the MPVIC
baseline alongside D1/D2/D3/D7. Benchmark II suptitle string updated likewise.
`trajectory_boundary.png` unchanged (Table IV compares only IMP vs. C5).
LaTeX rebuild: passed, 11 pages, no undefined refs.

LaTeX rebuild after this pass: **passed, 11 pages, `def:id`/`thm:equiv` refs resolved, no undefined references.**

## Model-based Physical AI positioning (vision layer)

Added the same framing to both files. Deliberately scoped to the **vision layer
only** — title, contributions, theorems, and experiments are UNCHANGED — to
position the work as an analytic interaction-dynamics *foundation for* model-based
physical AI without over-claiming to *be* a physical-AI system (the distinction
matters for conservative RA-L/T-RO reviewers).

| Location | Change (both files) |
|---|---|
| Abstract | +1 closing sentence: the configuration-invariant backbone is an analytic interaction-dynamics prior; a learned model need only capture the residual (intent, contact, environment) rather than re-learning robot dynamics. (LaTeX: replaced the older "foundation for interaction-centric control" sentence.) |
| Introduction | +1 paragraph after the contributions/results block connecting to world models $x_{k+1}=f(x_k,u_k)$: contact is the least data-efficient layer; the analytic backbone supplies it, leaving only the uncertain residual to a learned component. Explicit "we do not claim a physical-AI system." |
| Conclusion | +1 closing vision paragraph: interaction dynamics as a candidate analytic, predictive, safety-constrained substrate between a foundation model and robot torques. |

LaTeX rebuild after this pass: passed, 11 pages, no undefined references.

## Defensive-precision pass (burden-of-proof review)

A reviewer noted the elevated "Interaction Dynamics / Physical AI" framing raises
the burden of proof without introducing new math errors. Applied the low-risk,
claim-tightening fixes to both files:

| # | Fix | Both files |
|---|-----|------------|
| 5 | "configuration-independent" is now attached ONLY to the transition matrix, never to the model. Contribution 2 retitled "A configuration-independent state-transition matrix" (was "...interaction model") with an explicit "the model as a whole is not config-independent" clause. Abstract, intro Physical-AI paragraph, and conclusion Physical-AI paragraph all reworded from "configuration-invariant model" to "linear model with a fixed, configuration-independent transition matrix." | ✓ |
| 2 | Scoped the term: added a sentence stating we use "interaction dynamics" in a specific modeling sense (augmented error-and-force dynamics at the contact port), closely related to operational-space/error-dynamics formulations [13],[25], differing in what is the modeled state — not a claim to a new phenomenon. | ✓ |
| 1 | Sharpened Definition 1: added a passage distinguishing it from a bare tracking-error double integrator (force as an internally-modeled disturbance state + constraints, taken as one predicted-and-optimized object; "the object the controller is designed on"). | ✓ |
| 3 | Lifted the "Exactness caveat" out of Proposition 1: markdown relabels it "Remark (Exactness of the reduction)"; LaTeX (compact) adds a matching `\begin{remark}[Exactness of the reduction]`. The robustness-section back-reference updated "exactness caveat" → "exactness remark" in both. |
| — | (#7) additional interaction tasks — Option A (time-varying forces) done; see below. B/C/D (second robot, surface contact, co-manipulation) remain optional. |

## Time-varying interaction experiment (#7, Option A) — added

New self-contained, reproducible experiment `simulation/time_varying_experiment.py`
(does not touch the shared benchmark scripts). Static end-effector hold under a
sinusoidal push (12 N, frequency swept), measuring the $N$-step
disturbance-prediction RMS $\varepsilon_N$ that §III-D defines but never reported.

- **Design rationale:** on a moving trajectory the configuration-driven terms of
  (6a) make $\hat d$ oscillate at the tracking frequency, confounding the metric;
  a static hold isolates the force. A frequency sweep at fixed amplitude gives a
  monotonic $L_d$ (disturbance rate) axis.
- **Result (Table VI, both files):** the horizon-extrapolation gap
  $\varepsilon_N-\varepsilon_1$ tracks the §III-D bound term $L_d N\Delta t$ to
  within ~10% across a decade of rate (0.46/0.53, 1.08/1.07, 2.22/2.13,
  4.38/4.27 N) — confirming the error is linear in force rate, floor
  $e_K\approx0.38$ N. Tracking degrades gracefully: sub-mm up to $L_d\approx21$ N/s,
  ~1 mm at 43 N/s; offset-free recovered as $f\to0$.
- **Placement:** new experiments subsection G (markdown) / subsection after
  Robustness (LaTeX), Table VI, referencing `\eqref{eq:dist}`, `\ref{thm:zss}`.
- LaTeX rebuild: clean, 12 pages, no undefined refs.

Note: the metric is estimator-self-consistency — $\varepsilon_N=\mathrm{RMS}\,
\|\hat d(k)-\hat d(k{-}N)\|$ (flat random-walk prediction = past estimate), which
is exactly §III-D's "estimate propagated $N$ steps before the measurement." Uses
$\hat d = $ `mpc_ctrl.x_aug[6:9]` (force-form). Run with n_cycles-equivalent 11 s
static hold; `PYTHONPATH=.../simulation`.

## Framework-level stability theorem (#6) — added (Tier B)

Added a workspace-level stability theorem, de-risked first by an LMI feasibility
probe against the real FR3 `Λ⁻¹` range.

- **De-risk probe** (`cvxpy`, installed for this): sampled `Λ⁻¹(q)` along both
  benchmark trajectories (4600 samples, eig `Λ⁻¹ ∈ [0.059, 0.360]` kg⁻¹ →
  task inertia 2.8–16.9 kg), built the 64-vertex entry-wise box, solved the
  vertex quadratic-stabilizability LMI. **Feasible:** common `P` (cond ≈ 3.7),
  closed-loop spectral radius ≤ 0.996 at every vertex, Lyapunov increment
  −1.5e-3 (directly verified by eigenvalue computation, not solver-reported).
- **Theorem 3 (Workspace Stability of the Interaction Dynamics)** added to §III
  right after Remark 4: if the vertex LMI (9b) is feasible, a single quadratic
  Lyapunov function certifies exponential stability of the LPV interaction
  dynamics along ANY workspace trajectory (proof: the Schur-complement block is
  affine in `B_d(ρ)`, hence holds on the whole polytope by convexity of the PSD
  cone). Removes Remark 4's frozen-config caveat. Followed by **Remark 5
  (Numerical certificate)** citing the probe numbers.
- **Renumbering:** the new theorem is Theorem 3; the joint-limit invariance
  theorem became **Theorem 4**. Markdown renumbered manually (2 refs); LaTeX
  auto-renumbers (`\label{thm:workspace}`, `thm:invariance`) — all symbolic refs
  resolve. Contribution 2 now cites Theorem 3 (constant `A_d` ⇒ finite vertex
  LMI ⇒ workspace stability). Remark 4 rewritten to point forward to Theorem 3.
- **Honest scope in the theorem text:** certifies the predictive-plus-estimator
  core with input constraints inactive (as Theorem 2); the guaranteed common-`P`
  rate is conservative vs. the realized MPC; CBF/energy-tank filters remain
  conditional (Theorem 4, Prop 2).

LaTeX rebuild: clean, **12 pages** (was 11), no undefined/multiply-defined refs.
Note: `cvxpy` is only needed to re-run the probe, not to build the paper.

## Fresh-audit pass (current text)

Independent re-read of the current paper against the 7-point review. The review
points are well-addressed (#1,2,3,5,6 strong; #4 borderline; #7 partial by
design). Found and fixed defects introduced by the recent additions:

1. **Cross-ref bug** (markdown): "estimator and QP of §IV-C" → §IV-C is the
   energy tank; the Kalman estimator is §III-C. Fixed to "§III-B–C / see §III-C".
2. **Metric definition vs experiment** (both): §III-D defines $\varepsilon_N$
   against the true $d(k)$; §VI-G measures it against $\hat d(k\mid k)$. Added a
   sentence stating the proxy substitution.
3. **Floor mislabel** (both): §VI-G called the $f{=}0$ floor "steady-state
   estimator error $e_K$"; it is the estimate's $N$-step process-noise jitter.
   Reworded as the "$e_K$-analogue."
4. **Missing forward-ref** (both): §III-D "Metric" paragraph now points to
   §VI-G / Table VI where $\varepsilon_N$ is measured.
5. **Conclusion omitted Theorem 3** (both): added the workspace-stability
   guarantee to the conclusion's theory summary.
6. **Minor imprecision** (markdown): Definition 1 attributed the actuator
   constraint to §IV; it enters the QP of §III-B. Fixed.

No new math/logic errors: Theorem 3 proof sound, MPVIC/Table VI numbers
consistent, cross-refs otherwise resolve. LaTeX rebuild clean, 12 pages.
Judgment calls left for the author: (#4) abstract length; (#7) task diversity.

## De-duplication pass (length trim)

Audited for repeated framing claims (the "constant $A_d$ / robot dependence in
$B_d$ / precomputed $\Phi$ / 30-var QP" cluster was stated ~8–10 times).
Applied the high-confidence, zero-information-loss cuts to both files:

- **Deleted Discussion "Constant $A_d$ advantage"** — fully redundant with
  Contribution 2 (structure) and §V line 352 (OSQP < 0.5 ms timing).
- **Deleted Discussion "Platform generality"** — the "needs only $M,C+G,J_v$ /
  any torque-controlled arm" claim is already §V line 339 and the conclusion.
- **Compressed the intro Physical-AI paragraph** (~90→~55 words); the full
  forward-looking vision stays in the conclusion.

Net ~250 words removed. "configuration-independent" count 11→8 (remaining uses
are each in a distinct role). Markdown Discussion keeps its three unique
paragraphs (Separation of concerns, Joint-limit design, When-not-to-use); the
compact LaTeX Discussion keeps Separation of concerns. LaTeX rebuild clean, 12
pages. Not yet committed.

## Verification

LaTeX rebuild after this pass: passed, 11 pages, no undefined refs/citations. The
new Exactness remark shifts auto-numbering, but the only remark cross-reference
is symbolic (`\ref{rem:feasibility}`), so it resolves correctly.

## Verification

After this sync pass, rebuild with:

```bash
cd pHRI/arXiv
latexmk -pdf -interaction=nonstopmode phri_main.tex
```

Latest rebuild passed after adding the energy-tank layer and MuJoCo hardware-path verification wording.
