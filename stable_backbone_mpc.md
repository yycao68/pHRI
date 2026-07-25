# Impedance-Backbone MPC: a correction-authority-robust architecture for the pHRI controller

Status: **exploratory, not yet in the paper.** This documents a proposed architectural change, its implementation as a new controller option, and a simulation comparison against the current proposed controller (D7, `DI-MPC + Kalman 500 Hz`) — now including a protocol-matched 3-cycle re-run (§4.3) confirming the 1-cycle snapshot generalizes. **§7 (horizon scheduling) is a settled negative result: §2c's frozen-Jacobian formulation is retained.** A constant-velocity coast extrapolation was tried and removed (unjustifiable second approximation, no benefit); reference scheduling along the trajectory generator's own $(q_d,\dot q_d,\ddot q_d)$ — the theoretically cleaner alternative — was then implemented and found to be not just unnecessary but a measurable **regression** (up to 26× worse steady-state error) in the regime where the torque constraint binds, because it schedules against the undisturbed reference exactly when the real robot has deflected away from it during contact. Reference-scheduling code remains available for a future longer-horizon/slower-rate revision but is not recommended now. Nothing in `double_integrator_phri_ieee.md` or `arXiv/phri_main.tex` has been touched; question 1 in §6 has a recommendation but is explicitly left as your editorial call, not decided here.

## 1. The problem this addresses

The paper's Layer 1 ($\tau_{\rm ff}$, feedforward nonlinear inversion) reduces the closed loop to a linear double integrator only if the *entire* commanded torque is actually realized:

$$\ddot e = -\Lambda^{-1}(q)\,F_{\rm mpc} + d(t)$$

only holds if $\tau = \tau_{\rm ff} + J_v^\top F_{\rm mpc}$ is realized exactly, i.e. $|\tau_i| \le \tau_{\max,i}$ for every joint $i$.

If any predicted step needs more torque than the actuators can produce, the *real* closed loop is

$$\ddot e = \Lambda^{-1} J M^{-1}\Big[\operatorname{sat}\big(\tau_{\rm ff} + J_v^\top F_{\rm mpc}\big) - \tau_{\rm ff}\Big] + d$$

and the saturation residual re-enters the dynamics: the predicted double-integrator trajectory and the realized trajectory diverge. The paper already handles this for the **first** applied step — constraint (9b),

$$-\tau_{\max} \;\le\; \tau_{\rm base}(k) + J_v^\top(q_k)\,F_{{\rm mpc},0|k} \;\le\; \tau_{\max},$$

— and the "Constraint interpretation" remark explicitly notes that horizon-wide torque rows are not required for feasibility of the *applied* action, only for a stronger predicted-trajectory-realizability claim, which the manuscript does not make. That scoping is correct and honestly stated. But it leaves two related gaps worth closing:

1. **Steps $1,\dots,N-1$ of the horizon can imply an unrealizable torque.** Those steps still shape the first-step optimum through the QP cost, so the first step, while itself feasible, can be the output of a plan that assumes impossible future authority.
2. **The entire corrective torque comes from one box-constrained decision variable.** In every mode currently implemented (default LQ-MPC, `impedance_track`, `variable_impedance`), $F_{\rm mpc}$ *is* the whole correction. If it is forced to zero by a tight $F_{\max}$ bound or by the solver-failure fallback, the commanded torque degrades to $\tau = \tau_{\rm ff}$ alone: feedforward with **zero corrective stiffness**, not a stable regulator. The experiments below directly test the tight-bound case; they do not inject a real solver timeout or crash.

Neither gap contradicts anything currently claimed in the paper (which already scopes its offset-free/stability claims to the constraint-feasible regime), but both are real robustness weaknesses worth closing before claiming improved robustness to loss of correction authority.

## 2. Proposed architecture

Split the corrective torque into two parts instead of one:

**(a) A fixed, positively damped impedance backbone, commanded independently of the QP**

$$F_{\rm bb} = K_{\rm bb}\,e + D_{\rm bb}\,\dot e, \qquad D_{\rm bb} = 2\,\zeta_{\rm bb}\sqrt{K_{\rm bb}}$$

$$\tau_{\rm base} = \tau_{\rm ff} + J_v^\top F_{\rm bb} + \tau_{\rm orient} + \tau_{\rm null}$$

$F_{\rm bb}$ is *not* a QP decision variable — it is computed directly from the current state and folded into $\tau_{\rm base}$ every MPC update, the same way $\tau_{\rm ff}$, $\tau_{\rm orient}$, $\tau_{\rm null}$ already are. It remains in the commanded torque even if the QP below returns exactly zero.

The familiar scalar rule $D_{\rm bb}=2\zeta_{\rm bb}\sqrt{K_{\rm bb}}$ is exactly critically damped only for unit effective mass. Here the force-to-acceleration map is $\Lambda^{-1}(q)$, generally anisotropic and configuration dependent, so $\zeta_{\rm bb}=1$ is a **nominal unit-effective-mass tuning**, not an exact modal-critical-damping statement. Positive $K_{\rm bb}$ and $D_{\rm bb}$ still provide restoring stiffness and damping in the local continuous-time model. Exact modal tuning would require a $\Lambda(q)$-dependent damping matrix and a corresponding scheduled prediction model.

**(b) The QP now shapes only a bounded additional correction $F_{\rm mpc}$**

$$\tau = \tau_{\rm base} + J_v^\top F_{\rm mpc}, \qquad \|F_{\rm mpc}\|_\infty \le F_{\max}$$

predicted through **the backbone dynamics linearized along a nominal trajectory**, not the open-loop double integrator. *(Revised wording — see the note at the end of this subsection: the earlier draft said "predicted through the backbone's own closed loop," which reads as more exact than it is; "linearized along a scheduled nominal trajectory" is the accurate framing, consistent with gain-scheduling terminology.)* By default this trajectory is a single frozen point (the current configuration), giving a time-invariant closed loop:

$$G_{\rm bb} = \begin{bmatrix} K_{\rm bb} & D_{\rm bb} \end{bmatrix} \in \mathbb{R}^{3\times 6}$$

$$A_{\rm cl} = A_d + B_d(\rho_k)\,G_{\rm bb} \qquad \text{(linearized at the current configuration only)}$$

$$x_{i+1|k} = A_{\rm cl}\,x_{i|k} + B_d\big(F_{{\rm mpc},i|k} + \hat d\big)$$

The QP minimizes the same LQ running/terminal cost as the default branch ($\bar Q$, $\bar R$), built from $A_{\rm cl}$ instead of $A_d$, with the same offset-free input-centering trick ($\hat d$ absorbed into the additional term, not the backbone — the backbone alone is *not* claimed offset-free, only restoring and damped). Concretely: if $F_{\rm mpc}\to 0$, the commanded controller retains non-zero corrective stiffness instead of reverting to bare feedforward.

**Actuator-limit caveat.** QP independence is not actuator independence. If $\tau_{\rm base}$ itself exceeds the joint limits, the simulator/robot still clips it and the backbone is not realized exactly. The horizon constraint can allocate the *additional* $F_{\rm mpc}$ only when the affine feasible set is nonempty; it cannot repair an already-infeasible $\tau_{\rm base}$. Accordingly, the evidence below supports robustness to loss of **additive QP correction authority**, not a blanket guarantee under arbitrary actuator saturation.

**Note (added on review):** if the horizon-wide torque constraint (c) below is scheduled along a varying nominal trajectory rather than frozen, $B_d$ varies with it — and leaving $A_{\rm cl}$ fixed while the torque map varies would be an internal inconsistency (the map used to check torque feasibility would disagree with the map used to predict the state). §7 generalizes this to a genuinely time-varying $A_{{\rm cl},i} = A_d + B_{d,i}\,G_{\rm bb}$, scheduled together with the torque constraint, not just once at the current configuration.

**(c) Horizon-wide torque realizability (frozen-Jacobian approximation)**

Rather than only constraining the first step, replicate the same affine row — frozen at the current $J_v(q_k)$, using the *current* $\tau_{\rm base}(k)$ (which now includes the backbone) as a constant offset — for every horizon step:

$$-\tau_{\max} \;\le\; \tau_{\rm base}(k) + J_v^\top(q_k)\,F_{{\rm mpc},i|k} \;\le\; \tau_{\max}, \qquad i = 0,\dots,N-1$$

This is the simplest fix identified in the discussion: it does not track how $J_v$ or $\tau_{\rm base}$ actually evolve along the horizon (a local approximation, same caveat the paper already states for the first-step-only version), but it stops the QP from planning around a future input that is obviously unrealizable from the current configuration.

**Remark (scope of the frozen-Jacobian row).** This is the direct horizon-wide analogue of the paper's existing "Constraint interpretation" remark for (9b), and should carry the identical caveat, stated explicitly rather than left implicit: freezing $J_v(q_k)$, $\tau_{\rm base}(k)$ at the current configuration makes each row of §2c a **necessary-near-$q_k$, not sufficient-along-the-horizon** realizability check. As $q_{i|k}$ drifts from $q_k$ over the horizon, the frozen row can pass while the true configuration-dependent constraint at step $i$ would have failed (or vice versa) — it is a *local* screen against obviously-unrealizable plans, not a certificate that the whole predicted trajectory is realizable. A tight version needs the SQP/RTI re-linearization along the horizon noted in §5; until that exists, any paper language should say "frozen-Jacobian horizon-wide extension of (9b)," not "(9b) extended to the whole horizon" unqualified — the first phrasing is honest about the approximation, the second reads as exact.

Both (a)+(b) and (c) are independent knobs and were implemented as such — (c) is a general improvement applicable to any mode, not only the backbone.

## 3. Implementation

All in `simulation/`, no paper file touched.

- **`impedance_mpc.py`** — `ImpedanceMPCParams` gained:
  - `backbone_track: bool`, `k_backbone`, `zeta_backbone` — architecture (a)+(b) above ($K_{\rm bb}$, $\zeta_{\rm bb}$).
  - `horizon_torque_constraint: bool` — architecture (c) above, usable with any mode.

  `ImpedanceMPCController.control()` gained an `elif self.p.backbone_track:` branch (new helper `_build_closed_loop_horizon(Bd, A_cl)` builds $\Phi$, $\Gamma$, $\bar D$ around a supplied closed-loop $A_{\rm cl}$, mirroring the existing `_build_Gamma`/$\Phi$/$\bar D$ construction which is fixed to the open-loop $A_d$). `tau_base` assembly now folds in `F_backbone` (zero in every other mode, equal to $F_{\rm bb}$ in the backbone branch). `_torque_constraint_matrix`/`_torque_constraint_sparse` generalized from a single 7-row block (first step) to $N$ tiled 7-row blocks when `horizon_torque_constraint=True`; both the `scipy` and `osqp` solve paths tile their bound vectors accordingly.

- **`phri.py`** — `make_mpc_controller` recognizes `"Backbone"` in the controller name and sets `backbone_track=horizon_torque_constraint=True`; added `F_MAX_OVERRIDE` (monkeypatchable, same pattern as `F_HUMAN`) so a comparison script can sweep the corrective-force bound $F_{\max}$ without duplicating the controller-construction code. New controller name used below: `"DI-MPC + Kalman + Backbone 500 Hz"` (C6).

- **`stable_backbone_comparison.py`** (new) — runs the standard circular-trajectory + step-force benchmark shared with the paper's other experiments, comparing C1 (`Impedance`), C5 (`DI-MPC + Kalman 500 Hz`, current proposed), and C6 (new). Two experiments: a normal-condition benchmark, and an $F_{\max}$ stress sweep down to 0 N (zero additive corrective-force authority). This produces the same applied additive force as the solver's zero-output fallback, but it is not a runtime solver-fault injection.

Regression coverage is now preserved in `simulation/test_stable_backbone_mpc.py`: a deliberately infeasible impedance-force reference confirms that the joint-torque row clips the corresponding force at **every** one of the $N$ steps (while the first-step-only mode leaves later steps unconstrained), the dense/SLSQP and sparse/OSQP constraint matrices are checked for exact agreement, the scheduled LTV builder is checked against its frozen special case, and full one-step FR3 `control()` calls cover the default, impedance-reference, and backbone branches on both solvers.

## 4. Results

Both experiments use the paper's standard scenario: 3-D circular reference ($R=0.12$ m, 8 s period), classical 15 N step push at $t\in[3,6]$ s of each cycle, FR3 in MuJoCo, 1 kHz inner loop, 500 Hz QP.

### 4.1 Normal benchmark ($F_{\max} = 150$ N, no induced saturation)

| Controller | RMS contact (mm) | Peak deflection (mm) | Steady-state (mm) |
|---|---:|---:|---:|
| C1 Impedance (classical) | 40.90 | 51.18 | 45.037 |
| C5 DI-MPC + Kalman 500 Hz (current proposed) | 0.15 | 0.76 | 0.021 |
| **C6 + Impedance backbone (new)** | **0.15** | **0.75** | **0.021** |

**C6 matches C5 to measurement precision when the QP is never actually saturated** — adding the backbone costs nothing under normal operation, as expected (the additional term still has the full 150 N of authority to work with, and the backbone contributes a QP-independent restoring term the unconstrained QP would have produced anyway).

### 4.2 $F_{\max}$ stress sweep — the key comparison

| $F_{\max}$ (N) | C5 RMS$_c$ (mm) | C6 RMS$_c$ (mm) | C5 peak (mm) | C6 peak (mm) | C5 SS (mm) | C6 SS (mm) |
|---:|---:|---:|---:|---:|---:|---:|
| 150 | 0.15 | 0.15 | 0.76 | 0.75 | 0.021 | 0.021 |
| 20 | 0.15 | 0.15 | 0.75 | 0.75 | 0.021 | 0.021 |
| 5 | 306.70 | **22.42** | 396.57 | **28.68** | 358.135 | **24.368** |
| 1 | 352.40 | **37.48** | 450.88 | **46.93** | 391.585 | **41.168** |
| 0 (zero additive authority) | 360.14 | **41.13** | 456.04 | **50.76** | 407.206 | **45.492** |

At $F_{\max} \ge 20$ N there is enough headroom that both controllers are unaffected. Below that, **C5's error jumps by ~30-40× (0.15 → 300+ mm)** the moment the corrective force can no longer supply the needed authority — this is exactly the predicted failure mode: with $F_{\rm mpc}$ forced toward zero, the commanded controller approaches bare feedforward, i.e. no corrective stiffness against the 15 N push. **C6 degrades gracefully instead**, settling at 20-50 mm in the $F_{\max} = 0$ limit (backbone-only command at $K_{\rm bb} = 300$ N/m) — an 8-14× smaller error than C5 across the saturated range.

Verified over a longer 3-cycle run at $F_{\max} = 0$ (not just the 1-cycle snapshot above) that both controllers stay finite/periodic rather than drifting further: C5 settles around 400-475 mm RMS with peaks near 630 mm; C6 settles around 35-45 mm RMS with peaks near 200 mm (the 200 mm peak comes from the free-space circular-tracking transient, not the push — a 300 N/m backbone alone cannot track a fast 0.12 m circle at high bandwidth, which is expected and separate from the push-rejection result above).

Figure: `simulation/stable_backbone_comparison.png`. Raw numbers: `simulation/stable_backbone_comparison.json`.

### 4.3 Protocol-matched re-run (Table I's exact 3-cycle scenario)

The results above use the same 1-cycle (1 push event) scenario for speed. Table I in the paper actually runs **3 cycles / 3 push events over 24 s** and averages the metrics across all three — worth checking directly rather than assuming the 1-cycle snapshot generalizes. Re-ran both experiments at `n_cycles=3` (`python3 stable_backbone_comparison.py --n-cycles 3`, writes `stable_backbone_comparison_3cycle.{json,png}`, does not overwrite the 1-cycle files).

**Normal benchmark, 3 cycles — reproduces Table I's C1/C7 rows almost exactly** (small residual differences from the published table, e.g. 12.61 mm vs. 12.8 mm RMS total, are consistent with the exact-ZOH discretization fix already applied to `impedance_mpc.py` in this working tree; not a new discrepancy introduced by the backbone architecture):

| Controller | RMS total (mm) | RMS contact (mm) | Peak defl. (mm) | SS error (mm) |
|---|---:|---:|---:|---:|
| C1 Impedance | 35.56 | 41.07 | 51.77 | 44.815 |
| C5 DI-MPC+Kalman 500 Hz (paper's C7) | 12.61 | 0.15 | 0.77 | 0.022 |
| **C6 + Backbone** | **12.66** | **0.15** | **0.76** | **0.022** |

C6 is indistinguishable from C5 at this precision under the paper's actual benchmark protocol, not just the 1-cycle spot check — confirms §4.1's "costs nothing when unsaturated" claim was not a single-event artifact.

**$F_{\max}$ stress sweep, 3 cycles (3 independent push events averaged per condition, not one):**

| $F_{\max}$ (N) | C5 RMS$_c$ (mm) | C6 RMS$_c$ (mm) | C5 peak (mm) | C6 peak (mm) | C5 SS (mm) | C6 SS (mm) | C5/C6 ratio (RMS$_c$) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 150 | 0.15 | 0.15 | 0.77 | 0.76 | 0.022 | 0.022 | 1.0× |
| 20 | 0.15 | 0.15 | 0.76 | 0.76 | 0.021 | 0.021 | 1.0× |
| 5 | 317.55 | **22.54** | 412.27 | **29.82** | 353.589 | **24.065** | 14.1× |
| 1 | 413.96 | **37.71** | 552.48 | **47.92** | 389.267 | **40.962** | 11.0× |
| 0 | 424.38 | **41.35** | 560.83 | **51.60** | 407.029 | **45.276** | 10.3× |

Averaging over 3 independent push events per condition instead of 1 doesn't change the story: the degrade-gracefully gap (10-14×) matches the 1-cycle sweep's 8-14× within noise, and the crossover point (headroom below ~20 N) is identical. The 1-cycle numbers were not cherry-picked or an artifact of which push realization got sampled.

Figure: `simulation/stable_backbone_comparison_3cycle.png`. Raw numbers: `simulation/stable_backbone_comparison_3cycle.json`.

## 5. What this does and doesn't show

**Shown:** the two-term (QP-independent commanded backbone + bounded additive QP correction) architecture improves this deterministic corrective-authority stress test — no measurable cost when unsaturated and 8-14× smaller error when the additive correction is heavily curtailed or forced to zero.

**Not (yet) shown / open:**

- ~~Single seed, single scenario~~ — **addressed in §4.3**: re-ran both experiments at the paper's actual `n_cycles=3` protocol (3 independent push events averaged, matching Table I exactly), not just the 1-cycle snapshot. Note the correction to the original framing: the paper's push/sustained-force benchmarks (Table I/III) are themselves *deterministic* single-run-per-condition (no measurement noise, no RNG seed) — "paired multi-seed statistics" describes a *different* experiment (§VI-F, measurement-noise/model-mismatch robustness, Monte Carlo over 5 noise seeds in `cloud_verify/robustness_verification.py`). Matching Table I's actual protocol means 3-cycle averaging, not noise seeds; that is what §4.3 now provides, and it confirms the 1-cycle numbers generalize (14.1×/11.0×/10.3× vs. the original 8-14× estimate). A genuine noise-seed Monte Carlo for C6 specifically (does the backbone change noise sensitivity?) is still open and would need its own run through the `cloud_verify` harness rather than `phri.py`.
- The horizon-wide torque constraint (§2c) is a **frozen-Jacobian local approximation** — it does not track how $J_v$/$\tau_{\rm base}$ actually evolve along the horizon, only prevents obviously-unrealizable plans from the current configuration. A tighter (SQP/RTI, re-linearizing along the horizon) version was discussed but not implemented. See the Remark added to §2c for the precise necessary-not-sufficient scoping.
- The $F_{\max}$ sweep is not an actuator-limit or solver-timing experiment. The MuJoCo plant retains the physical FR3 joint limits, and no timeout/crash is injected. A separate fault-injection test is needed before making runtime fault-tolerance claims.
- No re-derivation of the paper's Theorem 1/2/3 statements for this architecture (e.g. does Theorem 1's impedance-equivalence limit still hold with a backbone in the loop — plausible since the unconstrained optimum still reduces the additive term to the correct residual, but not checked here).
- Not yet decided whether/how this folds into the manuscript: as a replacement for D7, as an added robustness ablation/appendix result, or left out of this submission entirely.

## 6. Open questions for next step

1. **Still open — editorial call, not made here.** Promote C6 to the paper's main proposed controller, add it as a new robustness ablation/appendix (Section VI), or keep it out of this submission? Recommendation given the §4.3 evidence: **add as a robustness ablation, not a replacement for D7.** C6 costs nothing when unsaturated (§4.1, §4.3) and only pays off when $F_{\max}$ is starved below about 20 N, a condition the paper doesn't currently claim to defend against. Framing it as "D7 plus an optional correction-authority-robust variant" is a strictly additive contribution, whereas replacing D7 would require re-deriving Theorem 1/2/3 for the backbone-in-the-loop case first (still unchecked, see §5). No change made to `double_integrator_phri_ieee.md` or `arXiv/phri_main.tex` — this stays a recommendation pending your decision.
2. **Resolved — see §4.3.** The paper's own push/sustained-force benchmarks are deterministic (no noise seed), so the right match is `n_cycles=3` protocol-averaging, not a noise Monte Carlo; done, and the 1-cycle numbers hold up (14.1×/11.0×/10.3× vs. the original 8-14× estimate). A true noise-seed sweep for C6 (via `cloud_verify`) is a separate, still-open piece of work if reviewers ask whether the backbone changes noise sensitivity.
3. **Resolved — see the Remark in §2c.** State it as "frozen-Jacobian horizon-wide extension of (9b)" with the necessary-not-sufficient caveat spelled out, not as an unqualified "(9c) extended to the whole horizon" — the latter overclaims exactness the frozen-Jacobian approximation doesn't have.

## 7. Horizon scheduling: investigated, not adopted

**The frozen-Jacobian formulation of §2c is retained.** A constant-velocity "coast" extrapolation was tried first as a horizon-scheduling alternative and removed again — it added a second, hard-to-justify approximation (why constant velocity? how large is the error? why is it acceptable?) on top of the linearization/sampled-data/discrete-MPC approximations the paper already carries, for no measurable benefit. What replaced it — reference scheduling along the trajectory generator's own $(q_d,\dot q_d,\ddot q_d)$, the theoretically cleaner choice — was implemented and tested rather than assumed, and turned out to be **not just unnecessary but measurably worse** in the one regime clean enough to judge (§7.2). That is a stronger reason to keep §2c than "no difference": scheduling at all carries a real downside here, not just unrealized upside.

### 7.1 Reference-scheduled horizon model

Along the prediction horizon, robot-dependent quantities are evaluated on the nominal reference trajectory $(q_{d,i},\dot q_{d,i},\ddot q_{d,i})$:

$$J_i = J_v(q_{d,i}), \qquad \Lambda_i = \big(J_i M(q_{d,i})^{-1} J_i^\top\big)^{-1}, \qquad \tau_{{\rm ff},i} = \tau_{\rm ff}(q_{d,i},\dot q_{d,i},\ddot q_{d,i})$$

precomputed before the QP is solved and held constant during optimization, so the horizon-wide torque constraint stays affine in the decision variable:

$$-\tau_{\max} \;\le\; \tau_{{\rm ff},i} + J_i^\top F_{i} \;\le\; \tau_{\max}, \qquad i=0,\dots,N-1$$

This controller has no joint-space $q_d(t)$ of its own to draw on, though — `circular_ref()` only gives a **Cartesian** reference $p_d(t)$, and redundancy is resolved online (the null-space centering term), not against a precomputed joint trajectory. `phri.precompute_joint_reference()` builds one via closed-loop resolved-rate IK sharing the controller's own redundancy-resolution objective, integrated once offline before the episode ($\dot q_d = J_v^+[\dot p_d+k_p(p_d-{\rm FK}(q_d))]+\bar N[-k_{\rm null}(q_d-q_{\rm null})]$, via the new position/Jacobian-only `FR3MuJoCoEnv.shadow_kinematics()`, ~0.004 ms/call). $\ddot x_{d,i}$ needs no such construction — it's already exact from `circular_ref()` at $t_k+i\,\Delta t_{\rm mpc}$.

The backbone's closed-loop prediction is scheduled consistently with this: $A_{{\rm cl},i}=A_d+B_{d,i}G_{\rm bb}$ (`_build_scheduled_closed_loop_horizon`, an LTV generalization of the frozen $A_{\rm cl}=A_d+B_dG_{\rm bb}$ used by default) rather than leaving the backbone's prediction matrix frozen while the torque map varies. Both are unit-verified: the LTV builder reduces exactly to the frozen one when unscheduled, and matches a manual forward simulation to floating-point precision ($9\times10^{-16}$) for a genuinely time-varying case.

*(Wording note, also fixed in §2b: "predicted through the backbone's own closed loop" overclaimed precision — the accurate statement is that the QP predicts the backbone dynamics **linearized along a nominal trajectory**, a single frozen point by default or the scheduled trajectory above, consistent with gain-scheduling terminology.)*

### 7.2 Why it isn't used: negligible when it would help, harmful when it matters most

At the paper's 500 Hz / 10-step ($N\Delta t_{\rm mpc}=20$ ms) horizon, frozen and reference-scheduled are indistinguishable under the normal 15 N push (both 0.151/0.758/0.0206 mm RMS/peak/SS) — over 20 ms the Jacobian, inertia, and gravity vector simply don't change enough to matter, so freezing them at $q_k$ costs nothing.

Sweeping `tau_max` down (`phri.TAU_MAX_SCALE`, `simulation/horizon_schedule_comparison.py`) to actually bind the constraint tells the more important story:

| `tau_max` scale | Frozen RMS$_c$ (mm) | Reference-scheduled RMS$_c$ (mm) | Frozen SS (mm) | Reference-scheduled SS (mm) |
|---:|---:|---:|---:|---:|
| 1.00 | 0.151 | 0.151 | 0.0206 | 0.0206 |
| **0.30** | 0.493 | **0.582** | 0.0215 | **0.567** |
| 0.15 | 441.96 | 956.68 | 335.5 | 965.5 |

At scale 0.30 — the constraint clearly binds, but the system hasn't otherwise collapsed, the cleanest test point — reference scheduling is **worse**: RMS up 18%, steady-state error up **26×**. The mechanism: $q_d(t)$ is the *undisturbed* reference, but the torque constraint binds hardest exactly when the human push has deflected the real arm away from it, so scheduling against the planned configuration evaluates the wrong local model exactly when it matters. (At scale 0.15 both formulations are deep in a separate failure mode — the QP-independent gravity/orientation/null-space part of $\tau_{\rm base}$ alone approaches the scaled `tau_max`, making the QP itself near-infeasible regardless of scheduling scheme; reference-scheduled is worse there too, but that regime isn't a clean test of either approach.)

An error-decay correction ($q_i\to q_{d,i}+\rho^i(q_k-q_{d,k})$, `ImpedanceMPCParams.schedule_rho`) mostly recovers the scale-0.30 gap at $\rho\approx0.9$ (0.507 vs. Frozen's 0.493 mm RMS) — unsurprising, since high $\rho$ mostly just tracks the actual state — but doesn't beat Frozen, and doesn't help in the deep-saturation regime either. It confirms the diagnosis rather than motivating adoption.

### 7.3 Conclusion

Freezing the robot-dependent quantities over the horizon introduces negligible error at this controller's 20 ms prediction horizon, verified experimentally rather than assumed — §2c's frozen-Jacobian formulation is kept as-is. Reference scheduling remains implemented (`ImpedanceMPCParams.horizon_torque_schedule`, `phri.precompute_joint_reference`) for a future revision that moves to a longer horizon or slower QP rate where 20 ms may stop being negligible, but is **not recommended** at the current operating point: it adds implementation complexity (an offline IK-based reference generator, shadow kinematics/dynamics queries, an untuned correction parameter) while measurably regressing exactly where the torque constraint matters most, unless corrected — and even corrected, it only ties the much simpler frozen version.
