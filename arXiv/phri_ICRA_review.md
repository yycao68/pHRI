# Detailed review of phri_combined.tex for the ICRA version

## Outcome

The journal manuscript is 16 pages and cannot be converted to an eight-page
conference paper by formatting alone. The new phri_ICRA.tex is a conference
rewrite, not a replacement for phri_combined.tex. Its compiled PDF is seven
pages including references.

## Corrections made in the ICRA version

1. **Estimator description.** The implementation propagates the covariance and
   recomputes the Kalman gain at every MPC update. It is therefore described as
   a recursive Kalman filter, not a frozen steady-state filter.

2. **Meaning of the estimated state.** The state is a force-equivalent
   aggregate disturbance containing contact, coupling, friction, and model
   error. It is not called human intent and is not presented as an identifiable
   human-force estimate.

3. **QP cost.** The implemented condensed gradient contains
   \(\bar R d_N\), corresponding to the centered penalty
   \(\|U+d_N\|_{\bar R}^2\). This term is necessary for
   \(F_{\mathrm{mpc}}=-\hat d\) to be cost-free at equilibrium.

4. **Torque-constraint scope.** The default implementation constrains the
   complete torque for the first applied move only. The paper no longer implies
   horizon-wide torque feasibility or recursive feasibility.

5. **Safety scope.** The reported boundary experiment uses the soft null-space
   barrier and workspace-reference correction. It does not exercise the final
   one-step CBF, so it is described as empirical avoidance rather than certified
   forward invariance.

6. **Passivity scope.** The optional tank accounts only for the translational
   predictive-force channel. It does not cover feedforward, orientation,
   null-space, or their coupled port power. The conference paper makes no
   full-port passivity or arbitrary-environment stability claim.

7. **Fair baseline interpretation.** The nominal 300 N/m impedance versus MPC
   comparison is explicitly identified as gain-confounded. The primary causal
   result is C4 versus C5 under identical MPC tuning. The response-matched
   impedance and ideal measured-force cancellation results are retained to show
   what is and is not unique to the proposed realization.

8. **Saturation accounting.** The fair comparison distinguishes raw command
   excess from applied torque and uses the logged \(10^{-3}\) Nm threshold. The
   MPC excesses below \(2\times10^{-5}\) Nm are treated as solver-tolerance
   effects, not physical saturation.

9. **Theoretical scope.** The zero-offset proposition is stated only for a
   frozen configuration, an unbiased constant-disturbance estimate, a stable
   closed loop, a feasible QP, and inactive steady-state constraints.
   Moving-reference and active-constraint results remain empirical.

10. **Correction-authority stress test.** The conference table uses the current
    three-cycle artifact. The optional C8 backbone is presented as graceful
    degradation under a tightened additive-force bound, not as an actuator-fault
    or boundedness guarantee.

## Evidence retained

- Primary estimator ablation: C4 steady-state error 2.766 mm versus C5
  0.0420 mm, a 65.8x reduction with identical horizon, weights, rate, and torque
  interface.
- Response-matched impedance: 2.594 mm steady-state error, 0.454% saturated
  samples, and 378.17 W peak positive joint power.
- C4/C5: 113.15/113.14 W peak positive joint power, 2400 successful QP solves,
  zero failures, and no thresholded saturation.
- Ideal measured-force cancellation: 1.390 mm total steady-state error. This
  prevents an incorrect claim that constant-load cancellation is unique to MPC.
- Time-varying ground-truth reconstruction: all five measured horizon errors
  remain below the reported finite-horizon rate bound in the simulator.
- Current C8 stress artifact: nominal performance matches C7, while reduced
  correction authority improves finite-duration error but does not preserve
  offset-free regulation.

## Remaining submission risks

1. **Simulation-only validation is the largest reviewer risk.** The paper should
   not claim deployability or human safety until the FR3 hardware path logs
   applied torque, estimator state, solver status, CBF slack, and tank energy.

2. **Novelty is primarily architectural and experimental.** MPC, disturbance
   augmentation, operational-space control, and affine input constraints are
   established ideas. The strongest framing is the compact auditable interface
   plus the controlled causal ablation, not a new general MPC theory.

3. **The predictive variable-impedance baseline is in-house.** It must remain
   labeled as a mechanism-level comparator rather than a reproduction of a
   published method.

4. **Anonymous author metadata is still a placeholder.** Replace it according
   to the ICRA submission stage and insert the official conference header or
   paper ID required by the current template.

5. **Confirm the exact page policy for the target year.** This version is seven
   pages including references, below the requested eight-page cap, but the
   official call may distinguish technical pages, references, and purchased
   extra pages.

## Verification performed

- latexmk completed with all citations and cross-references resolved.
- The final log contains no overfull boxes.
- All seven PDF pages were rendered and visually inspected.
- pytest -q simulation/test_stable_backbone_mpc.py: 15 tests passed.
- Numerical claims were checked against:
  - simulation/fair_offset_free_comparison.json
  - simulation/time_varying_ground_truth_results.json
  - simulation/force_sweep.json
  - simulation/stable_backbone_comparison_3cycle.json
