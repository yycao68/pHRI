---
header-includes:
  - \usepackage{placeins}
---

# Two-Rate Energy-Authorized Residual MPC for Impedance-Causal Physical Interaction

## Abstract

Residual model predictive control (MPC) can improve disturbance rejection around a compliant robot, but its corrective wrench is an active interaction port. A passivity check performed only when the MPC updates does not authorize the wrench held between updates, and strict time-domain passivity control can remove most of the predictive correction. We study a rate-separated architecture in which a motion-to-wrench impedance remains the physical nominal controller, a 50 Hz MPC proposes only a translational residual wrench, and a 1 kHz projection jointly enforces a residual-energy budget and the complete joint-torque envelope. This is not presented as the first combination of impedance, MPC, and passivity: passive model-predictive impedance, predictive variable-impedance/admittance, and energy-tank controllers are established. The investigated contribution is the explicit residual-port authorization and its inter-update realization. A continuous-time composition proposition and a fast-sample energy-floor lemma state the certificate boundary. An equation-faithful translational generalization of the Hannaford--Ryu time-domain passivity observer/controller provides an external baseline. In a torque-controlled 7-DoF Franka FR3 MuJoCo benchmark, 20 matched trials randomize passive-wall properties, disturbance amplitude, and phase. The proposed method has zero tank-floor violations, zero nominal-torque infeasibilities, and zero QP failures. It reduces residual-position RMS by 23.3% relative to passive impedance and by 34.5% relative to strict time-domain passivity control, while remaining 16.3% worse than energetically unguarded MPC. A force-separation study shows that 50% leakage of intentional force into the disturbance estimate increases intentional-axis fidelity error by 29.3%; adding estimator delay, colored noise, and a constant velocity-estimate bias leaves the safety certificates untouched but still degrades task fidelity, most from the velocity bias. The results support the two-rate authorization mechanism in full rigid-body simulation, but not hardware-level safety or universal superiority.

**Keywords:** impedance control, residual model predictive control, passivity, energy tank, multi-rate control, physical human--robot interaction

## 1. Introduction

Impedance control exposes a desired motion-to-wrench relation at the robot's physical interaction port [1]. Predictive control offers complementary benefits: disturbance preview, finite-horizon allocation, and explicit actuator constraints. The difficulty is energetic. An additive predictive wrench can inject energy even when the nominal impedance is passive, and a decision made at the slow optimizer rate need not remain admissible as velocity and actuator headroom evolve during the held interval.

The causal direction matters. Integrating

\[
M_I\ddot x_I+D_I\dot x_I+K_Ix_I=F_h
\tag{1}
\]

as a force-to-motion reference and regulating \(x-x_I\) can expose a convenient double-integrator residual after inverse-dynamics cancellation. It is nevertheless an admittance-causal construction. In this paper the physical nominal is instead an impedance-causal mapping from motion error to wrench. Equation (1) is retained only as an analytical intentional-response reference. This choice yields a clean physical-port power decomposition but makes the residual prediction matrices depend on the rendered stiffness, damping, and operational inertia. We do not claim gain-independent QP reuse.

The architecture contains three components:

1. a 1 kHz Cartesian impedance that produces the complete nominal joint torque;
2. a 50 Hz MPC that proposes an additive translational residual wrench; and
3. a 1 kHz realization layer that scales the held proposal against measured velocity, tank energy, and complete joint-torque headroom.

The contributions are:

- a causal and energetic decomposition separating nominal impedance, intentional-response reference, and residual-wrench authorization;
- a continuous-time composition result and a discrete fast-sample invariant that include the realized, not merely requested, residual wrench;
- a torque-controlled 7-DoF FR3 benchmark with full mass matrix, Jacobian, gravity/Coriolis bias, orientation hold, null-space control, and per-joint torque limits;
- an equation-faithful Hannaford--Ryu passivity-observer/controller (PO/PC) baseline sharing the same nominal controller and raw MPC proposal; and
- a direct force-separation leakage study that measures degradation of the intentional interaction response.

The paper remains a simulation study. Its claim is a reusable realization interface and evidence for inter-update authorization, not a new passivity or MPC principle.

## 2. Relation to Existing Work

The broad combination of impedance, MPC, and passivity is established. Cao, Cheng, and Li use a bottom variable-impedance controller and a top MPC that computes complementary torque under a stored-energy constraint; they prove passivity and feasibility and validate on a Franka Panda [2]. This is the closest architectural precedent and rules out novelty claims based only on stacking an impedance loop and predictive correction.

Predictive impedance adaptation is also mature. Haninger, Hegeler, and Peternel optimize trajectory and impedance using learned interaction models [3]. Xue *et al.* combine predictive variable impedance, environment estimation, robustness, and passive switching [4]. Shen *et al.* embed a passivity index in a predictive variable-impedance controller for a hydraulic manipulator [5]. Mahfouz *et al.* optimize admittance parameters with passivity constraints and validate with seven participants [6]. These methods optimize or schedule the rendered compliance; the present method fixes the physical nominal impedance and authorizes a separate residual wrench.

Energy supervision predates all of these predictive controllers. Hannaford and Ryu's time-domain PO/PC measures sampled port energy and adds exactly the damping required to eliminate generated energy [7]. Ferraguti, Secchi, and Fantuzzi use energy tanks to render time-varying stiffness passively [8], and related layered energy-tank architectures have been demonstrated in robotic surgery [9]. Guo *et al.* introduce ultimate passivity, switching between performance and conservative modes while retaining an ultimate energy bound [10].

**Table 1. Closest architectures and the remaining distinction.**

| Method | Optimized or supervised quantity | Energy mechanism | Validation |
|---|---|---|---|
| Hannaford--Ryu [7] | arbitrary sampled port | zero-reference PO/PC damping | haptic hardware |
| Tank-based impedance [8], [9] | time-varying interaction behavior | stored energy | robot prototypes |
| Passive MP impedance [2] | complementary torque | predicted stored-energy constraint | Franka experiment |
| Predictive impedance/VIC [3]--[5] | trajectory and/or impedance | task-specific constraints/passivity | robot experiments |
| Predictive admittance [6] | admittance parameters | embedded passivity constraint | Jaco-2, seven participants |
| Ultimate passivity [10] | controller mode | ultimate energy bound | impedance/admittance robots |
| **This study** | additive residual wrench | 50 Hz proposal, 1 kHz energy/torque projection | 7-DoF rigid-body simulation |

The remaining question is therefore narrow: can a predictive residual wrench be given a finite, replenishable energy budget and realized at a faster rate without altering the nominal impedance or violating the complete torque interface?

## 3. Robot and Causal Decomposition

### 3.1 Joint and task dynamics

For the 7-DoF arm,

\[
M(q)\ddot q+h(q,\dot q)=\tau+J_v(q)^\top F,
\qquad F=F_h+d+F_e,
\tag{2}
\]

where \(h\) is the complete gravity/Coriolis bias, \(F_h\) is intentional human force, \(d\) is a rejectable force component, and \(F_e\) is the passive environment wrench. The translational operational inertia is

\[
\Lambda=(J_vM^{-1}J_v^\top)^{-1}.
\tag{3}
\]

The MuJoCo implementation evaluates \(M,h,J_v\), and the rotational Jacobian \(J_\omega\) from the current nonlinear state at every 1 kHz sample.

### 3.2 Physical nominal impedance

For fixed nominal position \(p_0\) and orientation \(R_0\), define \(e=p-p_0\), \(v=J_v\dot q\), and

\[
F_I=-K_Ie-D_Iv.
\tag{4}
\]

The complete nominal torque is

\[
\tau_I=h+J_v^\top F_I+J_\omega^\top F_R+N_v^\top\tau_0,
\tag{5}
\]

where \(F_R\) holds orientation and the dynamically consistent translational null-space projector \(N_v\) contains posture regulation. The residual wrench is applied through the same physical channel,

\[
\tau=\tau_I+J_v^\top F_r.
\tag{6}
\]

Ignoring the disclosed auxiliary-task leakage, the translational storage

\[
H=\tfrac12v^\top\Lambda v+\tfrac12e^\top K_Ie
\tag{7}
\]

has the familiar local power form

\[
\dot H\le F^\top v-v^\top D_Iv+F_r^\top v.
\tag{8}
\]

The last term is the residual port that must be authorized. Equation (8) is used for controller construction; the paper does not elevate the varying-\(\Lambda\), regularized, sampled implementation to an exact global storage identity.

### 3.3 Analytical intentional reference and residual model

The reference state is driven only by \(F_h\):

\[
\ddot x_I=\Lambda^{-1}(F_h-D_I\dot x_I-K_Ix_I).
\tag{9}
\]

With \(z=e-x_I\), the frozen local residual model used at each manager update is

\[
\ddot z=\Lambda^{-1}(-K_Iz-D_I\dot z+F_r+\hat d).
\tag{10}
\]

Unlike the earlier admittance-reference construction, (10) explicitly depends on \(K_I,D_I\), and \(\Lambda(q)\). That is the computational cost of retaining an impedance-causal physical nominal.

## 4. Predictive Proposal and Two Fast Authorizers

### 4.1 Residual MPC

At \(T_m=20\) ms, (10) is zero-order-hold discretized as

\[
\xi_{k+1}=A_k\xi_k+B_k(F_{r,k}+\hat d_k),
\qquad \xi=[z^\top,\dot z^\top]^\top.
\tag{11}
\]

The horizon-\(N\) proposal solves

\[
\min_{F_{r,0:N-1}}
\frac12\sum_{i=1}^{N}\xi_i^\top Q\xi_i
+\frac12\sum_{i=0}^{N-1}F_{r,i}^\top RF_{r,i}
\tag{12}
\]

subject to a Cartesian wrench box and the current-model torque envelope

\[
|\tau_I+J_v^\top F_{r,i}|\le\rho\tau_{\max},
\qquad \rho=0.28.
\tag{13}
\]

The 28% envelope represents a deliberately derated continuous budget, chosen before the accepted run to keep the nominal torque feasible while preserving headroom pressure for the residual; it is not a manufacturer continuous-duty specification, and the absolute FR3 limits remain the MuJoCo safety backstop regardless. The benchmark rejects any trial in which \(\tau_I\) itself is infeasible. Because (13) freezes \(J_v\) and \(\tau_I\), only the fast layer certifies the realized nonlinear sample.

### 4.2 Proposed finite-energy authorization

Let the tank obey, away from its upper cap,

\[
\dot E=v^\top D_Iv-F_r^\top v,
\qquad E\ge E_{\min}>0.
\tag{14}
\]

**Proposition 1 (ideal composition).** If (8) holds, the realized residual wrench is the same \(F_r\) used in (14), and an authorization mechanism maintains \(E\ge E_{\min}\), then

\[
\dot H+\dot E\le F^\top v.
\tag{15}
\]

*Proof.* Add (8) and (14). Residual power and nominal damping cancel. Energy discarded at the tank cap adds only nonnegative dissipation. \(\square\)

At fast sample \(\ell\), the torque-feasible scale \(\alpha_{\tau,\ell}\) is the largest value in \([0,1]\) satisfying

\[
|\tau_{I,\ell}+J_{v,\ell}^\top
(\alpha_{\tau,\ell}F^{\rm raw}_{r,k})|
\le\rho\tau_{\max}.
\tag{16}
\]

Define \(\bar F_{r,\ell}=\alpha_{\tau,\ell}F^{\rm raw}_{r,k}\). The energy scale is

\[
\alpha_{E,\ell}=
\begin{cases}
1,&\bar F_{r,\ell}^\top v_\ell\le0,\\
\min\!\left(1,
\dfrac{E_\ell-E_{\min}+h v_\ell^\top D_Iv_\ell}
{h\bar F_{r,\ell}^\top v_\ell}\right),&\text{otherwise}.
\end{cases}
\tag{17}
\]

The applied wrench and ledger update are

\[
F_{r,\ell}=\alpha_{E,\ell}\bar F_{r,\ell},
\tag{18}
\]

\[
E_{\ell+1}=\min\{E_{\max},E_\ell+h(v_\ell^\top D_Iv_\ell-F_{r,\ell}^\top v_\ell)\}.
\tag{19}
\]

**Lemma 1 (fast-sample interface invariant).** If \(E_0\ge E_{\min}\) and the nominal torque satisfies \(|\tau_I|\le\rho\tau_{\max}\), then (16)--(19) ensure
\(E_\ell\ge E_{\min}\) and
\(|\tau_{I,\ell}+J_{v,\ell}^\top F_{r,\ell}|
\le\rho\tau_{\max}\) at every fast sample.

*Proof.* Equation (16) constructs a feasible line segment from the feasible nominal torque. Further scaling by \(\alpha_E\in[0,1]\) remains on that segment. For nonpositive residual power, (19) cannot reduce the ledger. For positive power, (17) limits withdrawal to the available energy above \(E_{\min}\) plus the current damping contribution. \(\square\)

### 4.3 External Hannaford--Ryu PO/PC baseline

The external baseline shares the same \(F_r^{\rm raw}\), nominal torque, torque scaling, estimator, and sample rate. Following the impedance-causal series PO/PC in [7], generalized from a scalar to the 3-D translational port, define

\[
W_{\ell+1}^{\rm pred}=W_\ell-h\bar F_{r,\ell}^\top v_\ell.
\tag{20}
\]

If this prediction is negative, the controller adds the minimum isotropic damping

\[
\gamma_\ell=
\frac{-W_{\ell+1}^{\rm pred}}{h\|v_\ell\|^2},
\qquad
F_{r,\ell}=\bar F_{r,\ell}-\gamma_\ell v_\ell;
\tag{21}
\]

otherwise \(\gamma_\ell=0\). A final torque-feasible scalar is applied and the observer is updated from the actual wrench. This is an equation-faithful translational generalization of [7], with two disclosed adaptations: the published scalar port is extended by \(\|v\|^2\), and all output is passed through the same FR3 torque interface. It has zero initial energy reference, whereas the proposed tank permits finite stored energy and harvesting of nominal damping. This is the intended scientific comparison.

## 5. Benchmark Design

### 5.1 Plant, controllers, and signals

The plant is the torque-controlled Franka FR3 model from MuJoCo Menagerie, integrated at 1 kHz. Built-in position actuators are disabled; the benchmark applies joint torque directly. The four controllers are:

- **B1 Passive impedance:** equations (4)--(5), \(F_r=0\);
- **B2 Unguarded MPC:** equations (11)--(13), with fast torque scaling but no residual-energy restriction;
- **B3 Hannaford--Ryu PO/PC:** the same raw MPC plus (20)--(21);
- **B4 Two-rate tank:** the same raw MPC plus (16)--(19).

All controllers use \(K_I=180\) N/m, \(D_I=28\) N s/m, a 50 Hz manager, a 1 kHz torque loop, \(N=10\) (0.20 s), \(E_0=0.08\) J, \(E_{\min}=0.02\) J, and \(E_{\max}=0.30\) J. The intentional force contains an 8 N push along \(x\) and a -5 N push along \(z\). Rejectable force contains three sinusoids and a 12 N, 7 ms pulse beginning at 1.507 s, between manager ticks. A unilateral passive wall starts 35 mm from the nominal pose.

### 5.2 Matched randomized protocol

Twenty matched trials randomize:

- passive-wall stiffness: 500--1500 N/m;
- passive-wall damping: 8--20 N s/m;
- rejectable-force scale: 0.8--1.2; and
- the three sinusoidal phases.

Every controller receives identical realization parameters for each seed. Main metrics are 3-D residual-position RMS/peak, minimum energy ledger, PO energy, authorization activity, peak fraction of the derated joint-torque envelope, nominal infeasibility, and QP failures. Paired statistics are computed over seeds. Solver timing covers only `OSQP.solve`, not model construction, sensing, or torque communication.

### 5.3 Force-separation leakage test

The main benchmark assumes exact force labels. The leakage test directly relaxes that assumption:

\[
\hat d=d+F_e+\lambda F_h+n_F,
\qquad \lambda\in\{0,0.1,0.25,0.5\}.
\tag{22}
\]

To isolate leakage from disturbance rejection, this test disables \(d\) and the wall, retains 0.05 N estimator noise, and runs five matched seeds per level. It reports RMS error along the intentional-force axis and the ratio between realized and reference mean displacement during the sustained push.

A second, satellite sweep at a fixed mid-range leakage (\(\lambda=0.25\)) adds three sensing imperfections the main sweep above holds at their simplest setting: a one-manager-tick (20 ms) estimator delay, AR(1) colored noise (\(\phi=0.9\), same stationary standard deviation as the white-noise baseline so only temporal correlation is varied), and a constant 5 mm/s velocity-estimate bias along the intentional-force axis, applied only to the state fed to the residual MPC (the torque loop itself still uses the true measured velocity). Each is toggled individually and then combined, five matched seeds per condition.

## 6. Results

![Torque-controlled FR3 response, energy audit, torque utilization, and force-separation leakage.](simulation/fr3_two_rate_results.png)

\FloatBarrier

### 6.1 Matched FR3 benchmark

**Table 2. Twenty matched FR3 trials, mean ± sample standard deviation. Torque ratio is relative to the derated 28% continuous envelope.**

| Controller | residual RMS (mm) | residual peak (mm) | minimum ledger/PO (J) | authorization active | peak torque ratio |
|---|---:|---:|---:|---:|---:|
| Passive impedance | 25.42 ± 0.76 | 37.07 ± 1.44 | 0.0800 ± 0.0000 | 0% | 0.853 ± 0.002 |
| Unguarded MPC | **16.78 ± 0.91** | **29.15 ± 1.65** | -0.0325 ± 0.0143 | 0% | 0.875 ± 0.005 |
| Hannaford--Ryu PO/PC | 29.77 ± 0.97 | 43.33 ± 1.90 | \(-1.3\times10^{-19}\) PO | 84.56 ± 1.53% | 0.867 ± 0.006 |
| **Two-rate tank** | 19.50 ± 0.70 | 31.58 ± 1.71 | **0.0200 ± 0.0000** | 27.38 ± 4.45% | 0.868 ± 0.011 |

The unguarded controller's counterfactual common ledger crosses the 0.02 J floor in 20/20 trials. This does not mean a physical tank becomes negative; it measures energy the controller would withdraw without authorization. The PO/PC observer remains nonnegative to numerical precision in every trial, and the proposed tank never crosses its floor.

The proposed controller reduces residual RMS by 23.3% relative to passive impedance (paired difference -5.914 mm, 95% CI [-6.430, -5.398], \(p=1.12\times10^{-15}\)). It reduces RMS by 34.5% relative to strict PO/PC (difference -10.267 mm, 95% CI [-10.696, -9.837], \(p=1.21\times10^{-21}\)). Unguarded MPC remains 16.3% better than the proposed method (proposed-minus-unguarded +2.727 mm, 95% CI [2.154, 3.300], \(p=5.59\times10^{-9}\)). Thus, finite energy storage recovers much of the performance removed by zero-reference PO/PC but does not eliminate the energetic cost of passivity-oriented authorization.

Every accepted trial has zero nominal-torque infeasibility and zero QP failure. The largest torque ratio is below 0.889 for the proposed controller. Mean solver-core time is 0.0810 ms; the mean per-run 95th percentile is 0.0897 ms and the largest recorded solve is 0.284 ms. The 50 Hz deadline is therefore met by the solver core, but this is not an end-to-end timing certificate.

### 6.2 Force-separation leakage

**Table 3. Intentional-force leakage, five matched noise seeds per level.**

| leakage \(\lambda\) | intentional-axis error RMS (mm) | realized/reference response ratio | minimum tank (J) |
|---:|---:|---:|---:|
| 0 | 12.061 ± 0.010 | 0.733 | 0.020 |
| 0.10 | 12.743 ± 0.010 | 0.715 | 0.020 |
| 0.25 | 13.793 ± 0.011 | 0.688 | 0.020 |
| 0.50 | 15.598 ± 0.012 | 0.642 | 0.020 |

At 50% leakage, fidelity error is 29.3% higher than at zero leakage and the mean response ratio falls by 9.10 percentage points. The tank and torque invariants still hold; the failure is semantic rather than numerical. The controller safely does the wrong thing because part of the intentional human input is mislabeled as a disturbance. This result makes force decomposition a first-order interface requirement rather than a footnote.

**Table 4. Sensing-realism sweep at fixed \(\lambda=0.25\) leakage, five matched seeds per condition.**

| Condition | intentional-axis error RMS (mm) | response ratio | minimum tank (J) |
|---|---:|---:|---:|
| Baseline (\(\lambda=0.25\), no added realism) | 13.796 ± 0.011 | 0.688 | 0.020 |
| Delay only (20 ms) | 13.755 ± 0.011 | 0.689 | 0.020 |
| Colored noise only | 13.796 ± 0.041 | 0.688 | 0.020 |
| Velocity bias only (5 mm/s) | 14.146 ± 0.011 | 0.679 | 0.020 |
| All combined | 14.107 ± 0.041 | 0.680 | 0.020 |

The baseline row is this sweep's own \(\lambda=0.25\) draw (different seeds from, but statistically consistent with, Table 3's 13.793 mm). None of the three individually degrades fidelity error by more than 2.5% relative to baseline, and the tank floor and torque envelope hold in every condition -- the safety certificates are not sensitive to these particular sensing imperfections at these levels. The one-tick estimator delay and the colored-noise correlation structure leave the mean essentially unchanged; colored noise instead widens the across-seed standard deviation roughly fourfold (0.011 to 0.041 mm), because temporally correlated noise resists the horizon's implicit averaging. The velocity-estimate bias is the dominant of the three, degrading fidelity error by 2.5% and the response ratio by 0.9 percentage points, because it corrupts the state the residual MPC itself feeds back on, not merely the disturbance estimate. The combined condition tracks the velocity-bias-only condition closely rather than summing the three effects, mirroring the leakage sweep's own message: safety (tank, torque) survives these corruptions, but task fidelity degrades in proportion to which channel is corrupted, and no causal claim beyond the tabulated numbers is made for why the combined condition does not exceed the velocity-bias-only one.

### 6.3 Interpretation

The benchmark supports three claims. First, finite-horizon residual correction can improve the realized intentional impedance response under disturbances. Second, strict zero-reference PO/PC is substantially more conservative than a finite tank that can spend initial and dissipated energy. Third, the fast layer can preserve its explicit tank and torque interfaces even when the force classifier is wrong, so those certificates must not be confused with task or intent correctness.

The comparison does not establish superiority over passive model-predictive impedance [2], predictive variable-impedance methods [3]--[5], or predictive admittance [6]. Those controllers optimize different quantities and several have physical experiments. Table 1 is a representation comparison, not a numerical ranking.

## 7. Scope and Limitations

Validation is nonlinear 7-DoF rigid-body simulation, not hardware, and contact is a virtual unilateral passive wall rather than measured material interaction; there are no human participants, contact-force safety thresholds, or claims of clinical/industrial readiness. Proposition 1 is an ideal continuous-time composition statement, and Lemma 1 certifies the implemented energy and torque interfaces at fast samples, not a global sampled-data passivity theorem for the entire robot, orientation task, null-space controller, estimator, and environment; similarly, the MPC horizon freezes the current Jacobian, operational inertia, and nominal torque, so the fast projection enforces realized input feasibility but recursive MPC feasibility and state constraints are not proved. The Hannaford--Ryu baseline is equation-faithful at the translational port but necessarily generalized from a scalar haptic interface and passed through the same FR3 torque projection, so it is not a reproduction on the original Excalibur device. The main force labels are exact; the leakage study quantifies one failure mode, and Section 6.2 additionally tests estimator delay, colored noise, and a constant velocity-estimate bias, individually and combined, at one representative leakage level -- but not as a learned human-intent estimator, and not swept jointly across every leakage level.

## 8. Conclusion

This paper retains an impedance-causal physical nominal and assigns MPC only an additive residual-wrench port. The structure sacrifices gain-independent residual matrices but exposes the exact power that must be authorized. A 50 Hz predictive proposal is therefore paired with a 1 kHz projection that accounts for measured residual power and complete joint-torque headroom.

The torque-controlled FR3 study adds two pieces of evidence absent from the earlier low-order verification: an external, equation-faithful time-domain passivity baseline and a direct intent/disturbance leakage test. The finite tank occupies a useful middle ground between unguarded MPC and strict zero-reference PO/PC, while the leakage results show that energy correctness cannot substitute for correct interpretation of human input. The next decisive validation is a torque-controlled FR3/Panda contact experiment with measured wrench, velocity noise, latency, and complete end-to-end timing.

## Reproducibility

From this directory:

```bash
cd simulation

MPLCONFIGDIR=/tmp/phri_impedance_fr3_mpl \
XDG_CACHE_HOME=/tmp/phri_impedance_fr3_mpl \
python3 verify_fr3_two_rate_benchmark.py --seeds 20 --leakage-seeds 5 --realism-seeds 5

python3 -m pytest -q \
  test_fr3_two_rate_benchmark.py \
  test_two_rate_passive_residual.py \
  test_residual_mpc.py
```

All simulation, verification, and test scripts live in `simulation/`. The primary script writes `simulation/fr3_two_rate_results.json` and `simulation/fr3_two_rate_results.png`. The earlier 1-DoF benchmark remains available as a secondary algebra and inter-update audit. `state_of_art_search_log.md` records the literature-search scope and novelty decision.

## References

[1] N. Hogan, “Impedance Control: An Approach to Manipulation: Part I—Theory,” *Journal of Dynamic Systems, Measurement, and Control*, 107(1), 1--7, 1985. [doi:10.1115/1.3140702](https://doi.org/10.1115/1.3140702).

[2] R. Cao, L. Cheng, and H. Li, “Passive Model-Predictive Impedance Control for Safe Physical Human--Robot Interaction,” *IEEE Transactions on Cognitive and Developmental Systems*, 2023. [doi:10.1109/TCDS.2023.3275217](https://doi.org/10.1109/TCDS.2023.3275217).

[3] K. Haninger, C. Hegeler, and L. Peternel, “Model Predictive Impedance Control with Gaussian Processes for Human and Environment Interaction,” *Robotics and Autonomous Systems*, 165, 104431, 2023. [doi:10.1016/j.robot.2023.104431](https://doi.org/10.1016/j.robot.2023.104431).

[4] J. Xue, W. Liang, Y. Wu, and T. H. Lee, “Model Predictive Variable Impedance Control Towards Safe Robotic Interaction in Unknown Disturbance-Rich Environments,” *Robotics and Autonomous Systems*, 189, 104961, 2025. [doi:10.1016/j.robot.2025.104961](https://doi.org/10.1016/j.robot.2025.104961).

[5] J. Shen, L. Fang, K. Zhang, H. Zong, M. Cheng, R. Ding, J. Zhang, and B. Xu, “Passivity-Constrained Variable Impedance Control Based on Hierarchical Decoupling Controller for Hydraulic Manipulators,” *Mechatronics*, 109, 103340, 2025. [doi:10.1016/j.mechatronics.2025.103340](https://doi.org/10.1016/j.mechatronics.2025.103340).

[6] D. M. Mahfouz, P. Di Lillo, O. M. Shehata, E. I. Morgan, and F. Arrichiello, “Passivity-Constrained Model Predictive Variable Admittance Control for Safe and Adaptive Physical Human--Robot Interaction,” *IEEE Robotics and Automation Letters*, 11(4), 2026.[doi:10.1109/LRA.2026.3666354](https://doi.org/10.1109/LRA.2026.3666354).

[7] B. Hannaford and J. H. Ryu, “Time-Domain Passivity Control of Haptic Interfaces,” *IEEE Transactions on Robotics and Automation*, 18(1), 1--10, 2002. [doi:10.1109/70.988969](https://doi.org/10.1109/70.988969).

[8] F. Ferraguti, C. Secchi, and C. Fantuzzi, “A Tank-Based Approach to Impedance Control with Variable Stiffness,” in *IEEE ICRA*, pp. 4948--4953, 2013. [doi:10.1109/ICRA.2013.6631284](https://doi.org/10.1109/ICRA.2013.6631284).

[9] F. Ferraguti, N. Preda, A. Manurung, M. Bonfè, O. Lambercy, R. Gassert, R. Muradore, P. Fiorini, and C. Secchi, “An Energy Tank-Based Interactive Control Architecture for Autonomous and Teleoperated Robotic Surgery,” *IEEE Transactions on Robotics*, 31(5), 1073--1088, 2015. [doi:10.1109/TRO.2015.2455791](https://doi.org/10.1109/TRO.2015.2455791).

[10] X. Guo, Z. Liu, V. Crocher, Y. Tan, D. Oetomo, and A. H. A. Stienen, “Ultimate Passivity: Balancing Performance and Stability in Physical Human--Robot Interaction,” *IEEE Transactions on Robotics*, 41, 2050--2066, 2025. [doi:10.1109/TRO.2025.3546856](https://doi.org/10.1109/TRO.2025.3546856).

[11] B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd, “OSQP: An Operator Splitting Solver for Quadratic Programs,” *Mathematical Programming Computation*, 12, 637--672, 2020. [doi:10.1007/s12532-020-00179-2](https://doi.org/10.1007/s12532-020-00179-2).
