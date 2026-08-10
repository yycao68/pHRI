---
header-includes:
  - \usepackage{placeins}
---

# Two-Rate Energy-Authorized Residual MPC for Impedance-Causal Physical Interaction

## Abstract

Residual model predictive control (MPC) can reject disturbances around a compliant robot, but the corrective wrench also creates an active interaction port. Checking passivity only when the MPC updates leaves the wrench held between updates unchecked, whereas strict time-domain passivity control can suppress much of the predictive correction. We consider a two-rate architecture: a motion-to-wrench impedance serves as the nominal physical controller, a 100 Hz MPC proposes a translational residual wrench, and a 1 kHz projection enforces both a residual-energy budget and the full joint-torque envelope. The novelty is not the combination of impedance, MPC, and passivity; passive model-predictive impedance, predictive variable-impedance/admittance, and energy-tank control are already established. Instead, we focus on authorizing the residual port between MPC updates. A continuous-time composition proposition and a fast-sample energy-floor lemma support the design, which we compare with a translational generalization of the Hannaford--Ryu time-domain passivity observer/controller. In 20 matched MuJoCo trials with a torque-controlled 7-DoF Franka FR3, the proposed method produces no tank-floor violations, nominal-torque infeasibilities, or QP failures. It reduces residual-position RMS by 23.7% relative to passive impedance and by 35.4% relative to strict time-domain passivity control, although it remains 19.5% worse than energetically unguarded MPC. The importance of the fast projection appears in a manager-rate guard that uses the same authorization rule but checks it once every 10 ms rather than every 1 ms. Its tracking is statistically indistinguishable from the proposed controller (\(p=0.656\)), yet it breaches the tank floor in all 20 trials; fast-sample authorization produces no violations. A separate force-decomposition test exposes a different limitation. Because the wall reaction is included in the disturbance estimate, the controller pushes into contact without engaging the energy tank. The energy and torque checks therefore remain valid even though the force has been classified incorrectly.

**Keywords:** impedance control, residual model predictive control, passivity, energy tank, multi-rate control, physical human--robot interaction

## 1. Introduction

Impedance control exposes a desired motion-to-wrench relation at the robot's physical interaction port [1]. Predictive control offers complementary benefits: disturbance preview, finite-horizon allocation, and explicit actuator constraints. The difficulty is energetic. An additive predictive wrench can inject energy even when the nominal impedance is passive, and a decision made at the slow optimizer rate need not remain admissible as velocity and actuator headroom evolve during the held interval.

The architecture contains three components:

1. a 1 kHz Cartesian impedance that produces the complete nominal joint torque;
2. a 100 Hz MPC that proposes an additive translational residual wrench; and
3. a 1 kHz realization layer that scales the held proposal against measured velocity, tank energy, and complete joint-torque headroom.

Section 2 gives an informal overview of the loop. Sections 4 and 5 then develop the robot model, nominal impedance, analytical admittance reference, residual dynamics, MPC, and authorization layers.

The contributions are:

- a causal and energetic decomposition separating nominal impedance, intentional-response reference, and residual-wrench authorization;
- a continuous-time composition result and a discrete fast-sample invariant that include the realized, not merely requested, residual wrench;
- a torque-controlled 7-DoF FR3 benchmark with full mass matrix, Jacobian, gravity/Coriolis bias, orientation hold, null-space control, and per-joint torque limits;
- an equation-faithful Hannaford--Ryu passivity-observer/controller (PO/PC) baseline sharing the same nominal controller and raw MPC proposal; and
- a direct force-separation leakage study that measures degradation of the intentional interaction response.

Our aim is a reusable realization interface and evidence for inter-update authorization, not a new passivity or MPC principle.

## 2. How the Two-Rate Loop Works

The controller divides prediction and realization between a slow manager and the robot's fast servo loop.

**Fast loop (1 kHz).** The nominal impedance computes a wrench from the current position and velocity errors, much like a spring-damper. This wrench is converted to joint torque. The loop also converts the most recent *residual* wrench from the slow manager using the robot's current configuration. Before the residual is added, it is scaled to respect the instantaneous torque headroom and available tank energy. This authorization runs at every servo tick.

**Slow loop (100 Hz).** Every 10 ms, the manager predicts the impedance error over a 0.20 s horizon and chooses an additive residual wrench subject to the joint-torque envelope. It computes a tracking correction each period rather than waiting for a failure condition. The optimization does not include the tank budget; the fast loop enforces that budget when it realizes the proposed wrench.

**Why two rates?** Horizon prediction and optimization are too costly to repeat at every servo tick. However, velocity, tank energy, and actuator headroom can change while the proposed wrench is held. The fast layer therefore recomputes the torque conversion, energy ledger, and actuator margin at each tick, scaling the held correction down (never up) when necessary. Sections 4 and 5 formalize the two loops, and Section 6 describes the ablation without fast authorization.

## 3. Relation to Existing Work

The broad combination of impedance, MPC, and passivity is established. Cao, Cheng, and Li use a bottom variable-impedance controller and a top MPC that computes complementary torque under a stored-energy constraint; they prove passivity and feasibility and validate on a Franka Panda [2]. This is the closest architectural precedent and rules out novelty claims based only on stacking an impedance loop and predictive correction.

Predictive impedance adaptation is also mature. Haninger, Hegeler, and Peternel optimize trajectory and impedance using learned interaction models [3]. Xue *et al.* combine predictive variable impedance, environment estimation, robustness, and passive switching [4]. Shen *et al.* embed a passivity index in a predictive variable-impedance controller for a hydraulic manipulator [5]. Mahfouz *et al.* optimize admittance parameters with passivity constraints and validate with seven participants [6]. These methods optimize or schedule the rendered compliance; the present method fixes the physical nominal impedance and authorizes a separate residual wrench.

Energy supervision predates all of these predictive controllers. Hannaford and Ryu's time-domain PO/PC measures sampled port energy and adds exactly the damping required to eliminate generated energy [7]. Ferraguti, Secchi, and Fantuzzi use energy tanks to render time-varying stiffness passively [8], and related layered energy-tank architectures have been demonstrated in robotic surgery [9]. Guo *et al.* introduce ultimate passivity, switching between performance and conservative modes while retaining an ultimate energy bound [10].

A structurally similar pattern recurs outside the impedance/passivity literature: a slow supervisory layer checking or correcting a fast nominal controller's request before it reaches the plant is the shared architecture of reference and command governors [12], control-barrier-function safety filters [13], and predictive safety certification for learned controllers [14]. These typically supervise a generic nominal controller against state or actuator constraints; the present method instead supervises a specific residual-wrench port against an explicit energy budget, and its nominal controller is the physical impedance law itself rather than an arbitrary or learned policy. The two-rate split of Section 2 -- a fast, always-on realization layer re-checking a slower proposal every servo tick -- is this same slow-check/fast-guard pattern applied to residual-energy authorization rather than state or actuator constraints; Section 7.1's B4 result (a guard re-checked only at the slow rate) provides a concrete example, in this residual-energy setting, of why a safety-relevant slow-rate proposal may require revalidation at the faster realization rate.

These closest architectures differ along the same three axes: what quantity is optimized or supervised, what mechanism enforces energy safety, and how the method was validated. Hannaford and Ryu's PO/PC [7] supervises an arbitrary sampled port with zero-reference damping, validated on haptic hardware. Tank-based impedance methods [8], [9] instead supervise time-varying interaction behavior through stored energy, validated on robot prototypes. Passive model-predictive impedance [2] optimizes complementary torque under a predicted stored-energy constraint, validated on a Franka experiment. Predictive impedance/variable-impedance methods [3]--[5] optimize the trajectory and/or impedance itself under task-specific constraints or passivity, validated on robot experiments. Predictive admittance [6] optimizes admittance parameters under an embedded passivity constraint, validated with a Jaco-2 and seven participants. Ultimate passivity [10] instead switches controller mode to retain an ultimate energy bound, validated on impedance/admittance robots. This study optimizes an additive residual wrench through a 100 Hz proposal and a 1 kHz energy/torque projection, validated in 7-DoF rigid-body simulation.

This leaves a focused question: can a predictive residual wrench draw from a finite, replenishable energy budget and be realized at a faster rate without changing the nominal impedance or violating the torque interface?

## 4. Robot Dynamics and Control Architecture

We now define the robot dynamics, physical impedance controller, analytical admittance reference, and error model used by the MPC. The notation is collected below.

**Notation.**

- \(q,\dot q\) -- joint position, velocity (rad, rad/s)
- \(M(q),h(q,\dot q)\) -- joint mass matrix, gravity/Coriolis bias (kg·m², N·m)
- \(J_v(q),J_\omega(q)\) -- translational, rotational task Jacobian
- \(\tau\) -- commanded joint torque (N·m)
- \(F=F_h+d+F_e\) -- total task-space force: intentional, rejectable, environment (N)
- \(\Lambda\) -- translational operational (task-space) inertia (kg)
- \(p,v=J_v\dot q\) -- task position, velocity (m, m/s)
- \(e=p-p_0\) -- position error against the fixed nominal pose \(p_0\) (m)
- \(K_I,D_I\) -- rendered impedance stiffness, damping (N/m, N·s/m)
- \(F_I\) -- impedance-law wrench, the physical nominal (N)
- \(\tau_I\) -- complete nominal joint torque: impedance + orientation + posture (N·m)
- \(x_I\) -- admittance-law reference trajectory, analytical only, never commanded (m)
- \(z=e-x_I\) -- residual: how far the real error is from the admittance reference (m)
- \(F_r\) -- MPC's proposed residual wrench (N)
- \(H\) -- impedance storage, an energy-like Lyapunov function (J)
- \(E\) -- residual-energy tank ledger (J)

### 4.A Joint and task dynamics

For the 7-DoF arm,

\[
M(q)\ddot q+h(q,\dot q)=\tau+J_v(q)^\top F,
\qquad F=F_h+d+F_e.
\tag{1}
\]

Here, \(h\) is the complete gravity/Coriolis bias, \(F_h\) is the intentional human force, \(d\) is the rejectable force component, and \(F_e\) is the passive environment wrench. The translational operational inertia is

\[
\Lambda=(J_vM^{-1}J_v^\top)^{-1}.
\tag{2}
\]

The matrix \(\Lambda\) is the task-space mass presented at the end-effector and varies with configuration. The MuJoCo implementation evaluates \(M,h,J_v\), and the rotational Jacobian \(J_\omega\) from the current nonlinear state at every 1 kHz sample.

### 4.B Impedance control: the physical nominal

Impedance control commands a wrench directly from the measured motion error — it is *error-to-force* causal. For fixed nominal position \(p_0\) and orientation \(R_0\), define \(e=p-p_0\), \(v=J_v\dot q\), and

\[
F_I=-K_Ie-D_Iv.
\tag{3}
\]

This law acts as a virtual spring-damper between the end-effector and the fixed nominal pose. It is the physical controller executed at 1 kHz. The complete nominal torque is

\[
\tau_I=h+J_v^\top F_I+J_\omega^\top F_R+N_v^\top\tau_0,
\tag{4}
\]

where \(F_R\) holds orientation and the dynamically consistent translational null-space projector \(N_v\) contains posture regulation. The residual wrench proposed by the MPC (Section 5) is added through the same physical channel, never a separate one:

\[
\tau=\tau_I+J_v^\top F_r.
\tag{5}
\]

Ignoring the disclosed auxiliary-task leakage, define the translational storage as

\[
H=\tfrac12v^\top\Lambda v+\tfrac12e^\top K_Ie.
\tag{6}
\]

The storage \(H\) combines task-space kinetic energy with the potential energy of the virtual spring. Its local power balance is

\[
\dot H\le F^\top v-v^\top D_Iv+F_r^\top v.
\tag{7}
\]

External work \(F^\top v\) increases the storage, nominal damping \(-v^\top D_Iv\) dissipates it, and \(F_r^\top v\) is the power supplied or removed by the residual wrench. The tank in Section 5.2 authorizes this residual port. Equation (7) is exact under the constant-\(\Lambda\) approximation used in Proposition 1. When \(\Lambda\) varies with configuration, differentiating (6) adds the term \(\tfrac12v^\top\dot\Lambda v\). We therefore rely on Lemma 1's fast-sample ledger and torque invariant for the implemented nonlinear plant, rather than treating (7) as an exact storage identity.

### 4.C Admittance control and the analytical intentional reference

A different, and in this literature very common, way to structure a compliant controller is *admittance* control: instead of mapping error to force, integrate a force-to-motion law forward to get a reference trajectory, then have an inner loop track that trajectory. Concretely,

\[
M_I\ddot x_I+D_I\dot x_I+K_Ix_I=F_h.
\tag{8}
\]

The state \(x_I\) is the trajectory of a virtual mass-spring-damper driven by the measured human force \(F_h\). It is obtained by integrating (8), rather than by commanding a wrench. An inner loop that regulates \(p-x_I\) after compensating for the robot dynamics can yield a convenient double-integrator residual model. The construction remains *admittance-causal*: force drives a motion reference.

Equation (8) is not the physical controller. The impedance law in Section 4.B supplies the commanded wrench and the port-power decomposition in (7), while (8) serves only as an **analytical intentional-response reference**. Because the physical controller is impedance-causal, the residual model depends on the rendered stiffness, damping, and operational inertia. Unlike the residual model of a literal admittance controller, it is not a gain-independent double integrator, and the QP matrices cannot be reused independently of those gains.

### 4.D Error-based residual dynamics

Equation (8) introduces a generic admittance mass \(M_I\). Here we set \(M_I\equiv\Lambda(q)\), using the same configuration-dependent operational inertia as the physical impedance law rather than treating the admittance mass as a free parameter. This is a deliberate substitution, not a general property of (8): it makes the analytical reference and the measured impedance error follow the same second-order structure. Their difference then yields (10) without an additional \(M_I^{-1}\) term. In task coordinates,

\[
\ddot x_I=\Lambda^{-1}(F_h-D_I\dot x_I-K_Ix_I).
\tag{9}
\]

The analytical reference is integrated alongside the robot state using the current \(\Lambda(q)\) at every fast tick; \(x_I\) is never commanded to the robot. Defining \(z=e-x_I\) as the difference between the measured impedance error and the admittance reference gives the frozen local model used at each manager update:

\[
\ddot z=\Lambda^{-1}(-K_Iz-D_I\dot z+F_r+\hat d).
\tag{10}
\]

Thus, \(z\) evolves under the nominal stiffness and damping, the MPC residual wrench, and the estimated disturbance \(\hat d\). Its explicit dependence on \(K_I,D_I\), and \(\Lambda(q)\) is the computational cost of retaining an impedance-causal physical controller.

## 5. Predictive Proposal and Two Fast Authorizers

### 5.1 Residual MPC

At \(T_m=10\) ms, (10) is zero-order-hold discretized as

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

### 5.2 Proposed finite-energy authorization

Let the tank obey, away from its upper cap,

\[
\dot E=v^\top D_Iv-F_r^\top v,
\qquad E\ge E_{\min}>0.
\tag{14}
\]

**Proposition 1 (ideal composition).** If (7) holds, the realized residual wrench is the same \(F_r\) used in (14), and an authorization mechanism maintains \(E\ge E_{\min}\), then

\[
\dot H+\dot E\le F^\top v.
\tag{15}
\]

*Proof.* Add (7) and (14). Residual power and nominal damping cancel. Energy discarded at the tank cap adds only nonnegative dissipation. \(\square\)

Proposition 1 inherits (7)'s own scope exactly: (7) is a controller-construction inequality for constant \(\Lambda\), not an exact storage identity for the FR3's configuration-varying \(\Lambda(q)\) (Section 4.B). The tank floor \(E\ge E_{\min}\) that Lemma 1 certifies at fast samples is therefore an energy-accounting invariant on the realized ledger, not a standalone passivity certificate for the varying-inertia plant; Table 1's "minimum ledger" column should be read accordingly.

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

**Lemma 1 (fast-sample interface invariant).** If \(E_0\ge E_{\min}\) and the nominal torque satisfies \(|\tau_I|\le\rho\tau_{\max}\), then (16)--(19) ensure \(E_\ell\ge E_{\min}\) and \(|\tau_{I,\ell}+J_{v,\ell}^\top F_{r,\ell}| \le\rho\tau_{\max}\) at every fast sample.

*Proof.* Equation (16) constructs a feasible line segment from the feasible nominal torque. Further scaling by \(\alpha_E\in[0,1]\) remains on that segment. For nonpositive residual power, (19) cannot reduce the ledger. For positive power, (17) limits withdrawal to the available energy above \(E_{\min}\) plus the current damping contribution. \(\square\)

### 5.3 External Hannaford--Ryu PO/PC baseline

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

Equation (21)'s gain is \(\gamma_\ell\propto\|v_\ell\|^{-2}\), unmodified from the published formula: as the port velocity approaches zero, \(\gamma_\ell\) diverges structurally, independent of the size of the underlying passivity deficit \(W_{\ell+1}^{\rm pred}\). Section 7.1 reports how often, and how severely, this structure is exercised at this benchmark's operating velocities.

## 6. Benchmark Design

### 6.1 Plant, controllers, and signals

The plant is the torque-controlled Franka FR3 model from MuJoCo Menagerie, integrated at 1 kHz. Built-in position actuators are disabled; the benchmark applies joint torque directly. The five controllers are:

- **B1 Passive impedance:** equations (3)--(4), \(F_r=0\);
- **B2 Unguarded MPC:** equations (11)--(13), with fast torque scaling but no residual-energy restriction;
- **B3 Hannaford--Ryu PO/PC:** the same raw MPC plus (20)--(21);
- **B4 Manager-rate guard:** the same raw MPC and (17)'s energy scale \(\alpha_E\), but \(\alpha_E\) is computed once per manager tick and held fixed across the ten fast ticks it spans, rather than recomputed every fast tick as in B5; isolates whether authorization must run at the fast rate specifically, not merely whether some energy authorization exists;
- **B5 Two-rate tank:** the same raw MPC plus (16)--(19), recomputing \(\alpha_E\) every fast tick.

All controllers use \(K_I=180\) N/m, \(D_I=28\) N s/m, a 100 Hz manager, a 1 kHz torque loop, \(N=20\) (0.20 s), \(E_0=0.08\) J, \(E_{\min}=0.02\) J, and \(E_{\max}=0.30\) J. The intentional force contains an 8 N push along \(x\) and a -5 N push along \(z\). Rejectable force contains three sinusoids and a 12 N, 7 ms pulse beginning at 1.507 s, between manager ticks. A unilateral passive wall starts 35 mm from the nominal pose.

### 6.2 Matched randomized protocol

Twenty matched trials randomize:

- passive-wall stiffness: 500--1500 N/m;
- passive-wall damping: 8--20 N s/m;
- rejectable-force scale: 0.8--1.2; and
- the three sinusoidal phases.

Every controller receives identical realization parameters for each seed. Main metrics are 3-D residual-position RMS/peak, minimum energy ledger, PO energy, authorization activity, peak fraction of the derated joint-torque envelope, nominal infeasibility, and QP failures. Paired statistics are computed over seeds. Solver timing covers only `OSQP.solve` [11], not model construction, sensing, or torque communication.

### 6.3 Force-separation leakage test

The main benchmark assumes exact access to the force components, while the default rejection channel deliberately aggregates the environment reaction \(F_e\) into \(\hat d\) (Section 7.2 quantifies the consequence). The leakage test relaxes the exact-access assumption:

\[
\hat d=d+F_e+\lambda F_h+n_F,
\qquad \lambda\in\{0,0.1,0.25,0.5\}.
\tag{22}
\]

To isolate leakage from disturbance rejection, this test disables \(d\) and the wall, retains 0.05 N estimator noise, and runs five matched seeds per level. It reports RMS error along the intentional-force axis and the ratio between realized and reference mean displacement during the sustained push.

A second, satellite sweep at a fixed mid-range leakage (\(\lambda=0.25\)) adds three sensing imperfections the main sweep above holds at their simplest setting: a one-manager-tick (10 ms) estimator delay, AR(1) colored noise (\(\phi=0.9\), same stationary standard deviation as the white-noise baseline so only temporal correlation is varied), and a constant 5 mm/s velocity-estimate bias along the intentional-force axis, applied only to the state fed to the residual MPC (the torque loop itself still uses the true measured velocity). Each is toggled individually and then combined, five matched seeds per condition.

A third check isolates the opposite term of (22): with \(\lambda=0\) (its default), \(F_e\) itself -- the wall's own reaction force -- is folded into \(\hat d\) at full weight whenever contact occurs, so the disturbance-rejection objective structurally includes cancelling legitimate contact resistance, not only failing to reject genuine disturbance as the leakage test above shows. Twenty matched seeds (wall stiffness/damping randomized as in Section 6.2, periodic disturbance silenced so any wall contact is attributable to the intentional force alone) compare passive impedance against the proposed controller on wall penetration, the corrective impulse commanded into the wall during contact, and authorization activity during contact.

## 7. Results

![Torque-controlled FR3 response, energy audit, torque utilization, and force-separation leakage.](simulation/fr3_two_rate_results.png)

### 7.1 Matched FR3 benchmark

**Table 1. Twenty matched FR3 trials, mean ± sample standard deviation. Torque ratio is relative to the derated 28% continuous envelope.**

| Controller | residual RMS (mm) | residual peak (mm) | minimum ledger/PO (J) | authorization active | peak torque ratio |
|---|---:|---:|---:|---:|---:|
| Passive impedance (B1) | 25.42 ± 0.76 | 37.07 ± 1.44 | 0.0800 ± 0.0000 | 0% | 0.853 ± 0.002 |
| Unguarded MPC (B2) | **16.23 ± 0.92** | **28.42 ± 1.68** | -0.0424 ± 0.0137 | 0% | 0.875 ± 0.005 |
| Hannaford--Ryu PO/PC (B3) | 30.00 ± 0.92 | 43.62 ± 1.79 | \(-1.3\times10^{-19}\) PO | 83.36 ± 1.79% | 0.867 ± 0.006 |
| Manager-rate guard (B4) | 19.40 ± 0.58 | 31.01 ± 1.59 | 0.0184 ± 0.0004 | 25.34 ± 3.91% | 0.910 ± 0.017 |
| **Two-rate tank (B5)** | 19.39 ± 0.61 | 31.21 ± 1.80 | **0.0200 ± 0.0000** | 29.79 ± 4.46% | 0.865 ± 0.010 |

The unguarded controller's counterfactual common ledger crosses the 0.02 J floor in 20/20 trials. This does not mean a physical tank becomes negative; it measures energy the controller would withdraw without authorization. The PO/PC observer remains nonnegative to numerical precision in every trial, and the proposed tank never crosses its floor.

**The velocity singularity in PO/PC does not fully explain its larger RMS.** In an instrumented six-seed check, \(\gamma_\ell\) from (21) depends strongly on port speed. Across seeds, the correlation between \(1/\|v_\ell\|\) and \(\gamma_\ell\) is 0.94--0.98. The mean gain is 40--60\(\times\) larger below 5 mm/s than above it, and individual gains reach \(10^4\)-\(10^5\). The robot operates in this low-speed range for 38--53% of each trial. This behavior follows directly from the velocity-squared denominator in (21), which is inherited from [7]. A practical variant, \(\texttt{tdpc\_regularized}\), sets \(\gamma_\ell=0\) below 5 mm/s. Over ten matched seeds, it reduces RMS from 30.05 \(\pm\) 1.04 mm to 29.16 \(\pm\) 1.11 mm, a 3.0% improvement. The tank controller still reaches 19.18 \(\pm\) 0.66 mm (paired difference +9.98 mm, 95% CI [9.36, 10.60], \(p=4.3\times10^{-11}\)). Regularization also lets the PO observer fall to \(-1.7\times10^{-3}\) J because correction is deferred inside the dead zone. The singular gain is therefore measurable, but the main performance gap comes from the frequent activation of zero-reference damping rather than from the magnitude of the low-speed gain alone.

**The manager-rate guard isolates the effect of authorization frequency.** B4 and B5 use the same MPC proposal and authorization rule. B5 recomputes \(\alpha_E\) at every fast tick, whereas B4 computes it once per manager period and holds it for the next nine fast ticks. Their tracking is statistically indistinguishable (paired difference +0.008 mm, 95% CI [-0.029, 0.045], \(p=0.656\)). Nevertheless, B4 breaches the tank floor in all 20 trials, with a mean violation of 0.0016 J (range 0.0010--0.0022 J), while B5 has no violations. At these dynamics, up to 10 ms of staleness is enough to invalidate the floor guarantee even when \(\alpha_E\) was correct when computed. The result depends on checking the authorization between manager updates, not simply checking it periodically.

B4's peak torque ratio (0.910, against B5's 0.865) is a symptom of the same staleness, not a separate one. Torque feasibility (16) is indexed by the fast sample \(\ell\) and recomputed fresh every tick for every controller, including B4 -- it is never held stale -- so \(|\tau_{I,\ell}+J_{v,\ell}^\top F_{r,\ell}|\le\rho\tau_{\max}\) is enforced identically regardless of which controller is running, and B4's maximum observed ratio across all 20 trials is 0.941, still inside the envelope. What differs is only the energy scale (17): B4's stale \(\alpha_E\) shrinks the already torque-feasible candidate wrench less aggressively than B5's freshly recomputed one would at that same instant, so a larger fraction of the torque-feasible wrench is applied on average -- the same underspending of authorization that drains the tank below its floor also shows up as higher realized torque utilization. The torque certificate remains satisfied throughout; the energy-floor certificate does not -- both effects trace to the same stale energy scaling, one benign and one the paper's central failure mode.

The staleness sweep gives the same picture. With re-check periods of \(\{1,2,5,10\}\) ms over ten matched seeds, the tank floor is already breached in all seeds at 2 ms. The mean violation grows from 5.9\(\times10^{-5}\) J at 2 ms to 7.4\(\times10^{-4}\) J at 5 ms and 1.6\(\times10^{-3}\) J at 10 ms. Residual RMS remains nearly constant at 19.18--19.20 mm. Once \(\alpha_E\) is held beyond a single fast sample, Lemma 1 no longer applies; the experiments show violations at the shortest stale period tested, without a corresponding tracking benefit.

The proposed controller reduces residual RMS by 23.7% relative to passive impedance (paired difference -6.024 mm, 95% CI [-6.500, -5.549], \(p=1.80\times10^{-16}\)). It reduces RMS by 35.4% relative to strict PO/PC (difference -10.605 mm, 95% CI [-10.985, -10.225], \(p=6.61\times10^{-23}\)). Unguarded MPC remains 19.5% better than the proposed method (proposed-minus-unguarded +3.166 mm, 95% CI [2.637, 3.695], \(p=1.25\times10^{-10}\)). Thus, finite energy storage recovers much of the performance removed by zero-reference PO/PC but does not eliminate the energetic cost of passivity-oriented authorization.

**Table 1's 19.5%-worse-than-unguarded figure is one point on a tunable authorization-performance curve, not the method's ceiling.** \(E_0=0.08\) J is a deliberately conservative default, chosen to keep authorization visibly active throughout the reported trials rather than to show the controller's best achievable tracking. Sweeping the initial budget above the fixed floor, \(E_0-E_{\min}\in\{0,0.02,0.06,0.12,0.25\}\) J (ten matched seeds, main protocol), moves the proposed controller continuously from PO/PC-like caution to unguarded-like aggression while the tank floor holds at every tested budget: residual RMS falls monotonically from \(21.49\pm2.24\) mm at zero budget to \(15.98\pm0.72\) mm at the largest tested budget -- matching, and at the largest budget slightly beating within sampling noise, unguarded MPC's own \(16.23\pm0.92\) mm from Table 1, with authorization-active fraction falling from 68.9% to exactly 0%. The initial tank budget \(E_0-E_{\min}\) is an explicit scalar tuning parameter that continuously trades authorization activity against residual-tracking performance in the present architecture, while the tank floor holds at every tested value. This differs structurally from methods in which passivity or impedance adaptation is embedded directly in the predictive controller, such as passive model-predictive impedance [2] or ultimate passivity [10]. The wall-contact metrics tell a sharper story: max penetration rises from \(5.46\) to \(7.70\) mm and the into-wall impulse from \(0.86\) to \(1.71\) N s as budget grows from \(0\) to \(0.06\) J, then **both saturate exactly at the paper's own default budget** and do not move further even as RMS keeps improving out to the largest tested budget -- the tank's incidental effect of tempering the wall-misclassification failure above is exhausted before its effect on overall tracking is. A larger budget is therefore not a free improvement: it buys lower RMS at the cost of a more aggressive, unmodified push into a misclassified contact, a coupling this benchmark's single fixed \(E_0\) does not expose on its own.

Every accepted trial has zero nominal-torque infeasibility and zero QP failure. The largest torque ratio is below 0.889 for the proposed controller. Mean solver-core time is 0.157 ms; the mean per-run 95th percentile is 0.164 ms and the largest recorded solve is 0.276 ms. That figure is `OSQP.solve` alone, not the full manager-tick computation: a per-stage timing pass on a representative trial (state acquisition, Jacobian/inertia via `nominal_torque`, disturbance-estimate assembly, the complete `ResidualMPC3D.control` call including horizon-matrix construction, and the fast projection) measures the full predictive-proposal call -- rebuilding the horizon matrices from the current \(\Lambda^{-1}(q)\) every manager tick, not just the OSQP call -- at 2.40 ms mean (2.56 ms p95), against 237 \(\mu\)s for state acquisition, 31.6 \(\mu\)s for the Jacobian/inertia stage, and 16.1 \(\mu\)s for the fast projection, each of which runs every 1 ms tick rather than only every manager tick. Summed at a manager tick this is comfortably inside the 10 ms budget (about 2.7 ms of 10), and at a fast-only tick comfortably inside 1 ms (about 0.28 ms); the 100 Hz deadline is therefore met by the full computation, not only the solver core, on this development machine's software stack -- but this remains a software timing measurement in simulation, not an end-to-end certificate including sensing, communication, or real-time OS scheduling on the actual hardware interface.

### 7.2 Force-separation leakage

**Table 2. Intentional-force leakage, five matched noise seeds per level.**

| leakage \(\lambda\) | intentional-axis error RMS (mm) | realized/reference response ratio | minimum tank (J) |
|---:|---:|---:|---:|
| 0 | 11.759 ± 0.007 | 0.741 | 0.020 |
| 0.10 | 12.399 ± 0.007 | 0.724 | 0.020 |
| 0.25 | 13.383 ± 0.007 | 0.698 | 0.020 |
| 0.50 | 15.073 ± 0.006 | 0.655 | 0.020 |

At 50% leakage, fidelity error is 28.2% higher than at zero leakage, and the mean response ratio falls by 8.6 percentage points. The tank and torque invariants still hold because the problem is not numerical: part of the intentional human input has been labeled as a disturbance. Force decomposition is therefore a core interface requirement, not a secondary implementation detail.

**Table 3. Sensing-realism sweep at fixed \(\lambda=0.25\) leakage, five matched seeds per condition.**

| Condition | intentional-axis error RMS (mm) | response ratio | minimum tank (J) |
|---|---:|---:|---:|
| Baseline (\(\lambda=0.25\), no added realism) | 13.384 ± 0.008 | 0.698 | 0.020 |
| Delay only (10 ms) | 13.368 ± 0.008 | 0.699 | 0.020 |
| Colored noise only | 13.374 ± 0.039 | 0.699 | 0.020 |
| Velocity bias only (5 mm/s) | 13.730 ± 0.008 | 0.690 | 0.020 |
| All combined | 13.704 ± 0.039 | 0.690 | 0.020 |

The baseline row is this sweep's own \(\lambda=0.25\) draw (different seeds from, but statistically consistent with, Table 2's 13.383 mm). None of the three individually degrades fidelity error by more than 2.6% relative to baseline, and the tank floor and torque envelope hold in every condition -- the safety certificates are not sensitive to these particular sensing imperfections at these levels. The one-tick estimator delay and the colored-noise correlation structure leave the mean essentially unchanged; colored noise instead widens the across-seed standard deviation roughly fivefold (0.008 to 0.039 mm), because temporally correlated noise resists the horizon's implicit averaging. The velocity-estimate bias is the dominant of the three, degrading fidelity error by 2.6% and the response ratio by 0.9 percentage points, because it corrupts the state the residual MPC itself feeds back on, not merely the disturbance estimate. The combined condition tracks the velocity-bias-only condition closely rather than summing the three effects, mirroring the leakage sweep's own message: safety (tank, torque) survives these corruptions, but task fidelity degrades in proportion to which channel is corrupted, and no causal claim beyond the tabulated numbers is made for why the combined condition does not exceed the velocity-bias-only one.

**Wall reaction is misclassified as rejectable disturbance, and the tank certificate does not see it.** Under intentional force alone (periodic disturbance silenced), passive impedance never reaches the wall in any of 20 randomized-wall seeds; the proposed controller contacts it in every seed (mean penetration 7.24 \(\pm\) 0.60 mm) and, while in contact, commands a mean 1.87 \(\pm\) 0.27 N s corrective impulse *into* the wall -- consistent with (22) folding the wall's own resistive \(F_e\) into \(\hat d\) and the MPC then working to cancel it. Energy-tank authorization is inactive throughout every contact window across all 20 seeds: the failure is a force-classification error, not an energy-budget or torque-limit violation, so the certificates this paper otherwise reports zero violations for provide no protection against it. This is a distinct and more severe failure mode than the leakage result above -- leakage causes the controller to under-respond to genuine human input, whereas this causes it to actively work against a physical constraint -- and it is present, at this magnitude, throughout the main benchmark's own default protocol (Section 6.1), not only in this isolated check.

### 7.3 Manager-rate sensitivity: no universal winner

Sections 6-7 fix the manager at 100 Hz. To check whether that choice was load-bearing, the main matched benchmark, the leakage sweep, and the sensing-realism sweep were rerun at 20 Hz and 50 Hz as well, holding the horizon duration fixed at 0.20 s throughout -- only the re-check interval changes (50, 20, or 10 ms), not the manager's lookahead.

**Table 4. Manager-rate sweep, main matched benchmark, two-rate controller only (20 seeds).**

| Manager rate | RMS (mm) | Peak (mm) | Authorization active | vs. unguarded MPC | Solve max / period |
|---|---:|---:|---:|---:|---:|
| 20 Hz | 18.96 ± 0.53 | 30.71 | 17.2% | +5.1% | 0.140 ms / 50 ms |
| 50 Hz | 19.50 ± 0.70 | 31.58 | 27.4% | +16.3% | 0.169 ms / 20 ms |
| 100 Hz | 19.39 ± 0.61 | 31.21 | 29.8% | +19.5% | 0.285 ms / 10 ms |

In this oscillatory wall-contact-plus-disturbance scenario, 20 Hz gives the lowest RMS, the least-frequent intervention, and (in this run) the smallest computational burden relative to its own deadline; solve-time maxima are wall-clock and have been observed to vary run-to-run by 2-3\(\times\) at a fixed rate, so `rate_sweep.py`'s own output is the source of record rather than this table's specific figures. A representative trial's diagnostics explain the ranking, not just describe it: at 20 Hz the applied correction has a total variation of 12.9 N/s and 0.75 activation events per second, versus 34.6 N/s and 4.25 events per second at 100 Hz -- almost triple the chatter. More tellingly, at 20 Hz tracking is *better* while the tank is active than while idle (15.2 versus 18.3 mm RMS); at 100 Hz this reverses (20.6 versus 16.1 mm). A slower manager commits to fewer, larger, apparently more coherent corrections in this scenario; a faster one re-solves before the previous correction has had time to act, producing more frequent, less decisive adjustments -- consistent with a chatter mechanism, not a fluke of one metric, though the total-variation and activation-count diagnostics are correlational and do not rule out other rate-dependent factors (discretization error, decision-variable count, or estimator behavior differing with the re-solve rate) contributing alongside it.

**Table 5. Manager-rate sweep, force-separation leakage (5 seeds per level).**

| Manager rate | \(\lambda=0\) (mm) | \(\lambda=0.5\) (mm) | % degradation |
|---|---:|---:|---:|
| 20 Hz | 13.01 ± 0.03 | 17.21 ± 0.03 | 32.3% |
| 50 Hz | 12.06 ± 0.01 | 15.60 ± 0.01 | 29.3% |
| 100 Hz | 11.76 ± 0.01 | 15.07 ± 0.01 | 28.2% |

Under this different scenario -- a sustained intentional push with no wall or oscillatory disturbance -- the ranking **reverses**: 100 Hz gives the lowest error at every leakage level and 20 Hz the highest. Tracking a step-like sustained push is a settling-time problem rather than a disturbance-rejection problem, so a slower supervisory loop is plausibly sluggish to lock onto the correct steady correction -- the opposite failure mode from the oscillatory-disturbance case above. The sensing-realism sweep shows the same qualitative pattern (velocity bias dominant, colored noise widens variance, delay negligible) at all three rates, only shifted to each rate's own baseline; that shift is consistent with, not independent evidence for, the ranking in this table.

**No manager rate dominates in both regimes tested, and neither compromises safety.** Zero tank violations, zero QP failures, and zero nominal infeasibilities hold at every rate in every scenario reported in this paper. The 100 Hz design point used throughout Sections 6-7.2 is the better choice for the sustained-push/leakage regime this paper otherwise emphasizes, but Table 4 shows it is not the best choice for the oscillatory wall-contact scenario, where a slower manager measurably reduces both intervention frequency and tracking error. Matching manager rate to the anticipated disturbance's own time scale -- rather than assuming faster is always safer -- is a real design question this benchmark exposes rather than resolves.

This sweep varies the proposal rate: how often the MPC computes a new raw residual wrench. B5's fast energy and torque authorization remains fixed at 1 kHz in every row of Table 4 and Table 5, exactly as defined in Section 6.1 -- only the manager's own re-check interval for issuing a new proposal changes, never how often \(\alpha_E\) is recomputed against the fast-tick state. This is distinct from the B4/B5 experiment of Section 7.1, which holds the proposal rate fixed at 100 Hz throughout and instead varies whether that same fast-tick energy authorization is refreshed every tick or held stale. The proposal rate is therefore a performance-design parameter, tunable to the disturbance's own time scale as this section shows; the fast authorization rate is part of the interface guarantee Section 7.1 studies, and was not varied here.

### 7.4 Anticipatory disturbance forecasting: a negative result

A separate check replaces the frozen zero-order hold on \(\hat d_k\) in equation (11) with a per-axis Kalman-filtered forecast of the benchmark's known disturbance frequencies through the full horizon. Across four conditions that vary how much of the disturbance is genuinely harmonic, the forecaster matches the frozen hold only when the content is purely harmonic (\(p=0.90\)); any non-harmonic content -- the pulse used throughout this paper, that pulse in isolation, or wall contact -- costs measurable tracking accuracy instead, and a longer horizon widens rather than closes that gap. Appendix A gives the full protocol, Table 6, and horizon-sweep results.

### 7.5 Interpretation

Three findings stand out. First, the finite, replenishable tank retains much more of the MPC tracking benefit than zero-reference PO/PC. It can spend both its initial energy and energy recovered from nominal damping, making it less conservative under disturbance. Second, the residual authority must be refreshed at the fast realization rate. B4 tracks like B5 but breaches the tank floor whenever \(\alpha_E\) is held over a manager period, so slow periodic authorization provides a weaker guarantee. Third, correct energy and torque accounting does not ensure correct interaction behavior. If the force classifier is wrong, the fast layer can satisfy both interfaces while suppressing intentional input or pushing against a wall that has been mislabeled as a disturbance.

The secondary studies also reveal two practical limits. No proposal rate is best in every scenario: oscillatory disturbances and sustained steps favor different manager rates. Likewise, the harmonic forecaster matches the frozen hold only for genuinely harmonic inputs and becomes less accurate as non-harmonic content enters the signal, especially at longer horizons. None of the tested forecasting configurations gives a robust improvement.

The comparison does not establish superiority over passive model-predictive impedance [2], predictive variable-impedance methods [3]--[5], or predictive admittance [6]. Those controllers optimize different quantities and several have physical experiments. The comparison in Section 3 is architectural, not a numerical ranking.

## 8. Scope and Limitations

The validation uses a nonlinear 7-DoF rigid-body simulation rather than hardware. Contact occurs against a virtual unilateral wall, not a measured material surface, and the study includes neither human participants nor contact-force safety thresholds. We therefore make no claim of clinical or industrial readiness.

Proposition 1 is an ideal continuous-time composition result. Lemma 1 certifies the implemented energy and torque interfaces at fast samples, but it is not a global sampled-data passivity theorem for the complete robot, orientation task, null-space controller, estimator, and environment. The MPC horizon also freezes the current Jacobian, operational inertia, and nominal torque. The fast projection enforces feasibility of the realized input, but recursive MPC feasibility and state constraints are not proved. The Hannaford--Ryu baseline is faithful to the translational-port equation, yet it generalizes a scalar haptic interface and passes the result through the FR3 torque projection; it is not a reproduction on the original Excalibur device.

The main benchmark assumes exact access to the force components, although \(F_e\) is deliberately included in \(\hat d\). The leakage study examines corruption by \(F_h\), and the sensing study adds delay, colored noise, and velocity bias at one leakage level. It does not implement a learned human-intent estimator or sweep these effects jointly over every leakage value. The wall-classification test uses one intentional-force protocol and randomized wall stiffness and damping. Contact geometry, approach angle, and possible remedies such as removing \(F_e\) from \(\hat d\) after contact detection remain untested.

Finally, the anticipatory forecaster assumes that disturbance frequencies are known exactly; it is not an online frequency-identification method. Its horizon sweep uses eight seeds, compared with twenty for the paired study in Appendix A. The B4 staleness sweep covers held intervals of 1, 2, 5, and 10 ms at the simulated dynamics. It shows that the floor fails at the shortest tested interval beyond B5's 1 ms, but does not establish behavior for longer holds or other fast-to-slow rate ratios.

## 9. Conclusion

We retain an impedance-causal physical controller and use MPC only to propose an additive residual wrench. This choice gives up gain-independent residual matrices, but it exposes the residual-port power that must be authorized. The resulting architecture pairs a 100 Hz predictive proposal with a 1 kHz projection based on measured residual power and current joint-torque headroom.

The FR3 simulation extends the earlier low-order verification with a time-domain passivity baseline, a manager-rate guard, and an intent/disturbance leakage test. The finite tank lies between unguarded MPC and strict zero-reference PO/PC in both authority and tracking performance. However, the wall-contact experiment shows why a valid energy account is not enough: the tank remains inactive while the controller pushes against a wall whose reaction has been classified as a disturbance.

The manager-rate comparison provides the clearest evidence for the proposed interface. B4 uses the same rule as B5 and tracks it closely (\(p=0.656\)), but violates the tank floor in all 20 trials when authorization is refreshed only once per manager period. The additional sweeps qualify the 100 Hz design choice. It works well for sustained-push tracking, whereas the oscillatory case favors a slower manager. Harmonic forecasting is similarly conditional: it matches the frozen hold on harmonic signals but degrades when pulses or contact enter the disturbance. A possible next step is to adapt the predictor and horizon to the detected disturbance regime, although the present results do not establish that such switching would outperform the frozen hold. The more important validation is a torque-controlled FR3 or Panda contact experiment with measured wrench, velocity noise, latency, and end-to-end timing.

## Reproducibility

From this directory:

```bash
cd simulation

MPLCONFIGDIR=/tmp/phri_impedance_fr3_mpl \
XDG_CACHE_HOME=/tmp/phri_impedance_fr3_mpl \
python3 verify_fr3_two_rate_benchmark.py --seeds 20 --leakage-seeds 5 --realism-seeds 5

python3 rate_sweep.py --leakage-seeds 5 --realism-seeds 5

python3 anticipatory_disturbance_study.py --seeds 20

python3 horizon_sweep.py --seeds 8

python3 -m pytest -q \
  test_fr3_two_rate_benchmark.py \
  test_two_rate_passive_residual.py \
  test_residual_mpc.py \
  test_rate_sweep.py \
  test_harmonic_disturbance_predictor.py \
  test_horizon_sweep.py
```

All simulation, verification, and test scripts live in `simulation/`. The primary script writes `simulation/fr3_two_rate_results.json` and `simulation/fr3_two_rate_results.png`; its `wall_classification_check` (20 seeds by default, `--wall-check-seeds`) is the source for the wall-misclassification numbers in Section 7.2, and Table 1's manager-rate guard (B4) row runs as one of `run_benchmark`'s five controllers in the same output. The same script's `energy_budget_sweep` (10 seeds, `--energy-budget-seeds`), `po_pc_regularization_check` (10 seeds, `--po-pc-regularization-seeds`), and `authorization_period_sweep` (10 seeds, `--auth-period-seeds`) are the sources for, respectively, the energy-budget tradeoff, the PO/PC dead-zone comparison, and the staleness-threshold finding, all reported alongside the B4/B5 and PO/PC discussion in Section 7.1; the per-stage timing breakdown cited there is a further exploratory diagnostic, not a committed script. `rate_sweep.py` produces `simulation/rate_sweep_results.json`, the source for Table 4/5 and the manager-rate diagnostics of Section 7.3. `anticipatory_disturbance_study.py` produces `simulation/anticipatory_disturbance_results.json`, the source for all four rows of Table 6 (Appendix A). `horizon_sweep.py` produces `simulation/horizon_sweep_results.json`, the source for Appendix A's horizon-length numbers. The earlier 1-DoF benchmark remains available as a secondary algebra and inter-update audit. `state_of_art_search_log.md` records the literature-search scope and novelty decision.

## References

[1] N. Hogan, “Impedance Control: An Approach to Manipulation: Part I—Theory,” *Journal of Dynamic Systems, Measurement, and Control*, 107(1), 1--7, 1985. [doi:10.1115/1.3140702](https://doi.org/10.1115/1.3140702).

[2] R. Cao, L. Cheng, and H. Li, “Passive Model-Predictive Impedance Control for Safe Physical Human--Robot Interaction,” *IEEE Transactions on Cognitive and Developmental Systems*, 2023. [doi:10.1109/TCDS.2023.3275217](https://doi.org/10.1109/TCDS.2023.3275217).

[3] K. Haninger, C. Hegeler, and L. Peternel, “Model Predictive Impedance Control with Gaussian Processes for Human and Environment Interaction,” *Robotics and Autonomous Systems*, 165, 104431, 2023. [doi:10.1016/j.robot.2023.104431](https://doi.org/10.1016/j.robot.2023.104431).

[4] J. Xue, W. Liang, Y. Wu, and T. H. Lee, “Model Predictive Variable Impedance Control Towards Safe Robotic Interaction in Unknown Disturbance-Rich Environments,” *Robotics and Autonomous Systems*, 189, 104961, 2025. [doi:10.1016/j.robot.2025.104961](https://doi.org/10.1016/j.robot.2025.104961).

[5] J. Shen, L. Fang, K. Zhang, H. Zong, M. Cheng, R. Ding, J. Zhang, and B. Xu, “Passivity-Constrained Variable Impedance Control Based on Hierarchical Decoupling Controller for Hydraulic Manipulators,” *Mechatronics*, 109, 103340, 2025. [doi:10.1016/j.mechatronics.2025.103340](https://doi.org/10.1016/j.mechatronics.2025.103340).

[6] D. M. Mahfouz, P. Di Lillo, O. M. Shehata, E. I. Morgan, and F. Arrichiello, “Passivity-Constrained Model Predictive Variable Admittance Control for Safe and Adaptive Physical Human--Robot Interaction,” *IEEE Robotics and Automation Letters*, 11(4), 2026. [doi:10.1109/LRA.2026.3666354](https://doi.org/10.1109/LRA.2026.3666354).

[7] B. Hannaford and J. H. Ryu, “Time-Domain Passivity Control of Haptic Interfaces,” *IEEE Transactions on Robotics and Automation*, 18(1), 1--10, 2002. [doi:10.1109/70.988969](https://doi.org/10.1109/70.988969).

[8] F. Ferraguti, C. Secchi, and C. Fantuzzi, “A Tank-Based Approach to Impedance Control with Variable Stiffness,” in *IEEE ICRA*, pp. 4948--4953, 2013. [doi:10.1109/ICRA.2013.6631284](https://doi.org/10.1109/ICRA.2013.6631284).

[9] F. Ferraguti, N. Preda, A. Manurung, M. Bonfè, O. Lambercy, R. Gassert, R. Muradore, P. Fiorini, and C. Secchi, “An Energy Tank-Based Interactive Control Architecture for Autonomous and Teleoperated Robotic Surgery,” *IEEE Transactions on Robotics*, 31(5), 1073--1088, 2015. [doi:10.1109/TRO.2015.2455791](https://doi.org/10.1109/TRO.2015.2455791).

[10] X. Guo, Z. Liu, V. Crocher, Y. Tan, D. Oetomo, and A. H. A. Stienen, “Ultimate Passivity: Balancing Performance and Stability in Physical Human--Robot Interaction,” *IEEE Transactions on Robotics*, 41, 2050--2066, 2025. [doi:10.1109/TRO.2025.3546856](https://doi.org/10.1109/TRO.2025.3546856).

[11] B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd, “OSQP: An Operator Splitting Solver for Quadratic Programs,” *Mathematical Programming Computation*, 12, 637--672, 2020. [doi:10.1007/s12532-020-00179-2](https://doi.org/10.1007/s12532-020-00179-2).

[12] E. Garone, S. Di Cairano, and I. Kolmanovsky, “Reference and Command Governors for Systems with Constraints: A Survey on Theory and Applications,” *Automatica*, 75, 306--328, 2017. [doi:10.1016/j.automatica.2016.08.013](https://doi.org/10.1016/j.automatica.2016.08.013).

[13] A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, “Control Barrier Functions: Theory and Applications,” in *Proc. European Control Conference (ECC)*, 3420--3431, 2019. [doi:10.23919/ECC.2019.8796030](https://doi.org/10.23919/ECC.2019.8796030).

[14] K. P. Wabersich and M. N. Zeilinger, “Linear Model Predictive Safety Certification for Learning-Based Control,” in *Proc. IEEE Conference on Decision and Control (CDC)*, 2018. [doi:10.1109/CDC.2018.8619829](https://doi.org/10.1109/CDC.2018.8619829).

## Appendix A. Anticipatory Disturbance Forecasting (Supplementary)

Section 7.4 summarizes this check; full protocol and results follow.

The main benchmark's disturbance term (Section 6.1) contains three sinusoids of known, fixed frequency. An **anticipatory** variant replaces the frozen zero-order hold on \(\hat d_k\) in equation (11) with a per-axis Kalman-filtered forecast of that known-frequency model through the full horizon, tested against the frozen hold under a set of wall-disabled, twenty-seed conditions that vary how much of the disturbance is genuinely harmonic: a **pure-harmonic ablation** (the three sinusoids only, the paper's usual 12 N pulse suppressed), **periodic-only** (the scenario used throughout Sections 6-7, sinusoids plus that pulse), and **pulse-only** (sinusoids silenced, the pulse alone -- a negative control with no harmonic content at all). A fourth, satellite condition reruns the full matched benchmark (wall active, everything on).

**Table 6. Anticipatory (harmonic forecast) vs. two-rate (frozen hold), twenty seeds, paired by seed.**

| Condition | two-rate RMS (mm) | anticipatory RMS (mm) | paired difference (mm) | 95% CI | \(p\) |
|---|---:|---:|---:|---:|---:|
| Pure-harmonic ablation (pulse removed) | 20.234 ± 1.684 | 20.239 ± 1.625 | +0.005 | [-0.087, 0.098] | \(9.04\times10^{-1}\) |
| Periodic-only (as tested, Sections 6-7) | **20.409 ± 1.679** | 20.737 ± 1.628 | +0.328 | [0.220, 0.436] | \(4.24\times10^{-6}\) |
| Pulse-only\(^\dagger\) | **21.212 ± 0.026** | 21.583 ± 0.010 | +0.371 | [0.359, 0.383] | \(8.72\times10^{-24}\) |
| Full matched (satellite) | 19.133 ± 0.634 | **18.773 ± 0.457** | -0.360 | [-0.541, -0.179] | \(5.23\times10^{-4}\) |

\(^\dagger\)With the sinusoids silenced, the per-seed random phases no longer reach the trial; this condition adds the same 0.05 N estimator noise used by the leakage and sensing-realism sweeps (Section 6.3) as its sole source of seed-to-seed variation.

The forecaster is statistically indistinguishable from the frozen hold only on the pure-harmonic ablation (\(p=0.90\)); every departure from perfectly harmonic content costs measurable tracking accuracy -- the same pulse present throughout the rest of this paper (+1.6%), that pulse alone (+1.75%, with no harmonic content to fit at all), or wall contact (the full-matched row, discussed below). The effect is monotonic in non-harmonic content, not in disturbance complexity generally: the model is not simply less accurate on harder signals, it is actively misled whenever the signal is not the class it assumes.

A longer prediction horizon does not fix this. An eight-seed sweep (`horizon_sweep.py`, \(N=10\) to \(110\), i.e. 0.10-1.10 s, past a full period of the slowest sinusoid) shows the as-tested (pulse-present) gap widening from 0.3% at 0.10 s to 3.9% by 0.40 s before plateauing near 4%, while the pure-harmonic ablation grows far more slowly over the same range (0.4% to 1.0%) -- two compounding but separable effects: a small, genuine cost from optimizing a horizon-averaged objective even for a perfectly-known sinusoid, and a much larger cost from leaning harder on a forecast contaminated by non-harmonic content. The full-matched row's apparent advantage in Table 6 does not survive this check either: it reverses to an 8-13% disadvantage for \(N\ge40\), so it is a narrow artifact of the paper's own \(N=20\) design point rather than a real benefit of forecasting under contact. The added complexity -- a known-frequency assumption, three new filter hyperparameters, and the extra per-tick Kalman update/forecast computation, none of which the frozen hold needs -- is not paid for by any condition tested.
