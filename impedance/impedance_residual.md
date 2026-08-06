---
header-includes:
  - \usepackage{placeins}
---

# Two-Rate Energy-Authorized Residual MPC for Impedance-Causal Physical Interaction

## Abstract

Residual model predictive control (MPC) can improve disturbance rejection around a compliant robot, but its corrective wrench is an active interaction port. A passivity check performed only when the MPC updates does not authorize the wrench held between updates, and strict time-domain passivity control can remove most of the predictive correction. We study a rate-separated architecture in which a motion-to-wrench impedance remains the physical nominal controller, a 100 Hz MPC proposes only a translational residual wrench, and a 1 kHz projection jointly enforces a residual-energy budget and the complete joint-torque envelope. This is not presented as the first combination of impedance, MPC, and passivity: passive model-predictive impedance, predictive variable-impedance/admittance, and energy-tank controllers are established. The investigated contribution is the explicit residual-port authorization and its inter-update realization. A continuous-time composition proposition and a fast-sample energy-floor lemma state the certificate boundary. An equation-faithful translational generalization of the Hannaford--Ryu time-domain passivity observer/controller provides an external baseline. In a torque-controlled 7-DoF Franka FR3 MuJoCo benchmark, 20 matched trials randomize passive-wall properties, disturbance amplitude, and phase. The proposed method has zero tank-floor violations, zero nominal-torque infeasibilities, and zero QP failures. A manager-rate-only guard sharing the identical authorization rule but re-checked once per 10 ms manager period instead of every 1 ms fast tick tracks statistically indistinguishably (\(p=0.656\)) yet breaches the floor in 20/20 trials, directly substantiating that fast, not merely periodic, authorization is what the guarantee rests on. The proposed controller reduces residual-position RMS by 23.7% relative to passive impedance and by 35.4% relative to strict time-domain passivity control, while remaining 19.5% worse than energetically unguarded MPC. A force-separation study shows that 50% leakage of intentional force into the disturbance estimate increases intentional-axis fidelity error by 28.2%; adding estimator delay, colored noise, and a constant velocity-estimate bias leaves the safety certificates untouched but still degrades task fidelity, most from the velocity bias; separately, the wall's own reaction force is folded into the same disturbance estimate by construction, so the controller commands a corrective push into physical contact with zero energy-tank engagement, a classification failure the tank certificate cannot see. A manager-rate sweep (20/50/100 Hz, fixed 0.20 s lookahead) finds no universal best rate: the slower manager reduces correction chatter and wins under oscillatory wall-contact disturbance, while the faster manager wins at tracking a sustained intentional push; safety invariants hold at every rate tested. An anticipatory variant that forecasts the known-frequency disturbance through the horizon instead of holding it constant matches the frozen hold only when the disturbance is genuinely harmonic, and is measurably worse (1.6-1.75%) once any non-harmonic content -- a brief pulse, or contact -- enters the estimate, a gap that widens rather than closes with a longer horizon. The results support the two-rate authorization mechanism in full rigid-body simulation, but not hardware-level safety or universal superiority.

**Keywords:** impedance control, residual model predictive control, passivity, energy tank, multi-rate control, physical human--robot interaction

## 1. Introduction

Impedance control exposes a desired motion-to-wrench relation at the robot's physical interaction port [1]. Predictive control offers complementary benefits: disturbance preview, finite-horizon allocation, and explicit actuator constraints. The difficulty is energetic. An additive predictive wrench can inject energy even when the nominal impedance is passive, and a decision made at the slow optimizer rate need not remain admissible as velocity and actuator headroom evolve during the held interval.

The architecture contains three components:

1. a 1 kHz Cartesian impedance that produces the complete nominal joint torque;
2. a 100 Hz MPC that proposes an additive translational residual wrench; and
3. a 1 kHz realization layer that scales the held proposal against measured velocity, tank energy, and complete joint-torque headroom.

Section 2 walks through this loop once in plain language before any equations. Section 4 then builds the physical model the loop runs on: robot dynamics, the impedance law that is the actual physical nominal, the admittance law used only as an analytical reference, and the error dynamics the MPC predicts. Section 5 gives the MPC itself and the two ways its proposal is authorized before reaching the actuator.

The contributions are:

- a causal and energetic decomposition separating nominal impedance, intentional-response reference, and residual-wrench authorization;
- a continuous-time composition result and a discrete fast-sample invariant that include the realized, not merely requested, residual wrench;
- a torque-controlled 7-DoF FR3 benchmark with full mass matrix, Jacobian, gravity/Coriolis bias, orientation hold, null-space control, and per-joint torque limits;
- an equation-faithful Hannaford--Ryu passivity-observer/controller (PO/PC) baseline sharing the same nominal controller and raw MPC proposal; and
- a direct force-separation leakage study that measures degradation of the intentional interaction response.

The paper remains a simulation study. Its claim is a reusable realization interface and evidence for inter-update authorization, not a new passivity or MPC principle.

## 2. How the Two-Rate Loop Works

This section describes the loop once, in words, before any of it is formalized in Section 4 and Section 5.

**Fast loop, at the robot's own servo rate, 1 kHz.** The physical nominal controller is an impedance law: it looks at the current position and velocity error and computes a wrench directly, the way a stiff spring-damper would. This wrench is converted to joint torque and, added to it, the *residual* wrench the slow loop most recently published (converted to torque at the *current* configuration, not cached from when it was computed). The result is scaled if needed so it never asks for more torque or more stored energy than is available, and only then sent to the robot. This last step is the fast authorization; it runs every 1 kHz tick regardless of what the slow loop is doing.

**Slow loop, every manager period, 100 Hz -- ten times slower than the fast loop, not ten times a second.** The manager does not touch the impedance law. Instead it asks: if the impedance controller keeps doing what it's doing, and a human keeps pushing the way they currently appear to be pushing, will the resulting torque and stored energy stay legal over the manager's own look-ahead (Section 5.1 fixes this at 0.20 s, 20 manager steps)? It rolls the impedance law's own behavior forward, and if that rollout would leave the actuator envelope or the energy budget infeasible, it solves a small optimization for the smallest additive wrench correction that keeps the rollout legal. If nothing is about to go wrong, the correction is exactly zero and the impedance law runs unmodified.

**Why two rates, and why authorize twice.** The manager is slow because predicting several steps ahead and solving an optimization problem costs computation; running it at the full 1 kHz servo rate would be wasteful when the physical dynamics only drift meaningfully over tens of milliseconds. But a wrench correction computed once and held for several fast-loop ticks is a promise made about the future — by the time it is actually applied, the measured velocity or the remaining energy may have moved. The fast authorization is what keeps that promise honest between manager updates: it recomputes the torque conversion, the energy ledger, and the actuator margin every single tick, and scales the held correction down (never up) if reality has moved since the manager last checked. Sections 4 and 5 give the exact equations behind each of these three pieces; Section 6 shows what changes when the fast authorization is switched off.

## 3. Relation to Existing Work

The broad combination of impedance, MPC, and passivity is established. Cao, Cheng, and Li use a bottom variable-impedance controller and a top MPC that computes complementary torque under a stored-energy constraint; they prove passivity and feasibility and validate on a Franka Panda [2]. This is the closest architectural precedent and rules out novelty claims based only on stacking an impedance loop and predictive correction.

Predictive impedance adaptation is also mature. Haninger, Hegeler, and Peternel optimize trajectory and impedance using learned interaction models [3]. Xue *et al.* combine predictive variable impedance, environment estimation, robustness, and passive switching [4]. Shen *et al.* embed a passivity index in a predictive variable-impedance controller for a hydraulic manipulator [5]. Mahfouz *et al.* optimize admittance parameters with passivity constraints and validate with seven participants [6]. These methods optimize or schedule the rendered compliance; the present method fixes the physical nominal impedance and authorizes a separate residual wrench.

Energy supervision predates all of these predictive controllers. Hannaford and Ryu's time-domain PO/PC measures sampled port energy and adds exactly the damping required to eliminate generated energy [7]. Ferraguti, Secchi, and Fantuzzi use energy tanks to render time-varying stiffness passively [8], and related layered energy-tank architectures have been demonstrated in robotic surgery [9]. Guo *et al.* introduce ultimate passivity, switching between performance and conservative modes while retaining an ultimate energy bound [10].

A structurally similar pattern recurs outside the impedance/passivity literature: a slow supervisory layer checking or correcting a fast nominal controller's request before it reaches the plant is the shared architecture of reference and command governors [12], control-barrier-function safety filters [13], and predictive safety certification for learned controllers [14]. These typically supervise a generic nominal controller against state or actuator constraints; the present method instead supervises a specific residual-wrench port against an explicit energy budget, and its nominal controller is the physical impedance law itself rather than an arbitrary or learned policy. The two-rate split of Section 2 -- a fast, always-on realization layer re-checking a slower proposal every servo tick -- is this same slow-check/fast-guard pattern applied to residual-energy authorization rather than state or actuator constraints; Section 7.1's B4 result (a guard re-checked only at the slow rate) is, in this framing, a direct empirical test of why that fast/slow split is load-bearing in a predictive-safety-filter architecture generally, not only in this one.

These closest architectures differ along the same three axes: what quantity is optimized or supervised, what mechanism enforces energy safety, and how the method was validated. Hannaford and Ryu's PO/PC [7] supervises an arbitrary sampled port with zero-reference damping, validated on haptic hardware. Tank-based impedance methods [8], [9] instead supervise time-varying interaction behavior through stored energy, validated on robot prototypes. Passive model-predictive impedance [2] optimizes complementary torque under a predicted stored-energy constraint, validated on a Franka experiment. Predictive impedance/variable-impedance methods [3]--[5] optimize the trajectory and/or impedance itself under task-specific constraints or passivity, validated on robot experiments. Predictive admittance [6] optimizes admittance parameters under an embedded passivity constraint, validated with a Jaco-2 and seven participants. Ultimate passivity [10] instead switches controller mode to retain an ultimate energy bound, validated on impedance/admittance robots. This study optimizes an additive residual wrench through a 100 Hz proposal and a 1 kHz energy/torque projection, validated in 7-DoF rigid-body simulation.

The remaining question is therefore narrow: can a predictive residual wrench be given a finite, replenishable energy budget and realized at a faster rate without altering the nominal impedance or violating the complete torque interface?

## 4. Robot Dynamics and Control Architecture

This section builds the physical model underneath Section 2's informal walkthrough, in the order the loop actually needs it: the robot's own dynamics first (4.A), then the impedance law that is the real physical controller (4.B), then the admittance law that is *not* used physically but supplies the analytical reference the residual tracks (4.C), and finally the error-coordinate model the MPC of Section 5 actually predicts (4.D). The notation below collects every symbol introduced along the way.

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
\qquad F=F_h+d+F_e,
\tag{1}
\]

*in words:* joint torque and task-space force together produce joint acceleration through the usual rigid-body dynamics; \(h\) is the complete gravity/Coriolis bias, \(F_h\) is intentional human force, \(d\) is a rejectable force component, and \(F_e\) is the passive environment wrench. The translational operational inertia is

\[
\Lambda=(J_vM^{-1}J_v^\top)^{-1},
\tag{2}
\]

*in words:* \(\Lambda\) is how much task-space mass the robot presents at the end-effector once the whole arm's inertia is projected through the current Jacobian — it is what converts a task-space force into a task-space acceleration, and it changes with configuration. The MuJoCo implementation evaluates \(M,h,J_v\), and the rotational Jacobian \(J_\omega\) from the current nonlinear state at every 1 kHz sample.

### 4.B Impedance control: the physical nominal

Impedance control commands a wrench directly from the measured motion error — it is *error-to-force* causal. For fixed nominal position \(p_0\) and orientation \(R_0\), define \(e=p-p_0\), \(v=J_v\dot q\), and

\[
F_I=-K_Ie-D_Iv.
\tag{3}
\]

*in words:* the physical nominal controller behaves like a spring-damper bolted between the end-effector and the fixed nominal pose — the further away or the faster it moves, the harder it pulls back. This is the actual physical controller running at 1 kHz; nothing else in this paper commands the robot directly. The complete nominal torque is

\[
\tau_I=h+J_v^\top F_I+J_\omega^\top F_R+N_v^\top\tau_0,
\tag{4}
\]

where \(F_R\) holds orientation and the dynamically consistent translational null-space projector \(N_v\) contains posture regulation. The residual wrench proposed by the MPC (Section 5) is added through the same physical channel, never a separate one:

\[
\tau=\tau_I+J_v^\top F_r.
\tag{5}
\]

Ignoring the disclosed auxiliary-task leakage, the translational storage

\[
H=\tfrac12v^\top\Lambda v+\tfrac12e^\top K_Ie
\tag{6}
\]

*in words:* \(H\) is the impedance law's own energy-like quantity — kinetic energy in the task-space inertia plus potential energy in the virtual spring — and it has the familiar local power form

\[
\dot H\le F^\top v-v^\top D_Iv+F_r^\top v.
\tag{7}
\]

*in words:* storage grows from external work \(F^\top v\), shrinks from the impedance law's own damping \(-v^\top D_Iv\), and the last term, \(F_r^\top v\), is the power the residual wrench injects or removes — this is the port that Section 5.2's tank must authorize, because nothing about the impedance law itself limits it. Equation (7) is used for controller construction; the paper does not elevate the varying-\(\Lambda\), regularized, sampled implementation to an exact global storage identity.

### 4.C Admittance control and the analytical intentional reference

A different, and in this literature very common, way to structure a compliant controller is *admittance* control: instead of mapping error to force, integrate a force-to-motion law forward to get a reference trajectory, then have an inner loop track that trajectory. Concretely,

\[
M_I\ddot x_I+D_I\dot x_I+K_Ix_I=F_h,
\tag{8}
\]

*in words:* \(x_I\) is a virtual mass-spring-damper being pushed around by the measured human force \(F_h\) — it is the trajectory a compliant admittance-causal robot would follow, computed by integrating this ODE forward rather than commanding a wrench directly. Regulating \(p-x_I\) with an inner tracking loop, after canceling the robot's own dynamics, can expose a convenient double-integrator residual model. It is nevertheless *admittance-causal*: motion is the commanded quantity, force only drives the reference.

This paper does **not** use (8) as the physical controller — Section 4.B's impedance law is the actual commanded wrench, chosen specifically because it gives the clean physical-port power decomposition of Equation (7), which the energy-tank authorization of Section 5.2 depends on. Equation (8) is retained only as an **analytical intentional-response reference**: a stand-in for "how the interaction ought to feel," against which the impedance law's actual behavior is compared. This choice has a real cost: because the physical nominal is impedance-causal, the residual model that Section 5's MPC predicts against depends on the rendered stiffness, damping, and operational inertia (Section 4.D below), rather than being a gain-independent double integrator the way a literal admittance controller's residual would be. We do not claim gain-independent QP reuse.

### 4.D Error-based residual dynamics

Equation (8) is stated with a generic admittance mass \(M_I\) because that is how admittance control is usually introduced; this paper sets \(M_I\equiv\Lambda(q)\), the same configuration-dependent operational inertia already entering the physical impedance law of Section 4.B, rather than treating \(M_I\) as a free design parameter. This is a deliberate substitution, not a general property of (8): it is what lets the analytical reference and the actual impedance-driven error obey the same second-order structure below, so the two can be directly differenced into the residual double integrator (10) instead of carrying an independent \(M_I^{-1}\) term throughout. Written directly in task coordinates with this substitution,

\[
\ddot x_I=\Lambda^{-1}(F_h-D_I\dot x_I-K_Ix_I).
\tag{9}
\]

*in words:* this is Equation (8) with \(M_I=\Lambda(q)\) inverted through, so it can be integrated alongside the real robot state at every fast tick -- using the current, actual \(\Lambda(q)\) at each tick, since the reference is never commanded and so never needs to be frozen the way the MPC's own horizon model is in Section 5.1 -- without \(x_I\) ever being commanded to the robot. With \(z=e-x_I\) — how far the impedance law's actual error is from where the admittance reference thinks it should be — the frozen local residual model used at each manager update is

\[
\ddot z=\Lambda^{-1}(-K_Iz-D_I\dot z+F_r+\hat d).
\tag{10}
\]

*in words:* \(z\) drifts according to the same impedance stiffness/damping, plus whatever residual wrench the MPC adds, plus the estimated disturbance \(\hat d\) it is trying to reject. Unlike a literal admittance-reference construction, (10) explicitly depends on \(K_I,D_I\), and \(\Lambda(q)\) — the computational cost, flagged in 4.C, of retaining an impedance-causal physical nominal.

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

Every controller receives identical realization parameters for each seed. Main metrics are 3-D residual-position RMS/peak, minimum energy ledger, PO energy, authorization activity, peak fraction of the derated joint-torque envelope, nominal infeasibility, and QP failures. Paired statistics are computed over seeds. Solver timing covers only `OSQP.solve`, not model construction, sensing, or torque communication.

### 6.3 Force-separation leakage test

The main benchmark assumes exact force labels. The leakage test directly relaxes that assumption:

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

\FloatBarrier

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

**PO/PC's velocity-singular gain is real, but is not the dominant cause of its degraded RMS.** An instrumented six-seed check finds \(\gamma_\ell\) (equation (21)) strongly dependent on port speed: the Pearson correlation between \(1/\|v_\ell\|\) and \(\gamma_\ell\) is 0.94--0.98 across seeds, mean \(\gamma_\ell\) is 40--60\(\times\) larger when \(\|v_\ell\|<5\) mm/s than otherwise -- a regime occupied 38--53% of each trial -- and per-tick gains reach \(10^4\)-\(10^5\). Equation (21)'s velocity-squared denominator, inherited unmodified from [7], is a genuine structural property of the published formula, not a numerical curiosity. But a disclosed practical variant -- \(\gamma_\ell\) forced to zero below a 5 mm/s velocity dead zone (\(\texttt{tdpc\_regularized}\), not a replacement for the equation-faithful baseline reported elsewhere) -- only partly closes the gap: ten matched seeds give 30.05 \(\pm\) 1.04 mm for equation-faithful PO/PC and 29.16 \(\pm\) 1.11 mm regularized, a modest 3.0% recovery, while the proposed tank remains far ahead at 19.18 \(\pm\) 0.66 mm (regularized-vs-proposed paired difference +9.98 mm, 95% CI [9.36, 10.60], \(p=4.3\times10^{-11}\)). The dead zone also costs PO/PC's own certificate: its observer dips to \(-1.7\times10^{-3}\) J instead of remaining nonnegative to numerical precision, since deferring the correction across the dead zone is itself a small, disclosed passivity violation. So the singular gain is a real, measured property of the equation-faithful baseline, but PO/PC's frequent, appropriately-triggered zero-reference damping activation -- not primarily the gain's magnitude at low speed -- is what mainly separates it from the proposed controller's finite-energy alternative.

**The manager-rate guard directly tests the title's claim.** B4 shares B5's MPC proposal and authorization rule exactly, differing only in how often \(\alpha_E\) is recomputed: every fast tick for B5, once per manager tick and held stale across the following nine for B4. Its tracking is statistically indistinguishable from B5's (paired difference +0.008 mm, 95% CI [-0.029, 0.045], \(p=0.656\)) -- fast re-authorization costs nothing in tracking quality -- yet its tank floor is breached in every one of 20 trials (mean violation 0.0016 J, range 0.0010-0.0022 J), against zero violations in 20/20 for B5. A staleness of at most one manager period (10 ms) is therefore sufficient to break the floor guarantee at this benchmark's dynamics, even though the held \(\alpha_E\) was itself computed correctly at the instant it was set: the guarantee requires re-checking, not merely checking. Between-update authorization is not a redundant safety margin; it is what the guarantee actually rests on.

There is no soft margin between the two endpoints. Sweeping B4's own re-check period over \(\{1,2,5,10\}\) ms (ten matched seeds, same protocol) finds the floor already breached in **10 of 10 seeds** at 2 ms -- one fast tick of staleness beyond B5's own 1 ms -- with a small mean violation (5.9\(\times10^{-5}\) J) that grows toward B4's 10 ms value (7.4\(\times10^{-4}\) J at 5 ms, 1.6\(\times10^{-3}\) J at 10 ms) as the period lengthens, while residual RMS stays flat across the whole range (19.18-19.20 mm, no discernible trend). The guarantee does not degrade gracefully with a small amount of staleness and then fail past some tolerable threshold; it fails at the first opportunity, with a severity that scales with how stale the authorization is allowed to become, and costs nothing in tracking quality to avoid entirely.

The proposed controller reduces residual RMS by 23.7% relative to passive impedance (paired difference -6.024 mm, 95% CI [-6.500, -5.549], \(p=1.80\times10^{-16}\)). It reduces RMS by 35.4% relative to strict PO/PC (difference -10.605 mm, 95% CI [-10.985, -10.225], \(p=6.61\times10^{-23}\)). Unguarded MPC remains 19.5% better than the proposed method (proposed-minus-unguarded +3.166 mm, 95% CI [2.637, 3.695], \(p=1.25\times10^{-10}\)). Thus, finite energy storage recovers much of the performance removed by zero-reference PO/PC but does not eliminate the energetic cost of passivity-oriented authorization.

**This "middle ground" is a tradeoff curve, not a single operating point.** Sweeping the initial budget above the fixed floor, \(E_0-E_{\min}\in\{0,0.02,0.06,0.12,0.25\}\) J (ten matched seeds, main protocol; \(E_0=0.08\) J, budget \(0.06\) J, is the value used throughout this paper), moves the proposed controller continuously between PO/PC-like caution and unguarded-like aggression: residual RMS falls monotonically from \(21.49\pm2.24\) mm at zero budget to \(15.98\pm0.72\) mm at the largest tested budget (approaching, and at the largest budget slightly beating within sampling noise, unguarded MPC's own \(16.23\pm0.92\) mm from Table 1), while authorization-active fraction falls from 68.9% to exactly 0%. The wall-contact metrics tell a sharper story: max penetration rises from \(5.46\) to \(7.70\) mm and the into-wall impulse from \(0.86\) to \(1.71\) N s as budget grows from \(0\) to \(0.06\) J, then **both saturate exactly at the paper's own default budget** and do not move further even as RMS keeps improving out to the largest tested budget -- the tank's incidental effect of tempering the wall-misclassification failure above is exhausted before its effect on overall tracking is. A larger budget is therefore not a free improvement: it buys lower RMS at the cost of a more aggressive, unmodified push into a misclassified contact, a coupling this benchmark's single fixed \(E_0\) does not expose on its own.

Every accepted trial has zero nominal-torque infeasibility and zero QP failure. The largest torque ratio is below 0.889 for the proposed controller. Mean solver-core time is 0.159 ms; the mean per-run 95th percentile is 0.179 ms and the largest recorded solve is 0.399 ms. That figure is `OSQP.solve` alone, not the full manager-tick computation: a per-stage timing pass on a representative trial (state acquisition, Jacobian/inertia via `nominal_torque`, disturbance-estimate assembly, the complete `ResidualMPC3D.control` call including horizon-matrix construction, and the fast projection) measures the full predictive-proposal call -- rebuilding the horizon matrices from the current \(\Lambda^{-1}(q)\) every manager tick, not just the OSQP call -- at 2.40 ms mean (2.56 ms p95), against 237 \(\mu\)s for state acquisition, 31.6 \(\mu\)s for the Jacobian/inertia stage, and 16.1 \(\mu\)s for the fast projection, each of which runs every 1 ms tick rather than only every manager tick. Summed at a manager tick this is comfortably inside the 10 ms budget (about 2.7 ms of 10), and at a fast-only tick comfortably inside 1 ms (about 0.28 ms); the 100 Hz deadline is therefore met by the full computation, not only the solver core, on this development machine's software stack -- but this remains a software timing measurement in simulation, not an end-to-end certificate including sensing, communication, or real-time OS scheduling on the actual hardware interface.

### 7.2 Force-separation leakage

**Table 2. Intentional-force leakage, five matched noise seeds per level.**

| leakage \(\lambda\) | intentional-axis error RMS (mm) | realized/reference response ratio | minimum tank (J) |
|---:|---:|---:|---:|
| 0 | 11.759 ± 0.007 | 0.741 | 0.020 |
| 0.10 | 12.399 ± 0.007 | 0.724 | 0.020 |
| 0.25 | 13.383 ± 0.007 | 0.698 | 0.020 |
| 0.50 | 15.073 ± 0.006 | 0.655 | 0.020 |

At 50% leakage, fidelity error is 28.2% higher than at zero leakage and the mean response ratio falls by 8.6 percentage points. The tank and torque invariants still hold; the failure is semantic rather than numerical. The controller safely does the wrong thing because part of the intentional human input is mislabeled as a disturbance. This result makes force decomposition a first-order interface requirement rather than a footnote.

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

### 7.4 Anticipatory disturbance forecasting: a negative result

A separate check replaces the frozen zero-order hold on \(\hat d_k\) in equation (11) with a per-axis Kalman-filtered forecast of the benchmark's known disturbance frequencies through the full horizon. Across four conditions that vary how much of the disturbance is genuinely harmonic, the forecaster matches the frozen hold only when the content is purely harmonic (\(p=0.90\)); any non-harmonic content -- the pulse used throughout this paper, that pulse in isolation, or wall contact -- costs measurable tracking accuracy instead, and a longer horizon widens rather than closes that gap. Appendix A gives the full protocol, Table 6, and horizon-sweep results.

### 7.5 Interpretation

The benchmark supports four claims. First, finite-horizon residual correction can improve the realized intentional impedance response under disturbances. Second, strict zero-reference PO/PC is substantially more conservative than a finite tank that can spend initial and dissipated energy. Third, authorization must run at the fast rate specifically, not merely exist: the B4 manager-rate guard shares B5's MPC proposal and authorization rule exactly, tracks statistically indistinguishably from it, and still breaches the tank floor in every trial once \(\alpha_E\) is held stale for even one manager period -- periodic re-authorization is not a cheaper substitute for continuous inter-update authorization, it is a different, weaker guarantee. Fourth, the fast layer can preserve its explicit tank and torque interfaces even when the force classifier is wrong, so those certificates must not be confused with task or intent correctness: leakage causes under-response to genuine human input with the certificates intact, and, more severely, folding the wall's own reaction into the same disturbance estimate causes the controller to push into physical contact with zero tank engagement, since the failure is a classification error the energy and torque certificates were never designed to catch. A fifth, unplanned finding (Section 7.3) is that no single manager rate is best across scenario types -- the right rate depends on whether the disturbance is oscillatory or a sustained step, and the paper's own headline rate is a choice for one of those regimes, not a universal optimum. A sixth, also unplanned finding (Section 7.4, detailed in Appendix A) is that a harmonic disturbance forecaster only matches the frozen hold when the disturbance is genuinely harmonic, and loses to it -- more so with a longer horizon -- once any non-harmonic content is present; the one condition where it appeared to win reverses under the same check, so no configuration tested here supports forecasting as a robust improvement.

The comparison does not establish superiority over passive model-predictive impedance [2], predictive variable-impedance methods [3]--[5], or predictive admittance [6]. Those controllers optimize different quantities and several have physical experiments. The comparison in Section 3 is architectural, not a numerical ranking.

## 8. Scope and Limitations

Validation is nonlinear 7-DoF rigid-body simulation, not hardware, and contact is a virtual unilateral passive wall rather than measured material interaction; there are no human participants, contact-force safety thresholds, or claims of clinical/industrial readiness. Proposition 1 is an ideal continuous-time composition statement, and Lemma 1 certifies the implemented energy and torque interfaces at fast samples, not a global sampled-data passivity theorem for the entire robot, orientation task, null-space controller, estimator, and environment; similarly, the MPC horizon freezes the current Jacobian, operational inertia, and nominal torque, so the fast projection enforces realized input feasibility but recursive MPC feasibility and state constraints are not proved. The Hannaford--Ryu baseline is equation-faithful at the translational port but necessarily generalized from a scalar haptic interface and passed through the same FR3 torque projection, so it is not a reproduction on the original Excalibur device. The main force labels are exact; the leakage study quantifies one failure mode, and Section 7.2 additionally tests estimator delay, colored noise, and a constant velocity-estimate bias, individually and combined, at one representative leakage level -- but not as a learned human-intent estimator, and not swept jointly across every leakage level. The wall-classification check (Section 7.2) quantifies the resulting into-wall push under one intentional-force protocol and randomized wall stiffness/damping; it does not sweep contact geometry, approach angle, or a fix (e.g. excluding \(F_e\) from \(\hat d\) when contact is independently detected), which is left for future work rather than substituted into the equation-faithful estimator tested here. The anticipatory forecaster (Section 7.4, Appendix A) assumes the disturbance frequencies are known exactly -- a disclosed modeling choice, not a general online frequency-identification method -- and its horizon-length sweep uses a smaller, eight-seed sample rather than Appendix A's paired twenty-seed statistic. The manager-rate guard (B4, Section 7.1) tests one staleness duration -- one manager period, 10 ms, at this benchmark's dynamics -- not a sweep of held-interval lengths; it establishes that this duration already breaches the floor, not the maximum staleness the floor can tolerate.

## 9. Conclusion

This paper retains an impedance-causal physical nominal and assigns MPC only an additive residual-wrench port. The structure sacrifices gain-independent residual matrices but exposes the exact power that must be authorized. A 100 Hz predictive proposal is therefore paired with a 1 kHz projection that accounts for measured residual power and complete joint-torque headroom.

The torque-controlled FR3 study adds three pieces of evidence absent from the earlier low-order verification: an external, equation-faithful time-domain passivity baseline, a manager-rate-only guard that isolates the paper's own title claim, and a direct intent/disturbance leakage test. The finite tank occupies a useful middle ground between unguarded MPC and strict zero-reference PO/PC, while the leakage results, and a companion check finding the tank inactive throughout every contact window while the controller pushes into the wall, together show that energy correctness cannot substitute for correct interpretation of human input and environment alike. The manager-rate guard is the more direct result: sharing B5's authorization rule exactly but re-checked once per manager period instead of every fast tick, it tracks indistinguishably from B5 (\(p=0.656\)) yet breaches the tank floor in 20/20 trials -- inter-update authorization is not a redundant safety margin on top of periodic checking, it is what the guarantee actually rests on. A manager-rate sweep (Section 7.3) further shows that the 100 Hz design point used throughout is a choice suited to sustained-push tracking, not a universally optimal rate: an oscillatory-disturbance scenario is better served by a slower, less chatter-prone manager, and the two regimes disagree on which rate wins. An anticipatory-forecasting variant (Section 7.4, Appendix A) shows that horizon foresight is not a free improvement either: it matches the frozen hold only for genuinely harmonic content and loses to it, more so with a longer horizon, once any non-harmonic content is present; the one scenario where it looked beneficial reversed under the same check. A regime-adaptive scheme that switches predictor and horizon by detected disturbance content is a candidate follow-up, though nothing here shows it would beat, rather than merely match, the frozen hold. The next decisive validation is a torque-controlled FR3/Panda contact experiment with measured wrench, velocity noise, latency, and complete end-to-end timing.

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

[6] D. M. Mahfouz, P. Di Lillo, O. M. Shehata, E. I. Morgan, and F. Arrichiello, “Passivity-Constrained Model Predictive Variable Admittance Control for Safe and Adaptive Physical Human--Robot Interaction,” *IEEE Robotics and Automation Letters*, 11(4), 2026.[doi:10.1109/LRA.2026.3666354](https://doi.org/10.1109/LRA.2026.3666354).

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

A longer prediction horizon does not fix this. An eight-seed sweep (`horizon_sweep.py`, \(N=10\) to \(110\), i.e. 0.10-1.10 s, past a full period of the slowest sinusoid) shows the as-tested (pulse-present) gap widening from 0.3% at 0.10 s to 3.9% by 0.40 s before plateauing near 4%, while the pure-harmonic ablation grows far more slowly over the same range (0.4% to 1.0%) -- two compounding but separable effects: a small, genuine cost from optimizing a horizon-averaged objective even for a perfectly-known sinusoid, and a much larger cost from leaning harder on a forecast contaminated by non-harmonic content. The full-matched row's apparent advantage in Table 6 does not survive this check either: it reverses to an 8-13% disadvantage for \(N\ge40\), so it is a narrow artifact of the paper's own \(N=20\) design point rather than a real benefit of forecasting under contact. The added complexity -- a known-frequency assumption and three new filter hyperparameters the frozen hold needs none of -- is not offset by a compute saving either, since the predictor itself costs under 2% of per-tick solve time; it simply is not paid for by any condition tested.
