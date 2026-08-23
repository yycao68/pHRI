# Toward Interaction Dynamics: A Predictive Framework for Safe Physical Human-Robot Interaction

**Yongyan Cao$^{1}$ and Jinshan Tang$^{2}$**  
*$^{1}$Voryx Robotics LLC, San Jose, CA 95136, USA — yongyancao@gmail.com*  
*$^{2}$George Mason University, Dept. of Health Administration and Policy, Fairfax, VA, USA — jtang25@gmu.edu*

---

## Abstract

Safe physical human-robot interaction (pHRI) requires tracking a commanded motion, yielding under human forces, respecting actuator and joint limits, and staying predictable under persistent contact. Classical impedance control trades accuracy for safety: a sustained force produces the steady-state bias $e_\infty=-K_d^{-1}F_h$. We propose a predictive framework that models the interaction error itself: an operational-space feedforward cancels gravity and Coriolis terms and normalizes the task inertia, leaving a linear interaction-dynamics backbone — a double-integrator reduction of the interaction error — whose state-transition matrix is configuration-independent. This converts nonlinear torque-controlled pHRI into a linear constrained-control problem: offset-free tracking, actuator feasibility, quadratic stabilizability over a certified polytope of configurations, and sampled-data joint-limit safety follow from a 30-variable convex QP at 100 Hz with a Kalman filter that rejects persistent forces without steady-state error. In MuJoCo simulation of a 7-DOF Franka FR3, it reduces steady-state error under a sustained 15 N force from 44.8 mm to below 0.05 mm, holds sub-millimeter tracking on four 3-D circles, and is robust to position-measurement noise and up to 30% inertial mismatch. Because the backbone reduces contact to a linear model with a fixed transition matrix, it may serve as a dynamics prior for model-based physical AI.

**Keywords:** interaction dynamics, impedance control, model predictive control, physical human-robot interaction, disturbance rejection, joint-limit safety, redundant manipulator

---

## I. Introduction

Robots are moving from structured, isolated workcells into continuous physical contact with people and unstructured environments—collaborative assembly, assistive and rehabilitation devices, surgical robots, and service robots all require the machine to stay safe and predictable while executing a precise task [17], [18], [19]. In these settings the quality of the physical interaction is not a secondary concern but a primary control objective.

Classical robot control is organized around *robot* dynamics: trajectory tracking, inverse dynamics, and whole-body control take the manipulator as the principal dynamical system and treat contact forces as external disturbances or constraints [13]. This is highly effective in free space but becomes restrictive once interaction dominates behavior, because the object the controller reasons about—the robot's configuration-space dynamics—is not the object the task cares about, namely the *interaction* at the contact port. This motivates a shift in modeling emphasis: rather than predict robot motion and react to interaction, we make the **interaction dynamics**—the closed-loop relation among commanded motion, contact force, actuator and joint limits, and safety filters at the contact port—the quantity that is explicitly modeled, predicted, and optimized (formalized in Definition 1). We use the term in a deliberately specific modeling sense—the augmented error-and-force state at the contact port taken as the object of prediction and constraint—not as a claim to a new physical phenomenon; it is closely related to the operational-space and error-dynamics formulations [13], [25] and differs from them in *what is treated as the modeled state* (the interaction error, not the configuration) rather than in the underlying mechanics. This paper develops one concrete, offset-free predictive realization of it.

Impedance and admittance control are the dominant pHRI paradigms [1], [2]: they prescribe a desired dynamic relation between motion and contact force and are valued for their simplicity and physical transparency [17], [18]. Two limitations persist. First, they are fundamentally *reactive*—the desired port behavior is enforced by feedback after the force acts, and a fixed stiffness fixes the accuracy–safety trade-off: under a sustained force the steady-state deflection is $e_\infty=-K_d^{-1}F_h$ (a 15 N push through 300 N/m gives 50 mm), so stiffening for accuracy raises contact force [19] while integral action removes the bias only within a narrow stable-gain budget and is prone to windup [22], [23]. Second, task tracking, interaction shaping, and constraints are typically designed in separate modules, precluding a single optimization.

Model predictive control (MPC) addresses the second gap by folding constraints, actuator limits, and future prediction into one optimization [3], [21], and offset-free MPC removes steady-state error by augmenting an integrating disturbance state estimated by a Kalman filter [4]. Yet most impedance-MPC formulations either optimize impedance *parameters*—which enter the prediction nonlinearly and cap update rates at 10–30 Hz [5], [6], [14]—or compensate the estimated disturbance only reactively at the current step [7]. Variable impedance methods [8], [20] adapt the apparent stiffness but likewise give no offset-free guarantee under persistent unknown force. What is missing is a formulation in which the interaction error itself is the linear dynamical object exposed to standard constrained-control machinery.

Our hypothesis is that safe interaction is best posed on a predictive representation of the interaction error, in which impedance behavior is not a separate controller but an emergent property of the optimized dynamics. Computed-torque and operational-space linearization to a double integrator are classical [13], [25]; the contribution here is to make that reduction the **backbone** of the predictive interaction problem. An operational-space feedforward cancels gravity and Coriolis terms and normalizes the task through the operational-space inertia, leaving a residual task-error double integrator whose discrete state-transition matrix $A_d$ is *constant*, with all robot configuration dependence isolated in the input matrix $B_d(\rho_k)$—the scheduling-dependent structure familiar from LPV predictive control [24].

This paper makes four contributions:

1. **A predictive interaction-dynamics formulation.** We recast torque-controlled pHRI so that the modeled and optimized object is the interaction error at the contact port rather than the robot configuration dynamics, unifying compliance, tracking, disturbance rejection, and safety in one convex program while preserving the measured robot dynamics through $M$, $C\dot q+G$, and $J_v$.

2. **A configuration-independent state-transition matrix.** Operational-space feedforward cancellation makes the discrete transition matrix $A_d$ *constant* across all configurations, confining the entire robot dependence to the input (control-effectiveness) matrix $B_d(\rho_k)$—the classical LPV structure in which $A$ is fixed and only $B$ is scheduled. This lets the free-response matrix $\Phi$ be precomputed once, keeps the online problem a fixed 30-variable QP at 100 Hz at every configuration, and reduces the LPV backbone's closed-loop stability to a finite vertex LMI certifying quadratic stabilizability, by one fixed feedback gain, over a polytope of configurations (Theorem 3) rather than only at a frozen configuration — a statement about the backbone under that certified gain, not directly about the deployed receding-horizon MPC.

3. **Offset-free predictive impedance.** We prove (Theorem 1) that classical task-space impedance [1] is a *special case* of this framework — in the unconstrained, disturbance-free limit the predictive feedback reduces to it for the passive symmetric component of the realized gain — and (Theorem 2) that an input-centered augmented-Kalman disturbance state gives zero steady-state error under constant bounded human force when the stated stability and feasibility conditions hold, removing the fixed accuracy–safety trade-off of impedance control.

4. **Safety and passivity integration.** A null-space inverse-barrier potential [15], a one-step joint-limit control-barrier filter on the final applied torque, and an energy-tank scaling of the predictive task force provide practical joint-limit regulation and sampled task-channel passivity, *conditional* on feasibility and per-sample enforcement.

On a 7-DOF Franka FR3 in MuJoCo simulation, the controller reduces steady-state error under a sustained 15 N force from 44.8 mm (classical impedance) to below 0.05 mm—a >800× reduction—while holding sub-millimeter tracking on four 3-D circles and remaining robust to position-measurement noise and up to 30% inertial mismatch. Although developed for pHRI, the interaction-dynamics backbone requires only $M$, $C\dot q+G$, and $J_v$, offering a path toward interaction models that transfer across configurations for manipulation and whole-body control; a full realization of that vision is left to future work.

The same analytic reduction points toward an interaction-dynamics prior for *model-based physical AI*; we return to this in the conclusion rather than claim it here.

---

## II. System Dynamics and Problem Formulation

Consider an $n$-DOF torque-controlled serial manipulator. The framework applies to rigid-body torque-controlled serial manipulators satisfying the standard operational-space assumptions (invertible mass matrix, full-rank task Jacobian away from singularities); experiments use the Franka FR3 ($n = 7$) [16] as the validation platform. The rigid-body equations of motion are:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau + J^\top(q)\mathcal{F}_h \tag{1}$$

where $q \in \mathbb{R}^n$, $M(q)$ is the symmetric positive-definite inertia matrix, $C(q,\dot{q})\dot{q} + G(q)$ is the Coriolis-plus-gravity vector (available as `qfrc_bias` in MuJoCo or via the robot model API), $\tau$ is the commanded joint torque, and $\mathcal{F}_h \in \mathbb{R}^6$ is the human-applied wrench. $J(q) = [J_v^\top(q);\; J_\omega^\top(q)]^\top \in \mathbb{R}^{6\times n}$ is the full geometric Jacobian, with $J_v \in \mathbb{R}^{3\times n}$ the translational sub-Jacobian and $J_\omega \in \mathbb{R}^{3\times n}$ the rotational sub-Jacobian [25]. $\mathcal{F}_h$ is defined as positive when the human pushes \emph{onto} the robot (reaction force transmitted to the robot structure), so it enters (1) with a positive sign, adding to the generalized force that accelerates the joints.

Let $p \in \mathbb{R}^3$ denote the end-effector Cartesian position. The translational Jacobian $J_v(q) \in \mathbb{R}^{3\times n}$ satisfies $\dot{p} = J_v(q)\dot{q}$ [25]. The operational-space inertia [13] is:

$$\Lambda(q) = \bigl(J_v M^{-1} J_v^\top\bigr)^{-1} \in \mathbb{R}^{3\times3} \tag{2}$$

$\Lambda(q)$ is symmetric positive-definite; its off-diagonal entries couple the $x$, $y$, $z$ tip directions. The dynamically-consistent pseudoinverse [13], [12] is $\bar{J}_v = M^{-1}J_v^\top\Lambda$, which ensures that forces applied through $J_v^\top$ produce no acceleration in the null space of $J_v$.

After feedforward cancellation (§III-A), the human force $F_h \in \mathbb{R}^3$ and joint friction appear as an aggregated Cartesian *acceleration* disturbance $d_\text{acc}(t)$ (units m/s² — kept notationally distinct throughout from its *force*-equivalent $d$, units N, introduced in §III-C and used from Layer 2 onward). We model $d_\text{acc}(t)$ as a random walk:

$$\dot{d}_\text{acc} = w(t), \quad w \sim \mathcal{N}(0, Q_d) \tag{3}$$

which captures both constant forces (physiological tremor, sustained pushes) and slow-varying loads (unmodelled payloads, tool changes).

The control objective is to design a torque law $\tau$ such that for any bounded persistent disturbance $d$:
- $\|e(\infty)\| = 0$ where $e = p_d - p$ (zero steady-state tracking error)
- $\|e(t)\|$ is minimized over a finite prediction horizon (minimum transient deflection)
- All joint positions stay inside $q_i \in [q_{\min,i} + \epsilon,\; q_{\max,i} - \epsilon]$ when the certified one-step safety filter is enforced.

This objective is not posed on the robot configuration but on the interaction error at the contact port. We make that modeling shift precise.

**Definition 1 (Interaction Dynamics).** We call *interaction dynamics* the closed-loop dynamical system whose state is the interaction error at the contact port — the task-tracking error and its rate, augmented by the persistent interaction force — and whose evolution, under the actuator, joint-limit, and passivity constraints, is driven by the task control input. Formally, where classical robot control takes the configuration-space dynamics
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau + J^\top(q)\mathcal{F}_h$$

as the object to be predicted and regulated, interaction dynamics takes the augmented error system
$$x_{k+1} = A\,x_k + B\,u_k + E\,d_k$$

as the primary modeling object, with $x_k$ the interaction error state, $u_k$ the task force, and $d_k$ the interaction disturbance. What distinguishes this from a bare tracking-error double integrator is not the state equation in isolation but the *object* it defines: the error, the interaction force (as an estimated, internally-modeled disturbance state rather than an exogenous input), and the actuator/joint-limit/passivity constraints are taken together as one predicted-and-optimized system. Plain error dynamics describe how a fixed controller's error evolves; the interaction dynamics of Definition 1 are the thing the controller is designed *on*. The remainder of the paper constructs one concrete, offset-free realization of this object: §III shows that the transition matrix $A=A_d$ is configuration-independent (8), that the robot dependence is confined to $B=B_d(\rho_k)$, and that $E$ propagates the estimated interaction force (11); the actuator constraint enters the QP of §III-B, and §IV closes the loop with the joint-limit and passivity constraints. Modeling the *interaction* rather than the *configuration* is what lets compliance, tracking, disturbance rejection, and safety be posed as a single constrained optimization.

---

## III. Linear Interaction-Dynamics Backbone and Finite-Horizon Realization

### A. Layer 1 — Feedforward Nonlinear Inversion

Classical Cartesian impedance control [1] directly commands the joint torque as $$\tau = C\dot{q}+G + J_v^\top(\Lambda\ddot{p}_d + \mu - K_d e - D_d\dot{e}),$$ where $\mu = \Lambda J_v M^{-1}C\dot{q} - \Lambda\dot{J}_v\dot{q}$ collects the task-space Coriolis and centripetal terms [13], embedding the full impedance law inside a single static feedback law. While this approach shapes the closed-loop impedance, it treats the human force $F_h$ and all unmodeled dynamics as a disturbance to be passively rejected through the impedance spring-damper — with no predictive look-ahead and no mechanism to drive steady-state error to zero. Furthermore, the gravity and Coriolis compensation terms $C\dot{q}+G$ and the configuration-varying inertia $\Lambda(q)$ are recomputed implicitly at each step but remain entangled with the impedance feedback gain, making systematic constraint enforcement or disturbance augmentation structurally difficult.

The proposed Layer 1 explicitly separates these concerns. Following the operational-space decomposition of Khatib [13], the full torque command is partitioned into four orthogonal channels via the null-space projector $\bar{N} = I - \bar{J}\,J$ [12], where $\bar{J} = M^{-1}J^\top\Lambda_6$ is the dynamically-consistent pseudoinverse of the full $6$-DOF pose Jacobian $J$ and $\Lambda_6 = (J M^{-1} J^\top)^{-1}\in\mathbb{R}^{6\times6}$ is the full-pose operational-space inertia (both the translational MPC task and the orientation channel below are actively controlled, so the projector annihilates the full pose task, not only its translational part):

$$\tau = \tau_\text{ff} + J_v^\top F_\text{mpc} + J_\omega^\top F_\text{orient} + \bar{N}^\top \tau_\text{null} \tag{4}$$

The feedforward term cancels all known nonlinearities at the current measured state using the computed-torque method [25]:

$$\tau_\text{ff} = \underbrace{C(q,\dot{q})\dot{q} + G(q)}_{\texttt{qfrc\_bias}} + J_v^\top \Lambda(q)\,\ddot{p}_d \tag{5}$$

The first term compensates gravity and Coriolis forces [25]; the second maps the desired Cartesian acceleration $\ddot{p}_d$ through the operational-space inertia $\Lambda(q)$ [13] to produce the inertia-consistent feedforward torque. Writing the Layer-2 task channel as $\tau_\text{mpc} \equiv J_v^\top F_\text{mpc}$, define the known non-MPC torque offset
$$\tau_\text{base}=\tau_\text{ff}+J_\omega^\top F_\text{orient}+\bar{N}^\top\tau_\text{null}.$$
The **final applied command is the joint torque** $\tau=\tau_\text{base}+\tau_\text{mpc}$, which the hardware saturates at the per-joint actuator limit $|\tau_i|\le\tau_{\max,i}$ (on the FR3, $\tau_\text{max}=[87,87,87,87,12,12,12]$ Nm). Layer 2 constrains this applied receding-horizon torque, while the hardware interface keeps clipping and rate limiting as final runtime guards. Unlike classical impedance control [1], which folds both compensation and feedback into a single torque command, equation (5) isolates all configuration-dependent nonlinearities so that the MPC in Layer 2 operates on a purely linear residual plant.

**Proposition 1.** After applying (5), the residual error dynamics are represented by the linear double integrator

$$\ddot{e} = -\Lambda^{-1}(q)\,F_\text{mpc} + d_\text{acc}(t) \tag{6}$$

with $e = p_d - p$; every term left over from the feedforward cancellation — not just the human force — is collected into the *acceleration*-form disturbance $d_\text{acc}(t)$, defined below (not to be confused with its force-equivalent $d=-\Lambda(q)d_\text{acc}$, §III-C, which is the quantity actually estimated and cancelled by the controller from Layer 2 onward):

$$d_\text{acc}(t) = \underbrace{-\Lambda^{-1}(q)\,\bar{J}_v^\top J^\top \mathcal{F}_h}_{\text{projected pHRI force}} \;\underbrace{-\,J_v M^{-1} J_\omega^\top F_\text{orient}}_{\text{orientation coupling}} \;\underbrace{-\,\dot{J}_v(q,\dot{q})\dot{q}}_{\text{Jacobian derivative}} \;+\; \underbrace{\epsilon_m(t)}_{\text{model error}} \tag{7}$$

where $\bar{J}_v^\top = \Lambda J_v M^{-1} \in \mathbb{R}^{3\times n}$ is the row-transposed dynamically-consistent pseudoinverse and $J^\top \in \mathbb{R}^{n\times 6}$ is the full geometric Jacobian transpose (so $\bar{J}_v^\top J^\top \in \mathbb{R}^{3\times 6}$ acts on the $6$-D wrench $\mathcal{F}_h$; equivalently the term is $-J_v M^{-1} J^\top \mathcal{F}_h$). The four components have distinct physical origins:
(i) the *projected pHRI force* maps the full human wrench $\mathcal{F}_h \in \mathbb{R}^6$ through joint space to translational task space — for a purely translational force $\mathcal{F}_h = [F_h^\top, 0]^\top$ this reduces to the dominant disturbance during contact:
$$-\Lambda^{-1}\Lambda J_v M^{-1}J_v^\top F_h = -\Lambda^{-1}F_h,$$

Because $e = p_d - p$, any positive acceleration of $p$ induced by the human force appears with a negative sign in $\ddot{e}$, which is why a force that is positive in the joint dynamics enters (6) as a negative error acceleration.
(ii) the *orientation coupling* $-J_v M^{-1}J_\omega^\top F_\text{orient}$ is the residual translational acceleration induced by the orientation PD torque. With the simple additive orientation channel in (4), this mobility cross-term is generally structural rather than a numerical artifact; it is bounded and, in its force-equivalent form, correctly captured by $\hat{d}$;
(iii) the *Jacobian derivative term* $-\dot{J}_v\dot{q}$ arises because $\ddot{p} = J_v\ddot{q} + \dot{J}_v\dot{q}$, contributing a centripetal-like acceleration that is small at typical operating speeds;
(iv) the *model error* $\epsilon_m(t)$ captures residual gravity/Coriolis cancellation error from imperfect model knowledge.

**Remark (Exactness of the reduction).** Under *ideal* model cancellation the residual plant is exactly the linear double integrator (6); in practice the terms in (7) — orientation coupling, the Jacobian-rate $\dot J_v\dot q$, and gravity/Coriolis cancellation error $\epsilon_m$ — are not identically zero. They are bounded and slowly varying at typical operating speeds, and rather than being neglected they are *modelled* as the acceleration disturbance $d_\text{acc}(t)$, whose force-equivalent $d=-\Lambda(q)d_\text{acc}$ the augmented Kalman state $\hat d$ estimates and the MPC rejects (Theorem 2). The controller therefore acts on a double integrator driven by $d_\text{acc}(t)$, with the configuration dependence confined to the input matrix $B_d(\rho_k)$ — leaving $A_d$ constant (the property exploited in Layer 2). The double-integrator reduction is thus a *cancellation-plus-disturbance* model, not an assumption of perfect inversion.

**Disturbance representation.** The decomposition (7) is written in acceleration units (m/s²) to expose its physical origins. The estimator and QP of §III-B–C instead operate on the equivalent *force-form* disturbance $-\Lambda(q)\,d$ — the quantity that enters through the input matrix $B_d(\rho_k)$ in (11), is constant for a constant human force, and is the form realized in code (see §III-C).

### B. Layer 2 — Discrete LPV Model and QP

Define the error state $x_e = [e^\top,\; \dot{e}^\top]^\top \in \mathbb{R}^6$ and scheduling variable $\rho = q$. Exact zero-order-hold (ZOH) discretization at MPC sample time $\Delta t$ gives:

$$x_{e,k+1} = \underbrace{\begin{bmatrix}I_3 & \Delta t I_3 \\ 0 & I_3\end{bmatrix}}_{A_d\;\text{(constant)}} x_{e,k} + \underbrace{\begin{bmatrix}-\tfrac{\Delta t^2}{2}\Lambda^{-1}(\rho_k) \\ -\Lambda^{-1}(\rho_k)\Delta t\end{bmatrix}}_{B_d(\rho_k)} F_{\text{mpc},k} \tag{8}$$

Equation (8), together with the disturbance augmentation (11) below, *is* the interaction dynamics of Definition 1 in concrete form: the state $x_e$ is the interaction error at the contact port — not the robot configuration $q$ — the input $F_\text{mpc}$ is the task force, and $\hat d$ is the estimated interaction disturbance. Identifying $A=A_d$, $B=B_d(\rho_k)$, and $E$ with the disturbance-propagation block gives the linear system $x_{k+1}=Ax_k+Bu_k+Ed_k$ of Definition 1. Everything that follows — offset-free tracking, actuator feasibility, and joint-limit safety — is a property of this interaction-dynamics model, not of the robot configuration dynamics (1).

**Structural result.** The continuous-time state matrix of the double integrator (6), $A_c = \begin{bmatrix}0 & I_3 \\ 0 & 0\end{bmatrix}$, is nilpotent ($A_c^2=0$), so the matrix exponential terminates exactly at first order and $A_d = e^{A_c\Delta t} = I + A_c\Delta t$ — the constant transition matrix in (8) — is configuration-independent and exact, with no discretization approximation; the exact ZOH input integral $B_d(\rho_k)$ is likewise closed-form. The free-response matrix $\Phi = [A_d;A_d^2;\ldots;A_d^N]\in\mathbb R^{6N\times6}$ and the stage-cost Hessian blocks are precomputed once at startup; only the input-to-state matrix $\Gamma(\rho)\in\mathbb R^{6N\times3N}$, built from $B_d(\rho_k)$, is updated online. This scheduling-dependent input matrix follows the LPV-MPC structure of [24], where a parameter-varying input matrix is optimized subject to box constraints; here the scheduling variable $\rho_k=q_k$ is the current joint configuration. The LPV pair $(A_d,B_d(\rho))$ is uniformly controllable, and the disturbance state augmented in §III-C is detectable, at every configuration away from kinematic singularities (proof: supplementary material, §S1, Remark S1).

*Implementation note (regularization).* In code the operational-space inverse is formed as $\Lambda^{-1}(q) = J_v M^{-1} J_v^\top + \sigma I$ with $\sigma = 10^{-6}$ for numerical conditioning. This Tikhonov term is negligible in the well-conditioned workspace interior, but it mildly damps the estimated disturbance (and the realized stiffness) along directions that become ill-conditioned as a kinematic singularity is approached. Operation is restricted to the singularity-free workspace, where its effect stays below the measurement-noise floor.

The receding-horizon QP with $N = 10$ steps, $3N = 30$ decision variables $U = [F_{\text{mpc},0};\ldots;F_{\text{mpc},N-1}]$, is:

$$\min_U \;\frac{1}{2}U^\top H U + h^\top U \tag{9a}$$
$$\text{s.t.}\quad -\tau_\text{max}\le \tau_\text{base}(0)+J_v^\top(q_0)F_{\text{mpc},0}\le\tau_\text{max}, \tag{9b}$$
$$-F_\text{max}\le F_{\text{mpc},k}\le F_\text{max},\quad k=0,\ldots,N-1. \tag{9c}$$

where $F_\text{max}$ is the task-space (Cartesian) corrective-force bound on the QP's decision variable (distinct from the joint-torque limit $\tau_\text{max}$ in (9b)); $\tau_\text{base}(0)=\tau_\text{ff}+J_\omega^\top F_\text{orient}+\bar{N}^\top\tau_\text{null}$ is the known non-MPC torque offset at the current configuration, as defined in §III-A; $H = \Gamma^\top \bar{Q}\Gamma + \bar{R}$, with stage cost $Q = \text{blkdiag}(K_d, D_d) \in \mathbb{R}^{6\times6}$, terminal cost $Q_f = \gamma Q$ ($\gamma = 5$), 
$$\bar{Q} = \text{blkdiag}(Q,\ldots,Q,Q_f) \in \mathbb{R}^{6N\times6N}, \bar{R} = \text{blkdiag}(R,\ldots,R) \in \mathbb{R}^{3N\times3N}.$$ 

Let $d_N=\mathbf 1_N\otimes\hat d$. The linear term is 
$$h=\Gamma^\top\bar Q\,x_\text{free}+\bar R\,d_N,$$
which corresponds to the offset-free input-centred effort penalty
$\|U+d_N\|_{\bar R}^2$. The free response is 
$$x_\text{free} = \Phi\,x_e + D_\text{bar}\,\hat{d},$$ 

where $D_\text{bar} \in \mathbb{R}^{6N\times3}$ is the disturbance propagation matrix with $j$-th block $\sum_{l=0}^{j-1}A_d^l B_d(\rho_k)$, which accumulates how a constant $\hat{d}$ maps through the prediction horizon (see §III-C). Equivalently, $D_\text{bar}=\Gamma(\mathbf 1_N\otimes I_3)$. Only $F_{\text{mpc},0}$ is applied; the horizon shifts forward (receding-horizon principle).

**Constraint interpretation (feasible final control).** The final applied command is the joint torque $\tau=\tau_\text{base}+J_v^\top F_{\text{mpc},0}$, where $\tau_\text{base}$ contains feedforward, orientation, and null-space terms. The hardware saturates this total command at the per-joint actuator limit $\tau_\text{max}$. Constraining only the correction, $\|F_\text{mpc}\|_\infty\le F_\text{max}$, would not keep $\tau$ within $\tau_\text{max}$ because it ignores the feedforward and secondary-task torque offset. We therefore constrain the first applied receding-horizon torque directly, as in (9b). Since $\tau_\text{base}(0)$ and $J_v(q_0)$ are known at solve time, the constraint is affine in $F_{\text{mpc},0}$; the nominal applied torque satisfies $|\tau_i|\le\tau_{\max,i}$ whenever the QP is feasible. Runtime clipping and rate limiting remain as final safety guards; horizon-wide torque rows can be added but are not required for feasibility of the applied receding-horizon action. If an over-aggressive reference makes $\tau_\text{base}(0)$ alone exceed $\tau_\text{max}$, the constraint could similarly be softened with a penalized slack so the QP stays feasible and returns a minimum-violation command — analogous to the joint-limit CBF relaxation of §IV-B, though for this constraint the present implementation instead returns a zero correction on infeasibility (§S8), which is simpler but not minimum-violation.

The following result establishes that classical impedance control is not a competitor to the proposed framework but a special case of it: it is what the interaction dynamics reduce to once prediction, constraints, and disturbance augmentation are switched off. This locates impedance control inside the interaction-dynamics picture and shows precisely which ingredients extend it.

**Theorem 1 (Impedance Equivalence).** Fix a configuration and assume the input constraints are inactive and the disturbance estimate is zero. Then the first MPC move is a linear state feedback $F_\text{mpc} = K_e e + K_{\dot e}\dot e$. If the realized gain blocks are symmetric positive definite, or more generally have symmetric parts that define passive stiffness and damping, set $K_\text{eff}=\operatorname{sym}(K_e)\succ0$ and $D_\text{eff}=\operatorname{sym}(K_{\dot e})\succ0$; the conservative part of the closed loop is the classical task-space impedance:
$$\Lambda(q)\,\ddot{e} + D_\text{eff}\,\dot{e} + K_\text{eff}\,e = -F_h, \tag{10}$$

where $F_h$ is the constant force-equivalent disturbance applied to the end effector.

*Proof.* With inactive constraints and $\hat d=0$, the QP (9) reduces to an unconstrained strictly convex quadratic because $H=\Gamma^\top\bar Q\Gamma+\bar R\succ0$ when $\bar R\succ0$. Its unique minimizer is
$$U^\star = -H^{-1}\Gamma^\top\bar Q\Phi x_e.$$

Let $S=[I_3\;0\;\cdots\;0]$ select the first force block. The applied force is
$$F_\text{mpc}=SU^\star = [K_e\;K_{\dot e}]\begin{bmatrix}e\\\dot e\end{bmatrix},$$

for fixed matrices $K_e,K_{\dot e}$ determined by $(A_d,B_d,\bar Q,\bar R,Q_f,N)$. Under the stated symmetry/sign condition, the symmetric parts define $K_\text{eff}$ and $D_\text{eff}$. The exact linear closed loop uses the full gains $K_e,K_{\dot e}$; only their symmetric parts admit the conservative spring-damper interpretation. The skew part of $K_{\dot e}$ is workless gyroscopic damping coupling, whereas the skew part of $K_e$ is a circulatory position force and is not passive in general. These skew terms vanish in the common isotropic/commuting case and are otherwise treated as part of the stabilizing LQR feedback rather than as classical impedance parameters. The residual plant is $\ddot e=-\Lambda^{-1}F_\text{mpc}+d_\text{acc}$. For a translational constant human force represented in force form by $F_h=-\Lambda d_\text{acc}$, multiplication by $\Lambda$ gives $\Lambda\ddot e=-F_\text{mpc}-F_h$. Substituting the linear feedback gives the stated impedance. $\square$

Thus the unconstrained MPC realizes a configuration-dependent linear impedance whenever the gain blocks have the passive stiffness/damping sign structure; constraints and disturbance augmentation are the mechanisms that extend this linear impedance behavior.

**Remark (Optimality vs. passivity — motivation for the energy tank).** Theorem 1 exposes a structural tension. Because the predictive gain is chosen for *optimality* — it is the LQR/Riccati image of $(Q,R)$ — rather than for passivity, the skew part of $K_e$ is a circulatory position force that is not passive in general and can inject energy along particular trajectories. For the robot in isolation this is benign: the DARE renders the closed loop asymptotically stable (supplementary material, §S1, Remark S1). But in pHRI the environment is itself a dynamical system, and a non-passive controller coupled to a human need not preserve stability of the *coupled* system even when each part is individually stable. This is the mathematical motivation for the energy-tank layer of §IV-C: rather than restricting $(Q,R)$ to the conservative subset that yields passive gains — sacrificing predictive performance — we keep the optimal predictive law and restore a *sampled energy-budget certificate on the scaled translational predictive-force channel* at runtime (Proposition 2), which bounds the positive work that channel can deliver. This is a channel-level certificate, not a full-port passivity proof — see the scoping discussion in §IV-C — so it does not by itself guarantee coupled stability against an arbitrary passive human or environment; it limits one identified non-passive source (the predictive channel's circulatory stiffness) while leaving feedforward, orientation, and null-space contributions to the port outside its accounting. The optimality that makes the predictive layer accurate is thus part of, not the whole of, what safe co-manipulation requires.

**Remark 2 (Prescribed vs. realized gains).** The cost weights $Q=\text{blkdiag}(K_d,D_d)$, $R$ are design *penalties*; the realized impedance $(K_\text{eff},D_\text{eff})$ is their LQR image — a nonlinear (Riccati) map of $(Q,R)$ — so in general $(K_\text{eff},D_\text{eff})\neq(K_d,D_d)$. The weights *shape* the realized impedance: the high-bandwidth tracking of Section VI corresponds to the stiff, well-damped $K_\text{eff}$ that the chosen $(Q,R)$ induce. If instead a *specific* $(K_d,D_d)$ must be matched exactly, the impedance gain can be prescribed directly ($F_\text{mpc}=K_d e + D_d\dot e$), rendering the equivalence exact for the prescribed gains at the cost of the predictive look-ahead. We adopt the LQR form, for which Theorem 1 already establishes the analytical connection between the unconstrained MPC and a realized classical impedance, and generalizes it with constraint handling and offset-free disturbance rejection.

**Corollary 1 (Existence of a Gain-Scheduled Infinite-Horizon Implementation).** The infinite-horizon predictive realization admits a pure state-feedback law $F_{\text{mpc},k} = -K_\infty(\rho_k)\,x_{e,k}$ with optimal gain:

$$K_\infty(\rho_k) = \bigl(R + B_d^\top(\rho_k) P_\infty(\rho_k) B_d(\rho_k)\bigr)^{-1} B_d^\top(\rho_k) P_\infty(\rho_k) A_d$$

Under the DARE detectability conditions established in the supplementary material (§S1, Remark S1), the stabilizing solution $P_\infty(\rho)$ exists for each fixed $\rho$, and $K_\infty$ depends on $\rho_k$ only through $\Lambda^{-1}(q_k)$ — a symmetric $3\times3$ SPD matrix, so the scheduling space is the 6-dimensional manifold of its independent entries. Since the stabilizing DARE solution is continuous in its coefficients, $K_\infty(\cdot)$ is continuous over this compact domain. $\square$

**Remark 3 (Practical 1 kHz implementation).** Continuity of $K_\infty$ over a compact, low-dimensional (6-D) domain makes an offline-precomputed lookup attractive: the gain can be tabulated on a grid of $\Lambda^{-1}$ values and retrieved by interpolation, reducing the online cost to a single $3\times6$ matrix–vector product. We present this as a deployment *option* rather than a validated result — grid density, interpolation error, and the resulting closed-loop quality are implementation choices we do not characterize here. The experiments in Section VI instead run the finite-horizon QP (9) at 100 Hz with a 1 kHz inner loop, which additionally enforces the first applied torque constraint and exploits prediction look-ahead; a hardware demonstration of the gain-scheduled 1 kHz law is future work.

### C. Kalman Disturbance Augmentation for Offset-Free Tracking

For the estimator we represent the disturbance in **force form**: the augmented state $d \in \mathbb{R}^3$ (units N) is the force-equivalent of the task-*acceleration* disturbance of (7), related by $d = -\Lambda(q)\,d_\text{acc}$. This form is used for two reasons: (i) it enters the discrete error dynamics through the *same* input matrix $B_d(\rho_k)$ as the control $F_\text{mpc}$, so a single matrix builds both channels in (11); and (ii) for a constant human force it is itself constant — whereas the acceleration form $d_\text{acc} = \Lambda^{-1}(q)(\cdot)$ varies with configuration as $\Lambda(q)$ changes — so the constant-disturbance hypothesis of Theorem 2 matches a sustained push exactly. It is the quantity the implementation stores as $\hat{d}$ and cancels through $F_\text{mpc} = -\hat{d}$ at steady state. We model it as a random walk:

$$\begin{bmatrix}x_{e,k+1} \\ d_{k+1}\end{bmatrix} = \underbrace{\begin{bmatrix}A_d & B_d(\rho_k) \\ 0 & I_3\end{bmatrix}}_{A_\text{aug}} \begin{bmatrix}x_{e,k} \\ d_k\end{bmatrix} + \begin{bmatrix}B_d(\rho_k) \\ 0\end{bmatrix} F_{\text{mpc},k} + \begin{bmatrix}0 \\ w_k\end{bmatrix}, w\sim\mathcal{N}(0,Q_w) \tag{11}$$

The disturbance block $d_{k+1}=d_k+w_k$ is the standard integrating (random-walk) disturbance model [4]: its deterministic part is a pure integrator (eigenvalues at 1), which by the internal-model principle is what delivers offset-free rejection of a constant disturbance, while the process noise $w$ (covariance $Q_w$) lets the filter *track* slowly-varying disturbances. A steady-state Kalman filter observing $[e;\;\dot{e}]$ produces the estimate $\hat{d}$ of this state via the measurement update; $\hat{d}$ is not frozen — it is driven by the innovation each step and converges to the true disturbance. Both $e$ and $\dot{e}$ are measured directly (position from forward kinematics, velocity from $J_v\dot{q}$), so the observation matrix is $C=[I_6\;\,0]$ and the filter is 9-dimensional — the 6 error states and the 3 integrating disturbance states — with no additional velocity-estimation states. The estimate enters the free response, $x_\text{free} = \Phi\,\hat{x}_e + D_\text{bar}\,\hat{d}$, pre-compensating the QP for the persistent disturbance before it accumulates in the error state.

*Implementation note (process-noise scaling).* The random-walk covariance $Q_w$ is a fixed tuning constant in the present implementation. For a disturbance-tracking bandwidth invariant to the QP rate it should scale with the sample time, $Q_w = Q_d\,\Delta t$, which is the exact discretization of the continuous random walk (3); the single constant used here was verified to behave well across the 100–500 Hz range tested.

**Theorem 2 (Offset-Free Steady-State Tracking).** Consider the augmented model (11) at a fixed configuration, with constant disturbance $d_k=d_\infty$. Assume: (i) the augmented estimator is detectable and its estimation error is asymptotically stable, so $\hat d_{k}\to d_\infty$; (ii) the receding-horizon controller stabilizes the nominal augmented closed loop; and (iii) the input constraint is inactive at the steady state. Then the steady-state tracking error is zero: $\lim_{k\to\infty} e_k=0$. If $d_k\to d_\infty$ asymptotically, the same conclusion holds provided the estimator remains asymptotically tracking.

*Proof.* Let $\hat d_N=\mathbf 1_N\otimes\hat d$ and define the centred input sequence $V=U+\hat d_N$. Since $D_\text{bar}=\Gamma(\mathbf 1_N\otimes I_3)$, the identity
$$\Phi x_e+D_\text{bar}\hat d+\Gamma U=\Phi x_e+\Gamma V$$

holds for any disturbance estimate $\hat d$, not only at steady state. The unconstrained cost can therefore be written, up to constants, as
$$\tfrac12(\Phi x_e+\Gamma V)^\top\bar Q(\Phi x_e+\Gamma V)+\tfrac12V^\top\bar R V.$$

Thus the nominal controller is exactly the stabilizing linear regulator for the disturbance-free backbone in the variable $V$: $V^\star=-Kx_e$. The applied first input is therefore $F_\text{mpc}=v_0-\hat d$, and the plant evolves as
$$x_e^+=(A_d-B_dK_0)x_e+B_d(d_\infty-\hat d),$$

where $K_0$ is the first block row of $K$. By assumption (ii), the nominal closed loop is asymptotically stable. By assumption (i), the additive input $B_d(d_\infty-\hat d)$ vanishes, so the stable linear system converges to the same equilibrium. Hence $x_{e,k}\to0$ and therefore $e_k\to0$.

The theorem does not claim zero error for arbitrary non-convergent time-varying forces or for steady states that require sustained actuator saturation; those cases require a separate invariant-set or anti-windup analysis. $\square$

**Remark 4 (Frozen-configuration scope).** Theorem 2 is a *frozen-configuration* offset-free result: it fixes $\rho_k = q$ so that $B_d(\rho_k)$ and the regulator gain are constant. Theorem 3 below relaxes this to a single *fixed feedback law* certified over a polytope of configurations, rather than one frozen configuration — but that certified law is not the deployed receding-horizon MPC's realized policy (see the scope discussion accompanying Theorem 3), so Theorem 3 does not by itself remove the frozen-configuration caveat for the actual controller. The moving-reference experiments in §VI are an empirical illustration, not a direct consequence of Theorem 3; the per-equilibrium offset-free property is Theorem 2 applied at each sustained-contact configuration the trajectory visits.

**Theorem 3 (Quadratic Stabilizability of the Interaction-Dynamics Backbone).** If the operational-space inverse-inertia stays within a compact polytope $\mathcal P$ over the operating region of interest, a finite vertex LMI feasibility test certifies a *single* fixed linear feedback gain $K$ that makes the LPV backbone $x_{e,k+1}=A_d x_{e,k}+B_d(\rho_k)F_{\text{mpc},k}$ exponentially stable — by one common quadratic Lyapunov function — uniformly over $\mathcal P$, independent of how fast the configuration $\rho_k$ varies. This is a statement about the LPV *backbone* under that single certified gain, not directly about the finite-horizon receding-horizon MPC actually deployed, whose realized policy at each step is the QP's own minimizer rather than necessarily the certified $K$; the disturbance estimator, the receding-horizon re-solve, and the safety/passivity filters (Theorem 4, Proposition 2) are separately-scoped results, not covered by this one certificate. For the FR3, sampling $\Lambda^{-1}(q)$ along the §VI circular benchmark (4700 samples, 64 polytope vertices, all confirmed positive-definite) gives a feasible, well-conditioned certificate at guaranteed rate $\rho\le0.996$ over the *sampled operating region traversed by the benchmark trajectory* — not a verified bound over the entire singularity-free workspace. The full statement, the LMI, the proof, and the numerical instantiation are in the supplementary material (§S2, Theorem S1), reproducible from the released script (`lmi_workspace_stability_probe.py`).

### D. Disturbance Prediction Validity over the Horizon

A natural question for any offset-free MPC is how good the disturbance prediction is *over the prediction horizon*, not merely at the current step. The QP propagates the augmented model (8)+(11) under the random-walk prediction $\hat{d}_{k+i\mid k}=\hat{d}_{k\mid k}$ — a flat extrapolation that is exact only for a strictly constant disturbance. For a time-varying human force the prediction degrades with horizon depth $i$. We address this concern in three parts.

**Bound.** Assume the human force is Lipschitz in time, $\|\dot{d}(t)\|_2\le L_d$, and let $\|d_k-\hat{d}_{k\mid k}\|_2\le e_K$ be the steady-state Kalman estimation error. Since the random-walk prediction is the identity, the $i$-step prediction error obeys

$$\|d_{k+i}-\hat{d}_{k+i\mid k}\|_2 \;\le\; e_K + L_d\,i\,\Delta t, \qquad i=0,\dots,N-1, \tag{12}$$

where $L_d$ (units N/s) is the time-Lipschitz constant bounding the disturbance rate, $\|\dot{d}(t)\|_2\le L_d$ ($L_d\to0$ for a sustained push, $L_d\approx5$ N/s for a brisk guidance force), and $e_K$ is the steady-state Kalman estimation error. The bound follows by the triangle inequality on $d_{k+i}=d_k+\int \dot{d}\,d\tau$. The first term is the *current* estimation error (bounded by filter design, shrinking as the filter converges); the second is the *extrapolation* error, linear in horizon depth and unavoidable for any constant-disturbance internal model.

**Why it is not load-bearing.** Three properties keep (12) benign. (i) *Receding horizon:* only $F_{\text{mpc},0}$ is applied and the QP is re-solved every $\Delta t$ with a fresh $\hat{d}_{k\mid k}$, so the loose late-horizon predictions never reach the plant — they only shape the near-term optimum. (ii) *Short horizon:* at $N=10$, $\Delta t=10$ ms (100 ms; 20 ms at 500 Hz) the extrapolation term is small in absolute units — for a sustained push (the case of Theorem 2) $L_d\approx0$ and the prediction is exact, while for a brisk voluntary guidance force at $\|\dot{F}_h\|\lesssim 5$ N/s the worst-case end-of-horizon error is $L_d N\Delta t\lesssim0.5$ N, a small fraction of the available control authority. (iii) *Structured augmentation:* when the human force has known temporal structure — physiological tremor (8–12 Hz) or a slow guidance ramp — the random-walk block can be replaced by the corresponding internal model (a harmonic oscillator state at the tremor frequency, or a constant-plus-ramp double integrator) so that the component is *predicted* forward exactly rather than extrapolated flat. This grows the estimator by a few states, leaves the constant-$A_d$ error block (and hence the precomputed $\Phi$) untouched, and removes that component from $L_d$ in (12).

**Metric.** To quantify prediction quality directly rather than assert it, the natural diagnostic is the **$N$-step disturbance-prediction RMS**, $\varepsilon_N=\mathrm{RMS}_k\,\|d_k-\hat{d}_{k\mid k-N}\|$ — the error of the estimate propagated $N$ steps before the measurement — monitored alongside the one-step value $\varepsilon_1$. For the step-force benchmark (Section VI), the disturbance is piecewise constant, so $\varepsilon_N\approx\varepsilon_1$ except across the force-onset transient; the gap $\varepsilon_N-\varepsilon_1$ isolates exactly the horizon-extrapolation error and is the quantity to watch when porting the controller to time-varying interaction. This diagnostic is measured directly under time-varying forces in the supplementary material (§S6, Table SIV), where the gap is shown to grow linearly with the disturbance rate as (12) predicts.

---

## IV. Orientation Stabilization and Null-Space Control

### A. Orientation Stabilization

Orientation is regulated by a PD law in the operational space [13]. The rotational Jacobian $J_\omega \in \mathbb{R}^{3\times7}$ relates joint velocity to end-effector angular velocity $\omega = J_\omega\dot{q}$ [25]. The axis-angle orientation error $e_R \in \mathbb{R}^3$ is extracted from the relative rotation matrix $R_d^\top R$ [25], where $R$ and $R_d$ are the current and desired end-effector orientations. The orientation torque is:

$$\tau_\text{orient} = J_\omega^\top(-K_\text{rot}\,e_R - D_\text{rot}\,\omega) \tag{13}$$

with $K_\text{rot} = 20\,\text{Nm/rad}$, $D_\text{rot} = 6\,\text{Nm·s/rad}$ (critically damped). This follows the operational-space impedance structure of [13], [18], and runs at the full 1 kHz inner-loop rate. It is implemented as a *separate* control channel from the translational QP in (4) — not a dynamically decoupled one: (4)'s null-space projector $\bar N^\top$ applies only to $\tau_\text{null}$, and $J_\omega^\top F_\text{orient}$ still couples into translational acceleration through the mass matrix, as made explicit by the orientation-coupling term in (7). That residual dynamic coupling is part of $d_\text{acc}(t)$ (7), whose force-equivalent is what $\hat d$ estimates and cancels — not eliminated by projection.

### B. Null-Space Joint-Limit Safety and Invariance

Because both the translational MPC channel and the orientation PD channel are actively controlled, the primary task is the full 6-DOF pose. A manipulator with $n > 6$ joints then has an $(n-6)$-dimensional null space; for the FR3 ($n=7$) this gives 1 null-space DOF. The null-space torque $\bar{N}^\top\tau_\text{null}$ (where $\bar{N} = I - \bar{J}\,J$ is the dynamically-consistent null-space projector of the full pose Jacobian [12], [13]) does not affect end-effector position or orientation, enabling secondary objectives to be pursued without interfering with the task [12].

**Null-space inverse-barrier.** Joint-limit regulation via null-space repulsive potentials is a standard secondary objective [13], [15]. We adopt an inverse-barrier form [15] based on the clearance (in rad) to the nearest limit:

$$\phi_i(q) = \min(q_i - q_{\min,i},\; q_{\max,i} - q_i) \ge 0$$

The barrier gradient in joint space is:

$$g_i(q) = \begin{cases} +k_b\!\left(\tfrac{1}{\phi_i} - \tfrac{1}{\delta_i}\right) & \phi_i < \delta_i,\; q_i \text{ near lower limit} \\ -k_b\!\left(\tfrac{1}{\phi_i} - \tfrac{1}{\delta_i}\right) & \phi_i < \delta_i,\; q_i \text{ near upper limit} \\ 0 & \text{otherwise} \end{cases} \tag{14}$$

where $\delta_i = \eta(q_{\max,i} - q_{\min,i})$ is the activation threshold ($\eta = 0.10$) and $k_b = 50$ Nm. The barrier is *continuous* ($g_i\to0$ as $\phi_i\to\delta_i^-$, since $1/\phi_i-1/\delta_i\to0$), so the commanded torque has no jump; its gradient, however, has a slope discontinuity at the activation boundary $\phi_i=\delta_i$ (the term switches on at $-k_b/\delta_i^2$). This is bounded and benign for the null-space PD law (which uses $g_i$, not $\nabla g_i$); where gradient continuity is required — for a barrier embedded in the QP, or to avoid exciting joint-torque-sensor noise and unmodeled structural modes via $\dot\tau$ steps on physical hardware (e.g. the FR3) — $g_i$ can be replaced by a $C^1$ smoothly-clipped variant (a cubic or quadratic blend over $[\delta_i-\beta,\delta_i]$) without changing the safe-region behavior. The two cases for the lower/upper limits do not overlap (a joint is near at most one limit), so the sign selection introduces no discontinuity. Combined with a centering spring toward the neutral pose [12], the null-space torque is:

$$\tau_\text{null} = -k_\text{null}(q - q_0) + g(q) - d_\text{null}\dot{q} \tag{15}$$

with centering stiffness $k_\text{null} = 10$ Nm/rad, joint damping $d_\text{null} = 2$ Nm·s/rad, toward the FR3 neutral pose $q_0 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]^\top$ (joints 4 and 6 have ranges excluding zero, so $q_0$ must be set to a feasible pose).

**Task-space workspace projection.** When joints approach limits, the null-space alone cannot prevent violations for task-constrained DOF [15]. We additionally project the barrier gradient into Cartesian space via the Jacobian pseudo-inverse [13] and offset $p_d$:

$$\delta p = k_\text{ws}\,\bigl(J_vJ_v^\top + \epsilon_r I\bigr)^{-1} J_v\,g(q), \quad \|\delta p\| \leq p_\text{max} \tag{16}$$

with $k_\text{ws} = 5\times10^{-4}$ m/(Nm/rad), $p_\text{max} = 0.06$ m, and Tikhonov regularization $\epsilon_r = 10^{-8}$ which prevents ill-conditioning near kinematic singularities [25]. The QP then optimizes toward $p_{d,\text{eff}} = p_d + \delta p$, steering the task-space target away from limit-inducing configurations.

**Conditional sampled-data joint-limit filter.** The dual-barrier controller above provides practical joint-limit regulation. A conditional forward-invariance certificate is obtained by adding a one-step joint-limit control-barrier filter to the commanded torque. Let
$$h_i^-(q)=q_i-q_{\min,i}-\epsilon,\qquad h_i^+(q)=q_{\max,i}-\epsilon-q_i,$$

and define the certified safe set
$$\mathcal C_\epsilon=\{q:\ h_i^-(q)\ge0,\ h_i^+(q)\ge0,\ i=1,\ldots,n\}.$$
At sampling time $k$, approximate the next joint position under constant torque over one servo period $T_s$ by
$$q_{k+1|k}=q_k+T_s\dot q_k+\tfrac12T_s^2M^{-1}(q_k)(\tau_k+\hat\tau_{\rm ext,k}-b_k), \tag{17}$$

where $b_k=C(q_k,\dot q_k)\dot q_k+G(q_k)$, $\hat\tau_{\rm ext,k}=J^\top(q_k)\hat{\mathcal F}_{h,k}$ is the measured or estimated external generalized force, and $\tau_k$ is the final command after the nominal MPC and null-space terms. If no reliable external-force estimate is available, both the lower- and upper-limit inequalities in (18) should be robustly tightened using a bound on $\|\tau_{\rm ext}-\hat\tau_{\rm ext}\|$ propagated through $\tfrac12 T_s^2 M^{-1}(q_k)$. The safety filter selects the closest feasible torque to the nominal command subject to
$$h_i^\pm(q_{k+1|k})\ge(1-\alpha)h_i^\pm(q_k),\qquad i=1,\ldots,n,\quad \alpha\in(0,1]. \tag{18}$$
Since (17) is affine in $\tau_k$, (18) adds linear inequalities to the final torque projection.

**Theorem 4 (Sampled-Data Joint-Limit Invariance).** Assume $q_0\in\mathcal C_\epsilon$, the model used in (17), including $\hat\tau_{\rm ext,k}$ or a valid two-sided robust tightening, matches the sampled plant at $k{+}1$ or encloses it within the tightened lower/upper bounds, and the safety-filter constraints (18) are feasible and enforced at every sample. Then $\mathcal C_\epsilon$ is forward invariant for the sampled joint positions: $q_k\in\mathcal C_\epsilon$ for all $k\ge0$.

*Proof.* Suppose $q_k\in\mathcal C_\epsilon$. Then $h_i^\pm(q_k)\ge0$ for every joint. By (18),
$$h_i^\pm(q_{k+1})=h_i^\pm(q_{k+1|k})\ge(1-\alpha)h_i^\pm(q_k)\ge0,$$

where equality between $q_{k+1}$ and $q_{k+1|k}$ follows from the one-step model-matching assumption. Hence $q_{k+1}\in\mathcal C_\epsilon$. Induction from $q_0\in\mathcal C_\epsilon$ proves the claim. $\square$

The theorem is deliberately conditional: it certifies invariance only when the one-step constraints are feasible and enforced. In the implementation the filter is the *final* torque projection (applied after the nominal MPC and null-space terms), so it takes priority over the soft null-space barrier and workspace projection, which only *bias* the arm away from limits. When the constraints (18) cannot all be met — e.g., the barrier and CBF pull in conflicting directions, or `\tau_base` alone is already limit-inducing — they are relaxed with a penalized slack $s\ge0$ (the QP solves $\min\|\tau-\tau_\text{nom}\|^2 + w\,s^2$ s.t. $h_i^\pm(q_{k+1|k})\ge(1-\alpha)h_i^\pm(q_k)-s$), returning the minimum-violation command rather than becoming infeasible; a logged $s=0$ is exactly the per-sample certificate that Theorem 4 held without relaxation. The experiments below report the empirical behavior of the dual-barrier joint-limit regulation law; hardware runs intended to claim certified joint-limit safety should log the CBF residuals and slack $s$. The certificate is sampled-data: possible intersample motion is covered only to the extent that the margin $\epsilon$ and any robust tightening dominate the within-sample excursion.

**Remark (Robust tightening bound and ISSf).** The one-step predictor $q_{k+1|k}=q_k+\Delta t\,\dot q_k+\tfrac12\Delta t^2\,\ddot q_k$ in (17) holds the commanded acceleration $\ddot q_k = M^{-1}(\tau_k-(C\dot q+G)+\hat\tau_{\rm ext,k})$ over the servo interval. Let $\delta\tau_k$ collect the unmodeled generalized force — the human-torque estimation error $J^\top(F_h-\hat F_h)$, the within-step torque change from finite jerk, and inertial-model error — with a known bound $\|\delta\tau_k\|\le B_\tau$. The induced one-step position error is $\tfrac12\Delta t^2 M^{-1}\delta\tau_k$, bounded by $\Delta q_{\rm safe}=\tfrac12\Delta t^2\,\|M^{-1}\|_2\,B_\tau$. Tightening the one-step constraints by $\Delta q_{\rm safe}$ — equivalently, shrinking $\mathcal C_\epsilon$ by $\Delta q_{\rm safe}\,\|\nabla h_i\|$ — makes Theorem 4 *input-to-state safe* (ISSf): for any disturbance within $B_\tau$ the sampled positions remain in $\mathcal C_\epsilon$, provided $\epsilon\ge\Delta q_{\rm safe}$. At the servo rate $\Delta t=1$ ms with $\|M^{-1}\|_2\lesssim10$ kg$^{-1}$, this gives $\Delta q_{\rm safe}\approx5\times10^{-6}\,B_\tau$ rad ($B_\tau$ in N·m) — well within the $\epsilon$ margin used in the experiments — and the residual intersample motion is covered by the same margin.

### C. Energy-Tank Passivity Layer for Co-Manipulation

The offset-free predictive controller is designed for precision disturbance rejection; intentional co-manipulation additionally needs a passivity certificate at the human-robot port. We therefore add a sampled energy tank on the translational task channel. Let $F_k^0$ be the predictive task force before passivity filtering, $v_k=\dot p_k$ the measured end-effector translational velocity, and $E_k\in[E_{\min},E_{\max}]$ the tank energy. The layer scales only the task force,
$$F_k=s_kF_k^0,\qquad s_k\in[0,1], \tag{19}$$

leaving feedforward gravity/Coriolis compensation, orientation regulation, and null-space torques outside the tank accounting. Define the raw output power $P_k^0=\max\{(F_k^0)^\top v_k,0\}$ and optional human recharge power
$$P_{h,k}=\gamma\max\{\hat F_{h,k}^\top v_k,0\},\qquad \gamma\in[0,1].$$

The scale is chosen as
$$s_k=\begin{cases}
1, & P_k^0=0,\\[2mm]
\min\!\left(1,\dfrac{(E_k-E_{\min})/T_s+P_{h,k}}{P_k^0}\right), & P_k^0>0,
\end{cases} \tag{20}$$

and the tank update is
$$E_{k+1}=\operatorname{sat}_{[E_{\min},E_{\max}]}\!\left(E_k+T_s(P_{h,k}-\max\{F_k^\top v_k,0\})\right). \tag{21}$$

**Proposition 2 (Sampled Passivity of the Task Channel).** If $E_0\ge E_{\min}$ and (20)--(21) are enforced at every servo sample, then $E_k\ge E_{\min}$ for all $k$ and the predictive task-force channel satisfies
$$\sum_{j=0}^{k-1}T_s\max\{F_j^\top v_j,0\}\le E_0-E_{\min}+\sum_{j=0}^{k-1}T_sP_{h,j}.$$

*Proof.* By construction, (20) makes $T_s\max\{F_k^\top v_k,0\}\le E_k-E_{\min}+T_sP_{h,k}$. Substituting this into (21) gives $E_{k+1}\ge E_{\min}$ before saturation, and the saturation preserves the lower bound. Summing the unsaturated energy balance from $0$ to $k-1$ gives the stated dissipation inequality. $\square$

**Scope.** The certificate in Proposition 2 bounds only the scaled translational predictive-force channel $F_k$ — it is a channel-level energy budget, not a full 6-DOF port passivity proof, and should not be read as guaranteeing coupled stability against an arbitrary passive environment. Each excluded channel is *locally* well-behaved in isolation — the orientation channel is a passive PD ($K_\text{rot}, D_\text{rot} > 0$) with respect to its own $(\tau_\text{orient}, \omega)$ port, the null-space torque only redistributes energy within the redundant subspace, and the feedforward gravity/Coriolis compensation is workless at steady state — but this does not by itself bound the *cross-coupling* power these channels can deliver into translation through the mass matrix: (7) explicitly retains an orientation-to-translation coupling term $-J_v M^{-1}J_\omega^\top F_\text{orient}$, so $\tau_\text{orient}$ can do translational work that the tank accounting does not see. A full-port certificate — bounding the coupled channels jointly, not just each in isolation — is a genuine extension, not a straightforward one, and is left to future work.

In hardware logs, the certificate is the tuple `passivity_certified`, `passivity_energy`, and `passivity_scale`. Values `passivity_scale<1` indicate samples where the tank actively reduced the predictive task force to preserve passivity. Setting $\gamma=0$ gives the conservative certificate used by default; choosing $\gamma>0$ allows a configured fraction of measured human input work to recharge the tank.

---

## V. Real-Time Implementation

The two-layer architecture is robot-agnostic: Layer 1 requires only $M$, $C\dot{q}+g$, and $J_v$ from the robot model; Layer 2 is a fixed-size QP in Cartesian space independent of $n$. We describe deployment on the Franka FR3 using the Franka Control Interface (FCI) [16] as the reference implementation; the same code structure applies to any torque-controlled arm by substituting the model API.

| Layer | Computation | Rate | Latency budget |
|---|---|---|---|
| 1 — Feedforward | $\tau_\text{ff}$: `qfrc_bias` + $J_v^\top\Lambda\ddot{p}_d$ | 1 kHz | <0.1 ms |
| 2 — QP | OSQP warm-started, 30 variables, first torque row | 100 Hz | <1 ms |
| Kalman | 9-state predict + update | 100 Hz | <0.1 ms |
| Orientation | $J_\omega^\top(-K_\text{rot}e_R - D_\text{rot}\omega)$ | 1 kHz | <0.1 ms |
| Null-space | Barrier gradient + projection | 1 kHz | <0.1 ms |

**QP solver.** We use OSQP [9] with the problem form 
$$\min \frac{1}{2}u^\top P u + q^\top u \text{, s.t. } l \leq Iu \leq \bar{u}.$$

Warm-starting from the previous solution, in our simulation implementation (single-threaded, commodity CPU), reduced cold-start latency from 5 ms to under 0.5 ms. Although the Hessian $H = \Gamma^\top\bar{Q}\Gamma + \bar{R}$ varies with configuration through $B_d(\rho_k)$, updating it requires only overwriting the pre-allocated non-zero elements of the upper-triangular CSC sparse matrix — a $O(N^2)$ coefficient update completed in under 0.05 ms via `osqp_update_P(Px=...)`, preserving OSQP's factorization-reuse structure and keeping the measured total latency below 0.5 ms in these runs (full timing statistics on the target real-time controller are future hardware work). The linear term $h = \Gamma^\top\bar{Q}\,x_\text{free}+\bar R d_N$ is updated similarly via `osqp_update_lin_cost(q=...)`; the $+\bar R d_N$ term is required by the offset-free input-centering argument in Theorem 2. This interface maps directly to the OSQP C++ API for deployment on any real-time robot controller (libfranka for the FR3, or equivalent).

**ZOH policy.** Between QP solves, $F_\text{mpc}$ is held constant (zero-order hold). Layer 1 ($\tau_\text{ff}$) is recomputed at 1 kHz from fresh $(q, \dot{q})$, continuously cancelling configuration-varying gravity and Coriolis. The ZOH window is 10 ms at 100 Hz, causing a bounded transient of order $\|d\| \cdot \Delta t_\text{MPC}$ at each force onset.

**MuJoCo simulation.** All experiments use the FR3 model from MuJoCo Menagerie [10] with the MuJoCo physics engine [11] at 1 kHz (dt = 1 ms). The `qfrc_bias` and `mj_jac` interfaces provide $M$, $C\dot{q}+g$, and $J_v$ — the same quantities available from any physics engine or robot model library. The hardware-path MuJoCo verification suite runs the same `FR3ImpedanceMPCHardwareInterface` used for real FR3 deployment, including torque-rate limiting, the one-step CBF projection, and the energy-tank passivity filter; direct benchmark scripts are used only for algorithm comparisons and figures. Porting to a different manipulator requires only a new MuJoCo XML model (or equivalent URDF) and updated joint-limit parameters.

Every numeric constant used across the experiments of §VI is consolidated in Table SI of the supplementary material.

---

## VI. Experiments

We evaluate the controller on two dynamic benchmarks and two stress tests, comparing against classical baselines and a predictive variable-impedance comparator. Eight controllers appear in the tables below; **DI-MPC** denotes the proposed double-integrator MPC realization, and **MPVIC** is the predictive variable-impedance comparator, in the class of [8], [14]. MPVIC rolls out the same horizon and uses the same Kalman estimate $\hat d$ as the proposed controller, but only to *schedule* the apparent stiffness $K^\star\in\{200,\ldots,3000\}$ N/m rather than to cancel the force — isolating variable-impedance adaptation from offset-free augmentation. All controllers use $K_d = 300$ N/m nominal, critically damped, with the inner feedforward loop running at 1 kHz for every variant. Tables report every controller; plots show four representative curves (D1–D3, D7) for readability.

| ID | Controller | QP rate | Disturbance estimator |
|---|---|---|---|
| C1 | Classical Impedance | — | — |
| C2 | Admittance ($K_a = 100$ N/m) | — | Virtual spring |
| C3 | PI Impedance ($K_\text{int} = 80$ N/ms) | — | Integral |
| MPVIC | Variable-Impedance MPC (predictive) | 100 Hz | Kalman (stiffness scheduling only) |
| C4 | DI-MPC | 100 Hz | None |
| C5 | DI-MPC + Kalman | 100 Hz | Augmented Kalman |
| C6 | DI-MPC | 500 Hz | None |
| C7 | DI-MPC + Kalman | 500 Hz | Augmented Kalman |

**Benchmark I — circular trajectory under step force.** The end-effector tracks a 12 cm radius circle in the XZ sagittal plane ($\omega = 2\pi/8$ rad/s) while a 15 N step force $F_h = [0, 0, -15]$ N is applied for 3 s of every 8 s cycle (3 cycles, 24 s total); metrics are averaged over the three force events. Table I reports the full C1–C7 ablation; the corresponding figure plots D1–D3 and D7 against the MPVIC baseline.

*Table I — Benchmark I: Circular Trajectory Under 15 N Step Force*

| Controller | RMS total (mm) | RMS contact (mm) | Peak defl. (mm) | SS error (mm) |
|---|:---:|:---:|:---:|:---:|
| C1 — Impedance | 35.6 | 41.1 | 51.8 | 44.8 |
| C2 — Admittance | 113.9 | 174.7 | 210.5 | 186.7 |
| C3 — PI Impedance | 36.2 | 27.4 | 43.7 | 21.4 |
| MPVIC — Var.-Imp. MPC (pred.) | 12.9 | 4.5 | 7.5 | 4.8 |
| C4 — MPC 100 Hz | **11.4** | 2.2 | 2.9 | 2.8 |
| C5 — MPC+Kalman 100 Hz | **11.4** | 0.5 | 2.6 | **<0.05** |
| C6 — MPC 500 Hz | 12.8 | 0.8 | 1.1 | 1.1 |
| C7 — MPC+Kalman 500 Hz | 12.6 | **0.2** | **0.8** | **<0.05** |

The comparison isolates five mechanisms:

- **Kalman augmentation removes steady-state error (C4 vs. C5).** With QP rate and horizon held fixed, folding $\hat d$ into the free response drives steady-state error below 0.05 mm — down from 2.8 mm over C4 and 44.8 mm over C1. This is the cleanest isolated comparison here: the only thing that changes between C4 and C5 is whether the disturbance estimate is used, so the improvement is attributable to offset-free cancellation itself, not a confounded gain change.
- **A large early gain, but gain-confounded (C1 vs. C4).** With no disturbance model at all, C4's QP still cuts contact-window RMS from 41.1 to 2.2 mm (19×) and peak deflection from 51.8 to 2.9 mm. However, C1's nominal $K_d=300$ N/m is a design choice, not C4's realized closed-loop gain (Remark 2), so unlike the C4-vs-C5 comparison above, this one does not separate how much of the improvement is predictive look-ahead versus a stiffer realized gain at the same horizon; a matched-gain, horizon-length ablation is left to future work.
- **Adapting stiffness alone is not enough (MPVIC vs. C5).** MPVIC uses the *same* $\hat d$ as C5 but only to schedule stiffness, cutting contact RMS to 4.5 mm — better than every reactive baseline — yet it retains a 4.8 mm steady-state error, since a finite apparent stiffness can only bound deflection at $e_\infty=-K^{\star-1}\hat d\ne0$. Offset-free cancellation removes this residual entirely (0.03 mm, $\sim$140$\times$): the concrete separation between the proposed controller and the predictive variable-impedance methods [8], [14] it generalizes. MPVIC here is our own discrete-stiffness scheduler in that family, not a reproduction of a specific published algorithm, and its candidate stiffness set and force penalty have not been swept for sensitivity; the comparison shows one representative instance of the class, not an upper bound on it.
- **Rate and estimation are largely orthogonal (C4–C7).** Raising the QP rate from 100 to 500 Hz mainly shrinks the zero-order-hold transient (peak 2.9→1.1 mm without Kalman); adding the Kalman estimator mainly removes the steady-state error. C7 combines both and gives the lowest contact-window RMS, peak deflection, and steady-state error of any controller, though its total RMS (12.6 mm, averaged over the whole cycle including the force-free majority of it) is marginally higher than C4/C5's (11.4 mm) — the faster QP rate sharpens the contact transient without improving free-motion tracking, so it is not simply better on every metric.
- **Admittance yields by design (C2).** Its 174.7 mm contact-window RMS matches the equilibrium prediction $15/100=150$ mm — the expected behavior of intentional compliance, not a controller failure.

A second free-space benchmark — tracking the same circle across four 3-D planes — and a third dynamic benchmark — a three-waypoint reach-and-hold task under directionally varied pushes — reproduce the same separation and are reported in the supplementary material (Tables SII, SIII) rather than here.

**Joint-limit safety (boundary test).** A 20 cm radius circle pushes the arm toward its workspace boundary; Table II reports the minimum fractional joint margin (0 = at limit, 0.5 = at range center) and RMSE.

This test exercises the soft null-space barrier and workspace projection only, not the final one-step CBF safety filter (Theorem 4) — no CBF residual or slack is logged here, so the result below is empirical joint-limit avoidance by the dual soft barrier, not a demonstration of the theorem's sampled-data invariance certificate.

*Table II — Boundary Test (R = 20 cm)*

| Controller | RMSE (mm) | Min joint margin | Result |
|---|:---:|:---:|:---:|
| IMP | 26.97 | 0.049 | ✗ LIMIT VIOLATION |
| MPC (C5, 100 Hz QP) | 12.35 | 0.084 | ✓ AVOIDED |
| MPC (C5, 30 Hz QP) | 11.21 | 0.083 | ✓ AVOIDED |

- Classical impedance violates the 5% margin; the dual-barrier MPC avoids it, holding a 0.083–0.084 margin at both 100 Hz and 30 Hz — joint-limit avoidance is insensitive to QP rate.
- The elevated MPC RMSE (12.4 mm vs. 0.3 mm at R = 12 cm) reflects the workspace projection (16) deliberately trading tracking accuracy for clearance near the boundary.

**Robustness to position-measurement noise and model mismatch.** The architecture depends on a Kalman estimator, sensitive to measurement noise, and on Layer-1 cancellation, sensitive to model error, so we stress-test each independently: the plant is always the true MuJoCo model, and only the controller's *inputs* (added EE-position noise) or its *model* (the feedforward inertia, scaled up by 0 to 30% relative to the true plant) are perturbed. Velocity noise, sensing latency, and force-sensor noise are not modeled here.

*Table III — Robustness of MPC + Kalman (100 Hz QP)*

| EE position noise (1σ) | SS error (mm) | Peak (mm) |
|---|:---:|:---:|
| 0 mm | 0.08 | 1.6 |
| 1 mm | 0.19 ± 0.03 | 1.6 |
| 2 mm | 0.36 ± 0.04 | 1.6 |
| 5 mm | 0.89 ± 0.07 | 2.2 |

| Inertia mismatch | RMSE, Kalman on (mm) | RMSE, Kalman off (mm) |
|---|:---:|:---:|
| 0 % | 0.24 | 0.96 |
| +10 % | 0.24 | 0.87 |
| +20 % | 0.23 | 0.80 |
| +30 % | 0.23 | 0.74 |

- Under a 10 N step disturbance (top table; steady-state error as mean $|e|$ over the last 0.5 s of the push, mean ± std over 5 measurement-noise seeds), even an unrealistically large 5 mm (1σ) position noise leaves steady-state error below 1 mm, with the transient peak rising only modestly (1.6→2.2 mm) — the integrating Kalman state averages out zero-mean sensor noise.
- Under free-space XZ-circle tracking with a deliberately wrong feedforward inertia (bottom table), Kalman-augmented RMSE stays essentially invariant to up to 30% inertia error (0.24→0.23 mm), since the resulting feedforward force error is slowly varying and is absorbed by $\hat d$ — the empirical counterpart of the exactness remark (§III). Disabling the Kalman exposes the mismatch (0.74–0.96 mm, 3–4× worse), confirming that it is the disturbance state, not the nominal model, that confers the robustness.

**Further experiments and parameters.** The supplementary material reports three further ablations run under the same codebase: a time-varying-force horizon-prediction study (Table SIV), a human-force magnitude and shape sweep (Table SV), and an impedance-backbone ablation, **C8**, that degrades gracefully rather than collapsing to bare feedforward when the QP's corrective authority is curtailed (Table SVI) — along with the full numeric parameter table (Table SI).

---

## VII. Discussion and Conclusion

This paper argued that safe pHRI is best designed by predicting *interaction dynamics* directly (Definition 1): operational-space feedforward cancellation exposes a linear interaction-dynamics backbone — a double-integrator reduction of the interaction error — with configuration dependence confined to the input matrix $B_d(\rho_k)$, turning compliance, tracking, disturbance rejection, and safety into properties of one 30-variable convex QP. Within its scope the controller recovers classical impedance in the unconstrained, disturbance-free limit (Theorem 1), is offset-free under constant bounded force (Theorem 2), and is quadratically stabilizable by one fixed feedback gain over a certified polytope of configurations (Theorem 3). On a Franka FR3 in MuJoCo it reduces steady-state error under a sustained 15 N force by >800× over classical impedance, cuts free-space circle RMSE by 77–97×, keeps joints inside their limits at the workspace boundary, and is robust to position-measurement noise and up to 30% inertial mismatch; the supplementary material adds a second waypoint-hold benchmark, a disturbance-rate sweep, and an impedance-backbone ablation (§S8) that degrades gracefully rather than collapsing when the optimizer's corrective authority is curtailed.

The framework is one realization, not a replacement for all pHRI modes: for intentional co-manipulation, admittance control remains the appropriate paradigm, and near the workspace boundary joint-limit regulation needs the task-space projection alongside the null-space barrier. Because the interface needs only $M$, $C\dot{q}+G$, and $J_v$, it ports across torque-controlled arms by substituting the robot model and joint-limit parameters; and because it reduces contact to an analytic linear model with a fixed transition matrix, it may serve as a dynamics prior for model-based physical AI, letting a learned model capture only the residual uncertainty of intent, contact, and environment. The next validation step is real FR3 pHRI testing with synchronized tracking, force-estimation, CBF-residual, and energy-tank logs.

---

## References

[1] N. Hogan, "Impedance control: An approach to manipulation: Parts I–III," *ASME J. Dyn. Syst. Meas. Control*, vol. 107, no. 1, pp. 1–24, 1985.

[2] S. Chiaverini, B. Siciliano, and L. Villani, "A survey of robot interaction control schemes with experimental comparison," *IEEE/ASME Trans. Mechatronics*, vol. 4, no. 3, pp. 273–285, 1999.

[3] D. Q. Mayne, J. B. Rawlings, C. V. Rao, and P. O. M. Scokaert, "Constrained model predictive control: Stability and optimality," *Automatica*, vol. 36, no. 6, pp. 789–814, 2000.

[4] G. Pannocchia and J. B. Rawlings, "Disturbance models for offset-free model-predictive control," *AIChE Journal*, vol. 49, no. 2, pp. 426–437, 2003.

[5] R. Cao, L. Cheng, and H. Li, "Passive model-predictive impedance control for safe physical human–robot interaction," *IEEE Trans. Cognitive Developmental Syst.*, vol. 16, no. 2, pp. 426–435, Apr. 2024, doi: 10.1109/TCDS.2023.3275217.

[6] K. Haninger, C. Hegeler, and L. Peternel, "Model predictive impedance control with Gaussian processes for human and environment interaction," *Robotics Autonomous Syst.*, vol. 165, p. 104431, 2023.

[7] G. Wang, Z. Lin, F. Min, D. Li, and N. Liu, "Ensuring safe physical HRI: Integrated MPC and ADRC for interaction control," *Actuators*, vol. 14, no. 12, p. 608, 2025.

[8] J. Xue, W. Liang, Y. Wu, and T. H. Lee, "Model predictive variable impedance control towards safe robotic interaction in unknown disturbance-rich environments," *Robotics Autonomous Syst.*, vol. 189, p. 104961, 2025.

[9] B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd, "OSQP: An operator splitting solver for quadratic programs," *Math. Program. Comput.*, vol. 12, no. 4, pp. 637–672, 2020.

[10] Google DeepMind, "MuJoCo Menagerie: A collection of physics-based simulation models," GitHub, 2022. [Online]. Available: https://github.com/google-deepmind/mujoco_menagerie [Accessed: Jun. 2026].

[11] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," in *Proc. IEEE/RSJ IROS*, 2012, pp. 5026–5033.

[12] L. Sentis and O. Khatib, "Synthesis of whole-body behaviors through hierarchical control of behavioral primitives," *Int. J. Humanoid Robotics*, vol. 2, no. 4, pp. 505–518, 2005.

[13] O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation," *IEEE J. Robotics Autom.*, vol. 3, no. 1, pp. 43–53, 1987.

[14] L. Roveda, N. Iannacci, F. Vicentini, N. Pedrocchi, F. Braghin, and L. M. Tosatti, "Optimal impedance via model predictive control for robot-aided rehabilitation," *Control Eng. Practice*, vol. 83, pp. 11–23, 2019.

[15] H. Sadeghian, L. Villani, M. Keshmiri, and B. Siciliano, "Task-space control of robot manipulators with null-space compliance," *IEEE Trans. Robot.*, vol. 30, no. 2, pp. 493–506, 2014.

[16] Franka Robotics GmbH, "Franka Research 3 (FR3) Technical Documentation," 2023. [Online]. Available: https://frankaemika.github.io/docs [Accessed: Jun. 2026].

[17] L. Villani and J. De Schutter, "Force control," in *Springer Handbook of Robotics*, B. Siciliano and O. Khatib, Eds., 2nd ed., Springer, 2016, pp. 195–220.

[18] A. Albu-Schäffer, C. Ott, and G. Hirzinger, "A unified passivity-based control framework for position, torque and impedance control of flexible joint robots," *Int. J. Robot. Res.*, vol. 26, no. 1, pp. 23–39, 2007.

[19] S. Haddadin, A. Albu-Schäffer, and G. Hirzinger, "Requirements for safe robots: Measurements, analysis and new insights," *Int. J. Robot. Res.*, vol. 28, no. 11–12, pp. 1507–1527, 2009.

[20] R. Ikeura and H. Inooka, "Variable impedance control of a robot for cooperation with a human," in *Proc. IEEE ICRA*, 1995, pp. 3097–3102.

[21] D. Q. Mayne, "Model predictive control: Recent developments and future promise," *Automatica*, vol. 50, no. 12, pp. 2967–2986, 2014.

[22] Y.-Y. Cao, Z. Lin, and D. G. Ward, "Anti-windup design of output tracking systems subject to actuator saturation and constant disturbances," *Automatica*, vol. 40, no. 7, pp. 1221–1228, Jul. 2004.

[23] Y.-Y. Cao, Z. Lin, and D. G. Ward, "An antiwindup approach to enlarging domain of attraction for linear systems subject to actuator saturation," *IEEE Trans. Autom. Control*, vol. 47, no. 1, pp. 140–145, Jan. 2002.

[24] Y.-Y. Cao and Z. Lin, "Min–max MPC algorithm for LPV systems subject to input saturation," *IEE Proc. Control Theory Appl.*, vol. 152, no. 3, pp. 266–272, May 2005.

[25] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*. Springer, 2009.
