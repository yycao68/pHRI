# Predictive Saturation Management through a Behavior--Realization Interface

## Working Paper Draft

This manuscript narrows an earlier universal-predictor framing to conditional certificate transfer through behavior--realization separation.

### Abstract

High-rate robot controllers such as proportional--derivative control, impedance control, reinforcement-learning policies, neural-network policies, and AI-generated behavior policies can all produce satisfactory nominal motion while still losing their intended closed-loop dynamics when actuator saturation occurs. Direct clipping is computationally cheap, but it intervenes only after the requested command has become unrealizable and hides the resulting change in behavior.

This paper proposes a two-rate architecture in which a nominal controller exposing a task-acceleration request remains in a \(1~\mathrm{kHz}\) fast loop, while a slower model-predictive realization manager forecasts loss of actuator authority and modifies the requested dynamics before saturation. The fast controller is not replaced by MPC. Instead, it supplies a nominal task-acceleration request and, when available, a preview model; the predictive layer seeks a realizable acceleration sequence close to that nominal preview, with a reactive fallback when the horizon problem is infeasible. The robot-dependent realization map converts actuator boxes into configuration-dependent admissible task-acceleration sets, exposing directional authority loss.

The universal double-integrator prediction backbone is treated as a convenient interface rather than as the principal novelty. The central contribution is a certificate-transfer result: a feasibility, stability, or dissipativity certificate established in the abstract behavior coordinates transfers to a physical robot when its realization map satisfies an explicit refinement condition and the prediction error lies inside the certificate's robustness margin. A concrete, deliberately simple certified action set is now enforced inside the QP itself, so a feasible solution satisfies both the actuator-feasibility and certificate-membership conditions by construction rather than by a check applied after the fact; the reported experiments additionally audit the realization-side conditions along sampled trajectories. This separates reusable behavior-level reasoning from robot-specific actuator geometry while retaining the latter where it is physically unavoidable.

A deterministic suite of 111 configurations evaluates five controller interfaces, three robot-specific realization surrogates, eight stress scenarios (one reserved for the horizon-constraint ablation, plus a ninth dedicated scenario reserved for the certified-action-set ablation), and eight ablation families. Horizon-wide constraints remove a \(3.587~\mathrm{Nm}\) planned future violation missed by a first-step check, while a common \(0.03~\mathrm{m/s}\) empirical budget contains the sampled successor defects of all three realization maps. A dedicated ablation confirms the certified action set changes the manager's output when it binds (holding peak speed at \(0.580~\mathrm{m/s}\) versus \(0.597~\mathrm{m/s}\) unconstrained), while leaving the eight main stress scenarios unaffected, consistent with their own comfortable certificate margin. This sampled audit does not by itself establish a workspace-wide invariance proof. Severe disturbance and model/preview mismatch explicitly fail the sampled refinement checks.

---

# 1. Introduction

Modern robot software increasingly combines two very different time scales.

At the fast time scale, a controller must react to sensors and produce an actuator command with low latency. Depending on the application, this controller may be:

- a joint-space PD controller;
- a Cartesian impedance or admittance controller;
- an operational-space or whole-body controller;
- a reinforcement-learning policy;
- a neural-network policy;
- a learned residual controller; or
- an AI-generated behavior policy executed through a lower-level servo.

At the slower time scale, the system can predict future motion and reason about coupled constraints. Model predictive control is useful at this layer, but replacing every fast controller with a different MPC formulation would destroy the modularity that makes high-rate controllers attractive.

The practical failure considered in this paper is actuator saturation. Let a nominal controller request

\[
\tau_k^{0}=\pi_\theta(s_k),
\]

where \(s_k\) is the controller observation and \(\pi_\theta\) may be analytic, optimized, or learned. The conventional implementation applies

\[
\tau_k=\operatorname{clip}\!\left(\tau_k^{0},
\tau_{\min},\tau_{\max}\right).
\]

Although clipping protects the actuator command numerically, it silently changes the realized acceleration and therefore the closed-loop behavior. Once clipping occurs, the robot is no longer executing the PD, impedance, RL, or neural policy that was designed or trained.

This paper studies a different architecture:

```text
                 nominal behavior / command
  PD, impedance, RL, neural, or AI policy at 1 kHz
                          |
                          v
              Predictive realization manager
                    at 50--200 Hz
          forecast saturation and compute correction
                          |
                          v
        fast robot-dependent realization map at 1 kHz
                          |
                          v
                       actuators
```

The predictive layer does not decide the task and does not replace the fast controller. Its responsibility is narrower:

> Preserve as much of the nominal controller's requested dynamics as possible while preventing a predicted loss of realizability.

The paper deliberately concedes that feedback linearization to a double-integrator-like model is classical. The scientific question is not whether different robots can be written locally as double integrators. It is whether a certificate designed at that separated behavior interface can be transferred to different robots through explicit, verifiable conditions on their realization maps.

## 1.1 Research question

The main research question is:

> Under what conditions can one behavior-level feasibility, stability, or passivity certificate be reused across different fast controllers and different robots, despite configuration-dependent actuator saturation?

## 1.2 Contributions

The intended contributions are:

1. A two-rate architecture that preserves an existing \(1~\mathrm{kHz}\) nominal controller and uses slower MPC only for anticipatory saturation correction.
2. A controller-agnostic nominal-command contract covering analytic and learned fast controllers, together with explicit preview assumptions.
3. A robot-specific realizability map that represents actuator-limited task acceleration as a configuration-dependent zonotope.
4. A directional-authority metric and a near-boundary braking stress case that distinguish anticipatory intervention from saturation that has already occurred, without claiming a computed viability boundary.
5. A certificate-transfer theorem connecting an abstract behavior-level certificate to a physical robot through a robust refinement condition.
6. Reproducible comparisons across PD, impedance, RL, and neural nominal controllers under identical actuator and disturbance conditions.

---

# 2. Positioning and Scope

## 2.1 What is not claimed as new

For a fully actuated robot, computed-torque or operational-space feedback linearization can produce a local model of the form

\[
\ddot e = u+d.
\]

This representation is classical. Calling it a universal predictor does not by itself create a sufficient research contribution. Predictive safety filters, reference governors, and robot-specific MPC controllers can also adopt such coordinates when their assumptions permit.

Accordingly, this paper does **not** claim:

- that double-integrator prediction is new;
- that feedback linearization is new;
- that separating a nominal controller from a constraint filter is new in the abstract;
- that a single configuration-independent feasible set describes every robot; or
- that the proposed predictive manager dominates all safety filters.

## 2.2 What is claimed

The reusable object is not the robot's physical feasible set. That set remains robot- and configuration-dependent.

The reusable object is:

1. a behavior-coordinate interface;
2. a certificate stated at that interface; and
3. a transfer rule specifying when a robot-specific realization map preserves the certificate.

This distinction is essential. The easy part,

\[
\ddot e=u+d,
\]

is shared. The difficult part,

\[
u\in\mathcal A_r(q,\dot q),
\]

depends on the robot \(r\), its configuration, its bias torque, and its actuator limits. Separation makes it possible to retain this physical dependence without re-deriving the behavior-level certificate from the beginning for every robot.

## 2.3 Relation to neighboring architectures

### Reference and command governors

A reference governor modifies the command supplied to a pre-stabilized inner loop so that predicted constraints are satisfied. This is architecturally close to the proposed method. The difference is that the exchanged object here is a requested local dynamics or acceleration, and the paper asks whether a certificate on that object transfers through different robot realization maps.

### Predictive safety filters

A predictive safety filter minimally modifies an unsafe nominal command and uses a model plus a safe terminal condition to establish recursive feasibility. The proposed MPC can be implemented as a predictive safety filter. The distinct research claim is therefore not the existence of another filter, but the cross-controller and cross-robot certificate-transfer interface.

### Control barrier functions

A CBF-QP usually enforces an instantaneous or short-horizon invariance inequality. It is a natural high-rate final safety layer and may remain downstream of the predictive manager. The proposed manager instead focuses on anticipatory loss of actuator authority and on avoiding trajectories from which the fast controller can no longer prevent saturation.

### Anti-windup control

Anti-windup compensators reduce performance degradation after saturation or account for saturation in closed-loop design. The proposed layer is complementary: it predicts an impending realizability loss and changes the requested dynamics before hard clipping is required.

### Robot-specific MPC

A robot-specific MPC can jointly optimize the nonlinear robot dynamics and all constraints. That formulation may be less conservative and more accurate, but it must be redesigned or reparameterized with the robot. The proposed architecture trades some optimality for a reusable behavior-level problem plus a robot-specific realization contract.

### Approximate simulation and assume--guarantee contracts

The transfer question belongs to the established theory of approximate simulation. Girard and Pappas formalize quantitative relations between an abstract system and a concrete system, and show how an interface can refine an abstract controller while bounding concrete behavior. Contract-based control similarly separates assumptions made about a component from the guarantees it must deliver. The present work does not claim this abstraction/refinement logic as new. Its narrower contribution is to make actuator-limited realization the interface obstruction and to derive a checkable refinement margin from the configuration-dependent acceleration zonotope, prediction defect, and saturation geometry.

---

# 3. System and Fast-Controller Contract

## 3.1 Robot dynamics

Consider a torque-controlled robot

\[
M_r(q)\ddot q+h_r(q,\dot q)
=\tau+J_r(q)^\top F_h,
\]

where \(M_r(q)\succ0\), \(h_r(q,\dot q)\) contains Coriolis, centrifugal, gravity, and modeled friction terms, \(\tau\) is actuator torque, and \(F_h\) is an interaction wrench.

The actuator box is

\[
\mathcal T_r
=
\left\{
\tau\in\mathbb R^{n_r}
\;\middle|\;
\tau_{\min,r}\le\tau\le\tau_{\max,r}
\right\}.
\]

Torque-rate limits may also be imposed:

\[
\left|\tau_k-\tau_{k-1}\right|
\le \dot\tau_{\max,r}T_f,
\]

where \(T_f=1~\mathrm{ms}\) is the fast-loop sample time.

## 3.2 Controller-agnostic nominal-command interface

Every fast controller is wrapped by the same interface, and it must expose a task-acceleration request, not an arbitrary torque:

\[
\mathcal I_\theta:
(s_k,\xi_k)
\mapsto
\left(a_{\mathrm{req},k}^0,\mathcal P_{\theta,k},\Sigma_{\theta,k}\right).
\]

Here:

- \(a_{\mathrm{req},k}^0\) is the nominal task-acceleration requested at the current \(1~\mathrm{kHz}\) sample;
- \(\mathcal P_{\theta,k}\) is an optional preview operator that predicts future nominal requests along a candidate state rollout; and
- \(\Sigma_{\theta,k}\) bounds preview uncertainty or model error.

The nominal pre-projection torque is then *defined*, not assumed, as the robot's own realization of that request (Section 4.2),
\[
\tau_k^0=\tau_{\mathrm{base},r}(x_k)+H_r(x_k)a_{\mathrm{req},k}^0,
\]
so the fast-loop correction of Section 6.4 is well defined by construction. A controller that instead outputs torque directly is outside this interface unless it exposes the decomposition \(\tau_k^0=\tau_{\mathrm{base},r}(x_k)+\tau_{\perp,k}^0+H_r(x_k)a_{\mathrm{req},k}^0\), where \(\tau_{\perp,k}^0\) is a policy-dependent secondary-torque term reported alongside \(\tau_{\mathrm{base},r}(x_k)\), not merged into it: \(\tau_{\mathrm{base},r}\) remains the purely robot-dependent quantity used throughout, and testing such a controller's realizability against \(\mathcal A_r^{\mathrm{tight}}(x)\) below requires subtracting \(\tau_{\perp,k}^0\) from the available actuator budget in addition to \(\tau_{\mathrm{base},r}(x_k)\).

The current request \(a_{\mathrm{req},k}^0\) is mandatory. Preview is not assumed to be exact. If the controller has no reliable preview model, the manager may use a zero-order hold, a local linearization, an ensemble bound, or a learned predictor, but the resulting uncertainty must appear explicitly in \(\Sigma_{\theta,k}\).

This prevents an unjustified universality claim: an opaque policy with no current request, no query access, and no bounded preview error is outside the theorem.

## 3.3 Examples of the interface

All five interfaces below expose \(a_{\mathrm{req}}^0\) directly and need no torque decomposition; secondary torque (gravity, orientation, null-space) belongs to \(\tau_{\mathrm{base},r}(x)\) in the realization map, not to the controller's own request.

### PD controller

\[
a^0
=
K_p(q_d-q)+K_d(\dot q_d-\dot q).
\]

Its preview operator evaluates the same analytic law on predicted states and reference samples.

### Cartesian impedance controller

\[
a^0
=
\left(
-K_xe-D_x\dot e+F_{\mathrm{ff}}
\right)/M_d.
\]

Its preview is analytic when the desired pose and interaction-force forecast are available.

### Reinforcement-learning policy

\[
a^0=\pi_{\mathrm{RL}}(o_k).
\]

Preview may query the policy on predicted observations. For stochastic policies, the manager uses a confidence set or bounded disturbance model rather than a single deterministic rollout.

### Neural-network controller

\[
a^0=\pi_{\mathrm{NN}}(s_k;\theta).
\]

Preview can use direct network evaluation, local Jacobian bounds, interval bounds, or an ensemble. A point prediction without a validated error bound provides empirical forecasting, not a transferable certificate.

### AI-based behavior controller

A language model, diffusion policy, or world-model planner should not normally close a raw \(1~\mathrm{kHz}\) loop. It may provide a behavior command \(b_k\) to a fast policy that itself exposes the required interface,

\[
b_k=\pi_{\mathrm{AI}}(\mathcal C_k),
\qquad
a^0_k=\pi_{\mathrm{fast}}(s_k,b_k).
\]

Only the fast execution contract and its bounded preview enter the analysis; the semantics of the upstream AI output are outside the certificate.

---

# 4. Behavior Coordinates and Physical Realization

## 4.1 Behavior coordinate

Let the controlled task error be

\[
z=
\begin{bmatrix}
e\\
\dot e
\end{bmatrix}.
\]

Over a local operating region, feedback linearization gives

\[
\ddot e = a_{\mathrm{req}}+w_r(x),
\]

where \(a_{\mathrm{req}}\in\mathbb R^{d}\) is the complete requested task acceleration --- already including whatever response to the interaction wrench its issuing controller or the realization map of Section 4.2 provides, not a quantity to which a further interaction-force term must be added --- and \(w_r\) is the residual left over after that response: realization error, discretization, unmodeled coupling, state-estimation error, and intra-period command drift, but not a second, separate accounting of the interaction force.

\(w_r\) is therefore not a matched-disturbance specialization of arbitrary model error or contact; it is the residual after force has already been accounted for once, upstream. Section 7.2 bounds it empirically as the successor defect \(\mathcal D_{z,r}\) rather than absorbing it silently into the nominal prediction.

With slow sample time \(\Delta t=mT_f\), the general abstract predictor is

\[
z_{\ell+1}
=
Az_\ell+Ba_{\mathrm{req},\ell}+w_\ell,
\qquad
w_\ell\in\mathcal W_\ell,
\]

\[
A=
\begin{bmatrix}
I&\Delta t I\\
0&I
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
\frac{1}{2}(\Delta t)^2I\\
\Delta t I
\end{bmatrix}.
\]

This prediction backbone is intentionally not presented as the novelty. The implemented QP of Section 6.2 uses the nominal case \(w_\ell=0\); Section 7.2 bounds the resulting successor mismatch empirically as \(\mathcal D_{z,r}\), and Theorem 1's condition 3 requires that bound to lie inside the certificate margin \(\mathcal E_\star\), so the nominal-model choice is checked rather than assumed away.

## 4.2 Robot-dependent realization map

Let \(\tau_{\mathrm{base},r}(x)\) contain bias compensation and fast secondary terms. A task-acceleration request \(a_{\mathrm{req}}\) is realized locally through

\[
\tau
=
\tau_{\mathrm{base},r}(x)
+H_r(x)a_{\mathrm{req}},
\]

where, for a conventional operational-space realization,

\[
H_r(x)=J_r(q)^\top\Lambda_r(q),
\qquad
\Lambda_r(q)
=
\left(J_rM_r^{-1}J_r^\top\right)^{-1}.
\]

Because the robot dynamics of Section 3.1 already include \(J_r(q)^\top F_h\), the actuator only supplies the part of the task force not already delivered by the interaction wrench; \(\tau_{\mathrm{base},r}(x)\) and the realization map are evaluated net of this feedforward term, and \(\hat\tau_r\), \(\tau_r^{\mathrm{pre}}\) (Section 6.3) depend on the current or forecast \(F_h\) as well as on \((x,a_{\mathrm{req}})\), suppressed notationally below.

All actuator-consuming secondary commands, including orientation and
null-space torques, must be included in
\(\tau_{\mathrm{base},r}(x)\). Omitting them makes
\(\mathcal A_r(x)\) optimistic because those channels consume the same actuator
margin available to \(H_r(x)a_{\mathrm{req}}\).

The admissible behavior request set is therefore

\[
\mathcal A_r(x)
=
\left\{
a_{\mathrm{req}}\in\mathbb R^d
\;\middle|\;
\tau_{\min,r}
\le
\tau_{\mathrm{base},r}(x)+H_r(x)a_{\mathrm{req}}
\le
\tau_{\max,r}
\right\}.
\]

The behavior dynamics may be written in common coordinates, but \(\mathcal A_r(x)\) cannot be made robot-independent without discarding the actuator geometry that determines saturation.

## 4.3 Equivalent acceleration zonotope

Alternatively, write the local task acceleration generated by torque as

\[
a=b_r(x)+G_r(x)\tau,
\]

with

\[
G_r(x)=J_r(q)M_r(q)^{-1}
\]

when the \(\dot J\dot q\), bias, and external-force terms are collected in \(b_r(x)\).

Let

\[
\tau_c=\frac{\tau_{\max,r}+\tau_{\min,r}}{2},
\qquad
\Delta\tau=\frac{\tau_{\max,r}-\tau_{\min,r}}{2}.
\]

Then the admissible acceleration set is the zonotope

\[
\mathcal Z_r(x)
=
b_r(x)+G_r(x)\tau_c
+G_r(x)\operatorname{diag}(\Delta\tau)[-1,1]^{n_r}.
\]

For generators in general position in a \(d\)-dimensional task, a zonotope generated by \(n_r\) actuator directions has at most

\[
2\binom{n_r}{d-1}
\]

facets. This structure enables support-function and directional-authority calculations without enumerating every actuator-box vertex.

---

# 5. Predictive Saturation Metrics

## 5.1 Saturation margin

For a predicted torque \(\tau_{i|\ell}\), define the normalized actuator margin, centered so that \(\mu=1\) at \(\tau_c\) and \(\mu=0\) at either bound,

\[
\mu_{i|\ell}
=
\min_j
\left\{
\frac{\tau_{\max,j}-\tau_{j,i|\ell}}
{\tau_{\max,j}-\tau_{c,j}},
\frac{\tau_{j,i|\ell}-\tau_{\min,j}}
{\tau_{c,j}-\tau_{\min,j}}
\right\},
\qquad
\tau_{c,j}=\frac{\tau_{\max,j}+\tau_{\min,j}}{2},
\]

which for symmetric limits reduces to \(\mu_{i|\ell}=1-\max_j|\tau_{j,i|\ell}|/\tau_{\max,j}\). Then:

- \(\mu_{i|\ell}>0\): the command is inside the actuator box;
- \(\mu_{i|\ell}=0\): at least one actuator lies on its limit;
- \(\mu_{i|\ell}<0\): the nominal predicted command is unrealizable.

This scalar is a diagnostic only and is not computed in the experiments of Section 9; the directional measure below is the one used there.

The horizon margin is

\[
\mu_\ell^{\min}
=
\min_{i=0,\ldots,N-1}\mu_{i|\ell}.
\]

## 5.2 Directional authority

Given a current feasible task acceleration \(a_c\) and a unit direction \(d\), define the remaining positive authority against the tightened realizable set \(\mathcal A_r^{\mathrm{tight}}(x)\) of Section 6.3, not the untightened zonotope \(\mathcal Z_r(x)\) of Section 4.3, so the metric accounts for secondary-torque consumption and uncertainty exactly as the optimizer does:

\[
\alpha_r^{+}(x,a_c,d)
=
\max_{\alpha\ge0}
\left\{
\alpha
\;\middle|\;
a_c+\alpha d\in\mathcal A_r^{\mathrm{tight}}(x)
\right\}.
\]

Similarly,

\[
\alpha_r^{-}(x,a_c,d)
=
\max_{\alpha\ge0}
\left\{
\alpha
\;\middle|\;
a_c-\alpha d\in\mathcal A_r^{\mathrm{tight}}(x)
\right\}.
\]

A controller may retain substantial overall torque margin while losing authority in the direction required to reject an interaction force. Directional authority therefore reveals a failure that scalar torque utilization can hide.

Because \(\mathcal A_r^{\mathrm{tight}}(x)\) is itself an affine image of a box (the same construction as \(\mathcal Z_r\) in Section 4.3, once \(\tau_{\mathrm{base},r}\) and the tightening margin are subtracted from the torque box first), computing \(\alpha_r^{+}\) does not require facet enumeration or a general linear program: for a halfspace representation \(\{v:Av\le b\}\) of \(\mathcal A_r^{\mathrm{tight}}(x)\), the maximum feasible step along \(d\) from a feasible \(a_c\) is the closed form \(\alpha_r^{+}=\min_{j:\,(Ad)_j>0}(b_j-(Aa_c)_j)/(Ad)_j\), which is exactly the ray--halfspace intersection the implementation evaluates.

## 5.3 Finite-horizon feasibility diagnostic

The implemented manager reports whether its finite-horizon QP is feasible. This diagnostic answers whether the particular frozen-model problem has a constraint-satisfying sequence at the current update. It is not membership in a viability kernel: the implementation has no separately constructed terminal invariant set or certified backup policy. The corresponding experiment is therefore called the **near-boundary braking stress case**, not a point of no return.

---

# 6. Predictive Realization Manager

## 6.1 Two-rate operation

The fast loop runs every \(T_f=1~\mathrm{ms}\):

1. read the current robot state;
2. evaluate the nominal controller \(\tau_k^0=\pi_\theta(s_k)\);
3. interpolate or hold the latest predictive correction;
4. recompute the robot-dependent realization terms;
5. apply torque-rate limiting and a final emergency projection; and
6. send the resulting torque to the actuators.

The predictive manager runs every \(\Delta t\in[5,20]~\mathrm{ms}\):

1. roll out the nominal controller or its bounded preview;
2. predict actuator and state margins;
3. solve for the minimum required dynamics correction;
4. publish a correction sequence or correction policy to the fast loop; and
5. log the predicted intervention and remaining authority.

## 6.2 Optimization problem

Let \(a_{\mathrm{req},i|\ell}^0\) be the behavior acceleration induced by the fixed nominal rollout. The implemented manager selects the full acceleration sequence \(a_{\mathrm{req},i|\ell}\):

\[
\begin{aligned}
\min_{a_{\mathrm{req},0:N-1}}
\quad&
\sum_{i=0}^{N-1}
\left(
\|a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i|\ell}^0\|_{W_{a_{\mathrm{req}}}}^2
+\|a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i-1|\ell}\|_{W_{\Delta}}^2
\right)
\\
\text{s.t.}\quad&
z_{i+1|\ell}
=
Az_{i|\ell}
+Ba_{\mathrm{req},i|\ell},
\\
&
\tau_{i|\ell}
=
\hat\tau_{\mathrm{base},r}(\hat x^0_{i|\ell})
+\hat H_r(\hat x^0_{i|\ell})a_{\mathrm{req},i|\ell},
\\
&
\tau_{\min,r}+\varepsilon_\tau
\le
\tau_{i|\ell}
\le
\tau_{\max,r}-\varepsilon_\tau,
\\
&
z_{i|\ell}\in\mathcal X,
\qquad
a_{\mathrm{req},\min}\le a_{\mathrm{req},i|\ell}\le a_{\mathrm{req},\max},
\\
&
-\Delta a_{\mathrm{req},\max}
\le a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i-1|\ell}
\le \Delta a_{\mathrm{req},\max}.
\end{aligned}
\]

Here \(\hat x^0_{i|\ell}\) is fixed from the nominal rollout when the QP matrices are assembled. This is the nominal case \(w_{i|\ell}=0\) of Section 4.1's general predictor: the predicted position and speed constraints implicitly assume \(a_{\mathrm{req},i|\ell}\) is held fixed for the full \(\Delta t\) it labels. This is not violated by force per se: the realization map compensates for the currently measured \(F_h\) rather than a forecast one (Section 3.1), the fast loop recomputes its nominal request and cancels the interaction wrench at every \(1~\mathrm{kHz}\) sample regardless of controller type, and the observed successor-defect magnitudes are numerically similar across the five tested interfaces (Section 9.3) even under sudden disturbance --- confirmed directly against the reported metrics, though the study runs one deterministic trajectory per case and this is not a statistical claim. What the nominal-model choice misses is intra-step drift: if the fast controller's own request changes materially within one \(20~\mathrm{ms}\) manager period --- most acutely when an impulse arrives between manager updates --- the frozen \(a_{\mathrm{req},i|\ell}\) held throughout that step no longer matches what the fast loop actually applies, exactly the component of \(w_{i|\ell}\) the sampled \(\mathcal D_{z,r}\) check of Section 7.2 must contain. Constructing a tightened bound \(\mathcal W_{i|\ell}\) directly, rather than checking its empirical consequence after the fact, is left to future work (Section 15). The state, acceleration, rate, and tightened torque bounds are hard. The implementation contains neither slack variables nor a separately constructed terminal invariant set. Before solving, the manager checks whether the nominal rollout \(a_{\mathrm{req},0:N-1}^0\) already satisfies every constraint above; if so, it is returned unmodified rather than passed through the objective, so the rate-smoothing term cannot perturb an already-feasible request, giving the exact-pass-through property \(a_{\mathrm{req},0:N-1}^0\in\mathcal F_N\implies\Delta a_{\mathrm{req},\ell}=0\), where \(\mathcal F_N\) is the horizon-wide feasible set above. The QP is solved only when this check fails. The applied first correction is \(\Delta a_{\mathrm{req},\ell}=a_{\mathrm{req},0|\ell}-a_{\mathrm{req},0|\ell}^0\).

## 6.3 Robust tightening

Let the manager's predicted pre-saturation torque be

\[
\hat\tau_r(x,a_{\mathrm{req}})
=
\hat\tau_{\mathrm{base},r}(x)+\hat H_r(x)a_{\mathrm{req}}.
\]

Suppose interval analysis, Lipschitz bounds, reachability, or validated identification provides a componentwise torque-realization error set

\[
\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\subseteq
\left\{
\delta\tau:
|\delta\tau|\le\bar\delta_{\tau,r}(x,a_{\mathrm{req}})
\right\},
\]

such that the true torque required before clipping satisfies

\[
\tau_r^{\mathrm{pre}}(x,a_{\mathrm{req}})
\in
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}}).
\]

The robustly admissible behavior set is

\[
\mathcal A_r^{\mathrm{tight}}(x)
=
\left\{
a_{\mathrm{req}}:
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\subseteq\mathcal T_r
\right\}.
\]

For a box uncertainty bound, membership has the directly checkable form

\[
\tau_{\min,r}+\bar\delta_{\tau,r}(x,a_{\mathrm{req}})
\le
\hat\tau_r(x,a_{\mathrm{req}})
\le
\tau_{\max,r}-\bar\delta_{\tau,r}(x,a_{\mathrm{req}}).
\]

The one-step abstraction defect that remains when no clipping occurs is bounded separately by

\[
\Pi_r\!\left(
f_r^d(x,\tau_r^{\mathrm{pre}},F_h)
\right)
-F\!\left(\Pi_r(x),a_{\mathrm{req}}\right)
\in
\mathcal D_{z,r}(x,a_{\mathrm{req}},F_h).
\]

Unlike the earlier matched-disturbance notation, \(\mathcal D_{z,r}\) may contain unmatched force coupling, discretization error, interpolation error, and secondary-channel coupling. Without verified \(\mathcal D_{\tau,r}\) and \(\mathcal D_{z,r}\), the MPC provides nominal frozen-model plan feasibility only, not a transferable physical certificate.

## 6.4 What is applied at \(1~\mathrm{kHz}\)

The slow MPC correction is not a cached full torque. The fast loop applies

\[
\tau_k^{\mathrm{pre}}
=
\tau_k^0
+H_r(x_k)\Delta a_{\mathrm{req},k}^{\mathrm{MPC}}
=
\tau_{\mathrm{base},r}(x_k)+H_r(x_k)\left(a_{\mathrm{req},k}^0+\Delta a_{\mathrm{req},k}^{\mathrm{MPC}}\right),
\qquad
\tau_k^{\mathrm{app}}
=
\operatorname{proj}_{\mathcal T_r}\!\left(\tau_k^{\mathrm{pre}}\right),
\]

where \(H_r(x_k)\) is recomputed from the current state. Writing the high-rate emergency projection or CBF correction's effect as \(\Delta\tau_k^{\mathrm{proj}}=\tau_k^{\mathrm{app}}-\tau_k^{\mathrm{pre}}\), so that \(\tau_k^{\mathrm{app}}=\tau_k^{\mathrm{pre}}+\Delta\tau_k^{\mathrm{proj}}\), makes the no-clipping branch of Theorem 1 immediate: it is precisely the case \(\Delta\tau_k^{\mathrm{proj}}=0\).

This distinction matters. Holding the full torque from the slow loop would turn modeling drift between updates into an artificial disturbance and would no longer preserve a true \(1~\mathrm{kHz}\) nominal controller.

---

# 7. Certificate Transfer

## 7.1 Lineage: approximate simulation and control refinement

The abstract behavior model and physical robot are a concrete--abstract system pair in the sense of approximate simulation. In that literature, an interface refines an abstract input into a concrete input while a simulation relation or simulation function bounds the resulting output error. The result below is a robust-invariance specialization of that machinery. Its new object is not the logic of refinement itself, but a constructive actuator-saturation condition: the simulation defect is bounded by testing the requested behavior against a configuration-dependent, uncertainty-tightened realization set.

## 7.2 Abstract certificate and concrete defect decomposition

Let the abstract behavior dynamics be

\[
z_{\ell+1}=F(z_\ell,a_{\mathrm{req},\ell}).
\]

Let

\[
\mathcal S
=
\left\{
z:V(z)\le c
\right\}
\]

be a certified abstract safe or stable set.

Rather than designing the manager around a single abstract policy, define the *certified action set* at every \(z\in\mathcal S\),

\[
\mathcal K_{\mathrm{cert}}(z)
=
\left\{a: F(z,a)\oplus\mathcal E_\star\subseteq\mathcal S\right\}
\neq\emptyset,
\qquad
\forall z\in\mathcal S,
\]

so that \(\mathcal S\) is robustly invariant under any measurable selection \(\kappa\) with \(\kappa(z)\in\mathcal K_{\mathrm{cert}}(z)\) for all \(z\in\mathcal S\). The predictive manager, including its correction of the nominal fast controller, need only select some \(a_{\mathrm{req},\ell}\in\mathcal K_{\mathrm{cert}}(z_\ell)\); it is not required to reproduce a single robot-independent \(\kappa\), and different robots selecting different members of \(\mathcal K_{\mathrm{cert}}(z)\) is expected, not a failure of transfer. The robot-independent object is the set \(\mathcal K_{\mathrm{cert}}\).

For robot \(r\), define:

- \(x^r\) be the physical state;
- \(\Pi_r(x^r)=z\) be the abstraction map;
- \(f_r^d(x^r,\tau,F_h)\) be the one-step physical dynamics; and
- \(\hat\tau_r(x^r,a_{\mathrm{req}})\) be the predicted pre-saturation realization command.

The physical actuator applies

\[
\tau_r^{\mathrm{app}}
=
\operatorname{proj}_{\mathcal T_r}
\left(\tau_r^{\mathrm{pre}}\right),
\]

where componentwise clipping is the Euclidean projection for a torque box. Its nonlinear saturation residual is

\[
c_{\tau,r}
=
\tau_r^{\mathrm{app}}-\tau_r^{\mathrm{pre}}.
\]

Assume a verified local sensitivity set \(\mathcal L_r(x)c_{\tau,r}\) bounds the additional abstract successor error caused by this torque residual. The complete constructive defect bound is then

\[
\Pi_r\!\left(
f_r^d(x,\tau_r^{\mathrm{app}},F_h)
\right)
-F\!\left(\Pi_r(x),a_{\mathrm{req}}\right)
\in
\mathcal D_{z,r}(x,a_{\mathrm{req}},F_h)
\oplus
\mathcal L_r(x)c_{\tau,r}.
\]

Inside \(\mathcal A_r^{\mathrm{tight}}(x)\), every admissible true pre-command lies in \(\mathcal T_r\), so projection is the identity and \(c_{\tau,r}=0\). On or outside the untightened saturation boundary, the same formula remains valid but the clipping defect must be added to the abstract error budget.

## 7.3 Constructive theorem

**Theorem 1 (Realizability-margin condition for certificate transfer).**  
Let \(\mathcal K_{\mathrm{cert}}\) and \(\mathcal S\) be as in Section 7.2. For robot \(r\), consider a certified physical operating region \(\mathcal X_r\) such that \(\Pi_r(\mathcal X_r)\subseteq\mathcal S\). Suppose that, for every \(x\in\mathcal X_r\), every admissible \(F_h\), and every \(a_{\mathrm{req}}\in\mathcal K_{\mathrm{cert}}(\Pi_r(x))\) that the robot-specific manager may select:

1. the true pre-saturation torque obeys
   \[
   \tau_r^{\mathrm{pre}}(x,a_{\mathrm{req}})
   \in
   \hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}});
   \]
2. the manager enforces the realizability-margin condition
   \[
   \hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
   \subseteq\mathcal T_r,
   \]
   equivalently \(a_{\mathrm{req}}\in\mathcal A_r^{\mathrm{tight}}(x)\);
3. the no-clipping abstraction defect is constructively bounded by
   \[
   \mathcal D_{z,r}(x,a_{\mathrm{req}},F_h)\subseteq\mathcal E_\star;
   \]
4. interpolation, torque-rate limiting, secondary torques, and the final high-rate projection are included in \(\mathcal D_{\tau,r}\), \(\mathcal D_{z,r}\), or the definition of the physical realization; and
5. \(x_0\in\mathcal X_r\), with the physical successor remaining inside the operating region whenever its abstraction remains in \(\mathcal S\).

Then the actuator projection is inactive,

\[
c_{\tau,r}=0,
\]

the approximate refinement relation follows rather than being assumed,

\[
\Pi_r\!\left(
f_r^d(x,\tau_r^{\mathrm{app}},F_h)
\right)
\in
F\!\left(\Pi_r(x),a_{\mathrm{req}}\right)\oplus\mathcal E_\star
\subseteq\mathcal S,
\]

the last inclusion following directly from \(a_{\mathrm{req}}\in\mathcal K_{\mathrm{cert}}(\Pi_r(x))\), and therefore, if Condition 5's operating-region containment is verified for \(i=1,\ldots,N\) steps ahead,

\[
\Pi_r(x_{\ell+i})\in\mathcal S,
\qquad
i=0,\ldots,N,
\]

which is the finite-horizon guarantee that applies to the implemented manager, since \(\mathcal K_{\mathrm{cert}}\) is instantiated and enforced as a QP constraint (Remark below) rather than only checked after the fact; the experiments additionally audit Conditions 1--2 and the sampled successor defect, since \(\mathcal K_{\mathrm{cert}}\)-membership alone does not by itself verify Conditions 1--3 hold for the chosen \(a_{\mathrm{req}}\). Extending this to \(\Pi_r(x_k)\in\mathcal S\) for all \(k\ge0\) additionally requires the operating-region condition to hold indefinitely, a separate and independent claim.

**Proof.** By Conditions 1--2, \(\tau_r^{\mathrm{pre}}\in\mathcal T_r\); hence projection onto \(\mathcal T_r\) is the identity and \(c_{\tau,r}=0\). The concrete defect decomposition therefore reduces to \(\mathcal D_{z,r}\), which is contained in \(\mathcal E_\star\) by Condition 3, placing the successor in \(F(\Pi_r(x),a_{\mathrm{req}})\oplus\mathcal E_\star\subseteq\mathcal S\) directly by the definition of \(\mathcal K_{\mathrm{cert}}\) --- no separate robust-invariance premise about a specific policy is needed. Finite induction from \(x_0\in\mathcal X_r\) over \(i=0,\ldots,N\) gives the horizon result; extending it to all \(k\ge0\) requires Condition 5 to hold at every step, which this argument alone does not establish.

The theorem's nontrivial, falsifiable work is the construction of \(\mathcal D_{\tau,r}\) and \(\mathcal D_{z,r}\) and verification of their containments against the available actuator margin. If those sets are merely postulated without a reproducible bound, the result collapses back to a tautological refinement assumption.

**Remark (concrete instantiation).** The QP of Section 6.2 enforces \(a_{\mathrm{req},i|\ell}\in\mathcal A_r^{\mathrm{tight}}(\hat x^0_{i|\ell})\cap\mathcal K_{\mathrm{cert}}(z_{i|\ell})\), so that any feasible solution provably satisfies both sets by construction rather than by a check applied after the fact. The instantiation used is deliberately simple. Let

\[
V(z)=\max\!\left\{\frac{|e_1|}{\mathrm{pos}_{\max}},\frac{|e_2|}{\mathrm{pos}_{\max}},\frac{|\dot e_1|}{\mathrm{spd}_{\max}},\frac{|\dot e_2|}{\mathrm{spd}_{\max}}\right\},
\qquad
\mathcal S=\{z:V(z)\le1\},
\]

an \(\infty\)-norm storage function chosen, in place of a quadratic \(z^\top Pz\), specifically because it keeps \(\mathcal S\) and \(\mathcal K_{\mathrm{cert}}\) linear and solvable by the same OSQP instance already used for \(\mathcal A_r^{\mathrm{tight}}\), without a QCQP solver. Here \(\mathcal S\) is exactly the existing position/speed operating region \(\mathcal X\) of Section 6. With \(\epsilon_{\mathrm{audit}}=0.03~\mathrm{m/s}\) standing in for the abstract radius of \(\mathcal E_\star\)---this numeric choice ties the prospectively enforced bound to the same value retrospectively checked by the audit of Section 9, not a general requirement of Theorem 1---the certified action set tightens only the one-step-ahead velocity block,

\[
\mathcal K_{\mathrm{cert}}(z)
=
\left\{a:\left|(Az+Ba)_{\dot e}\right|\le\mathrm{spd}_{\max}-\epsilon_{\mathrm{audit}}\right\}.
\]

Position is left untightened: only the velocity-space successor defect is measured and audited here, so tightening position would not be backed by a measured quantity. Because deceleration at the actuator limit removes \(\epsilon_{\mathrm{audit}}\) of speed within \(\epsilon_{\mathrm{audit}}/(\Delta t\,a_{\max})\ll1\) of one manager period for the parameters used, \(\mathcal K_{\mathrm{cert}}(z)\) is nonempty for every \(z\in\mathcal S\). Section 9 reports a dedicated ablation showing this constraint changes the manager's output relative to leaving it out; in the eight main stress scenarios it does not bind, consistent with Section 9's own finding that the audited defect leaves most of the certificate radius unused.

The region-persistence clause is likewise an explicit conditional assumption, not a consequence of the implemented finite-horizon QP. A recursively feasible terminal set or certified backup policy would be one way to establish it over an unbounded horizon. Without such a construction, the horizon in this study should be read as finite --- matching the manager's own \(N=12\)-step horizon --- rather than indefinite.

### Norm-checkable specialization

Let \(\|\cdot\|_P\) be the norm used by the abstract certificate and suppose the implementation establishes

\[
\begin{aligned}
\|\eta_{\mathrm{disc}}\|_P&\le\bar\eta_{\mathrm{disc}},\\
\|\eta_{\mathrm{hold}}\|_P&\le\bar\eta_{\mathrm{hold}},\\
\|\eta_{\mathrm{sec}}\|_P&\le\bar\eta_{\mathrm{sec}},\\
\|\eta_{F_h}\|_P&\le L_F\bar\delta_F,\\
\|\eta_{\tau}\|_P&\le L_\tau\bar\delta_\tau.
\end{aligned}
\]

Here the terms respectively bound discretization, slow-to-fast interpolation, secondary-channel coupling, unmatched interaction-force error, and torque-realization error. Then a directly testable sufficient condition for Condition 3 is

\[
\bar\eta_{\mathrm{disc}}
+\bar\eta_{\mathrm{hold}}
+\bar\eta_{\mathrm{sec}}
+L_F\bar\delta_F
+L_\tau\bar\delta_\tau
\le
\bar\epsilon_\star,
\]

where the abstract certificate has been verified for

\[
\mathcal E_\star
=
\left\{
\eta:\|\eta\|_P\le\bar\epsilon_\star
\right\}.
\]

Together with the componentwise actuator test

\[
\min_j
\left\{
\hat\tau_j-\tau_{\min,j},
\tau_{\max,j}-\hat\tau_j
\right\}
\ge
\|\bar\delta_\tau\|_\infty,
\]

this turns the transfer test into two numerical inequalities: the actuator margin must dominate torque-realization uncertainty, and the certificate radius must dominate the remaining successor defect.

### Saturation-boundary extension

To cover the nonlinear projection explicitly, define its set-valued residual

\[
\mathcal C_{\tau,r}(x,a_{\mathrm{req}})
=
\left\{
\operatorname{proj}_{\mathcal T_r}(\tau)-\tau:
\tau\in
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\right\}.
\]

Theorem 1 uses the transparent special case \(\mathcal C_{\tau,r}(x,a_{\mathrm{req}})=\{0\}\). More generally, including when uncertainty straddles the saturation boundary or clipping is active, the refinement and certificate still transfer if the **computed** nonlinear projection defect satisfies

\[
\mathcal D_{z,r}(x,a_{\mathrm{req}},F_h)
\oplus
\mathcal L_r(x)\mathcal C_{\tau,r}(x,a_{\mathrm{req}})
\subseteq
\mathcal E_\star.
\]

This is not automatic: loss of directional authority enlarges \(\mathcal C_{\tau,r}\), and transfer fails once its image no longer fits inside the abstract certificate's error budget.

## 7.4 Optional viability extension (not implemented in the benchmark)

For completeness, a future implementation with a certified terminal set could define the following object. It is not computed or claimed by the present experiments.

Define the certificate-preserving \(N\)-step viability set

\[
\begin{aligned}
\mathcal V_{N,r}^{\mathrm{cert}}
=
\big\{
z_0:\;&
\exists a_{\mathrm{req},0:N-1}\ \text{such that}\\
&
a_{\mathrm{req},i}\in\mathcal A_r^{\mathrm{tight}}(x_i),\\
&
\mathcal D_{z,r}(x_i,a_{\mathrm{req},i},F_{h,i})\subseteq\mathcal E_\star,\\
&
z_{i+1}\in F(z_i,a_{\mathrm{req},i})\oplus\mathcal E_\star,\\
&
z_N\in\mathcal X_f
\big\}.
\end{aligned}
\]

Three events must not be conflated:

1. \(a_{\mathrm{req},i}^0\notin\mathcal A_r^{\mathrm{tight}}(x_i)\): the nominal fast controller requires intervention;
2. a corrected \(a_{\mathrm{req},i}\in\mathcal A_r^{\mathrm{tight}}(x_i)\) still exists: Theorem 1 can remain active; and
3. \(z_i\notin\mathcal V_{N,r}^{\mathrm{cert}}\): no correction sequence in the chosen horizon can preserve the theorem's realizability and terminal conditions.

The third event would define a finite-horizon certificate boundary relative to the chosen model, error bounds, terminal set, and horizon; it would not imply that no controller could recover the physical robot. Because the current implementation has no certified \(\mathcal X_f\) and does not track \(\mathcal V_{N,r}^{\mathrm{cert}}\), this extension is outside the experimental claims.

This construction links the previously separate objects:

\[
\text{tightened realizability}
\Longrightarrow
\text{refinement}
\Longrightarrow
\text{certificate transfer},
\]

while \(\mathcal V_{N,r}^{\mathrm{cert}}\) identifies the states from which the implication can be maintained over the horizon.

## 7.5 What transfers and what remains robot-specific

The theorem does not eliminate per-robot work. Each robot must verify:

- the abstraction map \(\Pi_r\);
- the predicted realization map \(\hat\tau_r\);
- the torque-error set \(\mathcal D_{\tau,r}\);
- the no-clipping successor defect \(\mathcal D_{z,r}\);
- the operating region; and
- the tightened admissible set.

What transfers is the robust abstract certificate \((F,\mathcal K_{\mathrm{cert}},\mathcal S,\mathcal E_\star)\). A new robot need not repeat that proof if it verifies the constructive margin conditions above.

That is the paper's proposed irreducibility claim:

> Separation does not make physical feasibility universal; it makes the logical boundary between a reusable certificate and robot-specific feasibility explicit and testable.

## 7.6 Sketch: task-channel dissipativity transfer (not a formal result)

This is deliberately not stated as a corollary. It is a conditional, first-version sketch: it assumes a bounded realization-power defect rather than constructing that bound from the manager's own directional-authority margin, and it never relates the abstract storage function below to a physical robot storage function or fixes sign conventions between the task-error convention \(e\) and physical interaction power. A stronger result --- bounding the defect using \(\alpha_r^+\) so that dissipativity holds precisely when the manager preserves realizability, including through active clipping, and grounded in an actual physical storage function --- requires a defined energy port and an in-horizon tank that this benchmark does not implement, and is left to future work.

Suppose the abstract certificate additionally supplied a storage function \(V_\tau\) on the task-force channel with the sampled balance

\[
V_\tau(z_{\ell+1})-V_\tau(z_\ell)
\le
\Delta t\,w_\ell^\top\dot e_\ell+\varepsilon_\ell,
\]

where \(\dot e_\ell\) is the task-velocity block already carried in \(z_\ell\) and \(w_\ell\) is the task-space force accounted for by the certificate (for example, the certified component of \(F_h\)). Suppose further that the realized task-channel force is consistent with the certificate's accounting up to a bounded defect,

\[
\left(\Lambda_r(x_\ell)a_{\mathrm{req},\ell}\right)^\top\dot e_\ell
\le
w_\ell^\top\dot e_\ell+\delta_\ell,
\qquad
|\delta_\ell|\le\bar\delta,
\]

where \(\Lambda_r(x_\ell)a_{\mathrm{req},\ell}\) is the task-space force realized through \(H_r(x_\ell)=J_r(q)^\top\Lambda_r(q)\) (Section 4.2); both sides of this inequality are stated at the task port, so no joint-space secondary-channel power enters \(\delta_\ell\).

then this sketch would give, on the no-clipping branch of Theorem 1 (\(\tau_r^{\mathrm{app}}=\tau_r^{\mathrm{pre}}\), so that \(\delta_\ell\) is the residual model and discretization defect at the task port only) and with the accumulated bounds on \(\varepsilon_\ell\) and \(\delta_\ell\) inside the certificate margin \(\mathcal E_\star\), a task-channel dissipation budget transferring to the physical implementation over the horizon.

None of this is evaluated in the experiments of Section 9, and \(\bar\delta\) is assumed here, not constructed. Under active clipping, the achieved task acceleration departs from \(a_{\mathrm{req},\ell}\) by \(G_r(x_\ell)(\tau_r^{\mathrm{app}}-\tau_r^{\mathrm{pre}})\) (Section 4.3), and \(\delta_\ell\) would additionally have to absorb the resulting task-force power \(\left(\Lambda_r(x_\ell)G_r(x_\ell)(\tau_r^{\mathrm{app}}-\tau_r^{\mathrm{pre}})\right)^\top\dot e_\ell\), which is uncounted in the balance above and can violate it; a tank or dissipativity budget would have to cover the accumulated positive part of that term, and it cannot be replaced by an assumption that an unspecified defect is bounded. Bounding that term by the directional-authority margin \(\alpha_r^+\) --- so that dissipativity would hold precisely when the manager preserves realizability --- would extend this sketch to the clipping branch; we leave that construction, the physical storage function, the sign-convention check, and the energy-tank machinery to future work. We claim nothing here beyond the sketch itself: no conditional transfer, no full-port passivity, and no coupled stability against an arbitrary passive environment.

---

# 8. Simulation Implementation

The implementation is a deterministic two-dimensional task-space benchmark intended to test the architecture and the theorem's numerical conditions. It is not presented as a full rigid-body or hardware validation of any commercial robot. The nominal controller and final actuator projection execute at \(1~\mathrm{kHz}\); the predictive manager executes at \(50~\mathrm{Hz}\) with a 12-step horizon.

Three robot-specific realization maps are used:

- a planar two-link map;
- an FR3-inspired surrogate map; and
- a six-axis-arm surrogate map.

The latter two names denote actuator-geometry surrogates, not manufacturer-accurate dynamics. Each map has a different configuration-dependent torque projection and actuator box. All three use the same behavior state, MPC objective, certificate function, and abstract disturbance set

\[
\mathcal E=\{d_z:\|d_z\|_2\le 0.03~\mathrm{m/s}\}.
\]

The experiment suite contains 111 deterministic cases:

- 40 scenario--architecture comparisons;
- 30 controller-interface cases;
- 24 cross-realization refinement checks; and
- 17 targeted ablations.

A ninth scenario, reserved for the certified-action-set ablation and not registered in the main scenario or robot-transfer matrices, starts with velocity already inside the untightened speed box but outside the certificate-tightened one, isolating whether \(\mathcal K_{\mathrm{cert}}\) actually binds (Section 9.4).

Five fast-controller interfaces are exercised without changing the manager formulation:

1. analytic PD;
2. analytic impedance;
3. a small policy trained by a deterministic evolution-strategy procedure;
4. a fixed-feature neural policy fitted to impedance demonstrations; and
5. a scripted AI-conditioned motion primitive executed by a PD servo.

The learned and AI-conditioned cases establish software-interface compatibility only. They are not evidence of general RL, neural-network, or foundation-model safety.

The compared realization architectures are direct clipping, a reactive \(1~\mathrm{kHz}\) projection, a scalar reference governor followed by the same reactive projection, and the proposed horizon-wide vector correction with final high-rate actuator projection. A nominal diagnostic without torque projection is retained only to expose the underlying request; all nominal accelerations still share the configured \(\pm12~\mathrm{m/s^2}\) bound.

---

# 9. Results

## 9.1 Scenario study

![Directional authority, torque demand, workspace margin, and correction in representative scenarios.](results/directional_authority_results.png)

![Directional authority, torque demand, workspace margin, and correction in the near-boundary braking scenario.](results/near_boundary_braking_results.png)

The near-boundary braking case starts with an outward velocity already close to the position boundary under a shrinking torque budget. Direct clipping lets the position overshoot the boundary; the reactive projection, reference governor plus projection, and proposed manager all arrest it at the boundary. Because no viability kernel or terminal invariant set is computed, this result supports only finite-horizon constraint handling in the tested trajectory.

![Scenario-level comparison of clipping, reactive projection, reference governor, and the proposed manager.](results/scenario_summary.png)

The principal scenario results are summarized below. “Pre-projection torque excess” measures whether the requested realization remained inside the actuator box; “applied excess” is zero whenever the final projection is enabled. The transfer column requires every sampled refinement condition, including the theorem's no-clipping branch.

| Scenario | Method | Pre-projection torque excess (Nm) | Workspace excess (mm) | Warning lead (s) | Sampled refinement checks |
|---|---|---:|---:|---:|---:|
| No saturation | Clipping | 0.000 | 0.000 | -- | Yes |
| No saturation | Proposed | 0.000 | 0.000 | -- | Yes |
| Slow saturation | Clipping | 0.000 | 78.970 | -- | No |
| Slow saturation | Proposed | 0.000 | 0.001 | 0.412 | Yes |
| Sudden disturbance | Clipping | 7.208 | 0.000 | -- | No |
| Sudden disturbance | Proposed | 11.976 | 0.000 | 0.084 | No |
| Directional collapse | Clipping | 0.693 | 103.914 | -- | No |
| Directional collapse | Proposed | 0.000 | 0.003 | 0.339 | Yes |
| Near-boundary braking | Clipping | 0.000 | 52.537 | -- | No |
| Near-boundary braking | Proposed | 0.000 | 0.011 | 0.566 | Yes |
| Model mismatch | Clipping | 12.665 | 190.434 | -- | No |
| Model mismatch | Proposed | 1.981 | 193.598 | 0.558 | No |
| Preview mismatch | Clipping | 19.917 | 190.191 | -- | No |
| Preview mismatch | Proposed | 4.409 | 344.265 | 0.170 | No |

With the exact-pass-through bypass of Section 6.2, the proposed manager's warning lead for slow saturation and directional collapse (\(0.412\) and \(0.339~\mathrm{s}\)) now falls between the reactive projection's (\(0.419\) and \(0.340~\mathrm{s}\)) and the reference governor's (\(0.395\) and \(0.306~\mathrm{s}\)), rather than exceeding both as an earlier, still-perturbable version of the manager reported. Intervention is now triggered only by genuine infeasibility, not partly by the smoothing term; the remaining differences across all three methods are at most a few tens of milliseconds.

The proposed manager prevents the slow, directional, and near-boundary braking violations in this benchmark. It does not rescue requests under abrupt disturbance or severe model/preview mismatch. In those cases the final projection still enforces the actuator box, but the pre-projection request is infeasible and the sampled successor defect exceeds the empirical budget; therefore Theorem 1 cannot be invoked.

## 9.2 Fast-controller substitution

![Controller-interface results on the slow-saturation scenario.](results/controller_transfer.png)

The same manager formulation and weights were used for all five controller interfaces. In the no-saturation case, the correction RMSE remained below \(0.01~\mathrm{m/s^2}\) for four of the five controllers. The exception is the small evolution-strategy policy, whose limited asymmetric training set does not cover the test trajectory and whose learned command retains a positive-\(y\) bias (correction RMSE \(\approx1.01~\mathrm{m/s^2}\)). This is an observed generalization error, not evidence of a specific causal failure of evolution strategies. The manager holds the state inside the workspace box but is compensating for a biased interface rather than staying inactive. Under slow saturation:

| Fast-controller interface | Realization RMSE (\(\mathrm{m/s^2}\)) | Correction RMSE (\(\mathrm{m/s^2}\)) | Warning lead (s) | Workspace excess (mm) | Sampled refinement checks |
|---|---:|---:|---:|---:|---:|
| PD | 0.732 | 0.726 | 0.371 | 0.001 | Yes |
| Impedance | 1.064 | 1.060 | 0.412 | 0.001 | Yes |
| Trained policy | 0.652 | 0.652 | 1.207 | 0.016 | Yes |
| Fitted neural policy | 0.655 | 0.649 | 0.356 | 0.001 | Yes |
| AI-conditioned proxy | 0.835 | 0.830 | 0.359 | 0.006 | Yes |

These results support interface substitution in the implemented operating region. They do not establish a controller-independent safety theorem: each preview error must still be covered by the verified uncertainty sets.

## 9.3 Cross-realization sampled refinement audit

![Observed successor defects and the common empirical budget for three realization maps.](results/sampled_interface_audit.png)

The behavior model, predictive objective, and \(0.03~\mathrm{m/s}\) empirical successor-defect budget were kept unchanged. Only the realization map, torque box, and sampled robot-specific error bounds changed.

| Realization map | Maximum successor defect (m/s) | Unused defect budget (m/s) | \(D_\tau\) containment margin (Nm) | Sampled refinement checks |
|---|---:|---:|---:|---:|
| Planar 2R | 0.007456 | 0.022544 | 0.002523 | Yes |
| FR3-inspired surrogate | 0.007696 | 0.022304 | \(8.66\times10^{-6}\) | Yes |
| Six-axis-arm surrogate | 0.007696 | 0.022304 | \(9.96\times10^{-5}\) | Yes |

All three reports use the same numerical budget. This is a sampled-trajectory numerical audit, not an analytic workspace-wide proof, a robust-invariance experiment, or validation of the surrogates as full robot models. Instantiating certificate transfer would additionally require an independently verified \((F,\mathcal K_{\mathrm{cert}},\mathcal S,\mathcal E_\star)\).

## 9.4 Ablations

![Paired ablations for horizon constraints, tightening, preview, fast remapping, final projection, rate smoothing, realization-map updates, and the certified action set.](results/ablation_summary.png)

The full-horizon controller produced zero predicted future torque violation in the horizon-ramp case. Constraining only the first predicted move also kept the executed first move inside the box, but left a \(3.587~\mathrm{Nm}\) planned future violation and produced \(31.180~\mathrm{mm}\) workspace excess. Thus checking only \(i=0\) hides the failure that motivated this study.

Removing uncertainty tightening produced \(0.0848~\mathrm{Nm}\) pre-projection excess; the final projection concealed this at the applied-torque channel but did not restore the no-clipping refinement condition. Disabling the final projection in the sudden-disturbance case produced \(10.538~\mathrm{Nm}\) applied excess, confirming that slow prediction cannot replace the high-rate actuator guard.

Removing the rate-smoothing term from the objective left correction RMSE under slow saturation statistically unchanged (\(1.0604\) versus \(1.0603~\mathrm{m/s^2}\) with smoothing on). With the exact-pass-through bypass already confining intervention to genuinely infeasible steps, smoothing's remaining role is to shape the QP's solution during those interventions, and this gradually ramping scenario does not exercise that distinction; its effect under a more abrupt correction is left to future work.

The remaining ablations are informative negative results. Zero-force preview increased workspace excess from \(344.265\) to \(873.921~\mathrm{mm}\), and neither preview option recovered viability under the deliberately severe preview mismatch. In the reduced-order benchmark, high-rate remapping did not outperform cached torque, and updating the realization map was numerically indistinguishable from freezing it. These two architectural benefits therefore remain hypotheses for nonlinear rigid-body and hardware tests, not demonstrated conclusions of the present experiment.

The eight main stress scenarios never approach the certificate-tightened velocity bound closely enough to exercise \(\mathcal K_{\mathrm{cert}}\), consistent with Section 9.3's finding that the audited defect leaves most of the certificate radius unused. A dedicated ninth scenario (Section 8) starts at \(v_0=0.58~\mathrm{m/s}\), already inside the narrow band above the tightened bound (\(0.57~\mathrm{m/s}\)) but below the untightened speed limit (\(0.60~\mathrm{m/s}\)), with a distant goal that keeps pulling the request further. With \(\mathcal K_{\mathrm{cert}}\) enforced, peak speed is held at its \(0.580~\mathrm{m/s}\) starting value; without it, peak speed climbs to \(0.597~\mathrm{m/s}\), toward the untightened limit. Because the initial state \(z_0\) itself is never constrained by the QP, the constrained case cannot undo an already-out-of-budget start --- it can only prevent further growth --- so this ablation isolates the constraint's effect on the manager's output rather than demonstrating recovery from an infeasible initial condition.

## 9.5 Timing

In the regenerated run, the manager's median-of-run-medians was \(1.695~\mathrm{ms}\), with a worst observed maximum of \(13.398~\mathrm{ms}\), below the \(20~\mathrm{ms}\) manager period. The fast-path median-of-run-medians was \(121.2~\mu\mathrm{s}\), but its worst observed maximum was \(2.466~\mathrm{ms}\), exceeding the \(1~\mathrm{ms}\) nominal period. This scheduling outlier's severity varies across regenerated runs and is characteristic of the non-real-time Python implementation; it establishes typical throughput, not hard real-time execution.

---

# 10. Claim Audit

The evidence supports six bounded conclusions, plus one conditional analytical result.

1. A common command/preview contract accepted all five implemented fast-controller interfaces without changing the predictive optimization statement.
2. Horizon-wide constraints exposed a future violation that a first-step-only check missed.
3. One empirical successor-defect budget passed sampled checks across three distinct realization maps.
4. The final \(1~\mathrm{kHz}\) projection remained necessary for disturbances arriving between manager updates.
5. The exact-pass-through bypass verifiably confines intervention to genuinely infeasible steps: correction RMSE under no saturation is exactly zero for four of five controllers, and removing the rate-smoothing term leaves correction under slow saturation statistically unchanged, confirming smoothing no longer perturbs already-feasible requests.
6. The concrete, velocity-tightening instantiation of \(\mathcal K_{\mathrm{cert}}\) is enforced as a hard QP constraint, and a dedicated ablation confirms it changes the manager's output when it binds while leaving all eight main stress scenarios unaffected.
7. Section 7.6 sketches, without stating as a corollary, a candidate task-channel dissipativity-transfer condition on the no-clipping branch; it is analytical only, not evaluated in the experiments, assumes rather than constructs its power-defect bound, and never relates the abstract storage function to a physical one.

The evidence does not support universal black-box-policy safety, hard real-time execution, recursive feasibility, a certified viability region, full-port passivity, or manufacturer-specific FR3 performance. The mismatch failures are not omitted: they mark where the sampled refinement checks and theorem premises cease to hold.

---

# 11. Reproducibility

All paper-facing results are generated by:

```bash
python3 pHRI/saturation/simulation/run_all_experiments.py
```

The regression suite is:

```bash
MPLCONFIGDIR=/tmp/mpl-saturation \
XDG_CACHE_HOME=/tmp/cache-saturation \
PYTHONPATH=pHRI/saturation/simulation \
pytest -q pHRI/saturation/simulation
```

The machine-readable report is
[`results/all_experiment_metrics.json`](results/all_experiment_metrics.json), and representative time histories are stored in
[`results/representative_logs.npz`](results/representative_logs.npz). Dependencies and command details are listed in [`README.md`](README.md) and [`requirements.txt`](requirements.txt).

---

# 12. Architecture Comparison

| Method | Preserves 1 kHz nominal controller | Predicts future saturation | Handles configuration-dependent actuator geometry | Exposes intervention residual | Provides cross-robot certificate transfer |
|---|---:|---:|---:|---:|---:|
| Direct clipping | Yes | No | Only at current torque | Usually no | No |
| Reactive CBF-QP | Yes | Limited | Yes | Yes | Not by default |
| Reference governor | Yes | Yes | Model-dependent | Reference residual | Not by default |
| Robot-specific MPC | Replaces or embeds it | Yes | Yes | Formulation-dependent | No |
| Proposed separated manager | Yes | Yes | Through \(\hat\tau_r\) and \(\mathcal A_r^{\mathrm{tight}}\) | Dynamics residual | Conditional on Theorem 1 |

The final row is a conditional architectural property, not an assertion that the conditions hold automatically. For each new robot, \(\mathcal D_{\tau,r}\), \(\mathcal D_{z,r}\), the tightened realization set, and the operating region must be checked again.

---

# 13. Discussion

## 13.1 Why the MPC is slow

The predictive layer reasons over a horizon and may use a nonlinear or locally linearized robot model. It therefore need not run at the actuator-servo rate. Its output is a correction plan, tightened command envelope, or local correction policy.

Fast disturbance rejection remains the responsibility of the \(1~\mathrm{kHz}\) controller and final safety projection. A sudden impulse can occur between MPC updates, so the slow predictor cannot be the only protection mechanism.

## 13.2 Why arbitrary controllers can share the architecture

They do not share internal parameters, training procedures, or control objectives. They share only:

- a current nominal actuator command;
- a physical state estimate;
- a behavior-coordinate projection;
- an optional bounded preview; and
- a correction channel.

This is a weaker and more credible form of universality than claiming one controller model for everything.

## 13.3 Why the feasible set cannot be universal

The feasible task-acceleration set depends on

\[
M_r(q),\quad J_r(q),\quad h_r(q,\dot q),\quad
\tau_{\min,r},\quad \tau_{\max,r},
\]

and therefore changes with robot and configuration. Any method that removes this dependence also removes the geometry responsible for saturation.

The architectural benefit is instead that this dependence appears in one realization module with a verifiable contract.

## 13.4 Why transfer is conditional

Certificate transfer is never automatic. It fails when:

- the realization map is infeasible;
- the model error exceeds \(\mathcal E_\star\);
- the robot leaves the verified operating region;
- the policy preview is unbounded or adversarial;
- an unmodeled contact changes the dynamics; or
- the fast implementation differs from the verified realization operator.

These failure conditions should be monitored and logged at runtime.

## 13.5 Relation to model-based physical AI

The architecture offers a model-based runtime boundary for physical-AI behavior sources: learned or language-conditioned modules may propose behavior, while a robot-specific realization model checks and modifies what can be executed. This does not certify the semantics of an AI command; it certifies only the explicitly modeled realization contract inside its verified operating region.

---

# 14. Conclusion

This paper does not propose MPC as a replacement for fast robot control. PD, impedance, RL, neural, and AI-conditioned controllers remain in the \(1~\mathrm{kHz}\) loop. A slower predictive realization manager forecasts when their requested dynamics will become unrealizable under actuator saturation and introduces the smallest behavior correction before direct clipping is required.

The double-integrator prediction backbone is useful but classical. The contribution is the separation that permits a behavior-level certificate to be reused through an explicit robot-specific refinement condition, and a certified action set stated in behavior coordinates. A deliberately simple instantiation of that certified action set is enforced inside the QP itself, so the theorem's key hypothesis holds by construction rather than by a check applied afterward. In 111 deterministic cases, the interface accepted five fast-controller types, one empirical successor-defect budget passed sampled checks across three realization surrogates, and a dedicated ablation confirmed the certified action set changes the manager's output when it binds. Only the abstract side of the theorem's premise is instantiated this way; the physical containment the robust-invariance premise also requires remains a sampled audit, not a workspace-wide proof. Horizon-wide correction prevented the slow, directional, and near-boundary braking violations, while abrupt disturbance and severe mismatch provided explicit counterexamples outside the theorem's premises.

Physical feasibility remains configuration-dependent; what transfers is the certificate and the logic connecting behavior to realization. The present results establish a reproducible reduced-order proof of concept, not hardware robustness, analytic whole-workspace verification, or hard real-time execution.

---

# 15. Remaining Validation Work

The reduced-order experiments resolve the implementation questions but leave the following publication-critical work:

1. derive analytic or interval-certified bounds for \(\mathcal D_{\tau,r}\) and \(\mathcal D_{z,r}\) over a stated workspace;
2. instantiate Theorem 1 with a terminal robust invariant set, and extend the dissipativity sketch of Section 7.6 from the no-clipping branch to active clipping by bounding the saturation-power term with the directional-authority margin \(\alpha_r^+\) and a constructed physical storage function, which requires a defined energy port and an in-horizon tank;
3. repeat the transfer audit on two full rigid-body robot models with manufacturer actuator limits;
4. evaluate contacts, orientation, null-space motion, torque rate, sensing delay, and state-estimation error;
5. compare against independently implemented, parameter-matched predictive safety-filter and vector or command-governor baselines that modify a multidimensional task command, not only a scalar parameter;
6. sweep disturbance magnitude, timing, and sensor noise across seeds to characterize the margin between each successful scenario and its failure boundary, rather than reporting single deterministic trajectories;
7. bound the intra-step drift of the fast controller's re-evaluated request within one manager period (the QP currently holds \(a_{\mathrm{req},i|\ell}\) fixed for its full \(\Delta t\), which the sudden-disturbance failure mode exposes, not a lack of force compensation in the realization map), and generalize the instantiated \(\mathcal K_{\mathrm{cert}}\) beyond its present scope --- it tightens only the one-step velocity block using an \(\infty\)-norm storage function, not a general quadratic or position-aware certificate, and was constructed for one 2-D reduced-order model rather than derived from a workspace-wide invariance proof;
8. move the fast path to a real-time implementation and report deadline misses over long-duration trials; and
9. validate the architecture on hardware with an independent emergency-stop and safety layer.

Until those checks are complete, the strongest defensible positioning is certificate-transferable predictive saturation management demonstrated in reduced-order simulation.

---

# References to Develop

1. N. Hogan, ``Impedance Control: An Approach to Manipulation,'' *Journal of Dynamic Systems, Measurement, and Control*, 1985.
2. O. Khatib, ``A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation,'' *IEEE Journal on Robotics and Automation*, 1987.
3. E. Garone, S. Di Cairano, and I. Kolmanovsky, ``Reference and Command Governors for Systems with Constraints: A Survey on Theory and Applications,'' *Automatica*, 2017.
4. A. D. Ames et al., ``Control Barrier Functions: Theory and Applications,'' *European Control Conference*, 2019.
5. K. P. Wabersich and M. N. Zeilinger, ``Linear Model Predictive Safety Certification for Learning-Based Control,'' *IEEE Conference on Decision and Control*, 2018.
6. Y.-Y. Cao, Z. Lin, and D. G. Ward, ``An Anti-Windup Approach to Enlarging Domain of Attraction for Linear Systems Subject to Actuator Saturation,'' *IEEE Transactions on Automatic Control*, 2002.
7. Y.-Y. Cao, Z. Lin, and D. G. Ward, ``Anti-Windup Design of Output Tracking Systems Subject to Actuator Saturation and Constant Disturbances,'' *Automatica*, 2004.
8. A. Girard and G. J. Pappas, ``Approximation Metrics for Discrete and Continuous Systems,'' *IEEE Transactions on Automatic Control*, vol. 52, no. 5, pp. 782--798, 2007.
9. A. Girard and G. J. Pappas, ``Hierarchical Control System Design Using Approximate Simulation,'' *Automatica*, vol. 45, no. 2, pp. 566--571, 2009.
10. P. Nuzzo, J. B. Finn, A. Iannopollo, and A. Sangiovanni-Vincentelli, ``Contract-Based Design of Control Protocols for Safety-Critical Cyber-Physical Systems,'' in *Design, Automation and Test in Europe Conference and Exhibition (DATE)*, 2014.
