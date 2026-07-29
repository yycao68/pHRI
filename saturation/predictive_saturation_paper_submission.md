# Predictive Saturation Management through a Behavior--Realization Interface

**Anonymous submission**

## Abstract

Clipping protects actuator bounds but can silently destroy a fast controller's designed or trained closed-loop behavior. We present a two-rate predictive realization architecture anticipating this loss without replacing the controller. A proportional--derivative, impedance, reinforcement-learning, neural-network, or behavior-conditioned controller runs unchanged at \(1~\mathrm{kHz}\); a \(50~\mathrm{Hz}\) manager seeks a request sequence inside a robot-specific, uncertainty-tightened feasible set evaluated along the nominal rollout, with a reactive fallback when the horizon problem is infeasible; a logged directional-authority indicator exposes loss hidden by scalar utilization. The main result gives sufficient conditions for a behavior-coordinate certificate to transfer to a physical robot---realization uncertainty must fit inside the actuator margin, and the successor defect inside the certificate margin---and a concrete, deliberately simple certified action set is now enforced inside the QP, so a feasible solution satisfies both by construction. A deterministic study of 111 configurations spans five controllers, three realization maps, eight scenarios, and eight ablations. Full-horizon enforcement removes a \(3.587~\mathrm{Nm}\) violation missed by a first-step constraint, and a cross-realization audit finds all successor defects below \(0.0077~\mathrm{m/s}\) against a common \(0.03~\mathrm{m/s}\) threshold. The method prevents sampled violations under slow saturation, directional-authority collapse, and near-boundary braking; abrupt disturbances and severe model or preview mismatch exceed the audit conditions, marking the result's boundary. A dedicated ablation confirms the certified action set changes the manager's output when it binds, while leaving the eight main stress scenarios unaffected, consistent with their own comfortable certificate margin. The study instantiates and enforces a deliberately simple certificate for this reduced-order model; it does not establish universal policy safety, a workspace-wide invariance proof, or real-time hardware performance.

**Index Terms—** physical interaction, actuator saturation, predictive constraint management, control refinement, model predictive control.

---

# I. Introduction

Robot control software increasingly combines high-rate feedback with lower-rate prediction. The fast layer may be a conventional proportional--derivative (PD) controller, an impedance controller, or a learned policy. Its purpose is to react to sensor feedback with low latency. A slower layer can reason over a horizon, but embedding every nominal controller inside a new robot-specific model-predictive controller sacrifices the modularity that makes the fast layer useful.

These fast controllers can approach saturation through different channels. Before command limiting, a fixed-gain PD law \(\tau\propto k_p(q_{\mathrm{goal}}-q)-k_d\dot q\) maps a sufficiently large tracking error to a large actuator request. An impedance law introduces interaction force through \(M_d\ddot e=-K_de-D_d\dot e+F_h\), so contact can increase the requested acceleration even when tracking error is modest. The implementation studied here imposes a common acceleration bound on every nominal interface, but this does not guarantee torque feasibility after configuration-dependent realization.

The same distinction applies to bounded learned policies. The \(\tanh\)-squashed policy used in Section VI bounds its requested acceleration, but the required torque is \(\tau=\tau_{\mathrm{base}}(x)+H_r(x)a_{\mathrm{req}}\); a fixed bound on \(a_{\mathrm{req}}\) does not imply a configuration-independent bound on \(\tau\). A neural policy fitted to impedance demonstrations can reproduce the same force-dependent requests without exposing interpretable stiffness and damping parameters. An upstream learned or language-conditioned module may likewise propose a motion primitive without representing the actuator geometry of the executing robot. These observations motivate a common realization interface, not a claim that every learned behavior is certifiable.

The shared consequence is an acceleration request that the current robot cannot realize in its current configuration. We therefore treat saturation management as an interface problem. Controller-specific measures remain useful, but the robot-aware realization layer developed here anticipates the common physical failure without requiring its predictive optimization to be reformulated for each upstream controller.

Formally, let a nominal controller of any of these kinds request

\[
\tau_k^0=\pi_\theta(s_k),
\]

where \(s_k\) is the controller observation and \(\pi_\theta\) may be analytic or learned. A conventional implementation applies

\[
\tau_k^{\mathrm{app}}
=
\operatorname{proj}_{\mathcal T_r}(\tau_k^0),
\qquad
\mathcal T_r
=
\{\tau:\tau_{\min,r}\le \tau\le\tau_{\max,r}\}.
\]

Projection protects the actuator command but changes the acceleration generated by the robot. After projection, the robot is no longer realizing the nominal PD, impedance, or learned closed-loop dynamics. This distinction is especially consequential in physical interaction control, where the requested acceleration determines apparent compliance and disturbance response.

The architecture developed below requires more than an arbitrary \(\tau_k^0\): it requires the nominal controller to expose the task-acceleration request that \(\tau_k^0\) realizes, so that the manager's correction is well defined (Section III). All five controller interfaces evaluated in Section VI already expose this request directly.

We study whether prediction can preserve the nominal fast controller while correcting its requested dynamics before the actuator command becomes unrealizable. The proposed architecture separates two responsibilities. A behavior layer specifies the nominal local dynamics at \(1~\mathrm{kHz}\). A predictive realization manager, executing at \(50~\mathrm{Hz}\), checks whether those dynamics can be produced by the current robot over a finite horizon and computes a minimum correction when required. A final high-rate projection remains necessary for disturbances that arrive between predictive updates.

Feedback linearization to a double-integrator-like behavior model is classical [1], [2]. Likewise, reference governors, predictive safety filters, anti-windup control, and model-predictive constraint handling are established ideas [3]--[7]. The contribution is therefore not another claim of universal prediction. The central question is instead:

> When can a behavior-level certificate be reused across different nominal controllers and different robots, despite configuration-dependent actuator saturation?

The answer requires preserving, rather than hiding, the robot-specific realization geometry. The common behavior dynamics provide a reusable location for prediction and certification, while the actuator-feasible acceleration set and uncertainty bounds remain specific to each robot.

The contributions are:

1. a two-rate architecture that retains an existing \(1~\mathrm{kHz}\) controller and uses slower MPC only to anticipate and correct loss of actuator realizability;
2. a robot-dependent acceleration-zonotope representation and a directional-authority indicator logged at every predictive update, exposing failures hidden by scalar torque utilization;
3. a conditional certificate-transfer theorem that separates a reusable certified action set from robot-specific actuator and successor-error tests, together with a deliberately simple instantiation of that set enforced as a QP constraint rather than checked after the fact;
4. a reproducible 111-configuration simulation study evaluating controller substitution, a sampled cross-realization interface audit, horizon-wide constraints, uncertainty tightening, final high-rate projection, the enforced certified action set, and failure outside the tested operating region.

The experiments are reduced-order and intentionally include negative cases. They establish the mechanism and its logical limits rather than hardware-level universality.

---

# II. Related Work

Impedance control specifies a desired dynamic relation between motion and interaction force [1], while operational-space control maps task accelerations or wrenches to joint torque [2]. Both rely on sufficient actuator authority. Direct clipping preserves torque bounds but changes the realized dynamics. Anti-windup methods explicitly address saturation-induced performance and stability degradation [6], [7], but are commonly designed around a particular closed-loop controller and often react to saturation rather than forecasting a configuration-dependent loss of task authority.

Model-reference adaptive impedance control instead asks the robot to reproduce a prescribed impedance model despite uncertain physical dynamics [11]. This is close to our behavior-coordinate viewpoint, but it does not by itself provide finite-horizon actuator-constraint management. The present work retains the fast interaction controller and alters its requested task acceleration before hard clipping is predicted. The final projection is not removed; it remains a last-resort actuator guard.

Reference and command governors modify a reference supplied to a pre-stabilized system so that predicted constraints remain satisfied [3]. Predictive safety filters similarly modify nominal actions using a predictive model and a recoverable terminal condition [5]. Robot-specific MPC can incorporate full nonlinear dynamics and actuator constraints directly. These approaches establish that predictive constraint management is valuable; the present work does not claim otherwise.

Recent interaction-control work combines MPC with impedance adaptation in several ways. Roveda *et al.* optimize impedance setpoints and damping using learned interaction dynamics [12]; Haninger *et al.* jointly optimize trajectory and impedance using Gaussian-process task models [13]; Anand *et al.* use a learned Cartesian impedance model inside MPC to adapt variable-impedance parameters across manipulation tasks [14]; and Xue *et al.* combine model-predictive variable impedance control with environment estimation, passivity, and safety-oriented mode switching [15]. In these methods, impedance parameters or trajectories are optimization variables. Our manager instead treats the upstream controller as fixed and selects a physically realizable task-acceleration sequence close to its request.

This difference does not make the proposed optimizer categorically separate from a reference governor or predictive safety filter. Either architecture could implement the realization manager. The contribution lies in the behavior--realization contract and the sufficient transfer condition, not in assigning a new name to constrained MPC. Unlike predictive safety filters with a verified terminal safe set [16], the present simulation enforces finite-horizon state and actuator constraints but does not establish recursive feasibility.

Control barrier functions provide a natural high-rate mechanism for enforcing state inequalities [4]. They are useful as the final projection in the proposed architecture, but an instantaneous constraint may intervene only after the state has approached a region from which actuator authority is insufficient. Our predictive layer instead evaluates future realization geometry and finite-horizon running constraints; the present implementation does not use a terminal recoverability condition.

The theoretical lineage is approximate simulation and control refinement [8], [9]. An interface maps an abstract input to a concrete input while bounding the mismatch between abstract and concrete successors. Contract-based design similarly separates assumptions about a component from the guarantees it delivers [10]. We specialize that logic to actuator-limited robot realization: the refinement defect is constructed from a tightened actuator set, a torque-realization error bound, and an abstract successor-error bound. This makes certificate transfer conditional and falsifiable.

---

# III. Problem Formulation

Consider a torque-controlled robot \(r\),

\[
M_r(q)\ddot q+h_r(q,\dot q)
=
\tau+J_r(q)^\top F_h,
\]

where \(M_r(q)\succ0\), \(h_r\) collects modeled bias terms, \(F_h\) is an interaction wrench, and \(\tau\in\mathcal T_r\).

The actuator set is the box

\[
\mathcal T_r
=
\{\tau:\tau_{\min,r}\le\tau\le\tau_{\max,r}\},
\]

with both inequalities interpreted componentwise.

The architecture requires the fast controller to expose a nominal task-acceleration request, not an arbitrary torque:

\[
\mathcal I_\theta(s_k,\xi_k)
\mapsto
\left(a_{\mathrm{req},k}^0,\mathcal P_{\theta,k},\Sigma_{\theta,k}\right).
\]

Here \(a_{\mathrm{req},k}^0\) is the current nominal task-acceleration request, \(\mathcal P_{\theta,k}\) is an optional preview operator, and \(\Sigma_{\theta,k}\) bounds preview uncertainty. An analytic PD or impedance controller can be evaluated along predicted states. A learned policy may be queried on predicted observations. If no validated preview is available, zero-order hold or a learned predictor may be used, but its error must enter \(\Sigma_{\theta,k}\). A black-box controller with neither query access nor bounded preview error is outside the certificate.

The nominal pre-projection torque is then defined, not assumed, as the robot's own realization of that request (the realization map is given below),

\[
\tau_k^0=\tau_{\mathrm{base},r}(x_k)+H_r(x_k)a_{\mathrm{req},k}^0,
\]

so the fast-loop correction of Section IV is well defined by construction. A controller that instead outputs torque directly is outside this interface unless it exposes the decomposition \(\tau_k^0=\tau_{\mathrm{base},r}(x_k)+\tau_{\perp,k}^0+H_r(x_k)a_{\mathrm{req},k}^0\), where \(\tau_{\perp,k}^0\) is a policy-dependent secondary-torque term reported alongside \(\tau_{\mathrm{base},r}(x_k)\), not merged into it: \(\tau_{\mathrm{base},r}\) remains the purely robot-dependent quantity used throughout, and testing such a controller's realizability against \(\mathcal A_r^{\mathrm{tight}}(x)\) requires subtracting \(\tau_{\perp,k}^0\) from the available actuator budget in addition to \(\tau_{\mathrm{base},r}(x_k)\). All five controller interfaces evaluated in Section VI already expose \(a_{\mathrm{req}}^0\) directly, with \(\tau_{\perp,k}^0\equiv0\), and require no such decomposition.

Let

\[
z=
\begin{bmatrix}
e^\top & \dot e^\top
\end{bmatrix}^{\top}
\]

denote a task-error state. Over a local operating region, the requested behavior can be written

\[
\ddot e=a_{\mathrm{req}}+w_r,
\]

where \(a_{\mathrm{req}}\in\mathbb R^d\) is the complete requested task acceleration --- already including whatever response to \(F_h\) its issuing controller or the realization map below provides, not a quantity to which a further interaction-force term must be added --- and \(w_r\) is the residual left over after that response: realization error, discretization, unmodeled coupling, state-estimation error, and intra-period command drift, but not a second, separate accounting of \(F_h\).

At the manager period \(\Delta t\), the general abstract predictor is

\[
z_{\ell+1}=Az_\ell+Ba_{\mathrm{req},\ell}+w_\ell,
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

This model is a common interface. The implemented QP of Section IV uses the nominal case \(w_\ell=0\); Section V bounds the resulting successor mismatch empirically as \(\mathcal D_{z,r}\), and Theorem 1's condition (3) requires that bound to lie inside the certificate margin \(\mathcal E_\star\), so the nominal-model choice is checked rather than assumed away. Let \(\tau_{\mathrm{base},r}(x)\) include bias compensation and every secondary command that consumes actuator authority, including orientation and null-space torque. A local realization map is

\[
\tau
=
\tau_{\mathrm{base},r}(x)+H_r(x)a_{\mathrm{req}},
\]

with, for example,

\[
H_r(x)=J_r(q)^\top\Lambda_r(q),
\qquad
\Lambda_r(q)=\left(J_rM_r^{-1}J_r^\top\right)^{-1}.
\]

Because the robot dynamics above already include \(J_r(q)^\top F_h\), the actuator only supplies the part of the task force not already delivered by the interaction wrench; \(\tau_{\mathrm{base},r}(x)\) and the realization map are evaluated net of this feedforward term. Consequently \(\hat\tau_r\) and \(\tau_r^{\mathrm{pre}}\), introduced in Section IV, depend on the current or forecast \(F_h\) as well as on \((x,a_{\mathrm{req}})\); we suppress this argument notationally rather than write it at every occurrence.

The acceleration request is realizable only if

\[
a_{\mathrm{req}}\in\mathcal A_r(x)
=
\left\{
a_{\mathrm{req}}:
\tau_{\min,r}
\le
\tau_{\mathrm{base},r}(x)+H_r(x)a_{\mathrm{req}}
\le
\tau_{\max,r}
\right\}.
\]

Equivalently, if local task acceleration is \(a=b_r(x)+G_r(x)\tau\), the actuator box maps to the zonotope

\[
\mathcal Z_r(x)
=
b_r(x)+G_r(x)\tau_c
+G_r(x)\operatorname{diag}(\Delta\tau)[-1,1]^{n_r},
\]

where

\[
\tau_c=\frac{\tau_{\max,r}+\tau_{\min,r}}{2},
\qquad
\Delta\tau=\frac{\tau_{\max,r}-\tau_{\min,r}}{2}.
\]

For a full-dimensional \(d\)-dimensional zonotope with \(n_r\) generators in general position, every choice of \(d-1\) generators determines a facet normal and a pair of opposing supporting facets. Hence the number of facets is \(2\binom{n_r}{d-1}\) (and no larger in degenerate cases). Its support function in direction \(y\) is the support of its center plus the sum of the absolute projected generators, so directional authority can be evaluated without enumerating all \(2^{n_r}\) box vertices.

Thus the behavior dynamics can share coordinates, but their feasible set cannot be robot-independent.

Given an arbitrary nominal controller that supplies \(a_{\mathrm{req}}^0\) or an equivalent torque command, determine a correction \(\Delta a_{\mathrm{req}}\) such that:

1. \(a_{\mathrm{req}}=a_{\mathrm{req}}^0+\Delta a_{\mathrm{req}}\) remains close to the nominal requested behavior;
2. the uncertain physical realization remains inside \(\mathcal T_r\) over the prediction horizon;
3. the predicted state satisfies the running workspace and speed constraints; and
4. the robot-specific realization can be tested against sufficient conditions for transferring an independently established abstract certificate.

---

# IV. Predictive Realization Manager

The nominal controller, state-dependent realization map, command-rate limiter, and final actuator projection execute every \(T_f=1~\mathrm{ms}\). The predictive manager executes every \(\Delta t=20~\mathrm{ms}\). Let \(a_{\mathrm{req},i|\ell}\) be the complete acceleration request, rather than a correction variable. The implemented manager rolls out the nominal controller, evaluates the realization map along that nominal rollout, and solves

\[
\begin{aligned}
\min_{a_{\mathrm{req},0:N-1}}
\quad&
\sum_{i=0}^{N-1}
\Big(
\|a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i|\ell}^0\|_{W_{a_{\mathrm{req}}}}^2
+\|a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i-1|\ell}\|_{W_\Delta}^2
\Big)
\\
\mathrm{s.t.}\quad&
z_{i+1|\ell}
=
Az_{i|\ell}
+Ba_{\mathrm{req},i|\ell},
\\
&
a_{\mathrm{req},i|\ell}
\in
\mathcal A_r^{\mathrm{tight}}(\hat x^0_{i|\ell}),
\quad i=0,\ldots,N-1,
\\
&
z_{i+1|\ell}\in\mathcal X,
\\
&
\|a_{\mathrm{req},i|\ell}\|_\infty\le a_{\max},
\qquad
\|a_{\mathrm{req},i|\ell}-a_{\mathrm{req},i-1|\ell}\|_\infty
\le \dot a_{\max}\Delta t .
\end{aligned}
\]

Here \(\hat x^0_{i|\ell}\) is the state on the fixed nominal rollout used to assemble the QP. This is the nominal case \(w_\ell=0\) of Section III's general predictor: the predicted position and speed constraints implicitly assume \(a_{\mathrm{req},i|\ell}\) is held fixed for the full \(\Delta t\) it labels. Because the realization map compensates for the currently measured \(F_h\) rather than a forecast one (Section III), \(w_\ell\) is not a re-accounting of force---the fast loop recomputes its nominal request and cancels the interaction wrench at every \(1~\mathrm{kHz}\) sample regardless of controller type, and the observed successor-defect magnitudes are numerically similar across the five tested interfaces (Table III) even under sudden disturbance, though the study runs one deterministic trajectory per case and this is not a statistical claim. What the nominal-model choice misses is instead intra-step drift: if the fast controller's own request changes materially within one \(20~\mathrm{ms}\) manager period---most acutely when an impulse arrives between manager updates---the frozen \(a_{\mathrm{req},i|\ell}\) held throughout that step no longer matches what the fast loop actually applies, and this is exactly the component of \(w_\ell\) that the sampled \(\mathcal D_{z,r}\) check of Section V must contain. Constructing a tightened bound \(\mathcal W_\ell\) directly, rather than checking its empirical consequence after the fact, is left to future work. The state and acceleration bounds are hard; the implementation contains neither slack variables nor a separately constructed terminal invariant set. Before solving, the manager checks whether the nominal rollout \(a_{\mathrm{req},0:N-1}^0\) already satisfies every constraint above; if it does, it is returned unmodified rather than passed through the objective, so the smoothing term cannot perturb an already-feasible request. The QP above is solved only when this check fails, giving the clean property that nominal-rollout feasibility implies exact pass-through, \(a_{\mathrm{req},0:N-1}^0\in\mathcal F_N\implies\Delta a_{\mathrm{req},\ell}=0\), where \(\mathcal F_N\) is the horizon-wide feasible set defined by the constraints above. Define the first correction as

\[
\Delta a_{\mathrm{req},\ell}=a_{\mathrm{req},0|\ell}-a_{\mathrm{req},0|\ell}^0.
\]

If the QP solver reports infeasibility, the implementation projects the first nominal acceleration onto the current tightened torque and one-step state polytope and tiles that reactive command over the stored sequence. If this instantaneous polytope is itself empty, it returns zero acceleration. The final high-rate torque projection remains active in either case. This fallback supplies a deterministic bounded command but does not recover horizon feasibility or satisfy the transfer conditions.

The fast loop applies

\[
\tau_k^{\mathrm{pre}}
=
\tau_k^0
+H_r(x_k)\Delta a_{\mathrm{req},k}
=
\tau_{\mathrm{base},r}(x_k)+H_r(x_k)\left(a_{\mathrm{req},k}^0+\Delta a_{\mathrm{req},k}\right),
\qquad
\tau_k^{\mathrm{app}}
=
\operatorname{proj}_{\mathcal T_r}\!\left(\tau_k^{\mathrm{pre}}\right),
\]

where \(H_r(x_k)\) is recomputed from the current state, so the slow loop publishes a behavior correction, not a cached full torque. Writing the final high-rate projection's effect as \(\Delta\tau_k^{\mathrm{proj}}=\tau_k^{\mathrm{app}}-\tau_k^{\mathrm{pre}}\), so that \(\tau_k^{\mathrm{app}}=\tau_k^{\mathrm{pre}}+\Delta\tau_k^{\mathrm{proj}}\), makes the no-clipping branch of Theorem 1 immediate: it is precisely the case \(\Delta\tau_k^{\mathrm{proj}}=0\).

Let \(\hat\tau_r(x,a_{\mathrm{req}})\) be the manager's predicted pre-projection torque. Suppose a verified componentwise error bound satisfies

\[
\tau_r^{\mathrm{pre}}(x,a_{\mathrm{req}})
\in
\hat\tau_r(x,a_{\mathrm{req}})
\oplus
\mathcal D_{\tau,r}(x,a_{\mathrm{req}}),
\]

\[
\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\subseteq
\{\delta\tau:|\delta\tau|\le\bar\delta_{\tau,r}(x,a_{\mathrm{req}})\}.
\]

The tightened request set is

\[
\mathcal A_r^{\mathrm{tight}}(x)
=
\left\{
a_{\mathrm{req}}:
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\subseteq\mathcal T_r
\right\},
\]

or, componentwise,

\[
\tau_{\min,r}+\bar\delta_{\tau,r}
\le
\hat\tau_r(x,a_{\mathrm{req}})
\le
\tau_{\max,r}-\bar\delta_{\tau,r}.
\]

The set must include state-estimation error, interpolation, secondary torque, torque-rate limiting, and any other implementation effect that can change the pre-projection torque.

For predicted torque \(\tau_{i|\ell}\), define normalized saturation margin, centered so that \(\mu=1\) at \(\tau_c\) and \(\mu=0\) at either bound,

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

which for symmetric limits reduces to \(\mu_{i|\ell}=1-\max_j|\tau_{j,i|\ell}|/\tau_{\max,j}\). This scalar is a diagnostic only and is not computed in the experiments of Section VI; the directional measure below is the one used there.

The minimum horizon margin is

\[
\mu_\ell^{\min}
=
\min_{i=0,\ldots,N-1}\mu_{i|\ell}.
\]

Scalar utilization does not reveal whether the robot retains authority in the direction required to reject an interaction force. Given current acceleration \(a_c\) and unit direction \(d\), define directional authority against the tightened realizable set, not the untightened zonotope \(\mathcal Z_r(x)\) of Section III, so that the metric accounts for secondary-torque consumption and uncertainty exactly as the optimizer does:

\[
\alpha_r^+(x,a_c,d)
=
\max_{\alpha\ge0}
\left\{
\alpha:
a_c+\alpha d\in\mathcal A_r^{\mathrm{tight}}(x)
\right\}.
\]

A small \(\alpha_r^+\) indicates directional authority collapse even if unused torque remains in other directions. Because \(\mathcal A_r^{\mathrm{tight}}(x)\) is itself an affine image of a box (Section III's \(\mathcal Z_r\) construction applies equally once \(\tau_{\mathrm{base},r}\) and the tightening margin are subtracted from the torque box first), \(\alpha_r^+\) does not require enumerating facets or solving a general linear program: for a halfspace representation \(\{v:Av\le b\}\) of \(\mathcal A_r^{\mathrm{tight}}(x)\), the maximum feasible step along \(d\) from a feasible \(a_c\) is the closed form \(\alpha_r^+=\min_{j:\,(Ad)_j>0}(b_j-(Aa_c)_j)/(Ad)_j\), which is exactly the ray--halfspace intersection the implementation evaluates.

The implemented manager also reports finite-horizon QP feasibility. This is a useful recoverability diagnostic, but it is not membership in a certified viability kernel because no terminal invariant set or backup policy is constructed. Section VI therefore uses the term *near-boundary braking stress case* rather than point of no return.

---

# V. Conditional Certificate Transfer

Let the abstract behavior model satisfy \(z_{\ell+1}=F(z_\ell,a)\) for an action \(a\), and let

\[
\mathcal S=\{z:V(z)\le c\}.
\]

Rather than fix a single abstract policy, suppose a nonempty *certified action set* is defined at every \(z\in\mathcal S\),

\[
\mathcal K_{\mathrm{cert}}(z)
=
\left\{a: F(z,a)\oplus\mathcal E_\star\subseteq\mathcal S\right\}
\neq\emptyset,
\qquad
\forall z\in\mathcal S,
\]

so that \(\mathcal S\) is robustly invariant under any measurable selection \(\kappa\) with \(\kappa(z)\in\mathcal K_{\mathrm{cert}}(z)\) for all \(z\in\mathcal S\). The certificate designer may work with one such selection, but the robot-independent object is the set \(\mathcal K_{\mathrm{cert}}\), not a distinguished element of it; different robots realizing different members of \(\mathcal K_{\mathrm{cert}}(z)\) is expected, not a failure of transfer.

For robot \(r\), let \(\Pi_r(x)=z\) be the abstraction map and \(f_r^d\) its sampled physical dynamics. When no torque projection occurs, suppose the successor mismatch is bounded by

\[
\Pi_r\!\left(f_r^d(x,\tau_r^{\mathrm{pre}},F_h)\right)
-F\!\left(\Pi_r(x),a_{\mathrm{req}}\right)
\in
\mathcal D_{z,r}(x,a_{\mathrm{req}},F_h)
\]

for every \(a_{\mathrm{req}}\) the robot-specific manager may select, not only for one distinguished policy. The set \(\mathcal D_{z,r}\) may include discretization, interaction-force, state-estimation, interpolation, and secondary-channel errors.

**Theorem 1 (Conditional realizability-margin certificate transfer).**  
Let \(\mathcal K_{\mathrm{cert}}\) and \(\mathcal S\) be as above. For robot \(r\), consider an operating region \(\mathcal X_r\) with \(\Pi_r(\mathcal X_r)\subseteq\mathcal S\). Suppose that for every \(x\in\mathcal X_r\), every admissible interaction wrench, and every \(a_{\mathrm{req}}\in\mathcal K_{\mathrm{cert}}(\Pi_r(x))\) that the robot-specific manager may select:

\[
\tau_r^{\mathrm{pre}}(x,a_{\mathrm{req}})
\in
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}}),
\tag{1}
\]

\[
\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\subseteq\mathcal T_r,
\tag{2}
\]

\[
\mathcal D_{z,r}(x,a_{\mathrm{req}},F_h)
\subseteq\mathcal E_\star.
\tag{3}
\]

Assume all implementation effects are included in (1)--(3) and \(x_0\in\mathcal X_r\). Then actuator projection is inactive and

\[
\Pi_r\!\left(f_r^d(x,\tau_r^{\mathrm{app}},F_h)\right)
\in
F\!\left(\Pi_r(x),a_{\mathrm{req}}\right)\oplus\mathcal E_\star
\subseteq\mathcal S,
\]

the last inclusion following directly from \(a_{\mathrm{req}}\in\mathcal K_{\mathrm{cert}}(\Pi_r(x))\), with no separate robust-invariance premise about a specific policy required.

Consequently, if the physical successor remains in \(\mathcal X_r\) for \(i=1,\ldots,N\) whenever its abstraction remains in \(\mathcal S\),

\[
\Pi_r(x_{\ell+i})\in\mathcal S,
\qquad
i=0,\ldots,N,
\]

which is the finite-horizon guarantee that applies to the implemented manager, since \(\mathcal K_{\mathrm{cert}}\) is instantiated and enforced as a QP constraint (Remark below) rather than only checked after the fact; the experiments additionally audit the realizability conditions (1)--(2) and the sampled successor defect, since \(\mathcal K_{\mathrm{cert}}\)-membership alone does not by itself verify conditions (1)--(3) hold for the chosen \(a_{\mathrm{req}}\). The unbounded conclusion \(\Pi_r(x_k)\in\mathcal S\) for all \(k\ge0\) additionally requires the operating-region assumption to hold indefinitely, a separate and independent claim discussed below.

**Proof.** Conditions (1)--(2) imply \(\tau_r^{\mathrm{pre}}\in\mathcal T_r\); hence projection is the identity and \(\tau_r^{\mathrm{app}}=\tau_r^{\mathrm{pre}}\). Condition (3) places the concrete successor inside \(F(\Pi_r(x),a_{\mathrm{req}})\oplus\mathcal E_\star\), which lies in \(\mathcal S\) directly by the definition of \(\mathcal K_{\mathrm{cert}}\) since \(a_{\mathrm{req}}\in\mathcal K_{\mathrm{cert}}(\Pi_r(x))\). Finite induction from \(x_0\in\mathcal X_r\) over \(i=0,\ldots,N\) gives the stated horizon result; extending it to all \(k\ge0\) requires the operating-region assumption to hold at every step, which this argument alone does not establish. \(\square\)

**Remark (concrete instantiation).** The QP of Section IV enforces \(a_{\mathrm{req},i|\ell}\in\mathcal A_r^{\mathrm{tight}}(\hat x^0_{i|\ell})\cap\mathcal K_{\mathrm{cert}}(z_{i|\ell})\), so that any feasible solution provably satisfies both sets by construction rather than by a check applied after the fact. The instantiation used is deliberately simple. Let

\[
V(z)=\max\!\left\{\frac{|e_1|}{\mathrm{pos}_{\max}},\frac{|e_2|}{\mathrm{pos}_{\max}},\frac{|\dot e_1|}{\mathrm{spd}_{\max}},\frac{|\dot e_2|}{\mathrm{spd}_{\max}}\right\},
\qquad
\mathcal S=\{z:V(z)\le1\},
\]

an \(\infty\)-norm storage function chosen, in place of a quadratic \(z^\top Pz\), specifically because it keeps \(\mathcal S\) and \(\mathcal K_{\mathrm{cert}}\) linear and solvable by the same OSQP instance already used for \(\mathcal A_r^{\mathrm{tight}}\), without a QCQP solver. Here \(\mathcal S\) is exactly the existing position/speed operating region \(\mathcal X\) of Section IV. With \(\epsilon_{\mathrm{audit}}=0.03~\mathrm{m/s}\) standing in for the abstract radius of \(\mathcal E_\star\)---this numeric choice ties the prospectively enforced bound to the same value retrospectively checked by the audit of Section VI.D, not a general requirement of Theorem 1---the certified action set tightens only the one-step-ahead velocity block,

\[
\mathcal K_{\mathrm{cert}}(z)
=
\left\{a:\left|(Az+Ba)_{\dot e}\right|\le\mathrm{spd}_{\max}-\epsilon_{\mathrm{audit}}\right\}.
\]

Position is left untightened: only the velocity-space successor defect is measured and audited here, so tightening position would not be backed by a measured quantity. Because deceleration at the actuator limit removes \(\epsilon_{\mathrm{audit}}\) of speed within \(\epsilon_{\mathrm{audit}}/(\Delta t\,a_{\max})\ll1\) of one manager period for the parameters used, \(\mathcal K_{\mathrm{cert}}(z)\) is nonempty for every \(z\in\mathcal S\). Section VI.E reports a dedicated ablation showing this constraint changes the manager's output relative to leaving it out; in the eight main stress scenarios it does not bind, consistent with Table III's own finding that the audited defect leaves most of the certificate radius unused.

The region-persistence clause is likewise an explicit conditional assumption, not a consequence of the implemented finite-horizon QP. A recursively feasible terminal set or certified backup policy would be one way to establish it over an unbounded horizon. Without such a construction, the horizon in this study should be read as finite --- matching the manager's own \(N=12\)-step horizon --- rather than indefinite.

The theorem does not make feasibility universal. Each robot must verify \(\Pi_r\), \(\hat\tau_r\), \(\mathcal D_{\tau,r}\), \(\mathcal D_{z,r}\), and \(\mathcal X_r\). What transfers is the abstract object

\[
\left(F,\mathcal K_{\mathrm{cert}},\mathcal S,\mathcal E_\star\right).
\]

A directly testable norm specialization is

\[
\bar\eta_{\mathrm{disc}}
+\bar\eta_{\mathrm{hold}}
+\bar\eta_{\mathrm{sec}}
+L_F\bar\delta_F
+L_\tau\bar\delta_\tau
\le
\bar\epsilon_\star,
\]

where \(L_F\) bounds the one-step abstraction defect induced by force-model error and \(L_\tau\) bounds the defect induced by torque-realization error in the selected norm, together with

\[
\min_j
\left\{
\hat\tau_j-\tau_{\min,j},
\tau_{\max,j}-\hat\tau_j
\right\}
\ge
\|\bar\delta_\tau\|_\infty.
\]

The first inequality requires the certificate radius to dominate the successor defect; the second requires actuator margin to dominate torque-realization uncertainty.

If clipping becomes active, let

\[
\mathcal C_{\tau,r}(x,a_{\mathrm{req}})
=
\left\{
\operatorname{proj}_{\mathcal T_r}(\tau)-\tau:
\tau\in\hat\tau_r(x,a_{\mathrm{req}})\oplus\mathcal D_{\tau,r}(x,a_{\mathrm{req}})
\right\}.
\]

Here \(\mathcal L_r(x)\) is a verified linear or set-valued one-sample sensitivity operator that maps a torque-projection residual into the resulting abstraction-space successor perturbation. Its induced norm is bounded by the scalar gain \(L_\tau\) used above.

Transfer can then hold only if the enlarged defect

\[
\mathcal D_{z,r}
\oplus
\mathcal L_r(x)\mathcal C_{\tau,r}
\subseteq
\mathcal E_\star.
\]

The experiments use the more transparent no-clipping branch of Theorem 1. When the request leaves the tightened set, the final projection protects the actuator but no longer implies preservation of the requested behavior certificate.

A further direction, sketched here but not developed as a formal result, is task-channel dissipativity transfer: if the abstract certificate additionally supplied a storage function on the task-force channel, and the physical realization were shown power-consistent with it up to a bounded defect on the no-clipping branch, the resulting dissipation budget would transfer alongside the state certificate above. Realizing this rigorously requires relating the abstract storage function to a physical robot storage function, fixing sign conventions between the task-error convention \(e\) and physical interaction power, and bounding the clipping-branch saturation-power term via the directional-authority margin \(\alpha_r^+\); none of this is constructed, related to a physical storage function, or evaluated in the present study, and we leave it to future work rather than state it as a corollary.

---

# VI. Experiments

## A. Protocol

The deterministic benchmark uses a two-dimensional interaction task. The fast controller and final projection execute at \(1~\mathrm{kHz}\); the manager executes at \(50~\mathrm{Hz}\) with horizon \(N=12\). Three configuration-dependent realization maps are evaluated: a planar 2R map, an FR3-inspired surrogate, and a six-axis-arm surrogate. The latter two reproduce different actuator geometries and limits but are not manufacturer-accurate rigid-body models.

The same behavior dynamics, predictive objective, and sampled audit threshold

\[
\epsilon_{\mathrm{audit}}
=
0.03~\mathrm{m/s}
\]

are used throughout. This scalar is an empirical acceptance threshold for the observed norm \(\|d_z\|_2\); it is deliberately not denoted by the certified set \(\mathcal E_\star\) of Section V. The simulation does not construct \(V\), \(\mathcal S\), or a workspace-wide proof of robust invariance. The fast-controller interfaces are PD, impedance, a small policy trained by deterministic evolution strategy, a fixed-feature neural policy fitted to impedance demonstrations, and an AI-conditioned motion primitive executed by a PD servo. The final two learned cases test the command/preview interface; they are not evidence of semantic AI safety or improved policy quality.

The 111 deterministic configurations comprise 40 scenario cases, 30 controller-interface cases, 24 cross-realization cases, and 17 ablations. The 40 scenario cases are eight scenarios evaluated with five channels: four realization architectures—direct clipping, a reactive \(1~\mathrm{kHz}\) projection, a scalar reference governor followed by the same reactive projection, and the proposed horizon-wide correction with final actuator projection—plus one nominal diagnostic without torque projection. The stress scenarios are no saturation, slow saturation, sudden disturbance, directional authority collapse, near-boundary braking, model mismatch, preview mismatch, and a dedicated horizon-ramp scenario used only for the horizon-constraint ablation of Section VI.E. A ninth scenario, starting with velocity already above the certificate-tightened speed bound, is used only for the certified-action-set ablation and does not enter the scenario or cross-realization matrices.

We report pre-projection torque excess, applied torque excess, workspace excess, behavior-realization RMSE, warning lead time, directional authority, sampled successor defects, and computation time. For a two-dimensional residual \(r_k\), RMSE is the pooled component-wise quantity

\[
\operatorname{RMSE}_{\mathrm{comp}}
=
\sqrt{\frac{1}{2K}\sum_{k=1}^{K}\|r_k\|_2^2}.
\]

The successor defect in Table III is instead the maximum vector norm and is not directly comparable with the component-wise RMSE. Applied torque remains within its box whenever the final projection is active; zero pre-projection excess is one sampled check for the no-clipping branch of Theorem 1, not proof of the abstract robust-invariance premise.

## B. Anticipatory saturation management

![Directional-authority stress case with a common impedance-controller interface.](results/directional_authority_results.png){width=70%}

Fig. 1 shows the directional-authority stress case. Direct clipping exceeds the actuator-feasible request set and the workspace bound. The predictive methods intervene earlier, but the vector correction preserves the workspace constraint while retaining a visible intervention residual.

![Near-boundary braking stress case; no viability kernel is inferred from this trajectory.](results/near_boundary_braking_results.png){width=70%}

Fig. 2 shows the near-boundary braking case, which starts with an outward velocity close to the position boundary under a shrinking torque budget. Direct clipping lets the position overshoot the boundary and settle outside it. The reactive projection, scalar reference governor plus projection, and proposed manager all arrest the position at the boundary. Because the experiment does not compute a viability kernel or terminal invariant set, it supports only finite-horizon constraint handling in this stress case.

![Scenario-level comparison of the four realization architectures.](results/scenario_summary.png){width=95%}

Fig. 3 summarizes the method-level trends, while Table I reports the clipping and proposed results. The proposed manager preserves the no-saturation behavior and prevents workspace violations in the slow-saturation, directional-collapse, and near-boundary braking scenarios. Warning precedes the limiting event by \(0.412\), \(0.339\), and \(0.566~\mathrm{s}\), respectively. The reactive projection and scalar reference governor plus projection also satisfy the sampled constraints in these three cases, with warning leads of \(0.419\) and \(0.395~\mathrm{s}\) for slow saturation and \(0.340\) and \(0.306~\mathrm{s}\) for directional collapse. Because the manager only intervenes when the nominal rollout is actually infeasible (Section IV), this lead time falls between the two baselines rather than exceeding both: intervention is no longer triggered partly by the smoothing term itself, only by genuine infeasibility. The remaining lead-time differences across all three methods are at most a few tens of milliseconds and should not be read as evidence that any one architecture anticipates saturation earlier than the others; the value of prediction here is in what happens after the warning, not its timing. In the near-boundary braking case, all three methods warn at approximately \(0.57~\mathrm{s}\).

| Scenario | Pre-proj. C/P (Nm) | Workspace C/P (mm) | Lead (s) | QP feasible | Audit |
|---|---:|---:|---:|---:|---:|
| No saturation | 0.000 / 0.000 | 0.000 / 0.000 | -- | 100% | Yes |
| Slow saturation | 0.000 / 0.000 | 78.970 / 0.001 | 0.412 | 100% | Yes |
| Sudden disturbance | 7.208 / 11.976 | 0.000 / 0.000 | 0.084 | 92.5% | No |
| Directional collapse | 0.693 / 0.000 | 103.914 / 0.003 | 0.339 | 100% | Yes |
| Near-boundary braking | 0.000 / 0.000 | 52.537 / 0.011 | 0.566 | 100% | Yes |
| Model mismatch | 12.665 / 1.981 | 190.434 / 193.598 | 0.558 | 45.0% | No |
| Preview mismatch | 19.917 / 4.409 | 190.191 / 344.265 | 0.170 | 40.0% | No |

: Scenario results.

In Table I, C/P denotes clipping/proposed, QP feasibility is the fraction of \(50~\mathrm{Hz}\) updates whose horizon problem is feasible, and Audit denotes whether all sampled realization checks pass. The QP is feasible at every update in the four successful stress cases, but only \(74/80\), \(36/80\), and \(32/80\) updates in sudden disturbance, model mismatch, and preview mismatch, respectively. Infeasible updates use the reactive fallback defined in Section IV, so the reported “proposed” trajectory includes that fallback and must not be interpreted as horizon-MPC behavior throughout.

The negative cases are not merely inconclusive. Under preview mismatch, the proposed correction acts on a force forecast that misses the sign change inside the horizon, increasing workspace excess from \(190.191~\mathrm{mm}\) with clipping to \(344.265~\mathrm{mm}\). Under model mismatch it is also slightly worse, \(193.598\) versus \(190.434~\mathrm{mm}\). The sampled interface audit rejects both cases, correctly indicating that the anticipatory correction is not trustworthy there.

The sudden-disturbance result illustrates why the slow manager cannot be the only protection layer. The wrench changes without advance information, while the implemented preview holds the measured wrench constant over the horizon; the resulting correction can therefore be misaligned with the short impulse and raises pre-projection excess from \(7.208\) to \(11.976~\mathrm{Nm}\). The final projection keeps the applied actuator command inside its box, but the pre-projection request is infeasible and the observed successor defect exceeds \(\epsilon_{\mathrm{audit}}\). These results identify operating conditions for which Theorem 1 cannot be invoked.

## C. Controller-interface substitution

![Behavior-realization residuals for the five nominal-controller interfaces.](results/controller_transfer.png){width=95%}

As illustrated in Fig. 4, the manager formulation and weights are unchanged across controller interfaces. Under no saturation, correction RMSE is below \(0.01~\mathrm{m/s^2}\) for four of the five controllers. The small evolution-strategy policy is the exception, with correction RMSE \(\approx1.01~\mathrm{m/s^2}\). Its limited training set does not cover the test trajectory symmetrically, and the learned policy retains a positive-\(y\) command bias. This is an observed generalization error rather than evidence of a specific failure of the evolution-strategy algorithm. The manager holds the state inside the workspace box, but is compensating for a biased interface rather than remaining inactive. This case is retained as an interface stress test: the architecture is not intended to repair behavior-policy quality, although its running constraints may incidentally reject a biased request.

Under slow saturation, realization RMSE is nearly equal to correction RMSE for every proposed-controller case (for example, \(0.732\) versus \(0.726~\mathrm{m/s^2}\) for PD). The reported residual is therefore almost entirely the manager's deliberate intervention, not unmodeled tracking error. Clipping appears to have a smaller realization RMSE because it follows the nominal request instead of enforcing the workspace constraint; in the impedance case that apparent fidelity accompanies \(78.970~\mathrm{mm}\) of workspace violation, whereas the proposed manager leaves \(0.001~\mathrm{mm}\). All five proposed cases pass the sampled interface audit (Table II). The results demonstrate substitution at the command/preview interface; they do not certify an arbitrary learned policy whose preview error is unbounded.

| Interface | Realization RMSE (\(\mathrm{m/s^2}\)) | Correction RMSE (\(\mathrm{m/s^2}\)) | Lead (s) | Excess (mm) |
|---|---:|---:|---:|---:|
| PD | 0.732 | 0.726 | 0.371 | 0.001 |
| Impedance | 1.064 | 1.060 | 0.412 | 0.001 |
| Trained policy | 0.652 | 0.652 | 1.207 | 0.016 |
| Fitted neural policy | 0.655 | 0.649 | 0.356 | 0.001 |
| AI-conditioned proxy | 0.835 | 0.830 | 0.359 | 0.006 |

: Controller substitution under slow saturation.

## D. Sampled interface audit across realization maps

![Observed successor defects versus the common audit threshold.](results/sampled_interface_audit.png){width=88%}

The behavior model, predictive objective, and \(0.03~\mathrm{m/s}\) audit threshold remain unchanged across the three realization maps. Only \(\hat\tau_r\), the actuator box, and the observed torque- and successor-error checks change. As shown in Fig. 5 and Table III, every observed successor defect is below \(0.0077~\mathrm{m/s}\), leaving more than \(0.0223~\mathrm{m/s}\) of the audit allowance unused.

| Realization map | Max. observed (m/s) | Unused audit (m/s) | Min. bound slack (Nm) |
|---|---:|---:|---:|
| Planar 2R | 0.007456 | 0.022544 | 0.002523 |
| FR3-inspired surrogate | 0.007696 | 0.022304 | \(8.66\times10^{-6}\) |
| Six-axis-arm surrogate | 0.007696 | 0.022304 | \(9.96\times10^{-5}\) |

: Sampled cross-realization interface audit.

The last column is not remaining actuator authority. It is the smallest sampled containment residual \(\bar\delta_{\tau,j}-|\tau_j^{\mathrm{pre}}-\hat\tau_j|\). For the cross-realization cases, the componentwise bounds use

\[
\bar\delta_\tau
=
0.03\,\mathbf 1_{n_r}
+0.008\max\!\left(|H_r(0,0)|,0.25\right)
\left|a_{\mathrm{req}}-\frac{F_h}{m_r}\right|,
\]

where the maximum is elementwise. These bounds range from \(0.0300\) to \(0.0926~\mathrm{Nm}\) over the sampled audit. The near-zero FR3 and six-axis residuals mean that the deterministic injected errors nearly attain their envelopes; they do not mean that the tightening is zero. The minimum planned actuator margins are \(0.233\), \(0.679\), and \(0.741~\mathrm{Nm}\) for the planar, FR3-inspired, and six-axis maps, respectively. Thus error-bound containment is the tight numerical check, while actuator authority is not binding on these sampled trajectories. Because the injected errors are constructed from the same envelopes, this is a consistency audit rather than independent validation of \(\mathcal D_{\tau,r}\).

This is a sampled-trajectory interface audit, not independent uncertainty validation, an analytic whole-workspace proof, or an experimental proof of robust invariance. It demonstrates only that the same numerical audit threshold and checking mechanism can be applied to multiple robot-specific realization maps. A full certificate-transfer experiment would additionally require independently identified uncertainty sets and a verified \((F,\mathcal K_{\mathrm{cert}},\mathcal S,\mathcal E_\star)\).

## E. Ablations and computation

![Paired ablations of the predictive and fast-path implementation choices.](results/ablation_summary.png){width=95%}

Fig. 6 summarizes the ablations. Constraining all predicted moves eliminates planned torque excess in the horizon-ramp case. Constraining only the first move preserves the currently applied command but leaves a \(3.587~\mathrm{Nm}\) future violation and produces \(31.180~\mathrm{mm}\) workspace excess. This result directly verifies the predictive-horizon constraint requirement.

Removing uncertainty tightening creates \(0.0848~\mathrm{Nm}\) pre-projection excess. The final projection hides that excess at the applied actuator channel but does not restore the no-clipping refinement condition. Conversely, disabling the final projection under sudden disturbance produces \(10.538~\mathrm{Nm}\) applied excess, confirming that prediction and high-rate protection have distinct responsibilities.

Removing the rate-smoothing term from the objective leaves correction RMSE under slow saturation statistically unchanged (\(1.0604\) versus \(1.0603~\mathrm{m/s^2}\) with smoothing on): with the exact-pass-through bypass of Section IV already limiting intervention to genuinely infeasible steps, the smoothing weight's remaining effect is confined to shaping the QP's solution during those already-required interventions, and this gradually ramping scenario does not exercise that distinction. Its role would be expected to differ under a more abrupt correction; that comparison is left to future work.

The certified-action-set ablation isolates \(\mathcal K_{\mathrm{cert}}\)'s effect directly. Starting from an initial speed of \(0.580~\mathrm{m/s}\), already above the certificate-tightened bound of \(0.570~\mathrm{m/s}\) but below the untightened \(0.600~\mathrm{m/s}\) limit, with the constraint enforced the manager holds the peak speed at the initial \(0.580~\mathrm{m/s}\) and never allows it to increase; with the constraint removed, the same scenario reaches \(0.597~\mathrm{m/s}\), consistent with the untightened bound rather than the certificate margin. \(z_0\) itself is not constrained, so neither variant can undo an already-out-of-budget start; the constrained case's inability to grow past it, against the unconstrained case's approach to the untightened limit, is exactly the QP-enforced membership \(a_{\mathrm{req},i|\ell}\in\mathcal K_{\mathrm{cert}}(z_{i|\ell})\) at work. This constraint does not bind in the eight main stress scenarios, whose peak speeds stay well under \(0.570~\mathrm{m/s}\)---consistent with Table III's own finding that the successor defect leaves most of the \(0.03~\mathrm{m/s}\) certificate radius unused there.

The preview and implementation ablations also identify non-results. Zero-force preview increases workspace excess from \(344.265\) to \(873.921~\mathrm{mm}\), but no preview option restores viability in the severe mismatch case. In this reduced-order model, recomputing the fast realization map does not outperform cached torque, and updating the realization map is numerically indistinguishable from freezing it. Benefits from these mechanisms therefore remain to be demonstrated on nonlinear rigid-body systems.

Increasing the manager rate from \(50\) to \(100~\mathrm{Hz}\) could shorten the interval over which an obsolete correction is held, but it would not predict an unannounced wrench change or repair an incorrect force model. Addressing the sudden-disturbance limitation therefore requires both timing and information improvements—for example, event-triggered re-solving, bounded disturbance observers, or preview uncertainty propagated into the horizon—while retaining the \(1~\mathrm{kHz}\) final projection.

In the final regenerated run, the manager's median-of-run-medians is \(1.695~\mathrm{ms}\), and its worst observed maximum is \(13.398~\mathrm{ms}\), below its \(20~\mathrm{ms}\) period. The fast path has a median-of-run-medians of \(121.2~\mu\mathrm{s}\) but a worst observed maximum of \(2.466~\mathrm{ms}\), exceeding its \(1~\mathrm{ms}\) nominal period. This scheduling outlier, absent from some regenerated runs and present in others at varying severity, is characteristic of the non-real-time Python implementation and establishes typical throughput, not hard real-time execution.

---

# VII. Discussion

The predictive optimization statement and empirical audit threshold remain unchanged across the implemented controller interfaces and realization maps. Physical feasibility does not transfer automatically: each robot must reconstruct and verify its actuator-feasible set and its refinement-error bounds.

This distinction also clarifies the relation to a reference governor or predictive safety filter. Those architectures can adopt the same behavior coordinates. What separation adds is an explicit proof boundary: a reusable behavior certificate on one side and a checkable robot-specific realization contract on the other. In the present study only an interface mechanism is sampled; the abstract certificate and independent uncertainty identification are absent. If the torque uncertainty exceeds the available margin, or an observed successor defect exceeds \(\epsilon_{\mathrm{audit}}\), the experimental audit rejects the case. This rejection is evidence of a failed sampled check, not proof about the theoretical set \(\mathcal E_\star\).

The final high-rate projection is essential but should not be confused with behavior preservation. It enforces the actuator box after an unexpected disturbance, while Theorem 1 states when projection remains inactive and the requested closed-loop behavior is actually realized. The mismatch experiments show that these two properties can diverge.

For model-based physical AI, the architecture provides a runtime boundary between behavior generation and physical realization. Learned, diffusion-based, or language-conditioned modules may propose behavior, while the realization model evaluates what the current robot can execute. This contract does not certify the semantics or intent of an AI-generated command; it certifies only the modeled physical refinement inside the verified operating region.

Several limitations remain. First, the robot substitutions are reduced-order actuator-geometry surrogates rather than full rigid-body or hardware systems. Second, the reported errors are observations along experiment trajectories rather than certified bounds over a continuous workspace. Third, only the abstract side of Theorem 1's premise is instantiated: \(\mathcal S\) and \(\mathcal K_{\mathrm{cert}}\) are concrete and enforced, using an \(\infty\)-norm storage function tightened on velocity alone for one 2-D reduced-order model, but the physical containment \(\mathcal D_{z,r}\subseteq\mathcal E_\star\) that robust invariance also requires is sampled along experiment trajectories, not established as a workspace-wide proof; “audit threshold” and “observed defect” are therefore kept distinct from the theoretical certificate set. Fourth, the benchmark omits orientation, redundant null-space tasks, sensor delay, contact transitions, state-estimation uncertainty, human-participant validation, and any construction or evaluation of the task-channel dissipativity sketch of Section V. Fifth, the reference-governor baseline includes a reactive projection and is an architecture-level comparator, not a reproduction of every established governor design. Its scalar command parameterization is especially restrictive in the directional-collapse case; a directional or vector reference governor would be expected to narrow the reported lead-time difference. Sixth, every reported case is a single deterministic trajectory per scenario, controller, and realization map; the study does not sweep disturbance magnitude, timing, or sensor noise to characterize how close a successful case is to its failure boundary, so the reported margins are point estimates rather than statistically characterized safety margins. Finally, the theorem's region-persistence clause and recursive feasibility are the same unresolved gap at different levels: discharging that assumption and defining a certified point of no return require a formally constructed terminal invariant set or backup policy. Finite-horizon running constraints alone do not imply global recoverability.

---

# VIII. Conclusion

This paper introduced a predictive realization architecture for actuator-limited fast robot controllers. The nominal controller remains at \(1~\mathrm{kHz}\), while a slower MPC layer forecasts robot-specific loss of realizability and modifies the requested acceleration before clipping. The proposed separation does not make actuator feasibility universal. It identifies which theoretical object may be reused—a certified action set stated in behavior coordinates—and which objects must be verified again—the realization map, actuator margin, uncertainty bounds, and operating region. A deliberately simple instantiation of that certified action set is enforced inside the QP itself, so the theorem's key hypothesis holds by construction rather than by a check applied afterward.

Across 111 deterministic reduced-order cases, full-horizon correction detects a future violation missed by a first-step constraint and accepts five nominal-controller interfaces. A sampled interface audit applies the same successor-defect threshold across three realization maps. Slow saturation, directional authority collapse, and near-boundary braking violations are prevented within the tested region. Abrupt disturbance and severe mismatch expose cases where the sampled audit fails, the QP frequently becomes infeasible, and the reactive fallback plus final actuator projection remain available. A dedicated ablation confirms the certified action set changes the manager's output when it actually binds; it does not bind in the eight main stress scenarios, whose successor defects fall well inside the certificate margin.

Future work will replace the surrogate maps with full rigid-body systems, certify uncertainty sets over continuous workspaces, construct terminal invariant sets, and evaluate the architecture in a real-time hardware loop. Within the present model, three gaps remain open: the QP's state prediction assumes each \(a_{\mathrm{req},i|\ell}\) is held fixed for its full \(\Delta t\), so it does not bound the intra-step drift of the fast controller's own re-evaluated request, which is what the sudden-disturbance failure mode exposes rather than any lack of force compensation in the realization map; the instantiated certified action set tightens only the one-step velocity block using an \(\infty\)-norm storage function, not a general quadratic or position-aware certificate, and was constructed for one 2-D reduced-order model rather than derived from a workspace-wide invariance proof; and the task-channel dissipativity sketch of Section V requires a constructed physical storage function and a directional-authority bound on the clipping-branch saturation-power term before it can be stated as a corollary. These steps, together with hardware validation, are necessary before claiming hardware-level certificate transfer or interaction safety.

---

# References

\begingroup
\small
\setlength{\parskip}{0.35em}

[1] N. Hogan, “Impedance Control: An Approach to Manipulation—Part I: Theory,” *Journal of Dynamic Systems, Measurement, and Control*, vol. 107, no. 1, pp. 1--7, 1985, doi: 10.1115/1.3140702.

[2] O. Khatib, “A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation,” *IEEE Journal on Robotics and Automation*, vol. 3, no. 1, pp. 43--53, 1987, doi: 10.1109/JRA.1987.1087068.

[3] E. Garone, S. Di Cairano, and I. Kolmanovsky, “Reference and Command Governors for Systems with Constraints: A Survey on Theory and Applications,” *Automatica*, vol. 75, pp. 306--328, 2017, doi: 10.1016/j.automatica.2016.08.013.

[4] A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, “Control Barrier Functions: Theory and Applications,” in *Proceedings of the European Control Conference*, pp. 3420--3431, 2019, doi: 10.23919/ECC.2019.8796030.

[5] K. P. Wabersich and M. N. Zeilinger, “Linear Model Predictive Safety Certification for Learning-Based Control,” in *Proceedings of the IEEE Conference on Decision and Control*, 2018.

[6] Y.-Y. Cao, Z. Lin, and D. G. Ward, “An Anti-Windup Approach to Enlarging Domain of Attraction for Linear Systems Subject to Actuator Saturation,” *IEEE Transactions on Automatic Control*, vol. 47, no. 1, pp. 140--145, 2002, doi: 10.1109/9.981734.

[7] Y.-Y. Cao, Z. Lin, and D. G. Ward, “Anti-Windup Design of Output Tracking Systems Subject to Actuator Saturation and Constant Disturbances,” *Automatica*, vol. 40, no. 7, pp. 1221--1228, 2004, doi: 10.1016/j.automatica.2004.02.012.

[8] A. Girard and G. J. Pappas, “Approximation Metrics for Discrete and Continuous Systems,” *IEEE Transactions on Automatic Control*, vol. 52, no. 5, pp. 782--798, 2007.

[9] A. Girard and G. J. Pappas, “Hierarchical Control System Design Using Approximate Simulation,” *Automatica*, vol. 45, no. 2, pp. 566--571, 2009.

[10] P. Nuzzo, J. B. Finn, A. Iannopollo, and A. Sangiovanni-Vincentelli, “Contract-Based Design of Control Protocols for Safety-Critical Cyber-Physical Systems,” in *Proceedings of the Design, Automation and Test in Europe Conference and Exhibition*, 2014.

[11] M. Sharifi, H. Salarieh, S. Behzadipour, and M. Tavakoli, “Nonlinear Model Reference Adaptive Impedance Control for Human--Robot Interactions,” *Control Engineering Practice*, vol. 32, pp. 9--27, 2014, doi: 10.1016/j.conengprac.2014.07.001.

[12] L. Roveda, S. Haghshenas, M. Caimmi, N. Pedrocchi, and L. M. Tosatti, “Q-Learning-Based Model Predictive Variable Impedance Control for Physical Human--Robot Collaboration,” *Artificial Intelligence*, vol. 312, art. 103771, 2022, doi: 10.1016/j.artint.2022.103771.

[13] K. Haninger, M. Radke, A. Vick, and J. Krüger, “Model Predictive Impedance Control with Gaussian Processes for Human and Environment Interaction,” *Robotics and Autonomous Systems*, vol. 165, art. 104431, 2023, doi: 10.1016/j.robot.2023.104431.

[14] A. S. Anand, S. D. S. Mohan, and B. Thananjeyan, “Model-Based Variable Impedance Learning Control for Robotic Manipulation,” *Robotics and Autonomous Systems*, vol. 170, art. 104531, 2023, doi: 10.1016/j.robot.2023.104531.

[15] T. Xue, N. Tsagarakis, and G. Xin, “Model Predictive Variable Impedance Control Towards Safe Robotic Interaction in Unknown Disturbance-Rich Environments,” *Robotics and Autonomous Systems*, vol. 189, art. 104961, 2025, doi: 10.1016/j.robot.2025.104961.

[16] K. P. Wabersich and M. N. Zeilinger, “A Predictive Safety Filter for Learning-Based Control of Constrained Nonlinear Dynamical Systems,” *Automatica*, vol. 129, art. 109597, 2021, doi: 10.1016/j.automatica.2021.109597.

\endgroup
