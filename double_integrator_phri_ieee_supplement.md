# Supplementary Material — Toward Interaction Dynamics: A Predictive Framework for Safe Physical Human-Robot Interaction

**Yongyan Cao$^{1}$ and Jinshan Tang$^{2}$**

This document accompanies `double_integrator_phri_ieee.md` and contains: the full controllability/detectability argument and LMI-based stabilizability proof for the LPV backbone, the parameter table, and additional experiments that supplement, but are not required to follow, the main paper's core argument — free-space tracking across four 3-D circle planes, the waypoint-hold benchmark (redundant with Benchmark I's conclusions), the time-varying-force horizon-prediction study, the human-force magnitude/shape sweep, and the impedance-backbone correction-authority ablation. Table numbers here are prefixed "S" and are independent of the main paper's numbering.

---

## S1. Controllability and Detectability of the LPV Backbone

**Remark S1 (Controllability and Detectability).** The LPV system (8) is uniformly controllable across all configurations $\rho_k$ in the singularity-free workspace. Since $\Lambda(q)$ in (2) is symmetric positive-definite for all $q$ away from kinematic singularities, $\Lambda^{-1}(q)$ is full rank, so $B_d(\rho_k)$ in (8) has full column rank 3 (via its velocity block). With $A_d$ mixing velocity into position, the position rows of $B_d$ and $A_d B_d$ differ by $-\Lambda^{-1}\Delta t^2$ (full rank), so the controllability matrix $\mathcal{C} = [B_d \mid A_d B_d]$ has rank 6 for all $\rho_k$. If $Q \succeq 0$, $R \succ 0$, and $(Q^{1/2},A_d)$ is detectable, the discrete algebraic Riccati equation (DARE) admits a stabilizing solution at every fixed configuration, and the infinite-horizon LQR gain
$$K_\infty = (R + B_d^\top P_\infty B_d)^{-1}B_d^\top P_\infty A_d$$
is well-defined everywhere. For the augmented estimator, observability of $[e;\dot e]$ together with full column rank of $B_d$ makes the constant disturbance state detectable: if a mode is unobservable from $C=[I_6\;0]$, its $x_e$ component is zero; the augmented dynamics then require $B_d d=0$, hence $d=0$.

This is the basis for Corollary 1's DARE-solvability claim and for the LPV-backbone stabilizability result of §S2.

---

## S2. Proof of Theorem 3 — Quadratic Stabilizability of the Interaction-Dynamics Backbone

**Theorem S1 (Quadratic Stabilizability of the Interaction-Dynamics Backbone).** Consider the LPV interaction dynamics $x_{e,k+1} = A_d\,x_{e,k} + B_d(\rho_k)\,F_{\text{mpc},k}$ with constant $A_d$ and $B_d(\rho)$ as in (8), and suppose the operational-space inverse-inertia stays in a compact polytope over the operating region of interest, $\Lambda^{-1}(q)\in\mathcal P=\operatorname{conv}\{L_1,\dots,L_V\}$ (e.g. the entry-wise box with $0\prec\lambda_{\min}I\preceq\Lambda^{-1}\preceq\lambda_{\max}I$) — a certified bound on $\Lambda^{-1}$, either established analytically over the entire singularity-free workspace or, as instantiated in Remark S3 below, sampled over a specific operating region — with vertex input matrices $B_v=[-\tfrac{\Delta t^2}{2}L_v;\,-L_v\Delta t]$ (affine in $\Lambda^{-1}$, so the polytope structure is preserved). If there exist $Q=Q^\top\succ0$ and $Y\in\mathbb R^{3\times6}$ such that, for every vertex $v=1,\dots,V$,
$$\begin{bmatrix} Q & (A_d Q + B_v Y)^\top \\ A_d Q + B_v Y & Q \end{bmatrix}\succ 0, \tag{S1}$$
then the *fixed* state-feedback law $F_\text{mpc}=K x_e$ with $K=YQ^{-1}$ makes $V(x)=x^\top P x$, $P=Q^{-1}$, a *common* quadratic Lyapunov function: $V(x_{e,k+1})\le \gamma\,V(x_{e,k})$ with $\gamma<1$ for *every* admissible configuration trajectory $\{\rho_k\}\subset\mathcal P$, so this one fixed linear feedback law is exponentially stabilizing uniformly over $\mathcal P$ — not only at a frozen configuration. This is a statement about the LPV *backbone* under a single certified feedback gain, not directly about the finite-horizon receding-horizon MPC actually deployed (see the Scope remark below).

*Proof.* $B_d(\rho)$ is affine in $\Lambda^{-1}(\rho)$, so for any $\rho$ with $\Lambda^{-1}(\rho)=\sum_v\alpha_v L_v$ ($\alpha_v\ge0$, $\sum_v\alpha_v=1$) we have $A_d Q + B_d(\rho)Y=\sum_v\alpha_v(A_d Q + B_v Y)$. The block matrix in (S1) is affine in this quantity, and the positive-semidefinite cone is convex, so (S1) holds for all $\rho\in\mathcal P$, not only at the vertices. A Schur complement on (S1) gives $(A_d Q + B_d(\rho)Y)\,Q^{-1}(A_d Q + B_d(\rho)Y)^\top\prec Q$; writing $A_\text{cl}(\rho)=A_d+B_d(\rho)K$ with $K=YQ^{-1}$ yields $A_\text{cl}(\rho)Q A_\text{cl}(\rho)^\top\prec Q$, i.e. $A_\text{cl}(\rho)^\top P A_\text{cl}(\rho)-P\prec0$ with $P=Q^{-1}$, for all $\rho\in\mathcal P$. Hence $V(x)=x^\top P x$ decreases along every trajectory regardless of how $\rho_k$ varies, which is exponential stability of this fixed-gain LPV closed loop. Composing with the detectable integrating-disturbance estimator of Theorem 2, the tracking error is offset-free at each constant-contact equilibrium the trajectory visits, *for this fixed-gain closed loop*. $\square$

This is the polytope-level counterpart of the per-configuration DARE solvability of Remark S1 and the continuous scheduled law of Corollary 1: it certifies that the interaction-dynamics *representation* — enabled by the constant $A_d$, which confines all configuration variation to the affine $B_d(\rho)$ and makes (S1) a finite vertex program — is quadratically stabilizable by a single Lyapunov function over $\mathcal P$, under one fixed certified gain.

**Scope.** Theorem S1 establishes exponential stability of the *nominal predictive backbone under one fixed, LMI-certified linear feedback gain* — the disturbance-free LPV closed loop $(A_d,B_d(\rho))$, with the input constraint inactive — not workspace-wide stability of the complete deployed controller. In particular it does **not** by itself analyze: the disturbance estimator (offset-free tracking under the Kalman estimator is Theorem 2's separate result, composed with Theorem S1 only informally, in the last line of the proof above); the finite-horizon receding-horizon re-solve, whose realized policy at each step is the QP's actual minimizer, not the certified fixed $K$ (the unconstrained QP realizes *a* member of this same class of linear state feedback, Theorem 1/Corollary 1, but not necessarily the specific certified $K$); or the safety and passivity filters (Theorem 4, Proposition 2). The complete controller's closed-loop guarantees are therefore this *collection* of separately-scoped results about different feedback laws, not one monolithic proof that the deployed MPC is workspace-stable.

**Remark S2 (Rate-independence and the frozen horizon).** The certificate uses a *single, parameter-independent* Lyapunov matrix $P=Q^{-1}$ shared by all vertices. A common-$P$ quadratic-stability certificate is robust to *arbitrary* time variation of $\rho_k\in\mathcal P$ — including arbitrarily fast changes — with no assumption on the parameter-variation rate $\|\rho_{k+1}-\rho_k\|$: the Lyapunov decrease holds at every vertex and hence, by convexity of (S1) in $\Lambda^{-1}$, for any $\rho_k\in\mathcal P$ regardless of how it moves, *provided the certified fixed $K$ is what is actually applied*. This gives a *structural reason*, not a proof, for why the frozen-$\Gamma$ approximation used online (built from the current $B_d(q_k)$ and held over the $N$-step horizon) is unlikely to be destabilizing: the gap between $B_d(q_k)$ and the true future $B_d(q_{k+i})$ is a bounded variation *inside* $\mathcal P$, and a fixed-$K$ closed loop over $\mathcal P$ would not be destabilized by that variation regardless of its rate. This motivates, but does not establish, that the deployed receding-horizon policy — which re-solves a QP rather than applying the certified $K$ — inherits the same robustness; the frozen horizon shapes only the *predicted* trajectory, while the receding-horizon re-solve keeps the *applied* first move matched to the current $\rho_k$, but no result in this paper proves the resulting closed loop shares Theorem S1's rate-independence.

**Remark S3 (Numerical instantiation of $\mathcal P$).** For the FR3, sampling $\Lambda^{-1}(q)$ along the §VI circular benchmark (4700 samples) gives $\operatorname{eig}\Lambda^{-1}\in[0.059,0.354]\ \text{kg}^{-1}$ (task inertia $2.8$–$16.9$ kg). The resulting entry-wise box has 64 corners in general, but not every corner of an independent per-entry box need be positive-definite; we discard any that are not (none were, here — all 64 corners were confirmed SPD, minimum eigenvalue $\approx0.023\ \text{kg}^{-1}$ across vertices, comfortably above the regularization floor). This instantiates $\mathcal P$ over the *sampled operating region traversed by the benchmark trajectory*, not a verified bound over the entire singularity-free workspace; extending the certified range to other trajectories or the full workspace would need re-sampling or an analytic eigenvalue bound, neither done here. Over this $\mathcal P$ ($\Delta t=10$ ms), the minimum-condition-number common-$P$ certificate at guaranteed rate $\rho\le0.996$ is feasible, returning a well-conditioned common $P$ ($\operatorname{cond}P\approx2.2$) with spectral radius $\le0.996$ and strictly negative Lyapunov increment ($\max\operatorname{eig}(A_\text{cl}^\top P A_\text{cl}-P)\approx-5\times10^{-3}$) at every vertex, *for the certified fixed $K=YQ^{-1}$* — the realized MPC, with its far larger tracking weights and different (receding-horizon) policy, is not shown to converge at this rate, only observed empirically to converge faster. This establishes the qualitative backbone-stabilizability claim rigorously over the sampled region rather than empirically, and is reproducible from the released script (`lmi_workspace_stability_probe.py`).

---

## S3. Parameters

Every numeric constant used across the experiments of the main paper (§VI) and this supplement is consolidated below, cross-checked against the released code (`simulation/impedance_mpc.py`, `simulation/phri.py`, `cloud_verify/lib/fr3_hardware_interface.py`) rather than restated from memory. Unless a table or figure caption states otherwise, these are the values used throughout.

*Table SI — Controller and Estimator Parameters*

| Parameter | Symbol | Value | Used in |
|---|---|---|---|
| Prediction horizon | $N$ | 10 | Layer 2 QP, all controllers |
| MPC sample time (100 Hz variant) | $\Delta t$ | 10 ms | C4/C5/G4/G5, most tables |
| MPC sample time (500 Hz variant) | $\Delta t$ | 2 ms | C6/C7/G6/G7, C8 |
| Inner feedforward/orientation/null-space rate | — | 1 kHz | all controllers |
| Position stage weight | $Q_\text{pos}$ | $2\times10^4\,I_3$ | Layer 2 cost $Q$ |
| Velocity stage weight | $Q_\text{vel}$ | $50\,I_3$ | Layer 2 cost $Q$ |
| Terminal cost scale | $\gamma$ | 5 | $Q_f=\gamma Q$ |
| Effort weight | $R$ | $10^{-6}\,I_3$ | Layer 2 cost |
| Corrective-force bound | $F_\text{max}$ | 150 N | (9c), all DI-MPC variants |
| FR3 joint torque limits | $\tau_\text{max}$ | $[87,87,87,87,12,12,12]$ Nm | (9b), hardware clipping |
| $\Lambda^{-1}$ regularization | $\sigma$ | $10^{-6}$ | (2), all controllers |
| Kalman process noise (disturbance) | $Q_d$ | 10.0 | (3), (11) |
| Kalman observation noise (position) | — | $10^{-3}$ | Kalman update |
| Kalman observation noise (velocity) | — | $10^{-2}$ | Kalman update |
| Orientation stiffness | $K_\text{rot}$ | 20 Nm/rad | (14) |
| Orientation damping | $D_\text{rot}$ | 6 Nm·s/rad | (14), critically damped |
| Null-space centering stiffness | $k_\text{null}$ | 10 Nm/rad | (16) |
| Null-space damping | $d_\text{null}$ | 2 Nm·s/rad | (16) |
| Null-space barrier gain | $k_b$ | 50 Nm | (15) |
| Barrier activation threshold | $\eta$ | 0.10 (fraction of joint range) | (15) |
| Workspace-projection gain | $k_\text{ws}$ | $5\times10^{-4}$ m/(Nm/rad)$^\dagger$ | (17) |
| Workspace-projection cap | $p_\text{max}$ | 0.06 m | (17) |
| Workspace-projection regularization | $\epsilon_r$ | $10^{-8}$ | (17) |
| Backbone stiffness (C8 only) | $K_\text{bb}$ | 300 N/m | §S8 |
| Backbone damping ratio (C8 only) | $\zeta_\text{bb}$ | 1.0 (nominal unit-mass tuning) | §S8 |
| CBF safety margin | $\epsilon$ (margin) | 0.05 rad | Theorem 4, `cloud_verify` |
| CBF contraction rate | $\alpha$ | 0.5 | (19) |
| CBF slack penalty weight | $w_s$ | $10^{8}$ | §IV-B slack QP |
| Tank initial energy | $E_0$ | 20.0 J | (20)–(22) |
| Tank lower bound | $E_\text{min}$ | 0.05 J | (20)–(22) |
| Tank upper bound | $E_\text{max}$ | 50.0 J | (20)–(22) |
| Human-recharge fraction | $\gamma$ (tank) | 0.0 (conservative default) | (21) |
| OSQP tolerances | $\epsilon_\text{abs},\epsilon_\text{rel}$ | $10^{-6}$ | Layer 2 QP |
| OSQP iteration cap | — | 4000 | Layer 2 QP |
| Circular benchmark radius / period | $R$, $T$ | 0.12 m, 8 s | Benchmark I, §VI-B |
| Human step force | $F_h$ | 15 N, $t\in[3,6]$ s per cycle | Benchmark I/II |
| Reference ramp time | $T_\text{ramp}$ | 1.5 s | all trajectory tracking |
| FR3 neutral (null-space target) pose | $q_0$ | $[0,-0.785,0,-2.356,0,1.571,0.785]$ rad | (16) |

Two parameter sets are estimator/CBF-internal and are not separately swept in the main paper's ablations: the Kalman noise covariances and the CBF/tank constants (the latter are exercised in the `cloud_verify` hardware-path suite rather than the direct benchmark scripts used for the main tables). Reproducing any table from the released code uses these values unless the table's own caption states a swept parameter (e.g. $F_\text{max}$ in Table SVI, measurement noise in the main paper's Table III).

$^\dagger$Benchmark I and Benchmark II (Table SIII) are run with $k_\text{ws}=0$: the driver scripts for those two circular/waypoint benchmarks disable workspace projection so its effect does not confound the controller-to-controller comparison. The $5\times10^{-4}$ default above is what the boundary test (Table II) actually uses, which is where the mechanism is exercised and discussed.

---

## S4. Four-Plane Circle Tracking

C5 (MPC + Kalman, 100 Hz QP, 1 kHz inner) is evaluated on four 3-D circle planes, complementing the XZ-plane result of Benchmark I. Table SII reports RMSE after the 1.5 s ramp.

*Table SII — Free-Space Circle Tracking (R = 12 cm, T = 8 s)*

| Plane | IMP RMSE (mm) | MPC RMSE (mm) | Improvement |
|---|:---:|:---:|:---:|
| XZ sagittal   | 23.54 | 0.24 | **×97** |
| XY horizontal | 25.06 | 0.33 | **×77** |
| YZ frontal    | 16.25 | 0.20 | **×79** |
| XZ→XY tilted  | 25.07 | 0.31 | **×80** |

**Analysis.** The MPC maintains sub-0.35 mm RMSE across all planes, demonstrating that the $\Lambda(q)$-adaptive compliance in the QP (eq. 9 of the main paper) handles the off-diagonal operational-space inertia coupling without gain scheduling — the controller was tuned and validated on the XZ plane alone (Benchmark I), and the other three planes are a held-out generalization check, not a separately re-tuned result.

---

## S5. Benchmark II — Reach-and-Hold under Human Push

**Scenario.** The robot navigates a triangle of three waypoints (A, B, C) in one lap. At each waypoint, 0.8 s after arrival, a directionally varied 15 N push fires for 2 s. The robot must recover and dwell within 35 mm of the target for 1 s before advancing (3 push events, one per waypoint).

*Table SIII — Benchmark II: Waypoint-Hold Under Directionally Varied Push*

| Controller | Waypoints | RMS free (mm) | RMS contact (mm) | Peak defl. (mm) |
|---|:---:|:---:|:---:|:---:|
| G1 — Stiff Impedance | 3/3 | 63.6 | 41.4 | 47.1 |
| G2 — Pure Admittance | 3/3 | 72.8 | 190.2 | 226.6 |
| G3 — Variable Compliance | 3/3 | 69.3 | 133.5 | 170.1 |
| MPVIC — Var.-Imp. MPC (pred.) | 3/3 | 51.2 | 5.0 | 6.0 |
| G4 — MPC 100 Hz | 3/3 | 47.9 | 2.2 | 2.6 |
| G5 — MPC+Kalman 100 Hz | 3/3 | **47.9** | 0.6 | 2.4 |
| G6 — MPC 500 Hz | 3/3 | 51.9 | 0.8 | 1.0 |
| G7 — MPC+Kalman 500 Hz | 3/3 | 51.4 | **0.2** | **0.8** |

The corresponding paper plot shows D1/D2/D3/D7 plus the predictive variable-impedance (MPVIC) baseline; the table above retains the full G1–G7 ablation.

**Analysis.** The static-waypoint hold confirms the same separation seen in Benchmark I, and is included here for completeness rather than as an independent finding. All four MPC variants (G4–G7) reach every waypoint with order-of-magnitude lower contact-window deflection than the reactive baselines: against the stiff-impedance baseline (G1), the MPC cuts contact-window RMS from 41.4 mm to 2.2 mm (G4) and peak deflection from 47.1 mm to 2.6 mm. Adding the Kalman augmentation improves the contact and free-motion metrics — G5 lowers contact-window RMS from 2.2 mm to 0.6 mm and slightly reduces peak (2.6 → 2.4 mm) — so unlike a higher impedance gain it carries no peak penalty. Raising the QP rate to 500 Hz then sharpens the first-contact transient further, since the shorter 2 ms ZOH window lets $\hat{d}$ converge before significant error accumulates: G7 attains 0.2 mm contact-window RMS and a 0.8 mm peak. The pure-admittance and variable-compliance baselines (G2, G3) yield by design, producing the large 190–134 mm contact-window deflections expected of intentional compliance. The predictive variable-impedance baseline (MPVIC) behaves oppositely to the reactive variable-compliance G3: instead of softening, it stiffens predictively to reject the push, cutting contact-window RMS to 5.0 mm — an order of magnitude better than G1–G3 — yet it stays a further order of magnitude above the offset-free G5 (0.6 mm), the same non-offset-free residual seen in Benchmark I. Adapting the apparent stiffness, whether reactively (G3) or predictively (MPVIC), does not substitute for predictive disturbance cancellation. As in Benchmark I, MPVIC here is our own discrete-stiffness scheduler, not a reproduced published algorithm; see the main paper's Benchmark I discussion for the corresponding caveat.

---

## S6. Time-Varying Interaction and Horizon-Prediction Validity

The benchmarks in the main paper apply constant or step forces. Because the framework targets *interaction* dynamics broadly, we additionally exercise a *time-varying* human force and measure a proxy for the $N$-step disturbance-prediction RMS $\varepsilon_N$ of §III-D — the quantity that bounds how well the flat random-walk prediction holds over the horizon. To isolate the effect of the force alone from the configuration-driven terms of (7), the robot holds a *static* target while a sinusoidal push $F_z=-A\sin(2\pi f t)$ of fixed amplitude $A=12$ N and increasing frequency $f$ is applied, sweeping the disturbance rate $L_d=A\,2\pi f/\sqrt2$. The controller is the same DI-MPC + Kalman (C5, 100 Hz QP).

**What $\varepsilon_N$ actually measures here.** Because the true disturbance $d_k$ is not directly logged in this experiment, we use the filtered estimate $\hat d_{k\mid k}$ as its proxy and the flat prediction $\hat d_{k\mid k-N}=\hat d_{k-N\mid k-N}$, so $\varepsilon_N=\mathrm{RMS}_k\|\hat d_{k\mid k}-\hat d_{k-N\mid k-N}\|$ measures the $N$-step *self-consistency of the estimate*, not (13)'s prediction error against the true $d_k$ directly — the two agree only to the extent $\hat d$ has converged to $d$. This is visible in the $L_d=0$ (constant-force) row: the theoretical extrapolation term $L_dN\Delta t$ is exactly zero there, yet the measured $\varepsilon_N-\varepsilon_1=0.34$ N is not — that residual is the estimator's own process-noise jitter accumulated over $N$ steps, a real but different quantity from (13)'s extrapolation term. Table SIV should therefore be read as evidence *consistent with* the linear-in-rate scaling (13) predicts, not a direct validation against ground truth; a true ground-truth check — computing $d_{a,k}$ from the logged simulation dynamics ($\ddot e_k+\Lambda^{-1}F_{\text{mpc},k}$, both already available since the injected force is known) rather than the filtered estimate — is a well-defined follow-up we have not run.

*Table SIV — Time-Varying Force: Horizon Self-Consistency Error (Estimator Proxy) and Tracking*

| Disturbance rate $L_d$ (N/s) | $\varepsilon_1$ (N) | $\varepsilon_N$ (N) | $\varepsilon_N-\varepsilon_1$ | Bound $L_d N\Delta t$ (N) | RMS tracking (mm) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 0.0 (constant) | 0.04 | 0.38 | 0.34 | 0.00 | 0.08 |
| 5.3 | 0.05 | 0.52 | 0.46 | 0.53 | 0.12 |
| 10.7 | 0.12 | 1.20 | 1.08 | 1.07 | 0.27 |
| 21.3 | 0.25 | 2.47 | 2.22 | 2.13 | 0.54 |
| 42.7 | 0.49 | 4.87 | 4.38 | 4.27 | 1.03 |

**Analysis.** The measured horizon-extrapolation gap $\varepsilon_N-\varepsilon_1$ (self-consistency proxy) tracks the §III-D bound term $L_d N\Delta t$ to within $\approx10\%$ across a decade of disturbance rate (0.46 vs. 0.53; 1.08 vs. 1.07; 2.22 vs. 2.13; 4.38 vs. 4.27 N), consistent with the flat-prediction error being *linear in the force rate* as (13) predicts, with a constant-force floor $\approx0.38$ N (the $f=0$ row) set by the estimate's process-noise jitter accumulated over the $N$ steps — the $e_K$-analogue term of (13), not itself a prediction-error measurement (see above). Tracking degrades gracefully: the controller holds sub-millimeter error up to $L_d\approx21$ N/s and $\approx1$ mm at 43 N/s, recovering the offset-free property (Theorem 2) as $f\to0$. This is consistent with the disturbance-prediction analysis of §III-D and the use of the flat random-walk model inside the receding-horizon loop, though a ground-truth (not self-consistency) measurement would be needed to validate it directly; for a *structured* high-rate force (e.g. physiological tremor) the harmonic-internal-model extension of §III-D would predict that component forward exactly rather than extrapolate it flat. The experiment is reproduced by `time_varying_experiment.py`.

---

## S7. Human-Force Magnitude and Shape Sweep

Benchmark I and §S6 fix the force magnitude at 12–15 N. Because the offset-free property (Theorem 2) is independent of the force amplitude, we sweep a sustained (step) push $F_z\in\{5,10,15,20,25\}$ N on the circular tracking task and report the steady-state and peak tip deflection for classical impedance (D1) and the proposed offset-free controller (D7, DI-MPC + Kalman, 500 Hz); a shape sweep at 15 N (step / linear ramp / 1 Hz sinusoid) complements the time-varying study of §S6.

*Table SV — Human-Force Magnitude Sweep (steady-state / peak tip deflection, mm)*

| Force $F_z$ (N) | 5 | 10 | 15 | 20 | 25 |
|---|:---:|:---:|:---:|:---:|:---:|
| D1 Impedance — SS | 26.9 | 33.7 | 45.0 | 58.5 | 73.1 |
| D1 Impedance — peak | 28.4 | 37.0 | 51.2 | 70.1 | 88.9 |
| **D7 proposed — SS** | **0.021** | **0.021** | **0.021** | **0.021** | **0.021** |
| **D7 proposed — peak** | **0.29** | **0.50** | **0.76** | **1.02** | **1.27** |

**Analysis.** Classical impedance deflects in proportion to the force — the $e_\infty=K_d^{-1}F_h$ bias combined with the dynamic tracking error grows from 27 mm at 5 N to 73 mm at 25 N — whereas the proposed controller holds a **0.021 mm steady-state deflection independent of magnitude**: the integrating disturbance state cancels the force whatever its size, so accuracy no longer trades against the human's effort. The first-contact peak grows only mildly (0.29 → 1.27 mm across 5–25 N), scaling with the force step before $\hat d$ converges. Under the three 15 N shapes the proposed controller keeps contact-window RMS at 0.15 mm (step), 0.10 mm (ramp), and 0.40 mm (1 Hz sinusoid) versus 40.9 / 26.0 / 33.0 mm for classical impedance; the sinusoid is the worst case, consistent with §S6 — the flat $\dot d=0$ model lags a fast-varying force, which the harmonic internal model of §III-D would recover. Reproduced by `force_sweep.py`.

![Human-force magnitude sweep: steady-state (a) and peak (b) tip deflection versus a sustained push, for classical impedance (D1, red) and the proposed offset-free controller (D7, green). The proposed steady-state deflection is invariant to force magnitude (0.021 mm), while classical impedance grows linearly.](simulation/force_sweep.png)

---

## S8. Correction-Authority Robustness (Impedance-Backbone Ablation)

Every controller in the main paper places the entire task-space correction inside one box-constrained decision variable, $F_\text{mpc}$: if it is driven toward zero — a tight $F_\text{max}$ bound, or a horizon step whose predicted torque is unrealizable — the commanded torque degrades to the feedforward term $\tau_\text{ff}$ alone, i.e. zero corrective stiffness. We evaluate an architectural ablation, **C8**, that decouples baseline corrective stiffness from the optimizer: a fixed, positively damped impedance law
$$F_\text{bb}=K_\text{bb}e+D_\text{bb}\dot e,\qquad K_\text{bb}=300\text{ N/m},$$
is commanded on every MPC update independent of the QP output ($D_\text{bb}=2\zeta_\text{bb}\sqrt{K_\text{bb}}$, a nominal unit-effective-mass tuning — $\Lambda^{-1}(q)$ is generally anisotropic, so this is not exact modal-critical damping), and the QP shapes only a bounded additional correction $F_\text{mpc}$ ($\lVert F_\text{mpc}\rVert_\infty\le F_\text{max}$) predicted through the backbone dynamics linearized at the current configuration, $A_\text{cl}=A_d+B_d(\rho_k)G_\text{bb}$. If the QP's output is driven to exactly zero for any reason, the commanded controller retains this non-zero corrective stiffness instead of reverting to bare feedforward. This is QP-independence, not actuator-limit independence: the **total commanded torque is $\tau=\tau_\text{base}+J_v^\top(F_\text{bb}+F_\text{mpc})$** — $F_\text{bb}$ is not optional and must be included in any torque-feasibility check — and C8 additionally applies (9b)'s row at *every* horizon step rather than only the first, frozen at the current configuration:
$$-\tau_\text{max}\le \tau_\text{base}(0)+J_v^\top(q_0)\big(F_\text{bb}+F_{\text{mpc},i}\big)\le\tau_\text{max}, \quad i=0,\ldots,N-1,$$
the frozen-Jacobian horizon-wide extension of (9b) that §III-B notes "can be added" but does not itself write out — it is not implied by (9b)/(9c) alone, and is specific to C8, not part of the default (C1–C7) formulation. Freezing $J_v(q_0)$, $\tau_\text{base}(0)$ across the horizon is a local approximation, not an exact prediction of future torque feasibility (§III-B's Constraint interpretation applies here too). With this in place, the evidence below supports robustness to loss of **additive QP correction authority** specifically, not a blanket guarantee under arbitrary actuator saturation or solver failure.

**Scenario and protocol.** Identical to Benchmark I (§VI-B): 12 cm circular reference, 15 N step push, 3 cycles / 24 s, contact-window metrics averaged over all three push events. We sweep the corrective-force bound $F_\text{max}\in\{150,20,5,1,0\}$ N, holding every other parameter fixed, emulating progressively severe loss of additive correction authority; $F_\text{max}=0$ N is the limiting case in which the QP contributes nothing.

**Results.** Under the normal protocol ($F_\text{max}=150$ N), C8 matches C7 to measurement precision — RMS/contact/peak/SS of 12.66/0.15/0.76/0.022 mm vs. C7's 12.61/0.15/0.77/0.022 mm (both reproduced under the present codebase for a self-consistent comparison; both agree with the main paper's Table I C7 row to within the small residual difference from an exact zero-order-hold correction applied after that table was generated). The stress sweep is the key comparison:

*Table SVI — Correction-Authority Robustness Ablation: $F_\text{max}$ Sweep, Contact-Window RMS / Peak Deflection (mm)*

| $F_\text{max}$ (N) | 150 | 20 | 5 | 1 | 0 |
|---|:---:|:---:|:---:|:---:|:---:|
| C7 (no backbone) — RMS contact | 0.15 | 0.15 | 317.6 | 414.0 | 424.4 |
| C7 (no backbone) — peak | 0.77 | 0.76 | 412.3 | 552.5 | 560.8 |
| **C8 (+ backbone) — RMS contact** | **0.15** | **0.15** | **22.5** | **37.7** | **41.3** |
| **C8 (+ backbone) — peak** | **0.76** | **0.76** | **29.8** | **47.9** | **51.6** |

**Analysis.** At $F_\text{max}\ge20$ N there is enough headroom that both controllers are unaffected and C8 costs nothing relative to C7 — the backbone's restoring term is exactly what the unconstrained QP would already have produced. Below that, C7's contact-window RMS jumps by roughly three orders of magnitude (0.15 → 424 mm) the moment the corrective force can no longer supply the needed authority: with $F_\text{mpc}$ forced toward zero, the commanded torque approaches bare feedforward against the sustained push. **C8 degrades gracefully instead**, settling at 22–41 mm even in the $F_\text{max}=0$ limit — a 10–14$\times$ smaller error than C7 across the curtailed range, and bounded rather than diverging. The offset-free/stability statements of Theorems 1–3 are proved for the standard (non-backbone) realization; C8's additive term retains the same offset-free mechanism (the $-\hat d$ centering trick is applied to $F_\text{mpc}$, not the backbone), but a formal re-derivation of the theorems for the backbone-augmented closed loop is left to future work. Reproduced by `stable_backbone_comparison.py --n-cycles 3`.

![Correction-authority robustness ablation: contact-window RMS error (a) and peak deflection (b) versus the QP's corrective-force bound $F_\text{max}$, for the proposed controller with (C8, green) and without (C7, red) the impedance backbone. C8 degrades gracefully as $F_\text{max}\to0$ while C7 collapses toward bare feedforward.](simulation/stable_backbone_comparison_3cycle.png)
