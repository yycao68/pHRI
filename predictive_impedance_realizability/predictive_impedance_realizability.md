# Is the Passive-Nominal Split Feasible?

### A design gate for Predictive Interaction Realizability on the FR3

*Working draft. Task 1 (the design gate), Task 2 (the merged controller) and
Task 3 (the authorization-vs-tracking sweep). Simulation only.*

---

## 1. Decision summary

Merging `phri2`'s behaviour–realization separation with `impedance_residual`'s
energy-authorized residual port requires `phri2` to acquire something it does
not have: a **nominal**. The proposed route is the *passive-nominal split*,

$$F_{\mathrm{cmd}} = F_{\mathrm{nom}} + F_r,\qquad F_{\mathrm{nom}} = -K_0 e - D_0 v,$$

in which $F_{\mathrm{nom}}$ is a small behaviour-agnostic spring–damper that
supplies a passivity floor and a harvestable dissipation term $v^\top D_0 v$,
and the realization QP optimizes only the residual $F_r$ that the tank then
authorizes. The merged Lemma 1 that this route buys is worthless unless its
first precondition is satisfiable: the **anchor**

$$a_\ell = \tau_{\mathrm{base},\ell} + J_{v,\ell}^\top F_{\mathrm{nom},\ell},
\qquad |a_\ell| \le \bar\tau \quad\text{at every tick},$$

must fit inside the derated torque envelope — *and* the $\alpha\to 0$ fallback
it degenerates to must still hold a sustained 20 N push inside the 0.06 m
workspace bound. Those two requirements pull in opposite directions. This
document reports whether they have a common solution.

**They do, but only under one of the two envelopes the source papers use, and
only in a sliver.**

| Torque envelope | Source | Feasible region | Verdict |
|---|---|---|---|
| $\rho\,\tau_{\max}$, $\rho = 0.28$ | `impedance_residual` | **0 / 900 cells** | **NO-GO, structural** |
| $87/87/87/\mathbf{31.5}/12/12/12$ N·m | `phri2` | **33 / 900 cells**, $k \in [360, 400]$ N/m, $d \in [21.3, 48.4]$ N·s/m | **GO, thin** |

Under the $\rho = 0.28$ envelope no $(K_0, D_0)$ works — not a tuning failure,
a structural one (§5.1). Under `phri2`'s own envelope the split is viable at
$K_0 = 380\,\mathrm{N/m}$, $D_0 = 29.1\,\mathrm{N \cdot s/m}$, with every hard
diagnostic sitting at 93–98 % of its limit (§5.2).

Three findings change what the merge can claim, and all three are load-bearing:

1. **The two papers' "derated budget" stress cases are not interchangeable.**
   The synthesis note treats both as $\rho = 0.28$. They are not:
   `phri2`'s own realized torque peaks at **1.30×** the $\rho = 0.28$ cap, so
   `phri2`'s published FR3 benchmark does not even *run* inside
   `impedance_residual`'s envelope. The merge inherits whichever envelope it
   is written against, and the answer flips between them.
2. **The note's four-diagnostic table admits false green cells.** Its row (1)
   is evaluated on a trajectory where $F_{\mathrm{nom}}$ is small, while its
   row (3) defines a displaced operating point the robot must actually hold.
   Checking each in its own place, 70 cells pass; requiring the precondition
   to hold on the *fallback itself* (row 1b, §4.3) leaves 33. Half the
   apparent region is an artifact of never asking one trajectory to satisfy
   both constraints.
3. **The $\alpha\to 0$ fallback transiently leaves the workspace box.** At the
   recommended cell the settled displacement is 55.8 mm against the 60 mm
   bound, but the transient peaks at **62.2 mm**. Gate on peak instead of
   settled displacement and the region drops from 33 cells to 6 — and the
   recommended cell is not among them.

The merged controller has since been implemented and run in closed loop (§7).
That settles the gate's main caveat and adds a fourth finding that outranks the
other three:

4. **The split converts a hard actuator guarantee into a conditional one.**
   In `phri2` the QP's torque constraint covers the *whole* command. Under the
   split the nominal sits outside the authorization loop — the 1 kHz servo can
   scale $F_r$ but never $F_{\mathrm{nom}}$ — so once
   $|\tau_{\mathrm{base}} + J_v^\top F_{\mathrm{nom}}| > \bar\tau$ nothing
   in the fast loop can prevent the overrun. With only 2.4 % of joint 4's cap
   left as headroom, that happens at disturbance amplitudes well inside what
   `impedance_residual`'s own benchmark uses.

§9 states the decision this hands to a human.

---

## 2. Why the split is needed at all

The two unsubmitted drafts are complementary halves of "predictive impedance
under saturation":

- **`impedance_residual`** holds a fixed physical impedance as nominal, lets a
  100 Hz MPC propose an additive residual wrench, and authorizes that residual
  at 1 kHz against both a joint-torque envelope and an energy tank. Its
  central result is that the residual port must be re-authorized *between*
  manager updates.
- **`phri2`** emits a desired contact-port acceleration
  $a_{id} = C_\theta x + G_\theta F_h$ from a behaviour layer, realizes it in a
  receding-horizon QP under a horizon-wide torque constraint, and reports the
  deviation as $r = r_{\mathrm{reg}} + r_{\mathrm{con}} + r_{\mathrm{mod}}$
  rather than hiding it in clipping.

`phri2` §VIII names its own gap in as many words: *"the realization cost does
not itself imply passivity … A dissipativity constraint on the realized port
variables, paired with an energy tank or passivity observer, is a natural
extension."* That extension is `impedance_residual`'s tank.

The graft does not work naively, for a structural reason. `impedance_residual`'s
tank sits on a nominal $\tau_I$ that is always-feasible and natively passive,
and it authorizes only the additive residual. **`phri2` has no nominal.** Its
$\tau_{\mathrm{base}}$ is feedforward plus orientation hold plus null-space
centering, with no impedance term at all; the entire interaction behaviour
lives inside $F_{\mathrm{cmd}}$. Two consequences follow:

1. Scaling $F_{\mathrm{cmd}}$ down at a fast tick degenerates not to a passive
   spring–damper but to **zero interaction behaviour** — gravity compensation
   and posture only, a free-floating arm.
2. The tank's measured advantage comes from harvesting the nominal damping
   $v^\top D_I v$. `phri2` has no harvestable damping term.

The passive-nominal split exists to fix both at once, at the cost of
introducing $(K_0, D_0)$ — and with them the question this document answers.

---

## 3. The merged fast re-authorization rule

For context, the rule the split is meant to enable. The QP supplies
$F_{r,k}$ every 20 ms; the following runs every 1 kHz tick $\ell$ ($h = 1$ ms):

1. Recompute from the current $(q,\dot q)$: $\tau_{\mathrm{base},\ell}$,
   $J_{v,\ell}$, $\Lambda_\ell$, and $F_{\mathrm{nom},\ell} = -K_0 e_\ell - D_0 v_\ell$.
2. **Anchor** $a_\ell = \tau_{\mathrm{base},\ell} + J_{v,\ell}^\top F_{\mathrm{nom},\ell}$,
   with the precondition $|a_\ell| \le \bar\tau$.
3. **Torque scale** $\alpha_{\tau,\ell}$ = largest $\alpha \in [0,1]$ with
   $|a_\ell + \alpha J_{v,\ell}^\top F_{r,k}| \le \bar\tau$ per joint.
4. **Energy scale** $\alpha_{E,\ell} = 1$ if $\bar F_{r,\ell}^\top v_\ell \le 0$,
   else $\min\!\big(1, (E_\ell - E_{\min} + h\,v_\ell^\top D_0 v_\ell)/(h\,\bar F_{r,\ell}^\top v_\ell)\big)$.
5. **Apply** and **ledger** as in `impedance_residual`.

Steps 1–2 are what §4–§6 gate. Steps 3–6 are implemented in
`pir_controller.py` and measured in §7.

---

## 4. Method

### 4.1 One FR3 model, imported not rewritten

Every mass matrix, Jacobian, task-space inertia, bias torque, null-space
projector, torque limit and scenario constant is imported from the existing
studies (`simulation/fr3_mujoco.py`, `simulation/fr3_impedance.py`,
`imp_reference/simulation/fr3_interaction_dynamics_mpc.py`,
`imp_reference/simulation/run_fr3_experiments.py`). Nothing is re-derived: a
second FR3 model would make this gate inconsistent with the papers it gates.
The regression suite asserts the 0.06 m bound *is*
`FR3MPCConfig.position_limit` rather than merely equalling it.

### 4.2 Scenario

`phri2`'s FR3 benchmark: the EE holds a fixed nominal pose while a smooth
sustained 20 N push along $-z$ (raised-cosine ramp, `human_force_at`) would,
under naive realization, drive it through the 0.06 m workspace bound. 6 s,
1 kHz servo, 50 Hz manager. Task-space inertia at the nominal pose is
$\Lambda_{zz} = 4.56$ kg; the bias torque on joint 4 is 18.96 N·m before any
control is applied at all.

Two torque envelopes are carried, because the two papers use different ones:

- **`rho_0.28`** — $0.28\,\tau_{\max}$ on every joint (24.36 N·m on joints 1–4,
  3.36 N·m on joints 5–7), `impedance_residual`'s continuous-envelope stress case.
- **`derated_joint4`** — stock FR3 limits with joint 4 derated 87 → 31.5 N·m,
  `phri2`'s own torque-activation stress case.

### 4.3 Diagnostics

Rows 1–4 follow the synthesis note's §13.2 table. Row 1b is added.

| # | Quantity | Checks | Pass |
|---|---|---|---|
| 1 | $\max_t \|\tau_{\mathrm{base}} + J_v^\top F_{\mathrm{nom}}\|_\infty / \bar\tau$ on the `phri2` replay | anchor feasible | $< 1$ |
| 1b | the same ratio, peaked over the closed-loop $\alpha\to0$ fallback run | anchor feasible *there too* | $< 1$ |
| 2 | $\lambda_{\min}(K_0),\ \lambda_{\min}(D_0)$ | passive | $> 0$ |
| 3 | settled $\|e_z\|$ at $\alpha\to0$ under the 20 N push | fallback holds | $\le 0.06$ m |
| 4 | $\max_t \|J_v^\top F_{\mathrm{nom}}\|_\infty / \bar\tau$ | not budget-dominating | $< \beta = 0.5$ |

**Why row 1b exists.** Rows 1 and 3 are measured at two different operating
points. Row 1 replays the trajectory `phri2`'s MPC actually produced, on which
the EE stays near the nominal pose, so $F_{\mathrm{nom}} = -K_0 e$ is small
whatever $K_0$ is. Row 3 describes the *displaced* equilibrium the arm settles
into once the residual is de-authorized, where by construction $K_0 e = F_h$.
Merged Lemma 1 requires $|a_\ell| \le \bar\tau$ at **every** tick, and the
fallback's ticks are ticks. Row 1b closes that hole by peaking the same ratio
over the fallback run itself — transient included, since Lemma 1's
precondition has no exemption for overshoot. Omitting it more than doubles
the apparent feasible region (§5.2).

Rows 1 and 4 are replay diagnostics: cheap, and exact *given* that the merged
controller's total command tracks `phri2`'s. That assumption is the main
caveat on this result and is stated as such in §7.

Row 3 is measured on a closed-loop MuJoCo run of the actual $\alpha\to 0$ law,
$\tau = \tau_{\mathrm{base}} + J_v^\top F_{\mathrm{nom}}$, not from static
algebra. The push hold is extended to 6 s and the displacement averaged over
the final 1.5 s: at $k = 380$ N/m against $\Lambda_{zz} = 4.56$ kg the fallback
spring has a 0.69 s natural period, so `phri2`'s own 2 s hold and a
sub-period averaging window read an oscillation rather than an equilibrium —
and biased the estimate in the flattering direction, making low-damping cells
look *better* than they are.

### 4.4 Grid

$k \in [10, 1200]$ N/m log-spaced, $d \in [2, 60]$ N·s/m, densified through
$k \in [300, 520]$ N/m and $d \in [8, 40]$ N·s/m: 36 × 25 = 900 cells. The
note proposes $k \le 400$ N/m and says to widen if the region sits at an edge.
It does — row 3 alone forces $k \gtrsim 350$ N/m — and the densification is
not cosmetic: on a 24 × 16 grid the region is ~40 N/m tall and the scan
reports it **empty** (§6.3).

---

## 5. Results

### 5.1 `rho_0.28`: structurally infeasible

| | value |
|---|---|
| $\tau_{\mathrm{base}}$ alone, as a fraction of joint 4's cap | **0.822** |
| `phri2`'s own realized torque, as a fraction of this cap | **1.300** |
| cells passing row 1 | 197 / 900 |
| cells passing row 1b | **0 / 900** |
| cells passing row 3 | 385 / 900 |
| **common region** | **0 cells** |
| best worst-normalized use over the whole grid | 1.238 |

Row 1b fails in **every cell**, with a grid-wide minimum of 1.144. This is not
a tuning result. Holding a 20 N push at the fallback equilibrium requires
$K_0 e = F_h$, so $J_v^\top F_{\mathrm{nom}} = -J_v^\top F_h$ *regardless of
$K_0$* — the gains set where the arm sits, not how hard it must push back.
On this pose that costs ~28.4 N·m on joint 4 asymptotically (§6.2), against a
24.36 N·m cap. No $(K_0, D_0)$ can change that.

The second row of the table is the more consequential one. `phri2`'s published
FR3 trajectory peaks at 1.30× this envelope, so the envelope is not one the
`phri2` benchmark ever operated in. Carrying $\rho = 0.28$ into the merge, as
the synthesis note assumes, does not merely tighten the design problem — it
changes the scenario to one in which the source result does not exist.

### 5.2 `derated_joint4`: feasible, thin

| | value |
|---|---|
| $\tau_{\mathrm{base}}$ alone, as a fraction of joint 4's cap | 0.636 |
| `phri2`'s own realized torque, as a fraction of this cap | 1.005 |
| cells passing row 1 / 1b / 3 | 577 / 702 / 385 |
| **common region** | **33 / 900 cells** |
| extent | $k \in [360, 400]$ N/m, $d \in [21.3, 48.4]$ N·s/m |
| region if row 1b omitted | **70 cells** |
| region if row 3 gated on peak rather than settled | **6 cells** |
| region after $\beta = 0.5$ | 33 cells (row 4 never binds) |

**Recommended operating point.** The note's literal criterion — minimize row 4
subject to rows 1–3 — selects $k = 360$ N/m, $d = 21.3$ N·s/m, which sits
0.4 % from violating row 1b. That is the wrong objective when the region is a
sliver: row 4 is the only diagnostic the note does not hard-gate. The cell
furthest from *every* hard constraint is the one to take:

$$\boxed{K_0 = 380\ \mathrm{N/m}, \qquad D_0 = 29.1\ \mathrm{N \cdot s/m}}$$

| diagnostic | value | limit | use |
|---|---|---|---|
| (1) replay anchor | 0.977 | 1 | 98 % |
| (1b) fallback anchor | 0.976 | 1 | 98 % |
| (3) settled $\|e_z\|$ | 55.8 mm | 60 mm | 93 % |
| (3′) peak $\|e_z\|$ | **62.2 mm** | 60 mm | **104 %** |
| (4) budget fraction | 0.348 | $\beta = 0.5$ | 70 % |

This is a damping ratio of $\zeta = 0.35$ at 1.45 Hz — a floor that is
genuinely a floor, not a second behaviour layer.

**The three-way squeeze.** The overlay figure shows the region bounded below
by row 3 (too soft and the fallback leaves the box), above by row 1 (too stiff
and the anchor overruns joint 4 on the operating trajectory), and on the left
by row 1b (too little damping and the fallback's overshoot overruns joint 4).
The note anticipated tensions (a)↔(c) and (c)↔(d). (d) never binds: row 4
ranges over 0.326–0.373 across the whole region, so $\beta$ could be set
anywhere above ~0.38 without changing a single cell. The binding third
constraint is the one the note did not list — damping against the fallback
transient.

### 5.3 Figures

| file | content |
|---|---|
| `pir_knot_scan_diag{1,1b,3,4}_{envelope}.png` | the four diagnostic maps, per envelope |
| `pir_knot_scan_overlay.png` | composite, full range and zoomed, with the recommended cell marked |
| `pir_knot_scan.json` | grid, all diagnostic arrays, `common_region_nonempty`, both recommendations |
| `pir_closed_loop_{scenario}_{envelope}.png/json` | §7: the merged controller in closed loop |
| `pir_e0_sweep.png/json` | §7.4–7.5: authorization vs tracking, both tightenings active |

---

## 6. Verification

A NO-GO on one envelope and a 33-cell GO on the other are both strong enough
claims to be attacked separately (`pir_verify.py`, `results/pir_verify.json`).

### 6.1 Check A — equilibrium without integration

Rows 1b and 3 are read off a settled simulation, and "settled" is a judgement
call for a lightly damped spring. Check A instead solves $\ddot q(q) = 0$
directly on MuJoCo's own forward dynamics — no time integration at all — and
compares. Across four cells the two methods agree to within **6.9 %** worst
case (1.2 % at the well-damped end), all root-finds converged, all simulations
flagged settled. The residual gap is the null-space posture spring, which
makes the settled configuration mildly path-dependent; it is smaller than the
7 % margin the recommended cell carries on row 3, but not by much, which is
part of why §7 calls that margin thin rather than adequate.

### 6.2 Check B — the fallback anchor does not depend on the gains

Sweeping $k$ over $[100, 1200]$ N/m at the root-found equilibrium:

| $k$ [N/m] | $\|e_z\|$ [m] | $k\,\|e_z\|$ [N] | joint-4 anchor [N·m] |
|---|---|---|---|
| 100 | 0.198 | 19.83 | 25.29 |
| 200 | 0.101 | 20.12 | 27.87 |
| 400 | 0.050 | 20.06 | 28.34 |
| 800 | 0.025 | 19.98 | 28.40 |
| 1200 | 0.017 | 19.95 | 28.39 |

$k\,|e_z|$ recovers the 20 N push to within 1 % across a 12× range of $k$,
confirming $K_0 e = F_h$ and therefore that the equilibrium anchor is a
property of the push and the pose, not of the gains. It converges to
~28.4 N·m — comfortably above the 24.36 N·m $\rho = 0.28$ cap and comfortably
below the 31.5 N·m derated-joint-4 cap. That single number is why the two
envelopes give opposite answers.

As a side result, the $k = 200$ row independently reproduces `phri2`'s own
reported figure that a pure $K_d = 200$ impedance gives 0.10 m of static
displacement against the 0.06 m bound.

### 6.3 Check C — grid refinement

Re-scanning $k \in [300, 500]$ × $d \in [8, 40]$ on a *linear* grid whose
points deliberately do not coincide with the main scan's:

| envelope | region | best worst-normalized use |
|---|---|---|
| `rho_0.28` | empty (0 / 99) | 1.244 |
| `derated_joint4` | **14 / 99** | 0.977 |

Two things follow. The `derated_joint4` verdict and its best cell are
reproduced independently (0.977 against the main scan's 0.977). And the
refinement is *necessary*: the original 24 × 16 grid reported
`derated_joint4` empty. The region is ~40 N/m tall inside a sweep spanning
1190 N/m, and a grid that samples it at two points misses it. Any re-run of
this gate at a different pose, push direction or envelope must re-densify
before trusting an empty answer.

### 6.4 Regression suite

`test_pir_knot_scan.py`, 12 tests, all passing. They pin the constants to
their sources, check the anchor arithmetic against the per-tick matrix
product, assert the zero-gain limit reduces the anchor to
$\tau_{\mathrm{base}}$, verify the $K_0 e = F_h$ relation on the plant, check
the averaging window exceeds the fallback spring's natural period, and assert
the published grid stays dense enough through the feasible band.

---

## 7. The merged controller, implemented

Task 1's gate was a *feasibility* argument; §7 of its first draft owed a
closed-loop measurement. `pir_controller.py` implements the merged rule of §3
and `run_pir_closed_loop.py` runs it on the FR3.

### 7.1 What the merge actually required

Three things changed relative to grafting the tank onto `phri2`, and all three
live in the QP:

1. **The nominal is closed-loop inside the horizon.** $F_{\mathrm{nom}}$ is
   state feedback, so the prediction runs on $A - BG_0$ with
   $G_0 = [K_0\ D_0]$. Freezing $F_{\mathrm{nom}}$ across the horizon would let
   the QP plan against a nominal its own plan invalidates.
2. **The torque rows became state-dependent.** In `phri2`,
   $\tau_k = \tau_{\mathrm{base}} + J_v^\top F_{\mathrm{cmd},k}$ is affine in
   the decision variable alone. Here
   $\tau_k = \tau_{\mathrm{base}} + J_v^\top(-G_0 x_k + F_{r,k})$, and $x_k$ is
   itself affine in every earlier residual, so each torque row carries the
   accumulated nominal reaction.
3. **The behaviour input and the disturbance model had to be split.** `phri2`
   has one force signal driving both $a_{id} = Cx + GF_h$ and the plant
   prediction, which is sound only when every external force is interaction
   force. Running under a disturbance, they are different signals. `control()`
   now takes `force_forecast` and `behaviour_forecast` separately, defaulting
   to equal — which recovers `phri2` exactly, and is asserted to. **This does
   not solve force misclassification**; deciding which measured force goes in
   which channel *is* that unsolved problem. It moves the assumption into a
   signature instead of hiding it in an equality.

One incidental gain: `phri2`'s QP has no principled infeasibility fallback and
drops to its clipped reactive law. Here $F_r = 0$ is exactly the $\alpha\to 0$
passive nominal the Task 1 gate certified, so an infeasible solve degrades to a
*certified* behaviour rather than an improvised one. (No solve was infeasible
in any run reported here: 0 / 300 per run.)

### 7.2 Rows (1) and (4), re-measured in closed loop

This is the debt the first draft recorded. At $K_0 = 380$ N/m,
$D_0 = 29.1$ N·s/m on `phri2`'s 20 N push:

| | replay estimate (Task 1) | merged closed loop | gap |
|---|---|---|---|
| row (1) anchor ratio | 0.9771 | **0.9762** | 0.09 % |
| row (4) budget ratio | 0.3478 | **0.3474** | 0.11 % |

The replay approximation holds. The gate's 2 % margin on row (1) survives, and
the GO verdict does not need revisiting on this scenario. A regression test
pins the two together at 1 % so the gate cannot silently drift from the
controller it gates.

### 7.3 Merged Lemma 1 and the four-term residual

Across every run in §7.4–7.5 (36 sweep points plus 8 closed-loop runs), with
the precondition holding, the conclusion held: $|\tau_\ell| \le \bar\tau$ at
every one of 6000 ticks, with $\alpha_\tau$ saturating the envelope exactly
($\max |\tau|/\bar\tau = 1.0000$) rather than overshooting it.

The four-term residual
$r = r_{\mathrm{reg}} + r_{\mathrm{con}} + r_{\mathrm{mod}} + r_{\mathrm{auth}}$
closes **to machine precision** ($4.4\times10^{-16}$) on every QP tick, with
$r_{\mathrm{con}}$ split into its plan-level and execution-level halves:

$$r_{\mathrm{con}} = \underbrace{a_{\mathrm{con}} - a_{\mathrm{unc}}}_{\text{QP, 50 Hz}} + \underbrace{\Lambda^{-1}(\alpha_\tau - 1)F_{r,k}}_{\text{servo, 1 kHz}}, \qquad r_{\mathrm{auth}} = \Lambda^{-1}(\alpha_E - 1)\,\bar F_{r,\ell}.$$

The closure is an identity, not a fit: the two servo terms telescope to
$\Lambda^{-1}(F_{r,\ell} - F_{r,k})$, which is exactly the gap between what the
QP proposed and what was applied. $r_{\mathrm{auth}}$ is identically zero
whenever authorization never fires — it measures intervention, it is not an
always-on correction.

### 7.4 The passivity axis: `impedance_residual`'s result transfers

Neither source benchmark exercises both axes. `phri2`'s push is monotone and
largely dissipative: the nominal harvests $v^\top D_0 v$ faster than the
residual drains the tank, and $\alpha_E \equiv 1$ throughout. So the sweep runs
on a **merged scenario** — that push plus `impedance_residual`'s own rejectable
disturbance (0.9/1.4/1.9 Hz sinusoids and a 12 N pulse placed deliberately
between two 50 Hz manager ticks) — and sweeps $E_0$ toward its floor and the
disturbance amplitude upward. 36 points, three variants:

| variant | tank floor held | breach points | breached ticks | worst tank |
|---|---|---|---|---|
| `pir` (1 kHz re-authorization) | **12 / 12** | 0 | 0 | 0.0200 J = $E_{\min}$ |
| `pir_manager_guard` (20 ms, held) | 4 / 12 | 8 | 2105 | −0.188 J |
| `pir_no_tank` | 4 / 12 | 8 | 5537 | −1.473 J |

**`impedance_residual`'s central result transfers to the merged port.** Fast
re-authorization holds the floor in every stressed configuration — and holds it
*with equality*, $\min E = E_{\min}$ exactly, which is what the $\alpha_E$
construction guarantees. The manager-rate guard tracks it closely and breaches
anyway, in 8 of the 12 configurations where the tank is loaded at all. The
20 ms staleness is the whole difference.

Authorization goes from silent to active as either axis is loaded: 0 % → 1.9 %
of ticks as $E_0 \to E_{\min}$, and 0 % → 18.1 % of ticks as the disturbance
scales 1 → 12.

### 7.5 The authorization-vs-tracking trade is *flatter*, not steeper

The synthesis note predicts the $E_0$ curve gets steeper after the merge,
because the two tightenings compound in series. It gets flatter. Over the whole
$E_0$ sweep, RMS realization residual varies by **0.03 %** (2.5095 →
2.5103 m/s²) — against `impedance_residual`'s own 21.49 → 15.98 mm swing on its
unmerged nominal.

The reason is the §7.6 boundary, and it is not good news: the passive nominal
has already absorbed the authority the tank would otherwise have taken away.
De-authorizing a residual that is 16 % of the command costs almost nothing,
because the other 84 % is a PD law the tank has no say over. A flat
authorization-vs-tracking curve here means the passivity axis is *cheap*
precisely because it is *weak*.

On the disturbance axis, where the residual does matter, the trade reappears:
RMS spans 2.510 → 4.026 m/s² (60 %). Notably `pir` tracks *better* than
`pir_no_tank` at disturbance scale 4 (2.624 vs 3.233 m/s²) — an unauthorized,
energy-injecting residual makes realization worse, not better. That is one
scenario at one seed and should not be leaned on.

### 7.6 The nominal dominates the command, and eats the headroom

At the only $(K_0, D_0)$ the gate certifies:

| | `pir` | `zero_nominal` ($K_0 = D_0 = 0$) |
|---|---|---|
| $\|F_{\mathrm{nom}}\|$ RMS | 14.05 N | 0 N |
| $\|F_r\|$ RMS | 2.11 N | 13.48 N |
| residual's share of the command | **15.6 %** | 100 % |
| anchor headroom left for $F_r$ | **2.4 %** of the cap | 36.4 % |

The note's §7 predicted this qualitatively — "$K_0/D_0$ large ⇒ floor eats
budget, a *new* double-tightening." The measurement is worse than the phrasing
suggests. $K_0 = 380$ N/m is not a small passivity floor sitting under the
behaviour: it is **stiffer than the desired impedance** ($K_d = 200$ N/m), so
the residual's job is partly to make the arm *softer* than the floor. The
merged controller at its certified operating point is a stiff PD with a 16 %
predictive correction, and the residual's torque headroom has collapsed
**15-fold**, from 36.4 % of joint 4's cap to 2.4 %.

### 7.7 Where the guarantee breaks

That 2.4 % headroom is what §1's fourth finding cashes out. Sweeping the
disturbance amplitude:

| disturbance scale | max anchor ratio | ticks with infeasible anchor | max $\|\tau\|/\bar\tau$ |
|---|---|---|---|
| 4 | 0.9888 | 0 / 6000 | 1.0000 |
| 8 | **1.0074** | 39 / 6000 | **1.0074** |
| 12 | **1.0629** | 558 / 6000 | **1.0629** |

Two things to read here. First, the applied-torque overrun **equals the anchor
overrun exactly** at both failing scales. That is not a coincidence: when the
precondition fails, $\alpha_\tau$ has already gone to zero and the fast layer
has *no remaining authority* — the overrun is entirely the nominal's, and
scaling the residual cannot touch it. Second, `pir_no_tank` fails identically
(1.0074 at scale 8), confirming this is a property of the nominal and has
nothing to do with the tank.

So Merged Lemma 1 is sound and its precondition is doing real work: it is not a
formality to be discharged once at design time. `phri2` guarantees the torque
envelope unconditionally, because its QP owns the whole command. PIR guarantees
it *conditional on the anchor being feasible*, and the Task 1 gate certified
that condition against exactly one scenario. It does not survive a 4× larger
disturbance. **This is the most important thing the implementation found, and
it is a cost of the split that the synthesis note did not anticipate.**

---

## 8. What this does not show

- **~~The merged controller is not implemented.~~** Discharged in §7.2: the
  replay estimate and the merged closed loop agree to 0.1 % on both rows, and a
  regression test now pins them together. What replaced this caveat is worse —
  see §7.7.
- **The anchor's feasibility was certified against one scenario and does not
  survive a larger one** (§7.7). Everything the gate says is conditional on a
  precondition that fails at 4× the source benchmark's disturbance.
- **The fallback leaves the workspace box in transient.** 62.2 mm peak against
  a 60 mm bound. `phri2`'s box is slack-relaxed rather than hard, so this is
  not a constraint violation in its formulation — but it means the
  $\alpha\to 0$ guarantee is "settles inside the box", not "stays inside it".
  If the bound is to be read as hard, the region is 6 cells, not 33, and the
  recommended cell is outside it.
- **One pose, one push direction, isotropic gains.** The whole gate is
  evaluated at `phri2`'s nominal hold pose under a $-z$ push, with
  $K_0 = k I$, $D_0 = d I$. Joint 4 is the binding joint in every single cell,
  and it is binding largely because of an 18.96 N·m bias torque at that
  specific configuration. An anisotropic $(K_0, D_0)$ — stiffer along the push
  axis, softer elsewhere — is the obvious first lever if the region needs
  widening, and the note explicitly defers it. A different pose could move the
  verdict in either direction.
- **Recursive feasibility is untouched**, in both source papers and here. The
  combined torque-envelope + energy-floor + workspace-slack feasibility
  question is harder jointly than separately and cannot be self-certified.
- **The force-misclassification pillar is untouched.** The tank meters
  $F_r^\top v$; if $F_h$ leaks into $\hat d$, or $F_e$ is folded into the
  disturbance, the power sign is wrong and every certificate here stays green
  while the arm pushes a mislabelled wall. **No green cell in this document
  implies anything about that pillar.**
- **Double tightening is not measured.** `phri2`'s horizon-wide torque
  tightening and `impedance_residual`'s tank tightening compound in series.
  That curve is Task 3 and is expected to get steeper after the merge.
- Simulation only, translational only, affine memoryless behaviour class.

---

## 9. Decision gate

Per §13.3 of the synthesis note, this is where autonomous work stops.

`common_region_nonempty == true` under `phri2`'s derated-joint-4 envelope, the
merged controller is implemented, Merged Lemma 1 holds in closed loop and the
four-term residual closes exactly. What is left is not a coding decision.

§7.6 and §7.7 have changed what the merge can claim. The certified operating
point leaves the residual 2.4 % of joint 4's cap, which makes the merged
controller mostly a stiff PD, makes the passivity axis cheap-because-weak, and
leaves Lemma 1's precondition failing at 4× the source disturbance. Two of the
three questions below are now sharper than they were before the implementation:

**Which envelope does the merged paper claim?** The two are not
interchangeable and they give opposite answers. Three options, in the order I
would rank them:

1. **Write the merge against `phri2`'s derated-joint-4 envelope.**
   $(K_0, D_0) = (380, 29.1)$, region of 33 cells, margins of 2–7 %. Honest,
   and the source benchmark actually runs there. Costs: the tank's numbers
   from `impedance_residual` were measured under $\rho = 0.28$ and would have
   to be re-derived, and the margins are thin enough that §7's first bullet
   could consume them.
2. **Keep $\rho = 0.28$ and abandon the passive-nominal split**, retreating to
   "dissipativity directly on the realized port, no nominal" — a different and
   harder design, and a separate planning decision. §5.1 says this is forced,
   not a preference: under $\rho = 0.28$ no nominal exists.
3. **Change the scenario** — a different nominal pose, a lower push magnitude,
   or a push direction that does not load joint 4 — and re-run this gate.
   Cheap to try (the scan is ~19 min on 4 cores) and it attacks the actual
   binding quantity, which is an 18.96 N·m bias torque at one configuration
   rather than anything about impedance.

Option 3 has gained weight since the implementation. The binding quantity is
still an 18.96 N·m bias torque at one configuration, and §7.7 shows that the
2.4 % of headroom it leaves is not enough to keep Lemma 1's precondition alive
under a moderately larger disturbance. A pose or push direction that does not
load joint 4 would widen the region, restore headroom to the residual, and make
the passivity axis worth something — all three of the implementation's bad
findings at once. I have not chosen among these; the measurements say what each
costs.

### Owed before any claim

- [ ] **Human:** pick the envelope, then confirm $(K_0, D_0)$.
- [x] ~~Implement the merged controller and re-measure rows 1 and 4 in its own
      closed loop~~ — §7.1–7.2. Replay and closed loop agree to 0.1 %.
- [x] ~~Task 3: re-sweep the authorization-vs-tracking curve ($E_0$) with both
      tightenings active~~ — §7.4–7.5. The trade is **flatter**, not steeper,
      and §7.5 says why that is a bad sign rather than a good one.
- [ ] **Task 2, the proof:** write merged Lemma 1 (per-joint $\alpha_\tau$
      closed form, $\alpha_\tau\!\cdot\!\alpha_E$ on-segment lemma) and merged
      Proposition 1 at submittable granularity. The implementation supplies the
      operating point and confirms both halves empirically; the write-up must
      state the precondition as a **standing hypothesis that can fail at run
      time** (§7.7), not as a design-time check.
- [ ] **New, and ahead of the proof:** decide what the controller does when the
      precondition fails. Today it simply overruns the envelope by the anchor's
      own excess. Options: fold the nominal into the authorization loop
      (scale $F_{\mathrm{nom}}$ too, at the cost of the passivity floor), make
      $K_0$ state-dependent, or escalate — which is the seam back into
      `certified-realizability`, exactly where the note said the missing
      transferability leg was.
- [ ] Re-run the Task 1 gate at a pose/push direction that does not load
      joint 4, and check whether the headroom problem is FR3-pose-specific.
- [ ] Try anisotropic $(K_0, D_0)$ before accepting the region as final.
- [ ] Resolve whether the 0.06 m bound is hard (region = 6 cells) or
      slack-relaxed (region = 33 cells).
- [ ] Explicitly disclaim the force-misclassification pillar in whatever is
      written. §7.1's channel split makes the assumption visible; it does not
      discharge it.

---

## 10. Reproducing

```bash
cd simulation
python3 pir_knot_scan.py                     # ~19 min on 4 cores  (Task 1 gate)
python3 pir_knot_scan.py --rescore           # re-derive verdicts/figures from stored maps
python3 pir_verify.py                        # ~3 min              (gate verification)
python3 run_pir_closed_loop.py --scenario push     # ~35 s          (Section 7.2, 7.6)
python3 run_pir_closed_loop.py --scenario merged   # ~35 s          (Section 7.3, 7.4)
python3 pir_e0_sweep.py                      # ~6 min              (Section 7.4, 7.5)
python3 -m pytest test_pir_knot_scan.py test_pir_controller.py -q
```

MuJoCo mesh assets are gitignored repo-wide and must be fetched from MuJoCo
Menagerie into `../simulation/models/franka_fr3/assets/`. Pinned dependency
versions are in `../impedance/simulation/requirements.txt`; `mujoco==3.10.0`
matters, since 3.13 renamed `MjData.qM` and `fr3_mujoco.py` reads it.
