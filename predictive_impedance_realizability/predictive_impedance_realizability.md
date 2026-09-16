# Is the Passive-Nominal Split Feasible?

### A design gate for Predictive Interaction Realizability on the FR3

*Working draft — Task 1 of the PIR synthesis programme. Simulation only.*

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

§8 states the decision this hands to a human. Task 2 (the merged proof) and
Task 3 (the $E_0$ re-sweep) are **not** started here.

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

Step 2 is the whole of this document. Steps 3–5 are not implemented here.

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

## 7. What this does not show

- **The merged controller is not implemented.** Rows 1 and 4 are evaluated by
  replaying the trajectory `phri2`'s MPC produced. That is exact only if the
  merged controller's *total* command tracks `phri2`'s; its QP has a different
  cost (it optimizes $F_r$, not $F_{\mathrm{cmd}}$) and a different feasible
  set, so its closed loop visits a somewhat different $(q, \dot q)$. Row 1's
  2 % margin at the recommended cell is not large enough to absorb an
  arbitrary amount of that drift. **Re-measuring rows 1 and 4 in the merged
  closed loop is the first thing Task 2 owes.**
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

## 8. Decision gate

Per §13.3 of the synthesis note, this is where autonomous work stops.

`common_region_nonempty == true` under `phri2`'s derated-joint-4 envelope, so
the passive-nominal split is **viable** and Task 2 (merged Lemma 1 /
Proposition 1 at a fixed operating point) and Task 3 (the $E_0$ re-sweep with
both tightenings active) are unblocked — *conditional on a human confirming
$(K_0, D_0)$ and, more importantly, on a decision the note did not anticipate*:

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

I have not chosen among these; the scan says what each costs.

### Owed before any claim

- [ ] **Human:** pick the envelope (the decision above), then confirm $(K_0, D_0)$.
- [ ] **Task 2, first step:** implement the merged controller and re-measure
      rows 1 and 4 in its own closed loop, not on the replay.
- [ ] **Task 2:** merged Lemma 1 (per-joint $\alpha_\tau$ closed form,
      $\alpha_\tau\!\cdot\!\alpha_E$ on-segment lemma) and merged Proposition 1
      at the confirmed operating point.
- [ ] **Task 3:** re-sweep the authorization-vs-tracking curve ($E_0$) with
      both tightenings active; quantify the steeper trade.
- [ ] Resolve whether the 0.06 m bound is hard (region = 6 cells) or
      slack-relaxed (region = 33 cells).
- [ ] Try anisotropic $(K_0, D_0)$ before accepting the region as final.
- [ ] Explicitly disclaim the force-misclassification pillar in whatever is
      written.

---

## 9. Reproducing

```bash
cd simulation
python3 pir_knot_scan.py      # ~19 min on 4 cores -> results/pir_knot_scan.json + figures
python3 pir_knot_scan.py --rescore   # re-derive verdicts/figures from the stored maps
python3 pir_verify.py         # ~3 min  -> results/pir_verify.json
python3 -m pytest test_pir_knot_scan.py -q
```

MuJoCo mesh assets are gitignored repo-wide and must be fetched from MuJoCo
Menagerie into `../simulation/models/franka_fr3/assets/`. Pinned dependency
versions are in `../impedance/simulation/requirements.txt`; `mujoco==3.10.0`
matters, since 3.13 renamed `MjData.qM` and `fr3_mujoco.py` reads it.
