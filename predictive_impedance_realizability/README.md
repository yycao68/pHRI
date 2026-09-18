# Predictive Interaction Realizability (PIR) — passive-nominal design gate

This directory holds the PIR synthesis programme: the `(K₀, D₀)` go/no-go scan
(Task 1), the merged controller (Task 2), the authorization-vs-tracking sweep
(Task 3), and the draft that reports all three.

Read `predictive_impedance_realizability.md` for the result. In three lines:

- The split is **viable** under `phri2`'s derated-joint-4 envelope and
  **structurally dead** under `impedance_residual`'s ρ = 0.28 envelope — the
  two papers' stress cases are not interchangeable.
- The merged controller works: Merged Lemma 1 holds in closed loop, the
  four-term residual closes to machine precision, and `impedance_residual`'s
  fast-vs-manager-rate authorization result transfers to the merged port.
- But at the only certified operating point the nominal is 84 % of the command
  and leaves the residual **2.4 %** of joint 4's torque cap — and Lemma 1's
  precondition, which the fast layer has no authority over, fails at 4× the
  source disturbance. §7.7 of the draft is the thing to read.

## Layout

```
predictive_impedance_realizability.md   the draft
simulation/pir_common.py                scenario constants, replay, the α→0 fallback law
simulation/pir_knot_scan.py             the (K₀, D₀) scan — Task 1
simulation/pir_verify.py                three independent checks on the scan's verdict
simulation/pir_controller.py            the merged controller — Task 2
simulation/run_pir_closed_loop.py       closed-loop runs and variant comparison
simulation/pir_e0_sweep.py              authorization vs tracking — Task 3
simulation/test_pir_knot_scan.py        regression tests for the gate
simulation/test_pir_controller.py       regression tests for the controller
results/                                figures and machine-readable results
paper.css                               the repo's shared paper stylesheet
predictive_impedance_realizability.pdf  the built draft
```

## Nothing here re-derives FR3 dynamics

Every mass matrix, Jacobian, task-space inertia, bias torque, null-space
projector, torque limit and scenario constant is imported from the existing
studies — `simulation/fr3_mujoco.py`, `simulation/fr3_impedance.py`,
`imp_reference/simulation/fr3_interaction_dynamics_mpc.py` and
`imp_reference/simulation/run_fr3_experiments.py`. A second, independent FR3
model would make the scan inconsistent with the papers it is meant to gate.
`test_pir_knot_scan.py` pins this: the 0.06 m bound is asserted to *be*
`FR3MPCConfig.position_limit`, not to equal it.

## Reproducing

```bash
pip install -r ../impedance/simulation/requirements.txt
# MuJoCo meshes are gitignored repo-wide; fetch them from MuJoCo Menagerie into
#   ../simulation/models/franka_fr3/assets/
cd simulation
python3 pir_knot_scan.py                         # ~19 min on 4 cores
python3 pir_verify.py                            # ~3 min
python3 run_pir_closed_loop.py --scenario merged # ~35 s
python3 pir_e0_sweep.py                          # ~6 min
python3 -m pytest test_pir_knot_scan.py test_pir_controller.py -q
```

Rebuild the PDF with the repo's own Pandoc + KaTeX + headless-Chrome builder:

```bash
CHROME_PATH=/path/to/chrome KATEX_DIST=/path/to/katex/dist \
  python3 ../build_paper_pdf.py predictive_impedance_realizability.md
```

On a container running as root, add `CHROME_FLAGS="--no-sandbox
--disable-dev-shm-usage"`. That override was added to `build_paper_pdf.py` in
the same spirit as its existing `CHROME_PATH` / `KATEX_DIST` overrides; it is
empty by default and changes nothing on macOS.

`pir_knot_scan.py --quick` runs a coarse grid for smoke-testing. Note that the
coarse grid is *not* a cheap version of the answer: the feasible region is a
thin sliver and a coarse grid reports it empty. That is a finding in its own
right, and §5 of the draft says so.

## Scope

Implemented: the gate, the merged controller, the four-term residual with
`r_auth`, and the `E₀` re-sweep.

**Not** implemented, and owed: the merged Lemma 1 / Proposition 1 write-up at
submittable granularity, a decision about what the controller should do when
Lemma 1's precondition fails at run time (today it simply overruns), and the
recursive-feasibility / escalation leg.

The force-misclassification pillar is untouched. `pir_controller.control()`
takes the behaviour input and the disturbance model as separate channels, which
makes the assumption visible in a signature — it does not discharge it. No
green result in this directory implies anything about that pillar.
