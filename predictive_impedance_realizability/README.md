# Predictive Interaction Realizability (PIR) — passive-nominal design gate

This directory holds **Task 1** of the PIR synthesis programme: the
`(K₀, D₀)` go/no-go scan that decides whether `phri2`'s behaviour–realization
architecture and `impedance_residual`'s energy tank can be merged through a
*passive nominal split*, plus the draft that reports the answer.

Read `predictive_impedance_realizability.md` for the result. In one line:
**the split is viable under `phri2`'s derated-joint-4 envelope and structurally
dead under `impedance_residual`'s ρ = 0.28 envelope** — so the two papers'
stress cases are not interchangeable, and the merge inherits whichever one it
is written against.

## Layout

```
predictive_impedance_realizability.md   the draft
simulation/pir_common.py                scenario constants, replay, the α→0 fallback law
simulation/pir_knot_scan.py             the (K₀, D₀) scan — Task 1 proper
simulation/pir_verify.py                three independent checks on the scan's verdict
simulation/test_pir_knot_scan.py        regression tests
results/                                figures, pir_knot_scan.json, pir_verify.json
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
python3 pir_knot_scan.py          # ~16 min on 4 cores; writes results/pir_knot_scan.json
python3 pir_verify.py             # ~3 min; writes results/pir_verify.json
python3 -m pytest test_pir_knot_scan.py -q
```

`pir_knot_scan.py --quick` runs a coarse grid for smoke-testing. Note that the
coarse grid is *not* a cheap version of the answer: the feasible region is a
thin sliver and a coarse grid reports it empty. That is a finding in its own
right, and §5 of the draft says so.

## Scope

This is the go/no-go gate only. The merged controller, the four-term residual
with `r_auth`, the merged Lemma 1 / Proposition 1 write-up, and the `E₀`
authorization-vs-tracking re-sweep are **not** implemented here, and the
force-misclassification pillar is untouched — no green cell in this directory
implies anything about it.
