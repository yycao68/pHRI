# Behavior–Realization Separation for pHRI

This folder contains a paper and reproducible simulations for a robot-control
architecture that separates desired interaction behavior from constrained
physical realization. The current behavior-layer interface supplies desired
acceleration, and the current realization runtime is a predictive QP. The
validated behavior layers are memoryless affine impedance and admittance;
the architecture is broader than this implementation, but other interfaces
are not claimed as validated.

## Run

From this directory:

```bash
# Architecture schematic (Figure 1)
python3 simulation/make_architecture_diagram.py

# Planar point-mass proof of concept
python3 simulation/run_experiments.py
python3 -m pytest simulation/test_interaction_dynamics_mpc.py simulation/test_benchmark_verification.py -q

# Online generator-switching demonstration (Section 5.3, Figure 3)
python3 simulation/run_generator_switching_experiment.py
python3 -m pytest simulation/test_generator_switching.py -q

# FR3/MuJoCo manipulator study
python3 simulation/run_fr3_experiments.py
python3 simulation/sweep_null_space_gains.py
python3 -m pytest simulation/test_fr3_interaction_dynamics_mpc.py simulation/test_fr3_benchmark_verification.py -q

# Torque-active runtime intervention ablation
python3 simulation/run_torque_activation_experiment.py
python3 -m pytest simulation/test_torque_activation_experiment.py -q

# FR3 QP solve-time study (warm-start on/off, scaled/unscaled conditioning)
python3 simulation/run_fr3_timing_study.py

# Paper PDF (local files only; no rendering server)
python3 simulation/build_paper_pdf.py paper.md
```

`phri2.tex` is a hand-maintained IEEEtran-journal LaTeX rendering of the
same manuscript (theorem/proposition/definition environments, full
bibliography); it is not generated from `paper.md` and must be edited and
recompiled (`pdflatex`) separately if the content changes.

The planar experiment writes `results/metrics.json` and
`results/interaction_dynamics_results.png`. The FR3 experiment writes
`results/fr3_metrics.json` and `results/fr3_interaction_dynamics_results.png`.
The torque-active ablation writes `results/torque_activation_metrics.json` and
`results/torque_activation_results.png`.

The FR3 study imports the shared MuJoCo/operational-space infrastructure from
`pHRI/simulation` (`fr3_mujoco.py`, `fr3_impedance.py`, `so3_utils.py`) rather
than duplicating it.

Required packages are `numpy`, `scipy`, `osqp`, `matplotlib`, `mujoco`, and `pytest`.

The deployed FR3 runtime warm-starts OSQP and uses the exact numerical
reparameterization `z = 1000 s` for each state slack. Physical slack costs,
constraints, and reported units are unchanged; the saved timing study also
reports scaled/unscaled Hessian condition numbers and 20 ms deadline counts.

## What the prototypes demonstrate

The same constrained realization runtime accepts two interchangeable behavior
layers:

1. impedance: `M_d a_id + D_d v + K_d p = f_h`;
2. force-guided admittance: `T a_id + v = Y f_h`.

The predictive runtime chooses robot force to minimize `a_actual - a_id` over
the horizon, subject to robot-force, force-rate, speed, and workspace bounds.
The comparator commands the instantaneous reference acceleration and clips
only force and force rate; it does not predict state constraints.

The planar version is a point-mass proof of concept. The FR3 version replaces
the exact double-integrator plant with a torque-controlled 7-DOF manipulator
in MuJoCo: the QP now enforces per-joint torque feasibility at every predicted
horizon step (not just the first), freezes the task-space inertia/Jacobian at
each solve, and slack-relaxes the Cartesian workspace/speed box rather than
treating it as hard, since a hard box has no recursive-feasibility guarantee
under the frozen-Jacobian approximation. A separate stress case derates joint
4's available budget and compares horizon-wide against first-step-only torque
feasibility. See `paper.md` Sections 6–8 for the experiments, architectural
discussion, and explicit limitations.
