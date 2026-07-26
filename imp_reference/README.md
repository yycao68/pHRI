# Configurable Interaction-Dynamics MPC

This folder contains the v2 paper draft and its first reproducible simulation.
It is intentionally separate from `pHRI/simulation`: the existing controller
uses an LQ tracking cost whose closed-loop gain induces an effective impedance,
whereas this prototype places the error between actual and *specified*
interaction dynamics directly in the MPC objective.

## Run

From this directory:

```bash
# Planar point-mass proof of concept
python3 simulation/run_experiments.py
python3 -m pytest simulation/test_interaction_dynamics_mpc.py -q

# FR3/MuJoCo manipulator study
python3 simulation/run_fr3_experiments.py
python3 -m pytest simulation/test_fr3_interaction_dynamics_mpc.py -q
```

The planar experiment writes `results/metrics.json` and
`results/interaction_dynamics_results.png`. The FR3 experiment writes
`results/fr3_metrics.json` and `results/fr3_interaction_dynamics_results.png`.
The FR3 study imports the shared MuJoCo/operational-space infrastructure from
`pHRI/simulation` (`fr3_mujoco.py`, `fr3_impedance.py`, `so3_utils.py`) rather
than duplicating it.

Required packages are `numpy`, `scipy`, `osqp`, `matplotlib`, `mujoco`, and `pytest`.

## What the prototypes demonstrate

The same constrained MPC realization layer accepts two interchangeable
generators:

1. impedance: `M_d a_id + D_d v + K_d p = f_h`;
2. force-guided admittance: `T a_id + v = Y f_h`.

The MPC chooses robot force to minimize `a_actual - a_id` over the horizon,
subject to robot-force, force-rate, speed, and workspace bounds. The comparator
commands the instantaneous reference acceleration and clips only force and
force rate; it does not predict state constraints.

The planar version is a point-mass proof of concept. The FR3 version replaces
the exact double-integrator plant with a torque-controlled 7-DOF manipulator
in MuJoCo: the QP now enforces per-joint torque feasibility at every predicted
horizon step (not just the first), freezes the task-space inertia/Jacobian at
each solve, and slack-relaxes the Cartesian workspace/speed box rather than
treating it as hard, since a hard box has no recursive-feasibility guarantee
under the frozen-Jacobian approximation. See `paper.md` Section 8 for the
architecture, scenario, results, and what is still out of scope (a nonlinear
generator, disturbance/noise/mass-mismatch sweeps, collision constraints,
baseline comparisons, and human-subject or hardware validation).
