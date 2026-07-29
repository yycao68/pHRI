# Predictive Saturation Experiments

This folder contains the paper draft and its complete deterministic simulation
suite.

The submission-style manuscript is
[`predictive_saturation_paper_submission.md`](predictive_saturation_paper_submission.md).
The longer
[`fast_controller_predictive_saturation_draft.md`](fast_controller_predictive_saturation_draft.md)
is retained as a technical and reproducibility companion.

## Scope

The experiments use a two-dimensional interaction task with three different
actuator geometries:

- `planar_2r`;
- `fr3_surrogate`; and
- `arm6_surrogate`.

The latter two are reduced-order actuator-geometry surrogates. They are not
rigid-body, MuJoCo, hardware, or human-participant validations.

The \(1~\mathrm{kHz}\) fast-controller interfaces are:

- PD;
- impedance;
- a small policy trained by deterministic evolution strategy;
- a fixed-hidden-layer neural policy trained from impedance demonstrations;
  and
- a scripted AI-conditioned behavior proxy with a PD executor.

The AI proxy tests only the software contract. It is not evidence about a
language model, diffusion policy, or foundation model.

## Reproduce all results

```bash
cd pHRI/saturation
python3 -m pip install -r requirements.txt
MPLCONFIGDIR=/tmp/mpl-saturation \
XDG_CACHE_HOME=/tmp/cache-saturation \
PYTHONPATH=simulation \
python3 simulation/run_all_experiments.py
```

The command runs 111 deterministic configurations:

- 40 scenario/baseline cases;
- 30 controller-interface cases;
- 24 cross-realization cases; and
- 17 targeted ablations, including a dedicated ninth scenario (unregistered
  in the main scenario/robot-transfer matrices) isolating the certified
  action set's effect.

It writes:

- `results/all_experiment_metrics.json`;
- `results/representative_logs.npz`;
- `results/directional_authority_results.png`;
- `results/scenario_summary.png`;
- `results/controller_transfer.png`;
- `results/near_boundary_braking_results.png`;
- `results/sampled_interface_audit.png`; and
- `results/ablation_summary.png`.

## Headline results

- Horizon-wide constraints reduce the planned future torque excess from
  \(3.587~\mathrm{Nm}\) to zero in the dedicated horizon-ramp case.
- The same \(0.03~\mathrm{m/s}\) empirical audit threshold contains the
  observed successor defects for all three realization maps; their maximum observed
  defects are \(0.00746\), \(0.00770\), and \(0.00770~\mathrm{m/s}\). This
  audit is not a robust-invariance certificate.
- An exact-pass-through bypass confines intervention to genuinely infeasible
  steps: correction RMSE under no saturation is exactly zero for four of the
  five controller interfaces.
- A concrete, deliberately simple certified action set (velocity-block
  tightening via an \(\infty\)-norm storage function) is now enforced as a
  hard QP constraint alongside the actuator-feasibility set. A dedicated
  ablation confirms it changes the manager's output when it binds, holding
  peak speed at \(0.580~\mathrm{m/s}\) versus \(0.597~\mathrm{m/s}\)
  unconstrained, while leaving the eight main stress scenarios unaffected.
- The slow, directional-collapse, and near-boundary braking cases satisfy
  every sampled interface-audit check under the proposed manager.
- Sudden disturbance and severe model/preview mismatch do not satisfy the
  transfer premises. The final projection still enforces applied actuator
  limits, but cannot restore the requested behavior.
- Timing values are regenerated on each run and stored in
  `results/all_experiment_metrics.json`. Meeting the nominal periods in a
  non-real-time Python run is not a hard real-time guarantee.

## Tests

```bash
cd pHRI/saturation
MPLCONFIGDIR=/tmp/mpl-saturation \
XDG_CACHE_HOME=/tmp/cache-saturation \
PYTHONPATH=simulation \
pytest -q simulation
```

The tests rerun the headline horizon, tightening, final-projection, inactive
manager, controller-interface, and cross-realization sampled interface checks.
