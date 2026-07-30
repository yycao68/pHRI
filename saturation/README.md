# Predictive Saturation Experiments

This folder contains the paper draft and its complete deterministic simulation suite.

The paper is [`predictive_saturation_paper_v3.md`](predictive_saturation_paper_v3.md).

## Scope

The experiments use a two-dimensional interaction task with three different actuator geometries:

- `planar_2r`;
- `fr3_surrogate`; and
- `arm6_surrogate`.

The latter two are reduced-order actuator-geometry surrogates. They are not rigid-body, MuJoCo, hardware, or human-participant validations.

The \(1~\mathrm{kHz}\) fast-controller interfaces are:

- PD;
- impedance;
- a small policy trained by deterministic evolution strategy;
- a fixed-hidden-layer neural policy trained from impedance demonstrations;
  and
- a conditioned motion primitive with a PD executor.

The conditioned motion primitive tests only the software contract. It is not evidence about a language model, diffusion policy, or foundation model.

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
- 17 targeted ablations, including a dedicated ninth scenario (unregistered in the main scenario/robot-transfer matrices) isolating the certified action set's effect.

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

- Horizon-wide constraints reduce the planned future torque excess from \(3.587~\mathrm{Nm}\) to zero in the dedicated horizon-ramp case.
- The same \(0.03~\mathrm{m/s}\) empirical audit threshold contains the observed successor defects for all three realization maps; their maximum observed defects are \(0.00746\), \(0.00770\), and \(0.00770~\mathrm{m/s}\). This audit is not a robust-invariance certificate.
- An exact-pass-through bypass confines intervention to steps at which the nominal rollout violates the complete behavior-realization constraint set: correction RMSE under no saturation is exactly zero for four of the five controller interfaces.
- A velocity-only certified action set is enforced as a hard QP constraint alongside the actuator-feasibility set. Its dedicated ablation starts at \(0.565~\mathrm{m/s}\), inside the \(0.600~\mathrm{m/s}\) certificate set and near the tightened \(0.570~\mathrm{m/s}\) nominal-successor bound, and yields peak speeds of \(0.5678~\mathrm{m/s}\) with the constraint and \(0.5968~\mathrm{m/s}\) without it. Both cases remain inside the certificate set; the ablation demonstrates reservation of the physical-defect margin.
- The slow, directional-collapse, and near-boundary braking cases satisfy every sampled interface-audit check under the proposed manager.
- Sudden disturbance and severe model/preview mismatch do not satisfy the transfer premises. The final projection still enforces applied actuator limits, but cannot restore the requested behavior.
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

The tests rerun the headline horizon, tightening, final-projection, inactive manager, controller-interface, and cross-realization sampled interface checks.
