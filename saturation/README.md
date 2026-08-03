# Predictive Saturation Experiments

This folder contains the paper draft and its complete deterministic simulation suite.

The current paper is [`predictive_saturation_paper_v4.md`](predictive_saturation_paper_v4.md).
The previous v3 manuscript is retained for revision history.

For anonymous review, the same self-contained files are distributed in
`predictive_saturation_v4_supplementary.zip`; its SHA-256 digest is written to
`predictive_saturation_v4_supplementary.zip.sha256`. The archive contains this
README, the dependency list, simulation and test code, saved metrics, and the
figures used by the paper.

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
- A matched ERG-style horizon trajectory-reference governor uses the same model,
  20 ms update, 0.24 s horizon, actuator-limit schedule, uncertainty tightening,
  and final projection as the proposed manager. Both handle the successful
  constraint cases comparably; the proposed vector correction has lower
  correction RMSE in three of the four successful or horizon-isolation cases.
  This comparator is not an exact reproduction of the Lyapunov/dynamic-safety-
  margin method cited by the paper.
- The torque-error envelope is fixed from declared coefficient boxes before the
  simulated true plants are generated. Separate deterministic seeds (31, 37,
  and 41) select held-out coefficients, phases, and frequencies that are not
  supplied to the manager. This is a synthetic held-out uncertainty test, not
  identification from hardware data.
- The paper now gives the complete operational-space bias/secondary-torque
  condition under which the requested acceleration is realized, and a separate
  two-rate corollary stating the inter-update torque-drift bound needed to
  certify the actual 1 kHz signal. The benchmark audits that signal after the
  fact but does not claim an analytical drift bound.
- The broader unpaired trajectory audit uses the Euclidean norm; its maximum
  observed successor defects are \(0.00746\), \(0.00770\), and
  \(0.00770~\mathrm{m/s}\) for the three realization maps. The paired
  start/end audit used by Table IV instead reports maximum \(\ell_\infty\)
  defects of \(0.006815\), \(0.006796\), and \(0.006796~\mathrm{m/s}\).
  Both use the same \(0.03~\mathrm{m/s}\) threshold, but the values are not
  directly interchangeable because their norms and record sets differ.
  Neither audit is a robust-invariance certificate.
- An exact-pass-through bypass confines intervention to steps at which the nominal rollout violates the complete behavior-realization constraint set: correction RMSE under no saturation is exactly zero for four of the five controller interfaces.
- A velocity-only certified action set is enforced as a hard QP constraint alongside the actuator-feasibility set. Its dedicated ablation starts at \(0.565~\mathrm{m/s}\), inside the \(0.600~\mathrm{m/s}\) certificate set and near the tightened \(0.570~\mathrm{m/s}\) nominal-successor bound, and yields peak speeds of \(0.5678~\mathrm{m/s}\) with the constraint and \(0.5968~\mathrm{m/s}\) without it. Both cases remain inside the certificate set; the ablation demonstrates reservation of the physical-defect margin.
- The slow, directional-collapse, and near-boundary braking cases satisfy every sampled interface-audit check under the proposed manager.
- Sudden disturbance and severe model/preview mismatch do not satisfy the transfer premises. The final projection still enforces applied actuator limits, but cannot restore the requested behavior.
- Timing values are regenerated on each run and stored in
  `results/all_experiment_metrics.json`. Meeting the nominal periods in a
  non-real-time Python run is not a hard real-time guarantee, and another run
  may contain wall-clock outliers above either nominal period.

## Tests

```bash
cd pHRI/saturation
MPLCONFIGDIR=/tmp/mpl-saturation \
XDG_CACHE_HOME=/tmp/cache-saturation \
PYTHONPATH=simulation \
pytest -q simulation
```

The tests rerun the headline horizon, matched-reference-governor anticipation,
tightening, final-projection, inactive manager, controller-interface, and
cross-realization sampled interface checks. The current suite contains 12 tests.
