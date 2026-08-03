# Impedance-Causal Nominal + Two-Rate Residual MPC

This folder contains the rewritten paper, state-of-the-art audit, redesigned
one-dimensional benchmark, raw results, and regression tests. All simulation,
verification, and test code lives in `simulation/`.

## Reproduce the results

```bash
cd simulation
python3 -m pip install -r requirements.txt

MPLCONFIGDIR=/tmp/phri_impedance_fr3_mpl \
XDG_CACHE_HOME=/tmp/phri_impedance_fr3_mpl \
python3 verify_fr3_two_rate_benchmark.py --seeds 20 --leakage-seeds 5 --realism-seeds 5

python3 -m pytest -q \
  test_fr3_two_rate_benchmark.py \
  test_two_rate_passive_residual.py \
  test_residual_mpc.py
```

## Artifacts

- `impedance_residual.md`: rewritten manuscript source.
- `state_of_art_search_log.md`: search protocol, closest-work matrix, and
  explicit novelty boundary.
- `simulation/verify_fr3_two_rate_benchmark.py`: primary torque-controlled
  7-DoF FR3 benchmark, faithful Hannaford--Ryu PO/PC baseline, leakage sweep,
  a sensing-realism sweep (estimator delay, colored noise, velocity-estimate
  bias), statistics, and plotting. Imports the shared FR3/MuJoCo utilities
  from the repo-level `pHRI/simulation/` (two directories up from this
  script).
- `simulation/fr3_two_rate_results.json` and
  `simulation/fr3_two_rate_results.png`: primary raw results and
  reader-facing figure.
- `simulation/test_fr3_two_rate_benchmark.py`: torque-interface, tank-floor,
  and PO/PC regression tests.
- `simulation/verify_two_rate_passive_residual.py`: secondary 1-DoF 50 Hz/1 kHz
  audit.
- `simulation/two_rate_passive_results.json`: full protocol, raw 30-seed
  records, and paired statistics.
- `simulation/two_rate_passive_results.png`: primary reader-facing result
  figure.
- `simulation/test_two_rate_passive_residual.py`: causality, energy-floor,
  and actuator regression tests.
- `simulation/verify_residual_mpc.py`, `simulation/residual_mpc_results.*`,
  and `simulation/test_residual_mpc.py`: retained legacy admittance-reference
  benchmark used to verify the causality--matrix-reuse contrast discussed in
  the paper.
- `impedance_residual.pdf`: rendered paper (regenerated after edits).

The main benchmark includes a direct sweep of intentional-force leakage into
the rejectable-disturbance estimate, plus a satellite sweep adding estimator
delay, colored noise, and a velocity-estimate bias at one representative
leakage level. It verifies controller architecture, not hardware safety or
human-subject performance. The literature search was run manually from
primary publisher/repository records because the optional multi-database API
credentials were unavailable; see the search log for scope.
