# Impedance-Causal Nominal + Two-Rate Residual MPC

This folder contains the rewritten paper, state-of-the-art audit, redesigned
one-dimensional benchmark, raw results, and regression tests. All simulation,
verification, and test code lives in `simulation/`.

## Reproduce the results

```bash
# One-time environment setup (Python >= 3.10).
cd simulation
python3 -m pip install -r requirements.txt

# One-time FR3 model download -- the primary 7-DoF benchmark below imports
# shared FR3/MuJoCo utilities from the repo-level pHRI/simulation/ (two
# directories up), whose mesh assets are gitignored (large binaries) and
# must be fetched once via that tree's own setup script. Skip this step and
# the benchmark fails at MuJoCo model load (found by external review; see
# ../../simulation/setup_model.py and its test_model_smoke.py).
python3 ../../simulation/setup_model.py

# Fast smoke test: does the model load and can a short trial run at all,
# before paying for the full 20-trial benchmark below.
python3 -m pytest -q test_fr3_smoke.py

MPLCONFIGDIR=/tmp/phri_impedance_fr3_mpl \
XDG_CACHE_HOME=/tmp/phri_impedance_fr3_mpl \
python3 verify_fr3_two_rate_benchmark.py --seeds 20 --leakage-seeds 5 --realism-seeds 5

python3 -m pytest -q \
  test_fr3_two_rate_benchmark.py \
  test_two_rate_passive_residual.py \
  test_residual_mpc.py \
  test_fr3_smoke.py
```

## Rebuild the paper PDF

```bash
# From this directory (impedance/); requires pandoc, KaTeX, and headless
# Chrome, all resolved from the repo-level pHRI/build_paper_pdf.py.
python3 ../build_paper_pdf.py impedance_residual.md --output impedance_residual.pdf
```

Note: the currently-committed `impedance_residual.pdf` was produced by a
different toolchain (`LaTeX via pandoc`/`xdvipdfmx`, 17 pages) than this
script currently produces (Pandoc+KaTeX+headless-Chrome, 11 pages, verified
directly) -- both render the same manuscript content, just with different
page counts/fonts from the different pipelines. This section documents how
to build it, not which historical toolchain produced the currently-checked-
in file; regenerate and review before replacing it if page-count/layout
parity with the committed PDF matters for your purpose.

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
