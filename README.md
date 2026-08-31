# pHRI

Research papers, simulation code, and hardware-verification code for physical
human–robot interaction (pHRI): predictive/impedance control, interaction
dynamics, safety certification (torque limits, energy tanks, passivity), and
behavior-realization architectures, mostly on a Franka FR3 manipulator
(MuJoCo simulation, with real-hardware verification on an OpenManipulator-X
and FR3 hardware-interface code for first real-robot tests).

## Shared infrastructure

- **`simulation/`** — the shared FR3/MuJoCo environment (`fr3_mujoco.py`,
  `fr3_impedance.py`, `so3_utils.py`) that every FR3-based sub-project below
  imports from two directories up. Also hosts its own demos, benchmarks, and
  the primary double-integrator pHRI controller and its verification suite.
  FR3 mesh assets are gitignored (large binaries, Apache-2.0-licensed from
  [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie));
  run `python3 simulation/setup_model.py` once before using any sub-project
  that needs the FR3 model. See `simulation/README.md`.
- **`cloud_verify/`** — the hardware-path MuJoCo suite and the real-FR3
  torque-interface adapter (energy-tank passivity, sampled joint-limit CBF
  safety, certified at torque level). Gitignored (large/credentialed); see
  `cloud_verify/README.md` for what it validates and how it relates to the
  algorithmic demos in `simulation/`.
- **`build_paper_pdf.py`** — local Pandoc+KaTeX+headless-Chrome markdown→PDF
  build, no rendering server. Usage: `python3 build_paper_pdf.py <paper.md>
  --output <paper.pdf>`, run from the folder containing the markdown. Some
  papers are LaTeX-native instead (built with `latexmk -pdf`); see each
  sub-project's own README for which applies.

## Papers and sub-projects

| Folder | Paper / focus | Status |
|---|---|---|
| `arXiv/` | "Interaction Dynamics" — predictive interaction-dynamics MPC for fixed-base pHRI. Two live variants: `phri_ICRA.tex` (8pp ICRA fork) and `phri_combined.tex` (16–17pp fuller version). `phri_main.tex`/`phri_main2.tex`/`phri_supplement*.tex` are earlier, superseded drafts kept for history. | Active |
| `saturation/` | Predictive-saturation certificate (K_cert) paper — a behavior-coordinate realization interface for predictive saturation management, targeting SCL/L-CSS. Current paper: `predictive_saturation_paper_v4.md` (+ `predictive_saturation_arxiv.tex`). | Active |
| `impedance/` | Two-rate residual MPC — impedance-causal nominal control plus a 100 Hz MPC residual authorized by a 1 kHz energy-tank/torque projection. Paper: `impedance_residual.md`. Code-reviewed 2026-08-30 (OSQP validation, numerical-warning hardening, FR3 reproducibility); see `impedance/README.md`. | Active |
| `imp_reference/` | Behavior-realization separation — an interaction-behavior generator (desired acceleration) decoupled from a constrained realization layer (predictive QP). Paper: `phri2.tex` (kept in sync with `paper.md`). Code-reviewed 2026-08-30 (same three fixes as above); see `imp_reference/README.md`. | Active |
| `openmanipulator_verify/` | Real-robot torque-level verification of the interaction-dynamics controller on a ROBOTIS OpenManipulator-X (4-DOF, current-control mode via the DYNAMIXEL SDK, not LeRobot). | Active |
| `funding/` | Grant/demo material (Voryx Robotics), including a MuJoCo demo video and its MPC. | Supporting material |
| `TASE_FINAL.pdf` | Third-party related-work paper (Salt Ducaju, Olofsson, and Johansson, *IEEE T-ASE*) kept for reference/citation — not authored in this repo. | Reference (external) |
| `paper_review_guide.md` | Internal checklist used when running a review pass over a paper draft in this repo. | Internal tooling |

## Conventions across sub-projects

- Each paper folder's simulation code lives in its own `simulation/` and
  imports the shared FR3 environment from the repo-level `simulation/` two
  directories up (`sys.path.insert(0, ...)` at the top of the relevant
  files) — run the repo-level `setup_model.py` once, not per sub-project.
- Papers are drafted in Markdown and/or LaTeX; where both exist they are
  kept manually in sync (see each sub-project's own README for which file
  is the source of truth and the exact build command).
- Result artifacts (`*.json`, `*.png`) committed alongside simulation code
  are evidence snapshots, not build outputs — re-run the corresponding
  script to regenerate and compare before trusting a number in a paper
  against a stale artifact.
