# T-RO Recovery Plan

The T-RO rejection was mainly about maturity: the manuscript is framed as
physical human-robot interaction, but the evidence is simulation-only and some
formal/safety claims are stronger than the implementation.

## Immediate Code Alignment

- The MPC now constrains the first applied joint torque inside the QP:
  `tau_base + J_v.T @ F_mpc(0)`, where `tau_base` includes feedforward,
  orientation, and null-space torques.
- The previous implementation only bounded Cartesian `F_mpc` and then relied
  on downstream torque clipping/rate limiting. Do not claim full horizon
  torque feasibility until the predicted-horizon torque rows are also added.
- Keep hardware clipping/rate limiting as the last software safety layer, but
  describe it as a runtime guard, not as the primary MPC constraint.

## Manuscript Reframing Before New Hardware Data

Change strong claims:

| Current wording | Safer wording |
|---|---|
| pHRI validation | simulated pHRI disturbance benchmarks |
| joint-limit safety | certified only when the one-step CBF filter is enforced and feasible |
| actuator never saturates | first applied torque is constrained when feasible; runtime guards remain |
| On a Franka FR3 | in a MuJoCo Franka FR3 model |
| formal zero-steady-state guarantee | offset-free result under stated stability, feasibility, and inactive steady-state constraint assumptions |

Recommended temporary title:

`A Linear Double-Integrator Backbone for Safe Physical Human-Robot Interaction`

Use the stronger pHRI/T-RO title only after real FR3 human-interaction data are
included.

## Real FR3 Experiments Needed For T-RO

Minimum hardware package:

| Test | Purpose | Required evidence |
|---|---|---|
| H1 hold | first-contact stability | error, torque, velocity, compute time |
| H2 small circle | tracking on real robot | command vs measured trajectory, RMSE, max error |
| H3 hand push in hold | pHRI disturbance rejection | push video, recovery time, `d_hat`, steady-state error |
| H4 hand push during circle | pHRI during motion | trajectory recovery, torque smoothness |
| H5 known payload | persistent disturbance estimate | payload mass, expected force, `d_hat`, SS error |
| H6 boundary ramp | joint-limit safety | min joint margin, CBF residual/slack if enabled, tracking tradeoff, no violation |

Use `pHRI/cloud_verify/verification/REAL_HARDWARE_TEST_PLAN.md` as the runbook.
All plots and metrics should come from logged CSV, not video-only evidence.

## Paper Additions For A Strong Resubmission

- Add a real hardware section with photos/video frames and synchronized plots.
- Add a table comparing simulation and hardware metrics.
- Add compute-time distribution on the real control computer: mean, p99, max.
- Add torque feasibility evidence: peak torque, RMS torque, max per-step torque
  change, and whether runtime clipping occurred.
- Add a limitations paragraph: no certified passivity and no formal
  human-subject study unless IRB/ethics approval exists; joint-limit safety is
  certified only for runs where the one-step CBF filter is feasible, enforced,
  and logged.

## Theory Work Needed For T-RO Strength

- Expand Theorem 2 from proof sketch into assumptions, observer error dynamics,
  closed-loop stability condition, and steady-state argument.
- Keep the forward-invariance result for joint limits, and ensure hardware
  experiments log the CBF residuals/slack if claiming certified safety.
- If claiming recursive feasibility, add slack-variable QP implementation and
  document the slack metric in experiments.
- If claiming total-torque constraints over the horizon, implement predicted
  torque rows for all `k = 0...N-1` or explicitly state only the first receding
  horizon action is constrained with the current model.

## Suggested Venue Strategy

- With only simulation: revise for RA-L/IROS/ICRA or arXiv.
- With real FR3 disturbance experiments and tightened proofs: T-RO becomes
  realistic.
- With real human-subject experiments, passivity/energy analysis, and multiple
  task scenarios: much stronger T-RO case.
