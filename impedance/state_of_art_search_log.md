# State-of-the-art search log

Search date: 2026-08-02. Scope: robot impedance/admittance control combined
with model predictive control, passivity constraints, energy tanks, additive or
residual correction, and multi-rate realization for physical interaction.

## Method and limitation

The intended Deep-tier multi-database workflow was stopped at its required
setup check because OpenAlex, Semantic Scholar, Crossref, and NCBI credentials
were unavailable in this environment. The search therefore used targeted title,
concept, DOI, publisher, institutional-repository, and arXiv queries. It is a
transparent state-of-the-art audit, not a PRISMA systematic review. The missing
API setup means citation-network expansion and database-level deduplication were
not performed.

Query clusters included:

- `residual MPC impedance control passivity robot interaction`
- `energy tank model predictive control impedance robot interaction`
- `passivity-constrained MPC impedance control pHRI`
- `nominal impedance residual controller model predictive`
- exact-title searches for the closest retrieved articles
- `feedback error learning inverse dynamics robot`
- `sampled time passivity observer controller haptics`

Inclusion required a primary publisher record, DOI landing page, author
repository, or arXiv record and a direct architectural connection. Generic MPC,
generic impedance reviews, and residual-learning papers without physical
interaction were used only as background.

## Closest work and consequence for novelty

| Work | Predictive role | Passivity mechanism | Validation | Relation to this draft |
|---|---|---|---|---|
| Cao, Cheng, and Li, TCDS 2023, doi:10.1109/TCDS.2023.3275217 | Top-loop MPC computes complementary torque over a bottom variable-impedance loop | Stored-energy constraint; feasibility and passivity analysis | Franka simulation and experiment | The closest prior architecture; rules out claiming that passive impedance plus predictive complementary torque is new by itself |
| Haninger, Hegeler, and Peternel, RAS 2023, doi:10.1016/j.robot.2023.104431 | MPC plans trajectory and impedance using GP interaction models | Safety and contact-stability constraints, not the same residual tank | Multiple collaborative robot tasks | Strong predictive-impedance baseline; optimizes impedance rather than authorizing a residual wrench |
| Xue et al., RAS 2025, doi:10.1016/j.robot.2025.104961 | Model-predictive variable impedance with estimation/robustness | Passive MPVIC construction | Silicone and human-arm experiments | Richer robot validation; again changes impedance rather than isolating a residual wrench |
| Shen et al., Mechatronics 2025, doi:10.1016/j.mechatronics.2025.103340 | MPC/QP trades impedance tracking against a passivity index | Soft passivity constraint | Heavy-duty hydraulic manipulator | Confirms that passivity-constrained predictive VIC is established |
| Mahfouz et al., RA-L 2026, doi:10.1109/LRA.2026.3666354 | MPC optimizes admittance parameters for assistive/resistive modes | Embedded passivity constraints | Kinova Jaco-2, seven subjects | Recent admittance-causal overlap; force-to-motion, unlike the selected impedance-causal branch |
| Guo et al., T-RO 2025, doi:10.1109/TRO.2025.3546856 | Switching controller, not MPC | Ultimate passivity trades conservative and nominal modes | Impedance and admittance robots | Important non-MPC comparator for performance/passivity tradeoffs |
| Ferraguti et al., T-RO 2015, doi:10.1109/TRO.2015.2455791 | Variable admittance and autonomy/teleoperation layers | Energy tanks | Surgical robot prototype | Establishes layered energy-tank interaction architectures well before this work |
| Hannaford and Ryu, TRA 2002, doi:10.1109/70.988969 | No prediction | Sample-level passivity observer/controller | Haptic hardware | Foundational comparator for the proposed 1 kHz energy authorization |

## Defensible contribution after the audit

The literature does **not** support claiming a new combination merely because a
passive impedance loop and an MPC correction are stacked. A defensible paper can
instead study the narrower implementation gap:

1. preserve an explicitly impedance-causal physical nominal mapping
   (motion/error to wrench), rather than calling a force-to-motion reference
   model "impedance feedforward";
2. restrict the predictive correction to a separately logged residual-wrench
   port;
3. couple residual energy authorization and total actuator saturation in a
   1 kHz projection while solving the predictive allocation at 50 Hz; and
4. demonstrate, with an inter-update pulse, why manager-rate passivity checks do
   not certify the held command between updates.

This choice has a cost: the residual dynamics contain the rendered stiffness
and damping. Therefore the earlier gain-independent residual-QP claim is not
retained. The paper presents this as a causality--reuse tradeoff, not as a
universal certificate-transfer theorem.

## Benchmark decision

The primary implemented benchmark now uses the torque-controlled 7-DoF Franka
FR3 MuJoCo model. It contains four matched controllers: passive impedance,
unguarded residual MPC, an equation-faithful 3-D translational generalization of
the Hannaford--Ryu impedance-causal PO/PC, and the proposed finite-tank two-rate
controller. It sweeps passive-wall stiffness/damping, disturbance amplitude and
phase, and separately sweeps intentional-force leakage into the disturbance
estimate. The complete derated joint-torque interface is checked at 1 kHz.

Cao et al. (2023) remains the closest predictive robot-level comparator, but is
not relabeled or approximated in the current numerical table. The implemented
Hannaford--Ryu baseline was selected because its published equations permit a
transparent, auditable reproduction. A future hardware study should add Cao et
al. and Guo et al. if complete implementation details can be matched.
