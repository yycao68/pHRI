
  cd /Users/yycao/Documents/git/ai_learn/pHRI/arXiv
  tar czf arxiv_submission.tar.gz phri_main.tex references.bib phri_main.bbl figures/

     🚜 Excavator Autonomous Digging Control — MuJoCo Simulation
 
  I've been working on advanced control strategies for autonomous excavator digging, and just wrapped up a
  simulation comparison across four controllers.
  
  The challenge: a 3-DOF excavator arm must follow an 8-waypoint digging trajectory through soil — while the
  cutting resistance varies nonlinearly with depth, soil cohesion, and rake angle. Classic controllers either
  need a perfect soil model or can't reject the disturbance in real time.

  Four controllers tested:

  🔵 Impedance Control — Cartesian spring-damper at the bucket tip. Tracks well when soil is known, but degrades
   as resistance builds.

  🟣 Impedance MPC — Two-layer architecture: 500 Hz feedforward cancels nominal dynamics, 100 Hz QP outer loop
  rejects soil disturbance predictively. No soil model required. MAE = 19.5 mm, RMSE = 30.0 mm.

  🟠 Joint PD Baseline — Gravity-compensated joint PD. Simple, interpretable — the standard industrial baseline.
  
  🟢 EfficientTrack — Joint PD + a learned trajectory adjustment policy. Neural corrections on top of the
  classical baseline. 

  The standout result: Impedance MPC with a pressure sensor achieves 6× RMS improvement over the reactive-only
  version (35 mm → 6 mm), reaching the full 15 cm reference dig depth. The Kalman-augmented disturbance
  estimator drives steady-state soil force rejection to near-zero — the same principle that makes Kalman
  augmentation powerful in pHRI contact control.

  Why Impedance MPC? The key insight is the two-layer split:
  - Layer 1 (feedforward, 500 Hz): cancels gravity + Coriolis + task-space inertia → residual plant is a linear
  double integrator
  - Layer 2 (QP, 100 Hz): receding-horizon correction on the linear residual → small constant QP, no
  re-linearisation needed
  
  This makes the approach tractable for real-time embedded control, even with unknown soil.

  The video shows the full trajectory visualisation (white upcoming path, colored actual trail, yellow target
  sphere), real-time error inset, per-controller description cards, and a final comparison table.

  Built in MuJoCo with a custom 3-DOF arm model, soil spring-damper contact, and the Fundamental Equation of
  Earthmoving for force generation.

  Code and simulation: open-source MuJoCo stack.
  
  #Robotics #AutonomousExcavation #ModelPredictiveControl #ConstructionTech #RoboticsSimulation #MuJoCo
  #ImpedanceControl #ControlEngineering

   
   🚜 What if excavators could dig themselves?

  Autonomous earthmoving is one of the last frontiers in construction automation. An excavator bucket tip moves
  through unknown, variable soil — cohesion, friction angle, and density all change with every stroke. That
  unpredictability is exactly what makes automation hard.

  I've been building and comparing control strategies for autonomous excavator digging in simulation, and the
  results are striking.

  The core problem: a digging arm must follow a precise trajectory while soil pushes back with forces that can't
   be measured directly — only felt through joint loads. The controller has to reject that disturbance in real
  time, without knowing the soil in advance.

  What I compared:

  A classical baseline (Joint PD), an impedance spring-damper, a learned trajectory correction policy, and a
  two-layer predictive controller I'm calling Impedance MPC — a 500 Hz inner loop that cancels known dynamics,
  plus a 100 Hz receding-horizon corrector that rejects whatever the soil throws at it.

  The results:

  ✅ Impedance MPC tracked the 8-waypoint dig path with 19.5 mm mean error — no soil model required

  ✅ Add a simple cylinder pressure sensor → error drops 6×, the arm reaches full reference dig depth (15 cm)

  ✅ The pressure signal lets the controller estimate soil resistance online and cancel it — the same principle
  behind force estimation in surgical and rehabilitation robotics, now applied to a 20-tonne machine

  Why this matters for the industry:

  - Operators currently rely on feel and years of experience to modulate force through varying ground
  conditions. Automating that judgment reduces fatigue, improves cycle consistency, and opens the door to remote
   or fully autonomous operation.
  - Fuel consumption tracks directly with cycle efficiency — a controller that stays on the planned trajectory
  wastes less energy fighting the wrong direction.
  - Grade control in road construction, trench digging, and foundation work all need sub-50 mm precision. We're
  demonstrating that level of accuracy without site-specific calibration.

  The simulation runs in MuJoCo with a physics-accurate 3-DOF arm, realistic soil contact, and cylinder pressure
   noise modeled from real sensor specs.

  Next step: hardware validation on a scaled hydraulic testbed.

  Happy to connect with anyone working on autonomous construction, hydraulic actuation, or off-highway vehicle
  automation.

  #AutonomousConstruction #Excavator #Earthmoving #ConstructionTech #Robotics #Automation #HeavyEquipment #MPC
  #FutureOfConstruction
  
  
  New work: Impedance MPC for autonomous excavator digging — no soil model required
  
  Sharing a simulation study that may be worth turning into a full paper (or patent application).

  The problem: autonomous excavator control requires tracking a dig trajectory through soil with unknown,
  nonlinearly varying resistance. Most published methods either need a perfect soil model at runtime, or learn
  it slowly across repeated strokes. Neither is ideal for a single-pass dig in changing ground conditions.

  Our approach — Impedance MPC:

  A two-layer architecture that separates the problem cleanly:

  Layer 1 (500 Hz feedforward): analytically cancels gravity, Coriolis, and task-space inertia. After
  cancellation, the residual plant is a constant linear double integrator — independent of arm configuration.

  Layer 2 (100 Hz QP outer loop): a receding-horizon corrector acting only on the linear residual. Because the
  state matrix is constant, the free-response matrix is precomputed once. The online solve is a small,
  warm-started QP — no re-linearisation, no soil model.

  Soil force is treated as an unmeasured disturbance to be rejected, not modelled.

  Key results (MuJoCo simulation, 8-waypoint dig trajectory):

  → MAE = 19.5 mm, RMSE = 30.0 mm — reactive only, zero soil knowledge

  → Add cylinder pressure sensor + Kalman disturbance estimator → 6× RMS improvement, full 15 cm reference dig
  depth achieved

  → The pressure-sensor variant drives steady-state soil force rejection to near-zero — a formal offset-free
  guarantee via the augmented integrating disturbance state

  What makes this novel relative to prior work:

  Most MPC approaches to excavator control (Filla 2017, Egli & Hutter 2022, Sotiropoulos & Asada 2020) linearise
   around a nominal trajectory and re-solve a high-dimensional QP at each step. The constant-A_d structure here
  means the QP size is fixed at 3N variables regardless of configuration — making 100 Hz+ feasible on embedded
  hardware without a GPU or offline trajectory optimisation.

  The Kalman augmentation provides the same offset-free guarantee used in process control (Pannocchia & Rawlings
   2003) and — as we've shown separately — in robot pHRI under human contact forces. This cross-domain
  connection between soil disturbance rejection and human force rejection in manipulation seems underexplored.

  Potential contributions for a paper:

  1. Formal proof that Impedance MPC recovers classical impedance control in the unconstrained limit (already
  derived)
  2. Constant-A_d structure theorem for feedforward-linearised manipulator dynamics
  3. Simulation benchmark: 4-controller comparison (Impedance, Impedance MPC, Joint PD, EfficientTrack) on
  standardised dig trajectory
  4. Pressure-sensor Kalman variant with formal steady-state error bound

  Open to discussing collaboration, co-authorship, or whether this fits better as a patent application given the
   two-layer architecture's potential for industrial deployment.

  If you're working in autonomous earthmoving, hydraulic robotics, or predictive control for underactuated
  systems — let's talk.

  #Robotics #ModelPredictiveControl #AutonomousExcavation #ControlTheory #ResearchHighlight #ImpedanceControl
  #Kalman #ConstructionRobotics


---

New work: Impedance MPC for physical human-robot interaction — near-zero deflection under 15 N contact

The problem: a Franka FR3 arm must hold precise 3D waypoints while a human applies a 15 N push at each one. The robot cannot simply go stiff (it would injure the human) or simply comply (it drifts too far). The controller must resist incidental contact precisely while remaining mechanically safe.

The scenario — reach-and-hold under scripted human push:

Three waypoints (A, B, C) arranged in a triangle. At each waypoint, a 15 N force fires 0.8 s after the robot first arrives, persists for 2.0 s, then releases. The robot must hold within a 35 mm radius for 1.0 s after the push ends before it can advance. Two laps per controller — six push events total. Push direction varies per waypoint (−z at A, +y at B, +z at C) to expose controllers to different disturbance axes.

Seven controllers compared:

🔵 Stiff Impedance (K = 300 N/m) — steady-state deflection = F/K = 15/300 = 50 mm. Confirmed in simulation: peak 49.6 mm. Safe, deterministic, cannot cancel a persistent force.

🟣 Pure Admittance — deflects by design (~227 mm peak). Correct choice for guidance; worst performer by the position-error metric.

🟢 Variable Compliance — blends K from 300 → 80 N/m on contact detection, snaps back after release. Peak 173.6 mm; recovers ~3× faster than admittance.

🟠 Impedance MPC (50 Hz) — two-layer architecture, 200 ms horizon, no Kalman. Peak 7.8 mm. Predictive look-ahead cuts contact error ~8× vs stiff impedance.

🔴 Impedance MPC + Kalman (50 Hz) — adds Kalman disturbance estimator. Contact RMS improves to 2.8 mm. Peak: 15.0 mm — counterintuitively worse than the no-Kalman version (7.8 mm).

🩵 Impedance MPC 500 Hz — same QP, zero-order hold drops from 20 ms to 2 ms. Peak 1.0 mm.

🌸 Impedance MPC + Kalman 500 Hz — best on every metric: 0.2 mm contact RMS, 0.8 mm peak, ~0.1 mm SS error.

The most interesting finding — the Kalman peak paradox:

At 50 Hz, adding the Kalman disturbance estimator improves steady-state accuracy (contact RMS: 5.0 → 2.8 mm) but makes the worst-case first-contact peak substantially worse (7.8 → 15.0 mm). The mechanism is a one-QP-interval estimation lag: at push onset, d̂ has not yet converged, so the QP under-corrects for exactly 20 ms. Over six push events, the maximum onset spike reaches 15 mm.

At 500 Hz, the lag window is 2 ms — short enough that d̂ converges before the error peak can accumulate. The paradox vanishes. G7 (500 Hz + Kalman) achieves both near-zero transient and near-zero steady-state simultaneously.

This reveals a clean orthogonality:
→ QP update rate governs transient peak deflection
→ Kalman estimator governs steady-state residual error
→ The two effects are structurally independent and strictly additive

The same two-layer Impedance MPC architecture that rejects unknown soil forces in our excavator simulation (see previous post) here rejects unknown human contact forces — with the same constant-A_d QP structure, same Kalman augmentation, and same formal offset-free guarantee. The cross-domain consistency suggests this is an architectural property, not a task-specific tuning result.

Potential contributions for a paper:

1. Formal analysis of the Kalman peak paradox in static-reference pHRI (vs. dynamic trajectory, where it is hidden by reference motion)
2. Orthogonality theorem: rate and estimation improve different temporal components of the disturbance response
3. Unified benchmark: 7-controller comparison across both disturbance-rejection (circular trajectory) and compliance-accuracy (reach-and-hold) tasks — showing the ranking is consistent for rejection and inverts for guidance
4. Cross-domain validation: same architecture performs comparably on soil disturbance (excavator) and human contact (pHRI), strengthening the generality claim

The simulation uses the FR3 MuJoCo model from MuJoCo Menagerie, 500 Hz inner loop, and a waypoint-relative push timing design that eliminates the confound between fast-approaching and slow-approaching controllers.

Open to discussing collaboration, co-authorship, or connecting with anyone working in surgical robotics, rehabilitation, or teleoperation where the precision-vs-compliance trade-off is a live design constraint.

#Robotics #pHRI #HumanRobotInteraction #ModelPredictiveControl #ImpedanceControl #ControlTheory #ResearchHighlight #FrankaRobotics #MuJoCo #Kalman #SafeRobotics