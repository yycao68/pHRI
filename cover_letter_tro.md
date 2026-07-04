Yongyan Cao
Voryx Robotic LLC
San Jose, CA 95136, USA
yongyancao@gmail.com

June 7, 2026

To the Editor-in-Chief and Associate Editors,
*IEEE Transactions on Robotics*

**Re: Submission of original research manuscript**

Dear Editors,

We are pleased to submit our manuscript, **"Impedance MPC for Physical
Human–Robot Interaction: Predictive Disturbance Rejection with Joint-Limit
Safety,"** by Yongyan Cao and Jinshan Tang, for consideration as a regular
paper in *IEEE Transactions on Robotics*.

**Problem.** Physical human–robot interaction (pHRI) requires a controller to be
simultaneously accurate during free motion and compliant under unplanned
contact. Classical impedance control faces a structural limitation: under a
sustained human force it settles to a steady-state error equal to the force
divided by the task stiffness (a 15 N push through a 300 N/m spring deflects
50 mm), and an integral channel removes this bias only within a narrow
stable-gain budget. This paper resolves that tension while retaining the
compliance and physical intuition of impedance control.

**Approach and contributions.** We propose a two-layer Impedance MPC:

1. *Constant-$A_d$ architecture.* A feedforward layer analytically cancels
   gravity, Coriolis, and task-space inertia, reducing the residual plant to a
   configuration-independent double integrator. The discrete state-transition
   matrix is therefore constant — the free-response matrix is precomputed once
   and each update reduces to a 30-variable convex QP solved at 100 Hz in under
   1 ms.
2. *Analytical bridge to classical impedance.* We prove (Theorem 1) that in the
   unconstrained, disturbance-free limit the MPC is *exactly* a classical
   task-space impedance controller with positive-definite realized stiffness and
   damping, positioning the method as an extension of impedance control rather
   than a replacement.
3. *Offset-free tracking.* An augmented Kalman filter estimates a persistent
   disturbance state, giving a formal zero-steady-state-error guarantee
   (Theorem 2) under constant or asymptotically constant human forces.
4. *Joint-limit safety.* A dual barrier — a null-space inverse-barrier potential
   plus a Jacobian-projected workspace correction — empirically enforces
   joint-limit safety across the tested workspace, including a boundary stress
   test.

**Key results.** On a 7-DOF Franka FR3, the controller attains sub-0.05 mm
steady-state error versus 44.8 mm for classical impedance (a >800-fold
reduction) under a sustained 15 N force, sub-millimeter tracking on four 3-D
circles, validated joint-limit safety at the workspace boundary, and graceful
robustness to measurement noise and inertial mismatch up to 30%.

**Fit for *IEEE Transactions on Robotics*.** The work pairs a control-theoretic
contribution — an offset-free, constraint-aware operational-space impedance MPC
with formal guarantees — with a real-time, platform-agnostic implementation
requiring only the mass matrix, bias forces, and translational Jacobian. We
believe it is of broad interest to the robotics community working on safe
human–robot collaboration, compliant manipulation, and real-time optimal
control.

**Originality and ethics.** This manuscript is original, has not been published
previously, and is not under consideration for publication elsewhere. All
authors have approved the submission and agree to its content. The work does not
involve human or animal subjects. The authors declare no conflicts of interest.

**Suggested topic areas.** Compliance and impedance control; force and tactile
sensing/control; model predictive and optimal control; physical human–robot
interaction; redundant manipulators.

We thank you for your time and look forward to the reviewers' feedback.

Sincerely,

Yongyan Cao (corresponding author), on behalf of the authors
Voryx Robotic LLC — yongyancao@gmail.com

Jinshan Tang
George Mason University, Dept. of Health Administration and Policy — jtang25@gmu.edu
