# Jacobian

Status: S4.4 single-column reading and finite-difference check implemented and run;
learner direction and unit interpretation is complete (S4.4 checked).

Goal: Relate joint velocities to end-effector velocity locally.

After activating and verifying mujoco, run from the repository root:

```bash
python examples/05_jacobian/main.py
```

The script reads attachment_site's home-position Jacobian, prints DOF column names,
perturbs only shoulder_pan by 1e-6 rad, refreshes with mj_forward, and compares
(p_after-p_before)/epsilon with the original column. It restores the home pose.
No mj_step, GUI, or IK is used.

2026-09-29 assistant validation: WSL2 Ubuntu 24.04.5, conda mujoco,
Python 3.12.14, MuJoCo 3.14.0; exit 0. Shape (3,6), max column difference
2.458e-7 m/rad (absolute tolerance 1e-6), restored position and time checks pass.

S4.4 has started: interpret the UR5e position Jacobian (3×nv), world-coordinate
rows, hinge columns in m/rad, and small-displacement predictions. The learner
completed the hypothetical displacement calculation and corrected the x-direction sign.
See [the lesson note](../../docs/06_jacobian.md). No IK or controller is planned.

The learner asked how a column is obtained. The note explains FK differentiation
and the hinge formula axis × (site position − joint anchor), all in world axes.
An assistant check at home matched that formula to mj_jacSite within 1e-12;
no new persistent implementation or physics stepping was added for this explanation.

S4.5 has started with the familiar planar two-link model's 2×2 xy Jacobian.
The learner correctly predicted the straight-pose displacement [0,0.0003] m.
After activating and verifying mujoco, run:

```bash
python examples/05_jacobian/planar_singularity.py
```

This prints the analytic xy Jacobian, singular values, and condition number for
q1=0 and q2=π/2, 0.001, and 0 (the last is an exact-singularity reference).
2026-09-29 assistant run: exit 0, Python 3.12.14, NumPy 2.5.3 in conda mujoco.
Hand-calculated matrices and analytic determinants passed checks. Condition numbers
were about 2.42, 4833.33, and infinity. The learner correctly identified weak
response in one direction and distinguished near-singular from singular poses
after clarification of the comparison groups. S4.5 is complete.
No IK, inverse solver, physics, or GUI is used; see the lesson note for conventions.
