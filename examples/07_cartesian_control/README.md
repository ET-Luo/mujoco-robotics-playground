# Cartesian control

Status: planar single-step and repeated geometric command examples implemented.

Goal: Express a motion objective in end-effector coordinates.

Scope: position-only Jacobian mapping with joint-step and command-speed limits.

S6.1: the position target and error are expressed
in world coordinates and meters; the Jacobian maps a small world-position correction
to joint increments in radians.

The learner answered the S6.1 frame and unit questions correctly; S6.1 is complete.
S6.2 has started with a hand calculation that uniformly scales a raw joint increment
to a 0.004 rad maximum absolute component. After correction, the learner confirmed
scale 0.4, limited increment (0, 0.004) rad, and predicted displacement
(-0.0012, 0) m. The assistant added [main.py](main.py) and ran it on 2026-09-29
in the checked mujoco environment (Python 3.12.14): exit 0, no import errors.
FK gave displacement (-0.0011999968, -0.000002399997) m and reduced error from
0.003 m to 0.0018000048 m. The limit and error-reduction assertions passed.

The learner correctly explained why scaling prevents a one-step arrival, confirmed the
0.004 rad limit, and identified finite-angle circular geometry as the small y discrepancy.
S6.2 is complete. S6.3 derives a per-update step from control period and maximum joint speed.

After correcting the period relationship, the learner confirmed that 0.2 rad/s over
0.01 s permits a 0.002 rad step. The assistant added [tracking.py](tracking.py).
On 2026-09-29 it ran in the checked mujoco environment with Python 3.12.14: exit 0,
no import errors. Three updates reduced error from 0.003 m to 0.000000590488 m at
the assumed time 0.06 s. The second joint command hit 0.2 rad/s for the first two
updates and fell to about 0.098206 rad/s for the third. Speed-limit assertions passed.
These are geometric command velocities; no MuJoCo dynamics, measured qvel, or GUI was run.
Learner interpretation is pending.
