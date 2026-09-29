# Inverse kinematics

Status: single planar position update implemented in [main.py](main.py).

Goal: Find a joint configuration for a desired end-effector pose.

Scope: one update at a fixed initial pose, followed by nonlinear FK verification.

S5.1 is complete with a planar two-link position-only hand calculation:
world-frame error, solving J*delta_q=error, step fraction alpha, and position
tolerance versus iteration-limit termination. See [the lesson note](../../docs/07_inverse_kinematics.md).
After correction of the error sign and half-step arithmetic, the learner correctly
predicted x=0.3985 m. An assistant FK calculation gives about 1.5 mm residual error;
the learner correctly explained that this exceeds the 0.1 mm success tolerance.

S5.2: the learner supplied the three correct update expressions. After verifying
WSL2 Ubuntu 24.04.5 and activating/checking the mujoco environment, the assistant ran:

```bash
python examples/06_inverse_kinematics/main.py
```

2026-09-29, Python 3.12.14: exit 0, no import errors. Error decreased from
0.003 m to 0.001500010937 m, still above the 0.0001 m tolerance, as predicted.
The learner correctly judged the error reduction and unmet success tolerance.
After initially choosing the linear prediction for validation, the learner received
an explanation and correctly confirmed that a full step also requires FK recomputation.
S5.2 is complete in the root README. S5.3 has started with a concept exercise:
refresh FK/Jacobian each iteration, allow at most 20 updates, and reject candidates
outside the teaching limits [-pi, pi] rad for each joint. The learner correctly
answered the limit and termination questions. [iteration.py](iteration.py) now
contained three TODOs: success, limits, and accepting the candidate. The learner filled
all three correctly. On 2026-09-29 the assistant ran it in the checked mujoco environment
(Python 3.12.14): exit 0, no import errors. Error decreased from 0.003 m to
0.000093753023 m after five accepted updates and the solver reported success.
All candidates stayed within the teaching limits, so the limit-failure branch was not exercised.
The learner correctly explained the six records for five updates, tolerance-based success,
and why this run does not validate the unvisited limit-failure branch. S5.3 is complete.
S5.4 uses `--target-x` and `--target-y`; defaults preserve the reachable S5.3 case.
The learner correctly predicted the 0.1 m geometric lower bound for target (0.8, 0) m.
On 2026-09-29 the default case still succeeded after five updates. The unreachable case:

```bash
python examples/06_inverse_kinematics/iteration.py --target-x 0.8 --target-y 0.0
```

exited 0 after reporting `FAILED: update limit reached`. Error was nonmonotonic near the
straight-arm singularity and ended at 0.165763890 m after 20 updates. No joint limit or
linear-solve failure occurred. Learner interpretation is pending; no physics or GUI was run.
The fixed Jacobian in main.py is specific to its initial pose; iteration.py includes
a function to recompute it. No physics or GUI is run.
