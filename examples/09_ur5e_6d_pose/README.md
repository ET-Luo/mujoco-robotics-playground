# UR5e 6D end-effector pose

Status: S10.1 experiment scaffold. The learning task remains incomplete until the learner runs the
experiment, compares it with the prediction, and explains the result.

This example reads the `attachment_site` pose at the UR5e home configuration, adds `0.1 rad` directly
to `wrist_3_joint`, and calls `mujoco.mj_forward`. It does not issue an actuator command, advance
simulation time, solve IK, or test dynamics.

After activating and verifying the `mujoco` conda environment, run from the repository root:

```bash
python examples/09_ur5e_6d_pose/main.py
```

The output separates the raw `(9,)` `site_xmat` storage from its `(3,3)` matrix view. Matrix columns
are the site's local +x/+y/+z axes expressed in world coordinates. The script also reports whether
the site origin lies on the local wrist rotation axis, then checks the predicted position, orientation,
local definition, and time changes.

On 2026-10-02, the assistant ran the script in WSL2 with conda environment `mujoco`, Python 3.12.14,
and MuJoCo 3.14.0. It exited 0: the axis distance and world-position change were both numerically zero,
the world orientation changed, and simulation time remained 0 s. This engineering check does not mark
S10.1 learned; the learner still needs to interpret the output.

## S10.2 orientation-error skeleton

[`orientation_error.py`](orientation_error.py) contains the axis-angle formula for a regular rotation
away from 0 and pi. The learner supplied the two core expressions: composing `R_error_world` and
extracting the skew-symmetric three-vector. Near-zero and near-pi numerical handling is deferred so it
does not obscure the relative-rotation and reference-frame lesson.

On 2026-10-03, the assistant ran it in the verified `mujoco` conda environment with Python 3.12.14.
It exited 0 and returned `[0, 0, 1.570796327]` rad in world axes for an identity current orientation
and a target rotated +90 degrees about world z. S10.2 remains unchecked until the learner explains
the result and the formula's stated edge-case boundary.

## S10.3 full-Jacobian experiment

[`full_jacobian.py`](full_jacobian.py) asks `mujoco.mj_jacSite` to fill both the position and rotation
Jacobians, stacks them into a 6-by-6 matrix, and checks the `wrist_3_joint` column with a direct
`1e-6 rad` configuration change. It compares both the world position difference and the world-frame
rotation vector. This is a kinematic finite-difference experiment: it does not command an actuator,
call `mj_step`, or measure dynamic tracking.

```bash
python examples/09_ur5e_6d_pose/full_jacobian.py
python examples/09_ur5e_6d_pose/full_jacobian.py --joint wrist_2_joint
```

The first command is the verified baseline. The second is the learner's meaningful modification: unlike
the on-axis wrist-3 site, wrist-2 should show a nonzero linear contribution. `--epsilon` is also exposed
for studying the trade-off between nonlinear truncation error and floating-point differencing error.

On 2026-10-03, the assistant ran it in WSL2 using the verified `mujoco` environment, Python 3.12.14,
and MuJoCo 3.14.0. It exited 0. The wrist-3 linear column was numerically zero and its angular column
was approximately `[3.67e-6, 3.67e-6, -1]`; the predicted and measured six-dimensional changes agreed
within the `1e-9` absolute check. S10.3 remains unchecked until the learner interprets the result.

After adding the modification interface, both the default command and `--joint wrist_2_joint` were
re-run on 2026-10-03 and exited 0. The wrist-2 column had a nonzero linear x component of about
`-0.1 m/rad` and an angular direction near world +y, giving the learner a concrete comparison case.
