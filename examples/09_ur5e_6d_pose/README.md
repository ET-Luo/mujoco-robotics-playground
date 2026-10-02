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
