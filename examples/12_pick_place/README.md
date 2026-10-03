# UR5e known-pose pick and place

## S11.1 — Integrated gripper, unloaded

This example attaches a deliberately simple two-finger gripper to the official UR5e
`attachment_site` with `MjSpec.attach`. It verifies the attachment transform, finger
joint/actuator transmissions, collision geometry, and symmetric unloaded opening. It
does not contain an object and therefore proves neither grasping nor collision-free arm
motion.

```bash
python examples/12_pick_place/integrated_gripper.py
python examples/12_pick_place/integrated_gripper.py --target 0.015
```

The target is each finger's outward slide displacement, not the total opening. With
finger half-width `0.005 m` and mounting centers at `±0.015 m`, the inner opening is
`opening = 0.02 m + q_left + q_right`. Thus the default target produces `0.08 m`, while
`0.015 m` produces `0.05 m`. Writing `ctrl` sets targets immediately; `qpos` approaches
them only through repeated `mj_step` calls.

## S11.2 — Known object, grasp, and pre-grasp poses

This geometry-only example defines a top-down grasp for a box with a known world pose.
The gripper's local `+z` is the approach axis and points along world `-z`; local `+x`
is the finger opening axis. No IK or motion is executed.

```bash
python examples/12_pick_place/pose_planning.py
python examples/12_pick_place/pose_planning.py --pregrasp-distance 0.15
```

The second command changes only the retreat distance. From pre-grasp to grasp, the
world displacement should be `[0, 0, -distance] m`, while the same vector expressed in
the gripper frame is `[0, 0, +distance] m`.
