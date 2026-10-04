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

## S11.3 — Home to pre-grasp reference

This example reuses bounded DLS IK and a zero-end-velocity cubic joint reference to
move geometrically from UR5e home to S11.2's `0.10 m` pre-grasp target. At every sampled
configuration it checks arm joint limits and MuJoCo contact pairs against the attached
open gripper and fixed box. Direct `qpos` assignment plus `mj_forward` is reference-path
validation, not actuator tracking.

```bash
python examples/12_pick_place/pregrasp_motion.py
python examples/12_pick_place/pregrasp_motion.py --duration 5
```

The longer duration follows the same joint-space path and has the same sampled poses
when `dt` is unchanged, but lowers analytic joint velocity in inverse proportion to
duration.

## S11.4 — Local-axis approach and bilateral contact

This example keeps the fingers open while solving a sequence of IK targets from
pre-grasp to grasp along local `+z_G`. It prints the same achieved displacement in
world and gripper coordinates. At grasp, MuJoCo dynamics closes both fingers and stops
at the first simultaneous left/object and right/object contact.

```bash
python examples/12_pick_place/approach_and_close.py
python examples/12_pick_place/approach_and_close.py --approach-distance 0.08
```

The fixed object isolates contact detection. Bilateral contact is only the phenomenon
for this lesson; it does not establish force closure, stable grasping, lifting, or
retention.

## S11.5 — Small lift with a free object

This example replaces the fixed box with a freejoint box on a ground plane. After
closing and settling under gravity, the UR5e position actuators follow a cubic lift
reference while the script measures object world-height change, object-to-gripper
relative motion, and bilateral finger contact retention.

```bash
python examples/12_pick_place/lift_object.py
python examples/12_pick_place/lift_object.py --lift-height 0.03
```

Success requires the object to rise to within `0.01 m` of the requested lift, vertical
relative slip no greater than `0.01 m`, bilateral contact during at least 95% of lift
steps, and bilateral contact at the final state. It does not test transfer or placement.

## S11.6a — Held transfer and supported descent

This example starts from the S11.5 held state, performs a speed-limited horizontal
transfer, then descends to a known object pose on the ground. It monitors commanded and
actual arm speed, relative-z drop evidence, bilateral contact, final object pose error,
and object-ground support. The fingers remain closed.

```bash
python examples/12_pick_place/transfer_and_descend.py
python examples/12_pick_place/transfer_and_descend.py --transfer-duration 2
```

The shorter transfer uses the same endpoints with a higher cubic command speed that
must remain below the configured limit. Compare speed margin against the shorter time
available for gravity-driven slip. This lesson does not release or retreat.

## S11.6b — Supported release and retreat

This example reproduces the supported S11.6a state, confirms support before opening,
waits until both finger-object contacts disappear, and then retreats opposite local
`+z_G`. Final checks cover object pose, support, finger separation, gripper clearance,
and low object linear/angular speed.

```bash
python examples/12_pick_place/release_and_retreat.py
python examples/12_pick_place/release_and_retreat.py --retreat-height 0.12
```

## S11.7 — Fixed-seed small variations

Run 20 complete trials with small variations in known object xy, sliding friction, and
mass. The actual sampled xy remains known to the grasp planner. Results report success
rate, failure stage, and successful final-position errors.

```bash
python examples/12_pick_place/robustness_trials.py
python examples/12_pick_place/robustness_trials.py --seed 7
```

This is a compact robustness check, not broad domain randomization or policy training.
