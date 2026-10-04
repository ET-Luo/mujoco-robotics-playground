"""Close on a free box, lift it, and check height, slip, and contact retention."""

import argparse

import mujoco
import numpy as np

from approach_and_close import actuator_ids
from pregrasp_motion import (
    ARM_JOINTS,
    FINGER_JOINTS,
    build_model,
    contact_names,
    cubic_reference,
    joint_addresses,
    site_pose,
    solve_pregrasp_ik,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lift-height", type=float, default=0.05)
    parser.add_argument("--lift-duration", type=float, default=1.5)
    parser.add_argument("--close-target", type=float, default=0.005)
    parser.add_argument("--settle-steps", type=int, default=500)
    args = parser.parse_args()
    if not np.isfinite(args.lift_height) or args.lift_height <= 0.01:
        parser.error("--lift-height must be finite and greater than 0.01 m")
    if not np.isfinite(args.lift_duration) or args.lift_duration <= 0.0:
        parser.error("--lift-duration must be positive and finite")
    if not 0.0 <= args.close_target <= 0.04:
        parser.error("--close-target must be in [0, 0.04] m")
    if args.settle_steps <= 0:
        parser.error("--settle-steps must be positive")
    return args


def bilateral_contact(model, data, left_pair, right_pair):
    pairs = contact_names(model, data)
    return left_pair in pairs and right_pair in pairs, pairs


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)
    model = build_model(free_object=True)
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    object_id = model.body("known_object").id
    object_free_qpos = int(model.jnt_qposadr[model.joint("known_object_free").id])
    arm_qpos, arm_dof = joint_addresses(model, ARM_JOINTS)
    finger_qpos, _ = joint_addresses(model, FINGER_JOINTS)
    arm_actuators = actuator_ids(model, (
        "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"
    ))
    finger_actuators = actuator_ids(
        model, ("left_finger_position", "right_finger_position")
    )
    left_pair = tuple(sorted(("known_object_collision", "left_finger_collision")))
    right_pair = tuple(sorted(("known_object_collision", "right_finger_collision")))

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    # The original UR5e home keyframe predates the appended freejoint, so set
    # the free body's world pose explicitly: xyz followed by unit quaternion.
    data.qpos[object_free_qpos:object_free_qpos + 7] = [
        -0.45, 0.20, 0.03, 1.0, 0.0, 0.0, 0.0
    ]
    data.qpos[finger_qpos] = 0.03
    data.ctrl[finger_actuators] = 0.03
    mujoco.mj_forward(model, data)

    grasp_position = np.array([-0.45, 0.20, 0.065])
    grasp_rotation = np.diag([1.0, -1.0, -1.0])
    q_grasp, grasp_updates, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, data.qpos[arm_qpos].copy(),
        grasp_position, grasp_rotation,
    )

    # Begin dynamics aligned and open, then close and settle under gravity.
    data.qpos[arm_qpos] = q_grasp
    data.qpos[finger_qpos] = 0.03
    data.qvel[:] = 0.0
    data.ctrl[arm_actuators] = q_grasp
    data.ctrl[finger_actuators] = args.close_target
    mujoco.mj_forward(model, data)
    for _ in range(args.settle_steps):
        mujoco.mj_step(model, data)

    held, settled_pairs = bilateral_contact(
        model, data, left_pair, right_pair
    )
    if not held:
        raise RuntimeError("object did not settle with bilateral finger contact")

    gripper_position_initial, _ = site_pose(data, site_id)
    object_position_initial = data.xpos[object_id].copy()
    relative_initial = object_position_initial - gripper_position_initial
    settled_qpos = data.qpos.copy()
    settled_qvel = data.qvel.copy()

    # IK supplies the arm endpoint; execution below uses actuator targets.
    # The Menagerie arm uses finite-stiffness position servos. Add explicit
    # endpoint headroom while keeping the object-lift and slip criteria unchanged.
    tracking_headroom = 0.01
    geometric_lift_target = args.lift_height + tracking_headroom
    lift_target_position = gripper_position_initial + np.array(
        [0.0, 0.0, geometric_lift_target]
    )
    q_lift, lift_updates, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_grasp,
        lift_target_position, grasp_rotation,
    )
    timestep = float(model.opt.timestep)
    _, q_reference, _ = cubic_reference(
        q_grasp, q_lift, args.lift_duration, timestep
    )

    # Restore the settled grasp before executing the lift reference.
    data.qpos[:] = settled_qpos
    data.qvel[:] = settled_qvel
    data.ctrl[arm_actuators] = q_grasp
    data.ctrl[finger_actuators] = args.close_target
    mujoco.mj_forward(model, data)

    bilateral_steps = 0
    lift_contact_pairs = set()
    for q_command in q_reference[1:]:
        data.ctrl[arm_actuators] = q_command
        data.ctrl[finger_actuators] = args.close_target
        mujoco.mj_step(model, data)
        bilateral, pairs = bilateral_contact(model, data, left_pair, right_pair)
        bilateral_steps += int(bilateral)
        lift_contact_pairs.update(pairs)

    for _ in range(args.settle_steps):
        data.ctrl[arm_actuators] = q_lift
        data.ctrl[finger_actuators] = args.close_target
        mujoco.mj_step(model, data)

    final_bilateral, final_pairs = bilateral_contact(
        model, data, left_pair, right_pair
    )
    gripper_position_final, _ = site_pose(data, site_id)
    object_position_final = data.xpos[object_id].copy()
    relative_final = object_position_final - gripper_position_final

    gripper_lift = gripper_position_final[2] - gripper_position_initial[2]
    object_lift = object_position_final[2] - object_position_initial[2]
    relative_change = relative_final - relative_initial
    bilateral_fraction = bilateral_steps / (q_reference.shape[0] - 1)
    minimum_object_lift = args.lift_height - 0.01

    print(f"grasp IK updates={grasp_updates}; lift IK updates={lift_updates}")
    print(f"settled contact pairs={sorted(settled_pairs)}")
    print(f"initial gripper position={gripper_position_initial} m")
    print(f"initial object position={object_position_initial} m")
    print(f"lift command height={args.lift_height:.6f} m duration={args.lift_duration:.3f} s")
    print(f"geometric EE target includes headroom={tracking_headroom:.6f} m; "
          f"total={geometric_lift_target:.6f} m")
    print(f"actual gripper world-z lift={gripper_lift:.9f} m")
    print(f"object world-z lift={object_lift:.9f} m")
    print(f"object-to-gripper relative change={relative_change} m")
    print(f"bilateral contact during lift={bilateral_steps}/{q_reference.shape[0] - 1} "
          f"({bilateral_fraction:.3f})")
    print(f"lift contact pairs={sorted(lift_contact_pairs)}")
    print(f"final contact pairs={sorted(final_pairs)}")

    assert object_lift >= minimum_object_lift
    assert abs(relative_change[2]) <= 0.01
    assert bilateral_fraction >= 0.95
    assert final_bilateral
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    print("PASS: world lift, relative slip, and bilateral contact retention verified")
    print("This does not test transfer, placement, release, or robustness")


if __name__ == "__main__":
    main()
