"""Approach along gripper +z, then stop at the first bilateral box contact."""

import argparse

import mujoco
import numpy as np

from pregrasp_motion import (
    ARM_JOINTS,
    FINGER_JOINTS,
    build_model,
    contact_names,
    joint_addresses,
    site_pose,
    solve_pregrasp_ik,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--approach-distance", type=float, default=0.10)
    parser.add_argument("--approach-samples", type=int, default=11)
    parser.add_argument("--close-target", type=float, default=0.005)
    parser.add_argument("--max-close-steps", type=int, default=1000)
    args = parser.parse_args()
    if not np.isfinite(args.approach_distance) or args.approach_distance <= 0.0:
        parser.error("--approach-distance must be positive and finite")
    if args.approach_samples < 2:
        parser.error("--approach-samples must be at least 2")
    if not 0.0 <= args.close_target <= 0.04:
        parser.error("--close-target must be in [0, 0.04] m")
    if args.max_close_steps <= 0:
        parser.error("--max-close-steps must be positive")
    return args


def opening_width(data, finger_qpos):
    return 0.02 + float(np.sum(data.qpos[finger_qpos]))


def actuator_ids(model, names):
    return np.asarray([model.actuator(name).id for name in names])


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)
    model = build_model()
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    arm_qpos, arm_dof = joint_addresses(model, ARM_JOINTS)
    finger_qpos, _ = joint_addresses(model, FINGER_JOINTS)
    arm_actuators = actuator_ids(model, (
        "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"
    ))
    finger_actuators = actuator_ids(
        model, ("left_finger_position", "right_finger_position")
    )

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[finger_qpos] = 0.03  # Keep 0.08 m opening during approach.
    data.ctrl[finger_actuators] = 0.03
    mujoco.mj_forward(model, data)
    q_seed = data.qpos[arm_qpos].copy()

    grasp_position = np.array([-0.45, 0.20, 0.065])
    grasp_rotation = np.diag([1.0, -1.0, -1.0])
    approach_axis_world = grasp_rotation[:, 2]  # local +z_G in world
    pregrasp_position = (
        grasp_position - args.approach_distance * approach_axis_world
    )

    achieved_positions = []
    approach_contacts = set()
    total_ik_updates = 0
    for alpha in np.linspace(0.0, 1.0, args.approach_samples):
        target_position = (
            pregrasp_position
            + alpha * args.approach_distance * approach_axis_world
        )
        q_seed, updates, _, _ = solve_pregrasp_ik(
            model,
            data,
            site_id,
            arm_qpos,
            arm_dof,
            q_seed,
            target_position,
            grasp_rotation,
        )
        total_ik_updates += updates
        data.qpos[finger_qpos] = 0.03
        mujoco.mj_forward(model, data)
        position, _ = site_pose(data, site_id)
        achieved_positions.append(position)
        approach_contacts.update(contact_names(model, data))

    achieved_positions = np.asarray(achieved_positions)
    displacement_world = achieved_positions[-1] - achieved_positions[0]
    displacement_gripper = grasp_rotation.T @ displacement_world
    expected_world = args.approach_distance * approach_axis_world
    max_line_deviation = 0.0
    for alpha, position in zip(
        np.linspace(0.0, 1.0, args.approach_samples), achieved_positions
    ):
        expected = achieved_positions[0] + alpha * displacement_world
        max_line_deviation = max(max_line_deviation, float(np.linalg.norm(position - expected)))

    print("Approach geometry:")
    print(f"  gripper +z_G in world={approach_axis_world}")
    print(f"  expected displacement in world={expected_world} m")
    print(f"  achieved displacement in world={displacement_world} m")
    print(f"  achieved displacement in gripper={displacement_gripper} m")
    print(f"  samples={args.approach_samples} total IK updates={total_ik_updates}")
    print(f"  max sampled line deviation={max_line_deviation:.9f} m")
    print(f"  contact pairs during open approach={sorted(approach_contacts)}")

    # Start dynamics exactly at the final geometric configuration. The arm
    # position servos hold the grasp pose while the two finger servos close.
    data.ctrl[arm_actuators] = data.qpos[arm_qpos]
    data.ctrl[finger_actuators] = 0.03
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    data.ctrl[finger_actuators] = args.close_target

    left_pair = tuple(sorted(("known_object_collision", "left_finger_collision")))
    right_pair = tuple(sorted(("known_object_collision", "right_finger_collision")))
    bilateral_step = None
    bilateral_pairs = set()
    for step in range(1, args.max_close_steps + 1):
        mujoco.mj_step(model, data)
        pairs = contact_names(model, data)
        if left_pair in pairs and right_pair in pairs:
            bilateral_step = step
            bilateral_pairs = pairs
            break

    if bilateral_step is None:
        raise RuntimeError("bilateral finger-object contact was not observed")

    final_finger_qpos = data.qpos[finger_qpos].copy()
    final_opening = opening_width(data, finger_qpos)
    print("\nClosing contact:")
    print(f"  close target per finger={args.close_target:.6f} m")
    print(f"  bilateral contact step={bilateral_step} time={data.time:.6f} s")
    print(f"  finger qpos={final_finger_qpos} m opening={final_opening:.6f} m")
    print(f"  contact pairs={sorted(bilateral_pairs)}")

    np.testing.assert_allclose(displacement_world, expected_world, atol=2e-4)
    np.testing.assert_allclose(
        displacement_gripper,
        [0.0, 0.0, args.approach_distance],
        atol=2e-4,
    )
    assert max_line_deviation < 2e-4
    assert not approach_contacts
    assert left_pair in bilateral_pairs and right_pair in bilateral_pairs
    assert bilateral_pairs == {left_pair, right_pair}
    print("PASS: local-axis approach and first bilateral contact verified")
    print("This does not test grasp stability, lifting, or object retention")


if __name__ == "__main__":
    main()
