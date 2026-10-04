"""Transfer a held free box, descend to support, and do not release it."""

import argparse

import mujoco
import numpy as np

from approach_and_close import actuator_ids
from lift_object import bilateral_contact
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
    parser.add_argument("--transfer-duration", type=float, default=3.0)
    parser.add_argument("--descent-duration", type=float, default=2.0)
    parser.add_argument("--max-joint-speed", type=float, default=0.5)
    args = parser.parse_args()
    for name in ("transfer_duration", "descent_duration", "max_joint_speed"):
        value = getattr(args, name)
        if not np.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    return args


def run_reference(model, data, q_reference, arm_actuators, arm_dof,
                  finger_actuators, close_target, left_pair, right_pair,
                  object_id, site_id):
    """Track a joint reference and return measured safety evidence."""
    bilateral_steps = 0
    support_steps = 0
    max_actual_speed = 0.0
    minimum_relative_z_change = 0.0
    object_initial = data.xpos[object_id].copy()
    gripper_initial, _ = site_pose(data, site_id)
    relative_z_initial = object_initial[2] - gripper_initial[2]
    ground_pair = tuple(sorted(("ground", "known_object_collision")))

    for q_command in q_reference[1:]:
        data.ctrl[arm_actuators] = q_command
        data.ctrl[finger_actuators] = close_target
        mujoco.mj_step(model, data)
        bilateral, pairs = bilateral_contact(model, data, left_pair, right_pair)
        bilateral_steps += int(bilateral)
        support_steps += int(ground_pair in pairs)
        max_actual_speed = max(
            max_actual_speed, float(np.max(np.abs(data.qvel[arm_dof])))
        )
        gripper_position, _ = site_pose(data, site_id)
        relative_z = data.xpos[object_id, 2] - gripper_position[2]
        minimum_relative_z_change = min(
            minimum_relative_z_change, relative_z - relative_z_initial
        )

    steps = q_reference.shape[0] - 1
    return {
        "steps": steps,
        "bilateral_fraction": bilateral_steps / steps,
        "support_steps": support_steps,
        "max_actual_speed": max_actual_speed,
        "minimum_relative_z_change": minimum_relative_z_change,
    }


def run_to_support(args, verbose=True):
    """Execute S11.6a and return its supported, still-closed state."""
    np.set_printoptions(precision=6, suppress=True)
    object_xy = np.asarray(getattr(args, "object_xy", [-0.45, 0.20]), dtype=float)
    object_mass = float(getattr(args, "object_mass", 0.05))
    object_friction = float(getattr(args, "object_friction", 2.0))
    model = build_model(
        free_object=True,
        object_position=[object_xy[0], object_xy[1], 0.03],
        object_mass=object_mass,
        object_friction=object_friction,
    )
    data = mujoco.MjData(model)
    timestep = float(model.opt.timestep)
    site_id = model.site("attachment_site").id
    object_id = model.body("known_object").id
    object_qpos = int(model.jnt_qposadr[model.joint("known_object_free").id])
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
    ground_pair = tuple(sorted(("ground", "known_object_collision")))
    close_target = 0.005
    grasp_rotation = np.diag([1.0, -1.0, -1.0])

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[object_qpos:object_qpos + 7] = [
        object_xy[0], object_xy[1], 0.03, 1, 0, 0, 0
    ]
    data.qpos[finger_qpos] = 0.03
    data.ctrl[finger_actuators] = 0.03
    mujoco.mj_forward(model, data)

    q_grasp, _, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, data.qpos[arm_qpos].copy(),
        np.array([object_xy[0], object_xy[1], 0.065]), grasp_rotation,
    )
    data.qpos[arm_qpos] = q_grasp
    data.qpos[finger_qpos] = 0.03
    data.qvel[:] = 0.0
    data.ctrl[arm_actuators] = q_grasp
    data.ctrl[finger_actuators] = close_target
    mujoco.mj_forward(model, data)
    for _ in range(500):
        mujoco.mj_step(model, data)

    held, _ = bilateral_contact(model, data, left_pair, right_pair)
    if not held:
        raise RuntimeError("failed to establish the initial bilateral grasp")

    # Lift with the same explicit 10 mm tracking headroom as S11.5.
    gripper_before_lift, _ = site_pose(data, site_id)
    q_lift, _, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_grasp,
        gripper_before_lift + np.array([0.0, 0.0, 0.06]), grasp_rotation,
    )
    _, lift_reference, _ = cubic_reference(q_grasp, q_lift, 1.5, timestep)
    # Restore the settled grasp after geometric IK changed arm qpos.
    data.qpos[arm_qpos] = q_grasp
    data.qvel[:] = 0.0
    data.ctrl[arm_actuators] = q_grasp
    mujoco.mj_forward(model, data)
    run_reference(
        model, data, lift_reference, arm_actuators, arm_dof,
        finger_actuators, close_target, left_pair, right_pair, object_id, site_id,
    )
    for _ in range(500):
        data.ctrl[arm_actuators] = q_lift
        data.ctrl[finger_actuators] = close_target
        mujoco.mj_step(model, data)

    held_position = data.xpos[object_id].copy()
    held_gripper, _ = site_pose(data, site_id)
    held_relative = held_position - held_gripper
    q_held = data.qpos[arm_qpos].copy()

    # Horizontal transfer: choose the gripper target from the desired object xy
    # and the measured held relative offset.
    desired_object_position = np.array([-0.30, -0.10, 0.03])
    transfer_gripper_target = held_gripper.copy()
    transfer_gripper_target[:2] = desired_object_position[:2] - held_relative[:2]
    q_transfer, _, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_held,
        transfer_gripper_target, grasp_rotation,
    )
    _, transfer_reference, transfer_qdot = cubic_reference(
        q_held, q_transfer, args.transfer_duration, timestep
    )
    transfer_peak_command = float(np.max(np.abs(transfer_qdot)))

    # Restore the actual held state after geometric IK.
    data.qpos[arm_qpos] = q_held
    data.qvel[:] = 0.0
    data.ctrl[arm_actuators] = q_held
    mujoco.mj_forward(model, data)
    transfer_metrics = run_reference(
        model, data, transfer_reference, arm_actuators, arm_dof,
        finger_actuators, close_target, left_pair, right_pair, object_id, site_id,
    )
    for _ in range(250):
        data.ctrl[arm_actuators] = q_transfer
        data.ctrl[finger_actuators] = close_target
        mujoco.mj_step(model, data)

    transfer_object = data.xpos[object_id].copy()
    transfer_gripper, _ = site_pose(data, site_id)
    transfer_relative = transfer_object - transfer_gripper
    q_transfer_actual = data.qpos[arm_qpos].copy()

    # Descend so the measured held relative offset predicts object center z=0.03 m.
    descent_gripper_target = transfer_gripper.copy()
    descent_gripper_target[:2] = desired_object_position[:2] - transfer_relative[:2]
    descent_gripper_target[2] = desired_object_position[2] - transfer_relative[2]
    q_descent, _, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_transfer_actual,
        descent_gripper_target, grasp_rotation,
    )
    _, descent_reference, descent_qdot = cubic_reference(
        q_transfer_actual, q_descent, args.descent_duration, timestep
    )
    descent_peak_command = float(np.max(np.abs(descent_qdot)))

    data.qpos[arm_qpos] = q_transfer_actual
    data.qvel[:] = 0.0
    data.ctrl[arm_actuators] = q_transfer_actual
    mujoco.mj_forward(model, data)
    descent_metrics = run_reference(
        model, data, descent_reference, arm_actuators, arm_dof,
        finger_actuators, close_target, left_pair, right_pair, object_id, site_id,
    )
    for _ in range(500):
        data.ctrl[arm_actuators] = q_descent
        data.ctrl[finger_actuators] = close_target
        mujoco.mj_step(model, data)

    final_object = data.xpos[object_id].copy()
    final_error = final_object - desired_object_position
    final_bilateral, final_pairs = bilateral_contact(
        model, data, left_pair, right_pair
    )
    final_supported = ground_pair in final_pairs
    max_command_speed = max(transfer_peak_command, descent_peak_command)
    max_actual_speed = max(
        transfer_metrics["max_actual_speed"], descent_metrics["max_actual_speed"]
    )
    minimum_relative_z_change = min(
        transfer_metrics["minimum_relative_z_change"],
        descent_metrics["minimum_relative_z_change"],
    )

    if verbose:
        print(f"desired supported object position={desired_object_position} m")
        print(f"held object position={held_position} m")
        print(f"after horizontal transfer={transfer_object} m")
        print(f"final object position={final_object} m")
        print(f"final pose error={final_error} m; norm={np.linalg.norm(final_error):.9f} m")
        print(f"transfer duration={args.transfer_duration:.3f} s "
              f"peak command={transfer_peak_command:.6f} rad/s")
        print(f"descent duration={args.descent_duration:.3f} s "
              f"peak command={descent_peak_command:.6f} rad/s")
        print(f"max actual arm speed={max_actual_speed:.6f} rad/s")
        print(f"minimum object-to-gripper relative-z change={minimum_relative_z_change:.9f} m")
        print(f"transfer bilateral fraction={transfer_metrics['bilateral_fraction']:.3f}")
        print(f"descent bilateral fraction={descent_metrics['bilateral_fraction']:.3f}")
        print(f"support detected during descent steps={descent_metrics['support_steps']}")
        print(f"final supported={final_supported} final bilateral={final_bilateral}")
        print(f"final contact pairs={sorted(final_pairs)}")

    assert max_command_speed <= args.max_joint_speed + 1e-12
    assert max_actual_speed <= args.max_joint_speed + 0.05
    assert minimum_relative_z_change >= -0.015
    assert transfer_metrics["bilateral_fraction"] >= 0.95
    assert descent_metrics["bilateral_fraction"] >= 0.95
    assert descent_metrics["support_steps"] > 0
    assert final_supported and final_bilateral
    assert np.linalg.norm(final_error) <= 0.015
    if verbose:
        print("PASS: speed-limited transfer, no-drop evidence, pose, and support verified")
        print("The fingers remain closed; no release or retreat was executed")
    return {
        "model": model,
        "data": data,
        "site_id": site_id,
        "object_id": object_id,
        "arm_qpos": arm_qpos,
        "arm_dof": arm_dof,
        "finger_qpos": finger_qpos,
        "arm_actuators": arm_actuators,
        "finger_actuators": finger_actuators,
        "left_pair": left_pair,
        "right_pair": right_pair,
        "ground_pair": ground_pair,
        "grasp_rotation": grasp_rotation,
        "desired_object_position": desired_object_position,
        "q_supported": data.qpos[arm_qpos].copy(),
    }


def main():
    args = parse_args()
    run_to_support(args)


if __name__ == "__main__":
    main()
