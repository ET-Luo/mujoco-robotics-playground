"""Release a supported box, retreat, and verify final placement evidence."""

import argparse
from types import SimpleNamespace

import mujoco
import numpy as np

from approach_and_close import opening_width
from pregrasp_motion import (
    contact_names,
    cubic_reference,
    site_pose,
    solve_pregrasp_ik,
)
from transfer_and_descend import run_to_support


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--retreat-height", type=float, default=0.08)
    parser.add_argument("--retreat-duration", type=float, default=2.0)
    parser.add_argument("--open-target", type=float, default=0.03)
    args = parser.parse_args()
    if not np.isfinite(args.retreat_height) or args.retreat_height <= 0.0:
        parser.error("--retreat-height must be positive and finite")
    if not np.isfinite(args.retreat_duration) or args.retreat_duration <= 0.0:
        parser.error("--retreat-duration must be positive and finite")
    if not 0.0 <= args.open_target <= 0.04:
        parser.error("--open-target must be in [0, 0.04] m")
    return args


def run_release_and_retreat(context, args, verbose=True):
    """Release one supported object and return final placement evidence."""
    np.set_printoptions(precision=6, suppress=True)
    model = context["model"]
    data = context["data"]
    site_id = context["site_id"]
    object_id = context["object_id"]
    arm_qpos = context["arm_qpos"]
    arm_dof = context["arm_dof"]
    finger_qpos = context["finger_qpos"]
    arm_actuators = context["arm_actuators"]
    finger_actuators = context["finger_actuators"]
    left_pair = context["left_pair"]
    right_pair = context["right_pair"]
    ground_pair = context["ground_pair"]
    grasp_rotation = context["grasp_rotation"]
    desired_object_position = context["desired_object_position"]

    supported_pairs = contact_names(model, data)
    assert ground_pair in supported_pairs
    assert left_pair in supported_pairs and right_pair in supported_pairs
    if verbose:
        print("\nRelease precondition: support confirmed before opening")

    # Open while holding the arm at the supported pose. Do not retreat until
    # both named finger-object contacts have disappeared.
    data.ctrl[finger_actuators] = args.open_target
    release_step = None
    for step in range(1, 1001):
        data.ctrl[arm_actuators] = data.qpos[arm_qpos]
        mujoco.mj_step(model, data)
        pairs = contact_names(model, data)
        finger_contacts_gone = left_pair not in pairs and right_pair not in pairs
        if ground_pair in pairs and finger_contacts_gone:
            release_step = step
            break
    if release_step is None:
        raise RuntimeError("supported release was not observed within 1000 steps")

    release_object_position = data.xpos[object_id].copy()
    release_gripper_position, _ = site_pose(data, site_id)
    release_opening = opening_width(data, finger_qpos)
    q_release = data.qpos[arm_qpos].copy()
    if verbose:
        print(f"release step={release_step} time={data.time:.6f} s")
        print(f"opening at contact separation={release_opening:.6f} m")
        print(f"object position at release={release_object_position} m")

    # For this top-down grasp, retreat opposite local +z_G: world +z.
    retreat_direction_world = -grasp_rotation[:, 2]
    retreat_target = release_gripper_position + args.retreat_height * retreat_direction_world
    q_retreat, _, _, _ = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_release,
        retreat_target, grasp_rotation,
    )
    timestep = float(model.opt.timestep)
    _, retreat_reference, _ = cubic_reference(
        q_release, q_retreat, args.retreat_duration, timestep
    )

    # Restore the real release state after geometric IK changed arm qpos.
    data.qpos[arm_qpos] = q_release
    data.qvel[arm_dof] = 0.0
    data.ctrl[arm_actuators] = q_release
    data.ctrl[finger_actuators] = args.open_target
    mujoco.mj_forward(model, data)

    finger_contact_steps = 0
    unsupported_steps = 0
    for q_command in retreat_reference[1:]:
        data.ctrl[arm_actuators] = q_command
        data.ctrl[finger_actuators] = args.open_target
        mujoco.mj_step(model, data)
        pairs = contact_names(model, data)
        finger_contact_steps += int(left_pair in pairs or right_pair in pairs)
        unsupported_steps += int(ground_pair not in pairs)

    for _ in range(500):
        data.ctrl[arm_actuators] = q_retreat
        data.ctrl[finger_actuators] = args.open_target
        mujoco.mj_step(model, data)

    final_pairs = contact_names(model, data)
    final_object_position = data.xpos[object_id].copy()
    final_gripper_position, _ = site_pose(data, site_id)
    final_error = final_object_position - desired_object_position
    retreat_achieved = float(np.dot(
        final_gripper_position - release_gripper_position,
        retreat_direction_world,
    ))
    object_dof = int(model.jnt_dofadr[model.joint("known_object_free").id])
    object_linear_speed = float(np.linalg.norm(data.qvel[object_dof:object_dof + 3]))
    object_angular_speed = float(np.linalg.norm(data.qvel[object_dof + 3:object_dof + 6]))
    final_opening = opening_width(data, finger_qpos)
    final_supported = ground_pair in final_pairs
    final_finger_contact = left_pair in final_pairs or right_pair in final_pairs

    if verbose:
        print("\nFinal placement evidence:")
        print(f"retreat requested={args.retreat_height:.6f} m achieved={retreat_achieved:.6f} m")
        print(f"retreat finger-contact steps={finger_contact_steps} "
              f"unsupported steps={unsupported_steps}")
        print(f"final object position={final_object_position} m")
        print(f"final pose error={final_error} m; norm={np.linalg.norm(final_error):.9f} m")
        print(f"final opening={final_opening:.6f} m")
        print(f"final object linear speed={object_linear_speed:.9f} m/s")
        print(f"final object angular speed={object_angular_speed:.9f} rad/s")
        print(f"final supported={final_supported} finger contact={final_finger_contact}")
        print(f"final contact pairs={sorted(final_pairs)}")

    assert retreat_achieved >= args.retreat_height - 0.01
    assert finger_contact_steps == 0
    assert unsupported_steps == 0
    assert np.linalg.norm(final_error) <= 0.015
    assert final_supported and not final_finger_contact
    assert final_opening >= 0.07
    assert object_linear_speed <= 0.01
    assert object_angular_speed <= 0.1
    if verbose:
        print("PASS: supported release, retreat, final pose, and low object speed verified")
    return {
        "final_position": final_object_position,
        "position_error": float(np.linalg.norm(final_error)),
        "linear_speed": object_linear_speed,
        "angular_speed": object_angular_speed,
        "retreat_achieved": retreat_achieved,
    }


def main():
    args = parse_args()
    # Reproduce S11.6a's default supported state and assert its preconditions.
    context = run_to_support(SimpleNamespace(
        transfer_duration=3.0,
        descent_duration=2.0,
        max_joint_speed=0.5,
    ))
    run_release_and_retreat(context, args)


if __name__ == "__main__":
    main()
