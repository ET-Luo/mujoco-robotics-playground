"""Define known object, grasp, and pre-grasp poses without solving IK."""

import argparse

import numpy as np


def make_transform(rotation, position):
    """Return a 4x4 homogeneous transform from R_AB and p_AB."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def validate_rotation(name, rotation):
    """Check that a 3x3 matrix is a proper rotation."""
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0, atol=1e-12)
    print(f"{name}: orthonormal=True, det={np.linalg.det(rotation):.1f}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pregrasp-distance",
        type=float,
        default=0.10,
        help="retreat from grasp along negative gripper +z, in meters",
    )
    args = parser.parse_args()
    if args.pregrasp_distance <= 0.0:
        parser.error("--pregrasp-distance must be positive")
    return args


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)

    # T_WO: known box pose. The 0.06 m-tall box rests on the world z=0 plane.
    rotation_world_object = np.eye(3)
    position_world_object = np.array([-0.45, 0.20, 0.03])
    transform_world_object = make_transform(
        rotation_world_object, position_world_object
    )

    # T_OG: desired gripper-root pose expressed in the object frame.
    # +x_G remains parallel to +x_O (finger opening axis), while +z_G points
    # toward -z_O for a top-down approach. The 0.035 m offset aligns the
    # finger geometry centers with the object center.
    rotation_object_gripper = np.diag([1.0, -1.0, -1.0])
    position_object_gripper = np.array([0.0, 0.0, 0.035])
    transform_object_gripper = make_transform(
        rotation_object_gripper, position_object_gripper
    )
    transform_world_grasp = transform_world_object @ transform_object_gripper

    rotation_world_grasp = transform_world_grasp[:3, :3]
    position_world_grasp = transform_world_grasp[:3, 3]
    approach_axis_world = rotation_world_grasp[:, 2]  # local +z_G in world

    # Retreat opposite the approach direction. Moving from pre-grasp to grasp
    # is therefore +distance along the end-effector's local +z_G axis.
    position_world_pregrasp = (
        position_world_grasp - args.pregrasp_distance * approach_axis_world
    )
    transform_world_pregrasp = make_transform(
        rotation_world_grasp, position_world_pregrasp
    )

    # The Menagerie UR5e base origin is at the world origin, but its axes are
    # rotated pi around world z. Same origin does not mean same frame.
    rotation_world_base = np.diag([-1.0, -1.0, 1.0])
    transform_world_base = make_transform(rotation_world_base, np.zeros(3))
    transform_base_world = np.linalg.inv(transform_world_base)
    transform_base_object = transform_base_world @ transform_world_object
    transform_base_grasp = transform_base_world @ transform_world_grasp

    validate_rotation("R_WO", rotation_world_object)
    validate_rotation("R_OG", rotation_object_gripper)
    validate_rotation("R_WG", rotation_world_grasp)
    validate_rotation("R_WB", rotation_world_base)

    delta_world = position_world_grasp - position_world_pregrasp
    delta_gripper = rotation_world_grasp.T @ delta_world
    finger_center_world = position_world_grasp + rotation_world_grasp @ np.array(
        [0.0, 0.0, 0.035]
    )

    print("\nWorld-frame poses:")
    print(f"  object p_WO={position_world_object} m")
    print(f"  grasp p_WG={position_world_grasp} m")
    print(f"  pre-grasp p_WP={position_world_pregrasp} m")
    print(f"  gripper +z_G expressed in world={approach_axis_world}")
    print(f"  pre-grasp -> grasp delta in world={delta_world} m")
    print(f"  same delta expressed in gripper={delta_gripper} m")

    print("\nBase-frame representations of the same physical poses:")
    print(f"  object p_BO={transform_base_object[:3, 3]} m")
    print(f"  grasp p_BG={transform_base_grasp[:3, 3]} m")

    np.testing.assert_allclose(approach_axis_world, [0.0, 0.0, -1.0], atol=1e-12)
    np.testing.assert_allclose(
        delta_gripper, [0.0, 0.0, args.pregrasp_distance], atol=1e-12
    )
    np.testing.assert_allclose(finger_center_world, position_world_object, atol=1e-12)
    np.testing.assert_allclose(
        transform_world_pregrasp[:3, :3], rotation_world_grasp, atol=1e-12
    )
    print("PASS: grasp alignment and local-axis pre-grasp offset verified")
    print("No IK, actuator command, dynamics step, contact, or grasp was executed")


if __name__ == "__main__":
    main()
