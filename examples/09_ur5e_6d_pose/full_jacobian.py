"""Check one UR5e full-Jacobian column using small pose differences."""

import argparse

import mujoco
import mujoco_menagerie
import numpy as np

from orientation_error import orientation_error_world


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=JOINT_NAMES, default="wrist_3_joint",
                        help="joint whose full-Jacobian column is checked")
    parser.add_argument("--epsilon", type=float, default=1e-6,
                        help="positive finite-difference angle in radians")
    args = parser.parse_args()
    if not np.isfinite(args.epsilon) or not 1e-8 < args.epsilon < np.pi - 1e-6:
        parser.error("--epsilon must be finite and between 1e-8 and pi-1e-6 rad")
    return args


def main():
    args = parse_args()
    np.set_printoptions(precision=12, suppress=True)
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    site_id = model.site("attachment_site").id
    joint_id = model.joint(args.joint).id
    qpos_index = int(model.jnt_qposadr[joint_id])
    dof_index = int(model.jnt_dofadr[joint_id])

    # mj_jacSite fills both arrays in place and returns None. Both are expressed
    # in world axes; columns correspond to velocity DOFs, not actuator IDs.
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    result = mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    assert result is None
    full_jacobian = np.vstack((jacp, jacr))

    print(f"Jp shape={jacp.shape}; hinge-column units=m/rad")
    print(f"Jr shape={jacr.shape}; hinge-column values are dimensionless (rad/rad)")
    print(f"Full Jacobian shape={full_jacobian.shape}")
    for joint_index in range(model.njnt):
        column_index = int(model.jnt_dofadr[joint_index])
        print(f"  column {column_index}: {model.joint(joint_index).name}")

    column = full_jacobian[:, dof_index].copy()
    position_before = data.site_xpos[site_id].copy()
    rotation_before = data.site_xmat[site_id].reshape(3, 3).copy()
    time_before = float(data.time)
    epsilon = args.epsilon  # rad
    predicted_pose_change = column * epsilon

    # Directly select a nearby configuration, then refresh derived quantities.
    data.qpos[qpos_index] += epsilon
    mujoco.mj_forward(model, data)
    position_change = data.site_xpos[site_id] - position_before
    rotation_after = data.site_xmat[site_id].reshape(3, 3).copy()
    rotation_vector = orientation_error_world(rotation_before, rotation_after)
    measured_pose_change = np.concatenate((position_change, rotation_vector))

    print(f"\n{args.joint} column={column}")
    print(f"epsilon={epsilon:.1e} rad")
    print(f"Predicted [position; rotation] change={predicted_pose_change}")
    print(f"Measured  [position; rotation] change={measured_pose_change}")
    print(f"Absolute difference={np.abs(measured_pose_change - predicted_pose_change)}")
    print(f"Simulation time: {time_before:.3f} -> {data.time:.3f} s")

    if args.joint == "wrist_3_joint":
        np.testing.assert_allclose(column[:3], 0.0, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(column[3:], [0.0, 0.0, -1.0], rtol=0.0, atol=1e-5)
    # A first-order Jacobian prediction differs from nonlinear FK by O(epsilon^2).
    finite_difference_tolerance = max(1e-9, epsilon**2)
    np.testing.assert_allclose(measured_pose_change, predicted_pose_change,
                               rtol=0.0, atol=finite_difference_tolerance)
    assert data.time == time_before
    print(f"PASS: {args.joint} column matches within atol={finite_difference_tolerance:.3e}")


if __name__ == "__main__":
    main()
