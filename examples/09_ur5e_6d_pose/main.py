"""Compare two UR5e end-effector poses without commanding or stepping the robot."""

import mujoco
import mujoco_menagerie
import numpy as np


def print_world_pose(label, data, site_id):
    """Print one site's world position and orientation matrix."""
    raw_rotation = data.site_xmat[site_id]
    rotation = raw_rotation.reshape(3, 3)
    print(f"\n{label}: time={data.time:.3f} s")
    print(f"site_xpos shape={data.site_xpos[site_id].shape}, value={data.site_xpos[site_id]} m")
    print(f"site_xmat raw shape={raw_rotation.shape}; reshaped={rotation.shape}")
    print(f"R_WE (dimensionless):\n{rotation}")
    for axis_name, world_direction in zip("xyz", rotation.T):
        print(f"  site +{axis_name} axis in world={world_direction}")


def main():
    np.set_printoptions(precision=9, suppress=True)
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    site_id = model.site("attachment_site").id
    joint_id = model.joint("wrist_3_joint").id
    qpos_index = int(model.jnt_qposadr[joint_id])
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    # mj_forward updates derived poses in data in place and does not advance time.
    mujoco.mj_forward(model, data)

    # Both vectors use wrist_3_link's local coordinates. Their difference parallel
    # to the joint axis means the site origin lies on that rotation axis.
    joint_axis = model.jnt_axis[joint_id].copy()
    joint_to_site = model.site_pos[site_id] - model.jnt_pos[joint_id]
    axis_distance = np.linalg.norm(np.cross(joint_to_site, joint_axis))
    print(f"wrist_3 local axis={joint_axis}")
    print(f"joint origin to site origin (parent-body frame)={joint_to_site} m")
    print(f"site-origin distance from rotation axis={axis_distance:.12f} m")

    local_position_before = model.site_pos[site_id].copy()
    world_position_before = data.site_xpos[site_id].copy()
    world_rotation_before = data.site_xmat[site_id].reshape(3, 3).copy()
    time_before = float(data.time)
    print_world_pose("Home", data, site_id)

    # Direct state assignment selects another configuration; it is not ctrl or motion.
    data.qpos[qpos_index] += 0.1  # rad
    mujoco.mj_forward(model, data)
    print_world_pose("After wrist_3 qpos +0.1 rad and mj_forward", data, site_id)

    position_change = data.site_xpos[site_id] - world_position_before
    rotation_changed = not np.allclose(
        data.site_xmat[site_id].reshape(3, 3), world_rotation_before,
        rtol=0.0, atol=1e-12,
    )
    print(f"\nWorld position change={position_change} m")
    print(f"Local site position unchanged={np.array_equal(model.site_pos[site_id], local_position_before)}")
    print(f"World site position unchanged={np.allclose(position_change, 0.0, rtol=0.0, atol=1e-12)}")
    print(f"World site orientation changed={rotation_changed}")
    print(f"Simulation time unchanged={data.time == time_before}")

    np.testing.assert_allclose(axis_distance, 0.0, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(position_change, 0.0, rtol=0.0, atol=1e-12)
    assert np.array_equal(model.site_pos[site_id], local_position_before)
    assert rotation_changed
    assert data.time == time_before
    print("PASS: wrist rotation changed orientation while the on-axis site origin stayed fixed")


if __name__ == "__main__":
    main()

