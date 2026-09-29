"""Read the UR5e attachment site's local position and world pose without stepping."""

import mujoco
import mujoco_menagerie
import numpy as np


def print_pose(label, model, data, site_id):
    print(f"\n{label}: time={data.time:.3f} s")
    print(f"Local site position in parent body (m): {model.site_pos[site_id]}")
    print(f"World site position (m): {data.site_xpos[site_id]}")
    # Nine row-major values become a 3x3 matrix. Columns are local axes in world coordinates.
    rotation = data.site_xmat[site_id].reshape(3, 3)
    print(f"R_WE (dimensionless):\n{rotation}")
    for axis, column in zip("xyz", rotation.T):
        print(f"Site +{axis} axis in world: {column}")


def main():
    np.set_printoptions(precision=6, suppress=True)
    # MjModel contains compiled model parameters; MjData holds state and derived poses.
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    body_id = int(model.site_bodyid[site_id])
    home_id = model.key("home").id
    # Reset in place to home, then refresh derived quantities without advancing time.
    mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)
    print(f"Site: attachment_site; parent body: {model.body(body_id).name}")
    print_pose("Home", model, data, site_id)
    local_before = model.site_pos[site_id].copy()
    position_before = data.site_xpos[site_id].copy()
    rotation_before = data.site_xmat[site_id].reshape(3, 3).copy()
    time_before = data.time

    joint_id = model.joint("shoulder_pan_joint").id
    qpos_index = int(model.jnt_qposadr[joint_id])
    # Assign a pose directly: this is not a target command or simulated motion.
    data.qpos[qpos_index] += 0.1
    mujoco.mj_forward(model, data)
    print_pose("After shoulder_pan qpos +0.1 rad and forward", model, data, site_id)
    rotation_after = data.site_xmat[site_id].reshape(3, 3)
    print(f"World position difference (m): {data.site_xpos[site_id] - position_before}")
    print(f"Local position unchanged: {np.array_equal(model.site_pos[site_id], local_before)}")
    print(f"World position changed: {not np.allclose(data.site_xpos[site_id], position_before)}")
    print(f"World orientation changed: {not np.allclose(rotation_after, rotation_before)}")
    print(f"Time unchanged: {data.time == time_before}")


if __name__ == "__main__":
    main()
