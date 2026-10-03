"""Apply one undamped UR5e 6D IK update for a small reachable pose target."""

import argparse

import mujoco
import mujoco_menagerie
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-scale", type=float, default=1.0,
                        help="scale applied to the small joint offset that generates the target pose")
    args = parser.parse_args()
    if not np.isfinite(args.target_scale) or not 0.1 <= args.target_scale <= 10.0:
        parser.error("--target-scale must be finite and in [0.1, 10]")
    return args


def orientation_error_world(current_rotation, target_rotation):
    """Return the current-to-target rotation vector in world axes, in radians."""
    relative_rotation = target_rotation @ current_rotation.T
    cosine_angle = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cosine_angle)
    if angle < 1e-12:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        raise ValueError("This S10.4 experiment excludes orientation errors near pi")
    skew_vector = np.array([
        relative_rotation[2, 1] - relative_rotation[1, 2],
        relative_rotation[0, 2] - relative_rotation[2, 0],
        relative_rotation[1, 0] - relative_rotation[0, 1],
    ])
    axis = skew_vector / (2.0 * np.sin(angle))
    return axis * angle


def site_pose(data, site_id):
    """Copy the site's world position and 3x3 world rotation matrix."""
    return (data.site_xpos[site_id].copy(),
            data.site_xmat[site_id].reshape(3, 3).copy())


def pose_error(current_position, current_rotation, target_position, target_rotation):
    """Stack world position error (m) and world orientation error (rad)."""
    position_error = target_position - current_position
    rotation_error = orientation_error_world(current_rotation, target_rotation)
    return np.concatenate((position_error, rotation_error))


def full_site_jacobian(model, data, site_id):
    """Return the 6-by-nv world-frame geometric Jacobian."""
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return np.vstack((jacp, jacr))


def main():
    args = parse_args()
    np.set_printoptions(precision=9, suppress=True)
    xml_path = mujoco_menagerie.get("universal_robots_ur5e").xml("scene")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    q_initial = data.qpos.copy()

    # Generate one reproducible, reachable target pose. This offset is not used
    # by the IK solve; the solver below sees only target position/orientation.
    base_offset = np.array([0.002, -0.0015, 0.001, -0.001, 0.0015, -0.002])
    generating_offset = args.target_scale * base_offset
    data.qpos[:] = q_initial + generating_offset
    mujoco.mj_forward(model, data)
    target_position, target_rotation = site_pose(data, site_id)

    # Return to the current pose before constructing its error and Jacobian.
    data.qpos[:] = q_initial
    mujoco.mj_forward(model, data)
    current_position, current_rotation = site_pose(data, site_id)
    error_before = pose_error(
        current_position, current_rotation, target_position, target_rotation
    )
    jacobian = full_site_jacobian(model, data, site_id)
    condition_number = np.linalg.cond(jacobian)

    # One undamped Newton/Jacobian update: no loop, step limit, or DLS yet.
    delta_q = np.linalg.solve(jacobian, error_before)
    q_candidate = q_initial + delta_q
    joint_limited = model.jnt_limited.astype(bool)
    within_limits = np.all(
        (~joint_limited)
        | ((q_candidate >= model.jnt_range[:, 0]) & (q_candidate <= model.jnt_range[:, 1]))
    )
    if not within_limits:
        raise RuntimeError("One-step IK candidate violates a joint position limit")

    data.qpos[:] = q_candidate
    mujoco.mj_forward(model, data)
    updated_position, updated_rotation = site_pose(data, site_id)
    error_after = pose_error(
        updated_position, updated_rotation, target_position, target_rotation
    )

    position_before = np.linalg.norm(error_before[:3])
    orientation_before = np.linalg.norm(error_before[3:])
    position_after = np.linalg.norm(error_after[:3])
    orientation_after = np.linalg.norm(error_after[3:])

    print(f"Target scale={args.target_scale:g}")
    print(f"Jacobian shape={jacobian.shape}; condition number={condition_number:.6f}")
    print(f"Target position={target_position} m")
    print(f"Position error before={error_before[:3]} m; norm={position_before:.12f} m")
    print(f"Orientation error before={error_before[3:]} rad; norm={orientation_before:.12f} rad")
    print(f"One-step delta_q={delta_q} rad")
    print(f"Target-generating offset (check only)={generating_offset} rad")
    print(f"Position error after={error_after[:3]} m; norm={position_after:.12f} m")
    print(f"Orientation error after={error_after[3:]} rad; norm={orientation_after:.12f} rad")
    print(f"Joint limits satisfied={within_limits}; simulation time={data.time:.3f} s")

    assert jacobian.shape == (6, 6)
    assert np.isfinite(delta_q).all()
    assert position_after < position_before
    assert orientation_after < orientation_before
    assert data.time == 0.0
    print("PASS: one 6D IK update reduced both position and orientation error")


if __name__ == "__main__":
    main()
