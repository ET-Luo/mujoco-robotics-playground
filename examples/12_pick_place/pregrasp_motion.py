"""Solve UR5e IK to pre-grasp and validate a collision-free joint reference."""

import argparse
from pathlib import Path

import mujoco
import mujoco_menagerie
import numpy as np


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
FINGER_JOINTS = ("left_slide", "right_slide")
POSITION_TOLERANCE = 1e-4  # m
ORIENTATION_TOLERANCE = 1e-3  # rad


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--max-joint-speed", type=float, default=0.5)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be positive and finite")
    if not np.isfinite(args.dt) or args.dt <= 0.0 or args.dt > args.duration:
        parser.error("--dt must be positive, finite, and no larger than duration")
    if not np.isfinite(args.max_joint_speed) or args.max_joint_speed <= 0.0:
        parser.error("--max-joint-speed must be positive and finite")
    return args


def build_model(free_object=False, object_position=(-0.45, 0.20, 0.03),
                object_mass=0.05, object_friction=2.0):
    """Compile UR5e, the attached gripper, and a known-pose box."""
    robot_path = mujoco_menagerie.get("universal_robots_ur5e").xml("ur5e")
    robot_spec = mujoco.MjSpec.from_file(str(robot_path))
    gripper_spec = mujoco.MjSpec.from_file(
        str(Path(__file__).with_name("gripper.xml"))
    )
    robot_spec.attach(
        gripper_spec, prefix="", site=robot_spec.site("attachment_site")
    )
    # These are adjacent, mechanically connected tool parts. Their coarse
    # collision shapes overlap near the flange and should not be treated as
    # environment contacts when the fingers close.
    robot_spec.add_exclude(
        name="wrist_left_finger", bodyname1="wrist_3_link", bodyname2="left_finger"
    )
    robot_spec.add_exclude(
        name="wrist_right_finger", bodyname1="wrist_3_link", bodyname2="right_finger"
    )
    robot_spec.add_exclude(
        name="wrist2_left_finger", bodyname1="wrist_2_link", bodyname2="left_finger"
    )
    robot_spec.add_exclude(
        name="wrist2_right_finger", bodyname1="wrist_2_link", bodyname2="right_finger"
    )

    if free_object:
        robot_spec.worldbody.add_geom(
            name="ground",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[1.0, 1.0, 0.02],
            friction=[1.0, 0.005, 0.0001],
            rgba=[0.8, 0.8, 0.8, 1.0],
        )

    # Known-pose object: fixed for S11.3/4, free on a ground plane for S11.5.
    object_body = robot_spec.worldbody.add_body(
        name="known_object", pos=object_position
    )
    if free_object:
        object_body.add_freejoint(name="known_object_free")
    object_body.add_geom(
        name="known_object_collision",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.02, 0.015, 0.03],
        mass=object_mass,
        friction=[object_friction, 0.005, 0.0001],
        rgba=[0.9, 0.5, 0.2, 1.0],
    )
    return robot_spec.compile()


def orientation_error_world(current_rotation, target_rotation):
    """Return target-from-current rotation vector in world axes, in radians."""
    relative_rotation = target_rotation @ current_rotation.T
    cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cosine)
    if angle < 1e-12:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        raise RuntimeError("orientation error too close to pi")
    skew_vector = np.array([
        relative_rotation[2, 1] - relative_rotation[1, 2],
        relative_rotation[0, 2] - relative_rotation[2, 0],
        relative_rotation[1, 0] - relative_rotation[0, 1],
    ])
    return angle * skew_vector / (2.0 * np.sin(angle))


def site_pose(data, site_id):
    return (
        data.site_xpos[site_id].copy(),
        data.site_xmat[site_id].reshape(3, 3).copy(),
    )


def joint_addresses(model, names):
    qpos = []
    dof = []
    for name in names:
        joint_id = model.joint(name).id
        qpos.append(int(model.jnt_qposadr[joint_id]))
        dof.append(int(model.jnt_dofadr[joint_id]))
    return np.asarray(qpos), np.asarray(dof)


def within_joint_limits(model, joint_names, q_values):
    for name, value in zip(joint_names, q_values):
        joint_id = model.joint(name).id
        if model.jnt_limited[joint_id]:
            lower, upper = model.jnt_range[joint_id]
            if value < lower or value > upper:
                return False
    return True


def solve_pregrasp_ik(model, data, site_id, arm_qpos, arm_dof, q_start,
                      target_position, target_rotation):
    """Run bounded DLS IK on only the six UR5e arm joints."""
    data.qpos[arm_qpos] = q_start
    mujoco.mj_forward(model, data)
    damping = 0.01
    max_joint_step = 0.05  # rad per geometric update
    max_updates = 80

    for update in range(max_updates + 1):
        position, rotation = site_pose(data, site_id)
        position_error_vector = target_position - position
        orientation_error_vector = orientation_error_world(rotation, target_rotation)
        position_error = float(np.linalg.norm(position_error_vector))
        orientation_error = float(np.linalg.norm(orientation_error_vector))
        if position_error < POSITION_TOLERANCE and orientation_error < ORIENTATION_TOLERANCE:
            return data.qpos[arm_qpos].copy(), update, position_error, orientation_error
        if update == max_updates:
            raise RuntimeError("IK reached its update budget")

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        # mj_jacSite fills both arrays in place; columns are model DOFs.
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        jacobian = np.vstack((jacp[:, arm_dof], jacr[:, arm_dof]))
        error = np.concatenate((position_error_vector, orientation_error_vector))
        task_matrix = jacobian @ jacobian.T + damping**2 * np.eye(6)
        delta_q_raw = jacobian.T @ np.linalg.solve(task_matrix, error)
        largest = float(np.max(np.abs(delta_q_raw)))
        scale = min(1.0, max_joint_step / largest) if largest > 0.0 else 1.0
        q_candidate = data.qpos[arm_qpos] + scale * delta_q_raw
        if not within_joint_limits(model, ARM_JOINTS, q_candidate):
            raise RuntimeError("IK candidate violates a joint limit")
        data.qpos[arm_qpos] = q_candidate
        mujoco.mj_forward(model, data)

    raise AssertionError("IK loop must return or raise")


def cubic_reference(q_start, q_goal, duration, dt):
    """Return time, q, and analytic qdot for a zero-end-velocity cubic."""
    intervals = int(round(duration / dt))
    if not np.isclose(intervals * dt, duration, atol=1e-12):
        raise ValueError("duration must be an integer multiple of dt")
    time = np.linspace(0.0, duration, intervals + 1)
    tau = time / duration
    phase = 3.0 * tau**2 - 2.0 * tau**3
    phase_rate = (6.0 * tau - 6.0 * tau**2) / duration
    delta = q_goal - q_start
    q = q_start + phase[:, None] * delta
    qdot = phase_rate[:, None] * delta
    return time, q, qdot


def contact_names(model, data):
    def descriptive_geom_name(geom_id):
        name = model.geom(geom_id).name
        if name:
            return name
        body_name = model.body(model.geom_bodyid[geom_id]).name
        return f"{body_name}/<unnamed:{geom_id}>"

    pairs = set()
    for contact in data.contact:
        names = tuple(sorted((
            descriptive_geom_name(contact.geom1),
            descriptive_geom_name(contact.geom2),
        )))
        pairs.add(names)
    return pairs


def main():
    args = parse_args()
    np.set_printoptions(precision=6, suppress=True)
    model = build_model()
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    arm_qpos, arm_dof = joint_addresses(model, ARM_JOINTS)
    finger_qpos, _ = joint_addresses(model, FINGER_JOINTS)

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[finger_qpos] = 0.03  # 0.08 m opening during transit
    mujoco.mj_forward(model, data)
    q_home = data.qpos[arm_qpos].copy()

    # Reuse S11.2's world-frame pre-grasp target for d=0.10 m.
    target_position = np.array([-0.45, 0.20, 0.165])
    target_rotation = np.diag([1.0, -1.0, -1.0])
    q_goal, updates, ik_position_error, ik_orientation_error = solve_pregrasp_ik(
        model, data, site_id, arm_qpos, arm_dof, q_home,
        target_position, target_rotation,
    )
    time, q_reference, qdot_reference = cubic_reference(
        q_home, q_goal, args.duration, args.dt
    )

    all_within_limits = True
    collision_pairs = set()
    for q_sample in q_reference:
        all_within_limits &= within_joint_limits(model, ARM_JOINTS, q_sample)
        data.qpos[arm_qpos] = q_sample
        data.qpos[finger_qpos] = 0.03
        # Direct qpos assignment + mj_forward checks geometry; it does not simulate tracking.
        mujoco.mj_forward(model, data)
        collision_pairs.update(contact_names(model, data))

    final_position, final_rotation = site_pose(data, site_id)
    final_position_error = float(np.linalg.norm(target_position - final_position))
    final_orientation_error = float(np.linalg.norm(
        orientation_error_world(final_rotation, target_rotation)
    ))
    peak_joint_speed = float(np.max(np.abs(qdot_reference)))

    print(f"IK status=SUCCESS updates={updates}")
    print(f"IK residual: position={ik_position_error:.9f} m "
          f"orientation={ik_orientation_error:.9f} rad")
    print(f"q_home={q_home} rad")
    print(f"q_pregrasp={q_goal} rad")
    print(f"trajectory: duration={args.duration:.3f} s dt={args.dt:.3f} s "
          f"samples={time.size}")
    print(f"peak analytic |qdot|={peak_joint_speed:.6f} rad/s "
          f"limit={args.max_joint_speed:.6f} rad/s")
    print(f"all trajectory samples within joint limits={all_within_limits}")
    print(f"contact pairs across trajectory={sorted(collision_pairs)}")
    print(f"final pose error: position={final_position_error:.9f} m "
          f"orientation={final_orientation_error:.9f} rad")

    assert all_within_limits
    assert peak_joint_speed <= args.max_joint_speed + 1e-12
    assert not collision_pairs
    assert final_position_error < POSITION_TOLERANCE
    assert final_orientation_error < ORIENTATION_TOLERANCE
    assert data.time == 0.0
    print("PASS: IK target, cubic reference, limits, speed, and sampled collisions verified")
    print("No mj_step or actuator tracking was executed")


if __name__ == "__main__":
    main()
