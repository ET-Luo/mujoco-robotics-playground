"""Track one UR5e 6D target with bounded geometric joint-velocity commands."""

import argparse

import mujoco
import mujoco_menagerie
import numpy as np

from iterative_ik import (
    full_site_jacobian,
    make_target,
    orientation_error_world,
    site_pose,
    within_joint_limits,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-dt", type=float, default=0.02,
                        help="assumed geometric control period in seconds")
    parser.add_argument("--max-joint-speed", type=float, default=0.5,
                        help="absolute command-speed limit for every joint, rad/s")
    parser.add_argument("--max-updates", type=int, default=200)
    args = parser.parse_args()
    if not np.isfinite(args.control_dt) or args.control_dt <= 0:
        parser.error("--control-dt must be positive and finite")
    if not np.isfinite(args.max_joint_speed) or args.max_joint_speed <= 0:
        parser.error("--max-joint-speed must be positive and finite")
    if args.max_updates <= 0:
        parser.error("--max-updates must be positive")
    return args


def main():
    args = parse_args()
    np.set_printoptions(precision=9, suppress=True)
    model = mujoco_menagerie.load("universal_robots_ur5e")
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    q_initial = data.qpos.copy()
    target_position, target_rotation = make_target(
        model, data, site_id, q_initial, "difficult"
    )
    data.qpos[:] = q_initial
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    damping = 0.01
    position_tolerance = 1e-4  # m
    orientation_tolerance = 1e-3  # rad
    speed_limits = np.full(model.nv, args.max_joint_speed)  # rad/s
    step_limits = speed_limits * args.control_dt  # rad per update
    saturated_updates = 0
    largest_command_seen = 0.0

    for update in range(args.max_updates + 1):
        position, rotation = site_pose(data, site_id)
        position_error_vector = target_position - position
        orientation_error_vector = orientation_error_world(rotation, target_rotation)
        position_error = np.linalg.norm(position_error_vector)
        orientation_error = np.linalg.norm(orientation_error_vector)
        if position_error < position_tolerance and orientation_error < orientation_tolerance:
            virtual_time = update * args.control_dt
            print(f"SUCCESS after {update} updates; virtual time={virtual_time:.6f} s")
            print(f"Final position error={position_error:.12f} m")
            print(f"Final orientation error={orientation_error:.12f} rad")
            print(f"Saturated updates={saturated_updates}")
            print(f"Largest |qdot_command|={largest_command_seen:.12f} rad/s")
            print(f"MuJoCo data.qvel={data.qvel} rad/s")
            assert largest_command_seen <= args.max_joint_speed + 1e-12
            np.testing.assert_allclose(data.qvel, 0.0, rtol=0.0, atol=0.0)
            assert data.time == 0.0
            print("PASS: commands respected speed limits; qvel remained an unexecuted-state value")
            return
        if update == args.max_updates:
            raise RuntimeError("update limit reached before satisfying both tolerances")

        error = np.concatenate((position_error_vector, orientation_error_vector))
        jacobian = full_site_jacobian(model, data, site_id)
        task_matrix = jacobian @ jacobian.T + damping**2 * np.eye(6)
        delta_q_raw = jacobian.T @ np.linalg.solve(task_matrix, error)

        # One common scale preserves the DLS joint-space direction while making
        # every component satisfy |delta_q_i| <= qdot_max_i * control_dt.
        ratios = np.divide(
            step_limits,
            np.abs(delta_q_raw),
            out=np.full_like(step_limits, np.inf),
            where=np.abs(delta_q_raw) > 0.0,
        )
        scale = min(1.0, float(np.min(ratios)))
        if scale < 1.0:
            saturated_updates += 1
        delta_q = scale * delta_q_raw
        qdot_command = delta_q / args.control_dt
        largest_command_seen = max(largest_command_seen, float(np.max(np.abs(qdot_command))))

        assert np.all(np.abs(delta_q) <= step_limits + 1e-12)
        assert np.all(np.abs(qdot_command) <= speed_limits + 1e-12)
        q_candidate = data.qpos + delta_q
        if not within_joint_limits(model, q_candidate):
            raise RuntimeError("velocity-limited candidate violates a joint position limit")
        if update == 0:
            print(f"control_dt={args.control_dt:.6f} s")
            print(f"speed limits={speed_limits} rad/s")
            print(f"step limits={step_limits} rad/update")
            print(f"first raw delta_q={delta_q_raw} rad")
            print(f"first scale={scale:.12f}")
            print(f"first qdot_command={qdot_command} rad/s")

        # This is geometric integration, not actuator execution. mj_forward
        # refreshes kinematics but does not turn qdot_command into data.qvel.
        data.qpos[:] = q_candidate
        mujoco.mj_forward(model, data)


if __name__ == "__main__":
    main()
