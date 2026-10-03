"""Run bounded iterative UR5e 6D IK for reachable and failure scenarios."""

import argparse
from dataclasses import dataclass

import mujoco
import mujoco_menagerie
import numpy as np


@dataclass
class IkResult:
    scenario: str
    status: str
    accepted_updates: int
    position_error: float
    orientation_error: float
    largest_step: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("all", "reachable", "difficult", "unreachable", "invalid"),
                        default="all")
    parser.add_argument("--max-updates", type=int, default=80)
    parser.add_argument("--max-joint-step", type=float, default=0.05,
                        help="maximum absolute component accepted per update, rad")
    parser.add_argument("--damping", type=float, default=0.01)
    args = parser.parse_args()
    if args.max_updates <= 0:
        parser.error("--max-updates must be positive")
    if not np.isfinite(args.max_joint_step) or args.max_joint_step <= 0:
        parser.error("--max-joint-step must be positive and finite")
    if not np.isfinite(args.damping) or args.damping <= 0:
        parser.error("--damping must be positive and finite")
    return args


def rotation_matrix_from_vector(rotation_vector):
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-12:
        return np.eye(3)
    axis = rotation_vector / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def orientation_error_world(current_rotation, target_rotation):
    relative_rotation = target_rotation @ current_rotation.T
    cosine_angle = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cosine_angle)
    if angle < 1e-12:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        raise ValueError("orientation error is too close to pi for this lesson")
    skew_vector = np.array([
        relative_rotation[2, 1] - relative_rotation[1, 2],
        relative_rotation[0, 2] - relative_rotation[2, 0],
        relative_rotation[1, 0] - relative_rotation[0, 1],
    ])
    return angle * skew_vector / (2.0 * np.sin(angle))


def site_pose(data, site_id):
    return (data.site_xpos[site_id].copy(),
            data.site_xmat[site_id].reshape(3, 3).copy())


def full_site_jacobian(model, data, site_id):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return np.vstack((jacp, jacr))


def valid_target(target_position, target_rotation):
    return (
        target_position.shape == (3,)
        and target_rotation.shape == (3, 3)
        and np.isfinite(target_position).all()
        and np.isfinite(target_rotation).all()
        and np.allclose(target_rotation.T @ target_rotation, np.eye(3), rtol=0.0, atol=1e-8)
        and np.isclose(np.linalg.det(target_rotation), 1.0, rtol=0.0, atol=1e-8)
    )


def within_joint_limits(model, q_candidate):
    limited = model.jnt_limited.astype(bool)
    return bool(np.all(
        (~limited)
        | ((q_candidate >= model.jnt_range[:, 0]) & (q_candidate <= model.jnt_range[:, 1]))
    ))


def make_target(model, data, site_id, q_initial, scenario):
    if scenario == "invalid":
        return np.array([np.nan, 0.0, 0.0]), np.eye(3)
    if scenario == "unreachable":
        data.qpos[:] = q_initial
        mujoco.mj_forward(model, data)
        position, rotation = site_pose(data, site_id)
        return position + np.array([2.0, 0.0, 0.0]), rotation

    offset = {
        "reachable": np.array([0.02, -0.015, 0.01, -0.01, 0.015, -0.02]),
        "difficult": np.array([0.30, -0.25, 0.20, -0.20, 0.20, -0.30]),
    }[scenario]
    data.qpos[:] = q_initial + offset
    mujoco.mj_forward(model, data)
    return site_pose(data, site_id)


def solve_iterative(model, data, site_id, q_initial, target_position, target_rotation,
                    scenario, max_updates, max_joint_step, damping):
    position_tolerance = 1e-4  # m
    orientation_tolerance = 1e-3  # rad
    last_step = 0.0

    if not valid_target(target_position, target_rotation):
        return IkResult(scenario, "INVALID_TARGET", 0, np.nan, np.nan, 0.0)

    data.qpos[:] = q_initial
    mujoco.mj_forward(model, data)
    for update in range(max_updates + 1):
        position, rotation = site_pose(data, site_id)
        position_error_vector = target_position - position
        try:
            orientation_error_vector = orientation_error_world(rotation, target_rotation)
        except ValueError:
            return IkResult(scenario, "INVALID_ORIENTATION_ERROR", update,
                            np.linalg.norm(position_error_vector), np.nan, last_step)
        position_error = np.linalg.norm(position_error_vector)
        orientation_error = np.linalg.norm(orientation_error_vector)

        if update == 0 or update % 10 == 0:
            print(f"  update={update:2d} pos={position_error:.9f} m "
                  f"rot={orientation_error:.9f} rad max_step={last_step:.9f} rad")
        if position_error < position_tolerance and orientation_error < orientation_tolerance:
            return IkResult(scenario, "SUCCESS", update, position_error,
                            orientation_error, last_step)
        if update == max_updates:
            return IkResult(scenario, "UPDATE_LIMIT", update, position_error,
                            orientation_error, last_step)

        error = np.concatenate((position_error_vector, orientation_error_vector))
        jacobian = full_site_jacobian(model, data, site_id)
        task_matrix = jacobian @ jacobian.T + damping**2 * np.eye(6)
        delta_q_raw = jacobian.T @ np.linalg.solve(task_matrix, error)
        if not np.isfinite(delta_q_raw).all():
            return IkResult(scenario, "NONFINITE_UPDATE", update,
                            position_error, orientation_error, last_step)

        largest_raw = np.max(np.abs(delta_q_raw))
        scale = min(1.0, max_joint_step / largest_raw) if largest_raw > 0.0 else 1.0
        delta_q = scale * delta_q_raw
        last_step = float(np.max(np.abs(delta_q)))
        q_candidate = data.qpos + delta_q
        if not within_joint_limits(model, q_candidate):
            return IkResult(scenario, "JOINT_LIMIT_REJECTED", update,
                            position_error, orientation_error, last_step)

        data.qpos[:] = q_candidate
        mujoco.mj_forward(model, data)

    raise AssertionError("loop must return through an explicit terminal status")


def main():
    args = parse_args()
    model = mujoco_menagerie.load("universal_robots_ur5e")
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    q_initial = data.qpos.copy()

    scenarios = ("reachable", "difficult", "unreachable", "invalid") \
        if args.scenario == "all" else (args.scenario,)
    results = []
    for scenario in scenarios:
        target_position, target_rotation = make_target(
            model, data, site_id, q_initial, scenario
        )
        print(f"\nScenario: {scenario}")
        result = solve_iterative(
            model, data, site_id, q_initial, target_position, target_rotation,
            scenario, args.max_updates, args.max_joint_step, args.damping,
        )
        results.append(result)
        print(f"  status={result.status} accepted_updates={result.accepted_updates} "
              f"pos={result.position_error:.9f} m rot={result.orientation_error:.9f} rad")

    by_scenario = {result.scenario: result for result in results}
    if args.scenario == "all":
        assert by_scenario["reachable"].status == "SUCCESS"
        assert by_scenario["difficult"].status == "SUCCESS"
        assert by_scenario["unreachable"].status in {"UPDATE_LIMIT", "JOINT_LIMIT_REJECTED"}
        assert by_scenario["invalid"].status == "INVALID_TARGET"
    assert data.time == 0.0
    print("\nPASS: every scenario ended with an explicit, expected status")


if __name__ == "__main__":
    main()
