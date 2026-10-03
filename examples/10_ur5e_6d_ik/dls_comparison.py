"""Compare undamped least-squares and DLS near a UR5e wrist singularity."""

import argparse

import mujoco
import mujoco_menagerie
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wrist-angle", type=float, default=0.01,
                        help="wrist_2 angle in radians; zero is the singular reference")
    args = parser.parse_args()
    if not np.isfinite(args.wrist_angle) or not 1e-4 <= abs(args.wrist_angle) <= 0.2:
        parser.error("absolute --wrist-angle must be finite and in [1e-4, 0.2] rad")
    return args


def full_site_jacobian(model, data, site_id):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    return np.vstack((jacp, jacr))


def rotation_matrix_from_vector(rotation_vector):
    """Return Exp(rotation_vector) for a nonzero world-frame vector."""
    angle = np.linalg.norm(rotation_vector)
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
    skew_vector = np.array([
        relative_rotation[2, 1] - relative_rotation[1, 2],
        relative_rotation[0, 2] - relative_rotation[2, 0],
        relative_rotation[1, 0] - relative_rotation[0, 1],
    ])
    return angle * skew_vector / (2.0 * np.sin(angle))


def pose_error(data, site_id, target_position, target_rotation):
    position_error = target_position - data.site_xpos[site_id]
    current_rotation = data.site_xmat[site_id].reshape(3, 3)
    rotation_error = orientation_error_world(current_rotation, target_rotation)
    return np.concatenate((position_error, rotation_error))


def damped_least_squares(jacobian, error, damping):
    """Return J.T (J J.T + lambda^2 I)^-1 error without forming an inverse."""
    task_matrix = jacobian @ jacobian.T + damping**2 * np.eye(jacobian.shape[0])
    return jacobian.T @ np.linalg.solve(task_matrix, error)


def evaluate_candidate(model, data, site_id, q_initial, delta_q,
                       target_position, target_rotation):
    candidate = q_initial + delta_q
    limited = model.jnt_limited.astype(bool)
    within_limits = np.all(
        (~limited)
        | ((candidate >= model.jnt_range[:, 0]) & (candidate <= model.jnt_range[:, 1]))
    )
    if not within_limits:
        raise RuntimeError("Candidate violates a joint position limit")
    data.qpos[:] = candidate
    mujoco.mj_forward(model, data)
    error_after = pose_error(data, site_id, target_position, target_rotation)
    return error_after, within_limits


def main():
    args = parse_args()
    np.set_printoptions(precision=9, suppress=True)
    model = mujoco_menagerie.load("universal_robots_ur5e")
    data = mujoco.MjData(model)
    site_id = model.site("attachment_site").id
    wrist_2_id = model.joint("wrist_2_joint").id
    wrist_2_qpos = int(model.jnt_qposadr[wrist_2_id])

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[wrist_2_qpos] = args.wrist_angle
    mujoco.mj_forward(model, data)
    q_initial = data.qpos.copy()
    initial_position = data.site_xpos[site_id].copy()
    initial_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    jacobian = full_site_jacobian(model, data, site_id)
    left_vectors, singular_values, _ = np.linalg.svd(jacobian)
    condition_number = singular_values[0] / singular_values[-1]

    # A 1e-3 numerical error along the weakest left-singular direction exposes
    # inverse amplification. Its first/last components use m/rad respectively.
    requested_error = 1e-3 * left_vectors[:, -1]
    target_position = initial_position + requested_error[:3]
    target_rotation = rotation_matrix_from_vector(requested_error[3:]) @ initial_rotation
    error_before = pose_error(data, site_id, target_position, target_rotation)

    methods = (
        ("least_squares", None),
        ("dls_small", 1e-3),
        ("dls_medium", 1e-2),
    )
    results = []
    for name, damping in methods:
        if damping is None:
            delta_q = np.linalg.lstsq(jacobian, error_before, rcond=None)[0]
        else:
            delta_q = damped_least_squares(jacobian, error_before, damping)
        error_after, within_limits = evaluate_candidate(
            model, data, site_id, q_initial, delta_q, target_position, target_rotation
        )
        results.append((name, damping, delta_q, error_after, within_limits))

    print(f"wrist_2 angle={args.wrist_angle:.6f} rad")
    print(f"singular values={singular_values}")
    print(f"condition number={condition_number:.6f}")
    print(f"initial position error={np.linalg.norm(error_before[:3]):.12f} m")
    print(f"initial orientation error={np.linalg.norm(error_before[3:]):.12f} rad")
    print("\nmethod        lambda     ||delta_q|| rad   pos_after m       rot_after rad")
    for name, damping, delta_q, error_after, within_limits in results:
        damping_text = "none" if damping is None else f"{damping:.0e}"
        print(f"{name:14s}{damping_text:>7s}     {np.linalg.norm(delta_q):.12f}   "
              f"{np.linalg.norm(error_after[:3]):.12f}   "
              f"{np.linalg.norm(error_after[3:]):.12f}")
        assert within_limits
        assert np.isfinite(delta_q).all()
        assert np.isfinite(error_after).all()

    update_norms = [np.linalg.norm(result[2]) for result in results]
    if np.isclose(args.wrist_angle, 0.01):
        assert condition_number > 500.0
        assert update_norms[2] < update_norms[1] < update_norms[0]
        medium_error = results[2][3]
        assert np.linalg.norm(medium_error) < np.linalg.norm(error_before)
    assert data.time == 0.0
    print("PASS: more damping reduced the near-singular joint update; candidates stayed finite")


if __name__ == "__main__":
    main()
