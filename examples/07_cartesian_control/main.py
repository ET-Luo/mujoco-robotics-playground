"""Apply one limited planar Cartesian position update and verify it with FK."""

import numpy as np


def planar_fk(q):
    """Return world xy position (2,) in meters for joint angles q (2,) in radians."""
    q1, q2 = q
    return np.array([
        0.4 * np.cos(q1) + 0.3 * np.cos(q1 + q2),
        0.4 * np.sin(q1) + 0.3 * np.sin(q1 + q2),
    ])


def planar_jacobian(q):
    """Return the world-xy position Jacobian (2,2) in m/rad."""
    q1, q2 = q
    return np.array([
        [-0.4 * np.sin(q1) - 0.3 * np.sin(q1 + q2), -0.3 * np.sin(q1 + q2)],
        [0.4 * np.cos(q1) + 0.3 * np.cos(q1 + q2), 0.3 * np.cos(q1 + q2)],
    ])


def main():
    np.set_printoptions(precision=12, suppress=True)
    q = np.array([0.0, np.pi / 2])
    position = planar_fk(q)
    target = np.array([0.397, 0.3])  # World xy, meters.
    error = target - position
    jac = planar_jacobian(q)

    delta_q = np.linalg.solve(jac, error)
    max_joint_step = 0.004  # Maximum absolute component per update, radians.
    largest_component = np.max(np.abs(delta_q))
    scale = min(1.0, max_joint_step / largest_component)
    delta_q_limited = scale * delta_q

    predicted_displacement = jac @ delta_q_limited
    q_new = q + delta_q_limited
    position_new = planar_fk(q_new)
    actual_displacement = position_new - position
    error_new = target - position_new

    print(f"Initial error: {error} m")
    print(f"Raw joint increment: {delta_q} rad")
    print(f"Scale: {scale:.12f}")
    print(f"Limited joint increment: {delta_q_limited} rad")
    print(f"Linear displacement prediction: {predicted_displacement} m")
    print(f"Actual FK displacement: {actual_displacement} m")
    print(f"Error norm: {np.linalg.norm(error):.12f} -> {np.linalg.norm(error_new):.12f} m")

    assert np.max(np.abs(delta_q_limited)) <= max_joint_step
    assert np.linalg.norm(error_new) < np.linalg.norm(error)


if __name__ == "__main__":
    main()
