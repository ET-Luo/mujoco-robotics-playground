"""Compare one nominal Jacobian controller across fixed and randomized link lengths."""

import numpy as np

from environments.reach.domain_randomization import SEED, sample_sequence


NOMINAL_LENGTHS = np.array([0.4, 0.3], dtype=np.float64)
INITIAL_Q = np.array([0.0, np.pi / 2], dtype=np.float64)
TARGET_XY = np.array([0.397, 0.300], dtype=np.float64)
CONTROL_DT = 0.02
MAX_JOINT_SPEED = 0.2
MAX_UPDATES = 20
TOLERANCE = 1e-4


def planar_fk(q, link_lengths):
    """Return actual world-xy end-effector position in meters."""
    q1, q2 = q
    l1, l2 = link_lengths
    return np.array(
        [
            l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
            l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
        ],
        dtype=np.float64,
    )


def planar_jacobian(q, link_lengths):
    """Return the world-xy position Jacobian in m/rad."""
    q1, q2 = q
    l1, l2 = link_lengths
    return np.array(
        [
            [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2)],
            [l1 * np.cos(q1) + l2 * np.cos(q1 + q2), l2 * np.cos(q1 + q2)],
        ],
        dtype=np.float64,
    )


def run_episode(actual_second_link_length):
    """Run feedback with actual FK but a fixed nominal-model Jacobian."""
    actual_lengths = np.array([0.4, actual_second_link_length], dtype=np.float64)
    q = INITIAL_Q.copy()
    initial_distance = float(np.linalg.norm(TARGET_XY - planar_fk(q, actual_lengths)))
    max_joint_step = MAX_JOINT_SPEED * CONTROL_DT

    for update in range(MAX_UPDATES + 1):
        actual_position = planar_fk(q, actual_lengths)
        error = TARGET_XY - actual_position
        distance = float(np.linalg.norm(error))

        if distance < TOLERANCE:
            return initial_distance, distance, True, update
        if update == MAX_UPDATES:
            return initial_distance, distance, False, update

        # The controller retains its nominal 0.30 m model of the second link.
        nominal_jacobian = planar_jacobian(q, NOMINAL_LENGTHS)
        delta_q = np.linalg.solve(nominal_jacobian, error)
        largest_component = float(np.max(np.abs(delta_q)))
        scale = min(1.0, max_joint_step / largest_component)
        q += scale * delta_q


def print_result(label, second_link_length, result):
    initial_distance, final_distance, success, updates = result
    print(
        f"{label}: l2={second_link_length:.8f} m, "
        f"initial={initial_distance:.9f} m, final={final_distance:.9f} m, "
        f"success={success}, updates={updates}"
    )


def main():
    fixed_result = run_episode(NOMINAL_LENGTHS[1])
    print_result("fixed", NOMINAL_LENGTHS[1], fixed_result)

    random_lengths = sample_sequence(SEED)
    random_results = []
    for episode, second_link_length in enumerate(random_lengths, start=1):
        result = run_episode(second_link_length)
        random_results.append(result)
        print_result(f"random_{episode}", second_link_length, result)

    random_final_distances = np.array(
        [result[1] for result in random_results], dtype=np.float64
    )
    random_successes = np.array(
        [result[2] for result in random_results], dtype=np.bool_
    )

    print(f"random_mean_final={np.mean(random_final_distances):.9f} m")
    print(f"random_max_final={np.max(random_final_distances):.9f} m")
    print(f"random_success_rate={np.mean(random_successes):.2f}")

    assert fixed_result[2]
    assert fixed_result[1] < TOLERANCE
    assert random_final_distances.shape == (5,)
    assert np.all(np.isfinite(random_final_distances))


if __name__ == "__main__":
    main()
