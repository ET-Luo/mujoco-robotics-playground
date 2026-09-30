"""Compare nominal Cartesian control with and without position observation noise."""

import numpy as np

from environments.reach.compare_randomized_control import (
    CONTROL_DT,
    INITIAL_Q,
    MAX_JOINT_SPEED,
    NOMINAL_LENGTHS,
    TARGET_XY,
    planar_fk,
    planar_jacobian,
)


SEED = 7
NOISE_STD = 0.001  # One standard deviation per world-xy component, meters.
UPDATES = 20
RMS_WINDOW = 5


def run_episode(noise_std, seed):
    """Run a fixed update budget and return true, noise-free distance history."""
    rng = np.random.default_rng(seed)
    q = INITIAL_Q.copy()
    max_joint_step = MAX_JOINT_SPEED * CONTROL_DT
    true_distances = []

    for update in range(UPDATES + 1):
        true_position = planar_fk(q, NOMINAL_LENGTHS)
        true_distance = float(np.linalg.norm(TARGET_XY - true_position))
        true_distances.append(true_distance)

        if update == UPDATES:
            break

        noise = rng.normal(0.0, noise_std, size=2)
        observed_position = true_position + noise
        observed_error = TARGET_XY - observed_position

        jacobian = planar_jacobian(q, NOMINAL_LENGTHS)
        delta_q = np.linalg.solve(jacobian, observed_error)
        largest_component = float(np.max(np.abs(delta_q)))
        if largest_component > 0.0:
            scale = min(1.0, max_joint_step / largest_component)
            q += scale * delta_q

    return np.array(true_distances, dtype=np.float64)


def summarize(label, distances):
    """Print final true distance and RMS over the final update window."""
    final_distance = float(distances[-1])
    tail_rms = float(np.sqrt(np.mean(np.square(distances[-RMS_WINDOW:]))))
    print(f"{label}_true_distances={distances}")
    print(f"{label}_final_true_distance={final_distance:.9f} m")
    print(f"{label}_tail_rms_true_distance={tail_rms:.9f} m")
    return final_distance, tail_rms


def main():
    baseline_distances = run_episode(noise_std=0.0, seed=SEED)
    noisy_distances = run_episode(noise_std=NOISE_STD, seed=SEED)

    baseline_final, baseline_rms = summarize("baseline", baseline_distances)
    noisy_final, noisy_rms = summarize("noisy", noisy_distances)

    print(f"final_distance_increase={noisy_final - baseline_final:.9f} m")
    print(f"tail_rms_increase={noisy_rms - baseline_rms:.9f} m")

    assert baseline_distances.shape == (UPDATES + 1,)
    assert noisy_distances.shape == (UPDATES + 1,)
    assert np.all(np.isfinite(baseline_distances))
    assert np.all(np.isfinite(noisy_distances))
    assert baseline_final < noisy_final
    assert baseline_rms < noisy_rms


if __name__ == "__main__":
    main()
