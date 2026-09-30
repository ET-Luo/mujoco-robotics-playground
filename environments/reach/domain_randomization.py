"""Check one seeded domain-randomization parameter for the Reach environment."""

import numpy as np

from environments.reach.reach_env import PlanarReachEnv


SEED = 7
EPISODES = 5


def sample_sequence(seed):
    """Return consecutive second-link samples from one seeded environment."""
    env = PlanarReachEnv(randomize_link_length=True)
    samples = []

    # Seed once. Later resets continue, rather than restart, the same random sequence.
    _, info = env.reset(seed=seed)
    samples.append(info["second_link_length"])
    for _ in range(EPISODES - 1):
        _, info = env.reset()
        samples.append(info["second_link_length"])

    env.close()
    return np.array(samples, dtype=np.float64)


def main():
    first = sample_sequence(SEED)
    second = sample_sequence(SEED)
    low, high = PlanarReachEnv().second_link_length_range

    in_range = bool(np.all((first >= low) & (first <= high)))
    reproducible = bool(np.array_equal(first, second))
    varies_across_episodes = bool(np.unique(first).size > 1)

    fixed_env = PlanarReachEnv()
    fixed_observation, fixed_info = fixed_env.reset(seed=SEED)
    fixed_env.close()

    print(f"range=[{low:.2f}, {high:.2f}] m")
    print(f"first_sequence={first}")
    print(f"second_sequence={second}")
    print(f"in_range={in_range}")
    print(f"reproducible={reproducible}")
    print(f"varies_across_episodes={varies_across_episodes}")
    print(f"fixed_l2={fixed_info['second_link_length']:.2f} m")
    print(f"fixed_reset_ee={fixed_observation[4:6]} m")

    assert in_range
    assert reproducible
    assert varies_across_episodes
    assert np.isclose(fixed_info["second_link_length"], 0.30)
    assert np.allclose(fixed_observation[4:6], [0.70, 0.0])


if __name__ == "__main__":
    main()
