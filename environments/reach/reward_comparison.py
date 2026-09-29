"""Compare linear and squared distance rewards at two distances."""

import numpy as np


def linear_reward(distance):
    """Return the negative distance reward."""
    return -distance


def squared_reward(distance):
    """Return the negative squared-distance reward."""
    return -(distance**2)


def main():
    distances = np.array([0.02, 0.20], dtype=np.float64)
    linear_rewards = linear_reward(distances)
    squared_rewards = squared_reward(distances)

    print("distance_m  linear_reward  squared_reward")
    for distance, reward_a, reward_b in zip(
        distances, linear_rewards, squared_rewards
    ):
        print(f"{distance:10.2f}  {reward_a:13.4f}  {reward_b:14.4f}")

    linear_improvement = linear_rewards[0] - linear_rewards[1]
    squared_improvement = squared_rewards[0] - squared_rewards[1]
    print(f"linear improvement:  {linear_improvement:.4f}")
    print(f"squared improvement: {squared_improvement:.4f}")

    assert np.all(np.diff(linear_rewards) < 0.0)
    assert np.all(np.diff(squared_rewards) < 0.0)
    assert np.isclose(linear_improvement, 0.18)
    assert np.isclose(squared_improvement, 0.0396)


if __name__ == "__main__":
    main()
