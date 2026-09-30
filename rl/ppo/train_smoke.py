"""Run a tiny CPU-only PPO integration check on the planar Reach environment."""

import time

import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import PPO

from environments.reach.reach_env import PlanarReachEnv


SEED = 7
TOTAL_TIMESTEPS = 256


def evaluate_episode(model):
    """Run one deterministic episode and report task-level results."""
    env = PlanarReachEnv()
    observation, _ = env.reset(seed=SEED)
    episode_return = 0.0

    for step in range(1, env.max_steps + 1):
        action, _ = model.predict(observation, deterministic=True)
        observation, reward, terminated, truncated, info = env.step(action)
        episode_return += float(reward)
        if terminated or truncated:
            break

    return {
        "steps": step,
        "return": episode_return,
        "distance": float(info["distance"]),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
    }


def print_evaluation(label, result):
    """Print one compact, readable evaluation summary."""
    print(
        f"{label}: steps={result['steps']}, return={result['return']:.6f}, "
        f"distance={result['distance']:.6f} m, "
        f"terminated={result['terminated']}, truncated={result['truncated']}"
    )


def main():
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.set_num_threads(1)

    print(f"stable_baselines3={stable_baselines3.__version__}")
    print(f"torch={torch.__version__}")
    print(f"cuda_available={torch.cuda.is_available()}")
    print(f"total_timesteps={TOTAL_TIMESTEPS}")

    env = PlanarReachEnv()
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=64,
        batch_size=32,
        n_epochs=2,
        gamma=0.99,
        seed=SEED,
        device="cpu",
        policy_kwargs={"net_arch": [32, 32]},
        verbose=1,
    )

    print_evaluation("before_training", evaluate_episode(model))
    start_time = time.perf_counter()
    model.learn(total_timesteps=TOTAL_TIMESTEPS)
    elapsed_seconds = time.perf_counter() - start_time
    print_evaluation("after_training", evaluate_episode(model))
    print(f"elapsed_seconds={elapsed_seconds:.3f}")


if __name__ == "__main__":
    main()
