"""Minimal reset and observation interface for a planar two-link Reach task."""

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class PlanarReachEnv(gym.Env):
    """Expose reset observations before adding actions or simulation stepping."""

    def __init__(self):
        self.link_lengths = np.array([0.4, 0.3], dtype=np.float64)
        self.target_xy = np.array([0.5, 0.2], dtype=np.float64)

        # Observation: q (rad), qvel (rad/s), end-effector xy (m), target xy (m).
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(8,),
            dtype=np.float64,
        )

        self.q = np.zeros(2, dtype=np.float64)
        self.qvel = np.zeros(2, dtype=np.float64)
        self.step_count = 0

    def _end_effector_xy(self):
        """Return planar forward kinematics in the world frame, in meters."""
        q1, q2 = self.q
        l1, l2 = self.link_lengths
        x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
        return np.array([x, y], dtype=np.float64)

    def _get_observation(self):
        """Return the current task state as a one-dimensional NumPy array."""
        end_effector_xy = self._end_effector_xy()

        # This concatenation was written by the learner.
        observation = np.concatenate(
            [self.q, self.qvel, end_effector_xy, self.target_xy]
        )
        return observation

    def reset(self, *, seed=None, options=None):
        """Restore the fixed initial state and return ``(observation, info)``."""
        super().reset(seed=seed)
        self.q[:] = 0.0
        self.qvel[:] = 0.0
        self.step_count = 0

        observation = self._get_observation()
        info = {}
        return observation, info
