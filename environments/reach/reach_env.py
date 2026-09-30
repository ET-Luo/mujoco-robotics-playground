"""Minimal Gymnasium interface for a planar two-link Reach task."""

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class PlanarReachEnv(gym.Env):
    """Use joint velocity commands to move a planar two-link end effector."""

    def __init__(self, randomize_link_length=False):
        self.nominal_link_lengths = np.array([0.4, 0.3], dtype=np.float64)
        self.link_lengths = self.nominal_link_lengths.copy()
        self.randomize_link_length = randomize_link_length
        self.second_link_length_range = (0.27, 0.33)
        self.target_xy = np.array([0.5, 0.2], dtype=np.float64)
        self.control_dt = 0.02
        self.max_joint_speed = 0.2
        self.success_tolerance = 0.01
        self.max_steps = 100

        # Action: commanded joint velocities in rad/s.
        self.action_space = spaces.Box(
            low=-self.max_joint_speed,
            high=self.max_joint_speed,
            shape=(2,),
            dtype=np.float64,
        )

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
        """Restore state, optionally sample link length, and return observation/info."""
        # Gymnasium initializes self.np_random here. Passing a seed restarts its sequence.
        super().reset(seed=seed)
        self.q[:] = 0.0
        self.qvel[:] = 0.0
        self.step_count = 0

        self.link_lengths[:] = self.nominal_link_lengths
        if self.randomize_link_length:
            low, high = self.second_link_length_range
            self.link_lengths[1] = self.np_random.uniform(low, high)

        observation = self._get_observation()
        info = {"second_link_length": float(self.link_lengths[1])}
        return observation, info

    def step(self, action):
        """Apply one geometric velocity update and return Gymnasium's five values."""
        # These state-update and ending expressions were written by the learner.
        clipped_action = np.clip(
            action,
            -self.max_joint_speed,
            self.max_joint_speed,
        )
        self.qvel[:] = clipped_action
        self.q[:] = self.q + self.qvel * self.control_dt
        self.step_count += 1

        observation = self._get_observation()
        distance = np.linalg.norm(self._end_effector_xy() - self.target_xy)
        reward = -distance
        terminated = distance < self.success_tolerance
        truncated = self.step_count >= self.max_steps and not terminated
        info = {"distance": distance}

        return observation, reward, terminated, truncated, info
