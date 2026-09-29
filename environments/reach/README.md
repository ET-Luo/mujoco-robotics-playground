# Reach

Status: documentation-only placeholder; no implementation yet.

Goal: Move an end effector to a target.

TODO: Define target coordinates, distance tolerance, and termination conditions.

S8.1 draft specification: observation has 8 values (two joint angles, two joint
velocities, world xy end-effector position, and world xy target). Action is two joint
velocity commands in [-0.2, 0.2] rad/s at an assumed 0.02 s control period. Reward is
negative world-xy distance. Success terminates below 0.01 m; 100 unsuccessful steps
truncate the episode. This is documentation only; no Gymnasium dependency or interface exists.
