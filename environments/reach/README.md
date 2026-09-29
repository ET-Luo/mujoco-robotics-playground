# Reach

Status: documentation-only placeholder; no implementation yet.

Goal: Move an end effector to a target.

TODO: Define target coordinates, distance tolerance, and termination conditions.

S8.1 completed specification: observation has 8 values (two joint angles, two joint
velocities, world xy end-effector position, and world xy target). Action is two joint
velocity commands in [-0.2, 0.2] rad/s at an assumed 0.02 s control period. Reward is
negative world-xy distance. Success terminates below 0.01 m; 100 unsuccessful steps
truncate the episode. The learner correctly checked the observation units, reward, success,
and timeout cases. S8.1 was documentation only and did not yet include an interface.

S8.2 has started with `reach_env.py`. It defines the standard Gymnasium environment base,
an eight-value observation space, planar forward kinematics, and reset state mutations.
The learner correctly assembled the four observation segments. Reset validation returned the
expected values, shape, and dtype, passed the observation-space check, and restored modified
state on a second reset. The learner correctly explained observation versus info and why the
end-effector position must be recomputed from reset joint state. S8.2 is complete. No `step`
method has been added.
