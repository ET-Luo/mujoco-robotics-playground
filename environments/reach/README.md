# Reach

Status: S8.3 complete; `step` is a geometric update, not MuJoCo dynamics.

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
end-effector position must be recomputed from reset joint state. S8.2 is complete.

S8.3 adds a two-value velocity `action_space` and a geometric `step`: actions are clipped,
stored as commanded `qvel`, and integrated for 0.02 s before reward and ending conditions
are evaluated. Gymnasium's environment checker and explicit action/ending boundary checks
pass. A 100-step random-action check truncated without success, as expected for commands
that do not deliberately pursue the target. The learner correctly explained that this tests
the environment logic rather than random-policy task success, and that assigned command
velocity is not a physics-engine-computed `data.qvel`. S8.3 is complete.

S8.4 compares `-distance` with `-(distance**2)` in `reward_comparison.py`. At 0.02 m
and 0.20 m, the learner correctly calculated linear rewards of -0.02/-0.20 and squared
rewards of -0.0004/-0.04. Both prefer smaller distance. Squaring increases the far/near
ratio while reducing absolute reward magnitude for distances below 1 m. The numerical
values, improvements, and monotonic preference checks pass. S8.4 is complete.
