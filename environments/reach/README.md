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

S9.1 adds an opt-in randomization of only the second link length in `[0.27, 0.33]` m.
The default environment remains fixed at 0.30 m. `reset(seed=...)` initializes Gymnasium's
environment RNG before the length is sampled; later unseeded resets continue that sequence.
With seed 7, two independent environments both produced
`[0.30750573, 0.32383283, 0.31654114, 0.28351243, 0.28800998]` m. All samples were in
range and varied between episodes. The fixed environment still reset to end-effector
position `(0.70, 0)` m. The learner correctly explained that matching independent sequences
show reproducibility, while variation within each sequence shows per-episode randomization.
They also explained why the RNG is seeded once and then allowed to advance; reseeding every
reset would repeat the first sample. Engineering and learning checks pass; S9.1 is complete.

S9.2 reuses the limited Jacobian feedback controller from Stage 6. Actual end-effector
positions use each episode's sampled link length, while the controller deliberately retains
the nominal 0.30 m Jacobian model. The fixed case succeeded in 3 updates with final distance
about 0.000000590 m. All five seed-7 randomized cases also met the 0.0001 m tolerance, but
needed 7, 17, 13, 11, and 8 updates. Their mean/max distances when success was detected were
about 0.000020622/0.000082822 m. This small geometric experiment shows slower convergence,
not random-case failure or general sim-to-real robustness. The learner correctly explained
that repeated actual-position feedback can correct small nominal-Jacobian errors, while the
extra updates show reduced convergence efficiency. They also limited the evidence to this
link-length range, initial state, target, and geometric model. S9.2 is complete.
