# Learning roadmap

The basic simulation and UR5e basics demos are runnable. Work through each phase
manually and record observations; subsequent stages remain placeholders.

## Phase 1 — MuJoCo Fundamentals
- `MjModel`: compiled model configuration
- `MjData`: changing state and computed quantities
- `mj_step`: advance simulation time
- `qpos`: generalized positions
- `qvel`: generalized velocities
- `ctrl`: actuator control inputs
- body, joint, geom, site, actuator: scene and actuation building blocks

Follow this order:

1. `examples/01_basic_simulation`: explain state and simulation time on one hinge.
2. `examples/02_ur5e_basics`: load the official UR5e, inspect joint/actuator mappings,
   and observe one small target change using the model's existing position servos.
3. `examples/02_joint_control`: the next learning stage, still documentation only.

The UR5e demo adds no custom controller or advanced algorithm.

## Phase 2 — Robotics Fundamentals
- Joint-space control and PD control
- Forward kinematics and Jacobian
- Inverse kinematics and singularities
- Cartesian control

Use examples 02–07. Explain each concept before writing a small manual implementation.

## Phase 3 — Manipulation
- End-effector control and gripper
- Collision, contact, and grasping
- Reach, pick, and pick-and-place

Use example 08 and the future task environments. Define success before implementing tasks.

## Phase 4 — Robot Learning
- Gymnasium, observation space, and action space
- Reward design
- PPO and SAC
- Domain randomization and sim-to-real

The `rl/` directories are placeholders. Add learning dependencies only when explicitly requested.
