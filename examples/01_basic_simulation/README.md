# Basic simulation

Goal: distinguish the compiled `MjModel` from changing `MjData` and observe
`mj_step` advance a passive hinge under gravity. There is no actuator or controller.

After activating `mujoco`, run from the repository root:

```bash
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
python examples/01_basic_simulation/main.py --viewer --steps 2500
```

Expect `nq = 1`, `nv = 1`, initial time zero, and default final time 2 seconds.
`qpos` is the hinge angle in radians and `qvel` its angular velocity in rad/s.
The capsule extends away from the hinge so gravity produces motion.
Closing the viewer early ends the simulation early. Viewer errors are reported
by MuJoCo/GLFW and may indicate WSLg, display, or OpenGL configuration issues.

TODO: change the number of steps and explain the resulting simulation time.
