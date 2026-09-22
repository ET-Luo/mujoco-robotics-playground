# MuJoCo notes

`MjModel` describes the compiled system; `MjData` stores its changing simulation state.
Generalized positions (`qpos`) and velocities (`qvel`) can have different dimensions;
the first example's single hinge gives one of each. `ctrl` contains actuator inputs;
the first example has no actuators.

TODO: record how `data.time` relates to step count and `model.opt.timestep`.

## Foundation validation (2026-09-22)

- Ubuntu 24.04 under WSL2; active conda environment: `mujoco`.
- Existing interpreter: Python 3.12.14; MuJoCo: 3.14.0. Python 3.11 remains
  the code compatibility target but was not available for this validation.
- `bash scripts/check_env.sh`: PASS; MuJoCo imported successfully.
- `python examples/01_basic_simulation/main.py`: PASS; `nq = nv = 1`,
  finite final state, and final time 2.000 seconds.
- `timeout 20s python examples/01_basic_simulation/main.py --viewer --steps 1000`:
  simulation reached 2.000 seconds, then reported `GLXBadDrawable` in
  `X_GLXSwapBuffers`; the process timed out (exit 124). Viewer success is not confirmed.
- `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, and WSLg 1.0.66 are present.
  The failure points to the GLX/WSLg graphics or viewer shutdown path, rather
  than physics or model loading. The exact cause is unconfirmed; `glxinfo`
  is unavailable. No graphics packages or overrides were introduced.
