# UR5e basics

## Learning goals

- Load the official Universal Robots UR5e robot model.
- Understand `nq`, `nv`, and `nu` and inspect joint and actuator names.
- Observe `qpos`, `qvel`, and `ctrl` before and after simulation.
- Distinguish the robot model from its simulation state.

## Run

From the repository root:

```bash
conda activate mujoco
echo $CONDA_DEFAULT_ENV
which python
# Continue only with the mujoco environment and its Python interpreter.
python -m pip install -r requirements.txt
bash scripts/check_env.sh
python examples/02_ur5e_basics/main.py --headless
python examples/02_ur5e_basics/main.py --viewer
```

Headless is the default. Both modes run 1,000 steps; use `--steps 2500` for a
longer run. The viewer closes after the steps complete, or can be closed early.
The official `mujoco_menagerie.load("universal_robots_ur5e")` API loads the scene
with its floor and robot. First use needs internet access to download this model;
the package caches assets under `~/.cache/mujoco_menagerie` by default, outside
this repository. No full Menagerie checkout or copied meshes are needed.

## Key concepts

| Concept | Meaning |
| --- | --- |
| `MjModel` | Compiled scene, robot structure, and physics/actuator settings |
| `MjData` | Mutable simulation state and computed quantities |
| `nq` / `qpos` | Number of generalized position coordinates / their values |
| `nv` / `qvel` | Number of generalized velocity coordinates / their values |
| `nu` / `ctrl` | Number of actuator commands / commanded values |
| Actuator | Model-defined mechanism that turns a command into force |

UR5e has six hinge joints, so positions are in radians and velocities in rad/s.
Here `nq = nv = nu = 6`; these dimensions need not match for other models.
`ctrl` means a target angle only after checking this model's actuator configuration.

The script prints every actuator-to-joint mapping and checks the compiled gain,
bias, dynamics, gear, and control limits. It initializes from the model's `home`
keyframe, then changes one verified position-servo target by at most 0.05 rad,
within both actuator and joint bounds. Other commands keep their home values.
The model already provides the servo behavior; no custom controller is implemented.
Other joints can still move due to gravity and coupled dynamics, and measured
`qpos` need not exactly equal commanded `ctrl`.

Validate headless physics first. A viewer error such as `GLXBadDrawable` is a
separate WSLg/OpenGL issue to investigate; preserve the exact error without
assuming that model loading or headless simulation failed.

Sources: [official package API](https://github.com/google-deepmind/mujoco_menagerie/blob/main/python/DOC.md)
and [official UR5e model](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e).

Next exercise: map each printed actuator to its joint, then repeat a single small
target change and compare the command with the observed joint angle. Leave PD,
IK, Cartesian control, planning, and RL for later stages.

## Validation (2026-09-22)

- Environment: WSL2 Ubuntu 24.04, conda `mujoco`, Python 3.12.14,
  MuJoCo 3.14.0, `mujoco-menagerie` 2026.9.1. Code targets Python 3.11
  compatibility; that interpreter version was not tested.
- Model asset revision: `2a3464301404aba712e25a68701afeeff4ddedf8`.
- `pwd`, `echo $CONDA_DEFAULT_ENV`, `which python`, `python --version`, and
  `bash scripts/check_env.sh` confirmed the intended environment and successful import.
- `python examples/02_ur5e_basics/main.py --headless` exited successfully:
  `nq/nv/nu = 6/6/6`, 8 bodies including world, 6 joints, and 6 actuators.
  All names and state arrays printed. After 1,000 steps, time was 2.000 s and
  state values were finite. The shoulder-pan target changed from -1.5708 to
  -1.5208 rad; its measured angle changed by 0.049998 rad.
- Separate GUI command:
  `timeout --kill-after=3s 20s python -u examples/02_ur5e_basics/main.py --viewer`.
  All simulation steps and final prints completed, but the process exited 134
  during shutdown with:

```text
Fatal glibc error: pthread_mutex_lock.c:450 (__pthread_mutex_lock_full): assertion failed: e != ESRCH || !robust
timeout: the monitored command dumped core
```

This differs from the earlier basic demo's `GLXBadDrawable`. The observed failure
is in the native GUI/thread teardown path; its exact cause is unconfirmed.
Clean viewer operation is not validated. No GUI workarounds were added.
