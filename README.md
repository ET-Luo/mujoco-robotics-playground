# mujoco-robotics-playground

A personal learning and interview-preparation project for Robotics Software
Engineering, Embodied AI, Robot Learning, Robotic Manipulation, and Reinforcement
Learning for Robotics. Learn one concept at a time using the official MuJoCo API.
The basic simulation and UR5e inspection demo are implemented; subsequent stages
are learning placeholders.

For a new session, read [project experience and progress](docs/project_handoff.md)
after [AGENTS.md](AGENTS.md). It records validated results, GUI limitations,
practical commands, and the next small learning task.

## Environment and installation

Use Ubuntu 24.04 under WSL2, Miniconda, and VS Code connected to WSL2.
Python 3.11 is the target; no dedicated GPU or CUDA is required for simulation.
The optional viewer requires a working WSLg/OpenGL display.

From the repository root, create the environment once if it does not exist:

```bash
conda create -n mujoco python=3.11
conda activate mujoco
echo $CONDA_DEFAULT_ENV
which python
```

Continue only when the environment is `mujoco` and Python belongs to it:

```bash
python -m pip install -r requirements.txt
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
```

Dependencies are limited to `mujoco` for physics, `numpy` for numerical checks,
`matplotlib` for future plots, and `mujoco-menagerie` for official robot models.
Requirements are intentionally unpinned; this foundation does not lock exact versions.
Never install into `base` or system Python.

Activate `mujoco` again in each new terminal. Select that environment's interpreter
in VS Code. The check script diagnoses the current environment without activating it.

## First example

The default run advances a single passive hinge for 1,000 steps (2 simulated
seconds), prints dimensions and initial/final state, and needs no display.

```bash
python examples/01_basic_simulation/main.py --viewer --steps 2500
```

The optional viewer runs for approximately five seconds or until closed. Model
paths resolve relative to the script, so the example also works from other directories.
If the viewer fails, preserve the error and inspect `DISPLAY`, `WAYLAND_DISPLAY`,
and WSLg/OpenGL availability. A successful headless run does not validate graphics.

## Repository structure

```text
examples/
  01_basic_simulation/       # main.py, simple_model.xml, README.md
  02_ur5e_basics/            # official model loading and actuator/state inspection
  02_joint_control/         # documentation only below this point
  03_pd_control/
  04_forward_kinematics/
  05_jacobian/
  06_inverse_kinematics/
  07_cartesian_control/
  08_contact_and_grasping/
controllers/                # reserved controller package
environments/              # reach/, pick/, pick_place/
rl/                        # gymnasium/, ppo/, sac/
assets/                    # future shared model assets
scripts/                   # check_env.sh
notebooks/                 # future exploratory notes
docs/                      # roadmap, fundamentals, interviews, workflow
tests/                     # future validation guidance
AGENTS.md                  # persistent development rules
requirements.txt           # minimal dependencies
```

## Learning roadmap and progress

- [x] Repository foundation, environment checker, and basic simulation implementation
- [x] Load the official UR5e model
- [x] Inspect UR5e joints and actuators
- [x] Observe qpos, qvel, and ctrl on UR5e
- [ ] Phase 1: MuJoCo fundamentals — model, state, stepping, joints, and actuators
- [ ] Phase 2: Robotics fundamentals — control, kinematics, Jacobians, and IK
- [ ] Phase 3: Manipulation — end-effector motion, contacts, and grasping
- [ ] Phase 4: Robot learning — environment design, PPO, SAC, and sim-to-real

Implementation completion does not mark a learning phase mastered. Follow the
[learning roadmap](docs/learning_roadmap.md) and
[development workflow](docs/development_workflow.md). No controllers, task
environments, or learning algorithms are implemented yet.

Before joint control, run the [UR5e basics demo](examples/02_ur5e_basics/README.md):

```bash
python examples/02_ur5e_basics/main.py --headless
python examples/02_ur5e_basics/main.py --viewer
```

The first run downloads only UR5e assets into the Menagerie user cache. Headless
simulation and viewer operation are validated separately. The two `02_` folders
are intentional: study `02_ur5e_basics` before `02_joint_control`.
UR5e headless validation passed; the viewer run completed its physics steps but
aborted during shutdown. See the demo README for the exact error and versions.
