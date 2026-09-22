# Repository rules

## Session Startup and Handoff
- At the start of every new conversation, read this file, then read [project experience and progress](docs/project_handoff.md) before planning, editing, or running project code.
- Read the relevant example README and source next; consult [the roadmap](docs/learning_roadmap.md) for stage order.
- Check `pwd` and `git status --short` before editing. Preserve existing uncommitted work.
- Treat the handoff's versions and validation results as dated observations, not proof of the current environment. Recheck the environment before execution.
- Follow the user's current request; the handoff's suggested next task is context, not permission to implement later learning stages automatically.
- After meaningful changes or new findings, update `docs/project_handoff.md` with progress, commands/results, known issues, and the next small learning task. Distinguish verified results from hypotheses and untested advice.
- Keep the handoff concise and current; link to detailed example notes instead of accumulating a conversation transcript. Do not mark GUI operation or learning mastery complete based only on headless validation.

## Environment Rules
- This repository MUST be developed inside WSL2 Ubuntu 24.04.
- The required conda environment is `mujoco`.
- Before running Python, tests, scripts, or installing dependencies, check `echo $CONDA_DEFAULT_ENV` and `which python`. The environment must be `mujoco`, and Python must belong to that environment.
- If the environment is not active, stop and report the issue. The user can activate it with `conda activate mujoco` before work resumes.
- Never install into `base`, use system Python, or use `sudo pip`.
- Prefer `python -m pip install ...` over bare `pip`.

## Python Rules
- Use Python 3.11-compatible, simple, explicit code.
- Use the official `mujoco` package, never deprecated `mujoco-py`.
- Prefer NumPy for numerical operations and CPU-compatible code. Do not assume CUDA or an NVIDIA GPU.
- Avoid unnecessary frameworks and abstractions.

## MuJoCo Rules
- Use the official Python API directly and keep its concepts visible for learning.
- Add concise explanatory comments where useful for `MjModel`, `MjData`, `qpos`, `qvel`, `ctrl`, `mj_step`, Jacobians, contacts, and actuators.
- Do not introduce Isaac Sim or Isaac Lab.
- Do not introduce ROS, ROS2, robosuite, or reinforcement learning libraries unless explicitly requested later.

## Learning Philosophy
- Each example should teach one primary concept in small incremental steps.
- Do not automatically implement advanced algorithms in full.
- For PD control, Jacobian IK, Cartesian control, PPO, and SAC, explain the concept first, create a minimal skeleton only if needed, and leave clear TODOs for manual implementation.

## Dependency Rules
- Keep dependencies minimal; avoid overlapping libraries.
- Before adding a necessary dependency, explain why, add it to `requirements.txt`, and install it only inside `mujoco`.

## Validation Rules
Before considering a task complete:
1. Verify the directory with `pwd`.
2. Verify the environment with `echo $CONDA_DEFAULT_ENV`.
3. Verify the interpreter with `which python`.
4. Run the relevant Python script and check for import errors.
5. Report exactly what was tested.

If a GUI viewer cannot open, report the error and explain whether WSLg, OpenGL, display configuration, or MuJoCo is the likely cause. Do not silently ignore failures.

## Git Rules
- Do not delete existing user work unless explicitly asked.
- Keep changes scoped; do not create large generated files.
- Do not commit environment folders, `.conda`, `.venv`, `__pycache__`, build outputs, or generated logs.
- Summarize modified files at the end of each task.

## Documentation Rules
- Give every major learning stage a short README.
- Keep documentation concise, technical, and aligned with actual repository state.
- Use Markdown code blocks or Mermaid diagrams when useful.
