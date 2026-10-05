# Development workflow

1. Open Ubuntu 24.04 in WSL2.
2. `cd ~/projects/mujoco-robotics-playground`
3. `conda activate mujoco`; verify `echo $CONDA_DEFAULT_ENV` and `which python`.
4. Run `bash scripts/check_env.sh`. It checks the conda interpreter and MuJoCo only; inspect its failure before proceeding. Then check the current task's required dependency versions and actual imports according to [Dependency Rules](../AGENTS.md#dependency-rules), repairing missing declared packages inside `mujoco`.
5. Open VS Code with `code .` and select the `mujoco` interpreter.
6. Implement one small learning objective and document what it demonstrates.
7. Run the relevant example, initially `python examples/01_basic_simulation/main.py`.
8. Review `git status --short` and `git diff`; check newly created files too.
9. Stage intended files and commit changes after review.

Before each validation session, run `pwd`, `echo $CONDA_DEFAULT_ENV`, and
`which python`. Do not run Python or install packages if the environment is wrong.
Install dependencies only with `python -m pip` in the verified `mujoco` environment.
For a task, install the needed requirements using the exact declarations in
[requirements.txt](../requirements.txt); full `-r requirements.txt` installation is for requested
full-environment setup and also includes Torch/RL dependencies.
Report the commands, import/simulation results, and whether the optional viewer was tested.

## Switching laptops

Git synchronizes dependency declarations, not installed packages. On every computer,
verify its own WSL2/Ubuntu, conda interpreter, required distribution versions, and imports.
Previous handoff results describe their recorded machine/date; a missing package here does
not invalidate that history or the learner's completed work.

For example, if the current task needs `cv2`, its declared distribution is
`opencv-python-headless>=4.10,<5`. After environment verification, preview a selective repair:

```bash
python -m pip install --dry-run 'opencv-python-headless>=4.10,<5'
```

Inspect the plan before installing. If it changes a working core dependency, use temporary
constraints for those installed versions and resolve a compatible plan first. Once the plan
fits the rules, use the same requirements/constraints without `--dry-run`, then verify:

```bash
python -c 'import cv2; print(cv2.__version__); print(cv2.__file__)'
python -m pip check
```

Also check the distribution version against the declared range, and run the task's minimal
experiment; neither import success nor `pip check` alone establishes runtime correctness.
These commands are a workflow example, not fresh validation of the current computer.
The agent may carry out a needed declared-dependency repair automatically; a NumPy-only task
does not need OpenCV installed just for an optional comparison.

`requirements.txt` currently has version ranges and some unpinned packages, so this policy
ensures task readiness rather than identical package versions on every laptop. Resolver
preview behavior is documented in the [official pip install reference](https://pip.pypa.io/en/stable/cli/pip_install/).

2026-09-30：本人明确确认已完成解释器/依赖核对、headless 示例、GUI 操作与退出，以及
目录和 Git diff 学习，S0.1～S0.4 已在根 README 勾选。该确认记录学习完成度，不作为
本次会话重新运行这些命令的证据。
