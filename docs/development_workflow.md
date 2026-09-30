# Development workflow

1. Open Ubuntu 24.04 in WSL2.
2. `cd ~/projects/mujoco-robotics-playground`
3. `conda activate mujoco`; verify `echo $CONDA_DEFAULT_ENV` and `which python`.
4. Run `bash scripts/check_env.sh`. Stop on failure and resolve the reported issue.
5. Open VS Code with `code .` and select the `mujoco` interpreter.
6. Implement one small learning objective and document what it demonstrates.
7. Run the relevant example, initially `python examples/01_basic_simulation/main.py`.
8. Review `git status --short` and `git diff`; check newly created files too.
9. Stage intended files and commit changes after review.

Before each validation session, run `pwd`, `echo $CONDA_DEFAULT_ENV`, and
`which python`. Do not run Python or install packages if the environment is wrong.
Install dependencies only with `python -m pip install -r requirements.txt` in `mujoco`.
Report the commands, import/simulation results, and whether the optional viewer was tested.

2026-09-30：本人明确确认已完成解释器/依赖核对、headless 示例、GUI 操作与退出，以及
目录和 Git diff 学习，S0.1～S0.4 已在根 README 勾选。该确认记录学习完成度，不作为
本次会话重新运行这些命令的证据。
