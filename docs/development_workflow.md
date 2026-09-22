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
