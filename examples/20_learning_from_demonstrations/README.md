# Stage 19 — Learning from Demonstrations

当前只实现[S19.1 episode contract](../../docs/22_1_episode_contract.md)。
状态唯一来源：[根README](../../README.md#stage-19--learning-from-demonstrations)。
复用Stage17 cartesian_spring XML/常量；helper也导入Matplotlib。
依赖requirements已声明mujoco/numpy/matplotlib；没有Torch或新依赖。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/episode_contract.py
python examples/20_learning_from_demonstrations/episode_contract.py --goal-x -.01
```

每命令3个闭环expert episode，每个T100/2s；fresh MjData重放校验。
NPZ observations(101,6)、actions(100,2)、time_s(101,)、terminated/truncated(100,)；
action是world XY绝对位置reference，50Hz发reference、1kHz实际physics与反馈。
数据/manifest只ignored tmp/s19_1_goal*。2026-10-10两命令exit0、均3/3合格、replay误差0。
CLI goal必须finite且±.03m内。没有train split/normalizer/BC/GUI/抓取；后续Task等待授权。
Run/Modify/五问Explain见学习包。
