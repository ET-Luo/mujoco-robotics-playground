# Stage 19 — Learning from Demonstrations

当前实现[S19.1 episode contract](../../docs/22_1_episode_contract.md)与[S19.2 expert dataset](../../docs/22_2_expert_demonstrations.md)。
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

2026-10-11 本人确认实验与预测完成，并准确回答五项Explain；S19.1 Learning Mastered。
本次仅同步文档，未重跑Python或仿真；工程runtime沿用2026-10-10记录。


## S19.2 — Expert demonstration / trajectory dataset

```bash
python examples/20_learning_from_demonstrations/generate_demonstrations.py
python examples/20_learning_from_demonstrations/generate_demonstrations.py --horizon 20
```

先按上文核验mujoco环境。默认24条100-action episode、seed1900…1923；world XY start/goal采样，
absolute reference与旧expert/低层controller不变。Modify只缩短horizon100→20。
每次打印新的ignored tmp/s19_2/run_<UTC timestamp>/，含24NPZ、manifest.json、dataset.png。
显式--output要求目录不存在，避免覆盖/混用旧数据；失败仍保存，bc_candidate只标记不筛选。
2026-10-11 DESKTOP-781D67A：默认24/24、2400transitions；短horizon0/24、480transitions。
重复数组完全一致/短组前缀一致，72保存episode独立动力学重算与hash/clock/outcome通过；两PNG已查看。
旧S19.1默认回归exit0；CLI非法参数/已有目录拒绝exit2。3.11仅语法验证，无GUI。
详细结果、五项Explain及边界见学习包；split/normalization/BC留到后续任务。
