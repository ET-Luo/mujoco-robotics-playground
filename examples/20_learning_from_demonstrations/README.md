# Stage 19 — Learning from Demonstrations

当前实现[S19.1 episode contract](../../docs/22_1_episode_contract.md)、[S19.2 expert dataset](../../docs/22_2_expert_demonstrations.md)、[S19.3 split/normalization](../../docs/22_3_split_normalization.md)、[S19.4 CPU BC](../../docs/22_4_behavior_cloning.md)与[S19.5 closed-loop评价](../../docs/22_5_closed_loop_evaluation.md)。
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

2026-10-11 本人确认S19.2实验与预测完成，并准确回答五项Explain；Learning Mastered。
补充：超时episode仍可能含正确expert动作；bc_candidate是成功标记，不是逐动作质量判定。
本次只同步文档，未重跑仿真；下一小任务S19.3等待明确请求。


## S19.3 — Episode-level split / train-only normalization

只依赖requirements已有NumPy，无local helper、MuJoCo/Torch或新安装。
先核验上文mujoco环境；传入一份明确的S19.2 dataset目录：

```bash
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default --split-seed 194
```

如果本机没有tmp/s19_2/default，先采集S19.2，将打印目录代入--dataset。
输出新ignored tmp/s19_3/run_*：split.json、normalizer.npz、train/validation/test.npz。
默认success-only是明确选择，排除IDs保留；不是逐动作质量判定。--selection all仅显式预处理，不保证适合BC。
2026-10-11 DESKTOP-781D67A：193/194均16/4/4、1600/400/400 rows、正确episode交集0；
错误row split每对共享24episode；held-out X+100mm时train统计不变，错误all-data均值+33.333mm。
常量合成probe/独立保存统计与映射/roundtrip/hash/非法输入检查通过；3.11只验语法，未重跑physics/GUI。
Run/Modify/五问Explain见学习包；Engineering Complete + Learning Mastered（2026-10-11 本人确认实验、预测与五项Explain）；下一任务S19.4等待明确请求。
解释精度：仅train内部混合BC rows；反归一化使用保存的有效scale（恒定通道为1），不用原始零std。
本次仅同步文档，未重跑Python/仿真；runtime沿用2026-10-11工程验证。


## S19.4 — Tiny CPU Behavior Cloning

依赖requirements已有torch==2.14.0+cpu/numpy/mujoco/matplotlib；复用episode_contract→cartesian_spring。
先核验mujoco/interpreter；prepared是S19.3输出，不是S19.2原数据。

```bash
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193 --epochs 20
```

若本机没有该目录，运行S19.3并传入其输出路径。6→32→32→2 Tanh MLP，1346参数，CPU1thread，
train1600rows/val400rows，Adam lr.001/batch64，val选择checkpoint；test文件未加载。
输出新ignored tmp/s19_4/run_*：policy.pt（带normalizer/split hash）、results.json、loss CSV/PNG与12rollout NPZ。
2026-10-11 DESKTOP-781D67A：200/20epochs exit0；best79/20、val RMSE .734/1.063mm，constant8.002mm；
两设置expert4/4、constant0/4、BC0/4，工程PASS不代表机器人成功。训练wall约1.730/.167s（不含rollout）。
saved checkpoint、24rollout独立物理/动作/scorer、MSE梯度/CLI/hash/3.11语法与两PNG检查通过；无GUI/OOD/test评价。
完整学习包解释offline loss与闭环失败；Learning Mastered（2026-10-11 本人确认实验、预测与五项Explain）。下一任务S19.5等待明确请求。
解释补充：用有效action_scale反归一化，reference→阻抗force→ctrl=force/gear；BC闭环结果仍0/4。
本次仅同步状态文档，未重跑训练/仿真；runtime沿用2026-10-11工程验证。


## S19.5 — Frozen test closed-loop evaluation

复用behavior_cloning.rollout与相同expert/controller；依赖已有CPU Torch/NumPy/MuJoCo/Matplotlib，无安装。
先核验上文mujoco/interpreter，传入同一S19.3 split与S19.4 checkpoint：

```bash
python examples/20_learning_from_demonstrations/evaluate_rollout.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt
python examples/20_learning_from_demonstrations/evaluate_rollout.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --duration-scale 2
```

其他电脑按S19.2→3→4重建ignored输入，使用实际打印路径，不能混split/hash。
输出新ignored tmp/s19_5/run_*：results.json、offline_predictions.npz、12rollout NPZ、rollout.png。
2026-10-11 DESKTOP-781D67A：test400rows/4episode，BC RMSE1.532mm vs constant9.586mm；
2s/4s expert4/4、constant0/4、BC0/4，BC全分母final mean11.909/12.000mm，speed门限通过但位置未通过。
Modify只延长三个policy执行horizon100→200；offline数据不变，成功容差不变，属于不同duration协议。
24保存轨迹独立Euler/动作/scorer/hash与共同前缀检查通过；checkpoint-split mismatch/损坏输入/非法CLI拒绝。
两PNG已查看、3.11仅验语法；没有重训/normalizer refit/GUI/OOD/恢复实验。test已使用，不再当未见盲测调参。
学习Run/Modify/五问见学习包，当前仅工程完成；下一任务S19.6等待明确请求。
