# Stage 22 — Contact-Aware Robot Learning

当前实现S22.1观测合同、S22.2示范记录器与S22.3 contact BC；后续子任务仍为规划。
[学习包](../../docs/25_1_observation_contract.md) · [Engineering / Learning状态唯一来源](../../README.md#stage-22--contact-aware-robot-learning)。

依赖已有mujoco/numpy/matplotlib，无local helper或新依赖。两slide球指端接触平面，headless。
具名pair的world接触合力是仿真真值proxy，touch是pad内法向力标量[N]，非真实触觉图像。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/observation_contract.py --output tmp/s22_1/my_run
python examples/23_contact_aware_learning/observation_contract.py --force-noise .2 --output tmp/s22_1/my_noise02
```

output必须为新目录；CSV/NPZ/JSON/PNG仅ignored tmp。训练两轨迹只用于拟合normalizer，未训练policy。
Modify只给观测力通道加噪，物理轨迹不变；四种通道配置维度6/9/7/10。
2026-10-12 Zero默认与noise .2通过，同状态paired法向力1.355968/0N，validation峰值7.335705N；
独立时间/normalization/noise/ablation核对通过。无GUI、硬件或学习策略表现验证。
2026-10-12 本人确认实验与预测完成，五项Explain通过；S22.1 Learning Mastered，不自动进入S22.2。

## S22.2 — Demonstration recorder

[学习包](../../docs/25_2_demonstration_recorder.md) · [代码](demonstration_recorder.py)。
复用S18.2 map_sample（helpers scaling_clutch→incremental_reference）与S22.1模型/观测；依赖仍为mujoco/numpy/matplotlib。
全部source=scripted_master，human_demonstrations=0；无键盘/设备/GUI或新依赖。
新的10维task observation v2用固定任务goal，2维reference action单独保存；不把标签放进输入。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/demonstration_recorder.py --output tmp/s22_2/my_run
python examples/23_contact_aware_learning/demonstration_recorder.py --replay tmp/s22_2/my_run --output tmp/s22_2/my_replay
python examples/23_contact_aware_learning/demonstration_recorder.py --scale .5 --output tmp/s22_2/my_scale05
```

输出目录须不存在；NPZ/JSON/CSV/manifest/PNG只ignored tmp。默认三条370action/371observation，50Hz记录/1kHz物理。
2026-10-12 Zero最终默认2/3 quality eligible（低面失败仍保留），sample峰touch1.122636N；单独reload/action replay全部10通道误差0。
Modify scale .5为0/3 eligible但记录回放通过；short horizon100为100/101，三条仍保存。
未分split或归一化，未训练policy；采样quality不是1kHz全过程安全证明，回放不证明真人数据或可学习性。
Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），状态见根README。不自动进入S22.3。

## S22.3 — Contact-aware BC / channel ablation

[学习包](../../docs/25_3_contact_bc.md) · [contact_task.py](contact_task.py) · [contact_bc.py](contact_bc.py)。
新增surface-following反馈专家；原S22.1/2文件不改。新10维schema包含x_goal[m]/F_target[N]，
2维action为actual-q锚定的reference delta[m]；经相同有界position servo执行。
依赖已有mujoco/numpy/matplotlib/torch==2.14.0+cpu，helper contact_task→observation_contract，无新库。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/contact_bc.py --output tmp/s22_3/my_run
python examples/23_contact_aware_learning/contact_bc.py --models-from tmp/s22_3/my_run/cohort --contact-input mean --output tmp/s22_3/my_mean
```

2026-10-12 Zero：40expert全部成功，32train/8val、4800/1200行；16/4reset groups、方向配对隔离，test4组保留未用。
四模式各三training seeds：state/force/touch/full validation成功24/20/24/23（各24trial）；
冻结mean输入为24/13/11/12，expert均8/8。增加通道不保证成功率提高，触觉与法向力在本例冗余。
50Hz policy/1kHz physics/3s；每trial150/151、3001physics snapshots，全程接触/超力失败锁存，末250ms稳定条件。
统一solver tolerance1e−12后完整重跑通过；两组208trial独立输出/physics/scorer核验与26独立action replay通过。
产物仅ignored tmp，output必须新目录；无test/GUI/真人、OOD或硬件验证。Engineering Complete，Learning Run/Modify完成、Explain待补第①⑤项；不自动进入S22.4。
