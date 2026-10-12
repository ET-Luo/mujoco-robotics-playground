# Stage 22 — Contact-Aware Robot Learning

当前实现S22.1观测合同、S22.2示范记录器、S22.3 contact BC、S22.4同任务ACT/Diffusion比较与S22.5三指专家可行性；后续子任务仍为规划。
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
产物仅ignored tmp，output必须新目录；无test/GUI/真人、OOD或硬件验证。Engineering Complete + Learning Mastered（2026-10-12 本人确认）；不自动进入S22.4。

## S22.4 — Contact ACT / Diffusion comparison

[学习包](../../docs/25_4_contact_policy_comparison.md) · [比较代码](contact_policy_comparison.py) · [模型代码](contact_sequence.py)。
同C轨道full10维观测与actual-q锚定delta动作，九模型BC/真实Transformer ACT/H10 epsilon Diffusion各三training seeds。
依赖已有mujoco/numpy/matplotlib/torch==2.14.0+cpu；helpers含S22.3数据/任务、S20.5 ACT及其导入链与noise_schedule，无新库，旧课程源码不改。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/contact_policy_comparison.py --output tmp/s22_4/my_run
python examples/23_contact_aware_learning/contact_policy_comparison.py --models-from tmp/s22_4/my_run/cohort --pulse-duration .1 --output tmp/s22_4/my_short
```

output须新目录；默认采集同40专家，训练九模型；模型恢复只读原cohort及其父data，全部产物只ignored tmp。
2026-10-12 Zero严格评价nominal expert8/8、BC23/24、ACT E1/E5各9/24、Diff E1/E5为0/48与16/48；test4组未用。
默认1s geometry pulse持续.2s：loss−15mm、overforce+30mm；所有fault任务失败锁存，但负载恢复与末段到达分别报告。
Modify.1s nominal逐值不变，Diff E5 loss恢复37→41/48，任务成功仍0；不宣称缩短pulse普遍改善表现。
训练采集solver1e−12/100，全部评价统一1e−14/200；九checkpoint不变，保留训练provenance并单独记录evaluation driver版本。
两组1056trial独立physics/scorer/plan检查、96stored-reference重积分与144DDIM plan逐步NumPy核对通过；PNG已查看。
无GUI/test/真人、真实移动表面或硬件验证。Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），不自动进入S22.5。

## S22.5 — Dexterous expert feasibility

[学习包](../../docs/25_5_dexterous_feasibility.md) · [入口](dexterous_feasibility.py)。
依赖mujoco/numpy/matplotlib；local helpers为S18.11 grasp_stability→multi_contact_grasp→hand_fixture及hand.xml。
复用原1kHz接触门控专家／模型／scorer，旧源码不改，无新包、policy训练或test。
28维sim proxy与6维absolute position reference[rad]，每trial4000 action/4001 observation。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/dexterous_feasibility.py --output tmp/s22_5/my_run
python examples/23_contact_aware_learning/dexterous_feasibility.py --pulse-scale 4 --output tmp/s22_5/my_strong
```

output须新目录，24个全部attempt保存NPZ、JSON及正常摩擦PNG，仅ignored tmp。
专家带接触计数／冻结目标记忆；当前obs不包含全部内部状态；不能把1kHz轨迹抽样冒充50Hz部署。
无GUI、真人、真实sensor、主动重抓、split或独立test，学习状态见根README。

2026-10-12 DESKTOP-781D67A：Engineering Complete；默认正常三指8/8、四倍4/8，全部计划8/24→4/24，低摩擦/缺指均未ready。
两组48trial合同／独立scorer／pulse审计、六条保存动作重积分通过，PNG已查看。
注意normal在当前reference写入后求解，仅审计proxy；S22.6须重整先观测后决策的数据时序。Learning Run/Modify完成，Explain待第1–2项补正。
