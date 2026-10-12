# S22.2 — Scripted teleoperation demonstrations / recorder

[状态唯一来源](../README.md#stage-22--contact-aware-robot-learning) · [代码](../examples/23_contact_aware_learning/demonstration_recorder.py) · [阶段导航](25_contact_aware_robot_learning.md)
前置S22.1；0.5～2小时。Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）。

## Problem → Why → Intuition

有接触日志并不等于有训练数据：需要明确“当时看到了什么”“随后命令了什么”“执行后发生了什么”。
本节将已有teleoperation mapper与两slide指端模型连接，保存有时间与来源的episode，随后重新执行动作检查记录正确性。
直觉：录像回看与重做实验不同。回放必须让MuJoCo再次积分，不能直接把记录的qpos赋给机器人伪造重现。

输入全部为**scripted master**，真人示范数量0。复用的是S18.2 `scaling_clutch.map_sample`，
以及S22.1的model、position servos与接触观测；没有调用旧S18.5的整段run，也没有接键盘、GUI或真实设备。
阶段规划中的最小键盘采集仍是另一个待授权子任务，本节不假装已经具备真人示范。

## Core Concepts / contract

| 字段 | shape / 单位 / 含义 |
| --- | --- |
| observations | (N+1,10)，原始SI值；q(2)[m]、qdot(2)[m/s]、task goal(2)[m]、F_W(3)[N]、touch(1)[N] |
| actions | (N,2)，绝对slide reference [m]，顺序qx/qz；通过position servo执行 |
| times | (N+1,)，simulation秒；o_t/action_t边界间隔20ms |
| held_ctrl | (N+1,2)，观测时正在保持的上一reference [m]，诊断信息，不属于10维policy输入 |
| master_M | (N+1,3)，脚本master坐标[m]；初始sample加N个被消费的sample |
| phase / engaged / rebase | (N,)，脚本阶段、clutch状态、重新锚定事件；只作来源审计，不输入policy |
| limited | (N,2)，reference速度/工作空间限幅标志 |
| metadata | source、初始q/qdot/ctrl、goal、表面高度、scale、时钟、source SHA、runtime版本、停止原因与quality |
| manifest | 三条episode的文件名/SHA/source/quality，所有attempt保留，无随机行split |

### 重要的schema调整

S22.1 `tip_contact_observation_v1`的goal是即将下发的reference，适合观察实验；
若直接把这个goal作为输入、同时把相同reference作为BC标签，就把答案泄漏到了输入。
本节使用`tip_contact_task_observation_v2`，仍为10维，但goal改为**固定任务上下文**：
`[direction*0.01, 0]`，表示扫描方向对应的名义x waypoint，以及释放后回到qz=0的高度。
它不是每tick示范action，也不是当前脚本终点的完整位置成功判据；脚本slide-back后x不会回到waypoint。
S22.1原文件与历史结果保留，loader拒绝旧schema混用。

F_W仍是仿真具名pair的privileged contact真值；touch仍是pad区域法向力和，不是硬件触觉图像。
高度/phase/master/rebase是teacher或环境配置，不能直接偷渡进后续policy observation。
十维观测不保证完整Markov性，脚本内部阶段与reference记忆也可能造成监督歧义；本节不证明可学习性。

## Mathematics / frame / timing

```text
delta_reference = scale * R_WM @ (master_k - anchor_previous)  # m
norm(delta_reference) <= speed_cap * master_dt                # 0.02 m/s * 0.02 s
reference_new = project_box(reference_old + limited_delta)
a_t = reference_new[[world_x, world_z]]                       # absolute slide coordinates [m]
(o_t, a_t, o_(t+1))                                           # one causal transition
held_ctrl[0] = initial_ctrl; held_ctrl[t+1] = a_t
```

reference采用world轴方向、以初始球心为原点的位移坐标；因此reference z=qz，球心world z=0.04+qz。
旧mapper的R_WM是绕z轴旋转90°，master+y对应world−x，master+z对应world+z。
脚本先用R_WM.T把期望world速度转换为master速度，再让原mapper映射回去，不修改原mapper的坐标合同。
参考工作空间x∈[−25,25]mm，z∈[−42,0]mm，未用y维锁定为0；这些是reference限幅，不是实际qpos硬限位。
执行器限力±10N，由已有position servo做局部反馈。reference不等于下一qpos，也不等于接触力目标。

clutch off与rebase均保持reference，并消费当前master sample更新anchor。
4.2s合成设备坐标跳变与clutch重新接合发生在同一个sample，rebase消除坐标重置引发的reference跳变。
保持reference时robot仍继续积分和局部反馈，不是物理停止或安全制动。
本节所有sample即时到达；没有网络lease、丢包、延迟或恢复协议验证。

## Math-to-Code / visible APIs

`collect`：构建model/data → 固定task goal → 记录初始o_0 → 每tick mapper生成action → 保存action → 积分 → 记录下一观测。
`map_sample`返回新reference、新anchor和限幅诊断，不改变输入数组；复用S18.1的增量、范数限速与box投影。
`MjModel.from_xml_string`由已有build_model编译并返回物理模型；`MjData(model)`创建可变qpos/qvel/ctrl/time。
`data.ctrl[:]=action`给两个position actuator下发米制位置reference，随后20次`mj_step`原地积分qpos/qvel/time，1kHz物理。
`observe`内`mj_forward`刷新同一当前状态的接触和sensor，不推进时间；`mj_contactForce`原地写入局部wrench，转换成world合力。
每个观测描述当前state与**上一interval的ctrl**；随后生成并执行a_t，不把执行后的力填回o_t。
本节将policy/mapper边界设为50Hz，以匹配既有mapper；S22.1独立观测实验为100Hz，两者不混用时钟。

`save_episode`保存NPZ/JSON、N行transition-start CSV；terminal o_N只在NPZ，不能伪造它的action。
`load_episode`用allow_pickle=False读取NPZ，核对文件SHA、source版本、schema、shape、time、held_ctrl和quality。
`validate`检查reference bounds/speed、clutch保持与固定goal。source SHA是版本/完整性检查，不是数据真实性证明。
`replay`仅恢复初始qpos/qvel/ctrl，逐个执行已记录action，检查全部观测与时间；不使用记录中间qpos推进物理。
runtime记录Python/MuJoCo/NumPy版本；source严格一致，库或机器变化仍可能产生数值差异，回放断言不会被忽略。
依赖requirements已有mujoco/numpy/matplotlib；helpers为observation_contract、scaling_clutch→incremental_reference；无新依赖。

## Minimal Experiment / expected result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/demonstration_recorder.py --output tmp/s22_2/my_run
python examples/23_contact_aware_learning/demonstration_recorder.py \
  --replay tmp/s22_2/my_run --output tmp/s22_2/my_replay
```

output须为新目录。每条全长7.4s，370action/371observation；收集接近→压紧→正/负向滑动→clutch reposition/rebase→返回滑动→退离→settle。
默认三条：scan_positive、scan_negative（水平面z=0），missing_surface（表面z=−0.1m，reference相同但碰不到）。
输出各episode NPZ/JSON/CSV、manifest.json、results.json、demonstrations.png，仅ignored tmp，无GUI需要。
预期前两条建立接触，低面失败；三条都完整保存并通过action replay。回放通过与示范合格是两个独立结论。

## Quality / expected vs actual

质量门槛仅用于数据审计：protocol完整370步、任一sample touch>0.2N、scan transition-end接触比例≥0.9、
末10观测touch≈0/|qz|<1mm/|vz|<5mm/s、采样touch峰<15N，全部满足才demonstration_eligible。
没有scan时fraction=null，不填0伪造测量。所有episode终止于采集预算：terminated=False/truncated=True；
这不等于失败，也不等于任务成功。没有接触或质量不合格的episode仍保存，不静默删除。
这些是**50Hz采样质量门槛**，不是1kHz全过程安全scorer；可能漏掉步间短暂接触丢失或峰值，不能声称全过程超力检查通过。

2026-10-12，Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；mujoco所属Python3.12.14；
MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2 metadata、真实import路径与API核验通过，无安装。
最终现场命令：上述脚本分别以`--output tmp/s22_2/verified`、`--scale .5 --output tmp/s22_2/scale05`、
`--replay tmp/s22_2/verified --output tmp/s22_2/replayed`、`--horizon 100 --output tmp/s22_2/short100`运行，均exit0。

| 实验 | 结果 |
| --- | --- |
| 默认3条全长 | 2/3 eligible；正/负扫描contact fraction均1，峰touch均1.122636N，release通过；低面无接触 |
| 默认逐action回放 + 单独reload replay | 三条10通道max abs error全部0；每条370步/7.4sim秒 |
| scale=.5 | 0/3 eligible，三条都无>0.2N接触；仍保存370/371且replay通过 |
| horizon=100 | 100action/101observation，2s；0/3 eligible，无scan故fraction=null；结束原因short budget |
| 独立检查 | 9条采集episode逐sample NumPy重算mapper/限速/box/clutch/rebase，CSV o_t/action对齐、固定goal、质量统计通过 |
| 失败注入 | shifted action、缺terminal、错时间、错schema、tampered NPZ SHA均拒绝 |
| 其他 | NaN scale CLI exit2、3.11语法通过（actual3.12），默认图已查看，local文件链接与git diff --check通过 |

scale=.5时同样master运动仅生成约−18mm的最深reference；球心约22mm，高于10mm接触边界，因而失去接触。
减小scale让映射更精细，但在固定脚本/时间预算内未必还能完成任务。eligible降低不是recorder故障。
短预算峰touch约0.108N，低于0.2N门槛；contact_seen=False不意味着几何上绝无contact。
当前三条数据只是小实验，无train/val/test split或normalizer，也不够证明泛化。
后续split须按episode/reset来源分组，镜像/scale/同reset变体不能随机按行混入不同集合；先采集独立reset组，再拟合train统计。

## Failure cases / Robotics context

把action放进goal输入会出现虚假的低BC loss；把执行后的force放进o_t会产生未来泄漏。
只记录N观测会丢terminal转移；把qpos复制回去的“回放”跳过执行器和动力学。
只保留eligible数据会隐藏不接触失败；同时成功率分母需按任务另定义，不能直接把2/3质量当学习策略成功率。
scripted_master不等于真人expert：动作没有人的调整、疲劳和反应延迟，也没有force-feedback设备。
示范者不读取接触反馈调整动作；遇到低面会失败。当前数据支持记录/审计练习，尚不是可靠contact-aware专家。
机器人应用是遥操作抓取/插装/表面扫描的数据准备；真实采集还需设备时间戳、输入权限、sensor标定与可用观测约定。

## Interview Capsule / Must Remember

**30秒：** 接触示范记录器保存N个reference动作和N+1个同步观测，写清单位、时钟、来源和初始状态。
我复用incremental teleoperation mapper与有界低层执行器，保留失败示范，再重新执行动作验证数据，不用复制qpos伪造回放。

**2分钟：** 说明10维task observation与2维绝对reference标签的区别，固定goal避免标签泄漏；解释50Hz mapper和1kHz servo/physics。
clutch与rebase保持reference并消费anchor，不能当物理停机。保存source、版本、SHA、terminal观测与预算停止原因。
默认两条接触、一条低面失败；scale减半后全部无接触，但记录回放仍正确。质量门槛是采样审计，非完整安全/泛化指标。
真人数量为0；后续需要独立reset组、episode级split和train-only归一化，不能把旧teleop日志或这些小样本直接当成熟训练集。

Must Remember：o_t先于a_t执行；reference≠actual qpos；任务goal≠teacher action；replay正确≠示范合格≠policy成功；脚本输入≠真人示范。

## My Verification / Run → Modify → Explain

状态仅根README记录；2026-10-12 本人确认实验与预测完成，五项Explain通过，Learning Mastered。
精度补充：held_ctrl是观测时保持的上一position reference[m]，不是实际输出力；truncated表示预算结束，完整370步同样为True。
失败数据可用于审计与后续有明确目标的恢复学习，但不自动是应模仿的正确action标签；split还须按共享reset/来源组隔离相关episode。

**Run：** 执行上述my_run与my_replay，检查manifest有三条、包括missing_surface失败；确认每条370/371，打开demonstrations.png看clutch区间。

**Modify：** 先预测最深reference与接触结果，再只改变scale：

```bash
python examples/23_contact_aware_learning/demonstration_recorder.py \
  --scale .5 --output tmp/s22_2/my_scale05
```

对比eligible、peak_touch_N与reference曲线；解释为什么clutch保持性质和回放准确性仍成立。

**Explain（五问）：**

1. 为什么N个动作必须保存N+1个观测？CSV为何只有N行，NPZ里的最后一观测是什么？
2. 本节固定task goal、teacher action、held_ctrl各表示什么？为什么不能沿用S22.1的goal作为BC输入？
3. clutch off与rebase怎样处理reference和anchor？为什么reference不动不代表机器人不动？
4. 怎样区分真实动作回放和直接复制qpos？回放误差0能证明什么、不能证明什么？
5. 为什么missing_surface和scale=.5失败仍保存？真人数据有多少，quality与truncated分别说明什么，后续应怎样split？

下一小任务S22.3 contact-aware BC与通道ablation，等待本人明确请求；不自动训练policy或开展键盘采集。
