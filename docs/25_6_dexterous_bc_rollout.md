# S22.6 — Dexterous learned policy rollout

[根README状态](../README.md#stage-22--contact-aware-robot-learning) · [任务与物理循环](../examples/23_contact_aware_learning/dexterous_task.py) · [BC入口](../examples/23_contact_aware_learning/dexterous_bc.py)。阅读、运行、修改及解释约0.5～2小时。

## Problem → Why → Intuition

S22.5证明固定三指专家可保持free object，但那份1kHz审计记录不能直接当BC数据：
接触力在当前动作写入后求解，且专家依赖冻结参考等记忆。
本课先建立因果采集循环，再学一个小BC，最后让模型实际驱动六个servo与自由物体。

直觉：离线一次预测很准，仍可能在闭环中慢慢改变HOLD参考，改变接触力，再把状态推离专家分布。
每20ms重预测一次，使微小误差通过动力学影响下次输入。动作限幅限制指令，但不会保证抓持成功。

## Core Concepts：任务边界与因果时序

复用S18.11三指模型、具名映射、object_contacts和评分阈值，不修改旧源码。
新任务`three_finger_causal_hold_v2`：50Hz专家／BC，1kHz物理及评分，4s窗口。
不能把结果直接归为旧1kHz专家结果；重新验证专家是本课实验的一部分。
MuJoCo solver统一tolerance1e−14、iterations200，Newton断言仍<1e−6N。

每个physics step：

1. 根据先前monitor状态施加当前world COM载荷／pulse，保留上一ctrl；`mj_forward`刷新当前状态下的力。
2. 用此**pre-action** normal更新连续接触gate、phase。
3. 每20步构造观测，推理新action，box＋rate限制，然后写入具名`ctrl`。
4. `mj_forward`重算新ctrl下的力，供physics评分；这些post-action力不进入当前policy输入。
5. 记录当前qpos/qvel/qacc/ctrl，再`mj_step`积分1ms；终点仅观测、不再输出动作。

真实传感器会读过去执行造成的力，本例仍是同state下仿真force solve proxy，未模拟传感器带宽／延迟。
环境新pulse可影响当前力，它已发生、可被当前观测看到；未来pulse方向／幅值不是输入。

### 38维观测与6维动作

| 输入段 | shape | 单位／frame／来源 |
| --- | --- | --- |
| finger q / qdot | 6+6 | rad / rad·s⁻¹，具名关节 |
| object xyz / quaternion | 3+4 | m，world；wxyz单位四元数，仿真proxy |
| object omega / velocity | 3+3 | rad/s、m/s，world，仿真proxy |
| normal per finger | 3 | N，上一ctrl下的具名物体接触法向力和，仿真proxy |
| previous executed reference | 6 | rad，控制器自己知道的上一条已限幅ctrl |
| elapsed time / phase one-hot | 1+3 | s；CLOSE/HOLD/FAULT，由在线接触monitor计算 |

`o=(38,)`，网络输出`raw=(6,)`，经限幅的`action=(6,)`是absolute reference[rad]，不是C轨道delta[m]或torque。
previous_ctrl不是当前标签，是已经执行的过去动作；它提供冻结参考的记忆。
phase由所有controller共享的接触计数monitor产生，非导入expert轨迹的oracle标签。
专家动作在CLOSE为`CLOSED*min(t,1)`；HOLD/FAULT为previous_ctrl。
BC始终独立预测六个参考；monitor不在HOLD时替BC冻结动作，也不提供重抓恢复。
输入足以表达该专家动作规则，但不宣称是包含全部接触历史／传感器内部状态的通用Markov状态。

全局gravity=0，mass=.03kg、friction=.7；2s后只对HOLD物体施加world向下.1N显式外载荷。
初始关节角U[0,.003]rad、物体xyz偏移各U[−.3,.3]mm、yaw U[−.03,.03]rad；都是同一小reset域。
不改变mass/friction，不完成S22.7；不扫noise/latency，不完成S22.8。

## Mathematics → Math-to-Code

归一化：`z=(o−mean_train)/scale_train`；两层64宽Tanh MLP，38→64→64→6。
动作同样由train行归一化，网络输出反归一化得到rad参考。
`loss=mean((pred_normalized−target_normalized)^2)`，对样本／6个关节取均值。
Adam lr=.001、batch128、CPU1thread、120epoch，training seeds226/227/228。
按validation **offline MSE**各自选择checkpoint；闭环结果不用于挑seed或调参。

限幅：先`box=clip(raw,0,[.9,1.2]×3)`，再
`action=previous_ctrl+clip(box−previous_ctrl,−.02,.02)`。
.02rad/20ms相当于参考变化每分量≤1rad/s，区别于实际关节速度限值。
MuJoCo position servo：`τ=clip(.25(action−q)−.02*qdot,−.08,.08)`[N·m]。

每1ms锁存S22.5同阈值的接触丢失、漂移、旋转、slip、非手指支撑、向下drop；2s锚定pose，
2.5s ready须HOLD且此前无失败；末250ms线／角速度恢复；ready不推出success。
评分是事后评价，失败后仍跑满4s，不是安全制动。`success=ready and 无全过程失败 and tail恢复`。

### API说明

`build_model`返回编译后的`MjModel`与μ，`MjData(model)`新建状态buffer；nq13/nv12/nu6。
`mj_forward(model,data)`无状态返回值，原地更新力／加速度等且不推进时间；两次调用分别用于因果观测与新动作评分。
`mj_step(model,data)`原地推进实际qpos/qvel/time；仅reset写qpos，rollout没有复制专家state。
`mj_objectVelocity(...,out6,0)`写入world[angular,linear]，区别于`xfrc_applied`的world[force,torque]。
`object_contacts`内部`mj_contactForce(...,out6)`写contact-frame wrench，经frame转world且处理物体受力符号；
`mj_jac`写world接触点Jacobian，用于物体／手指接触处相对速度及切向slip。
`mapping`分别解析joint qpos/dof与actuator id，XML乱序使简单ctrl切片无法保证具名顺序。

## Dataset / Minimal Experiment

train12个reset groups22600–22611、validation3组22650–22652，各收集baseline完整attempt。
test3组22700–22702仅预留，不采集、不归一化、不评价。
不随机按row split；同reset的全部perturbation共享一个group，评价只用validation。
失败attempt完整保存，只用任务成功expert作为监督标签，排除列表写manifest；失败不自动变为成功动作示范。
normalizer只拟合successful train的transition-start rows，不含terminal observation或validation。
每条200action／201observation；全4001个physics snapshots用于评分，避免只看50Hz采样漏掉短暂失败。
保存raw_action、实际bounded_action、previous_ctrl，便于区分模型与限幅器作用。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/dexterous_bc.py --output tmp/s22_6/my_run
```

output必须新目录；data/cohort/rollouts/JSON/PNG仅ignored tmp。脚本收集专家，训练三个模型，再评价
3个validation resets×5个pulse（baseline、Fx±、Tx±）×（expert＋3BC）共60次attempt。
模型source/checkpoint/data manifest SHA与task/schema/split检查；`--models-from`仅恢复冻结模型，不重训。
五个pulse是预先声明的教学子集，不冒充S22.5完整八方向矩阵。

## Expected / Actual / Explanation

预期：新50Hz专家先通过nominal；BC offline loss下降，但闭环不保证成功。
强pulse可能使expert和learner失败；未ready时pulse不施加，仍保留trial分母。
2026-10-12 / DESKTOP-781D67A：15条expert baseline全部成功，2400 train／600 validation label行，无label排除。
三个模型各7046参数，best epoch91/84/110；offline reference RMSE .003748/.002607/.003160rad，
training loop1.278/1.209/1.202s（不含采集／完整实验）。

| Controller | 默认ready / success | 四倍ready / success |
| --- | --- | --- |
| expert | 15/15、15/15 | 15/15、3/15 |
| BC seed226 | 0/15、0/15 | 0/15、0/15 |
| BC seed227 | 0/15、0/15 | 0/15、0/15 |
| BC seed228 | 0/15、0/15 | 0/15、0/15 |

专家四倍仅3条baseline成功，其余Fx±／Tx±触发contact loss／slip等；三reset的具体rotation失败不全相同。
BC默认合计0/45 ready/success，其中40次close_timeout；seed226/reset22652的五个pulse任务曾进入HOLD，
随后丢失接触，2s起锁存contact_loss，2.049s drift、2.11s downward_drop、2.122s nonfinger_support。
所有attempt保留，后四项时间以results.json为准；已重新核对固定seed对应的完整physics。
BC每组五次pulse虽为15条计划试验，但未ready所以pulse未施加，同reset轨迹重复；不能当独立15个随机样本。
多数BC没有进入HOLD，向下.1N外载荷也未施加；seed226/reset22652进入HOLD后仍施加载荷，即使ready=False。

`rollout.png`只画首validation reset的Fx+：BC漂移接近零是因为没有建立合格抓持且未触发载荷／pulse，
**不是比专家更稳定**。本机另检查reset22652的contact_diagnostics.png，显示曾接触又释放、载荷后掉落与未建立接触的区别。
验证目录`tmp/s22_6/initial`与`tmp/s22_6/strong`；默认和冻结四倍命令均exit0，全部完成60trial／组。
两组最大Newton残差3.93e−15/6.05e−15N，不放宽阈值；每组480个action tick触发box或rate限制。
默认与四倍的所有2.5s前轨迹相同，baseline和未ready轨迹全程逐值相同；同三个checkpoint不变。
闭环同state专家规则的raw参考RMSE为.0572～.3398rad，明显大于expert validation offline误差；
其中包括train没有的FAULT阶段，且raw动作尚未经limiter。它显示训练外偏差，不单独证明最初失败是哪一输入导致。
未做通道ablation、恢复数据或接管干预，不宣称contact输入无效或BC一定无法学会。

本机环境：WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco所属Python3.12.14，
MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2、Torch2.14.0+cpu；metadata、真实imports/paths/API通过，无安装。
独立核验命令`python tmp/s22_6/verify_artifacts.py`（本地临时脚本，不提交）：

- 135trial／540135physics行；27135个pre-action观测从保存state＋previous_ctrl独立重建，310点pre/post normal不同，证明不能混用两份力。
- 按来源组核对数据，重新拼接2400/600行，重算train-only均值／尺度；NumPy独立MLP、offline RMSE及CSV best epoch核对。
- 全trial重新计算scorer／first times、ready、pulse、servo／torque cap、box/rate动作与20step held ctrl、Newton及free-body平移积分关系。
- 默认与四倍的前缀、baseline及未ready全轨迹逐值一致；六条保存ctrl＋外载荷重积分，qpos/qvel逐值一致，post normal误差≤1e−12N。
- 两条完整原observer/monitor实际action回放的obs／physics／qpos／qvel／ctrl逐值一致，包括掉落失败；非中间state复制。
- 非finite／错shape动作、checkpoint SHA篡改、source metadata错配拒绝；epochs0／nan pulse／四倍不带冻结模型均CLI exit2。
- Python3.11语法通过（实际3.12）；两rollout PNG与本机contact诊断图已查看；链接／`git diff --check`通过。

两组BC推理时间合计18000次，median/p95约.129/.352ms，含归一化／tensor／网络／反归一化；
不含contact刷新、physics、评分、I/O，没有剔除首次调用，不是硬实时保证。
没有test／GUI、真人、真实sensor或硬件结论。

## Failure Cases / Robotics Context

- reference在HOLD阶段每次预测偏一点，就可能改变法向压力和物体稳定性，低offline RMSE无法排除累积漂移。
- pre/post-action力混用、把当前target作为输入、按row切分，都可能产生漂亮但不可信的离线成绩。
- 接触truth、物体pose/velocity来自仿真；真实部署要考虑可测性、同步、估计误差与传感器定义。
- 时间和phase让本任务可表达，不代表网络学到了任意抓取、force closure或主动恢复；没有goal泛化。
- limiter和扭矩cap约束输入与驱动范围，不证明不滑动、不drop或硬件安全。
- 强扰动失败可能反映固定HOLD专家能力边界；不能从一次差异宣称学习策略更优或接触通道必需。

应用：用有限窗口的多接触保持任务学习示范→监督预测→真实闭环反馈的差异；作为学习策略接入已有低层servo的最小桥梁。

## Interview Capsule

30秒：我重新用50Hz因果循环采集三指free-object专家，使用上一参考下的接触力、过去ctrl和在线接触phase训练BC。数据按reset分组，统计只用train、offline validation选模，然后用同1kHz失败锁存scorer评价闭环，保留全部失败。模型输出rad参考，限幅和servo再驱动实际动力学。

2分钟：说明S22.5数据为何不能直接训练；画出previous ctrl→forward/observe→BC→box/rate→servo→mj_step的时间轴。
解释38维输入、200/201合同、过去ctrl如何提供冻结目标记忆；指出monitor不替BC执行HOLD动作。
给出train/validation/test边界，比较offline RMSE与实际success，逐项看contact/pose/slip/tail失败。
最后区分参考变化限制、扭矩限制与全过程抓持稳定性，说明仿真proxy与未完成的真实部署范围。

## Must Remember / Run / Modify / Explain

Run：执行默认命令，查看三模型offline loss、validation各pulse success和失败first times。

Modify：先预测四倍pulse是否改变2.5s前轨迹／ready，再冻结同cohort运行：

```bash
python examples/23_contact_aware_learning/dexterous_bc.py --models-from tmp/s22_6/my_run/cohort --pulse-scale 4 --output tmp/s22_6/my_strong
```

比较默认与四倍的同reset／模型／pulse；baseline、扰动前和未ready轨迹应相同，区别ready与success。
四倍模型不重训；不按结果调参后再声称同冻结比较。

Explain：

1. 哪一次`mj_forward`产生policy输入？为什么另一份normal不能决定同一动作？
2. previous_ctrl和phase从哪里来，如何帮助表达专家记忆？为什么不是当前标签或expert oracle？
3. 6维action、.02rad rate cap、.08N·m torque cap分别限制什么？为何不保证实际速度与接触稳定性？
4. 为什么按reset group切分、normalizer只用train？200action需要多少obs，terminal行是否参与统计？
5. offline RMSE小为何还会闭环失败？ready／success／末段恢复怎样区分，为什么不删未ready试验？

My Verification：2026-10-12 本人确认实验与预测完成，并准确回答五项Explain；根README Run/Modify/Explain全部完成，Engineering Complete + Learning Mastered。
下一小任务S22.7等待明确请求，本课不自动实施。

本人解释覆盖pre/post-action力的因果时序、previous_ctrl与在线phase的来源、reference/rate/torque三层限制、
reset group隔离与train-only transition-start统计，以及闭环分布偏移与未ready不能证明抗扰。
范围精度：HOLD后丢失接触的trial仍可能施加2s后的.1N载荷，pulse另由ready门控，不能把无pulse等同无外载荷。
本次仅文档状态同步，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-12工程验证。
