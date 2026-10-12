# S21.1 — Multimodal paths / regression averaging

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [实验](../examples/22_diffusion_policy/multimodal_paths.py) · [任务与评分](../examples/22_diffusion_policy/dual_path_task.py)
前置S20.7；0.5～2小时。主概念：平方回归的条件均值未必是合法动作序列。

## Problem → Why → Intuition

从左侧到右侧，圆形禁入区挡住直线，上绕和下绕都合法。若同一初始观测下示范者各有一半概率选上下路径，平均计划会从中间穿过圆盘。
直觉：两条可行绕行路线的折中，不一定还是可行路线。数据的多种合理选择不是必须消除的测量噪声。
这解释了为什么需要建模条件动作分布；[Diffusion Policy原论文](https://arxiv.org/abs/2303.04137)讨论多模态动作生成，本节先验证问题，不实现去噪模型或预判其更优。

本节开启B轨道`xy_two_route_disk_v1`。A轨道S19/S20只有到达目标，其成功数字不能搬到新任务直接比较；BC/ACT在B轨道重新训练。

## Core Concepts / task contract

| 量 | 含义、shape、单位与frame |
| --- | --- |
| o | (6,)，world x,y[m]、vx,vy[m/s]、goal x,y[m] |
| a | (2,)，world XY绝对位置reference[m]，不是ctrl或下一帧qpos |
| A | (150,2)，完整脚本reference序列；训练ACT只预测H10 |
| z | ACT latent(4,)，无量纲；部署固定prior z=0 |
| radius | .010m，跟踪XY点的几何禁入半径；不是老visual sphere的碰撞尺寸 |
| time | T150×.02s=3s，reference50Hz，physics/阻抗feedback1kHz |

初始x∈[−.032,−.028]m、y=0、velocity=0；goal x∈[.028,.032]m、y=0。
脚本waypoints：start → (−.018,±.026) → (.018,±.026) → goal。
每段30个reference tick，u∈[0,1]使用3u²−2u³，最后60tick保持goal。
route side与script phase是expert内部信息，不进入o；机器人实际状态仍通过低层反馈跟踪reference。
同pair seed上下路的o_0完全相同、action_0的y相反；之后观测y/vy可能揭示已选择的route，不是每个时刻都不可辨识。

圆盘只作为显式几何任务约束，未加入MuJoCo contact geom，穿入时没有接触反力。
实际1kHz运动每段都检查禁入区，首次碰撞锁存；为观察末态继续积分，最终到达也不改写失败。
这是一项配置空间安全诊断，不是刚体碰撞/工具外形或硬件安全验证。

## Mathematics

给定同一o_0，上下完整计划分别为U、L，选择概率p与1−p。把每个坐标看成欧氏回归输出：

```text
R(M|o_0) = p ||M−U||² + (1−p) ||M−L||²
∇_M R = 2(M−pU−(1−p)L)
M* = E[A|o_0] = pU + (1−p)L
R(M*) = p(1−p)||U−L||²
```

代码风险用mean_(time,channel)代替sum，单位m²；只改变归约比例，不改变最优M。
当p=.5且上下对称时，M_y*=0；x由负到正，连续路径穿过原点，必进入半径10mm圆盘。
安全轨迹集合是非凸的，所以两条安全序列的凸组合未必安全。按同一个时间槽平均很重要，但正确对齐也不能保证模式平均安全。
位置/动作的线性归一化不会消除此问题，反归一化的条件均值仍是米单位的均值。

p=.8时M_y=(2p−1)U_y=.6U_y，上绕高度被缩小；可能够用，也可能仍不够，必须验证全过程clearance。
CLI只改变解析mean control的p，**不改变BC/ACT训练数据的平衡分布**，从而隔离均值权重这一个因素。
这也不同于S20.6：这里平均同初始条件下的两种路线，S20.6平均不同历史预测中指向当前时刻的动作。

几何检查：对于运动线段a→b，v=b−a，u=clip(−a·v/(v·v),0,1)，closest=a+uv。
clearance=||closest||−radius[m]；≤0判碰撞。v=0使用u=0；线段检查避免两个安全端点之间“穿透漏检”。
success=全过程无碰撞 AND 最后10观测每个位置误差<1mm、速度<2mm/s。
`tail_goal_reached`只检查后者，不等于success；所有trial固定时长truncated=True。

## Math-to-Code / APIs

`dual_path_task.references(initial,side)`返回(150,2)脚本reference；`reset(seed)`返回(6,)实际初始观测。
`segment_clearance(a,b)`返回线段到圆盘的signed clearance[m]；`clearance(points)`返回每段的clearance。
`rollout`返回轨迹字典与评分：每次创建独立model/data，只在reset写qpos/qvel。
`MjModel.from_xml_string(XML)`编译模型并返回MjModel；`MjData(model)`返回可变仿真状态。
`mj_forward(model,data)`原地刷新派生量，不推进时间；`mj_step(model,data)`原地积分qpos/qvel/time，无新状态返回。
控制每ms计算F=400(a−q)−Dqdot，D=2sqrt(mass×400)，mass=[2,1]kg；requested/gear写ctrl，actuator限幅到±20N。
完整`physics_xy`保存(T×20+1,2)，用于独立检查真实轨迹，而不是只检查reference。

`mean_risk(U,L,p)`返回加权均值与三种计划风险，并核对p(1−p)解析值和最优性。
这是给定初始条件的**完整计划条件均值反例**，不声称训练BC必然沿着整条均值轨迹执行。
BC每步看新actual状态，ACT每E步看新状态；模型偏差或微小不对称可以使策略选择某一路。

`make_dataset`按paired reset分组：8train/2validation/2test；每个非test reset生成上下两条轨迹。
train16episode/2400rows，validation4episode/600rows；test只保留reset seeds，不生成轨迹/读取test标签。
normalizer只用train。保留pair_id/episode_id/side/timestep；side只用于审计，不作为网络输入。
同reset的两路不能分到不同组，否则高度相关的任务条件会泄漏。

BC沿用6→32→32→2、100epochs/seed211；ACT沿用S20.5 D32/2head/FF64/latent4/H10、60epochs/seed212。
两者CPU1thread/Adam/lr.001/batch64；BC单步MSE、ACT masked reconstruction+.01KL。
选模分别用validation单步MSE和validation prior-zero chunk MSE，不能混比选模loss；共同offline指标只取当前槽RMSE。
模型checkpoint声明新task与paired dataset hash；旧A轨道weights不能直接通过新任务loader。
`--models-from`核验task/dataset/normalizer并重载，部署禁止posterior，无oracle route label。

## Minimal Experiment / Expected Result

依赖requirements已有NumPy/CPU Torch/MuJoCo/Matplotlib；helper依赖Stage20 TinyACT/BC/windows/latent/attention和旧XY fixture。
无安装或新库，无历史tmp输入，实验自动生成新任务数据、训练并评估validation。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/multimodal_paths.py
```

输出新ignored tmp/s21_1/run_*：dataset.json、protocol.json、normalizer/train/validation与20个示范NPZ、BC/ACT checkpoint/loss、
当前槽offline预测、14个validation trial NPZ、trials.csv、results.json、paths.png。
预期：专家两路安全；uniform mean风险小于单一路线，却碰撞；BC/ACT实测不预设失败方式或胜负。

## Actual Result / Explanation

2026-10-12，DESKTOP-781D67A，命令`python examples/22_diffusion_policy/multimodal_paths.py --output tmp/s21_1/verified`，exit0。
BC/ACT best96/60，trainloop1.345/14.342s（含epoch评分，不含示范采集/rollout），参数1346/31146；共同当前槽val RMSE .187/.408mm。

| 模式 | success | collision | tail goal reached | final mean error [mm] | 最小actual净空 [mm] |
| --- | --- | --- | --- | --- | --- |
| expert upper | 2/2 | 0/2 | 2/2 | <.001 | 15.268 |
| expert lower | 2/2 | 0/2 | 2/2 | <.001 | 15.268 |
| uniform mean | 0/2 | 2/2 | 2/2 | <.001 | −10.000 |
| BC | 0/2 | 0/2 | 0/2 | 18.099 | 9.546 |
| ACT E1 | 1/2 | 0/2 | 1/2 | .897 | 16.145 |
| ACT E5 | 2/2 | 0/2 | 2/2 | .306 | 14.334 |

均匀mean的全计划回归风险1.17817148e−4m²，小于选择单一路线的2.35634297e−4m²，但两次均碰撞。
这是相同o_0下的完整计划风险，与BC/ACT当前槽RMSE的标签范围不同，不能直接比数值大小。
默认weighted_mean(p=.5)与uniform相同；全部模式预先定义、全部attempt保留，不按rollout选模。

追加`--models-from tmp/s21_1/verified --mean-upper-weight .8 --output tmp/s21_1/weight08`，exit0，不重训。
weighted_mean成功2/2、碰撞0/2、actual minimum clearance5.599mm；几何reference净空5.600mm，峰值y15.6mm。
其他模式、归一化与checkpoint逐值一致。p=.8表示解析假设的偏置混合，不表示重采样训练数据。

两组共40个示范与28个评估trial（重复同seed），独立Euler重算204000步；用垂线/端点距离另算全部运动段的几何碰撞。
actual-state重新预测/执行索引/scorer/aggregate、paired split/train-only normalizer/窗口padding通过。
解析MSE最优值与偏移风险、tangent/静止段/穿透反例、四CLI拒绝/旧A轨道与dataset mismatch通过，3.11语法通过（actual Python3.12.14）。
两PNG已查看；图只展示首个validation reset，统计用全部两reset；新test保留，不运行GUI/OOD或Diffusion。
mode coverage只描述两个validation reset的实际穿越方向，不证明学习到完整分布。

ACT固定z=0只走上方；一次固定latent的成功不能证明上下模式都被生成，更不能说明Diffusion尚未实现就更好。
BC低offline误差却未成功，不应笼统归因于“BC均值撞障碍”：本次BC未碰撞，沿下方绕行但未稳定到达目标。
脚本phase未作为可观测输入，policy分布偏移和模型误差都可能影响执行；未做因果干预定位，不把这些假设写成已证原因。

## Failure Cases / Robotics Context

- 模式平均：两种路线都可行，但平均路线可能违反非凸约束。
- 模式切换：逐tick独立选择上下可能打断序列一致性，本节未做采样策略；后续要检查整个sequence。
- 条件含义混淆：有显式route指令时条件分布可能单峰；本节隐藏route，初始观测存在真实歧义。
- 数据泄漏：同reset的上下路必须同组，不能随机按row或按单route split。
- 只看末态：均值轨迹最后到达，但途中碰撞必须保留失败。
- 检查对象错误：reference安全不等于actual安全，visual geometry不等于本节point forbidden disk。

机器人应用：绕障碍、不同抓取侧、不同操作顺序等多解任务；可用route条件化或分布建模，生成后仍需安全与闭环验证。
本节不实现路线指令策略、latent采样、Diffusion或B轨道统一test排名。

## Interview Capsule

30秒：MSE的最优输出是条件均值；当同一状态对应上下两条合法路线时，均值可能穿过障碍，因为安全集合非凸。需要区分学习一个折中输出与生成一条连贯模式，同时独立检查实际轨迹安全。

2分钟：固定同reset与goal生成两种脚本示范，按pair分组防泄漏、train-only归一化。在完整计划上解析推导pU+(1−p)L，均匀平均风险最低但实际轨迹穿入几何禁入区。真实1kHz运动segment检查并锁存碰撞，最终到达不能洗掉失败。BC/ACT在新任务重训，validation offline选模、真实闭环评价；BC不必严格执行条件均值，ACT z=0也不保证模式覆盖。只有少量validation与单training seed，test保留到后续；Diffusion是后续条件分布建模方法，不能从当前反例推断它一定安全。

## Must Remember / My Verification

条件均值不保证可行；模式不是噪声；同reset成对分组；actual全过程检查；生成分布与安全验证是不同要求。
根README是状态唯一来源；助手执行不计本人Run。

- Run：运行上面命令，查看paths.png、mean_risk、collision与tail_goal_reached。
- Modify：把输出目录换成自己Run打印的路径，执行：

```bash
python examples/22_diffusion_policy/multimodal_paths.py \
  --models-from tmp/s21_1/verified --mean-upper-weight .8
```

先预测均值路线的y幅度与碰撞结果；模型不重训、训练分布不变，比较解析控制变化。

- Explain：①为何平方误差的最优输出是条件均值？②上下合法为何平均不合法，p=.8对y幅度有什么影响？③为什么同reset两route必须同split，side不进入obs意味着什么？④为什么均值到达goal仍判失败，为什么检查actual运动段？⑤为何不能说本次BC必然走均值、ACT成功就证明多模态覆盖或Diffusion更优？

本人已确认实验和预测完成，并准确回答五项Explain；Run/Modify/Explain全部完成，Learning Mastered（2026-10-12）。
解释覆盖MSE条件方差分解与条件均值、非凸安全集合/p=.8幅度、paired reset分组、actual运动段与全过程安全、有限模型行为与模式覆盖边界。
不自动推进S21.2。
