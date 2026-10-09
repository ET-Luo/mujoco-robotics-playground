# S18.3 — Teleoperation + impedance：低频输入与本地反馈

约0.5～2h；前置：[S18.1](21_1_incremental_teleoperation.md)、[S18.2](21_2_scaling_clutch.md)、[S17.2](20_2_cartesian_impedance.md)。
[代码](../examples/19_teleoperation_dexterous/teleoperation_impedance.py) / [示例README](../examples/19_teleoperation_dexterous/README.md)。
Engineering / Learning唯一状态：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

手柄只有50Hz输入，机器人是否也只能50Hz反馈？不能把“什么时候得到新目标”与“什么时候重新观察并控制机器人”混为一谈。
本课保持最近的参考位置，每1ms用**当前实际位置、速度**重新计算弹簧和阻尼力。
直觉：人每20ms告诉机器人下一目标，本地控制器在这20ms里仍持续观察、纠偏、阻尼。
即使手柄静止、目标不变，机器人仍可能运动，控制力也应随实际状态变化。

## Core Concepts / scope

复用Stage17的固定姿态XY滑台、motor、质量、接触模型和测力函数；复用S18.1的增量映射/norm限速/box。
本课synthetic master只沿X运动，R_WM将它映射到world Y；scale固定1。
为了与既有墙面配合，本fixture world anchor=[0,.06,0]m，而不是S18.1的[-.45,.20,.30]m。
box lower=[-.04,.015,-.02]m、upper=[.04,.09,.02]m；此锚点变更是换学习模型时的初始化，不是运行中recenter。
master周期h=.02/.1s；本地反馈和physics周期dt=.001s，分别50/10Hz输入与1000Hz反馈。
同一进程按整数step调度，输入不触发额外physics step；10s仿真固定10000步。不是线程/IPC/ROS，也不是实时调度保证。

| case | 环境 | 反馈更新 | 两个master样本之间 |
| --- | --- | --- | --- |
| free_local | 无墙 | 每1ms | 目标保持，重新计算motor力 |
| contact_local | +Y墙 | 每1ms | 目标保持，重新计算motor力 |
| contact_packet_feedback | +Y墙 | 每h秒 | motor命令保持，physics仍每1ms |

最后一组是对照。50Hz并不必然失败；10Hz本参数下明显失败。没有一般稳定性或“频率越高越安全”的证明。
clutch/recenter的完整接触任务与过期输入处理留S18.5；本课无keyboard、UR5e、姿态、真实设备或haptic反馈。
依赖requirements已有mujoco/numpy/matplotlib；local helpers为incremental_reference、cartesian_spring、contact_transition，依赖链仅上述三包。无新安装/依赖。

## Mathematics：meaning / shape / unit / frame

输入只在到达样本时更新：

```text
Δm_M = m_M[k] − m_M[k−1]                 # (3,) m，master frame
Δx_req_W = scale · R_WM · Δm_M            # (3,) m，world
α = min(1, v_cap·h / ||Δx_req_W||₂)       # zero时取1
x_ref_W[k] = clip(x_ref_W[k−1] + αΔx_req_W, lower, upper)
```

v_cap=.02m/s，reference样本间norm增量速度有界。实际脚本master速度.015m/s，requested正常增量无需限速。
拒绝运动直接消费；本实验最后到达box Y下界.015m，没有继续向外要求运动。
目标使用zero-order hold（ZOH，保持最新样本），明确选`v_d=0`：

\[
v_W=J_{p,W}\dot q,\quad
F_W=K(x_{ref,W}-x_W)-D v_W,\quad
\tau_{req}=J_{p,W}^{\mathsf T}F_W,\quad
ctrl_i=\tau_{req,i}/gear_i.
\]

| 量 | shape | 单位 / frame |
| --- | --- | --- |
| x_ref、x、v、F | (3,) | world；m、m、m/s、N |
| Jp | (3,2) | world site速度映射；本slide模型列无量纲 |
| K | 标量400 | N/m，逐分量作用 |
| D | (3,)=[56.568542,40,0] | world对角阻尼，N·s/m |
| q、qvel、τ_req | (2,) | slide位移m、速度m/s、广义力N |
| M | (2,2)=diag(2,1) | kg，真实有效质量 |

本模型是slide，虽然一般称torque loop，实际输出是**广义力N**；hinge才是N·m。
gear=[2,1]，每slide实际广义力cap20N。X motor自身forcerange±10N乘gear2后为±20N；Y为±20N。
无重力/被动阻尼/bias补偿/姿态控制；Jp=[[1,0],[0,1],[0,0]]。

参考ZOH阶跃不具有普通连续导数；本课不把`Δreference/dt`当期望速度，也不把master速度作为整段vd。
这是一项明确控制选择：移动目标会有滞后。连续匀速近似下，Y速度接近−.015m/s时：
`x−x_ref≈−D·v/K=+.0015m`，即实际比正在靠近墙的目标高约1.5mm。
相对理想连续目标还多了输入采样保持延迟；它与实际动力学滞后是两件事。
参考样本速度界不是ZOH连续导数界，也不是actual velocity/force界。

墙面normal为world +Y，球半径.02m，几何接触球心Y=.02m。ref=.015m不表示把实际球心强行放进墙。
稳态球心约.019995m，弹簧推墙约−2N，环境反力约+2N；接触柔性导致约5µm几何穿透。
法向condim1、solref=[.01,1]、solimp来自Stage17；没有切向摩擦任务。
动力学为`M qacc = actual_motor + Jpᵀ F_environment`，cap约束motor不能约束碰撞反力峰。

## Math-to-Code / MuJoCo APIs

- `MjModel.from_xml_string(XML)`把MJCF字符串编译为模型，返回MjModel；接触case由Stage17 `build_model()`构造墙/球pair。
- `MjData(model)`返回该模型的可变实际状态与计算buffer；只在初始化写qpos=[0,.06]m。随后qpos/qvel只由`mj_step`演化。
- `mj_forward(model,data)`无返回状态，原地刷新FK、动力学/接触求解，不推进time。本课先获取当前state，再设ctrl并重解当前接触。
- `mj_jacSite(model,data,jp,jr,site_id)`无新数组返回；写入world平移/角速度Jacobian(3,nv)，本例nv=2，用jp@qvel得到实际速度。
- `mj_fullM(model,data,inertia)`原地展开物理质量矩阵(nv,nv)，用于检查actual dynamics，不控制虚拟mass。
- `mj_contactForce(model,data,index,raw)`写入(6,) contact-frame力/力矩，前三项N、后三项N·m；Stage17 `measure`用contact frame行轴的转置及geom顺序sign转换为environment-on-probe world力。
- `mj_step(model,data)`无新state返回；原地推进qpos/qvel/time一个model timestep=.001s。每步执行，即使无新master输入。

CSV为pre-step：state在t_n；current motor与contact重解后记录；再积分到t_(n+1)。
Euler的独立检查为`v[n+1]=v[n]+dt*a[n]`、`q[n+1]=q[n]+dt*v[n+1]`。
`previous_command_reaction`与当前重解反力都记录，峰值取两者最大，不遗漏命令切换前反力。
`target_age`只是最新已接受目标距当前simulation time的时长，不是网络延迟或过期检测。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/teleoperation_impedance.py
python examples/19_teleoperation_dexterous/teleoperation_impedance.py --master-period .1
```

脚本每条命令跑三case，各10s simulation，CSV/JSON/Agg PNG位于ignored tmp/s18_3_period0.02或period0.1。
master 0–3s沿X−15mm/s；3–5s保持；5–8s反向+15mm/s；8–10s保持。
world reference Y从60mm降至15mm、保持、回60mm。自由空间实际能到15mm；墙面case停在约20mm并产生反力。
预期本地反馈两频率都恢复；10Hz输入reference阶梯较大，actual有更明显ripple和采样滞后。
只在输入样本时更新反馈的对照不能依靠1000Hz physics步长恢复阻尼及时性；需看实际结果，不先判所有低频必失败。

## Actual Result / Explanation

2026-10-09 DESKTOP-781D67A：WSL2 Ubuntu24.04.5/kernel6.6.87.2；同shell conda mujoco与所属Python3.12.14核验。
MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2 metadata/真实import路径/API通过；无安装，未用Python3.11执行。
两命令各exit0，每case10000×20CSV；3.11语法、CLI非法周期、独立CSV验证与links/diff检查通过；两PNG目视。

| 量 | 50Hz master + 1kHz反馈 | 10Hz master + 1kHz反馈 | 10Hz master + 10Hz反馈 |
| --- | --- | --- | --- |
| 新master样本数（t=0为初始化） | 499 | 99 | 99 |
| 反馈计算次数（含t=0） | 10000 | 10000 | 100 |
| physics步数 | 10000 | 10000 | 10000 |
| 环境反力峰N | 3.158396 | 3.712520 | 445.824427 |
| 4–5s平均反力N | 1.998002 | 1.998002 | 1.998002 |
| 首次有效接触s（反力>.2N） | 2.777 | 2.820 | .413 |
| 相对reference几何触点的时间差s | .097 | .120 | −2.287 |
| 末1s最大tracking error | <1µm | <1µm | 136.728mm |
| 末1s最大速度 | <.001mm/s | <.001mm/s | 2089.252mm/s |
| 饱和joint samples | 0 | 0 | 4900 |
| final_tracking_qualified（1mm/2mm/s） | True | True | False |

50Hz master + 50Hz反馈对照也通过末尾tracking，反力峰3.128205N，无饱和；不声称高频组所有指标都更小。
10Hz反馈对照4–5s接触保持仍成功，但退回失败；一次接触稳态正确不能证明完整任务正确。
其提前碰墙与负时间差说明严重振荡；该时间差不能再解释为正常响应延迟。
正常两本地case早期自由运动相对held target平均落后约1.5mm；相对连续理想目标的滞后另存JSON。
接触delay定义从reference第一次Y≤20mm到actual reaction>.2N，包括tracking、采样与阈值；无网络与wall-clock latency实测。

独立验证读取CSV，不调用run：核对packet时刻、target无packet时不变、三个case同reference、
自由/接触在触墙前actual相同、控制公式/held command、cap、mass动力学与Euler递推、静态反力平衡、退回后无contact。
独立线性分析：无接触/未饱和Y轴，反馈每h秒采样、力保持，1ms semi-implicit Euler积分：

```text
c = h(h+dt)/2
A = [[1−cK/m, h−cD/m], [−hK/m, 1−hD/m]]
```

它作用于固定目标误差与速度[e,v]。h=.001/.02/.1时谱半径约.982635/.746053/4.259467。
h=.1已有模大于1的特征值，解释对照为什么会放大偏差。这个局部线性结果不模拟接触和cap，也不证明全局稳定。

## Failure Cases / limits

- 只推进physics而不更新feedback：1000Hz physics仍在积分过时的力，阻尼方向和大小可能不合当前速度。
- 将ZOH跳变除以1ms当vd：把master采样增量变成速度尖峰，改变控制器并可能放大冲击；本课明确vd=0。
- 强制写qpos达到ref：丢掉本课要观察的质量、tracking、contact反力；运行中禁止这样做。
- cap20N不约束反力：10Hz对照约446N是碰撞动力学产生，不是motor输出超cap。
- 参考box不约束actual：失败对照实际运动可越过参考box，不能把参考约束称为避碰/硬件安全。
- master停发：本课没有模拟丢包或lease，目标持续保持，本地反馈继续；输入过期政策在S18.5。
- 本地反馈并非一般passivity保证；移动/阶跃目标可注入能量。本课未验证连续插值、通信延迟、加速度界或真实机器人稳定性。

## Robotics Context

遥操作/远程机器人常把操作者意图作为较慢的目标更新，将状态反馈放在机器人本地。
通信频率改变输入颗粒度，本地环频率决定状态误差多久才影响执行器；两个频率必须分开设计。
本例用XY fixture清楚展示接口，不引入新UR5e场景或重做已有动力学课程。

## Interview Capsule

**30秒：**master增量旋转/限幅得到world位置目标，目标在输入之间保持。
1kHz本地阻抗用当前FK/J/velocity算F=K误差−Dv，再用Jᵀ与gear转成motor命令。
低频输入不必等于低频反馈；本参数10Hz反馈对照失稳，即使physics仍1000Hz。

**2分钟：**说明reference、actual state、motor command的三种状态，分别给采样周期；
解释ZOH与vd=0选择、移动目标滞后，比较自由空间与墙面反力平衡；
用10Hz对照的谱半径/实际振荡说明陈旧反馈问题，最后界定模拟时间、线性分析与硬件安全边界。

## Must Remember

- 保持目标不等于保持力；实际状态变化时本地反馈继续更新。
- physics timestep、master interval、feedback interval分别计数。
- motor cap与环境接触峰值不同；reference界与actual界不同。

## My Verification — Run / Modify / Explain

Learning状态只在根README维护；助手运行不代替本人Run。
Run：默认运行，读图的reference、actual、reaction和motor，找出3–5s为何actual不能到15mm。
Modify：仅将`--master-period .1`；先预测reference阶梯、local计算次数、接触峰/时间差、packet-feedback恢复结果，再核对CSV/JSON。
Explain：
1. master没有新样本时，本地环还需要重新计算什么？为什么不能只保持motor力？
2. 为什么1000Hz physics不等于1000Hz反馈？指出代码中的两个触发条件。
3. 本课vd为什么设0？若用reference跳变量除以1ms，会改变什么？
4. ref=15mm、actual≈20mm时，约2N反力从哪里来？为何cap20N不能限制反力峰？
5. master改为10Hz后，哪些计数不变？哪些结果变差？本实验不能证明哪些真实系统性质？

2026-10-09 本人明确确认实验、预测均完成，并提交五项Explain；根README Run/Modify/Explain全部完成，Learning Mastered。
解释精度：本例稳态反力可由弹簧平衡直接估算400N/m×(20−15)mm≈2N；v≈0时阻尼近零。
本地控制公式仅使用位置/速度，contact测力用于诊断，不构成显式力反馈。
vd=0是ZOH目标下的明确设计选择，不是所有阶梯参考控制器的唯一方案。
10s内新master样本499→99（t=0初始化另计），local反馈/physics各10000次；频率比为5，样本计数比约5。
本次仅文档同步，未重跑数值或GUI，runtime沿用2026-10-09工程验证。
