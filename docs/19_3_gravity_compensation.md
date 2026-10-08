# S16.3 — UR5e Gravity Compensation Integration

[代码](../examples/17_force_dynamics/gravity_compensation.py) · [阶段入口](19_force_dynamics.md) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

S16.2知道了力矩预算，现在把它接到既有UR5e：如何由真实motor输出抵消重力，
而不是将bias直接写入无限制的qfrc_applied？重力补偿减轻位置反馈负担，
但补偿只让机器人“不因重力加速”，不提供回到某个角度的恢复力。
把手臂想成被托住重量的机构：推一下仍可离开原位；增加PD才会拉回指定姿态。
本课复用运动学模型与home，不重做IK、轨迹、gripper、ROS或接触控制。

## Core Concepts / Physical Contract

场景为Menagerie裸UR5e，world重力[0,0,−9.81]m/s²；六个hinge按各自轴右手正向。
力矩为joint-space广义力，不是六维world wrench，六个分量对应六个关节。
显式关闭contact与joint limits，避免掉落对照被地板/限位反力掩盖；
这允许baseline转过多圈，不表示真实UR5e允许这样的运动。保留原惯量、armature和被动项。

| 量 | shape / 单位 / 含义 |
| --- | --- |
| q / v / target | (6,), rad / rad/s / rad，使用name→id→qposadr/dofadr映射 |
| g(q) | (6,), N·m，在独立data中v=0求出的qfrc_bias |
| b(q,v) | (6,), N·m，实际运动状态的完整bias |
| Kp / Kd | (6,), N·m/rad / N·m·s/rad，逐关节反馈增益 |
| requested / qfrc_actuator | (6,), N·m，希望的与实际的joint torque |
| ctrl / actuator_force | (6,), 本hinge motor scalar torque coordinate，N·m；需乘gear得到joint torque |
| external / qfrc_applied | (6,), N·m，合成扰动力矩，非补偿通道或接触测量 |

原模型六个general actuator是position servo。本例在私有compiled model中改成
fixed gain1、bias NONE、无activation动态的motor；gain/bias参数清零，ctrl角度限幅关闭，
force限幅保留。cached MJCF与P0/P1源码不变。home keyframe原ctrl是角度，reset后明确清零。
前三关节joint cap±150N·m，后三±28N·m，是模型参数，不是本课核实的厂商硬件额定值。
shoulder-lift人为用gear2，其余gear1；motor scalar force cap必须除以gear，保持joint能力一致。

## Mathematics → Math-to-Code

```text
M(q) vdot + C(q,v)v + g(q) = tau_act + tau_passive + tau_external
zero_torque:  tau_requested = 0
gravity_only: tau_requested = g(q)
gravity_hold: tau_requested = g(q) + Kp*(q_target-q) - Kd*v
ctrl_i = tau_requested_i / gear_i
p_i = clip(ctrl_i, -joint_cap_i/gear_i, joint_cap_i/gear_i)
tau_actual_i = gear_i*p_i
```

本例无constraint；每步核验ncon=nefc=0、body_gravcomp=0和xfrc_applied=0，
保证没有接触或隐藏重力补偿来源。只有扰动通过qfrc_applied施加。
速度为零时g=b；运动时g通常不等于b。gravity函数把当前q复制到独立probe，
将probe速度清零后forward；实际robot qvel保持原值。

无外力、速度零、精确模型且未饱和时tau=g使加速度零；纯gravity没有位置恢复项，
不记住home。PD加恢复力与主动阻尼，但能力受总力矩cap限制。
cap_scale=.1时初始shoulder-lift/elbow各需约−15.857N·m，能力只有±15N·m，
所以home静态平衡已经不可实现，提高Kp也不能让motor突破上限。

重力还以势能独立核对：`U(q)=−sum_i m_i gravity_world·COM_world_i`，
central difference `∂U/∂q_i≈[U(q+eps)-U(q-eps)]/(2eps)`应等于g_i。
这是在模型内交叉检查符号和映射，不能验证真实机器质量参数。
每步用dense M核对完整力矩平衡；记录b−g暴露速度bias，未设计computed torque控制。
依据[官方 dynamics/actuation](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework)。

## API：输入、输出与状态变化

| API / array | 行为 / 本课用途 |
| --- | --- |
| mujoco_menagerie.load(name) | 编译并返回MjModel；依赖本地缓存或下载；使用既有UR5e模型 |
| MjData(model) | 创建mutable state及derived arrays；actual与probe分开 |
| model.joint/actuator(name) | 返回named view，读取id；jnt_qposadr/jnt_dofadr确定状态地址 |
| mj_resetDataKeyframe(model,data,key_id) | 原地初始化home q/v/ctrl/time，返回None；之后清除旧position ctrl |
| mj_forward(model,data) | 原地刷新kinematics/forces/bias/qacc，不推进time；当前时刻读数对齐 |
| mj_fullM(model,data,dst) | 当前3.13 binding原地填充(nv,nv)惯量矩阵，返回None |
| mj_step(model,data) | 原地积分一个dt，推进actual q/v/time；不返回新状态 |
| xipos / body_mass | (nbody,3) world质心m / (nbody,) kg；势能梯度检查 |

积分implicitfast、physics/control同为1ms；Python PD每step显式计算，未验证高增益/延迟稳定范围。
每条CSV先计算g→写ctrl/external→forward→记录→step，pre-step t0–4s共4001行。
初始化之外不赋actual qpos/qvel；probe赋值只是模型计算，不是瞬移控制。

## Minimal Experiment / Dependencies

requirements已声明mujoco、numpy、matplotlib、mujoco-menagerie；无local helper、无新增依赖。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/gravity_compensation.py
python examples/17_force_dynamics/gravity_compensation.py --no-pulse
python examples/17_force_dynamics/gravity_compensation.py --cap-scale 0.1
```

默认外力矩在.5≤t<.7s施加于shoulder-lift，+5N·m，200steps，三模式相同。
产物仅ignored tmp/s16_3_cap*_pulse*/：三份4001×49 CSV、results.json、response.png。
图前两行对照shoulder-lift与最大关节偏移；第三行显示hold的requested/actual/gravity力矩。
全幅对照可能掩盖默认hold小误差，以JSON peak/tail与CSV补充查看。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；conda mujoco所属Python3.12.14。
MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2、Menagerie2026.9.2 metadata与真实路径/import核验；
未安装包，未更改requirements；代码3.11兼容目标未用3.11执行。

| 默认有pulse / cap1 | peak max joint error (rad) | final max error (rad) | tail max speed (rad/s) |
| --- | --- | --- | --- |
| zero torque | 17.874892 | 13.960439 | 28.631693 |
| gravity only | 3.191614 | 3.191614 | .756684 |
| gravity + hold | .023475859 | 2.326e−9 | 1.880e−7 |

默认hold末.5s最大误差4.668e−8rad，三模式均无饱和。
no-pulse下gravity_only与hold全程q/v不变，数值误差0；这是理想同模型静态平衡，非硬件精度。
cap .1时hold动态饱和6584个joint-samples（不是6584个time steps），
peak误差3.578479rad、final1.799907rad、末窗口速度3.298201rad/s，未恢复也未静止。
此case通过的是“限幅生效且恢复失败被正确检出”，并不是hold性能PASS。
所有三条命令exit0，无import error；势能梯度最大误差9.349e−9N·m，
各组平衡残差≤1.43e−13N·m；每步actual=clipped requested=gear*scalar output核验。
运动中的b−g默认gravity_only最大.333781N·m、hold .010951N·m，证明两种bias读法不同。
zero torque掉落/连续旋转只作无约束教学对照，无碰撞、硬件安全或物理可行姿态保证。
无GUI、payload、真实摩擦/驱动带宽/测量噪声或contact验证。

## Failure Cases

- 把运行态bias当gravity：补偿额外速度项，变成另一个控制实验。
- 清零actual qvel求g：修改物理状态，伪造恢复；必须用probe。
- 未移除servo gain/bias或旧ctrl限制：motor ctrl仍被当角度/受角度范围夹断。
- gear2不除2、force cap不除2：分别给错力矩或改变joint能力。
- qfrc_applied=bias：绕过motor cap，掩盖能力不足。
- 相信gravity-only无扰动保持就是位置控制：扰动后没有回home的恢复项。
- 限幅后还用requested做受力分析：必须使用actual qfrc_actuator。
- 错把极小仿真误差推广到硬件：参数、时延、噪声与驱动约束未建模。

## Robotics Context / Interview Capsule

重力feedforward常与位置/速度feedback组合：模型承担可预测负载，反馈处理偏差；
加入payload须重算质量模型与驱动裕量。后续接触仍需区分motor输出和环境反力。
本课是Stage16前半段Integration，仅裸UR5e自由空间hold，不自动推进wrench/contact。

**30秒：** 在独立data中用当前q与零速度取g(q)，经motor gear和force cap施加，
用PD恢复home。纯补偿可静态托住重量但不恢复扰动；驱动不足时补偿加PD也可能失败。

**2分钟：** 写完整动力学，指出运行bias含速度项。说明name/address映射和servo改motor，
以gear2和joint cap解释ctrl/actual。描述同脉冲三模式对照、no-pulse平衡与低cap失败，
引用势能梯度/force balance检查，最后限定同模型无接触headless证据。

## Must Remember / My Verification / Run → Modify → Explain

g(q)=bias(q,0)；probe不改actual；joint torque=gear*scalar force；
重力补偿不提供位置恢复；反馈也受总输出限幅；小仿真误差不代表硬件精度。
Engineering见根README。2026-10-08 本人确认实验与预测完成，Run/Modify完成；
本人随后补正第2项，准确说明ctrl不夹断、scalar force夹为−7.5、
gear2映射实际−15N·m与约.857N·m缺口；五项Explain完成，Learning Mastered。
第5项已补充动态完整力矩预算。
本课ctrllimited=False；低cap shoulder-lift joint cap为±15N·m、gear2，
scalar forcerange为±7.5。requested=−15.857N·m时ctrl=−7.9285不被ctrl clamp，
actuator_force夹为−7.5，实际joint torque为−15N·m；没有ctrlrange±1的配置。
动态force balance包含M qacc、完整bias、passive、actuator、external及constraint，
不是仅比较motor与gravity。此次只更新文档，未重跑仿真，runtime沿用2026-10-08工程验证。

**Run：** 运行默认命令，查看JSON和图，比较外力撤去后三模式是否回home。

**Modify（只一项）：** 先预测将cap缩为.1对初始平衡、饱和、恢复的影响，再运行
`--cap-scale 0.1`；保持脉冲、gear与PD增益不变，用actual torque和tail指标核对。

**Explain（五问）：**

1. 为什么用独立probe的零速度bias求g，而不直接复制运动状态的bias？
2. shoulder-lift要求−15.857N·m、gear2时ctrl是多少？cap .1时actual最多是多少？
3. gravity-only无扰动保持home，为何受脉冲后不回home？PD提供了什么？
4. 为何不能通过qfrc_applied补齐motor饱和缺少的力矩来宣称补偿成功？
5. 势能梯度与force balance分别检查什么？关闭contact/limits限定了哪些结论？
