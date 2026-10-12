# S22.1 — Proprioception + force + tactile contract

[状态唯一来源](../README.md#stage-22--contact-aware-robot-learning) · [代码](../examples/23_contact_aware_learning/observation_contract.py) · [阶段导航](25_contact_aware_robot_learning.md)
前置S21.7；0.5～2小时。Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）。

## Problem → Why → Intuition

机器人知道“手指在哪里”，不一定知道“碰到了什么”。相同关节位置和速度，在不同表面高度下可以接触，也可以悬空。
本节把本体感知、接触力与触觉组织成可解释的观测合同，并检查通道移除损失了哪些信息。
只做观测与最小物理实验，不采集真人示范、不训练BC/ACT/Diffusion，不证明策略成功率。

## Core Concepts / schema

两轴slide指端，无重力；球半径10mm，初始中心world z=40mm；表面world z=0。
因此qz约−30mm开始接触。q不是球中心world z，中心z=0.04+qz。
观测版本`tip_contact_observation_v1`，每行float形状(10,)，所有原始通道在同一simulation timestamp读取。

| 槽 | 通道 | shape / 单位 / 坐标系 / 来源 |
| --- | --- | --- |
| 0:2 | q | (2,), m，沿world+x/+z的slide位移；qpos |
| 2:4 | qdot | (2,), m/s，相同轴；qvel |
| 4:6 | goal | (2,), m，本tick即将下发的关节位置reference，任务给定 |
| 6:9 | F_W | (3,), N，环境作用于tip的接触合力，world轴 |
| 9:10 | touch | (1,), N，pad体积内接触法向力之和，非布尔值、非Pa、非触觉图像 |

F_W来自具名tip/surface pair的仿真contact真值，属于privileged proxy；不宣称硬件直接可得。
真实部署需用可用的力传感器/估计器，并重新定义偏置、标定、坐标变换、带宽和延迟。
MuJoCo `<force>`测量父子body相互作用、以site轴表达，与这里的接触合力不是同一合同。
`<touch>`是本例实际使用的MuJoCo标量传感器。其定义见[官方XML sensor说明](https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor-touch)。
单球与水平面这个场景中touch≈Fz；一般多接触、斜面或摩擦下touch不等于合力模长。
本例无转动观测和wrench力矩，因此不能直接当完整UR5e/多指合同。

## Mathematics / meaning → code

```text
o_t = [q_t, qdot_t, goal_t, F_W,t, touch_t]                 # 10 scalars
F_W = sum_contacts sign * R_CW.T @ f_C                       # N, on tip
sign = +1 when tip is geom[1], else -1
z_t,j = (o_t,j - mean_train,j) / scale_train,j               # dimensionless
scale_j = std_train,j if std_train,j >= 1e-8 else 1
```

contact.frame的三行是contact轴在world中的方向，第一行为法向；因此用转置把局部力转到world。
`mj_contactForce`的六分量是[force;torque]，本节仅取前三项。关于接触轴与API见[官方仿真说明](https://mujoco.readthedocs.io/en/latest/programming/simulation.html#contacts)。
代码另外把Fx/Fz与`qfrc_constraint`比较；两个slide轴恰为world x/z，无joint limit和其他约束，才可做这个直接核对。
不能在任意机械臂上把三维力直接等同于广义关节力。

train两条轨迹，压入reference分别35/40mm；validation为37.5mm。
统计只用train所有402行（包含terminal观测；本节不制作action-label数据集），逐通道population std。
常量Fy通道用scale=1，避免除零或放大极小数；单位换算已固定为SI，未来换单位必须重设该常量通道约定。
validation/noise/airborne/paired不能参与拟合。这里只是合同验证划分，不是学习泛化性能证据。

## Math-to-Code / APIs and time

- `MjModel.from_xml_string(xml)`编译MJCF，返回model：几何、关节、执行器、touch定义和dt。
- `MjData(model)`返回可变运行状态；本例nq=nv=nu=2，qpos/qvel/ctrl各(2,)。
- `mj_step(model,data)`原地积分状态与time；本节1kHz物理，每10步观测一次，即100Hz。
- `mj_forward(model,data)`原地重算当前状态的接触、约束力、传感器，不推进时间。积分后调用，避免把新q与旧sensor混成一行。
- `mj_contactForce(model,data,index,wrench)`向预分配(6,)数组写入局部接触wrench，不返回新数组。
- `data.sensor('touch').data`提供该sensor在sensordata中的view；本节提取一个标量。

每tick先刷新当前state并观测，随后下发goal到ctrl，再积分10步；terminal row只观测、不再执行。
当前力依赖上一interval保持的ctrl；goal是接下来命令，不能解释为已执行后的结果。
paired快照在两模型使用相同q/qdot/ctrl/goal，只改变不可见表面高度。它是静态可辨识性反例，不是两条完整轨迹逐时刻相同，也不证明10维观测具有完整Markov性。

每个通道的采样时间等于time_s，延迟0，无历史buffer。仅force通道可加独立Gaussian N(0,σ²)噪声，seed221；
噪声在干净物理轨迹之后注入，touch保持干净，因此Modify是测量合同检查，不会改变运动。
噪声后负法向读数不裁剪，不代表物理吸引力；本节不模拟偏置/相关噪声/触觉噪声或真实带宽。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/observation_contract.py --output tmp/s22_1/my_run
```

依赖requirements已有mujoco/numpy/matplotlib；无local helper、新库或GUI。output须为新目录，避免覆盖旧结果。
输出CSV（time+10原始测量值）、NPZ（train/validation/paired/normalizer/四组ablation）、results.json和contract.png，均ignored tmp。
预期：approach无接触→press产生力→slide出现切向力→release失去接触；低表面轨迹一直无接触。
同状态paired中state-only距离0，有force或touch则不同。

## Actual Result → Explanation

2026-10-12，Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；conda mujoco的Python3.12.14，
MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2版本与真实import路径核验通过，无安装；代码Python3.11语法检查通过。
实际命令为上述脚本`--output tmp/s22_1/default`（等同默认），再`--force-noise .2 --output tmp/s22_1/noise02`，均exit0。

| 检查 | 结果 |
| --- | --- |
| 2s观测 | 每轨迹201行；physics1ms / observation10ms |
| validation touch峰值 | 7.335705 N，含碰撞瞬态，非稳态目标 |
| validation最小Fx | −3.101051 N，滑动摩擦反向 |
| paired接触 / 无接触 | Fz=touch=1.355968 / 0 N，前6槽完全相同 |
| state / state+force / state+touch / full | 6 / 9 / 7 / 10维，真正移除通道 |
| 力/传感器检查 | 全部采样touch与局部法向和一致，Fx/Fz与广义约束力一致 |
| independent NPZ重算 | 时间、train-only统计、normalization、ablation切片、noise隔离通过 |
| Modify σ=.2N | clean轨迹/统计逐值不变，仅三力轴按seed221改变 |
| 防御检查 | NaN noise参数exit2；Python3.11语法通过；actual运行3.12 |

首轮paired用ctrl=0，向上伺服使微小穿透处求解力为0，断言失败；修正为两快照相同向下ctrl后重跑通过。
这也说明几何接近或存在contact记录不等于承载正法向力。
图已目视检查：瞬态软接触允许穿透，不能把每个峰值解释成准静态压紧力。
按raw槽比较的paired距离仅用于是否相同，不是混合m/m/s/N的有意义物理距离或策略质量指标。
本例force/touch部分冗余；更多通道不自动增加可辨识信息，也未测policy表现。

## Failure Cases / Robotics Context

frame用错转置、geom符号反转会颠倒摩擦/法向力；只取第一个contact会漏掉其他点；用全部ncon会误收其他物体。
积分后不刷新sensor会造成错时。terminal行没有下一动作；未来recorder须保持N转移与N+1观测关系。
归一化后置0意味着train mean，不能假装通道被移除；本节按索引真实缩小输入shape。
用validation拟合normalizer、用contact truth假装硬件传感器、把touch当压力或布尔值都违背合同。
100Hz读取可能漏掉1kHz瞬态峰值；有限测试不证明硬实时或硬件安全。
机器人应用包括表面跟随、插装与抓持接触辨别；后续学习策略前先确认观测可得性与时间定义。

## Interview Capsule

**30秒：** 接触学习先定义观测合同。我把关节位置速度、任务目标、world接触力和pad法向标量按同一时间戳拼接，说明每项的单位、坐标系和来源，只用train做归一化。同关节状态可能对应不同接触，因此state-only存在歧义。

**2分钟：** 补充contact局部坐标转world及geom作用对象的符号，解释mj_step后用mj_forward刷新同一状态，区分即将执行goal与上一interval产生的力。演示6/9/7/10维通道移除和同状态反例。本例touch与Fz冗余，摩擦信息来自Fx；只是观测可辨识性验证，未训练策略。force源是仿真真值，部署需要替换与标定，噪声和延迟应分别验证，不能由该实验推断硬件安全或泛化。

## Must Remember / My Verification / Run → Modify → Explain

学习状态仅根README记录；2026-10-12 本人确认实验与预测完成，五项Explain通过，Learning Mastered。
精度补充：归一化后填0对应train mean，是保留维度的固定值消融；可以消除该通道样本变化，但不等同于移除通道。
本例两个paired快照ctrl也相同，差异来自表面高度；noise注入观测而非动力学，所以物理轨迹不变。

**Run：** 执行上面的my_run命令，打开contract.png，对照CSV与results.json找出approach/press/slide/release。

**Modify：** 先预测加噪后哪些数据会变、哪些不会，再执行：

```bash
python examples/23_contact_aware_learning/observation_contract.py \
  --force-noise .2 --output tmp/s22_1/my_noise02
```

比较两个NPZ的validation/measured/mean/scale，以及噪声图；解释为什么运动不变，Fy测量会变而clean Fy为0。

**Explain（五问）：**

1. 为什么paired的q/qdot/goal甚至ctrl完全相同，接触力仍然不同？本例qz=−30mm对应哪个world高度？
2. 为什么转换contact力使用frame.T，何时取负号？这里怎样独立检查力方向？
3. touch到底测什么，为什么此处约等于Fz、却不等于合力模长？它有哪些部署限制？
4. 一行的q、力和touch属于哪个时间，goal什么时候执行？为什么mj_step之后还要mj_forward？
5. 为什么normalizer只拟合train？移除通道与归一化后填0有何不同？σ=.2N为什么不改变机器人轨迹？

下一小任务S22.2示范recorder，等待本人明确请求。
