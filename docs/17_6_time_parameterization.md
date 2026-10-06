# S14.6 — Cubic Time Parameterization

[代码](../examples/15_motion_planning/time_parameterization.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。前置：[S14.5](17_5_path_smoothing.md)，
复用[已有cubic参考](../examples/11_trajectory/cubic.py)。

## Problem → Why

规划/shortcut输出一串几何waypoint，没有“什么时候到达”和“运动多快”。
现在给每条边分配时间，满足每个轴的速度与加速度上限，并在每个waypoint停到零速度。
只生成并验证参考，不执行actuator tracking，不自动接入UR5e。

## Intuition → Core Concepts

同一条路线可以快走或慢走。用同一个单调标量s(t)控制一段的全部关节，
使这一段保持原有直线几何；时间足够长则速度/加速度降到上限内。
每个waypoint停车，允许下一段换方向而不产生速度跳变。
停点是速度瞬时为零，不是插入正时长的休息；本课没有普通waypoint dwell。

| 概念 | 本课政策 |
| --- | --- |
| path | S14.5固定几何折线，shape (3,2)，world XY slide米 |
| trajectory | q(t)、qdot(t)、qddot(t)；参考值，无动力学执行 |
| limits | 各轴速度/加速度的绝对值上限，非向量范数上限 |
| segment timing | 同一段全部轴共享T，按最严格的轴/限制选T |
| continuity | 段间位置和速度连续C1；加速度一般不连续，不是C2 |

## Mathematics：形状、单位、frame

对一段q_a→q_b，Δq=q_b−q_a，shape `(2,)`，m；T为s，τ=t/T无量纲：

```text
s(τ) = 3τ²−2τ³
q(t) = q_a + s(τ) Δq                   (m)
qdot(t) = (6τ−6τ²) Δq / T              (m/s)
qddot(t) = (6−12τ) Δq / T²             (m/s²)
```

τ∈[0,1]，s单调从0到1，q保持原线段，qdot两端为零。
`max(6τ−6τ²)=1.5`在τ=.5；`max|6−12τ|=6`在两端，故对轴j：

```text
max |qdot_j| = 1.5 |Δq_j| / T
max |qddot_j| = 6 |Δq_j| / T²
T_v,j = 1.5 |Δq_j| / v_max,j
T_a,j = sqrt(6 |Δq_j| / a_max,j)
T_min = max_j(T_v,j, T_a,j)
T = max(1, ceil(T_min / dt)) dt         # 向上舍入到采样网格，dt=.01 s
```

这是给定零端速cubic族内的最小时长界，不是任意时间律的全局最短时间。
默认v_max=[.3,.2] m/s，a_max=[.6,.4] m/s²，均为教学设定，不是硬件规格。
UR5e旋转关节对应rad、rad/s、rad/s²，同样公式，但需要实际关节limits与碰撞模型。

一个轴Δq=0不要求额外时间；整段Δq=0时用一个dt静止hold，避免零时长除零。
相邻段的加速度左右极限：

```text
qddot_left  = -6 Δq_previous / T_previous²
qddot_right = +6 Δq_next / T_next²
```

它们通常不同。速度为零可以连续，加速度仍会跳；起终点若接stationary hold也有加速度跳。
有限加速度上限不限制jerk，跳变对应理想化jerk奇异性，不表示真实硬件可跟踪。

## Math-to-Code / APIs

按`time_path → main`阅读，关键计算保持显式。

1. `main`固定RRT-Connect seed7/h=.005，shortcut seed17/100，独立生成同一3点path，无保存文件依赖。
2. `time_path`按各轴解析峰值选T_min，向上舍入到dt；总采样限制200000个区间，超限拒绝。
3. 调用S10.8b的`comparison_trajectories(a,b,T,dt)`，取返回的cubic `(q,qdot,qddot)`。
   helper返回time和linear/cubic三元组；本课丢弃linear，删除两端额外hold样本，仅保留运动区间。
   helper内部没有模型加载/仿真，但其module顶层导入mujoco-menagerie，因此本课它也是必要import依赖。
4. 段间拼接去掉下一段重复t=0，global time严格递增、相邻间隔dt。
   knot处保存上一段加速度（左极限）；JSON另存左右极限与jump，不能把NPZ单行当成双侧值。
5. 速度峰值由解析式核验，不用“离散采样最大值”代替连续时间上界。奇数区间可能没有τ=.5采样点。
6. 重检path每条几何边，并检查全部时间采样配置。独立圆盘解析评分只验证几何，不参与配时。

MuJoCo API复用前课：`MjData(model)`创建可写状态；`mj_forward(model,data)`返回None，
原地更新FK/contact/cache，query不推进时间。live qpos/qvel/time/xpos保持不变。
本课不调用mj_step，没有ctrl跟踪、扭矩/力或实际动态安全判据。

## Minimal Experiment → Expected / Actual Result

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/time_parameterization.py
python examples/15_motion_planning/time_parameterization.py --velocity-scale 0.5
```

必要依赖：已声明NumPy / official MuJoCo / Matplotlib / mujoco-menagerie。
2026-10-06，DESKTOP-781D67A：Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2、
Menagerie2026.9.1；无新增/安装依赖，未加载UR5e资产；3.11兼容目标未在3.11执行。

| case | T1 / T2 | 总时间 / 样本数 | 起作用的主要限制 |
| --- | --- | --- | --- |
| default | 5.77 / 2.24 s | 8.01 s / 802 | x轴速度 |
| velocity×.5 | 11.54 / 4.47 s | 16.01 s / 1602 | x轴速度 |
| acceleration×.1（额外工程对照） | 10.75 / 6.68 s | 17.43 s / 1744 | x轴加速度 |

三命令均exit0，无import error。三组path逐元素相同、长度1.612792500m，
几何最小clearance28.191341mm不变；时间变化不改变静态几何安全。
默认每段解析peak |vx|=.299950/.298789 m/s，peak |ax|=.207938/.533552 m/s²；
连接time5.77s、速度两侧均0，加速度jump=[.741490,.092506] m/s²，明确非C2。
peak acceleration限制是各段左右单侧值，并不把jump幅度当某一时刻的加速度峰值。

速度上限减半在本case大致使时间加倍、加速度约变为1/4；dt向上取整导致总时间不是精确2倍。
一般若加速度限制原本主导，减小速度limit不一定马上改变T。
加速度限制主导时，限制缩放k使T按1/sqrt(k)增长（忽略网格舍入），不是1/k。

保存ignored `tmp/s14_6_timing_*/trajectory.npz`、`results.json`、`timing.png`。
NPZ的q/qdot/qddot shapes为 `(M,2)`，time `(M,)`；位置米、速度m/s、加速度m/s²。
图三行显示q/qdot/qddot与各轴limit，灰线是连接时刻；默认图已目视检查。
独立三组解析峰值/配时ceil/poly replay、knot左侧约定、时间严格递增/dt、
路径/长度不变、全部时间采样几何、零位移/加速度主导与输入/预算guards通过；CLI非法scale退出2。

## Explanation → Failure Cases

- 只按速度配时：短段可能超过加速度上限，必须同时取max(T_v,T_a)。
- 各轴单独配不同T：会离开原几何直线，原path碰撞检查不能直接复用。
- T向下取整：可能破坏极限；向上到dt网格。
- 只看采样peak：奇数间隔可能漏τ=.5，使用解析峰值证明cubic段内限速。
- duplicate knot时间：把两段都包含的端点直接拼接，会重复time；保存单时刻并单独记录两侧加速度。
- “停车等于C2”：零端速只保证C1，cubic端加速度非零；本课不引入quintic/jerk规划。
- “参考限速等于硬件安全”：没有扭矩、重力、接触、动态障碍或tracking误差验证。
- “总路程不变就安全”：只对静态模型保持同一几何线段成立；动态障碍会依赖时间。

## Robotics Context → Interview Capsule

机器人执行需要带时间的参考。停点策略容易解释并适合本阶段，但会在每个waypoint减速至零，
增加时间且仍有加速度跳变。后续UR5e整合必须分别检查参考几何、关节limits和实际tracking。

**30秒**：给每段采用零端速cubic，从解析峰值推导T≥1.5|Δq|/v_max和T≥sqrt(6|Δq|/a_max)，
取各轴最大并向上舍入。相同s(t)保留原几何，每个waypoint停点，位置速度连续但加速度通常跳变。
这是满足运动学参考限制，不是硬件执行证明。

**2分钟**：先固定已检查的path，再对每段分配共享时间。cubic s单调沿原边推进，
导数峰值给出速度与加速度约束，两者和各轴取max才够长。向上到dt网格，复用已有cubic helper，
解析核验而非依赖采样。拼接只保留一次knot，明确其加速度左值，并单独存左右jump。
三组实验验证限速/限加速度改变时间不改变几何。停点保证C1，非C2且不限制jerk；
没有动力学执行、扭矩能力或时变环境结论。

## Must Remember → Run / Modify / Explain

必须记住：共享段时间；速度和加速度双约束；解析peak；T向上取整；停点C1≠C2；参考≠执行。

**Run**：默认命令，对照各段duration、analytic peaks、knot的左右加速度和图。

**Modify**：先预测`--velocity-scale 0.5`对时长、速度、加速度、path和clearance的影响，再运行对照。

**Explain（五问）**：
1. 为什么T要取所有轴的速度与加速度约束最大值？1.5与6来自哪里？
2. 为什么同段全部轴共享s(t)/T才能保持已检查的几何边？
3. 为什么每个waypoint零速度能保证C1，却通常不是C2？
4. 为什么使用解析peak和向上舍入，不能只看采样peak或向下取整？
5. velocity limit减半一定使时间精确加倍吗？参考通过为何仍不证明实际执行安全？

**My Verification**：本人于2026-10-06明确确认实验与预测完成，五项Explain正确覆盖1.5/6解析峰值、各轴双约束最大值、共享时间律保持几何、停点C1非C2、向上舍入与参考/执行边界，Learning Mastered；状态只在根README维护。

精度补充：1.5是ds/dτ的峰值、6是|d²s/dτ²|的峰值；转换到时间导数后分别除以T和T²。若原段速度主导，限速减半使未量化最小时长翻倍；若原段加速度主导，新的速度限制仍可能成为主导，需重新取双约束最大值。
