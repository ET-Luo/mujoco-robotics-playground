# S14.2 — Configuration Space / Edge Checking

[代码](../examples/15_motion_planning/edge_checking.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。前置：[S14.1](17_motion_planning.md)。

## Problem → Why

S14.1能够判断单个配置。现在要判断从q_start到q_goal的一段连接是否可走。
如果只检查两端，即使两端合法，中间也可能撞障碍。
本课学习配置空间、距离、线性插值和检查分辨率；不实现RRT，也不给路径分配时间。

## Intuition → Core Concepts

机器人配置q就是确定几何姿态所需的关节坐标。二维模型有x/y两个slide，
因此q=(x,y)直接对应一个球体中心的world xy，单位都是米。
一般UR5e的q是六个关节角，配置空间与workspace不是同一空间。

把有半径的机器人缩成一个点，同时把障碍扩大，就得到这个特例的C-space障碍。
配置空间中的一条线，是“一串机器人姿态”，未必是一般机械臂末端在workspace里的直线。

| 概念 | 本课含义 |
| --- | --- |
| C-space | q∈[-1,1]² m，每一点代表一个完整机器人配置 |
| C-obstacle | 与物理障碍触碰或穿透的配置集合，闭圆盘 |
| C-free | joint bounds内、C-obstacle外的配置 |
| edge | 两个q之间的直线连接，无时间信息 |
| path | 多段edge的序列；本课手动提供绕行waypoints |
| resolution | 相邻检查点的最大配置空间距离，不是仿真dt |

## Mathematics：形状、单位、frame

球形机器人r=0.020 m，球形障碍r_o=0.025 m，中心c=[0.13,0] m。
二者高度都是world z=0.1 m，因此平面中心距离足以决定球–球碰撞。

```text
q, c ∈ R²  (shape (2,), world xy in m)
C_obstacle = {q : ||q-c||₂ ≤ r+r_o}，半径0.045 m
valid(q) = joint_bounds(q) AND ||q-c||₂ > 0.045
D = ||q_goal-q_start||₂  (m)
n = max(1, ceil(D/h))   (整数分段数)
q_i = q_start + (i/n)(q_goal-q_start), i=0,…,n
实际相邻距离 = D/n ≤ h
```

正常边有n+1个样本，包含两个端点。D=0的零长度边只检查一次，不能因为没有移动就跳过碰撞。
样本数不是`ceil(D/h)`；那是段数。NumPy广播用 `(n+1,1)*(2,)` 生成 `(n+1,2)` 配置数组。

本课Euclidean metric成立，因为两个维度同为米、平移轴正交。
混合m/rad不能直接解释成物理米距离，应明确scale/weights。
有界hinge必须遵守真实joint limits；真正周期角可考虑最短角差与对应插值，
不能只更换距离公式却保留不一致的插值。本课没有角度或wrap-around。

**独立解析评分**：将障碍中心投影到线段并截断至端点：

```text
v = b-a
α* = clip(((c-a)·v)/(v·v), 0, 1)    # v=0时取0
minimum_clearance = ||a+α*v-c||₂ - (r+r_o)
```

正值表示这条线段与圆盘严格分离；零是相切，负值表示进入圆盘。
这是本课球体+直线特例的连续几何答案，仅用于scorer，不输入采样判定。
joint bounds是凸矩形，两端在bounds内就能证明直线全段在bounds内；这不证明避开非凸C-free。

## Math-to-Code / APIs

按 `vector → configuration_report → check_edge → exact_segment_clearance → main` 阅读。

- `vector(q)`明确shape `(2,)`和有限数；错误数据抛ValueError，越界配置返回无效。
- `configuration_report`保留S14.1的独立MjData→写qpos→mj_forward→解释contact的流程。
  二维新fixture不需要原S14.1的四维phase/held接口，因此没有复用它的模型或参数。
- `MjModel.from_xml_string(xml)`编译MJCF、返回model；模型中仅两个有界slide与两个sphere。
- `MjData(model)`返回独立状态，qpos/qvel均shape `(2,)`，单位m / m·s⁻¹。
- `mj_forward(model, query)`返回None，原地更新FK、contacts和动力学cache；不积分或推进time。
  本课没有mj_step、ctrl轨迹、GUI或接触力判断。
- `check_edge`以配置Euclidean长度确定段数，检查所有点，保存first_invalid_index和每点原因。
  为展示完整采样，发现碰撞后仍检查剩余点；未来规划器可以保留相同规则并提前返回。
- `sampled_valid`只来自MuJoCo配置查询；`exact_valid`来自独立解析scorer+joint bounds。
  这两个字段分开保存，不能用scorer悄悄纠正采样结果。

每条边有10000段预算，超限明确抛错；CLI步长限制为[0.005,1] m。
live初始time=2 s、非零qvel用于验证隔离；所有query新建、时间未推进。

## Minimal Experiment → Expected / Actual Result

依赖：requirements已声明NumPy、official MuJoCo和Matplotlib。无新依赖、无需前课产物。
从项目根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/edge_checking.py
python examples/15_motion_planning/edge_checking.py --step-m 0.02
```

直线a=[−0.8,0]、b=[0.8,0] m，长度1.6 m，两端都合法。
手动绕行路径：a→[−0.8,0.25]→[0.8,0.25]→b，长度2.1 m，非自动搜索结果。

2026-10-06，DESKTOP-781D67A；Python3.12.14 / NumPy2.5.3 / MuJoCo3.14.0 / Matplotlib3.11.2。
两命令exit0，无import error；代码3.11兼容目标未在3.11执行。

| edge | h=0.4 m采样 | h=0.02 m采样 | 连续解析答案 |
| --- | --- | --- | --- |
| direct | 5点，通过（漏检） | 81点，拒绝 | 拒绝，最小clearance −45 mm |
| up | 2点，通过 | 14点，通过 | 通过，clearance 885 mm |
| across | 5点，通过 | 81点，通过 | 通过，clearance 205 mm |
| down | 2点，通过 | 14点，通过 | 通过，clearance 625 mm |
| zero_clear | 1点，通过 | 1点，通过 | 通过 |
| zero_blocked | 1点，拒绝 | 1点，拒绝 | 拒绝 |
| out_of_bounds | 2点，拒绝 | 17点，拒绝 | 拒绝，末端x=1.1 m |

JSON和C-space PNG在ignored `tmp/s14_2_edges_step*/`。蓝线是直连，绿线是手动绕行，
红盘为扩大的C-obstacle，虚线圆仅显示物理障碍半径；红叉是被查询检测出的非法采样点。
这是配置空间示意图，不是相机图或动态机器人画面。

## Explanation → Failure Cases

默认采样x=[−0.8,−0.4,0,0.4,0.8] m，障碍圆盘在y=0线上覆盖x∈[0.085,0.175] m，
没有一个采样点落入圆盘，因此采样判定误接受。细步长有x=0.1/0.12/0.14/0.16，检测到碰撞。
首次非法点是x=0.1 m（index45）；contact.dist=−0.015 m。

- **更细不是全局证明**：本次20mm捕获该穿越；对任意薄障碍、擦边或复杂机器人，任何固定正步长仍可能漏检。
- **采样网格变化**：改变h会改变ceil(D/h)，样本位置通常不是嵌套的，不能无条件说每次减小h都保留旧检测点。
- **零长度**：恰在障碍内必须拒绝，不能把零长度默认当安全。
- **越界**：碰撞几何clearance为正也可能违反关节范围，报告里要区分原因。
- **预算**：超预算代表检查没有完成，不能返回通过。
- **浮点相切**：严格接触边界易受数值roundoff影响；本课主要用例远离相切，不声称边界安全保证。
- **模型边界**：碰撞过滤/几何近似仍适用；真实机器人需要S14.1中的完整几何和phase/held policy。
- **路径与轨迹**：h是空间检查分辨率，dt是时间积分间隔；边没有速度、加速度或tracking结论。

## Robotics Context → Interview Capsule

之后RRT每次新增树边都需要调用edge checker；shortcut平滑也必须重新查整条替代边。
UR5e的joint-space线段能产生弯曲的workspace末端轨迹，关节分辨率与几何运动幅度并不等价。
本课二维配置空间等于中心xy只是一种便于理解的特例。

**30秒**：configuration validity检查单点；edge validity检查连接。
我按Euclidean配置距离和最大步长划分线段，包含两个端点，对每点用独立MuJoCo状态查碰撞和joint limits。
固定采样可能漏薄障碍，所以结果命名sampled_valid，并用圆盘解析距离展示反例。

**2分钟**：配置空间点编码完整关节状态。二维slide机器人中，圆形机器人使物理圆障碍
在C-space膨胀成半径和的圆盘。长度D的边按ceil(D/h)分段，n+1个点包含两端，
零长度也查一次。合法端点不保证中间避障，默认粗采样跨过圆盘而误接受；
细采样检测出非法中间点，手动绕行三段与独立解析投影评分都通过。
检查器与scorer隔离，保留漏检证据。一般机器人无法仅用球体公式保证连续安全，
需要明确分辨率、碰撞模型、任务phase和持物姿态，还要另行验证时间化与执行。

## Must Remember → Run / Modify / Explain

必须记住：配置空间不等于workspace；n段=n+1点；检查端点和零长度；空间步长≠dt；采样通过≠连续保证。

**Run**：运行默认命令，对照direct的sampled=True/exact=False与C-space图。

**Modify**：预测`--step-m 0.02`的direct结果、样本数与几何路径长度变化，再运行对照。
预期改变采样密度与判定，路径长度及解析clearance不变。

**Explain（五问）**：
1. 为什么本课q的两个维度用米，而UR5e通常用rad？C-space和workspace有何区别？
2. 为什么C-obstacle半径是20+25=45 mm？只检查物理障碍25 mm会漏掉什么？
3. 长度1.6 m、h=0.02 m为何需要81点？零长度边如何处理？
4. 为什么0.4 m漏检而0.02 m捕获？有限步长是否能保证任意路径连续无碰撞？
5. 为什么本课绕行仍不是RRT结果或可执行trajectory？joint limits通过与避障通过有何区别？

**My Verification**：Engineering已验证；本人Run/Modify/Explain待确认，状态仅在根README维护。
