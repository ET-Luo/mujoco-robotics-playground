# S17.2 — Cartesian impedance：给虚拟弹簧加阻尼

约0.5～2h。前置：[S17.1](20_1_cartesian_spring.md)。
[代码](../examples/18_compliant_control/cartesian_impedance.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning状态只见[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

S17.1的工具会越过目标、不断振荡。怎样使它回到目标并停下来？
保留位移产生的弹簧力，加一个与工具速度反向的力。工具往目标运动时，
弹簧拉向目标，阻尼则抵抗运动；越过目标后，两者可能同时帮助刹车。
阻尼不总指向目标，它针对速度，作用是耗能。
本课完成固定reference、自由空间的平移阻抗控制；未引入接触、admittance或增益矩阵扫描。

## Core Concepts

复用S17.1 XML、dt、initial、gear与cap；直接import该模块的常量，不调用上一课main/run。
因此local helper依赖链也是mujoco/numpy/matplotlib（Agg）；无Menagerie/ROS/RL或新依赖。
两slide保持工具姿态不变，x_W=[q_x,q_y,0]m，M=diag(2,1)kg。
gravity/contact/passive damping均为零，阻尼完全由显式motor速度反馈产生。
固定x_d=0、v_d=0；初始q=[.04,−.03]m、qdot=0；执行循环不重设qpos/qvel。

## Mathematics：shape / unit / frame

\[
v_W=J_{p,W}\dot q,\quad
F_{cmd,W}=K(x_{d,W}-x_W)+D(v_{d,W}-v_W),\quad
\tau_{cmd}=J_{p,W}^{\mathsf T}F_{cmd,W}.
\]

| 量 | 数学shape / NumPyshape | 意义与单位 |
| --- | --- | --- |
| x、x_d、v、v_d | 3×1 / (3,) | world位置m、world线速度m/s |
| K | 3×3 / 标量100实现 | 对称刚度100N/m，各轴相同 |
| D | 3×3 / (3,)逐分量相乘 | world对角速度反馈增益，N·s/m；Z项为0 |
| J_p | 3×2 / (3,2) | world工具线速度映射，slide列m/m |
| τ_cmd | 2×1 / (2,) | slide广义力N；若有hinge对应项为N·m |

本fixture J_p=[[1,0],[0,1],[0,0]]，J_r=0；结构固定orientation，没有姿态feedback。
未饱和、固定目标、无外力时，每轴满足
\[
m_i\ddot q_i+d_i\dot q_i+kq_i=0,\quad
\omega_{n,i}=\sqrt{k/m_i},\quad
\zeta_i=\frac{d_i}{2\sqrt{m_i k}}.
\]

默认d_i=2√(m_i k)，两轴ζ=1，即连续模型临界阻尼。
X有效质量2kg、Y为1kg，故D_xy=[28.284271,20]N·s/m；不能把相同D自动称为两轴同ζ。
`--damping-scale .5`只把D减半，ζ=.5；K、mass、initial、dt、cap均不变。
M是机器人真实有效质量，未人为指定虚拟mass或做惯量整形。
这里只能声称该fixture呈现指定弹簧/阻尼；一般机器人还有配置相关惯量、bias/重力与冗余问题。

能量E=½qdotᵀM qdot+½(x−x_d)ᵀK(x−x_d)。未饱和连续模型：
\[
\dot E=-v_W^{\mathsf T}D v_W\leq0.
\]

这是阻尼耗能的原因，而不是“目标处力为零所以停下”。若速度非零，
即便x=x_d，F_d=−Dv仍非零。错误使用+Dv会输入能量。
离散数值E不要求每一步严格下降，也不能把上述连续关系直接当所有dt下的稳定性保证。

## Math-to-Code / APIs

```python
velocity = jp @ data.qvel
spring = K * (target-position)
damper = -damping * velocity
requested = jp.T @ (spring+damper)
data.ctrl[:] = requested / GEAR
mujoco.mj_step(model, data)
```

这里控制输入为运动reference，输出为motor广义力；不需要先测量接触力才能计算阻抗律。
不能称为“测力产生reference”的admittance，也没有normal force target。

MjModel.from_xml_string(XML)返回编译模型；MjData(model)返回可变状态和计算buffer。
mj_forward(model,data)原地更新当前FK、site pose与动力学，不推进time。
mj_jacSite(model,data,jp,jr,site_id)将当前site原点的world线/角Jacobian写入(3,nv)buffer，
不是返回新的J；先forward再调用，v=Jp@qvel的结果为(3,)m/s。
mj_fullM(model,data,dst)原地展开(nv,nv)惯量，用于核验M与动力学预算。
mj_step(model,data)原地更新真实qpos/qvel/time；controller每1ms重算一次。
复用S17.1 Euler积分与API；[官方Python原地语义](https://github.com/google-deepmind/mujoco/blob/main/doc/python.rst)、
[官方积分说明](https://github.com/google-deepmind/mujoco/blob/main/doc/computation/index.rst)。

gear=[2,1]、gain=1，actual generalized force=gear×actuator_force。
joint force cap±20N，scalar forcerange分别±10/±20；ctrl本身不裁剪。
本课每步检查actual=clip(requested,−cap,+cap)，两组均无饱和。
真实饱和probe已由S17.1验证；本课没有专门制造动态饱和，不能声称验证了饱和恢复。

## Minimal Experiment / Expected → Actual

从仓库根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/cartesian_impedance.py
python examples/18_compliant_control/cartesian_impedance.py --damping-scale .5
```

Expected：纯spring持续振荡；ζ=1平滑恢复；ζ=.5出现衰减振荡，能量仍消耗。
每命令保存spring与impedance两个case，4001×19 CSV，pre-step t=0…4s。
settling定义为首次满足两轴|位置|<1mm、|速度|<2mm/s且直到4s记录末尾始终满足；
不是“第一次过零”，也不是无限未来保证。tail为最后.5s。

Actual（2026-10-08，Zero；两命令exit0，无import error）：

| case | D_xy N·s/m | settling s | 越过目标比例X/Y | 峰值速度m/s | tail最大位置/速度 |
| --- | --- | --- | --- | --- | --- |
| spring | [0,0] | 未settle | ≈100% / ≈100% | .300004 | .0400002m / .300004m/s |
| impedance ζ=1 | [28.2843,20] | .976 | 0 / 0 | .110550 | 3.09e−11m / 2.02e−10m/s |
| impedance ζ=.5 | [14.1421,10] | 1.381 | 16.2337% / 16.2047% | .164106 | 1.19e−7m / 6.72e−7m/s |

“越过目标比例”为目标另一侧最大位移幅度除以该轴初始偏移绝对值；
整体peak position会包含初始40mm，不能拿它代替overshoot。
两组impedance峰值actual joint force均4N，初始速度0所以初始阻尼力0；动态无饱和。
初始E=.125J；ζ=1/.5末E=1.32e−22/2.56e−14J，纯spring末E=.125219J。
连续解析解最大位置差：spring .154911mm、ζ=1 .093510mm、ζ=.5 .122545mm。
惯量/固定姿态/J与速度/功率/actual motor/非正阻尼功率/动力学残差检查通过（残差0N）。
脚本独立对照临界阻尼解q=q₀(1+ω_nt)e^(−ω_nt)和欠阻尼解析解。
PNG展示position、velocity、X弹簧/阻尼/actual force及E；JSON保留参数和失败对照metrics。
结果只写ignored tmp/s17_2_d1、tmp/s17_2_d0.5，无新依赖或requirements改动。
独立CSV半隐式recurrence逐点重算、控制律/能量预算/settling核对通过；
两组spring CSV逐元素相同。错误阻尼符号静态功率检出，未运行负阻尼动态轨迹。
CLI damping-scale nan/inf/0/−1各exit2；默认PNG已目视核验，文件链接与git diff --check通过。

## Explanation / Failure Cases

默认ζ=1时阻尼可以抵消部分弹簧力，减小加速并避免越过目标。
减半D后两轴都会越过目标，但阻尼仍持续移除能量；这次恢复较慢，不表示所有增益中D越大越快。
过大D也可能使响应迟缓；完整K/D/dt扫描留S17.3。
错误阻尼符号会使F_d·v>0；误用joint qvel当一般机器人的Cartesian velocity会混单位与frame。
一般必须计算Jp@qvel，而本fixture两轴数值恰巧相同。
仅检查位置可把高速过零误当到位；settling应同时考察速度与持续窗口。
较大dt、执行器饱和、噪声或延迟都可能破坏理想耗能关系；本课没有测力闭环或硬件验证。

## Robotics Context / Interview Capsule

位置reference通过刚度/阻尼规定机器人对偏移与运动的响应，适合引出工具顺应性。
自由空间恢复通过是接触实验的前提，不能直接视为接触冲击小或指定接触力已实现。
未做UR5e、contact、GUI、ROS或硬件验证；补偿项在本零gravity/零bias fixture中为零。

30秒：在F=K(x_d−x)中加入D(v_d−v)，v来自world site Jacobian乘qvel。
再用Jᵀ映射到有界motor。固定目标下阻尼功率−vᵀDv≤0，工具因此逐渐停下。

2分钟：说明reference/frame/shape/单位；写出M qdd+D qdot+Kq=0与ζ。
解释X/Y质量2/1kg导致临界D不同，默认ζ=1、半D为ζ=.5。描述motor gear/force cap，
用连续解、动力学预算、阻尼功率交叉检查；settling需要位置与速度持续满足门限。
最后说明真实机器人惯量/重力、离散dt、接触与饱和超出本自由空间验证。

## Must Remember / My Verification — Run / Modify / Explain

弹簧针对位移，阻尼针对速度；阻尼力不总朝目标。v=Jp qvel；Jᵀ映射广义力。
连续模型耗能并不等于任意离散实现稳定。README工程完成，Learning三项等待本人报告。

Run：运行默认命令，查看tmp/s17_2_d1/response.png，对照spring与impedance。
Modify：先预测再仅把D减半（--damping-scale .5），比较越过目标幅度、速度、settling和能量。
Explain：

1. 为什么阻尼项是−Dv？工具向目标运动时，弹簧与阻尼方向分别怎样？
2. 怎样由v=Jp qvel得到world site速度？qvel与Cartesian速度在一般机器人上能直接等同吗？
3. 怎样推导E_dot=−vᵀDv？为什么增加刚度不能代替阻尼？
4. 为什么X/Y临界D不同？D减半后ζ与越过目标行为怎样变化？
5. 为什么settling要同时检查位置和速度？本课自由空间结果能否证明接触/饱和/硬件稳定性？

现场环境：WSL2 Ubuntu24.04.5，Python3.12.14/MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2，
同shell核验mujoco环境/所属interpreter、metadata与真实module paths。无安装；未在3.11运行。
不自动实现S17.3，等本人完成本课学习并明确请求。
