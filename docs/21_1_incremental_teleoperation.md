# S18.1 — Cartesian incremental teleoperation

约0.5～2h；前置：[Stage17](20_compliant_contact_control.md)。
[代码](../examples/19_teleoperation_dexterous/incremental_reference.py) / [示例README](../examples/19_teleoperation_dexterous/README.md)。
Engineering / Learning状态唯一来源：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

手柄向右移1mm，应让机器人参考向哪里移、最多移多少？手柄坐标通常与机器人world不同，绝对读数也不等于机器人位置。
本课只取相邻master样本的位移，再转成world增量，累加到独立的机器人运动参考上。
直觉：master报告“刚才移动了多少”，reference决定“从当前参考再走多少”。超过速度或workspace的部分直接丢弃，不排队补走。

## Core Concepts / scope

synthetic master固定20ms采样，50Hz；每个位置为master frame M中的(3,)米。
初始master=[.30,−.20,.10]m，robot world参考锚点=[−.45,.20,.30]m。
固定unit motion scale=1；本课不实现可调motion scale、clutch/recenter或姿态遥操作，留S18.2及后续任务。
`--master-amplitude 2`改变合成master的运动幅度，不改变mapper的motion scale。
输出只是Cartesian reference；没有实际robot state、MuJoCo、motor、IK、ROS、接触、GUI或haptic device。
现有Stage17 controller的实际跟踪与接触响应留S18.3。本课数值实验也必须在conda mujoco执行。
依赖仅requirements已有NumPy/Matplotlib，无local helper或新依赖；使用Agg输出静态图。

## Mathematics：frame / shape / unit

R_WM的**列**是master各轴在world中的方向；本课为绕world Z的+90°旋转：

```text
R_WM = [[0,−1,0], [1,0,0], [0,0,1]]
+master X -> +world Y
+master Y -> −world X
+master Z -> +world Z
Δm_M[k] = m_M[k] − m_M[k−1]                 # (3,) m
Δx_req_W[k] = R_WM Δm_M[k]                  # (3,) m; scale=1
α = min(1, v_max Δt / ||Δx_req_W||₂)        # scalar; zero displacement uses α=1
Δx_speed_W = α Δx_req_W
x_candidate_W = x_ref_W[k−1] + Δx_speed_W
x_ref_W[k] = clip(x_candidate_W, lower_W, upper_W)
Δx_applied_W = x_ref_W[k] − x_ref_W[k−1]
v_sample = ||Δx_applied_W||₂ / Δt           # scalar m/s
```

R为(3,3)、无量纲、右手正交矩阵，RᵀR=I、detR=1。增量是向量，只旋转，不加frame原点。
绝对位置变换需要原点，但这里相减已消去master的常量位置偏置；robot reference依然需要独立初始锚点。
R_WMᵀ也合法正交，却表达反方向变换；仅检查det=1不能证明frame语义正确，因此代码另外检查具名轴方向。

world box bounds：lower=[−.49,.16,.28]m、upper=[−.41,.24,.32]m。
v_max=.02m/s，Δt=.02s，因此每帧reference增量norm≤.0004m=0.4mm。
workspace裁剪采用world逐轴clip；因为旧reference已在box内，每轴实际变化不会超过speed-limited增量，故norm速度界仍成立。
旧reference若在box外，函数直接拒绝；不能用一次大跳变“修复”后宣称满足速度界。

norm限幅用一个α保持requested方向；逐轴限制速度会让对角线norm超过v_max，例如三个轴均v_max时norm=√3 v_max。
workspace投影可改变方向：world Y已顶住边界时，斜向输入仍可沿未受限的X方向运动，不能说最终applied方向始终不变。

这只是样本间的平均增量速度界，不是零阶保持阶跃的连续时间导数界，也没有加速度/jerk限制。
图中连线用于展示样本关系；未实现独立高频插值、设备采样抖动或输入过期策略。

## Math-to-Code / APIs

`synthetic_master(amplitude)`返回times(551,)秒、master_positions(551,3)米、interval phases(550,)；初始样本0s，末尾11s。
`update_reference(...)`是pure function：reference/previous/current/bounds均(3,)，rotation(3,3)，dt秒、speed_cap米/秒。
返回新reference(3,)及诊断dict；不修改输入，不写实际qpos。输入或frame非法抛ValueError，调用者保留原状态。
`np.linalg.norm`返回增量Euclidean长度；`np.clip`返回新数组，本课不使用out参数修改传入reference。
调用者每帧把current master作为下一帧previous，包括被限幅拒绝的运动；否则会制造隐藏的累积欠账。

无约束时增量累加可望远镜求和：x_ref[k]=x_ref[0]+R(m[k]−m[0])。
有裁剪时两者不再等价。若始终把该绝对锚定目标clip到box，超出的目标仍在box外，master反向时可能还粘住边界；
本课逐帧只应用实际允许的增量，拒绝部分丢弃，所以反向第一帧即可向内移动。
这不表示所有绝对遥操作模式都错误；它是本课选择的增量语义。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/incremental_reference.py
python examples/19_teleoperation_dexterous/incremental_reference.py --master-amplitude 2
```

| master时段 | amplitude1速度，M frame | 观察目标 |
| --- | --- | --- |
| 0…2s | +X10mm/s | +world Y慢移，无速度限幅 |
| 2…4s | +X80mm/s | norm限幅到20mm/s，再受world Y上界限制 |
| 4…6s | −X10mm/s | 立即离开边界，不补偿被拒绝的位移 |
| 6…8s | +Y10mm/s | −world X方向 |
| 8…10s | +Z5mm/s | +world Z方向 |
| 10…11s | 0 | master停止，reference保持 |

Modify幅度2使上表运动速度翻倍，采样率/mapper/速度界/workspace不变。
每条命令包含bounded reference和忽略全部界的unbounded baseline。后者应超速/越界，是预期失败对照。
产物只ignored tmp/s18_1_a1/a2：reference.csv、results.json、incremental_reference.png。
CSV每行是一次更新后的参考，对应(previous_time,current_time]区间，不是机器人动态采样。

## Actual Result / Explanation

2026-10-09，Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；当前conda hook激活mujoco，同执行shell核验所属Python。
Python3.12.14、NumPy2.5.3、Matplotlib3.11.2 distribution/实际imports/module paths通过；无安装或requirements改动。
未在Python3.11运行；3.11语法解析通过。本课未执行MuJoCo或ROS。

两条命令exit0，各550×25CSV，所有数据有限；结果中的engineering PASS包含unbounded预期失败。

| 指标 | master幅度1 | master幅度2 |
| --- | --- | --- |
| 原始请求峰速度 | 80mm/s | 160mm/s |
| applied reference峰速度 | 20mm/s | 20mm/s |
| speed限幅sample | 100 | 100 |
| workspace裁剪sample | 50 | 100 |
| 首次workspace裁剪 | 3.02s | 2.02s |
| 4.02s反向第一帧world Y增量 | −.2mm | −.4mm |
| 末参考world XYZ | [−.47,.22,.31]m | [−.49,.20,.32]m |
| 无界末参考world Y | .36m，越界 | .52m，越界 |

| 时刻 | 幅度1参考相对world锚点(mm) | 幅度2参考相对world锚点(mm) |
| --- | --- | --- |
| 2s | [0,20,0] | [0,40,0] |
| 4s | [0,40,0] | [0,40,0] |
| 6s | [0,20,0] | [0,0,0] |
| 8s | [−20,20,0] | [−40,0,0] |
| 10/11s | [−20,20,10] | [−40,0,20] |

幅度翻倍不会让bounded结果整体翻倍：两个case在4s都顶住Y40mm上界；随后反向位移分别20/40mm，末Y分别20/0mm。
首个workspace裁剪晚于恰好触及边界的时刻：刚到界仍是允许更新，下一个向外样本才拒绝。
20mm/s浮点实测值约.020000000000000573m/s，检查使用1e−12容差；不是物理越速或加速度保证。

常量master offset逐帧reference不变核验、静态对角线norm/方向、边界斜向投影、错frame轴语义、
nonfinite/非法dt/反射或非正交R/越界初始reference拒绝及输入不变检查通过。
独立`python tmp/s18_1_validation/check.py`重算CSV轴公式、norm限幅、box投影、累加、解析milestones、
绝对锚定裁剪仍粘界而增量更新释放的对照，通过；CLI amplitude0/nan各exit2。
两PNG目视、本地Markdown文件链接/git diff --check通过；无实际robot/device/GUI/控制稳定性或安全验证。

## Failure Cases / Robotics Context

漏取差分会把master绝对坐标当每帧位移而持续漂移；R与Rᵀ混淆会转错方向。
mm/m混用会使requested位移大1000倍，即使最后裁剪，输入映射仍错误；本课统一米。
逐轴速度限幅不能控制对角线norm；初始参考越界、旧master不更新、隐藏未裁剪virtual target会破坏预期语义。
workspace box只是参考约束，不证明机器人可达、避碰、actual速度或接触力安全。
对应遥操作中从master到任务空间命令的前端；后续再连接局部执行器控制，不能将reference当actual位置。

## Interview Capsule

30秒：我先求master相邻位置差，在明确的R_WM中转到world，再用norm限速和box投影更新Cartesian reference。
被拒绝增量丢弃，因此从边界反向立即释放；结果只是参考，未证明机器人实际执行能力。

2分钟：给出Δm、R、单标量α、box投影及离散速度单位；用+master X→+world Y说明frame语义。
解释独立world锚点和master常量offset不变性；再用80/160mm/s请求都限制到20mm/s、反向第一帧与末位移解释非线性裁剪。
最后区分离散reference速度界、方向保持的适用阶段，以及实际robot与连续时间控制的验证边界。

## Must Remember / My Verification

状态只见根README；2026-10-09 本人确认实验与预测完成，并准确回答五项Explain；Run/Modify/Explain全部完成，Learning Mastered。
本人解释正确区分master绝对位置与增量、world锚点与frame旋转、norm限幅与workspace投影、拒绝增量丢弃及离散reference与actual安全边界。
必须记住：先差分后转frame；norm限幅与workspace投影不同；不保存被拒绝运动欠账；reference不是actual机器人状态。
Run：运行默认命令，查看JSON与PNG，确认bounded合格、unbounded超速/越界，以及4.02s反向释放。
Modify：仅把master-amplitude1→2；先预测原始/参考峰速度、第一次workspace裁剪时刻、4/6/11s参考位置，再对照第二条命令。
Explain：

1. master position、master delta、robot reference、actual robot position分别是什么？本课实际生成了哪些量？
2. R_WM列的含义是什么？+master X/Y映射到哪里？为什么增量无需加frame原点，robot reference仍需要初始锚点？
3. norm限幅为何用共同α而不逐轴限速？workspace投影后是否仍保证方向不变？
4. 到达边界后为什么必须继续更新previous master、丢弃被拒绝增量？反向第一帧应发生什么？
5. master幅度翻倍时哪些请求量翻倍、哪些bounded输出不一定翻倍？20mm/s参考界能否证明actual速度和接触安全？

下一小任务S18.2 motion scaling/clutch/recenter等待明确请求，不自动实现。
