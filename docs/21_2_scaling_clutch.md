# S18.2 — Motion scaling / clutch / recenter

0.5～2h；前置：[S18.1](21_1_incremental_teleoperation.md)。
[代码](../examples/19_teleoperation_dexterous/scaling_clutch.py) / [示例入口](../examples/19_teleoperation_dexterous/README.md)。
Engineering / Learning唯一状态：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

手柄活动范围有限，而机器人可能需要大范围移动与精细定位。缩放让小手部运动产生更小参考位移；
clutch让你暂时断开运动映射、把手移回舒适位置；recenter重新定义手柄读数原点。
三个操作都应保留当前robot reference。直觉是“换尺子或把手拿回来”，不是命令机器人回原点。

## Core Concepts

`engaged=True`表示运动映射接合；False期间reference保持，但每帧仍更新master锚点。
重新接合事件显式rebase：用当前master建立下一帧差分起点，并消费事件样本，不执行其位移。
`rebase=True`同样用于设备坐标重置，优先于接合；本课人为提供可靠事件标记，没有自动识别跳变。
scale为正、无量纲；0.2表示master移动1mm对应requested reference移动0.2mm。
改变scale只影响新增量，不重新缩放历史位移。reference锚点和world workspace都保持原定义。
纯synthetic平移，无姿态、MuJoCo、实际机器人、ROS或设备；依赖NumPy/Matplotlib和S18.1 local helper。

## Mathematics：意义 / shape / unit / frame

```text
Δm_M = m_M[k] − a_M[k−1]                  # (3,) m，master frame
active = engaged AND NOT rebase
Δx_req_W = scale · R_WM · Δm_M if active else 0  # (3,) m，world
α = min(1, v_max·dt / ||Δx_req_W||₂)       # zero时取1
x_ref_W[k] = clip(x_ref_W[k−1] + αΔx_req_W, lower_W, upper_W)
a_M[k] = m_M[k]                           # 所有有效样本都消费，包括inactive
```

R_WM为(3,3)无量纲，+master X→+world Y，+master Y→−world X。
reference初值[-.45,.20,.30]m，box为初值±[.04,.04,.02]m；dt=.02s、norm速度界=.02m/s。
active=False时旧reference在box内，因此clip后仍不变。坐标重置跳变只改变a_M，不能成为运动指令。
拒绝增量不积累欠账；scale变大也不改变速度界或box。
这里的连续性指事件不额外造成reference跳变，运动样本仍是离散位置；不宣称连续时间导数、加速度或jerk连续。

## Math-to-Code / APIs

`map_sample(reference, previous_master, current_master, scale, engaged, rebase)`返回新reference(3,)、
下一master anchor(3,)和诊断dict，不修改输入。位置米；dt/rotation/box复用S18.1常量。
先用`update_reference`验证所有位置与bounds，即使inactive也拒绝nonfinite读数；失败不返回新锚点。
再把scale·Δm作为虚拟master增量交给S18.1 norm/box映射，保留显式公式，避免重复限幅算法。
调用方承担保存返回状态及声明事件的责任。重接合样本被丢弃是明确选择，不是故障。
`np.clip`返回新数组，`np.linalg.norm`返回Euclidean长度；没有实际robot qpos或ctrl更新。

## Minimal Experiment / Expected → Actual

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/scaling_clutch.py
python examples/19_teleoperation_dexterous/scaling_clutch.py --fine-scale .5
```

| 时间s | 输入/事件 | 预期reference行为 |
| --- | --- | --- |
| 0–1 | scale1，master X20mm/s | world Y增加20mm |
| 1–2 | 静止切换fine scale | 保持，历史位移不重算 |
| 2–3 | master X10mm/s | scale.2/.5下增加2/5mm |
| 3–4 | clutch断开，master移动[-100,30,0]mm | 保持 |
| 4–5 | 4.02s显式重接合rebase，随后静止 | 保持 |
| 5–6 | master X10mm/s | 首帧增加.04/.10mm，无旧位移追赶 |
| 6–7 | 6.02s坐标读数加[300,-200,100]mm并rebase | 保持 |
| 7–8 | scale1，master X80mm/s | 限速20mm/s，再受Y边界限制 |
| 8–9 | master X−10mm/s | 8.02s立即离开边界，最终Y offset30mm |

2026-10-09，DESKTOP-781D67A：WSL2 Ubuntu24.04.5，conda mujoco Python3.12.14；
NumPy2.5.3 / Matplotlib3.11.2 metadata、实际imports及环境module paths核验通过，无新增安装。
两命令exit0；各450×19 CSV，JSON和Agg PNG只在ignored tmp/s18_2_scale0.2与scale0.5。
phase-end Y offsets(mm)：默认[20,20,22,22,22,24,24,40,30]，Modify[20,20,25,25,25,30,30,40,30]。
max sample speed20mm/s（浮点误差容差1e−12m/s）；workspace裁剪10/25sample。
检查全部phase解析端点、inactive/rebase保持、首恢复增量、reverse、bounds与norm；
invalid scale0/负/nan/inf和inactive nan拒绝，直接clutch/rebase恢复检查通过。
Python3.11语法兼容检查通过，但未使用3.11解释器执行。PNG已目视；没有GUI运行。

## Explanation / Failure Cases

1. scale切换仍采用初始绝对锚点：`x0+sR(m−m0)`会重算历史位移，本例静止切换原始Y跳变−16/−10mm。
2. clutch期间冻结旧master anchor：重接合会追赶移回手柄产生的位移，原始world增量[-6,-20,0]/[-15,-50,0]mm。
3. recenter坐标读数被当作运动：原始world增量[40,60,20]/[100,150,50]mm。

JSON记录以上**未限幅原始错误增量**，不是错误机器人实际位移。即使后续限速压成0.4mm一步，仍产生不该有的运动。
三组失败公式用于解析对照；未执行完整错误控制器轨迹。
invalid scale不能靠clutch忽略；真实设备还需要事件顺序、丢包、时间戳/过期与滤波策略，本课未验证。
workspace box不证明机器人可达、避碰、接触安全或实际tracking；细调也不等于机器人定位精度提高。

## Robotics Context

用于手柄遥操作、精细对准、显微操作中的粗细模式与手部重定位。
local controller随后跟踪reference；clutch保持目标不代表实际机器人位置冻结，外力和控制器动力学仍可能导致运动。
本课不加入S18.3控制环。

## Interview Capsule

**30秒：**以相邻master样本计算world增量，先乘scale再做norm限速和workspace投影。
clutch或recenter只重建master差分锚点，保留robot reference；拒绝位移直接消费，不追赶。

**2分钟：**解释frame、单位和scale顺序，说明绝对锚定模式切换scale为什么重算历史；
说明clutch断开仍更新master anchor与重接合rebase消费样本；说明coordinate reset事件与真实运动不同。
最后区分事件无跳变、离散速度界、实际tracking与硬件安全。

## Must Remember

- 修改尺子不移动旧reference；重置master原点不重置world workspace。
- 每个有效样本都消费；rebase事件优先且丢弃该样本运动。
- reference保持不是物理制动；缩放不替代速度和workspace约束。

## My Verification — Run / Modify / Explain

状态只在根README维护，助手验证不代替本人学习。
Run：执行默认命令，观察3–5s与6–7s的reference平台。
Modify：改`--fine-scale .5`，先预测细调2→5mm/s、恢复首帧.04→.10mm、boundary裁剪10→25sample，再对照结果。
Explain：
1. 为什么切换scale不能重新乘历史master位移？
2. clutch断开时为什么仍要消费master样本？
3. recenter修改哪个锚点？为什么workspace不随之移动？
4. 为什么限速不能修复遗漏rebase事件的语义错误？
5. reference在clutch期间保持，为什么不能宣称实际机器人停止？

2026-10-09 本人明确确认实验与预测均完成，并提交五项Explain；根README Run/Modify/Explain全部完成，Learning Mastered。
解释精度补充：本课每帧更新差分锚点，scale切换无需额外rebase；若采用绝对锚定映射，则须重建配对锚点以保留当前reference。
遗漏rebase产生错误增量，限速只能限制其大小；本课消费该样本并丢弃拒绝部分，不持续追赶完整错误目标。
本次仅文档同步，未重跑数值实验或GUI；runtime沿用2026-10-09工程验证。
