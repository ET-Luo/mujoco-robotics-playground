# Stage 18 — Teleoperation & Dexterous Foundations

**规划 skeleton，尚未实现/实验。** 状态见[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。
基础复用[Stage17](20_compliant_contact_control.md)，两个分支最终分别做Integration。

## Problem / Why / Intuition

人怎样通过增量命令引导顺应机器人？多个指尖怎样通过接触限制物体运动？
先用synthetic master可重复地学习reference映射，再用只含hand/object的轻量模型观察受力与滑移。
keyboard为可选输入；不需要真实haptic device。

## Core Concepts / Mathematics / Math-to-Code contract

Tele：`delta_x_robot_W = scale * R_WM @ delta_x_master_M`，m；参考更新有速度/工作空间界。
clutch脱开时冻结robot reference、重建master锚点；不是重置robot qpos或瞬移。
rotation增量后续单独注明左/右乘frame，初课只translation。
反馈概念：environment→robot力可映射到master显示/虚拟反馈力（N）；
屏幕箭头/力图不是人感受到的haptic force，不声明双边被动性或真实反馈稳定性。
Hand：fingertip position/J复用FK/J方法但核对新模型axis/dof；`tau_i=J_i.T F_i`。
friction cone：`||f_t|| <= mu*f_n, f_n>=0`，接触只能压不能拉；N。
抓取矩阵`w_object = G f_contacts`明确object参考点、contact frame和作用于object的force。
force closure是在给定接触/摩擦模型下可抵抗任意方向小wrench的性质；
一次保持成功不是其证明。本阶段限平面简化模型的可视化/可行性反例，不引入高阶优化黑盒。

## Tasks / Minimal Experiments / Expected evidence

每项0.5～2h；尚无可运行命令或Actual。

| Task | 单一概念 / 最小实验 | 验收设计 |
| --- | --- | --- |
| S18.1 Incremental teleoperation | synthetic translation master→bounded Cartesian reference | frame/增量/限速；无直接qpos控制 |
| S18.2 Motion scaling + clutch/recenter | 同一master轨迹、不同scale，脱开/重接 | reference连续、锚点无跳变、workspace gate |
| S18.3 Teleoperation + impedance | .1/.2 reference送Stage17本地controller | 自由空间/接触响应、force峰值和延迟；输入频率≠inner loop频率 |
| S18.4 Force feedback concept | 测量反力→master-frame虚拟力/图形 | sign/scale/frame、饱和；仅synthetic反馈，无真实haptics证明 |
| S18.5 Teleoperation Integration | scripted approach/touch/slide/clutch/recenter/retreat | contact/force/reference连续、过期输入本地处理；keyboard可选 |
| S18.6 Lightweight hand fixture | 固定palm，3finger×2hinge/capsule，单object | joint/actuator mapping、开闭限幅与self-contact；不加载humanoid |
| S18.7 Fingertip FK / Jacobian | 新hand tip site，数值差分核验J | 输出shape/frame/unit、关节耦合；不重复UR5e IK课 |
| S18.8 Touch/contact sensing | 每指named contact与force/可选touch sensor | binary touch和force区分、区域/方向限制，非高分辨率tactile图像 |
| S18.9 Multi-Contact Grasp Integration | 先两指再三指，phase-gated close/hold | force分布、object drift/slip，不把多触点等同closure |
| S18.10a Friction cone intuition | 单接触正压力/切向负载扫描 | mu、normal load与滑移，接触柔性影响；不重教碰撞检测 |
| S18.10b Force closure intuition | 简化平面点接触，wrench方向/摩擦约束可视化 | 可抵抗与不能抵抗的方向、grasp matrix参考点；不宣称3D一般closure |
| S18.11 Disturbance / Stability Integration | 固定hand抓取，受控小wrench脉冲与friction对照 | 全部trials分母、位姿漂移、slip/掉落、饱和；非in-hand reorientation |

## Expected / Actual / Explanation

Expected是验收计划；Actual未执行，参数/模型选择在任务实现时核验。
固定palm是局部hand学习fixture；不宣称移动手腕抓取/双臂/完整humanoid。

## Failure Cases / Stability / Robotics Context

增量frame错、clutch重接跳变、stale input、force feedback反号/延迟、
多指互相挤压、法向不足/摩擦不足、接触几何退化；触到物体不证明抓得稳。
MuJoCo刚性capsule/摩擦接触不模拟真实柔软指腹与高频触觉。
应用是遥操作辅助与多指稳定抓取基础，后续复杂in-hand manipulation另行规划。

## Interview Capsule / Must Remember（后续逐课填写）

30秒：解释reference增量/clutch或某一接触怎样改变object wrench。
2分钟：从frame/unit/sign说到controller/接触约束，再用一项disturbance结果界定能力。
必须记住：reference连续不等于安全；触点数不等于force closure；simulation feedback不是haptic device。

## My Verification / Run / Modify / Explain

Learning仍由本人报告；后续每Task给Run/一个Modify/3–5问Explain。
候选问题：clutch为什么需锚点重建？scale怎样影响接触？为何touch不是稳定抓取？
force closure与某次disturbance通过的证据边界是什么？
