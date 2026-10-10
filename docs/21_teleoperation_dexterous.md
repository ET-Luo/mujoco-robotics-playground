# Stage 18 — Teleoperation & Dexterous Foundations

**S18.1/18.2 Engineering Complete + Learning Mastered；S18.3 Engineering Complete + Learning Mastered；S18.4 Engineering Complete + Learning Mastered；S18.5 Engineering Complete + Learning Mastered；S18.6 Engineering Complete + Learning Mastered；S18.7 Engineering Complete + Learning Mastered；S18.8 Engineering Complete + Learning Mastered；S18.9 Engineering Complete + Learning Mastered；S18.10a Engineering Complete + Learning Mastered；S18.10b Engineering Complete，Learning待本人验证；S18.11为规划 skeleton。** 状态见[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。
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

每项0.5～2h；[S18.1学习包](21_1_incremental_teleoperation.md)含可运行synthetic master映射、norm/workspace界、独立数值检查和Run/Modify/Explain；[S18.2学习包](21_2_scaling_clutch.md)包含scale/clutch/recenter；[S18.3学习包](21_3_teleoperation_impedance.md)包含低频参考与1kHz本地反馈的实际动力学及失败对照。[S18.4学习包](21_4_force_feedback.md)包含world反力→master虚拟显示的sign/gain/norm cap与delay对照；[S18.5学习包](21_5_teleoperation_integration.md)已整合接触、滑动、clutch/recenter与stale恢复；[S18.6学习包](21_6_hand_fixture.md)已完成hand-only结构/mapping/开闭与self-contact；[S18.7学习包](21_7_fingertip_jacobian.md)已核验三指FK/world J/全列中心差分；[S18.8学习包](21_8_contact_sensing.md)已验证具名接触/world合力/法向和/区域touch；[S18.9学习包](21_9_multi_contact_grasp.md)已整合free cylinder/接触gate/两三指载荷漂移与滑动；[S18.10a学习包](21_10a_friction_cone.md)已核验单接触cone/μ与normal扫描及锥内微滑；[S18.10b学习包](21_10b_force_closure.md)已核验平面G/C/W、正零空间证书与有界负载；S18.11尚未实施。

| Task | 单一概念 / 最小实验 | 验收设计 |
| --- | --- | --- |
| S18.1 Incremental teleoperation | synthetic translation master→bounded Cartesian reference | frame/增量/限速；无直接qpos控制 |
| S18.2 Motion scaling + clutch/recenter | 同一master轨迹、不同scale，脱开/重接 | reference连续、锚点无跳变、workspace gate |
| S18.3 Teleoperation + impedance | .1/.2 reference送Stage17本地controller | 自由空间/接触响应、force峰值和延迟；输入频率≠inner loop频率 |
| S18.4 Force feedback concept | 测量反力→master-frame虚拟力/图形 | sign/scale/frame、饱和；仅synthetic反馈，无真实haptics证明 |
| S18.5 Teleoperation Integration | scripted approach/touch/slide/clutch/recenter/retreat | contact/force/reference连续、过期输入本地处理；keyboard可选 |
| S18.6 Lightweight hand fixture | 固定palm，3finger×2hinge/capsule，hand-only无object | joint/actuator mapping、开闭限幅与self-contact；不加载humanoid |
| S18.7 Fingertip FK / Jacobian | 新hand tip site，数值差分核验J | 输出shape/frame/unit、关节耦合；不重复UR5e IK课 |
| S18.8 Touch/contact sensing | 每指named contact与force/可选touch sensor | binary touch和force区分、区域/方向限制，非高分辨率tactile图像 |
| S18.9 Multi-Contact Grasp Integration | 先两指再三指，phase-gated close/hold | force分布、object drift/slip，不把多触点等同closure |
| S18.10a Friction cone intuition | 单接触正压力/切向负载扫描 | mu、normal load与滑移，接触柔性影响；不重教碰撞检测 |
| S18.10b Force closure intuition | 简化平面点接触，wrench方向/摩擦约束可视化 | 可抵抗与不能抵抗的方向、grasp matrix参考点；不宣称3D一般closure |
| S18.11 Disturbance / Stability Integration | 固定hand抓取，受控小wrench脉冲与friction对照 | 全部trials分母、位姿漂移、slip/掉落、饱和；非in-hand reorientation |

## Expected / Actual / Explanation

S18.1两master幅度实验通过：requested峰80/160mm/s，bounded参考峰均20mm/s，首次workspace裁剪3.02/2.02s，反向第一帧释放；详见[学习包](21_1_incremental_teleoperation.md)。S18.2两scale已完成并Mastered。S18.3两master周期/三case工程通过，本地反馈稳定；10Hz packet-feedback对照末tracking失败，详见[学习包](21_3_teleoperation_impedance.md)。S18.4两gain实验通过：显示稳态.999/1.5N、饱和1/2438sample、robot trace相同；详见[学习包](21_4_force_feedback.md)。S18.5两gap/三case工程验证完成，正确恢复无事件运动、接触/退回通过；错误恢复虽限速/末tracking通过，仍违反动作契约；详见[学习包](21_5_teleoperation_integration.md)。S18.6默认/过度闭合通过：乱序mapping、逐指隔离/重新张开；过闭合存在三对指间contact及小幅soft limit越界；详见[学习包](21_6_hand_fixture.md)。S18.7离线几何验证通过，δ×10预测误差约×100，详见[学习包](21_7_fingertip_jacobian.md)。S18.8三case/两区域位置工程验证通过，零touch不代表无contact、自接触正读数不证明抓取，详见[学习包](21_8_contact_sensing.md)。S18.9两负载/四case工程验证通过，低摩擦滑到palm和missing gate timeout作为失败对照；详见[学习包](21_9_multi_contact_grasp.md)。S18.10a两normal scale/各四扫描工程验证通过，实际接触力在锥内，首次报告与理想能力相近且锥内微滑可见；详见[学习包](21_10a_friction_cone.md)。S18.10b两μ/三布局静态工程验证通过，满秩/正span/有界负载区别明确；详见[学习包](21_10b_force_closure.md)。S18.11 Expected为计划，Actual未执行。
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
