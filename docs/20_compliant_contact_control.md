# Stage 17 — Compliant & Contact-Rich Control

**S17.1 Engineering Complete + Learning Mastered；S17.2 Engineering Complete + Learning Mastered；S17.3 Engineering Complete + Learning Mastered；S17.4 Engineering Complete + Learning Mastered；S17.5 Engineering Complete + Learning Mastered；S17.6 Engineering Complete、Learning待本人验证；其余Task仍为规划 skeleton。** 状态见[根README](../README.md#stage-17--compliant--contact-rich-control)。
先完成[Stage16](19_force_dynamics.md)；不用IK误差迭代代替动力学控制。

## Problem / Why / Intuition

位置reference撞上环境后，应允许怎样的位移，又应施多大力？
先把工具当有阻尼的虚拟弹簧，再区别“位移→力”的impedance与“测力→运动”的admittance。
目标是有边界的接触行为；几何到位/终点小误差不足以验收。

## Core Concepts / Mathematics / Math-to-Code contract

平移：`F_cmd_W = K_x (x_d_W-x_W) + D_x (v_d_W-v_W)`，N；K为N/m，D为N·s/m。
`tau_cmd = J_p_W.T @ F_cmd_W + compensation`，施加机器人驱动torque；
补偿项须明确gravity/bias，不能自动消除真实contact force。
初课固定orientation、少DOF，避免在m/rad混合6D增益中隐藏单位；姿态扩展再用N·m/rad。
Admittance：`M_v xdd + D_v xd + K_v(x-x_ref) = F_env_to_robot`，虚拟mass kg；
本地产生有速度/位移界的reference，再由现有inner controller跟踪。
Hybrid在同一surface frame中分开normal force与tangent position，selector不重复控制同一轴。
公式是课程设计；具体积分/反馈符号/饱和在对应Task才实现和验证。

## Tasks / Minimal Experiments / Expected evidence

每项0.5～2h；[S17.1学习包](20_1_cartesian_spring.md)含Run、Actual与Run/Modify/Explain；[S17.2学习包](20_2_cartesian_impedance.md)包含阻尼恢复对照；[S17.3学习包](20_3_impedance_sweep.md)包含增益、限幅与粗dt对照；[S17.4学习包](20_4_contact_transition.md)包含触碰事件、冲击与持续保持；[S17.5学习包](20_5_admittance.md)包含测力到运动reference、bias offset/drift与reference界；[S17.6学习包](20_6_normal_force.md)包含力误差反馈、fixed对照与失联gate；其余Task具体结果只在授权实现后填写。

| Task | 一个主要问题 / 最小实验 | 验收设计 |
| --- | --- | --- |
| S17.1 Cartesian virtual spring | 低DOF工具受位移后用Jᵀ施恢复力，固定orientation | 方向/单位/势能与力关系；无IK循环替代torque |
| S17.2 Cartesian impedance | spring加入速度反馈阻尼，在同fixture自由空间恢复 | 峰值/settling/速度，controller dt、torque限幅 |
| S17.3 stiffness/damping sweep | 少量K/D组合、同initial/load/seed | 对比轨迹、峰值力与振荡；区分阻尼改变和离散失稳 |
| S17.4 Contact Transition Integration | approach→touch→compliant hold低DOF平面 | 过渡峰值、接触丢失、持续窗口；过快approach失败对照 |
| S17.5 Admittance | synthetic或contact力驱动虚拟mass reference | 正负力方向、reference界、offset/漂移，不与impedance混称 |
| S17.6 Normal force control | 单轴压平面，约定environment→tool为+normal | force target与measured稳态/峰值；脱离接触gate、anti-windup若引入积分 |
| S17.7 Hybrid position-force | 同fixture切向位置+法向力selector | tangent error与normal force分别评价，错frame/错sign拒绝 |
| S17.8a Surface Following Integration / fixture | 已知平面切向短扫描，恒定normal load | 路径/测力/接触连续性、饱和、partial失败停止 |
| S17.8b Surface Following Integration / UR5e | 移植.8a到裸臂+简单probe，复用初始IK | joint/site/frame映射，actual扫描与反力、限速/力矩，非通用未知曲面 |
| S17.9 Worker / ROS Monitoring Integration | 独立固定dt controller，ROS只送reference/monitor | ROS延迟/停发时本地hold仍推进physics；timeout/cancel与pause分别测试 |

## Expected / Actual / Explanation

Expected为上表验收设计；S17.1 Actual见独立学习包：两组纯弹簧持续振荡，恢复力/能量梯度/motor核验通过；S17.2默认/半D均恢复，settling .976/1.381s，见独立学习包；S17.3六组增益恢复、粗dt失败对照见[学习包](20_3_impedance_sweep.md)；S17.4慢速接触合格、快速冲击超预算但最终保持，见[学习包](20_4_contact_transition.md)；S17.5合成测力正负响应、offset/drift与参考边界通过，actual跟踪误差另记录，见[学习包](20_5_admittance.md)；S17.6两目标闭环与失联退出验证见[学习包](20_6_normal_force.md)；其余Task未执行。
输出以小CSV/JSON/PNG为主；保持接触力序列与失败phase，不只输出终点。

## Failure Cases / Stability / Robotics Context

大K、小D、过大dt、过快撞击、错force方向、法向/切向frame混用、力饱和与积分累积，
均可使误差/冲击增大。接触模型参数和controller增益应分别记录。
应用为表面擦拭/装配接触；仅指定简化fixture，不宣称真实装配或通用稳定性。

## Interview Capsule / Must Remember（后续逐课填写）

30秒：说明这课输入是运动reference还是force，以及输出是torque还是motion。
2分钟：用frame/unit/sign、闭环方程、dt/饱和、一个失败case解释实验。
必须记住：impedance/admittance方向不同；force target不等于测到的force；稳定性取决于整体闭环。

## My Verification / Run / Modify / Explain

状态仅根README；未来Task的Learning空框，S17.1–S17.5已由本人确认完成。后续Task各指定一个Modify和3–5问Explain。
阶段候选问题：为何仅提高K会增加接触冲击？两种顺应控制输入/输出何异？
hybrid为何需共同surface frame？ROS停发后worker应怎样处理仍存在的重力/接触？
