# P2 — Contact-Rich Manipulation & Dexterous Robotics

2026-10-07：本轮 P0/P1 最终代码静态审查与 P2 规划。
核心问题：**How does a robot physically interact with the environment?**
任务状态只维护在[根 README](../README.md#p2--contact-rich-manipulation--dexterous-robotics)。
首轮仅实现 S16.1；本人于2026-10-08确认Run/Modify/Explain完成，Learning Mastered。
2026-10-08 本人授权S16.2；[学习包](19_2_manipulator_dynamics.md) Engineering Complete + Learning Mastered（本人确认实验、预测与五项Explain）；S16.3随后明确授权并Engineering Complete，见[学习包](19_3_gravity_compensation.md)；S16.4随后明确授权，仅推进当前Task。

## P1 最终状态审查与去重

本人已确认 P0/P1 完成；README Stage10–15 的细项均记录 Mastered。
“完成”对应规划内实验与学习，不等于通用机器人能力或现实部署。
本轮读取最终代码，不重跑 P0/P1；以下 runtime 引用 2026-10-07 既有现场记录。

| 已掌握内容 / 实现入口 | P2 如何复用 | 新课不重复的边界 |
| --- | --- | --- |
| [UR5e 6D pose](../examples/09_ur5e_6d_pose/README.md)、[Jacobian / IK / DLS](../examples/10_ur5e_6d_ik/README.md) | 读取 site pose/J，复用 bounded IK 作初始配置 | 不重新讲 FK 或重新开发 IK；J转置的新用途是力与功率映射 |
| [trajectory](../examples/11_trajectory/README.md) | 给接触前位置参考/速度上限 | 轨迹可行不等于接触稳定，不再重复 cubic 配时 |
| [P0 最终 trials](../examples/12_pick_place/robustness_trials.py)、[release/retreat](../examples/12_pick_place/release_and_retreat.py) | 模型、joint mapping、支撑/分离/低速判据 | P0 trials 从grasp初始化、known pose；夹住和lift不等于一般force closure |
| [camera geometry / PnP / RGB-D / ICP / hand-eye](../examples/13_perception_geometry/README.md) | 需要时消费已有pose估计和frame约定 | 不重做感知；本轮新实验无需OpenCV |
| [vision pipeline](../examples/14_vision_manipulation/vision_pick_place.py) | 私有IK、阶段执行、actual/truth评分分离 | 已读取正常力但只作事件门限；没有连续接触力闭环 |
| [collision / RRT integration](../examples/15_motion_planning/collision_aware_pick_place.py) | 接触前路径、pair/phase许可 | 采样几何合法不等于力/稳定性合法；不再写RRT |
| [ROS2 topic/service/action/TF2/URDF/JointState](../examples/16_ros2_integration/README.md) | 命令、状态、action lifecycle、低频监控 | 不重新教协议，不让ROS2承担高频inner loop |
| [final ROS worker](../examples/16_ros2_integration/manipulation_worker.py)、[adapter](../examples/16_ros2_integration/manipulation_ros.py) | conda/system解释器隔离、typed task、日志 | 当前tick每次授权10physics steps，推进受ROS/IPC请求节奏影响；P2需要独立worker loop |

P1最终视觉抓放连续home→pick→place→retreat；S14.7b包含held-object避障规划；
S15.6b完整ROS链复用S13.3b，**没有接入S14.7b障碍RRT**。不能把二者称为统一通用系统。
本机既有S14.7b 1/1、S15.6a SUCCEEDED、S15.6b placement_passed/误差0.714mm；
全程无GUI/真实视觉/硬件证据，ROS perception退出ExternalShutdownException保留。

已有S3单hinge PD、S2位置servo和P1 `qfrc_applied=qfrc_bias` 是本阶段前置经验。
S16.1只增actuator输入/输出/gear/饱和的同物理实验；S16.2/3解释bias与动力学，
不把“复制bias”误称为已经掌握完整动力学或真实motor补偿。
P1法向力门限已有基础；S16.5新内容为完整contact wrench、方向、坐标变换和力平衡。

## 顺序、预算与集成节奏

每项0.5～2小时，Engineering=Code/Experiment/Docs，Learning=Run/Modify/Explain。
任务逐项授权；新大型集成拆为最小fixture与机器人移植，避免单Task过载。

| Stage | 任务顺序 | 阶段输出与Integration |
| --- | --- | --- |
| 16 Force & Dynamics Foundations | S16.1 actuator语义 → .2 dynamics分项 → .3 UR5e gravity → .4 wrench → .5 contact force → .6 Jᵀ | .3是前半段Integration；.7把施力/测力/关节力矩串起来 |
| 17 Compliant & Contact-Rich Control | .1 spring → .2 impedance → .3 sweep → .4 contact transition → .5 admittance → .6 force → .7 hybrid | .4接触过渡Integration；.8a/.8b Surface Following；.9独立worker/ROS监控Integration |
| 18 Teleoperation & Dexterous Foundations | .1 incremental master → .2 scale/clutch → .3 tele+impedance → .4 feedback → .5 tele Integration；再.6 hand → .7 fingertip J → .8 touch → .9 grasp → .10a/.10b cone/closure | .5 tele Integration；.9 multi-contact Integration；.11 disturbance/stability Integration |

每课唯一主要概念与验收见[Stage16](19_force_dynamics.md)、[Stage17](20_compliant_contact_control.md)、
[Stage18](21_teleoperation_dexterous.md)。README逐项跟踪，未来实验不预填PASS。

## Physical contract：每课都明确

- 施力主体：environment→robot或robot→environment；两者成对反号。
- 坐标与参考点：world/tool/contact，wrench写 `[force; moment]`，力矩关于哪个点必须注明。
- 单位：m、rad、s、N、N·m，stiffness/damping注明平移/转动单位。
- 稳定性：积分dt、controller dt、增益、有效惯量、限幅、延迟、接触刚度分开记录。
- 评价：轨迹误差、力误差、峰值/振荡、接触保持、滑移/物体速度、失败停止；禁止单靠终点评分。
- 高刚度不自动等于高性能；稳定的自由空间控制不自动等于稳定接触。

## CPU与实现边界

AMD Ryzen7 7840HS / 无NVIDIA GPU / WSL2 Ubuntu24.04。
优先official MuJoCo+NumPy，Matplotlib用于离线图；初期1DOF/低DOF fixture，后接既有UR5e。
hand优先固定palm、3finger×2hinge、简单capsule collision教学MJCF；只加载hand/object/table，
不加载humanoid。与商业灵巧手/软指腹/真实触觉的差异必须记录；无需新框架/大型模型资产。
新课暂不增加依赖，后续如确需包再按AGENTS核验/声明/选择性安装。
不引入Isaac、RL训练、ACT/Diffusion Policy/VLA、大型tactile视觉网络或真实haptic device。
teleoperation默认synthetic master可重复，keyboard仅可选UI，不用UI帧率驱动物理积分。

## Controller worker / ROS2 contract（计划，未实现）

MuJoCo worker按固定simulation dt读取最新有界reference、算控制、施加torque、mj_step；
闭环在worker内完成，ROS callback不授权每个physics step，也不等待高频测力再算控制。
可选wall pacing不是硬实时保证；记录simulation rate与wall调度。
ROS2负责command/state/action/feedback/monitoring，经有界mailbox传reference；
age/timeout用monotonic wall time，state用simulation stamp，两种clock不能混用。
命令过期或cancel切换worker本地安全hold/damping模式，仍继续physics以验证负载行为；
仿真pause另作明确操作，不能作为“硬件停止”的替代。此生命周期改进留S17.9，
本轮不修改P1 worker或其历史结果。

## Sprint Learning skeleton

每个实际Task按 Problem → Why → Intuition → Core Concepts → Mathematics → Math-to-Code →
Minimal Experiment → Expected/Actual Result → Explanation → Failure Cases → Robotics Context →
30秒/2分钟Interview Capsule → Must Remember → My Verification → Run/Modify/Explain。
未来Stage skeleton只给问题、验收设计与未实现边界；执行后再写API/命令/actual结果。
Learning必须由本人明确报告，助手运行不代替学习。S16.2已Mastered；S16.3 Engineering Complete + Learning Mastered（2026-10-08 本人确认）；S16.4已明确授权并完成[工程学习包](19_4_wrench_frames.md)，Learning Mastered（2026-10-08 本人确认）；S16.5随后明确授权并完成[工程学习包](19_5_contact_wrench.md)，Learning Mastered（2026-10-08 本人确认）；S16.6已明确授权并完成[工程学习包](19_6_jacobian_transpose.md)，Learning Mastered（2026-10-08 本人确认）；S16.7已明确授权并完成[工程学习包](19_7_force_mapping_integration.md)，Learning Mastered（2026-10-08 本人确认）；Stage16学习项全部完成，S17.1已按明确请求完成[虚拟弹簧学习包](20_1_cartesian_spring.md)，Engineering Complete（2026-10-08），Learning Run/Modify/Explain全部完成，Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.2已按明确请求完成[阻抗学习包](20_2_cartesian_impedance.md)，Engineering Complete（2026-10-08），Learning Run/Modify/Explain全部完成，Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.3已按明确请求完成[增益扫描学习包](20_3_impedance_sweep.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.4已按明确请求完成[接触过渡学习包](20_4_contact_transition.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.5已按明确请求完成[Admittance学习包](20_5_admittance.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.6已按明确请求完成[法向力控制学习包](20_6_normal_force.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain；坐标与gate精度见学习包）；S17.7已按明确请求完成[Hybrid学习包](20_7_hybrid_position_force.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.8a已按明确请求完成[fixture扫描学习包](20_8a_surface_following_fixture.md)，Engineering Complete（2026-10-09），Learning Run/Modify/Explain待本人验证；下一小任务S17.8b等待明确请求。
