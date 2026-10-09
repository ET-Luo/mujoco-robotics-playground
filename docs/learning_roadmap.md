# 学习路线与仓库评估

更新：2026-10-08。任务与 Engineering/Learning checkbox 唯一维护在
[README](../README.md#learning-roadmap)。助手验证不能替代本人 Run/Modify/Explain。
P0（Stage0–11）与P1（Stage12–15）已由本人确认完成；P2 S16.1–S16.6 Engineering Complete + Learning Mastered（2026-10-08 本人确认）。

## 当前代码如何使用

| 入口 | 实际代码用途 | 继续学习方式 |
| --- | --- | --- |
| examples/01–08（包括两个有意保留的 02 目录） | 既有基础、控制、平面运动学、独立夹爪小实验 | 已完成阶段不重复教学；按需查阅 |
| examples/09_ur5e_6d_pose | 官方 UR5e 完整位姿 | 复用 frame/cache 阅读经验 |
| examples/10_ur5e_6d_ik | 6D error / Jacobian / DLS / bounded iteration | 已有求解器；不重新实现基础 IK |
| examples/11_trajectory | linear/cubic reference 与解析导数 | 复用 cubic，Stage 14 才扩展路径时间化 |
| examples/12_pick_place | UR5e 集成夹爪、known-pose lift/place/release/trials | 复用模型/执行判据；接口限制见 P1 审计 |
| examples/13_perception_geometry | camera_frames / pinhole_projection / rgb_depth_capture / camera_calibration / pnp_pose / rgbd_back_projection / rigid_alignment / icp_loop / hand_eye_geometry / hand_eye_calibration，CPU geometry、calibration 与 pose | Stage 12 已完成入口 |
| examples/14_vision_manipulation | perception_pose / grasp_candidates / vision_to_motion：packet/frame、candidate/IK、image/PnP→continuous motion / vision_pick_place 完整取放 / perception_noise 受控误差case / repeated_trials 分布评价 | 当前新工程入口 |
| examples/15_motion_planning | collision_checking / edge_checking / rrt / rrt_connect / path_smoothing / time_parameterization / ur5e_obstacle_planning / collision_aware_pick_place：阶段许可/持物查询、二维C-space/分辨率漏检/单树与双树RRT/checked shortcuts/cubic配时/六维UR5e/held取放与tracking | S14.1–S14.7b入口；学习均完成 |
| examples/16_ros2_integration | node_topic / pose_service / trajectory_action / tf2_frames / urdf_reference / joint_states_tf / manipulation_worker / manipulation_ros：完整CPU视觉抓放及状态/终态诊断；Int32发布订阅、Trigger缓存位置短请求/显式拒绝与双超时 | S15.1–S15.6b Engineering Complete；S15.1–S15.6b Learning Mastered |
| examples/17_force_dynamics | actuator_semantics / manipulator_dynamics / gravity_compensation / wrench_frames / contact_wrench / jacobian_transpose / force_mapping_integration：actuator语义、动力学、UR5e gravity hold与wrench几何 | S16.1–S16.7 Mastered；后续逐Task授权 |
| environments / rl | 既有 planar Reach / CPU PPO 学习代码与部分占位 | P1 不推进 RL |
| controllers / assets / notebooks / tests | 支持说明与部分占位 | 不为未来需求填满框架 |
| scripts/check_env.sh | 环境诊断 | 执行前核验环境，不能代替算法验证 |

详见[P0 最终审查与 P1 规划](p1_plan.md)。本轮未重跑 P0；历史数字标注在该文。
P1 顺序：geometry perception → vision manipulation → self-written planning → ROS2。
CPU NumPy 优先；S12.4 已新增 OpenCV headless 用于 calibration/PnP；NumPy ICP 与 RRT 自行实现。

## Stage 与笔记映射

| Stage | 笔记 |
| --- | --- |
| 0 Environment | [开发流程](development_workflow.md)、[交接记录](project_handoff.md) |
| 1 MuJoCo Basics | [01 MuJoCo](01_mujoco_basics.md)、[02 UR5e](02_ur5e_model.md) |
| 2 Joint Control | [03 Joint Control](03_joint_control.md) |
| 3 PD Control | [04 PD Control](04_pd_control.md) |
| 4 Robot Kinematics | [05 FK](05_forward_kinematics.md)、[06 Jacobian](06_jacobian.md) |
| 5 Inverse Kinematics | [07 IK](07_inverse_kinematics.md) |
| 6 Cartesian Control | [08 Cartesian Control](08_cartesian_control.md) |
| 7 Manipulation | [09 Manipulation](09_manipulation.md) |
| 8 Robot Learning | [10 Robot Learning](10_robot_learning.md) |
| 9 Domain Randomization / Sim-to-Real | 后续实验时在 10 中补充，必要时再拆分 |
| 10 Full UR5e 6D Motion | [11 UR5e 6D Pose](11_ur5e_6d_pose.md)、[12 UR5e 6D IK](12_ur5e_6d_ik.md)、[13 Trajectory](13_trajectory.md) |
| 11 Known-Pose Pick & Place | [14 Pick & Place](14_pick_place.md) |
| 12 Robot Perception Geometry | [15 Perception Geometry](15_robot_perception_geometry.md) |
| 13 Vision-Based Manipulation | [16 Vision Manipulation](16_vision_based_manipulation.md) |
| 14 Motion Planning | [17 Motion Planning](17_motion_planning.md) |
| 15 ROS2 Integration | [18 ROS2 Integration](18_ros2_integration.md) |
| 16 Force & Dynamics Foundations | [19 Force & Dynamics / S16.1](19_force_dynamics.md) |
| 17 Compliant & Contact-Rich Control | [20 Compliant Control skeleton](20_compliant_contact_control.md) |
| 18 Teleoperation & Dexterous Foundations | [21 Teleoperation / Dexterous skeleton](21_teleoperation_dexterous.md) |

## P2 当前边界

[P2审查与规划](p2_plan.md)明确已有内容复用、force/frame/unit/sign/stability合同和Integration节奏。
S16.1新内容是actuator语义/gear/饱和/扰动等价验证，不重复PD教程；S16.2已实现并核验单hinge解析动力学，见[学习包](19_2_manipulator_dynamics.md)；2026-10-08 本人确认实验、预测与五项Explain，Learning Mastered；S16.3已授权并完成[工程学习包](19_3_gravity_compensation.md)，2026-10-08 本人完成Run/Modify/Explain，Learning Mastered；S16.4已明确授权并完成[工程学习包](19_4_wrench_frames.md)，2026-10-08 本人完成Run/Modify/Explain，Learning Mastered；S16.5已明确授权并完成[工程学习包](19_5_contact_wrench.md)，2026-10-08 本人完成Run/Modify/Explain，Learning Mastered；S16.6已明确授权并完成[工程学习包](19_6_jacobian_transpose.md)，2026-10-08 本人完成Run/Modify/Explain，Learning Mastered；S16.7已明确授权并完成[工程学习包](19_7_force_mapping_integration.md)，Learning Mastered（2026-10-08 本人确认）；Stage16学习项全部完成，S17.1已按明确请求完成[虚拟弹簧学习包](20_1_cartesian_spring.md)，Engineering Complete（2026-10-08），Learning Run/Modify/Explain全部完成，Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.2已按明确请求完成[阻抗学习包](20_2_cartesian_impedance.md)，Engineering Complete（2026-10-08），Learning Run/Modify/Explain全部完成，Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.3已按明确请求完成[增益扫描学习包](20_3_impedance_sweep.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.4已按明确请求完成[接触过渡学习包](20_4_contact_transition.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.5已按明确请求完成[Admittance学习包](20_5_admittance.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.6已按明确请求完成[法向力控制学习包](20_6_normal_force.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain；坐标与gate精度见学习包）；S17.7已按明确请求完成[Hybrid学习包](20_7_hybrid_position_force.md)，Engineering Complete + Learning Mastered（2026-10-08 本人确认实验、预测并完成五项Explain）；S17.8a已按明确请求完成[fixture扫描学习包](20_8a_surface_following_fixture.md)，Engineering Complete（2026-10-09），Learning Run/Modify/Explain全部完成，Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S17.8b已按明确请求完成[UR5e扫描学习包](20_8b_surface_following_ur5e.md)，Engineering Complete（2026-10-09），Learning Run/Modify/Explain全部完成，Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S17.9已按明确请求完成[独立worker/ROS学习包](20_9_worker_ros_monitoring.md)，Engineering Complete（2026-10-09），Learning Run/Modify/Explain全部完成，Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；不自动启动Stage18。
P1 worker由ROS/IPC tick预算推动仿真；P2 S17.9已实现独立worker loop，ROS低频送有lease的reference描述并monitor；取消/超时继续physics，pause独立。
本课Run/Modify/Explain见[学习包](19_force_dynamics.md#my-verification--run--modify--explain)。

## P1 历史 handoff

本人已完成 [S12.5 Learning Package](15_5_pnp_pose.md) 的实验、验证与 Explain。
S12.6 [RGB-D Learning Package](15_6_rgbd_back_projection.md) 本人已明确确认实验、预测与 Explain，Learning Mastered。
S12.7a [Rigid Alignment Package](15_7a_rigid_alignment.md) 本人已确认实验、预测与 Explain，Learning Mastered。
S12.7b [ICP Loop Package](15_7b_icp_loop.md) 工程完成：NumPy NN/gate/SVD 左乘更新，近/远初值与 partial 对照；本人已确认实验、预测与 Explain，Learning Mastered。
S12.8a [Hand-Eye Geometry Package](15_8a_hand_eye_geometry.md) 工程完成；两种安装的闭环/AX=XB 与退化实验，无求解器；本人已确认 Run/Modify/Explain，Learning Mastered。
S12.8b [Hand-Eye Calibration Package](15_8b_hand_eye_calibration.md) 工程完成；两种安装 NumPy 分步估计、12/6 pose split、噪声与近轴对照；本人已确认 Run/Modify/Explain，Learning Mastered。
S13.1 [Perception Pose Package](16_vision_based_manipulation.md) 工程完成；packet/quality/time 检查、actual base chain、truth 隔离；本人已确认 Run/Modify/Explain，Learning Mastered。
S13.2 [Grasp Pose Package](16_2_grasp_pose_generation.md) 工程完成；四朝向/width/两端点 local IK，尺寸对照；本人已确认 Run/Modify/Explain，Learning Mastered。
S13.3a [Vision-to-Motion Package](16_3a_vision_to_motion.md) Engineering Complete；彩色点图像检测/PnP、upright prior、连续 home/pre/approach、偏置补偿与独立误差评价；本人已于2026-10-06确认实验、预测与五项Explain，Learning Mastered。
S13.3b [Vision Pick-and-Place Package](16_3b_vision_pick_place.md) Engineering Complete；连续home→取放/支撑事件/释放/退让，object truth只作评价，逐阶段与累计relative change分别报告；本人已确认Run/Modify/Explain并补正开口计算，Learning Mastered。
S13.4 [Error Propagation Package](16_4_perception_noise.md) Engineering Complete；单一post-fit pose/extrinsic fault、显式frame/pivot、精确传播、独立pipeline outcomes；本人于2026-10-06确认Run/Modify/Explain，Learning Mastered。
S13.5 [Repeated Trials Package](16_5_repeated_trials.md) Engineering Complete；seeded场景/观测、全部attempts统计、成功条件误差、失败phase、Wilson区间与wall/sim time；本人于2026-10-06确认Run/Modify/Explain，Learning Mastered。
S14.1 [Collision Checking Package](17_motion_planning.md) Engineering Complete：独立MjData、具体pair/phase/depth政策、持物变换与13个固定配置；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.2 [Configuration Space Package](17_2_configuration_space.md) Engineering Complete：二维障碍、含端点edge采样、粗细分辨率与独立解析评分；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.3 [RRT Package](17_3_rrt.md) Engineering Complete：单树sample/nearest/steer/edge/parent、固定seed与三类预算，独立解析评分；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.4 [RRT-Connect Package](17_4_rrt_connect.md) Engineering Complete：EXTEND三态/CONNECT循环、双树回溯拼接、8seed与统一预算比较；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.5 [Path Smoothing Package](17_5_path_smoothing.md) Engineering Complete：vertex shortcut、长度对比与最终全部边复查、粗oracle反例；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.6 [Timing Package](17_6_time_parameterization.md) Engineering Complete：逐段解析速度/加速度限制配时、零端速停点、C1与加速度jump；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.7a [UR5e Package](17_7a_ur5e_obstacle_planning.md) Engineering Complete：私有IK、六维双树/shortcut/cubic、reference与actual分别检查；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。S14.7b [Collision-Aware Pick Package](17_7b_collision_aware_pick_place.md) Engineering Complete：名义T_GO/phase接触、held双树与shortcut、自由物体实际取放、3seed与失败统计；本人确认实验、预测与Explain，Learning Mastered。Stage14学习项均完成；S15.1 Engineering Complete，系统Python/Jazzy默认与Modify真实收发通过；本人已确认Run/Modify/Explain，Learning Mastered；S15.2已工程完成且本人确认Run/Modify/Explain，Learning Mastered，见[Service学习包](18_2_ros2_service.md)。S15.3已明确请求并准备[Action包](18_3_ros2_action.md)，核心真实action实验与直接graph通过，Engineering Complete；CLI daemon图查询未解决；本人已确认Run/Modify/Explain，Learning Mastered；S15.4已完成[TF2工程包](18_4_ros2_tf2.md)且本人确认Run/Modify/Explain，Learning Mastered；S15.5已按请求完成[URDF/JointState工程包](18_5_urdf_joint_states.md)，本人已确认Run/Modify并补齐Explain，Learning Mastered；S15.6a已按请求准备[adapters包](18_6a_ros2_adapters.md)，独立worker/IPC、typed接口build及完整ROS执行/拒绝/cancel已通过，Engineering Complete；本人确认Run/Modify/Explain，Learning Mastered；S15.6b已按请求完成[完整manipulation包](18_6b_ros2_manipulation.md)：typed task action、视觉抓放、sim clock/actual JointState/TF、取消/超时/输入拒绝/闭爪失败与回归通过，Engineering Complete；本人确认Run/Modify并补正初始planning与后续action的范围，Learning Mastered；Stage15学习项全部完成。
