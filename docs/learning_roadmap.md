# 学习路线与仓库评估

更新：2026-10-06。任务与 Engineering/Learning checkbox 唯一维护在
[README](../README.md#learning-roadmap)。助手验证不能替代本人 Run/Modify/Explain。
Stage 0–11 已由本人确认完成；S12.1–S12.8b 已 Engineering Complete + Learning Mastered。

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

## 当前 handoff

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
S13.5 [Repeated Trials Package](16_5_repeated_trials.md) Engineering Complete；seeded场景/观测、全部attempts统计、成功条件误差、失败phase、Wilson区间与wall/sim time；Learning待本人Run/Modify/Explain。
Stage14–15仍未实现；不自动开始Stage14。
