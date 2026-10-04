# 学习路线与仓库评估

更新：2026-10-04。任务与 Engineering/Learning checkbox 唯一维护在
[README](../README.md#learning-roadmap)。助手验证不能替代本人 Run/Modify/Explain。
Stage 0–11 已由本人确认完成；本轮 P1 仅 S12.1 工程完成，Learning 待本人验证。

## 当前代码如何使用

| 入口 | 实际代码用途 | 继续学习方式 |
| --- | --- | --- |
| examples/01–08（包括两个有意保留的 02 目录） | 既有基础、控制、平面运动学、独立夹爪小实验 | 已完成阶段不重复教学；按需查阅 |
| examples/09_ur5e_6d_pose | 官方 UR5e 完整位姿 | 复用 frame/cache 阅读经验 |
| examples/10_ur5e_6d_ik | 6D error / Jacobian / DLS / bounded iteration | 已有求解器；不重新实现基础 IK |
| examples/11_trajectory | linear/cubic reference 与解析导数 | 复用 cubic，Stage 14 才扩展路径时间化 |
| examples/12_pick_place | UR5e 集成夹爪、known-pose lift/place/release/trials | 复用模型/执行判据；接口限制见 P1 审计 |
| examples/13_perception_geometry | 仅 camera_frames.py，synthetic optical pose chain | 当前唯一新工程入口 |
| environments / rl | 既有 planar Reach / CPU PPO 学习代码与部分占位 | P1 不推进 RL |
| controllers / assets / notebooks / tests | 支持说明与部分占位 | 不为未来需求填满框架 |
| scripts/check_env.sh | 环境诊断 | 执行前核验环境，不能代替算法验证 |

详见[P0 最终审查与 P1 规划](p1_plan.md)。本轮未重跑 P0；历史数字标注在该文。
P1 顺序：geometry perception → vision manipulation → self-written planning → ROS2。
CPU NumPy 优先；OpenCV 仅 calibration/PnP 时新增；NumPy ICP 与 RRT 自行实现。

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

阅读 [S12.1 Learning Package](15_robot_perception_geometry.md)，本人运行默认实验，
移动相机后确认恢复出的 base/world object pose 不变，回答末尾五问。
Stage 13–15 笔记为必要 skeleton，未实现。Engineering 完成即 STOP；不自动开始 S12.2。
