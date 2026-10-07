# Stage15 — ROS2 Integration

入口：[完整学习包](../../docs/18_ros2_integration.md)。
代码：[node_topic.py](node_topic.py)，独立 publisher/subscriber 两进程，Int32 序列与收发判据。
本课依赖 ROS2 Jazzy 的 rclpy/std_msgs（apt 管理），不依赖 MuJoCo、NumPy 或 GPU。
不把 ROS2 apt 包写入 pip requirements.txt；不使用 pip install rclpy。

2026-10-07 / Zero：系统 Python/Jazzy imports、默认/.1s双进程完整序列、
CLI图检查、无对端超时和非法参数拒绝通过；Engineering Complete；本人随后确认实验与预测并完成五问Explain，Learning Mastered。
状态唯一来源：[根 README](../../README.md#stage-15--ros2-integration)。
本人已授权 ROS2 使用系统 Python；AGENTS 已记录专用例外。安装命令见学习包。

## S15.2 — Service

[学习包](../../docs/18_2_ros2_service.md)、[pose_service.py](pose_service.py)。
依赖apt ros-jazzy-rclpy、ros-jazzy-std-srvs；无需colcon、自定义接口build或pip依赖。
Trigger空请求检查服务端缓存的world XYZ，返回success/message；不返回pose/path，不是IK/planner。
2026-10-07 / Zero：成功、越界/缺失/非有限数据拒绝、发现/响应超时、迟到server完成、
CLI type/call与绕过daemon的list通过；Engineering Complete；本人已确认实验与预测并回答五问Explain，Learning Mastered。

## S15.3 — Action

[学习包](../../docs/18_3_ros2_action.md)、[trajectory_action.py](trajectory_action.py)。
依赖 apt ros-jazzy-rclpy、ros-jazzy-control-msgs、ros-jazzy-trajectory-msgs、ros-jazzy-action-msgs。
control_msgs 提供标准 FollowJointTrajectory goal/feedback/result；无需ROS控制器框架或colcon。
只执行单软件关节的两点线性reference；无MuJoCo、真实控制器或物理停止保证。

2026-10-07 / Zero：成功、取消、abort、非法goal、并发busy、超时主动cancel、无server/guards
实测通过，取消terminal后无reference更新；直接graph通过。CLI daemon list/info空结果
未解决，不报告CLI查询通过；Engineering Complete；本人确认实验与预测完成并回答五问Explain，Learning Mastered。

## S15.4 — TF2

[学习包](../../docs/18_4_ros2_tf2.md)、[tf2_frames.py](tf2_frames.py)。
apt依赖rclpy、tf2-ros-py、tf2-py、geometry-msgs、tf2-msgs（均ros-jazzy前缀）；无新安装或pip包。
合成world/base/camera_optical/object/tool树，静态外参、动态观测、时间/年龄政策；非实时UR5e数据。
2026-10-07 / Zero：默认/移动camera、late listener静态获取、历史插值/逆/越界、
滞后stamp、停发early缓存stale与late缺失、guards通过；Engineering Complete；本人已确认实验与预测并回答五问Explain，Learning Mastered。

## S15.5 — URDF / JointState

[学习包](../../docs/18_5_urdf_joint_states.md)、[教学URDF](ur5e_kinematics.urdf)。
[urdf_reference.py](urdf_reference.py)：conda mujoco侧，需要mujoco/numpy/mujoco-menagerie，生成ignored FK JSON。
[joint_states_tf.py](joint_states_tf.py)：系统ROS侧，需要apt rclpy/sensor-msgs/tf2-ros-py/
robot-state-publisher/ament-index-python（ros-jazzy前缀），真实imports已核验，无新安装。
URDF只作运动学，无惯性/碰撞/controller，velocity/effort为占位；tool对应attachment_site。
2026-10-07 / Zero：3配置全frame MuJoCo/URDF/RSP比对、成对重排、错配拒绝、delta .4通过。
Engineering Complete；本人确认Run/Modify并补齐Explain固定变换具体值，Learning Mastered；无实时MuJoCo bridge或硬件验证。

## S15.6a — ROS2 adapters

[学习包](../../docs/18_6a_ros2_adapters.md)；[算法worker](adapter_worker.py)、[ROS adapters](ros_adapters.py)、
[PlanPose接口](s15_interfaces/srv/PlanPose.srv)、[build](build_interfaces.sh)、[launcher](run_adapters.sh)。
conda：mujoco/numpy/mujoco-menagerie/matplotlib（复用module顶层import），均requirements已声明。
ROS apt：rclpy/geometry-msgs/trajectory-msgs/control-msgs/action-msgs（ros-jazzy前缀）；
构建：cmake/make/g++、ros-jazzy-ament-cmake、ros-jazzy-rosidl-default-generators/runtime，
及ROS依赖自带系统Python构建包。CMake生成服务只放ignored tmp，固定system Python。
准备阶段缺g++；本人完成sudo安装后接口build/真实imports/type support通过。
2026-10-07 / Zero：standalone真实MuJoCo执行通过；IPC observe/plan/tick/stop、
停后sim clock/qpos不变、plan消费/非法workspace拒绝通过；syntax/bash -n/links通过。
完整ROS链实测SUCCEEDED，bad-frame/stale service拒绝且sim time0，cancel终态后state稳定。
Engineering Complete；本人确认实验与预测并回答五问Explain，Learning Mastered；启动命令见学习包。
本课无collision planner/gripper/真实视觉；取消暂停仿真，不保证物理制动。

## S15.6b — 完整 manipulation pipeline

[学习包](../../docs/18_6b_ros2_manipulation.md)、[worker](manipulation_worker.py)、
[ROS节点](manipulation_ros.py)、[launcher](run_manipulation.sh)。
新增typed PlanManipulation service与ExecuteManipulation action；接口包0.0.2保留PlanPose，需重新build。
conda复用S13.3b合成landmark/PnP与自由物体接触抓放，依赖前述数值包及requirements中的opencv-python-headless。
ROS apt另需rosgraph-msgs/sensor-msgs/tf2-ros-py/robot-state-publisher/ament-index-python（ros-jazzy前缀）。
2026-10-07 / Zero：默认16phase SUCCEEDED、四类输入拒绝、transfer取消、wall超时、close失败、
actual JointState/clock/终态TF与独立FK、旧S13.3b/S15.6a回归通过。Engineering Complete；本人确认Run/Modify并补正Explain的service规划范围，Learning Mastered。
取消仅暂停仿真并保留状态；固定场景，无障碍RRT、GUI、真实相机或硬件验证。
