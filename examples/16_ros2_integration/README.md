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
