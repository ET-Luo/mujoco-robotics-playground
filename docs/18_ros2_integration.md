# Stage 15 — ROS2 Integration

规划骨架，未安装、未实现、未实验。任务/状态见[根 README](../README.md#stage-15--ros2-integration)。
前置：perception/planning 原理与接口完成后，再选兼容 Ubuntu 24.04 的 ROS2 环境并核验。

Problem / Why：把已验证算法连接成可观察、可取消、frame/time 明确的机器人系统。
Intuition / Core Concepts：topic 流式数据、service 短请求、action 长任务；TF2 管理时变 frame tree。
Mathematics：transform chain 延续 Stage 12；timestamp/frame_id；joint rad、Cartesian m。
Math-to-Code：薄 ROS2 adapter，算法继续可独立 CPU 运行；MuJoCo 与 ROS2 解释器隔离方案待 S15.1 核验。
Minimal Experiment / Expected Result：先最小 node/topic，再 service/action/TF2/URDF，最后 manipulation pipeline。
Actual Result / Explanation：待实测，不预设 ROS2 已可用。
Failure Cases：QoS 不兼容、过期 transform、时钟不一致、重复 TF parent、cancel 后仍执行、joint 名称顺序错误。
Robotics Applications：perception node→planning service→execution action→MuJoCo adapter。
Interview Capsule：后续按 Task 整理 30 秒/2 分钟材料。
Must Remember：URDF 描述结构，不能代替 MuJoCo 接触动力学；明确 TF base 与 MuJoCo world 的关系。
My Verification：待本人 Run / Modify / Explain。
