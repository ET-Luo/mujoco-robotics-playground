# S15.1 — ROS2 环境与 node/topic

Problem：过去 perception、planning、execution 可以在一个 Python 程序里顺序调用。
现在学习两个独立进程怎样交换数据，并能观察是谁发、谁收、有没有收到。
本课只建立通信基础；状态见[根 README](../README.md#stage-15--ros2-integration)。

## Why → Intuition → Core Concepts

机器人相机持续产生观测，控制器持续接收状态；二者不必在同一进程中。
node 是 ROS 图中的参与者，topic 是有名称和消息类型的数据流。
同一进程可以有多个 node，node 并不等于操作系统进程。
publisher 发布消息；subscriber 的 callback 处理消息；executor 在 spin 时调度 callback。
创建 timer 不会自行启动后台执行线程。

```mermaid
flowchart LR
  P[进程 A: publisher node] -->|/s15_1/sequence : Int32| S[进程 B: subscriber node]
```

topic 的名字不是全局变量；DDS/RMW 负责发现与运输。
双方需要相同 domain、匹配 topic/type 和兼容 QoS。
本课 reliable + volatile + keep-last depth 10：请求可靠运输，保留有限历史；
新加入的订阅者不会因此获得发布者过去发出的所有消息。
reliable 不等于应用处理成功，也不是硬实时保证。

## 环境边界与安装方案

选 ROS2 Jazzy / Ubuntu 24.04 / CPU / ros-base（无需 RViz）。
官方 deb 的 Python 扩展需要匹配构建解释器；不能仅因 conda 与系统都是 Python 3.12
就假定兼容。来源：[官方 Python 边界说明](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/Using-Python-Packages.rst)。

**本人已于 2026-10-07 授权 ROS2 使用系统 Python；例外已写入 AGENTS.md。**
MuJoCo 继续使用 conda mujoco；ROS2 专用终端使用 Ubuntu 系统 Python；
本课两个节点都在 ROS2 侧，不做跨解释器 MuJoCo adapter。
不修改系统默认 Python，不向系统 pip 安装包，不将 /opt/ros 加进 conda PYTHONPATH。

本机 2026-10-07 / Zero：WSL2 kernel 6.18.33.2，Ubuntu 24.04.5；
/opt/ros 不存在，ros2 不在 PATH，未发现 ros-jazzy 安装；sudo -n true 报需要密码。
激活已存在 mujoco 后 Python 3.12.14，rclpy/std_msgs 均未找到。
无需安装前课的 NumPy/OpenCV/Torch；requirements.txt 已读且未改。

安装由本人在终端输入 sudo 密码完成，按
[官方 Ubuntu deb 安装指南](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst)
及其 repository/locale includes 设置 apt 源，再选择 `sudo apt install ros-jazzy-ros-base`。
先用 `apt-get -s install ros-jazzy-ros-base` 查看计划；不要为本课盲目 full-upgrade。
apt 源尚未配置时，单独执行安装命令不足以完成安装。
准备阶段助手未安装或修改系统源；本人随后完成 apt 安装，现场验证见 Actual。助手未请求或记录密码。
已检查 UTF-8 locale、Ubuntu universe/updates/backports 均存在。
2026-10-07 核验官方 ros-apt-source 最新 release 为 1.3.0，下载到 /tmp 并检查 deb metadata。
在本机由本人执行（下载缓存若不存在则先执行 curl）：

```bash
curl --fail --location -o /tmp/s15_1_ros2-apt-source_1.3.0.noble_all.deb https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb
sudo dpkg -i /tmp/s15_1_ros2-apt-source_1.3.0.noble_all.deb
sudo apt update
apt-get -s install ros-jazzy-ros-base
sudo apt install ros-jazzy-ros-base
```

1.3.0 是当日观察，不是要求所有机器固定该版本；后续安装重新核对官方 release。
系统 Python 现场核验为 /usr/bin/python3 / 3.12.3，退出 conda 后 rclpy/std_msgs 仍缺失。

安装后，每个 ROS2 终端准备：

```bash
cd /home/lucas/projects/mujoco-robotics-playground
# 若启动 shell 自动激活 base，逐层退出，直到环境名为空
while [ -n "${CONDA_PREFIX:-}" ]; do conda deactivate; done
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=51
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pwd
echo "${CONDA_DEFAULT_ENV:-}"
which python3  # 必须是 /usr/bin/python3
printenv ROS_DISTRO  # jazzy
/usr/bin/python3 -c 'import sys, rclpy, std_msgs; from std_msgs.msg import Int32; print(sys.version); print(rclpy.__file__, std_msgs.__file__); print(Int32())'
dpkg-query -W ros-jazzy-rclpy ros-jazzy-std-msgs
```

以上 module path、apt 版本、真实导入与通信结果必须在当前机器重新记录。
系统解释器若不是预期路径应停止，不要自动退回 conda。

## Mathematics → Math-to-Code

这里没有动力学。消息为 dimensionless 整数 k∈{0,…,N−1}，Int32 标量。
发布周期 Δt 单位 s，名义频率 f=1/Δt 单位 Hz；默认 0.5s→2Hz。
实际 callback 时间受调度影响，不宣称严格周期。topic 消息没有 frame 或 timestamp。
这适合通信实验，未来机器人状态消息需要明确 rad/m、frame 和时间，留到对应小课。

[代码](../examples/16_ros2_integration/node_topic.py) 保持 API 可见：
`rclpy.init` 初始化 context；`Node(name)` 创建图参与者；
`create_publisher(Int32, topic, qos)` 返回发送 handle；
`create_subscription(Int32, topic, callback, qos)` 返回订阅 handle；
`create_timer(period_s, callback)` 返回 timer；`publish(msg)` 提交消息，不返回接收确认。
`spin_once(node, timeout_sec=0.1)` 调度就绪回调、原地更新计数；
`destroy_node` 与 `shutdown` 分别释放节点及 context。

## Minimal Experiment → Expected / Actual Result

环境准备后，先启动终端 A 的 subscriber，再启动终端 B 的 publisher：

```bash
# A
/usr/bin/python3 examples/16_ros2_integration/node_topic.py subscriber
# B
/usr/bin/python3 examples/16_ros2_integration/node_topic.py publisher
```

subscriber 预期打印 0…9 和 `sequence_passed=True`，exit0。
publisher 等待发现订阅者再发送；20s 后报告 published=10/10 并退出。
发布计数不是接收证据；以 subscriber 的序列检查为本课判据。
订阅者缺席时 publisher 超时并 exit1；节点提前停止/不匹配时 subscriber 超时 exit1。
20s 是合作式 wall-time 预算，callback 阻塞会延迟退出。

运行期间第三个同环境终端观察：

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /s15_1/sequence --verbose
ros2 topic echo /s15_1/sequence
```

echo 自己也是订阅者，会触发 publisher 的发现门槛；诊断时不能把它当目标 subscriber 的 ACK。
节点结束后 graph 不再显示活动节点是正常现象。

Actual（2026-10-07 / Zero）：本人完成 apt 安装后，助手重新核验空 conda 环境、
/usr/bin/python3 3.12.3、ROS_DISTRO=jazzy；rclpy/std_msgs 真实模块路径均在 /opt/ros/jazzy。
apt rclpy=7.1.12-1noble.20260912.162354、std-msgs=5.3.8-1noble.20260911.094424、
ros-base=0.11.0-1noble.20261006.035407；dpkg --audit 无输出。

| 实验 | 实测结果 |
| --- | --- |
| 默认 period=.5s，两个进程 | 收到0…9，sequence_passed=True，两进程exit0 |
| Modify publisher --period .1s | 收到0…9，sequence_passed=True，两进程exit0 |
| 无对端，各 --timeout 1 | subscriber空序列、publisher发布0/10，均exit1 |
| period nan / timeout 0 / count 0 | 均exit2，拒绝非法参数 |
| node list / topic list -t / topic info --verbose | 两节点、Int32类型、1 publisher/1 subscriber，reliable/volatile |

从进程启动至 subscriber退出约5.492s/1.429s（包括启动/发现/退出开销，非性能benchmark）。
消息日志首末接收跨度约4.494s/.892s，符合9个间隔的名义4.5s/.9s。
publisher每组仍存活约20s；不能将其总耗时当作发送耗时。
CLI endpoint history depth 显示 UNKNOWN，不能据此声称 CLI 已验证 depth10；
代码 QoSProfile 明确设置 depth10。
工程日志/summary仅放 ignored tmp/s15_1_node_topic/，没有 GUI、MuJoCo、跨机器或硬件验证。
Engineering Complete；Learning Run/Modify/Explain 均未确认。

## Explanation → Failure Cases → Robotics Context

没有数据首先看节点是否在运行、是否 spin、domain/topic/type 是否相同，再看 QoS 与网络。
本课单机 localhost 缩小网络因素，不验证跨机器发现或 WSL↔Windows 通信。
两个节点独立运行，无固定启动顺序保证；等待 subscription count 只避免本课明显的初始丢包，
不能证明目标订阅者已处理数据。多个 subscriber、断线重连或同 topic 额外 publisher 超出本课序列假设。
每端 count 必须一致；timer 周期太长可能超过 timeout。
无 rclpy 时是环境缺失；扩展 ABI/共享库错误不能用反复 pip reinstall 掩盖。

应用：以后 perception 发布观测、execution 发布进度；短请求与长任务分别留到 S15.2/S15.3。
本课没有连接 MuJoCo、关节控制、service/action、TF2 或抓取 pipeline。

## Interview Capsule / Must Remember

30 秒：ROS2 node 通过带类型的 topic 异步收发。executor 调度 timer 与订阅回调，
通信成立需要名称、类型、domain 与 QoS 匹配。发布成功和应用处理成功必须分别验证。

2 分钟：用两个进程讲解 init→Node→publisher/subscription→spin→cleanup。
说明可靠运输与历史保留是不同维度，volatile 不重放旧消息。
用缺失 subscriber、不同 domain、未 spin 三种情况解释为什么“代码没报错”不代表收到了数据。
最后说明 apt ROS2 与 conda MuJoCo 的解释器/二进制边界，后续用薄 adapter 连接。

Must Remember：topic 不存共享变量；timer 需要 executor；reliable 不是应用 ACK；
频率不是硬实时；环境检查与真实收发都要通过才算工程实验完成。

## My Verification — Run / Modify / Explain

状态仅在根 README 更新；本人未报告前不勾选。
Run：按上面两个终端命令，保存 subscriber 序列与节点/topic 观察结果。
Modify：两端保持 count=10，将 publisher 改为 `--period 0.1`；预测 2Hz→10Hz，
观察收完消息更快，publisher 总存活时间仍由 timeout 决定。
Explain：

1. node、进程与 topic 的关系是什么？
2. 为什么创建 timer 后还要 spin？
3. 为什么 published=10 不能证明 subscriber 收到 10 条？
4. reliable 与 volatile 分别决定什么？
5. 为什么 apt ROS2 不能直接混进 mujoco conda 环境？

下一小步是本人完成本课 Run/Modify/Explain；不自动推进 S15.2。
