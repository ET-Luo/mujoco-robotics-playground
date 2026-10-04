# 项目经验与进度交接

最后整理：2026-10-04（P1 开始；仅 S12.1 Engineering Complete）。
新会话先读 [AGENTS.md](../AGENTS.md) → 本文 → [README](../README.md) → 当次示例。
README 是 Stage 10 起 Engineering / Learning 的唯一状态来源。

## 当前边界与下一步

用户明确声明 P0 已完成，并要求规划 P1 Stage 12–15；本轮只完成 S12.1 Camera Frames /
Coordinate Transform 的 Engineering package。**不自动开始 S12.2**。
Codex 负责 Code + Experiment + Docs；本人负责 Run + Modify + Explain。
S12.1 的本人三项均未确认，不能标记 Mastered。下一小任务为本人执行 S12.1 handoff；
之后只有收到明确请求才开始 S12.2 pinhole/intrinsic/projection。

## P0 最终状态与审查

Stage 0–9 历史 checkbox 不改。Stage 10 工程均完成；S10.1/2 原 README Learning 空框
按本次「P0 已完成」声明同步，并注明是用户整体确认，不是助手运行追认。
S10.3–10.8b、S11.1–11.7 原先已记录本人 Run/Modify/Explain 全部 Mastered。

[本轮静态审查 / 复用清单](p1_plan.md)覆盖最终 pose_planning、pregrasp_motion、
transfer_and_descend、release_and_retreat、robustness_trials。重要边界：

- 现有 IK target 为 world-frame；真实 UR5e base 与 world 同原点但 xy 轴相反。
- S11.3 home→pre-grasp 是几何 reference 检查；S11.4 approach 是固定物体实验。
- 最终 run_to_support 直接设置 q_grasp 后闭爪；trials 没有从 home 连续执行完整动态 approach。
- sampled object xy 直接输入 planner，未验证 perception/calibration noise。
- 接触检查与 collision exclusions 有阶段适用范围；不是通用 self/obstacle/held-object planner。
- bounded DLS、cubic、model builder、actuator mapping、contact criteria、trial accounting 可复用；
  IK 原地改变 data，π 附近 orientation error 拒绝，不能隐藏这些接口限制。

既有验证（2026-10-04，本轮未重跑）：S11.7 seed=20261004/7 各 20/20；
xy±0.005 m、friction [1.8,2.2]、mass [0.045,0.055] kg；
successful final position error mean=0.002299761/0.002302795 m，
max=0.002344323/0.002346447 m。只支持这些小范围 known-pose trials。
既有 S11.6a 5 s transfer 曾因累积 relative slip 失败；更慢不保证 retention 更好。
详细 P0 实验与概念保留在[6D Pose](11_ur5e_6d_pose.md)、[6D IK](12_ur5e_6d_ik.md)、
[Trajectory](13_trajectory.md)、[Pick & Place](14_pick_place.md)及[示例 README](../examples/12_pick_place/README.md)。

## P1 路线

CPU-first / Ryzen 7 7840HS / 无 NVIDIA GPU / WSL2 Ubuntu 24.04。
Stage 12 geometry→projection→render/calibration/PnP→RGB-D→NumPy ICP→hand-eye；
Stage 13 vision estimate→grasp→完整操作→noise/trials；Stage 14 collision/C-space→RRT→
RRT-Connect→smoothing→timing→UR5e/held-object；Stage 15 才 ROS2 node/topic/service/action/
TF2/URDF/pipeline。每个 Task 0.5～2h，多步骤系统拆成小任务。
无 Isaac、YOLO、SAM、大视觉模型训练、MoveIt。新依赖仅在需要的 Task 说明并安装到 mujoco。
本轮新增[规划](p1_plan.md)、[Stage 12](15_robot_perception_geometry.md)完整首课、
[Stage 13](16_vision_based_manipulation.md)、[Stage 14](17_motion_planning.md)、
[Stage 15](18_ros2_integration.md)骨架；未安装 OpenCV 或 ROS2。

## 本轮验证与已知限制

编辑前 pwd 正确、git status --short 为空；无既有未提交修改。
现场核验：WSL2 kernel 6.6.87.2-microsoft-standard-WSL2，Ubuntu 24.04.5。
起始 base；自动激活既有 mujoco，并在执行同一 shell 重新核验：

```text
pwd: /home/lucas/projects/mujoco-robotics-playground
CONDA_DEFAULT_ENV: mujoco
python: /home/lucas/miniconda3/envs/mujoco/bin/python
Python 3.12.14 / NumPy 2.5.3 / MuJoCo 3.14.0
```

代码兼容目标 3.11，本轮未在 3.11 执行；未修改环境版本或 requirements。
先加载官方 UR5e，mj_forward 后 base world rotation 实测 diag(-1,-1,1)，position=0。
以下 S12.1 命令均退出 0，无 import error：

```bash
python examples/13_perception_geometry/camera_frames.py
python examples/13_perception_geometry/camera_frames.py --camera-x -0.25
```

camera x 从 −0.35→−0.25 m，p_CO 从 [-0.10,-0.10,0.77]→[-0.20,-0.10,0.77] m；
p_BO 恒为 [0.45,-0.20,0.03] m，恢复 p_WO 恒为 [-0.45,0.20,0.03] m。
完整 rotation/translation chain、非原点 object test point、w=0 direction、rigid inverse 通过；
reflection 拒绝、wrong order 和 mm/m 混用失败演示通过。time=0。
这是 synthetic optical observation，无 GUI、renderer、标定、PnP、动力学或抓取验证。
新增/修改 Markdown 相对文件链接与 git diff --check 已检查；详见首课笔记。

## 本人 handoff

Run：默认 camera_frames.py 命令。
Modify：--camera-x -0.25，先预测 camera 坐标变化再确认 base pose 不变。
Explain：回答[首课末尾五问](15_robot_perception_geometry.md#my-verification--run--modify--explain)。
本轮 Engineering 后 STOP，不把助手已跑 Modify 计作本人完成。

## 环境与历史 GUI 经验

执行前必须重新核验，不以以上版本当作未来保证。当前不是 mujoco 时可自动用
`source /home/lucas/miniconda3/etc/profile.d/conda.sh`、`conda activate mujoco`，随后同 shell
检查环境名与 interpreter。禁止 base/system Python 运行、安装或 sudo pip。

用户于 2026-09-26 已确认 GUI 显示恢复；不继续把 WARN:COPY MODE 视作当前阻塞。
历史 viewer native 退出崩溃与 WSLg 共享内存问题需与 headless 验证分开；本轮未复测 GUI。
passive viewer 的 Python 循环没有暂停回调，空格不会暂停它。
历史具体错误与诊断见[MuJoCo notes](mujoco_notes.md)、[基础课](01_mujoco_basics.md)、
[UR5e 示例](../examples/02_ur5e_basics/README.md)。
