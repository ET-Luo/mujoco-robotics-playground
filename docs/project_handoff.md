# 项目经验与进度交接

最后整理：2026-10-05（S12.6 Engineering Complete，Learning 待本人验证）。
新会话先读 [AGENTS.md](../AGENTS.md) → 本文 → [README](../README.md) → 当次示例。
README 是 Stage 10 起 Engineering / Learning 的唯一状态来源。

## 当前边界与下一步

本人已明确完成 S12.1–S12.4 Run/Modify/Explain，并明确授权 S12.5。
S12.5 PnP 的 Code + Experiment + Docs 已完成；本人随后明确确认实验与验证均完成，
五问 Explain 覆盖核心概念，Run/Modify/Explain 全部确认，Learning Mastered。
仅合成对应点与已知 K/d → T_CO；没有 detector、base transform 或 manipulation。
用户本轮明确请求推进 S12.6，已完成工程；学习验证仍待本人，STOP，不自动开始 S12.7a。

## S12.5 实现与现场验证（2026-10-04）

新增 [pnp_pose.py](../examples/13_perception_geometry/pnp_pose.py) 与
[完整教学包](15_5_pnp_pose.md)。12 个有身份的非共面 3D landmark，米制 object geometry；
SOLVEPNP_ITERATIVE 返回 object→optical camera R/t，无 truth initial guess。
复用 camera_calibration 的 projection/RMS helper，不重新执行标定；没有新增依赖。

保存编辑前既有未提交工作。核验项目目录、WSL2 kernel 6.6.87.2、Ubuntu 24.04.5；
激活 mujoco 后在执行同 shell 核验环境名与 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
现场 Python 3.12.14 / NumPy 2.5.3 / cv2 4.14.0；3.11 兼容目标未执行。

```bash
python examples/13_perception_geometry/pnp_pose.py
python examples/13_perception_geometry/pnp_pose.py --noise-px 0.8
python examples/13_perception_geometry/pnp_pose.py --noise-px 0
```

三命令均退出 0，无 import error。default / Modify 的 translation error=
0.000389763 / 0.001472155 m，rotation error=0.378563 / 1.555400 degree，
noisy reprojection RMS=0.258050 / 1.029309 pixel。零噪声 t<1e-8 m、rotation<1e-4 degree、RMS<1e-6 pixel。
默认错误 focal+5% 的 RMS=0.264791 pixel，但 translation error=0.038792973 m；
低 pixel residual 不等于准确 metric pose。单次 solve+checks 40.425/10.511/13.444 ms，非稳定性能 benchmark。

独立核对三组 NPZ/CSV shapes、saved reprojection 与 pose error、inverse 与 positive depth；
固定 geometry、noise ×4 通过。too-few/nonfinite/coplanar guards 拒绝，脚本内 collinear 拒绝；
CLI nan/negative noise 退出 2。默认 PNG 已目视检查，为对应点/残差图，不是 RGB camera photograph。
产物只在 ignored tmp/s12_5_pnp_noise*/。Markdown links 与 git diff --check 通过。
未重跑前课/P0，未运行 GUI、真实视觉、机器人执行；PASS 仅表示本课工程检查通过。

## S12.4 历史验证（2026-10-04）

[Calibration package](15_4_camera_calibration.md)：24 train / 8 known-pose held-out boards，
输入 corners/metric geometry，估计 K/d 与各 board→camera pose，k3 固定 0。
默认/0.6 px/零噪声 training RMS=0.201785/0.807089/0.000010 pixel；
known-pose clean held-out RMS=1.514152/6.207453/0.000043 pixel。
尺度 ×2 使 tvec ×2 而 K/d/RMS 基本不变；不能据小 training RMS 认定 K 准确。
原不可靠 held-out threshold 已去除，实际结果/失败分析保留该课。
本课新增 opencv-python-headless>=4.10,<5，在 mujoco 安装 4.14.0.94；不改 NumPy。
本人已确认实验、对比与 Explain，Learning Mastered；本轮不重跑标定。

## 已完成 Stage 12 前课（历史证据，2026-10-04）

- S12.1：[frame package](15_robot_perception_geometry.md)，base/world axes 核对，camera 移动后
  p_CO 变化但恢复 p_BO 不变；本人三项确认，Mastered。
- S12.2：[projection package](15_2_pinhole_projection.md)，focal scale 对比、depth gate、
  ray ambiguity；本人实验、预测与 Explain 确认，Mastered。
- S12.3：[CPU RGB/depth package](15_3_rgb_depth_acquisition.md)，EGL llvmpipe、
  frame axes、surface axial depth 与 shapes 检查；本人实验/对比/Explain 确认，Mastered。
  OSMesa probe 因缺库失败，EGL 软件替代成功；详见该课，不重新安装 OSMesa。

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
[Stage 15](18_ros2_integration.md)骨架；OpenCV 在 S12.4 新增，ROS2 未安装。

## S12.1 历史验证与已知限制（2026-10-04）

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

本人已完成 [S12.5 Learning Package](15_5_pnp_pose.md) 实验、验证与 Explain；
README 三项已按本人明确报告更新。精确补充：尺度来自已知 metric 3D geometry，
PnP 求该尺度下的 R/t；相机标定估 K/d，而 PnP 固定它们。
本次仅更新 README、学习包、示例说明、roadmap、P1 plan 与 handoff；
检查相对文档链接、状态一致性及 git diff --check，未重跑实验/仿真/GUI。
上述 runtime 证据沿用 2026-10-04 工程验证，不是本次新运行。
本轮已按明确请求实现 S12.6；下一步为本人 Run/Modify/Explain，随后可选 S12.7a 等待明确请求。

## 环境与历史 GUI 经验

执行前必须重新核验，不以以上版本当作未来保证。当前不是 mujoco 时可自动用
`source /home/lucas/miniconda3/etc/profile.d/conda.sh`、`conda activate mujoco`，随后同 shell
检查环境名与 interpreter。禁止 base/system Python 运行、安装或 sudo pip。

用户于 2026-09-26 已确认 GUI 显示恢复；不继续把 WARN:COPY MODE 视作当前阻塞。
历史 viewer native 退出崩溃与 WSLg 共享内存问题需与 headless 验证分开；本轮未复测 GUI。
passive viewer 的 Python 循环没有暂停回调，空格不会暂停它。
历史具体错误与诊断见[MuJoCo notes](mujoco_notes.md)、[基础课](01_mujoco_basics.md)、
[UR5e 示例](../examples/02_ur5e_basics/README.md)。

## S12.6 现场工程验证（2026-10-05）

[学习包](15_6_rgbd_back_projection.md)、[代码](../examples/13_perception_geometry/rgbd_back_projection.py)。
起始 pwd 正确、git status --short 为空；WSL2 6.6.87.2 / Ubuntu 24.04.5。
base 自动切到 mujoco，同执行 shell 核验环境及 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14 / MuJoCo 3.14.0 / NumPy 2.5.3；3.11 兼容目标未执行。无新增依赖。

```bash
python examples/13_perception_geometry/rgbd_back_projection.py
python examples/13_perception_geometry/rgbd_back_projection.py --camera-height 1.0
```

两命令退出 0、无 import error，CPU EGL llvmpipe。各 76800 有效点；max pixel error
5.68e-14/2.84e-14 px，floor z≈0、top z≈0.06 m；实际 base 为 world xy 反向。
错误 normalized ray × Z 使离轴 floor patch mean z=0.126922/0.158652 m。
内置独立手算、invalid/empty depth、surface truth、base/world 链与 RGB mask/order 检查通过。
默认 PNG 目视检查；显示网格稀疏采样，NPZ 保存完整点云，产物仅 ignored tmp/s12_6_rgbd_z*/。
已知外参、ideal aligned RGB-D；无 GUI、真实相机、pose/ICP、动态操作验证。
Engineering 完成；Learning Run/Modify/Explain 未勾选。本人下一小任务：升高相机并解释
K/depth/extrinsic/表面高度与视野变化；不自动开始 S12.7a。
