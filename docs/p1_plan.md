# P1 — Perception, Planning & Robot Software Integration

2026-10-04 静态审查；P0 runtime 数字引用同日既有记录，本轮未重跑 P0。
[任务与状态唯一来源](../README.md#stage-12--robot-perception-geometry)。

## P0 最终审查与复用

用户本轮明确声明 P0 已完成。Stage 10 工程全部完成；S10.1/2 原 Learning 空框
按本次整体确认同步，S10.3–10.8b 与 Stage 11 原先已记录 Mastered。Stage 0–9 不改。

| 已审查实现 | 可直接复用 | 进入 P1 前需明确的边界 |
| --- | --- | --- |
| [pose_planning](../examples/12_pick_place/pose_planning.py) | T_WO→T_WG 的 grasp offset，local approach 约定 | 物体 pose 是已知输入；base rotation 目前写死 |
| [pregrasp_motion](../examples/12_pick_place/pregrasp_motion.py) | build_model、site_pose、joint_addresses、bounded DLS、cubic_reference、contact_names | IK 消费 W pose，原地修改 data；orientation error 在 π 附近拒绝；混合 m/rad scaling；sample contacts 不是连续碰撞证据 |
| [transfer_and_descend](../examples/12_pick_place/transfer_and_descend.py) | run_reference、run_to_support、实际 relative offset、速度/slip/support 证据 | 初始直接设置 q_grasp，未执行 home→approach 动力学；部分阶段清零 qvel，不能称通用执行接口 |
| [release_and_retreat](../examples/12_pick_place/release_and_retreat.py) | support-before-open、separation-before-retreat、最终低速与接触检查 | 最终 position error 不是完整 orientation error；只统计 ground/object 和 finger/object，未普遍拒绝 arm/environment contact |
| [robustness_trials](../examples/12_pick_place/robustness_trials.py) | fixed seed、trial accounting、failure stage、successful error | sampled xy 直接输入 planner；成功率断言仅要求 >0，不保证所有 trial 成功；failure stage 较粗 |

既有 S11.7 两 seed 各 20/20；xy±5 mm、friction [1.8,2.2]、mass [0.045,0.055] kg。
既有 mean final position error ≈2.30 mm，max≈2.35 mm，仅支持小范围 known-pose manipulation。
S11.3 单独验证 home→pre-grasp 几何路径，S11.4 单独验证固定物体 approach；
S11.7 最终组合从 grasp 初始化开始。因此 P0 已完成既定学习任务，不宣称通用从 home 出发、
全阶段无碰撞、视觉估计或现实硬件部署已完成。

复用函数前保持 frame 和 data mutation 可见；目前不重构 P0、不引入公共 controller 框架。
S13.3a 才将 estimated object pose 注入模型/执行流程，truth 仅用于评价，禁止直接作为 planner 输入。
S14.7 才接入更完整的碰撞策略。S12.1 使用实际 base cache，避免把旧模型约定当永久事实。

## P1 数据流与顺序

```text
RGB / depth + K + timestamp
 → T_CO estimate + quality
 → T_BO = T_BC T_CO
 → T_BG = T_BO T_OG → T_WG = T_WB T_BG (existing IK interface)
 → joint goal → collision-aware path → timed reference
 → UR5e actuators + grasp/release evidence
 → ROS2 nodes / TF2 / service / action
```

所有 Task 约 0.5～2h；大系统拆成接口、实验与评价多个 Task。
Stage 12 先合成几何，再 CPU 图像与点云；Stage 13 才接操作；Stage 14 加避障；Stage 15 最后集成。
每次 Codex 只完成当次授权 Task 的 Code + Experiment + Docs，然后用户 Run + Modify + Explain。

## 硬件与依赖预算

Ryzen 7 7840HS、无 NVIDIA GPU、WSL2 Ubuntu 24.04，CPU-first。
现有 NumPy/MuJoCo/Matplotlib 足够 S12.1；S12.2 几何仍不新增依赖。
OpenCV 到 calibration/PnP Task 才说明用途、写 requirements 并在 mujoco 中安装。
RGB-D 优先 MuJoCo 合成数据；不要求实体相机。ICP 用小规模 NumPy NN + SVD，自行实现；
限制点数/迭代并记录耗时。RRT/RRT-Connect 自行实现，不用 MoveIt。
ROS2 依赖/版本/WSL2 通信与 conda 隔离在 S15.1 再现场验证，不在本轮安装或预设可用。
无 Isaac、YOLO、SAM、大模型训练；现有 RL 依赖保留，P1 不增加 RL 任务。

## 阶段验收输出

| Stage | 主要输出 | 本轮范围 |
| --- | --- | --- |
| 12 | 可核对的 frame/projection/pose/point-cloud/calibration 小实验 | 仅 S12.1 已实现 |
| 13 | 真正由图像估计驱动的 pick-and-place 与噪声重复试验 | docs skeleton |
| 14 | 带 obstacle、self/held-object 检查的 joint path 与执行统计 | docs skeleton |
| 15 | frame/time 明确的 ROS2 manipulation pipeline | docs skeleton |

更新（2026-10-04）：本人已完成 S12.1 Run/Modify/Explain 并明确授权 S12.2；
S12.2 工程完成，且本人随后明确确认实验、预测与 Explain 完成，Learning Mastered，
见 [Learning Package](15_2_pinhole_projection.md)。下一可选任务 S12.3 等待明确请求；
不自动推进。上文首轮审计的范围描述保留为 dated observation。

更新（2026-10-04）：本人明确授权 S12.3，CPU EGL/llvmpipe RGB/depth 获取工程完成，
见 [Learning Package](15_3_rgb_depth_acquisition.md)。本人随后确认实验、对比与 Explain 完成，Learning Mastered；
不自动推进 S12.4 camera calibration，未安装 OpenCV。

更新（2026-10-04）：本人明确授权 S12.4；新增 OpenCV headless 依赖与 CPU synthetic-corner
camera calibration 工程，见 [Learning Package](15_4_camera_calibration.md)。使用 24 train / 8
known-pose held-out views；本人随后明确确认实验、对比与 Explain，Learning Mastered，不自动推进 S12.5。

更新（2026-10-04）：本人明确授权 S12.5；新增 [PnP Engineering Package](15_5_pnp_pose.md)，
已知非共面 metric 3D↔2D 对应与 K/d，估计 T_CO；报告像素/位姿误差，比较 noise 与 focal+5%。
默认、0.8 px 与零噪声及输入/产物检查通过；无新增依赖；本人随后确认实验、验证与 Explain，Learning Mastered，不自动开始 S12.6。

### S12.6 进展（2026-10-05）

[RGB-D package](15_6_rgbd_back_projection.md) Engineering Complete；复用 CPU acquisition，
有效 axial depth→camera/base 表面点云，独立 floor/top/base 核验、invalid depth 与 range 错误对照。
无新增依赖；Learning Run/Modify/Explain 待本人确认，不自动开始 S12.7a。
