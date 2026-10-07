# MuJoCo Robotics & Robot Learning Playground

## Project Introduction

以 MuJoCo + UR5e 为主线的个人学习与面试准备仓库。面向有 Python/C++ 和计算机科学
基础、正在补齐机器人学与动力学知识的学习者，逐步建立可解释的小实验。
目标岗位包括机器人软件、具身智能与 Robot Learning。

遵循 **先理解 → 再实现 → 再实验 → 再总结**。每次只处理一个小问题，保留官方 API
细节，以能解释代码、预测结果、验证假设为完成标准。

## Learning Goals

- MuJoCo：模型、状态、API 与仿真循环。
- Robotics Simulation：时间步、物理参数、接触与可重复实验。
- Robot Kinematics：坐标系、正运动学、Jacobian 与逆运动学。
- Robot Control：关节控制、PD 与笛卡尔控制。
- Manipulation：夹爪、Reach、Grasp、Pick and Place。
- Robot Learning：环境接口、奖励、PPO/SAC 与 sim-to-real 基础。
- P1：CPU geometry perception、vision manipulation、自写 motion planning，最后 ROS2 integration。

## Current Progress

P0（Stage 0–11）已由本人于 2026-10-04 明确确认完成。Stage 10–11 的细分状态见下方。
当前推进 **P1 — Perception, Planning & Robot Software Integration**；S12.1–S12.8b 已 Engineering Complete + Learning Mastered。

| 内容 | Engineering | Learning |
| --- | --- | --- |
| P0 / UR5e known-pose manipulation | 已完成规划内代码、实验、文档；最终实现静态审查完成 | 本人确认 P0 完成 |
| S12.1 Camera Frames / Coordinate Transform | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.2 Pinhole / Intrinsic / Projection | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.3 MuJoCo CPU RGB / Depth | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.4 Camera Calibration | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.5 PnP Object Pose | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.6 RGB-D Back Projection | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.7a Known-Correspondence Rigid Alignment | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.7b Nearest-Neighbor ICP Loop | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.8a Hand-Eye Geometry | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S12.8b Hand-Eye Calibration | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S13.1 Perception Pose→Base→World | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S13.2 Grasp Pose Generation | Code + Experiment + Docs 完成 | Run / Modify / Explain 完成，Mastered |
| S13.3a Vision-to-Motion | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S13.3b Vision Pick-and-Place | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S13.4 Pose / Extrinsic Error Propagation | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| S13.5 Repeated-Trial Evaluation | Code + Experiment + Docs 完成 | Run / Modify / Explain 已完成 — Mastered |
| Stage 14 | S14.1–S14.7b collision / planning / pick-place | Engineering Complete + Learning Mastered（全阶段） |
| Stage 15 | S15.1 Engineering Complete；其余小课未开始 | S15.1 Run / Modify / Explain 待本人完成 |

P0 最终 trials 使用已知 sampled object xy，且从 grasp 初始化；完整 home→approach 动力学串联、
视觉输入已由 S13.3a 在固定场景验证；通用避障仍是后续路线。历史 runtime 与本轮静态审查边界见[P1 审计](docs/p1_plan.md)。

## Environment and Quick Start

Ubuntu 24.04 / WSL2、Miniconda、VS Code WSL；CPU 优先，无需 NVIDIA 显卡。
Python 3.11 是代码兼容目标；历史验证使用 3.12.14。依赖版本未锁定。

```bash
cd ~/projects/mujoco-robotics-playground
# 仅首次且环境不存在时：conda create -n mujoco python=3.11
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
```

仅在环境为 `mujoco` 且解释器属于该环境时继续：

```bash
# 仅明确要完整初始化环境时执行（包含 Torch/RL 依赖）：
python -m pip install -r requirements.txt
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
python examples/02_ur5e_basics/main.py --headless
# GUI 已确认可见；可用长时运行练习交互：
python examples/02_ur5e_basics/main.py --viewer --steps 150000
```

基础依赖为 `mujoco`、`numpy`、`matplotlib`、`mujoco-menagerie`；既有 Stage 8 另有
Gymnasium、CPU PyTorch、Stable-Baselines3，见 requirements.txt。P1 S12.4 新增 OpenCV headless，用于 CPU calibration/PnP。Menagerie 首次使用
下载 UR5e 到用户缓存，不复制整个模型仓库。不要在 base 或系统 Python 安装依赖。
换电脑时，Git 只同步依赖声明，不同步已安装包。Agent 应核验当前任务所需版本与 imports，
按 [依赖规则](AGENTS.md#dependency-rules) 自动补齐已声明的必要依赖；窄任务不默认全量安装。
预览安装影响、核验和机器/日期记录示例见[跨电脑开发流程](docs/development_workflow.md#switching-laptops)。
当前 passive viewer 没有暂停回调，空格不会暂停 Python 仿真循环。

## Repository Structure

```text
examples/01_basic_simulation/       # 基础仿真
examples/02_ur5e_basics/            # 官方 UR5e 与状态/API
examples/02_joint_control/         # 关节目标实验
examples/03_pd_control/            # 最小力矩控制
examples/04_forward_kinematics/     # FK
examples/05_jacobian/              # Jacobian
examples/06_inverse_kinematics/    # 基础 IK
examples/07_cartesian_control/    # Cartesian tracking
examples/08_contact_and_grasping/ # 独立接触与夹爪
examples/09_ur5e_6d_pose/          # Stage 10 pose
examples/10_ur5e_6d_ik/            # UR5e 6D IK / DLS
examples/11_trajectory/            # linear / cubic reference
examples/12_pick_place/            # P0 最终 UR5e known-pose manipulation
examples/13_perception_geometry/   # P1 frames / projection / RGB-depth / calibration / PnP
controllers/ environments/ rl/     # 既有学习代码与占位，详见路线索引
assets/ scripts/ notebooks/ tests/ # 资源、工具与验证说明
docs/                             # 笔记、P1 规划、交接
AGENTS.md                         # 仓库规则
requirements.txt                  # 当前依赖；P1 本轮未新增
```

两个 `02_` 目录保留；目录编号不等于 Stage 编号。阅读顺序与笔记映射见
[学习路线索引](docs/learning_roadmap.md)。

## Learning Roadmap

每个子任务预计 **0.5～2 小时**，包含理解、一个小实现或现有代码阅读、实验、记录。
超出两小时就继续拆分，不以完成完整算法或训练收敛作为一个小任务。
下面是**学习完成度**：只有本人做过实验、记录结果并能解释时才勾选。
上面的工程进度另行保留；尚未确认掌握的任务不预先勾选。

### Stage 0 — Environment

- [x] S0.1（0.5h）：亲自核对解释器和依赖，运行环境检查，记录路径与版本。
- [x] S0.2（0.5～1h）：运行现有 headless 示例，记录命令、退出码与仿真时间。
- [x] S0.3（0.5～2h）：单独确认 GUI 画面、视角操作和退出状态；失败则记录可复现问题。
- [x] S0.4（0.5h）：熟悉目录和 Git diff，解释代码、笔记与模型缓存的存放位置。
  本人于 2026-09-30 明确确认 S0.1～S0.4 均已完成；Stage 0 完成。

### Stage 1 — MuJoCo Basics

- [x] Task 1（1～2h）：运行 [最小 UR5e pipeline](examples/02_ur5e_basics/simulation_pipeline.py)，观察默认 ctrl 下的 time/qpos/qvel，并亲自解释 MjModel、MjData 与 mj_step。
  本人观察、核心概念问答以及 GUI 验证均已完成，见[实验笔记](docs/01_mujoco_basics.md)。
- [x] S1.1（0.5～1h）：逐行阅读单铰链 XML，指出 body、joint、geom 和重力运动关系。2026-09-27 本人提供运行结果，并用 qpos 变化解释铰链转动。
- [x] S1.2（1h）：解释 MjModel/MjData、qpos/qvel/time；先预测再对比 500 与 1,000 步结果。2026-09-27 本人提供结果并正确解释。
- [x] S1.3（1h）：在 UR5e 输出中对应 nq/nv/nu、关节名称和状态数组的单位。
  2026-09-27 本人正确读取 elbow_joint 的索引、角度与角速度，并解释位置正负及运动方向。
- [x] S1.4（1～2h）：整理 UR5e body/joint/geom/site/actuator 的区别，列出关节与执行器映射。2026-09-27 本人完成元素问答并正确列出六组映射。
- [x] S1.5（1h）：用一个状态修改对比 mj_forward 与 mj_step，记录是否推进时间。
  2026-09-27 本人正确预测并确认缓存与时间变化，理解直接赋值姿态不是物理运动；[实验代码](examples/02_ur5e_basics/forward_vs_step.py) headless 已验证。

### Stage 2 — Joint Control

- [x] S2.1（1h）：阅读一个 UR5e 执行器定义，确认 ctrl 含义、gear 和有效范围。
  2026-09-27 本人正确判断单位、赋值与运动的区别、输入范围，并完成 gear=2 的目标换算，见[笔记](docs/03_joint_control.md)；未实现控制器。
- [x] S2.2（1h）：理解 home 初始化，只改变一个目标 0.05 rad，记录目标与实际角度。
  本人已提供 2 秒运行结果并完成解释：目标 -1.5208 rad、实际角度约 -1.52080194 rad，理解其他目标不变不保证其他关节不动，见[笔记](docs/03_joint_control.md)。
- [x] S2.3（1～2h）：采样一个关节的时间、目标和角度，画一张响应图。
  助手生成并验证 1,001 点 PNG/CSV；本人完成采样、读图与角度比较练习，见[笔记](docs/03_joint_control.md)。
- [x] S2.4（1h）：对比两个小目标变化，说明误差、限制及其他关节是否运动。
  助手完成 +0.02/+0.05 rad 实验；本人正确比较误差、辨认其他关节运动并理解范围内不保证零误差，见[笔记](docs/03_joint_control.md)。

### Stage 3 — PD Control

- [x] S3.1（1h）：说明位置/速度误差、力矩和单位；手算一次单关节 PD 输出。
  2026-09-28 本人完成误差与力矩手算、D 项方向练习，见[PD 笔记](docs/04_pd_control.md)。
- [x] S3.2（1～2h）：在最小单关节力矩模型中手写 PD，区分它与内置位置伺服。
  2026-09-28 本人填写[单关节 PD](examples/03_pd_control/README.md)并运行，正确解释制动与力矩输入区别；助手 headless 复跑通过。
- [x] S3.3（1h）：固定 Kd，仅对比两个 Kp，记录上升过程和超调。
  2026-09-28 本人完成首步预测、读图与速度/超调取舍解释；两组采样由助手运行，见[PD 笔记](docs/04_pd_control.md)。
- [x] S3.4（1h）：固定 Kp，仅对比两个 Kd，记录振荡与稳态误差。
  2026-09-28 本人完成预测、振荡与制动解释，理解到达目标不等于稳定；2 s、4 s 对比由助手运行，见[PD 笔记](docs/04_pd_control.md)。

### Stage 4 — Robot Kinematics

- [x] S4.1（1h）：画出世界/关节/末端坐标系，手算一个简单刚体变换。
  2026-09-28 本人理解局部/世界运动之别，正确完成 +90° 后杆中点的旋转与平移手算；坐标系示意由助手提供，见[FK 笔记](docs/05_forward_kinematics.md)。
- [x] S4.2（1～2h）：为平面两连杆手写 FK，用两个姿态核对末端位置。
  本人完成姿态手算、公式与 Python 两行核心实现；助手集成[最小示例](examples/04_forward_kinematics/main.py)，两个姿态数值核对通过。
- [x] S4.3（1h）：读取 UR5e 末端 site 位置和朝向，说明参考坐标系。
  本人完成局部/世界位姿与时间预测，正确读取 home 时 site +y/+z 轴的世界方向；读取实验由助手运行。
- [x] S4.4（1～2h）：读取位置 Jacobian，解释行列和单位，用一个关节有限差分核对一列。
  本人完成手算、方向和单位解释；助手单列差分核对通过，并补充几何叉乘来源，见[Jacobian 笔记](docs/06_jacobian.md)。
- [x] S4.5（1h）：比较两个姿态的 Jacobian，记录接近奇异时的数值现象。
  本人完成位移预测，理解某个方向响应变弱，并正确区分接近奇异与奇异；数值比较由助手运行，见[Jacobian 笔记](docs/06_jacobian.md)。

### Stage 5 — Inverse Kinematics

- [x] S5.1（1h）：定义位置误差、步长和停止条件，手算一次简化 IK 更新。
  本人半步位置预测正确，并判断约 1.5 mm 残差尚未满足 0.1 mm 成功容差，见[IK 笔记](docs/07_inverse_kinematics.md)。
- [x] S5.2（1～2h）：手写一个小位置目标的单次 Jacobian 更新，检查误差是否减小。
  本人写对三行更新表达式，正确判断误差减小但未成功；经讲解后确认完整增量也须重算 FK。助手运行[单步示例](examples/06_inverse_kinematics/main.py)，误差从 3 mm 降至约 1.5 mm。
- [x] S5.3（1～2h）：加入有上限的迭代循环和关节限制，记录误差随迭代变化。
  本人正确填写[迭代示例](examples/06_inverse_kinematics/iteration.py)的成功、限位和接受更新表达式，并正确解释 5 次更新/6 条记录、容差停止及限位分支未被本次运行验证，见[IK 笔记](docs/07_inverse_kinematics.md)。
- [x] S5.4（1h）：测试一个不可达目标，记录失败条件，不追求通用求解器。
  本人正确预测 `(0.8,0)` m 目标至少有 0.1 m 残差，并正确区分次数上限终止与几何不可达证明；实验在第 20 次以约 0.166 m 残差失败。Stage 5 完成。

### Stage 6 — Cartesian Control

- [x] S6.1（1h）：区分关节目标与末端目标，明确坐标系、输入输出及单位。
  本人正确说明 UR5e `ctrl` 是 rad 为单位的关节位置目标，世界系末端误差单位为 m，Jacobian 映射输出为 rad 的关节角增量，见[Cartesian Control 笔记](docs/08_cartesian_control.md)。
- [x] S6.2（1～2h）：将一个很小的末端位置误差转换为受限关节增量，先单步验证。
  本人经纠正后确认比例 0.4、受限增量 `(0,0.004)` rad 和线性位移 `(-0.0012,0)` m；[单步实验](examples/07_cartesian_control/main.py)误差降至约 1.8 mm，并正确解释限制与圆弧造成的线性化差异。
- [x] S6.3（1～2h）：重复该更新跟踪一个固定目标，记录误差和速度限制效果。
  本人确认周期减半时每轮增量限制也减半；[重复更新实验](examples/07_cartesian_control/tracking.py)在 3 次更新、假定 0.06 s 后达到容差，并正确解释限幅饱和及几何命令速度与实际 `qvel` 的区别。Stage 6 完成。

### Stage 7 — Manipulation

- [x] S7.1（1h）：在最小接触场景中识别碰撞几何与接触，记录一次接触信息。
  本人正确预测并解释初态无接触、球心约 0.05 m 时接触、`ncon` 随状态变化、临时接触不改变模型拓扑，以及离散软接触造成的少量穿入。
- [x] S7.2（1～2h）：检查一个夹爪模型的关节与命令，独立测试开合。
  本人经纠正确认开口变化来自两指位移之和、执行器—关节传动映射只规定目标来源，并理解空载开合不能证明抓取成功；[独立实验](examples/08_contact_and_grasping/gripper.py)开口从 0.04 m 增至 0.08 m。
- [x] S7.3（1～2h）：定义并验证一个固定目标 Reach 的成功判据。
  本人正确手算并验证世界系位置距离与严格 1 cm 边界，且正确区分 Reach、抓取成功和超时未成功；见[判据脚本](examples/08_contact_and_grasping/reach_criterion.py)。
- [x] S7.4（1～2h）：从预设对齐姿态做一次闭爪接触实验，记录成功/失败现象。
  本人正确解释双侧接触、软接触穿入、接触阻挡导致 `qpos != ctrl`，并理解双侧接触尚不能证明稳定抓取；见[闭爪实验](examples/08_contact_and_grasping/close_contact.py)。
- [x] S7.5（1～2h）：在已成功抓住的初始条件下测试一次小幅抬升。
  本人正确解释物体高度与相对滑移判据、有限刚度位置伺服的重力稳态下垂及负相对 z；[实验](examples/08_contact_and_grasping/lift.py)使物体升约 0.04062 m并满足判据。
- [x] S7.6（1～2h）：在已持物的初始条件下测试放置和释放，写出后续组合任务清单。
  本人正确解释地面支撑、无手指接触、目标位置与低速度各自排除的失败；[放置释放实验](examples/08_contact_and_grasping/place_release.py)四项检查通过，完整组合流程已写入 Manipulation 笔记。Stage 7 完成。

### Stage 8 — Robot Learning

- [x] S8.1（1h）：为已有 Reach 实验写出 observation/action/reward 和结束条件。
  本人正确写出 8 维 observation 各段单位、距离奖励，以及成功与时间截断的纸面结果；尚未实现接口或安装 RL 依赖。
- [x] S8.2（1～2h）：明确请求依赖后，仅实现 reset 和 observation 的最小接口。
  本人正确完成 observation 拼接并解释 observation/info 与 FK 派生量；reset 数值、shape、dtype、space 包含关系和状态恢复检查通过。
- [x] S8.3（1～2h）：加入 step，区分 terminated/truncated，用短随机动作回合检查接口。
  本人正确实现动作限幅、几何速度积分、距离奖励和两种结束条件，并解释随机策略超时不代表接口失败，以及命令速度不同于 MuJoCo 动力学的实际 `qvel`；环境与边界检查通过。
- [x] S8.4（1h）：对比两种距离奖励的数值，检查奖励是否符合任务目标。
  本人正确计算线性与平方距离奖励及改善量，并理解平方形式会放大远近的比例差异，但在小于 1 m 时数值绝对值更小；最小数值检查通过。
- [x] S8.5（1～2h）：解释 PPO 的采样和更新数据流，手算一个小样本目标；留下实现 TODO。
  本人正确计算单步 critic target、advantage，以及正负 advantage 下的 PPO clipped objective，并解释裁剪用于限制单批旧数据推动的策略变化；训练实现仍保留 TODO。
- [x] S8.6（1～2h）：解释 SAC 的 replay/critic/entropy，手算一个简化目标；留下实现 TODO。
  本人正确计算含 entropy 的双 critic target，并解释 replay 的 off-policy 数据复用、较小 critic 值对高估的约束，以及 `alpha` 对策略随机性的影响；训练实现仍保留 TODO。
- [x] S8.7（1～2h）：后续单独授权后，选一种算法做 CPU 短运行检查，记录数据和耗时，不要求收敛。
  PPO CPU 烟雾测试完成；本人正确区分流程连通与策略收敛，解释了短训练的证据边界，并由动作速度、控制周期和回合长度推出每关节最多变化 0.4 rad，确认当前回合预算内目标不可达。Stage 8 完成。

### Stage 9 — Domain Randomization / Sim-to-Real

- [x] S9.1（1h）：只随机化一个物理参数，固定 seed，核对采样范围及复现结果。
  第二连杆长度在 `[0.27,0.33]` m 内按回合采样；本人正确解释同 seed 序列复现、序列内部变化，以及只在首轮传入 seed 的 RNG 语义，范围与复现检查通过。
- [x] S9.2（1～2h）：用已有控制方法比较少量固定参数与随机参数回合，记录同一指标。
  标称 Jacobian 闭环在固定长度下 3 次更新成功，五个随机长度回合均成功但需 7～17 次；本人正确解释反馈纠偏、模型失配降低收敛效率，以及少量几何实验的 sim-to-real 证据边界。
- [x] S9.3（1h）：只加入一种观测噪声或延迟，对比一次基线实验。
  末端位置观测加入每轴 1 mm 标准差的高斯噪声后，最终真实距离约 0.570 mm、末 5 条 RMS 约 0.929 mm；本人正确解释瞬时误差、持续波动、零均值噪声及单 seed 证据边界。
- [x] S9.4（1h）：根据实验列出模型误差、执行器限制和真实部署前待验证事项，不连接真实机器人。
  本人基于已有实验整理了几何/动力学/接触/传感器模型误差、真实执行器限制及部署检查清单，并正确说明仿真成功不是现实安全许可。Stage 9 完成。

### Stage 10 — Full UR5e 6D Motion

Stage 10 起分开记录状态：Engineering 由已验证的代码/实验/docs 决定；Learning 只有本人
明确完成 Run、Modify、Explain 后才算 Mastered。旧流程下的助手运行不追认成本人的 Run。

- S10.1（0.5～1h）：读取并比较 UR5e `attachment_site` 的完整世界位姿。见 [6D Pose 笔记](docs/11_ur5e_6d_pose.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人明确确认 P0 已完成；非助手运行追认）
- S10.2（1～2h）：由相对旋转、轴角和旋转向量建立 world-frame orientation error。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人明确确认 P0 已完成；非助手运行追认）
- S10.3（1～2h）：读取 UR5e 6×6 site Jacobian，用单关节有限差分同时核对位置与朝向。见 [6D IK 笔记](docs/12_ur5e_6d_ik.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行默认实验、完成 wrist_2 对比并确认单位修正）
- S10.4（1～2h）：为很小的目标位姿完成一次未阻尼 6D IK 更新，分别比较 position 与 orientation error。见 [6D IK 笔记](docs/12_ur5e_6d_ik.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行默认与 5 倍目标实验并解释单步残差和奇异性风险）
- S10.5（1～2h）：在近奇异 UR5e 姿态比较普通最小二乘与两档 DLS，记录关节增量、残差、奇异值和稳定性。见 [6D IK 笔记](docs/12_ur5e_6d_ik.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成近奇异/远离奇异对比并解释 SVD 增益与阻尼折衷）
- S10.6（1～2h）：把 DLS 扩展为有次数、双容差、单步和关节位置限制的迭代 6D IK，测试可达、困难、不可达和无效目标。见 [6D IK 笔记](docs/12_ur5e_6d_ik.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成四场景运行、步长对比及终止状态解释）
- S10.7（0.5～1h）：由 `qdot_max * control_dt` 推导并检查每轮关节增量限制，区分几何命令速度与 MuJoCo `data.qvel`。见 [6D IK 笔记](docs/12_ur5e_6d_ik.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成 control-dt 对比并解释命令速度、实际 qvel 与安全边界）
- S10.8a（1～2h）：生成并绘制 joint-space 线性插值的 `q(t)`、分段速度和端点加速度尖峰。见 [Trajectory 笔记](docs/13_trajectory.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成 2/4 s 对比并解释连续性、端点冲击与 tracking 边界）
- S10.8b（1～2h）：生成并绘制满足端点零速度边界条件的 cubic trajectory，比较线性轨迹的 position、velocity 和 acceleration。见 [Trajectory 笔记](docs/13_trajectory.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs
  - Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成 2/4 s 对比并解释约束、时间缩放、连续性与 tracking 边界）

### Stage 11 — UR5e Known-Pose Pick & Place

- [x] S11.1（1～2h）：把简化双指夹爪接到 UR5e 末端，核对 attachment frame、关节/执行器映射和碰撞几何；只验证空载开合。见 [Pick & Place 笔记](docs/14_pick_place.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行默认与 0.015 m 目标实验，两者均 PASS，并解释 pose、opening、servo target 与碰撞证据边界）
- [x] S11.2（1h）：为已知位姿的简单方块手工定义 grasp 与 pre-grasp pose，解释 offset、world/base/end-effector frame 和朝向要求；不执行抓取。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证默认/修改实验，解释合法旋转、frame 表达与 IK 证据边界）
- [x] S11.3（1～2h）：复用 Stage 10 的 IK 与轨迹，从 home 到 pre-grasp，并检查位置/朝向容差、关节限制和碰撞。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证 3/5 s 实验，解释路径检查、时间缩放、离散碰撞与 arm-only Jacobian）
- [x] S11.4（1～2h）：沿末端局部轴从 pre-grasp 接近并闭爪，比较 world-frame 与 end-effector-frame 位移，只以双侧接触作为本步现象。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证 0.10/0.08 m 接近实验，解释 frame、IK 容差、命名 contact pair 与固定物体证据边界）
- [x] S11.5（1～2h）：小幅抬升，分别检查物体世界高度、相对滑移和接触保持；不把瞬时接触自动视为稳定抓取。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证 0.05/0.03 m 抬升，解释 world height、relative slip、接触保持与 headroom 证据边界）
- [x] S11.6a（1～2h）：持物限速转移并下降到已知放置位姿，监测掉落、位姿误差和物体—支撑面接触；尚不释放。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证 3/2 s transfer，解释时间累积滑移、contact/slip 区别、支撑与释放边界及 measured offset）
- [x] S11.6b（1～2h）：确认支撑后释放并撤离，检查最终物体位置、支撑接触、手指接触消失和低末速度，完成一次完整 known-pose pick-and-place。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人运行并验证 0.08/0.12 m retreat，解释 support-before-release、command/state、pose/velocity 与 load-transfer 证据）
- [x] S11.7（1～2h）：对物体 xy、摩擦和质量的小范围固定-seed 变化运行 20 次，统计成功率、失败阶段与最终位姿误差；不扩展为复杂 domain randomization。见 [Pick & Place 笔记](docs/14_pick_place.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人完成两组 fixed-seed trials，解释 reproducibility/coverage、empirical rate、known-pose 边界及 failure/error 统计口径）

## P1 — Perception, Planning & Robot Software Integration

CPU-first；无 Isaac/大型视觉模型；ICP 与规划先手写；ROS2 最后接入。
细化依据、复用边界与资源预算见[P1 规划](docs/p1_plan.md)。每个 Task 0.5～2h，当前已实现 S12.1–S12.8b；S12.1–S12.8b Engineering Complete + Learning Mastered。

### Stage 12 — Robot Perception Geometry

[阶段笔记](docs/15_robot_perception_geometry.md)。

- S12.1（0.5～2h）：Camera Frames / Coordinate Transform：optical/object/base/world；完整 pose 链、inverse、点与方向核验。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人明确确认移动相机实验完成；Run/Explain 已先行完成）

- S12.2（0.5～2h）：Pinhole / intrinsic：K、像素单位、3D→2D、可见深度和轴 convention；手写投影。见 [S12.2 Learning Package](docs/15_2_pinhole_projection.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人明确确认实验与预测完成，正确解释透视投影、光轴深度、内外参、偏移缩放、深度歧义与遮挡边界）

- S12.3（0.5～2h）：MuJoCo CPU RGB/depth acquisition：renderer→optical frame、图像/depth 语义、尺寸和耗时核验。见 [S12.3 Learning Package](docs/15_3_rgb_depth_acquisition.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人确认实验与对比分析完成，正确解释合法旋转、axial depth、可见表面、内参不变与渲染 API/时间边界）

- S12.4（0.5～2h）：Camera calibration：OpenCV 合成多视角标定板角点；估计 K/distortion，报告 known-pose held-out reprojection error 与尺度。见 [S12.4 Learning Package](docs/15_4_camera_calibration.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人确认实验与对比完成，正确解释标定输入/输出、多视角约束、参数单位、固定 k3、训练误差边界、metric scale 与 board→camera 方向）

- S12.5（0.5～2h）：PnP：已知非共面 3D↔2D 对应，估计 T_CO，比较 truth pose 与 reprojection error；含像素噪声与错误焦距对照。见 [S12.5 Learning Package](docs/15_5_pnp_pose.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-04 本人确认实验与验证均完成，正确解释已知量/未知位姿、多点几何约束、tvec 与 camera center、内参/位姿补偿及合法性检查边界；尺度来自已知 metric geometry）

- S12.6（0.5～2h）：RGB-D back projection：axial depth、K inverse、有效像素→camera/base point cloud。见 [S12.6 Learning Package](docs/15_6_rgbd_back_projection.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测均完成，正确解释 axial depth/range、索引约定、frame 链、相机移动与 roundtrip 的验证边界）

- S12.7a（0.5～2h）：ICP rigid alignment：已知对应的小点集，手写 NumPy centroid/SVD，纠正 reflection。见 [S12.7a Learning Package](docs/15_7a_rigid_alignment.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测均完成，Explain 覆盖 centroid、SVD、reflection、几何可观测性及 residual 与 pose error 的区别）

- S12.7b（0.5～2h）：ICP nearest-neighbor loop：小点云 CPU 最邻近、门限、停止条件；测试初值与 partial overlap。见 [S12.7b Learning Package](docs/15_7b_icp_loop.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测均完成，Explain 覆盖对应更新、增量组合、两种 RMS、局部停止与 gate 取舍）

- S12.8a（0.5～2h）：Hand-eye geometry：eye-in-hand / eye-to-hand，构造 AX=XB，多姿态可观测性。见 [S12.8a Learning Package](docs/15_8a_hand_eye_geometry.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测完成，补充正确的两种安装 X/Y 方向与固定关系；Explain 覆盖相对运动推导、退化与条件性；未求解 X）

- S12.8b（0.5～2h）：Hand-eye calibration：CPU 合成多姿态求解、held-out transform residual 和噪声对比。见 [S12.8b Learning Package](docs/15_8b_hand_eye_calibration.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测完成，并正确补充齐次符号、平移分块式与 clean-held 的 Y_mean 偏差；Explain 完成）



### Stage 13 — Vision-Based Manipulation

[阶段笔记](docs/16_vision_based_manipulation.md)。

- S13.1（0.5～2h）：Perception pose→base→world：frame/quality 接口，truth 与 estimate 分离，拒绝无效估计。见 [S13.1 Learning Package](docs/16_vision_based_manipulation.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测完成，Explain 覆盖 frame 链、原点/朝向、接口检查边界、RMS/accuracy、truth 隔离与 object/gripper 区别）

- S13.2（0.5～2h）：Grasp pose generation：对象尺寸/估计朝向→top-down candidates、pre-grasp，复用 IK 作可达筛选。见 [S13.2 Learning Package](docs/16_2_grasp_pose_generation.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-05 本人确认实验与预测完成，并正确解释 frame/tool offset/approach/local IK、开口与单根 slide 关系及不同朝向的夹持尺寸）

- S13.3a（0.5～2h）：Vision-to-motion 接入：CPU 合成图像 PnP estimate 驱动 home→pre-grasp→approach 动力学，不泄漏 truth。见 [S13.3a Learning Package](docs/16_3a_vision_to_motion.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，Explain 覆盖 truth 隔离、frame 链、upright prior、动力学/偏置补偿与 tracking/truth error；无 GUI/闭爪/抓取验证）

- S13.3b（0.5～2h）：Vision-based pick-and-place：连接已有 close/lift/transfer/release，分别检查估计与执行结果。见 [S13.3b Learning Package](docs/16_3b_vision_pick_place.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，并补正20 mm基准间隙、30/48 mm开口与close失败判据；无 GUI/真实视觉/硬件验证）

- S13.4（0.5～2h）：Perception noise：分别扰动 pose 与 extrinsic，比较误差传播及失败类型。见 [S13.4 Learning Package](docs/16_4_perception_noise.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，Explain覆盖frame方向、rotation pivot、lever arm、fit/diagnostic RMS及非线性失败阶段；无GUI/随机trials/真实视觉验证）

- S13.5（0.5～2h）：Repeated-trial evaluation：fixed seeds、小范围视角/物体变化，成功率、pose error、失败阶段与耗时。见 [S13.5 Learning Package](docs/16_5_repeated_trials.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，Explain覆盖end-to-end分母、成功条件偏差、seed/paired noise与wall time、小样本范围及失败证据分类；无GUI/真实硬件验证）



### Stage 14 — Motion Planning

[阶段笔记](docs/17_motion_planning.md)。

- S14.1（0.5～2h）：Collision checking：独立 MjData、self/environment/held-object 检查；按阶段定义允许接触。见 [S14.1 Learning Package](docs/17_motion_planning.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖状态隔离、两次forward、signed distance、持物frame链与配置/路径边界）

- S14.2（0.5～2h）：Configuration space / edge checking：q limits、距离、步长分辨率；二维障碍最小实验。见 [S14.2 Learning Package](docs/17_2_configuration_space.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖C-space、障碍膨胀、端点采样、分辨率边界与path/trajectory区别）

- S14.3（0.5～2h）：手写 RRT：sample/nearest/steer/edge check/parent，fixed seed 与 node/attempt/time budget。见 [S14.3 Learning Package](docs/17_3_rrt.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖搜索流程、两种步长、parent与末段检查、预算和路径边界）

- S14.4（0.5～2h）：手写 RRT-Connect：双树 extend/connect、path reconstruction，比较成功率与扩展数。见 [S14.4 Learning Package](docs/17_4_rrt_connect.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖EXTEND/CONNECT、逐步预算、双树方向拼接、工作量计数与seed比较边界）

- S14.5（0.5～2h）：Path smoothing：collision-checked shortcut，比较长度并重检所有边。见 [S14.5 Learning Package](docs/17_5_path_smoothing.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖三角不等式与碰撞、shortcut整边检查、端点保留、最终复查及最优/clearance/连续性边界）

- S14.6（0.5～2h）：Time parameterization：复用 cubic，按每段 velocity/acceleration limit 配时；分段停点与连续性。见 [S14.6 Learning Package](docs/17_6_time_parameterization.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖解析双约束、共享时间律、C1/C2、峰值与舍入、参考执行边界）

- S14.7a（0.5～2h）：UR5e obstacle planning：IK endpoint→joint RRT-Connect→smoothed timed path，几何与tracking分别检查。见 [S14.7a Learning Package](docs/17_7a_ur5e_obstacle_planning.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（2026-10-06 本人确认实验与预测完成，五项Explain覆盖IK/reference/actual边界、frame与局部搜索、采样限制、动态执行与未验证能力）

- S14.7b（0.5～2h）：Collision-aware pick-and-place：带物 transform、阶段接触策略、执行过程与重复试验统计。见 [S14.7b Learning Package](docs/17_7b_collision_aware_pick_place.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[x] Run　[x] Modify　[x] Explain — Mastered（本人确认实验与预测完成，Explain覆盖持物frame链、phase接触、名义与actual、支撑事件、失败停止与统计边界）



### Stage 15 — ROS2 Integration

[阶段笔记](docs/18_ros2_integration.md)。

- S15.1（0.5～2h）：环境与 node/topic：核验 WSL2 CPU ROS2、conda/系统 Python 边界，最小消息收发。见 [S15.1 Learning Package](docs/18_ros2_integration.md)。
  - Engineering：[x] Code　[x] Experiment　[x] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain（2026-10-07 系统 Python/Jazzy 真实导入、默认与0.1s双进程收发、graph CLI、无对端超时/非法参数检查通过；Engineering Complete，学习待本人验证）

- S15.2（0.5～2h）：Service：pose/planning 短请求与显式 failure response，避免执行长动作阻塞服务。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain

- S15.3（0.5～2h）：Action：trajectory execution goal/feedback/result/cancel，失败与取消语义。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain

- S15.4（0.5～2h）：TF2：world/base/camera/object/tool tree、时间戳、静态外参与过期变换拒绝。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain

- S15.5（0.5～2h）：URDF / joint states：UR5e 结构、名称/轴/单位与 MuJoCo frame 对照。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain

- S15.6a（0.5～2h）：ROS2 adapters：perception node→planning service→execution action，算法继续独立可运行。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain

- S15.6b（0.5～2h）：Final ROS2 manipulation pipeline：CPU simulation end-to-end、取消/超时/坏 pose 注入与诊断。
  - Engineering：[ ] Code　[ ] Experiment　[ ] Docs；Learning：[ ] Run　[ ] Modify　[ ] Explain



## Learning Notes and Workflow

新会话先读 [AGENTS.md](AGENTS.md) → [交接文档](docs/project_handoff.md) → 相关示例。
编号笔记按主题记录，实验后逐步填写；不预写本人的结论或面试题答案。
[仓库评估与笔记索引](docs/learning_roadmap.md)说明哪些代码适合先读。

Stage 10 起采用 Sprint Learning Mode：Codex 一次完成当前 Task 的 Engineering package，
然后本人按指定内容完成 Run → Modify → Explain；三项均明确完成后才标记 Learning Mastered。
纯文档维护不虚构实验；Stage 1 Task 1 已由本人确认完成。
