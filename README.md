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

## Current Progress

| 已有内容 | 工程状态 | 学习状态 |
| --- | --- | --- |
| 环境检查与仓库基础 | 已建立，历史验证通过；用户已确认 GUI 可见 | 交互操作与学习掌握程度待确认 |
| 单铰链仿真 | 默认 1,000 步，本人运行到 2 秒 | 已完成模型阅读、预测与状态解释 |
| 官方 UR5e 检查演示 | 加载、打印状态、单目标微调已验证 | 待逐项理解状态和执行器映射 |
| 自定义关节/PD 控制及后续阶段 | 未实现 | 未开始 |

当前已开始 **Stage 1 — Task 1 simulation pipeline**，代码已实现，核心概念问答已通过；已有代码触及 Stage 2 的目标设置，
但不代表已经完成关节控制学习。2026-09-26 用户确认 GUI 显示问题已解决，画面可见。
历史退出错误和显示问题仍保留作参考；具体交互与本次退出状态未单独确认。详见
[交接文档](docs/project_handoff.md)。新最小示例 headless 已通过；本次 GUI 在最终输出后退出码为 139。

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
# 仅首次安装或依赖确实缺失时执行：
python -m pip install -r requirements.txt
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
python examples/02_ur5e_basics/main.py --headless
# GUI 已确认可见；可用长时运行练习交互：
python examples/02_ur5e_basics/main.py --viewer --steps 150000
```

依赖只有 `mujoco`、`numpy`、`matplotlib`、`mujoco-menagerie`。Menagerie 首次使用
下载 UR5e 到用户缓存，不复制整个模型仓库。不要在 base 或系统 Python 安装依赖。
当前 passive viewer 没有暂停回调，空格不会暂停 Python 仿真循环。

## Repository Structure

```text
examples/
  01_basic_simulation/       # main.py + simple_model.xml：被动铰链
  02_ur5e_basics/            # main.py：模型、状态、执行器检查
  02_joint_control/         # 以下示例仅有 README
  03_pd_control/
  04_forward_kinematics/
  05_jacobian/
  06_inverse_kinematics/
  07_cartesian_control/
  08_contact_and_grasping/
controllers/                # README + 空包 __init__.py
environments/              # reach / pick / pick_place 文档占位
rl/                         # gymnasium / ppo / sac 文档占位
assets/                     # 共享资源说明，暂无模型资产
scripts/check_env.sh         # 环境诊断，不自动激活 conda
notebooks/                  # 说明文档，暂无 notebook
docs/                       # 学习笔记骨架、路线、交接与历史记录
tests/                      # 验证说明，暂无自动化测试套件
AGENTS.md                   # 学习与开发规则
requirements.txt            # 最小依赖
```

两个 `02_` 目录保留；目录编号不等于 Stage 编号。阅读顺序与笔记映射见
[学习路线索引](docs/learning_roadmap.md)。

## Learning Roadmap

每个子任务预计 **0.5～2 小时**，包含理解、一个小实现或现有代码阅读、实验、记录。
超出两小时就继续拆分，不以完成完整算法或训练收敛作为一个小任务。
下面是**学习完成度**：只有本人做过实验、记录结果并能解释时才勾选。
上面的工程进度另行保留；尚未确认掌握的任务不预先勾选。

### Stage 0 — Environment

- [ ] S0.1（0.5h）：亲自核对解释器和依赖，运行环境检查，记录路径与版本。
- [ ] S0.2（0.5～1h）：运行现有 headless 示例，记录命令、退出码与仿真时间。
- [ ] S0.3（0.5～2h）：单独确认 GUI 画面、视角操作和退出状态；失败则记录可复现问题。
  画面可见部分已由用户于 2026-09-26 确认；操作与退出记录尚待补充，整个任务暂不勾选。
- [ ] S0.4（0.5h）：熟悉目录和 Git diff，解释代码、笔记与模型缓存的存放位置。

### Stage 1 — MuJoCo Basics

- [ ] Task 1（1～2h，进行中）：运行 [最小 UR5e pipeline](examples/02_ur5e_basics/simulation_pipeline.py)，观察默认 ctrl 下的 time/qpos/qvel，并亲自解释 MjModel、MjData 与 mj_step。
  本人观察与核心概念问答已完成，已写入[实验笔记](docs/01_mujoco_basics.md)；headless 通过，GUI 正常退出仍待确认，整个任务暂不勾选。
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
- [ ] S9.3（1h）：只加入一种观测噪声或延迟，对比一次基线实验。
- [ ] S9.4（1h）：根据实验列出模型误差、执行器限制和真实部署前待验证事项，不连接真实机器人。

## Learning Notes and Workflow

新会话先读 [AGENTS.md](AGENTS.md) → [交接文档](docs/project_handoff.md) → 相关示例。
十份编号笔记按主题保留空白，实验后逐步填写；不预写答案。
[仓库评估与笔记索引](docs/learning_roadmap.md)说明哪些代码适合先读。

每个学习 Task：先写预测和 API 输入/输出，再完成最小实现或阅读实验，最后记录实测
结果、问题和 3～5 个面试问题。本人能够解释后才更新 checkbox。
纯文档维护不虚构实验；当前 Stage 1 Task 1 核心概念问答已通过，GUI 正常退出验证仍未通过。
