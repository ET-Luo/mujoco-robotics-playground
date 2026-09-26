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
| 单铰链仿真 | 默认 1,000 步，历史运行到 2 秒 | 待本人解释模型与状态 |
| 官方 UR5e 检查演示 | 加载、打印状态、单目标微调已验证 | 待逐项理解状态和执行器映射 |
| 自定义关节/PD 控制及后续阶段 | 未实现 | 未开始 |

当前处于 **Stage 0 收尾与 Stage 1 学习准备**；已有代码触及 Stage 2 的目标设置，
但不代表已经完成关节控制学习。2026-09-26 用户确认 GUI 显示问题已解决，画面可见。
历史退出错误和显示问题仍保留作参考；具体交互与本次退出状态未单独确认。详见
[交接文档](docs/project_handoff.md)。本次框架整理没有重新运行仿真。

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

- [ ] S1.1（0.5～1h）：逐行阅读单铰链 XML，指出 body、joint、geom 和重力运动关系。
- [ ] S1.2（1h）：解释 MjModel/MjData、qpos/qvel/time；先预测再对比 500 与 1,000 步结果。
- [ ] S1.3（1h）：在 UR5e 输出中对应 nq/nv/nu、关节名称和状态数组的单位。
- [ ] S1.4（1～2h）：整理 UR5e body/joint/geom/site/actuator 的区别，列出关节与执行器映射。
- [ ] S1.5（1h）：用一个状态修改对比 mj_forward 与 mj_step，记录是否推进时间。

### Stage 2 — Joint Control

- [ ] S2.1（1h）：阅读一个 UR5e 执行器定义，确认 ctrl 含义、gear 和有效范围。
- [ ] S2.2（1h）：理解 home 初始化，只改变一个目标 0.05 rad，记录目标与实际角度。
- [ ] S2.3（1～2h）：采样一个关节的时间、目标和角度，画一张响应图。
- [ ] S2.4（1h）：对比两个小目标变化，说明误差、限制及其他关节是否运动。

### Stage 3 — PD Control

- [ ] S3.1（1h）：说明位置/速度误差、力矩和单位；手算一次单关节 PD 输出。
- [ ] S3.2（1～2h）：在最小单关节力矩模型中手写 PD，区分它与内置位置伺服。
- [ ] S3.3（1h）：固定 Kd，仅对比两个 Kp，记录上升过程和超调。
- [ ] S3.4（1h）：固定 Kp，仅对比两个 Kd，记录振荡与稳态误差。

### Stage 4 — Robot Kinematics

- [ ] S4.1（1h）：画出世界/关节/末端坐标系，手算一个简单刚体变换。
- [ ] S4.2（1～2h）：为平面两连杆手写 FK，用两个姿态核对末端位置。
- [ ] S4.3（1h）：读取 UR5e 末端 site 位置和朝向，说明参考坐标系。
- [ ] S4.4（1～2h）：读取位置 Jacobian，解释行列和单位，用一个关节有限差分核对一列。
- [ ] S4.5（1h）：比较两个姿态的 Jacobian，记录接近奇异时的数值现象。

### Stage 5 — Inverse Kinematics

- [ ] S5.1（1h）：定义位置误差、步长和停止条件，手算一次简化 IK 更新。
- [ ] S5.2（1～2h）：手写一个小位置目标的单次 Jacobian 更新，检查误差是否减小。
- [ ] S5.3（1～2h）：加入有上限的迭代循环和关节限制，记录误差随迭代变化。
- [ ] S5.4（1h）：测试一个不可达目标，记录失败条件，不追求通用求解器。

### Stage 6 — Cartesian Control

- [ ] S6.1（1h）：区分关节目标与末端目标，明确坐标系、输入输出及单位。
- [ ] S6.2（1～2h）：将一个很小的末端位置误差转换为受限关节增量，先单步验证。
- [ ] S6.3（1～2h）：重复该更新跟踪一个固定目标，记录误差和速度限制效果。

### Stage 7 — Manipulation

- [ ] S7.1（1h）：在最小接触场景中识别碰撞几何与接触，记录一次接触信息。
- [ ] S7.2（1～2h）：检查一个夹爪模型的关节与命令，独立测试开合。
- [ ] S7.3（1～2h）：定义并验证一个固定目标 Reach 的成功判据。
- [ ] S7.4（1～2h）：从预设对齐姿态做一次闭爪接触实验，记录成功/失败现象。
- [ ] S7.5（1～2h）：在已成功抓住的初始条件下测试一次小幅抬升。
- [ ] S7.6（1～2h）：在已持物的初始条件下测试放置和释放，写出后续组合任务清单。

### Stage 8 — Robot Learning

- [ ] S8.1（1h）：为已有 Reach 实验写出 observation/action/reward 和结束条件。
- [ ] S8.2（1～2h）：明确请求依赖后，仅实现 reset 和 observation 的最小接口。
- [ ] S8.3（1～2h）：加入 step，区分 terminated/truncated，用短随机动作回合检查接口。
- [ ] S8.4（1h）：对比两种距离奖励的数值，检查奖励是否符合任务目标。
- [ ] S8.5（1～2h）：解释 PPO 的采样和更新数据流，手算一个小样本目标；留下实现 TODO。
- [ ] S8.6（1～2h）：解释 SAC 的 replay/critic/entropy，手算一个简化目标；留下实现 TODO。
- [ ] S8.7（1～2h）：后续单独授权后，选一种算法做 CPU 短运行检查，记录数据和耗时，不要求收敛。

### Stage 9 — Domain Randomization / Sim-to-Real

- [ ] S9.1（1h）：只随机化一个物理参数，固定 seed，核对采样范围及复现结果。
- [ ] S9.2（1～2h）：用已有控制方法比较少量固定参数与随机参数回合，记录同一指标。
- [ ] S9.3（1h）：只加入一种观测噪声或延迟，对比一次基线实验。
- [ ] S9.4（1h）：根据实验列出模型误差、执行器限制和真实部署前待验证事项，不连接真实机器人。

## Learning Notes and Workflow

新会话先读 [AGENTS.md](AGENTS.md) → [交接文档](docs/project_handoff.md) → 相关示例。
十份编号笔记按主题保留空白，实验后逐步填写；不预写答案。
[仓库评估与笔记索引](docs/learning_roadmap.md)说明哪些代码适合先读。

每个学习 Task：先写预测和 API 输入/输出，再完成最小实现或阅读实验，最后记录实测
结果、问题和 3～5 个面试问题。本人能够解释后才更新 checkbox。
纯文档维护不勾选学习任务，不虚构实验；本次仅整理框架，未开始 Stage 1。
