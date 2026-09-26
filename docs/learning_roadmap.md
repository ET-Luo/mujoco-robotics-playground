# 学习路线与仓库评估

评估日期：2026-09-26；基于代码与历史记录静态检查，本次没有运行新实验。
任务清单和学习 checkbox 统一维护在 [README](../README.md#learning-roadmap)，避免两份进度漂移。

## 当前代码如何使用

| 文件或目录 | 当前功能 | 学习用途 |
| --- | --- | --- |
| `examples/01_basic_simulation/main.py` 与 XML | 加载一个铰链、打印状态、推进仿真、可选 GUI | 首选入口，保留显式 API 和短循环 |
| `examples/02_ur5e_basics/main.py` | 官方模型加载、名称/状态打印、home 初始化、单目标变化 | 第二个入口；先读加载与打印，再读执行器检查 |
| `scripts/check_env.sh` | 检查 conda、解释器归属及 MuJoCo 导入 | 保留为环境工具，不作为机器人算法学习主线 |
| `controllers/__init__.py` | 只有包说明 | 不是已实现的控制器，不必提前设计类层次 |
| 后续 examples、environments、rl、assets、notebooks、tests | README 占位 | 不是已完成模块；tests 目前没有自动化套件 |
| 既有 docs | 历史记录、流程、概念占位与交接 | 保留；实验答案逐步写入对应编号笔记 |

从代码本身不能可靠判断生成来源。现有基础框架来自此前助手协作；需要警惕的是
学习顺序，而不是简单把生成代码判为无用：

- UR5e 的 `is_position_servo` 多条件检查同时引入 transmission、gain/bias、gear、
  actuator dynamics。检查有价值，但初学状态读取时认知负担较大；先理解输出，
  到 Stage 2 再逐项讲解。此次保留全部代码，不把检查隐藏进新框架。
- UR5e 内置位置伺服已经产生反馈行为，但修改 ctrl 不等于自己实现了 PD。
- argparse、GUI 计时/同步、异常检查属于支持代码；第一次阅读可先沿 headless 分支。
- 大量未来目录和通用 TODO 只是导航，不以填满目录衡量进步。

当前工程已有基础仿真、UR5e 加载与状态读取，触及一个目标命令实验；个人学习完成度
尚未确认。用户已开始 Stage 1 Task 1；新增 `simulation_pipeline.py` 为最小阅读入口，
旧 `main.py` 保留供后续检查执行器。当前不是已完成 Joint Control。
GUI 画面已由用户于 2026-09-26 确认可见，显示问题不再阻塞学习；交互与本次退出
状态未单独确认。最新证据见 [交接记录](project_handoff.md)。

## Stage 与笔记映射

| Stage | 笔记 |
| --- | --- |
| 0 Environment | [开发流程](development_workflow.md)、[交接记录](project_handoff.md) |
| 1 MuJoCo Basics | [01 MuJoCo](01_mujoco_basics.md)、[02 UR5e](02_ur5e_model.md) |
| 2 Joint Control | [03 Joint Control](03_joint_control.md) |
| 3 PD Control | [04 PD Control](04_pd_control.md) |
| 4 Robot Kinematics | [05 FK](05_forward_kinematics.md)、[06 Jacobian](06_jacobian.md) |
| 5 Inverse Kinematics | [07 IK](07_inverse_kinematics.md) |
| 6 Cartesian Control | [08 Cartesian Control](08_cartesian_control.md) |
| 7 Manipulation | [09 Manipulation](09_manipulation.md) |
| 8 Robot Learning | [10 Robot Learning](10_robot_learning.md) |
| 9 Domain Randomization / Sim-to-Real | 后续实验时在 10 中补充，必要时再拆分 |

编号笔记现在仅有五个标题。由本人实验后填写，不预先生成结论或面试答案。
旧的四个 Phase 已映射为 README 的 Stage 0～9；现有示例目录名称保持不变。

## 下一项建议

**Stage 1 Task 1 核心概念问答已通过**，本人观察与总结已写入
[实验笔记](01_mujoco_basics.md)。下一项建议是先预测再对比 500/1,000 步的时间和关节状态。
GUI 退出异常独立记录，不把 headless 成功当作 GUI 成功，也不以能跑替代本人理解。
