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
[实验笔记](01_mujoco_basics.md)。2026-09-27 本人完成 500/1,000 步预测、实测与解释，S1.2 已勾选。
S1.3 已完成：本人正确预测、读取并解释 elbow_joint 的索引和状态，见
[UR5e 笔记](02_ur5e_model.md)。S1.4 的 body/joint/geom 问答已通过，
attachment_site 的局部/世界位置与 actuator 基础问答也已通过；本人正确列出六组
执行器—关节映射，S1.4 已勾选。S1.5 的预测和观察问答均通过，已勾选；
forward_vs_step.py headless 验证通过，GUI 状态不变。
S1.1 也已完成：本人运行被动铰链示例并正确解释 qpos 变化。至此 S1.1～S1.5
学习练习均完成，GUI 正常退出仍独立待办。S2.1 也于 2026-09-27 完成：助手静态核对
执行器参数，本人正确回答单位、输入范围与 gear 目标换算，见
[Joint Control 笔记](03_joint_control.md)；未实现控制器或运行新仿真。
S2.2 的 home 理解与 step 前预测已通过；本人已提供 2 秒单目标实验输出，
本人已纠正“只有对应关节会运动”的判断，结果解释完成，S2.2 已勾选。
助手未重跑；具体数据见 03 笔记。用户已选择 S2.3：正在理解采样时间、目标与角度，
采样点数和目标曲线预测已通过；main.py 的 --plot 已实现并经助手 headless 验证，
PNG/CSV 已生成，本人完成读图、斜率解释与角度比较练习，S2.3 已勾选。
S2.4 目标与范围预测已通过；main.py 新增 --delta 和逐关节变化输出，两组独立 home
实验及输入拒绝检查通过，本人结果解释已完成，S2.4 已勾选。Stage 2 学习练习均完成，
2026-09-28 本人完成 S3.1 的误差、力矩手算与 D 项方向练习，README 已勾选；
S3.2 的[单关节 PD](../examples/03_pd_control/README.md)三行公式已由本人填写并运行，
助手复跑通过，末态误差约 0.000002 rad；本人已解释制动与力矩输入含义，S3.2 已勾选。
S3.3 本人正确预测首步力矩 2/8 N·m；助手已完成 Kp=10/40、Kd=2 的逐步采样与对比。
首次达到 0.48 rad 分别为 0.422/0.094 s，B 组超调约 0.01104 rad；本人正确解释读图和取舍，
S3.3 已勾选。S3.4 本人完成 D 项计算与初始加速度判断，助手完成 Kp=40、Kd=0.5/2
的 2 s 和 4 s 对比：较小 Kd 振荡明显，两组 4 s 末段误差与速度均接近零。
本人已正确解释振荡、制动与经过目标不等于稳定，S3.4 已勾选，Stage 3 学习练习完成。
用户已选择 S4.1：开始单杆 W/J/E 坐标系与 p_W=R_WJ*p_J+t_WJ，
本人已完成绕 +z 转 +90° 的杆中点手算，正确得到旋转位移 (0,0.2,0) m 和
世界位置 (0,0.2,0.5) m，S4.1 已勾选，见[FK 笔记](05_forward_kinematics.md)。
S4.2 已开始：本人正确预测 (0°,0°)/(90°,0°) 时末端为 (0.7,0,0)/(0,0.7,0) m。
本人也正确判断 (90°,90°) 的末端为 (-0.3,0.4,0) m，并通过截图提供正确 x/y 公式。
本人已写出两行 NumPy FK 公式；助手集成最小函数并在 mujoco 环境运行，两个姿态
均与手算一致（绝对容差 1e-12 m），S4.2 已勾选。未做 MuJoCo 对照或 GUI 验证。
用户已选择 S4.3：开始 attachment_site 局部定义、世界 site_xpos/site_xmat 讲解，
本人三项预测已通过；site_pose.py 读取 home 和 shoulder_pan qpos +0.1 rad 的世界位姿，
助手运行退出 0，局部位置/时间不变、世界位置/朝向改变。本人正确解释 site +y/+z
分别朝世界 -y/-z，S4.3 已勾选。用户已选择 S4.4：开始位置 Jacobian 的行列与单位，
本人完成单列手算并纠正 x 方向；助手新增 examples/05_jacobian/main.py 并于
2026-09-29 在 mujoco 环境运行退出 0，单列差分最大误差 2.458e-7 m/rad。
本人正确解释真实 x/y 位移均减小及 m 与 m/rad 的区别，S4.4 已勾选。
FK 偏导与 hinge 几何叉乘来源已补充并数值核对。用户已选择 S4.5：先用平面
两连杆的 2×2 xy Jacobian 理解伸直与弯折的瞬时运动方向，待预测后做近奇异数值比较。
本人位移预测已通过；助手运行 planar_singularity.py，弯折/近伸直条件数约 2.42/4833.33，
完全伸直参考为 inf，矩阵和行列式检查通过；本人理解某个方向响应变弱，
并正确区分接近奇异与奇异，S4.5 已勾选，Stage 4 学习练习完成。
用户已选择 S5.1：沿用两连杆弯折姿态，讲解位置误差、步长比例和停止条件，
本人经符号/小数纠正后正确预测半步 x=0.3985 m；助手 FK 核算残差约 1.5 mm，
本人正确判断尚未满足 0.1 mm 成功容差，S5.1 已勾选。
S5.2 本人已正确实现三行单次更新；助手于 2026-09-29 运行单步示例，误差从 3 mm
降至约 1.5 mm。本人正确判断未成功，经讲解后确认完整增量也须 FK 检查，S5.2 已勾选。
S5.3 本人已正确完成候选角与停止状态判断，并正确填写 iteration.py 三个核心表达式。
助手运行后误差从 0.003 m 降至 0.000093753023 m，第 5 次更新成功；本人正确解释
初态记录、容差停止及未触发分支不能视为已验证，S5.3 已勾选。
S5.4 本人正确预测 target=(0.8,0) m 至少有 0.1 m 残差；助手运行后在第 20 次以
约 0.165764 m 残差报告次数上限失败。本人正确区分直接终止原因与几何不可达证明，
S5.4 已勾选，Stage 5 完成。S6.1 已开始关节目标与世界系末端目标的单位/映射辨析，
尚未实现 Cartesian controller，见[IK 笔记](07_inverse_kinematics.md)和
[Cartesian Control 笔记](08_cartesian_control.md)。
详见 [PD 笔记](04_pd_control.md)。用户已授权自动激活 mujoco，见 AGENTS.md。
GUI 退出异常独立记录，不把 headless 成功当作 GUI 成功，也不以能跑替代本人理解。
