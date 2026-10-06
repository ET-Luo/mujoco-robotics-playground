# Stage 14 — Motion Planning

S14.1：独立状态的单姿态碰撞查询、接触分类、阶段许可和固定持物变换。
[完整学习包](../../docs/17_motion_planning.md) · [状态唯一来源](../../README.md#stage-14--motion-planning)。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/collision_checking.py
python examples/15_motion_planning/collision_checking.py --allowed-depth-m 0.001
```

只需要 requirements 已声明的 NumPy / official MuJoCo，无新增依赖。
4-DOF Cartesian + yaw 教学夹爪与球形碰撞体；不加载 UR5e、不需要前课产物。
13 个固定姿态，输出 ignored `tmp/s14_1_collision_depth*/results.json`。
默认接受6/13；1 mm深度门限接受1/13，退出0代表工程检查通过，绝非全部姿态可行。
先预测 grasp/place 的变化，再运行第二条命令。完整数学、API、结果与 Explain 见学习包。
无动力学/GUI/抓取验证，无连续路径保证；UR5e规划整合留到S14.7。

本人于2026-10-06确认实验与预测完成，并正确回答五项Explain；Learning Mastered，状态见根README。
S14.2随后已按明确请求实现，见下。

## S14.2 — Configuration Space / Edge Checking

[学习包](../../docs/17_2_configuration_space.md) · [代码](edge_checking.py)。
二维slide球体机器人；圆障碍的C-space膨胀、Euclidean配置距离、含端点插值、采样分辨率。

```bash
python examples/15_motion_planning/edge_checking.py
python examples/15_motion_planning/edge_checking.py --step-m 0.02
```

需已声明NumPy/MuJoCo/Matplotlib，无新增依赖。
默认direct端点合法、5点采样误通过；细步长81点拒绝，独立解析评分证明直连穿障碍。
手动绕行三段两组都通过。JSON/PNG在ignored `tmp/s14_2_edges_step*/`。
Modify先预测步长对样本数/判定/路径长度的影响，再对照。
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered；无RRT、时间化、GUI或动态执行。

## S14.3 — 手写 RRT

[学习包](../../docs/17_3_rrt.md) · [代码](rrt.py)。复用S14.2配置/边查询，
显式sample/nearest/steer/edge/parent，goal bias和node/attempt/time预算。

```bash
python examples/15_motion_planning/rrt.py
python examples/15_motion_planning/rrt.py --seed 19
python examples/15_motion_planning/rrt.py --max-nodes 2
```

默认/seed19成功，49/36 nodes、长度2.181/1.861m；max_nodes=2预期exit1，node_budget，无path。
Modify先预测预算失败，再运行第三条命令。需NumPy/MuJoCo/Matplotlib，无新增依赖。
ignored tmp/s14_3_rrt_*/存JSON/树图。路径按采样复查，圆盘解析评分独立于搜索。
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。没有平滑、时间化、RRT-Connect、UR5e或动态执行。

## S14.4 — 手写 RRT-Connect

[学习包](../../docs/17_4_rrt_connect.md) · [代码](rrt_connect.py)。
固定start/goal树身份，EXTEND三态、CONNECT重复扩展、交换角色、反转goal链拼接。

```bash
python examples/15_motion_planning/rrt_connect.py
python examples/15_motion_planning/rrt_connect.py --max-attempts 20
```

复用NumPy/MuJoCo/Matplotlib；无新增依赖。默认8seed两者8/8；20attempt预算下RRT0/8、Connect8/8。
比较统一均匀采样（单树goal_bias=0），统计所有尝试；same seed不是same sample sequence。
该命令是评价器，预算失败仍exit0；看各run的status与null path，而非只看进程退出码。
输出ignored tmp/s14_4_connect_*/JSON与首个seed的双图。Modify先预测20attempt预算影响。
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。无平滑、时间化、UR5e或动态执行。

## S14.5 — Collision-Checked Path Shortcuts

[学习包](../../docs/17_5_path_smoothing.md) · [代码](path_smoothing.py)。
新生成固定seed7 RRT-Connect路径，再用独立seed17做vertex shortcuts，保留起终点并重检所有输出边。

```bash
python examples/15_motion_planning/path_smoothing.py
python examples/15_motion_planning/path_smoothing.py --attempts 0
```

需NumPy/MuJoCo/Matplotlib，无新依赖。默认13→3顶点、1.774460→1.612793m，接受6次；0预算保持原path。
Modify先预测0预算对路径和检查的影响，再运行对照。产物仅ignored tmp/s14_5_shortcut_*/JSON/PNG。
粗step=.4m会误接受碰撞直连，独立解析评分拒绝（exit1），不掩盖采样漏检。
Engineering Complete；Learning待本人Run/Modify/Explain。仍为折线，无时间化/动力学/UR5e验证。
