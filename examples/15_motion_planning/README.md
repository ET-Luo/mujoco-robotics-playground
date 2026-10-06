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
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。仍为折线，无时间化/动力学/UR5e验证。

## S14.6 — Cubic Time Parameterization

[学习包](../../docs/17_6_time_parameterization.md) · [代码](time_parameterization.py)。
固定S14.5几何path，复用S10.8b cubic helper，按各轴速度/加速度解析峰值给各段配时，每个waypoint零端速。

```bash
python examples/15_motion_planning/time_parameterization.py
python examples/15_motion_planning/time_parameterization.py --velocity-scale 0.5
```

需NumPy/MuJoCo/Matplotlib；旧cubic module导入mujoco-menagerie，也是已声明必要import依赖，无新安装。
默认T=5.77/2.24s、total8.01s；限速减半total16.01s，path与静态clearance不变。
Modify先预测时间/速度/加速度/几何变化。ignored tmp/s14_6_timing_*/保存NPZ/JSON/三行图。
每个waypoint零速度、C1但通常非C2；JSON保存两侧加速度，NPZ连接点保留左值。
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered；仅参考生成，无mj_step/UR5e动态执行验证。

## S14.7a — UR5e Obstacle Planning and Tracking

[学习包](../../docs/17_7a_ur5e_obstacle_planning.md) · [代码](ur5e_obstacle_planning.py)。
官方裸UR5e、固定world球障碍：私有IK→六维joint RRT-Connect→shortcut→cubic→独立实际tracking检查。

```bash
python examples/15_motion_planning/ur5e_obstacle_planning.py
python examples/15_motion_planning/ur5e_obstacle_planning.py --velocity-scale 0.5
```

依赖NumPy/MuJoCo/Matplotlib/Menagerie，均已声明；无OpenCV/RL或新增安装。
默认14nodes/25扩展、9→3path；参考8.488s、执行10.488s；tracking max=.039322rad、final position=.024871mm。
限速减半tracking=.019763rad、sim18.974s；Modify先预测path/时间/tracking变化。
`--max-nodes 2`预期exit1/partial结果，无execution。ignored tmp/s14_7a_ur5e_*/存JSON/NPZ/图。
裸臂compiled碰撞几何有覆盖限制；使用理想bias外力；离散检查不保证连续/硬件安全。
Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。无gripper/持物/phase或GUI验证。

## S14.7b — Collision-Aware Pick-and-Place

[学习包](../../docs/17_7b_collision_aware_pick_place.md) · [代码](collision_aware_pick_place.py)。
已知pose输入；名义held T_GO、phase/pair/depth策略，transit/transfer RRT-Connect+shortcut+cubic，实际自由物体接触取放。

```bash
python examples/15_motion_planning/collision_aware_pick_place.py
python examples/15_motion_planning/collision_aware_pick_place.py --trials 1 --close-target 0.014
```

需已声明NumPy/MuJoCo/Matplotlib/Menagerie/OpenCV（复用module imports），无新增依赖。
固定场景planner seeds7/8/9共3/3成功，final error .758/.930/.807mm；不代表物理随机化或普遍100%可靠。
Modify先预测14mm slide的48mm opening与close失败。评价器exit0不代表每trial成功，检查status/失败phase。
`--trials 1 --max-nodes 2`在transfer规划失败，保留已执行close/lift，不继续transfer。
产物仅ignored tmp/s14_7b_pick_*/JSON/CSV/NPZ/图；包含partial失败。
Engineering Complete；Learning待本人Run/Modify/Explain。无weld/物体truth目标修正/GUI/硬件验证。
