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
