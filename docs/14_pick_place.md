# UR5e Known-Pose Pick & Place

## S11.1 — 把简化夹爪接到 UR5e（Engineering Complete）

### Problem → Why

Stage 7 的夹爪是独立模型。进入完整操作任务前，先只解决机械集成：夹爪根坐标系是否真的
跟随 UR5e 的 `attachment_site`，手指 joint/actuator 是否一一对应，碰撞 geom 是否有效，
空载开合是否对称。本课没有物体，也不移动机械臂，因此不宣称抓取或整机无碰撞运动。

### Intuition

把 `attachment_site` 想成法兰上的安装坐标系 `{E}`。夹爪根 body 的局部原点和局部轴对齐
`{E}` 后，机械臂运动会通过刚体树自动带动夹爪。两个手指沿夹爪局部 `±x` 滑动；每个
joint 的 `qpos` 是从安装中心向外的距离，不是总开口。

### Core concepts and APIs

- `mujoco.MjSpec.from_file(path)` 读取可编辑的 MJCF specification；返回 `MjSpec`，尚未是
  仿真用的 `MjModel`。
- `robot_spec.attach(gripper_spec, site=...)` 原地修改 robot specification，把子模型根
  frame 安装到给定 site；本例无额外 offset。
- `robot_spec.compile()` 把组合 specification 编译为不可变拓扑的 `MjModel`。
- `mujoco.MjData(model)` 创建可变状态；`qpos` 中 UR5e hinge 的单位为 rad，手指 slide 为 m。
- `mujoco.mj_forward(model, data)` 根据当前状态原地更新派生位姿，不推进时间。
- `mujoco.mj_step(model, data)` 原地推进一个动力学时间步，使 position actuator 驱动手指；
  不返回新状态。

`data.site_xpos[site_id]` 与 `data.xpos[base_id]` 均为 world-frame `(3,)` 米；对应 `xmat`
均为展平的 `(9,)` 无量纲旋转矩阵。本例要求两组位姿数值重合，证明没有隐藏安装偏移。

### Mathematics → code

安装变换为

\[
{}^W T_G = {}^W T_E\,{}^E T_G, \qquad {}^E T_G=I,
\]

所以夹爪根 `{G}` 与 attachment frame `{E}` 的世界位置和旋转相等。

手指中心初始位于 `x=±0.015 m`，每个手指 x 半宽为 `0.005 m`，零位内侧间距为
`0.02 m`。两关节沿相反方向向外为正，因此

\[
w = 0.02 + q_L + q_R\quad [m].
\]

代码按名称得到 joint/actuator ID，再检查
`model.actuator_trnid[actuator_id, 0] == joint_id`。单位传动下，position actuator 的
`ctrl` 与 slide target 都是米。手指 geom 的 `contype` 与 `conaffinity` 非零，表示它们不是
纯视觉几何；这不等价于已经验证所有未来碰撞组合。

### Minimal experiment

入口：[integrated_gripper.py](../examples/12_pick_place/integrated_gripper.py)，子模型：
[gripper.xml](../examples/12_pick_place/gripper.xml)。

```bash
python examples/12_pick_place/integrated_gripper.py
python examples/12_pick_place/integrated_gripper.py --target 0.015
```

默认从每指 `q=0.01 m`（开口 `0.04 m`）开始，命令每指 `0.03 m`，运行 1000 个
`0.002 s` 步长。预期终态开口 `0.08 m`。修改实验预期终态开口 `0.05 m`。

### Expected / actual result

2026-10-03 助手在 WSL2、已核验的 `mujoco` conda 环境、
`/home/lucas/miniconda3/envs/mujoco/bin/python`、MuJoCo 3.14.0 下验证：

- 默认命令退出 0；安装位置/旋转误差均为 `0`，终态双指 `qpos=[0.03, 0.03] m`，
  开口 `0.08 m`，空载运动中 finger contact pair 数为 0。
- `--target 0.015` 退出 0；终态双指 `qpos=[0.015, 0.015] m`，开口 `0.05 m`。
- 两个 finger collision geom 的 `contype=1`、`conaffinity=1`；两组 transmission、对称性
  和 `0.1 mm` 目标误差断言通过。首次试跑使用了不合理的 `1e-12 m` 动力学对称容差，因
  两指相差约 `7.2e-8 m` 而失败；改为与目标检查一致的物理容差后两组完整复跑通过。

### Explanation and failure cases

- 写 `ctrl` 只设伺服目标；`qpos` 要经 `mj_step` 才运动。
- 若把 `q=0.03 m` 误当总开口，会漏掉零位几何间距和另一手指位移。
- frame 原点重合但旋转不重合时，夹爪仍会沿错误方向开合。
- geom 可碰撞且空载无接触，只说明这条静止机械臂测试路径正常；加入物体、地面或移动机械臂
  后仍可能碰撞。
- position servo 的空载收敛不证明有载荷时能保持目标，也不证明抓取稳定。

### Robotics context

真实末端工具集成也要明确 tool-center-point 安装变换、关节零位、传动方向、软硬限位和
collision model。这里的 `attachment_site` 相当于已知 flange/tool frame；后续 grasp pose
必须说明是物体、世界、base 还是 end-effector frame，但那属于 S11.2。

### Interview capsule

**30 秒：** 我用 MuJoCo `MjSpec.attach` 将简化平行夹爪安装到 UR5e 的
`attachment_site`，验证根 body 与 site 的世界位姿完全一致，核对两个 actuator 的 joint
transmission 和可碰撞 geom，再用动力学空载开合验证对称位置响应。这个实验只证明模型集成，
不证明抓取或整机轨迹安全。

**2 分钟：** 除上述内容外，应指出 hinge `qpos` 是 rad、finger slide `qpos/ctrl` 是 m；
总开口为零位间距加两个关节位移。`mj_forward` 只更新派生量而不走时间，`mj_step` 才使
servo target 通过动力学影响状态。碰撞位有效只代表 geom 被纳入过滤规则；真正任务还需
检查具体 contact pairs、物体接触、负载保持和沿轨迹的自碰撞/环境碰撞。

### Must remember

1. 安装正确需要同时核对 position 与 orientation，不能只看原点。
2. actuator `ctrl`、joint `qpos` 与 opening 是三个不同量。
3. collision geom 存在、空载无接触、抓取成功是三层不同证据。
4. 本课只完成 S11.1，不提前实现物体 grasp pose 或 IK。

### Run / Modify / Explain handoff

- **Run**：亲自运行默认命令，确认 frame error、mapping、终态 `qpos/opening` 和 PASS。
- **Modify**：运行 `--target 0.015`，在运行前算出预期开口，再核对实际值。
- **Explain**：回答：
  1. 为什么 origin error 为零仍不足以证明 attachment orientation 正确？
  2. 默认每指 target 为 `0.03 m` 时，为什么总开口是 `0.08 m`？
  3. `ctrl` 写入后为何 `qpos` 不会瞬间相等？
  4. `contype/conaffinity` 非零且空载 contact 为零，仍不能证明哪些事情？

## Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已解释：同一原点不代表完整 pose 相同；默认开口为
`0.02 + 0.03 + 0.03 = 0.08 m`；`ctrl` 是 position actuator 的目标而非对 state 的瞬时
赋值；碰撞 geom 已启用且本次空载无 finger contact，仍不能证明稳定抓取、运动轨迹无碰撞
或 collision geometry 与 visual geometry 完全一致。本人随后亲自运行默认与
`--target 0.015` 两条命令，确认开口符合 `0.08/0.05 m` 预期且均 PASS；Run、Modify、
Explain 均完成，S11.1 Learning Mastered。

## S11.2 — 手工定义 grasp 与 pre-grasp pose（Engineering Complete）

### Problem → Why

物体 pose 已知并不自动给出机器人末端目标。必须额外指定：夹爪相对物体的 offset、哪根
末端局部轴是 approach axis、手指开合轴如何对准物体，以及 pre-grasp 应沿哪个 frame 的
方向后退。本课只生成和检查两个 6D pose，不运行 IK、trajectory、actuator 或 contact。

### Intuition

方块 frame 记为 `{O}`，world 为 `{W}`，夹爪根/末端目标为 `{G}`，pre-grasp 为 `{P}`。
本例选择 top-down grasp：夹爪 `+z_G` 朝世界下方 `-z_W`，手指沿 `±x_G` 开合；因此两指
从物体的 world `±x` 两侧包围。S11.1 的 finger geom 中心位于夹爪根的 `+0.035 m z_G`，
所以夹爪根目标放在物体中心上方 `0.035 m`，手指中心才与物体中心重合。

pre-grasp 不是“world z 随便加一个数”的通用定义，而是从 grasp 沿 approach axis 的反向
后退。在本例 top-down 特例中，它恰好表现为 world `+z`。

### Core concepts, frames, shapes, and units

- `p_WO`：物体原点在 world 中的位置，shape `(3,)`，单位 m。
- `R_WO`：把 object-frame 向量表达为 world-frame 向量的旋转，shape `(3,3)`，无量纲。
- `T_WO`：齐次变换，shape `(4,4)`；旋转在左上，位置在右上。
- `T_OG`：grasp 相对 object 的固定设计 offset，不是当前机械臂 FK。
- `T_WG = T_WO T_OG`：期望 gripper pose 在 world 中的表达。
- `R_WG[:, 2]`：局部 `+z_G` 在 world 中的方向。本例为 `[0,0,-1]`。
- `{B}`：UR5e base frame。当前 Menagerie 模型的 base 原点与 world 原点重合，但轴绕
  world z 旋转 π；所以 `p_WO` 与 `p_BO` 的数值不同。这个关系是当前模型安装约定，不应
  当作所有 UR5e 场景的通则。

旋转矩阵必须满足 `R.T @ R = I` 且 `det(R)=+1`。仅位置合理而 orientation 不合理，会让
腕部到达物体附近却以错误方向接近。

### Mathematics → code

物体水平放置，因此 `R_WO=I`，其已知中心为

\[
p_{WO}=(-0.45,\ 0.20,\ 0.03)\ \mathrm{m}.
\]

top-down 末端朝向和 object-frame offset 定义为

\[
R_{OG}=\operatorname{diag}(1,-1,-1),\qquad
p_{OG}=(0,0,0.035)\ \mathrm{m}.
\]

`R_OG` 保留 `+x`，同时翻转 y/z；其行列式为 `+1`，所以这是合法的 180° rotation，
不是 reflection。齐次变换复合为

\[
T_{WG}=T_{WO}T_{OG},
\]

得到 `p_WG=(-0.45, 0.20, 0.065) m`。令 local approach unit vector
`a_W=R_WG e_z=(0,0,-1)`，后退距离 `d>0`，则

\[
p_{WP}=p_{WG}-d a_W.
\]

从 pre-grasp 到 grasp 的同一物理位移为

\[
\Delta p_W=d a_W=(0,0,-d),\qquad
\Delta p_G=R_{WG}^{T}\Delta p_W=(0,0,d).
\]

这展示了向量不变，但坐标数值取决于表达 frame。

### Minimal experiment

[pose_planning.py](../examples/12_pick_place/pose_planning.py)只用 NumPy 构造并复合变换，
检查旋转、finger-center 对齐、pre-grasp offset 和 world/base 两种表达：

```bash
python examples/12_pick_place/pose_planning.py
python examples/12_pick_place/pose_planning.py --pregrasp-distance 0.15
```

### Expected / actual result

2026-10-03 助手在 WSL2、核验后的 `mujoco` conda 环境和 Python 3.12.14 中运行：

- `d=0.10 m`：`p_WP=(-0.45,0.20,0.165) m`，pre→grasp 位移在 world 中为
  `[0,0,-0.10] m`，在 gripper 中为 `[0,0,+0.10] m`。
- `d=0.15 m`：`p_WP=(-0.45,0.20,0.215) m`，两种表达相应为
  `[0,0,-0.15] m` 和 `[0,0,+0.15] m`。
- object 的 world/base 位置分别为 `[-0.45,0.20,0.03] m` 与
  `[0.45,-0.20,0.03] m`；它们描述同一物理位置。
- 两条命令均退出 0，所有 rotation、alignment 和 offset 断言通过。

没有加载动力学状态、求 IK、发送 actuator command、调用 `mj_step` 或产生 contact。

### Explanation and failure cases

- 若误用 `p_WP=p_WG+d a_W`，pre-grasp 会继续朝物体内部移动，而不是后退。
- 只指定位置不指定 orientation，会留下无穷多个末端姿态，无法保证 top-down 或开合轴对齐。
- 把 local offset 直接加到 world position，只有两 frame 轴恰好对齐时才正确；一般应先乘旋转。
- base 与 world 原点重合不代表 axis 相同；本模型的 x/y 数值符号会翻转。
- 几何 pose 合法不代表 UR5e 可达、不奇异、不碰撞；这些是 S11.3 的 IK/trajectory 检查。
- grasp pose 也不证明手指宽度、摩擦和夹持力足以稳定抓住实际物体。

### Robotics context

工业抓取通常先在 object frame 中设计候选 grasp，再通过感知或已知工位标定得到 `T_WO`，
最后复合出 robot 所需的 world/base target。pre-grasp 提供直线接近段的起点，也给安全检查、
视觉修正和夹爪预张开留下空间。

### Interview capsule

**30 秒：** 我在 object frame 中定义 top-down grasp offset，通过 `T_WG=T_WO T_OG`
得到 world-frame 末端目标，再沿末端局部 approach axis 的反方向生成 pre-grasp。验证同一
pre→grasp 位移在 world 中是向下，在 gripper frame 中是 `+z`，并明确这一步只做几何规划，
不代表 IK 可达或抓取成功。

**2 分钟：** 还应说明完整 pose 包含 position 与 orientation；局部 offset 必须经过旋转再
转换到 world。`R_WG[:,2]` 是 gripper `+z` 在 world 中的表达，故
`p_pre=p_grasp-d R_WG[:,2]`。本例用 `R_OG=diag(1,-1,-1)` 实现 top-down 且保持合法
rotation。world/base/end-effector 是不同表达 frame；坐标可以不同但表示同一物理量。

### Must remember

1. grasp 是相对物体设计的 6D pose，不只是一个 xyz 点。
2. pre-grasp offset 必须注明方向属于哪个 frame。
3. rotation matrix 的列是局部轴在父 frame 中的方向。
4. pose generation、IK feasibility、collision-free motion 和 grasp stability 是不同证据层。

### Run / Modify / Explain handoff

- **Run**：运行默认命令，确认 `p_WG/p_WP`、两种 delta 表达和 PASS。
- **Modify**：先预测再运行 `--pregrasp-distance 0.15`，确认 grasp 不变而 pre-grasp 改变。
- **Explain**：回答：
  1. 为什么 `R_OG=diag(1,-1,-1)` 是 rotation，而 `diag(1,1,-1)` 不是？
  2. 为什么从 pre-grasp 到 grasp 在 world 中是 `-z`，在 gripper 中却是 `+z_G`？
  3. 为什么 base 与 world 原点相同，物体坐标仍可能不同？
  4. 本课生成的两个合法 pose 还不能证明哪些执行层面的性质？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [ ] Run [ ] Modify [ ] Explain
