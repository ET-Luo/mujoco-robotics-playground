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
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已运行并验证默认与修改实验。Explain 确认：合法 3D rotation 需正交且
`det(R)=+1`，因此 `diag(1,-1,-1)` 是纯旋转，而 determinant 为 `-1` 的
`diag(1,1,-1)` 是 reflection；同一几何向量在不同 frame 中可有不同坐标分量；同一原点
不代表坐标轴朝向相同；几何目标合法不能证明 IK 有解或解的质量，也不能证明关节限位、
奇异性、轨迹碰撞、动力学跟踪与抓取稳定。S11.2 Learning Mastered。

## S11.3 — 从 home 到 pre-grasp（Engineering Complete）

### Problem → Why

S11.2 只定义了目标 pose。本课把它变成一条经过检查的几何 motion reference：先从 UR5e
home 用受限 DLS 求 pre-grasp joint configuration，再生成零端点速度 cubic joint trajectory，
逐样本检查末端终点误差、关节限位、速度上限和碰撞。夹爪保持 `0.08 m` 张开，不向 grasp
接近、不闭爪。

### Intuition

任务分两层：IK 回答“末端目标对应哪组关节角”，trajectory 回答“如何从 home 平滑连接到
这组角度”。终点合法不保证中间路径合法，所以必须对整条离散 reference 检查 limits 和
contacts。反过来，离散几何检查通过也不等于真实 servo 能准确跟踪。

### Core concepts and APIs

- `mujoco.mj_jacSite(model, data, jacp, jacr, site_id)`：原地填写 `(3,nv)` position/angular
  Jacobian；本例只选择六个 UR5e DOF 列，不让两个 finger slide 参与 IK。
- DLS 每轮在 world frame 使用 `[position_error(m), orientation_error(rad)]`，并限制每个
  geometric update 最大为 `0.05 rad`。
- `data.qpos[...] = q_sample` 后调用 `mujoco.mj_forward`：原地更新 FK、collision detection
  与 `data.contact`，但不推进 `data.time`，也不产生实际 `qvel` tracking。
- `data.contact`：当前 configuration 的接触数组；脚本把 geom name pair 汇总，而不是只看
  `ncon` 数量。
- 方块固定在 world `(-0.45,0.20,0.03) m`，夹爪两指保持 `q=0.03 m`，即开口 `0.08 m`。

### Mathematics → code

DLS 更新沿用 Stage 10：

\[
\Delta q=J^T(JJ^T+\lambda^2I)^{-1}e,
\]

其中 `J` 为 `(6,6)`，`e` 为 `(6,)`，`Δq` 为 `(6,)`。候选关节角通过 limits 后才接受，
每轮重新计算 FK、误差与 Jacobian；成功条件同时要求

\[
\lVert e_p\rVert < 10^{-4}\ \mathrm{m},\qquad
\lVert e_R\rVert < 10^{-3}\ \mathrm{rad}.
\]

由 `q_0` 到 `q_f` 的 cubic reference 使用

\[
s(\tau)=3\tau^2-2\tau^3,\quad
q(t)=q_0+s(\tau)(q_f-q_0),\quad \tau=t/T,
\]

\[
\dot q(t)=\frac{6\tau-6\tau^2}{T}(q_f-q_0).
\]

因此同一 joint-space path 的 peak velocity 与 `1/T` 成正比；增加 duration 不改变 path，
只改变时间参数化。

### Minimal experiment

[pregrasp_motion.py](../examples/12_pick_place/pregrasp_motion.py)编译 UR5e、已安装夹爪和固定
方块，求 IK 并逐个 trajectory sample 调用 `mj_forward`：

```bash
python examples/12_pick_place/pregrasp_motion.py
python examples/12_pick_place/pregrasp_motion.py --duration 5
```

### Expected / actual result

2026-10-04 助手在 WSL2、核验后的 `mujoco` conda 环境、Python 3.12.14、MuJoCo 3.14.0
中运行：

- IK 均在 21 次 accepted updates 后成功，位置残差 `0.000062233 m`，朝向残差
  `0.000001508 rad`。
- `q_home=[-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0] rad`；
  `q_pregrasp≈[-0.693762,-1.269379,2.156367,-2.457783,-1.570796,0.877033] rad`。
- `T=3 s`：301 samples，peak analytic joint speed `0.443492 rad/s`。
- `T=5 s`：501 samples，peak analytic joint speed `0.266095 rad/s`；比例约为 `3/5`。
- 两组全部 samples 均在 joint limits 内，汇总 contact pair 均为空，终点误差与 IK residual
  一致；两条命令退出 0、断言通过。

未运行 GUI、`mj_step` 或 actuator tracking；`data.time` 始终为 0。

### Explanation and failure cases

- IK 成功只说明找到了一个满足终点容差的 configuration，不说明从 home 的路径安全。
- endpoint limits 合法不够；虽然 cubic 在每个关节上位于两端角度之间，本课仍显式检查每个
  sample，保持检查逻辑适用于后续更一般轨迹。
- `dt=0.01 s` 的 contact-free samples 不是连续碰撞证明；两个采样姿态之间仍可能漏掉短暂
  碰撞。更高安全要求需要更密采样或连续碰撞检测。
- 未加入 floor，因此“无碰撞”只覆盖已编译模型中的 UR5e、夹爪和固定方块，不覆盖未知环境。
- duration 变长降低 reference velocity，但不会改变 IK 解、几何 path 或 clearance。
- 直接写 `qpos` 不产生 actuator lag、torque saturation、actual `qvel` 或 tracking error。

### Robotics context

工业 motion pipeline 通常依次做 target construction、IK、time parameterization、collision
checking 和 controller execution。本课完成前三项的最小版本及离散 collision checking；
S11.4 才沿末端局部轴接近并闭爪。

### Interview capsule

**30 秒：** 我用受限 DLS 从 UR5e home 求到已知 pre-grasp pose，再用 cubic time law 生成
零端点速度 joint reference。对每个离散 configuration 检查 joint limits 和 MuJoCo contact
pairs，并检查终点位置/朝向容差。3 秒与 5 秒走相同几何路径，后者 peak velocity 按 `1/T`
降低。该结果是 sampled geometric validation，不是 actuator tracking 或连续碰撞证明。

**2 分钟：** 还应说明 Jacobian 只取六个 arm DOF，避免 finger joints 被 IK 当成末端 pose
自由度；DLS 每轮重算 FK/Jacobian 并限制步长。cubic 的 `q(t)` 由同一 scalar phase 作用于
所有 joint，保持 joint-space path，仅改变 timing。`mj_forward` 在直接设置 qpos 后更新
kinematics/contact，不推进时间。离散无接触的证据范围取决于模型内容与 sampling resolution。

### Must remember

1. IK endpoint feasibility 与 path feasibility 是两项检查。
2. trajectory duration 改速度，不改变同一 cubic joint-space path。
3. collision result必须注明模型包含哪些物体以及采样分辨率。
4. reference qdot 与真实 `data.qvel`/actuator tracking 不能混为一谈。

### Run / Modify / Explain handoff

- **Run**：运行默认命令，确认 IK residual、301 samples、peak speed、limits/contact 和 PASS。
- **Modify**：运行 `--duration 5`，确认 IK/q_goal/path checks 不变，peak speed 降为约
  `0.266095 rad/s`。
- **Explain**：回答：
  1. 为什么只验证 pre-grasp endpoint 的 IK 和 joint limits 仍不足以验证整条路径？
  2. 为什么 duration 从 3 s 变 5 s 后 joint-space path 不变，而 peak qdot 变为约 `3/5`？
  3. `mj_forward` 逐样本得到零 contact，为什么不能当作连续碰撞证明或动力学跟踪证据？
  4. 为什么 IK Jacobian 必须排除两个 finger slide DOF？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已实际运行并验证默认 3 s 与修改后的 5 s 实验。Explain 确认：终点 IK/limits 合法
不能推出所有中间 configuration 合法；对 `q(t)` 求导且 `tau=t/T` 会带出 `1/T`，所以
同一路径 peak qdot 变为 `3/5`；离散无碰撞不等于连续无碰撞，且 `mj_forward` 不推进时间、
不含 actuator/力矩/惯性，不能证明动力学跟踪；pre-grasp 的 attachment-site motion 应只由
六个 arm DOF 完成。补充精确化：finger joints 位于 attachment site 下游，对该 site 的
Jacobian columns 本来为零；显式排除它们用于保证 arm-only 任务语义和手指保持张开。
S11.3 Learning Mastered。

## S11.4 — 沿局部轴接近并闭爪（Engineering Complete）

### Problem → Why

pre-grasp 到 grasp 不是任意两点插值：接近方向必须绑定末端 frame。本课保持 orientation 和
张爪状态不变，用一串 IK targets 沿夹爪局部 `+z_G` 接近；到达 grasp 后才运行闭爪动力学，
并在首次左右手指同时接触固定方块时停止。双侧接触只是现象，不等于稳定 grasp。

### Intuition

本例 `+z_G` 在 world 中指向 `[0,0,-1]`。因此“沿 gripper `+z` 前进 0.10 m”在 world
坐标中看起来是向下 `0.10 m`。几何箭头没有改变，改变的是坐标表达。接近阶段手指保持
`0.08 m` 开口，避免提前碰撞；到 grasp pose 后将每指 position target 从 `0.03 m` 改为
`0.005 m`，手指逐步向内运动，接触力使实际 qpos 在到达 target 前被物体阻挡。

### Core concepts and APIs

- `a_W = R_WG[:,2]`：gripper local `+z_G` 在 world frame 的方向，shape `(3,)`，无量纲。
- 每个 approach target 为 `p(alpha)=p_pre+alpha*d*a_W`，单位 m，`alpha∈[0,1]`。
- 每个 target 都从上一 IK 解 warm-start；这减少更新次数，但不把 IK 变成连续控制器。
- `mujoco.mj_forward` 用于 approach 的几何 pose/contact 检查，不推进时间。
- `mujoco.mj_step` 用于闭爪阶段，原地推进 `0.002 s` 动力学；position actuator 的
  `ctrl` 是目标，实际 finger `qpos` 受惯性与 contact constraint 影响。
- contact 判据按 geom name set 检查左右两对，不依赖 MuJoCo 返回顺序。

### Mathematics → code

给定 grasp orientation

\[
R_{WG}=\operatorname{diag}(1,-1,-1),\qquad
a_W=R_{WG}e_z=(0,0,-1),
\]

从 pre-grasp 到 grasp 的位移为

\[
\Delta p_W=d a_W=(0,0,-d).
\]

同一位移表达在 gripper frame：

\[
\Delta p_G=R_{WG}^{T}\Delta p_W=(0,0,d).
\]

数值符号不同不是运动矛盾，而是 basis 不同。脚本还用各个 achieved IK position 到首尾
直线插值的距离衡量 sampled line deviation；默认最大约 `5.59e-5 m`，来源于每个 IK target
允许的有限位置容差。

双侧接触集合要求同时包含

```text
{left_finger_collision, known_object_collision}
{right_finger_collision, known_object_collision}
```

且本实验终止时不允许额外 contact pair。

### Minimal experiment

[approach_and_close.py](../examples/12_pick_place/approach_and_close.py)复用 S11.3 的集成模型和
DLS solver：

```bash
python examples/12_pick_place/approach_and_close.py
python examples/12_pick_place/approach_and_close.py --approach-distance 0.08
```

### Expected / actual result

2026-10-04 助手在 WSL2、核验后的 `mujoco` conda 环境、Python 3.12.14、MuJoCo 3.14.0
中运行：

- `d=0.10 m`：world achieved displacement 约
  `[-0.000023,0.000050,-0.099972] m`；gripper 表达约
  `[-0.000023,-0.000050,+0.099972] m`；11 samples 共 41 次 IK updates，sampled line
  deviation 最大 `0.000055928 m`。
- `d=0.08 m`：world/gripper 的主分量分别为 `-0.079950/+0.079950 m`；11 samples 共
  32 次 IK updates，最大 line deviation `0.000067851 m`。
- 两组 open approach contact pairs 均为空；闭爪均在第 28 step、`0.056 s` 首次双侧接触。
- 接触时 opening 约 `0.03653 m`，小于方块名义宽度 `0.04 m`，反映 MuJoCo soft contact
  的有限 penetration；实际 finger qpos 未到 `0.005 m` command target。
- 终止时 contact set 恰好只有左右 finger–object 两对；两条命令退出 0、断言通过。

首次实现只断言存在双侧物体接触，但输出同时暴露 finger 与未命名 wrist collision geom 的
额外接触。随后把未命名 geom 报告改为 `body/<unnamed:id>`，定位为 mechanically adjacent
wrist₂/wrist₃ 的 coarse collision shapes，并为这四组相邻 tool-body pairs 添加 MuJoCo
collision exclusions。修正后完整复跑通过，S11.3 的 5 s 路径也回归通过。

### Explanation and failure cases

- 把 `[0,0,d]` 直接当 world displacement 会向上离开物体；必须先用 `R_WG` 变换局部轴。
- 逐点 IK 只近似 Cartesian line；有限 tolerance 会留下几十微米横向 deviation。
- 只看 `ncon>=2` 不可靠：两条 contact 可能都来自同一手指，或包含工具自身伪碰撞。
- collision exclusion 只应覆盖已知 mechanically adjacent、粗几何重叠的 body pairs；若广泛
  关闭 collision，会隐藏真实自碰撞。本课仍严格要求 finger–object 之外没有其他 pair。
- 方块固定，因此 bilateral contact 不能证明物体自由时不会被推走、旋转或滑落。
- soft-contact penetration 表示 collision constraint 的数值行为，不应把 `opening<0.04 m`
  解读为刚性方块真实压缩。
- 本课不检查 contact force closure、摩擦裕量、保持时间、抬升或 object retention。

### Robotics context

真实抓取常将长距离 motion planning 与最后的 task-space guarded approach 分开。局部接近轴
来自 tool frame；闭爪终止可由位置、力矩、电流或接触传感器触发。本例用仿真 contact pair
作为最小事件，但没有实现力控或触觉反馈。

### Interview capsule

**30 秒：** 我从 pre-grasp 保持夹爪朝向和开口，用连续 warm-start IK targets 沿 tool
local `+z` 接近。该轴在 world 中是 `-z`，所以同一位移在 world/gripper frame 分别主要为
`-0.10/+0.10 m`。到 grasp 后用动力学闭爪，并只把首次左右 finger–object 同时接触作为
本课成功现象，不宣称稳定抓取。

**2 分钟：** 还应说明 sampled IK path 有有限横向误差；`mj_forward` 负责 approach 的几何
检查，`mj_step` 才推进闭爪动力学。contact 判据应检查命名 pair 集合而非数量或顺序。
调试中发现 coarse wrist geom 与相邻 finger 的伪自碰撞，添加了严格限定的 adjacent-body
exclusions，并要求最终只剩两组 finger–object contact。固定物体和双侧瞬时接触仍不足以
证明 force closure、摩擦保持或可抬升。

### Must remember

1. local-axis command 必须经过 rotation 才能得到 world displacement。
2. 双侧接触要按左右 geom pair 分别验证，不能只数 contact 数量。
3. position target 不等于实际 finger qpos，接触会阻挡运动。
4. bilateral contact、stable grasp 和 successful lift 是三层不同证据。

### Run / Modify / Explain handoff

- **Run**：运行默认命令，确认 world/gripper displacement、line deviation、open approach
  无接触、第 28 step 双侧 contact 和 PASS。
- **Modify**：先预测再运行 `--approach-distance 0.08`，确认主位移改为 `-0.08/+0.08 m`，
  grasp pose 与闭爪接触现象不变。
- **Explain**：回答：
  1. 为什么同一 approach displacement 在 world 中主要为 `-z`，在 gripper 中为 `+z_G`？
  2. 为什么 achieved path 有几十微米横向 deviation，而不是精确直线？
  3. 为什么必须检查左右命名 contact pairs，而不能只判断 `ncon>=2`？
  4. 为什么固定方块上的 bilateral contact 不能证明 stable grasp 或 lift success？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已运行并验证默认 `0.10 m` 与修改后的 `0.08 m` 实验。Explain 确认：world `-z` 与
gripper `+z_G` 是同一空间位移在不同 frame 中的坐标表达；Cartesian targets 是理想值，
逐点 IK 只保证在有限 tolerance 内，因此存在几十微米横向 deviation；`ncon>=2` 只表示
至少两个 contact points，不能证明左右手指分别接触物体且没有额外不允许接触，必须检查
geom identity。本人还指出真实抓取需要足够摩擦抵抗重力；补充校准为固定方块没有运动
自由度，瞬时双侧接触未验证物体是否被推走/旋转/滑落、接触保持、相对滑移或随夹爪抬升。
因此这里只证明 bilateral contact event。S11.4 Learning Mastered。

## S11.5 — 自由物体小幅抬升（Engineering Complete）

### Problem → Why

S11.4 的方块固定在 world，双侧 contact 无法证明夹爪能承受重力。本课把方块改为带
freejoint 的 `0.05 kg` 刚体并加入 ground，在重力下闭爪稳定后，用 UR5e position actuators
执行小幅向上 cubic reference。分别检查物体 world-z、物体相对夹爪的滑移、以及左右接触
保持；不做水平转移或放置。

### Intuition

要区分三个量：夹爪升了多少、物体升了多少、物体相对夹爪滑了多少。如果夹爪上升而物体
留在桌面，不能叫 lift；如果物体上升但相对夹爪持续向下滑，也不是可靠保持。双侧接触应在
整个 lift 中持续存在，终态也应存在，而不是只记录闭爪瞬间。

### Core concepts and APIs

- `freejoint` 给物体 7 个 `qpos`（world xyz + quaternion）和 6 个 `qvel`，使其能受重力、
  contact 和 friction 运动。
- 原 UR5e `home` keyframe 创建早于追加 freejoint，因此 reset 后显式设置物体 7 个 qpos 为
  `[-0.45,0.20,0.03,1,0,0,0]`。
- `data.body("known_object").xpos` 是物体 body origin 的 world position，shape `(3,)`、m。
- arm `ctrl` 跟踪 cubic joint targets，finger `ctrl=0.005 m` 持续闭合；调用 `mj_step` 后
  包含惯性、重力、servo lag 与 soft contact。

### Mathematics → code

以闭爪稳定后的状态为 baseline：

\[
\Delta z_G=z_{G,f}-z_{G,0},\qquad \Delta z_O=z_{O,f}-z_{O,0},
\]

\[
\Delta z_{rel}=(z_{O,f}-z_{G,f})-(z_{O,0}-z_{G,0})
=\Delta z_O-\Delta z_G.
\]

负的 `Δz_rel` 表示物体相对夹爪向下滑，不代表物体 world z 必然下降。成功条件为

\[
\Delta z_O\ge h_{request}-0.01\ \mathrm{m},\qquad
|\Delta z_{rel}|\le0.01\ \mathrm{m},
\]

抬升控制步中 bilateral contact 比例至少 `95%`，且终态仍为 bilateral。

Menagerie position servos 有有限 stiffness。首次直接把 requested height 当 geometric EE
target 时，默认 actual gripper/object lift 仅为 `42.06/32.25 mm`，再加 `9.81 mm` 相对
下滑，未达到 unchanged `40 mm` object-lift 下限。脚本没有放宽判据，而是显式加入
`10 mm` target headroom：

\[
h_{EE,target}=h_{request}+0.01\ \mathrm{m}.
\]

输出仍分别报告 requested、geometric target、actual gripper lift 和 object lift。

### Minimal experiment

[lift_object.py](../examples/12_pick_place/lift_object.py)复用集成模型、IK 和 cubic reference：

```bash
python examples/12_pick_place/lift_object.py
python examples/12_pick_place/lift_object.py --lift-height 0.03
```

### Expected / actual result

2026-10-04 助手在 WSL2、核验后的 `mujoco` conda 环境、Python 3.12.14、MuJoCo 3.14.0
中运行：

- 请求 `0.05 m`、geometric target `0.06 m`：actual gripper lift `0.052069766 m`，object
  lift `0.042117614 m`，relative change z `-0.009952 m`。
- 请求 `0.03 m`、geometric target `0.04 m`：actual gripper lift `0.032048637 m`，object
  lift `0.022438687 m`，relative change z `-0.009610 m`。
- 两组 lift 均为 750/750 steps bilateral contact；终态 contact set 只有左右
  finger–object pairs，ground contact 已消失。
- 两条命令均退出 0，world lift、relative slip、contact retention 和有限值断言通过。

为避免手指与桌面摩擦阻止闭合，将 finger collision z half-size 调为 `0.028 m`，在 nominal
grasp 留出 `2 mm` tip clearance；这是保留真实 ground collision 的几何修正，不是用
exclusion 隐藏接触。S11.1 默认空载与 S11.4 `0.08 m` approach/close 均回归通过。

### Explanation and failure cases

- `gripper world-z lift>0` 不能证明物体离地，必须直接测量 object world pose。
- 只比较最终 contact 可能漏掉中途掉落后重新接触，因此统计整个 lift 的 bilateral fraction。
- bilateral contact 仍不自动证明 force closure；本课增加了重力抬升和滑移证据，但未测试
  扰动、质量或摩擦变化。
- geometric headroom 只补偿 finite-stiffness tracking；它不能替代 actual state checks。
- 初始/早期 lift 可有 ground contact；终态 ground contact 消失才证明已离开支撑面。
- 本课不证明 transfer、加速度裕量、放置、release 或 robustness。

### Robotics context

实际抓取验收常同时监测 tool pose、object pose、finger/contact state 和 slip。工业系统可能
使用视觉、触觉、编码器或电机电流估计滑移；这里使用 MuJoCo ground truth，先建立证据结构。

### Interview capsule

**30 秒：** 我把固定方块替换为 freejoint 物体，在重力下闭爪稳定后用 UR5e position
actuators 执行小幅 cubic lift。分别测量夹爪和物体 world-z、物体相对夹爪的 z 变化，并统计
整个 lift 的 bilateral contact。默认物体上升约 `42.1 mm`、相对下滑约 `10 mm`，750/750
steps 保持双侧接触；这比瞬时 contact 更强，但仍不是鲁棒性证明。

**2 分钟：** 还应说明 freejoint state 初始化、finite-stiffness tracking 与显式 `10 mm`
target headroom。成功基于 measured object motion，而不是 command；要求 object lift 接近
请求值、relative slip 受限、接触持续且终态离地。`Δz_rel=Δz_object-Δz_gripper`，所以负值
表示相对下滑。实验未覆盖扰动、参数变化、transfer 或 placement。

### Must remember

1. gripper lift、object lift、relative slip 是三个不同证据。
2. 瞬时 bilateral contact 不等于接触保持或稳定抬升。
3. command/IK target 与 actual simulated motion 必须分开报告。
4. freejoint qpos 包含 xyz 和 unit quaternion，追加后要正确初始化。

### Run / Modify / Explain handoff

- **Run**：运行默认命令，确认 actual gripper/object lift、relative z、750/750 bilateral、
  final ground contact 消失和 PASS。
- **Modify**：运行 `--lift-height 0.03`，确认 geometric target/actual lift 相应减小，但滑移
  判据和 contact retention 仍通过。
- **Explain**：回答：
  1. 为什么只看夹爪 world-z 上升不能证明物体被抬起？
  2. `relative change z<0` 的物理意义是什么？它是否表示物体 world z 下降？
  3. 为什么检查 lift 全程 bilateral fraction 和终态 contact，而不只检查一个瞬间？
  4. `10 mm` headroom 解决哪一层问题，为什么不能替代 actual object-state checks？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已运行并验证请求 `0.05/0.03 m` 两组实验。Explain 确认：夹爪上升时物体仍可能因
摩擦不足而滑落，必须直接检查 object world-z；`relative change z<0` 表示物体相对夹爪
向下滑，不代表 object world-z 一定下降；bilateral fraction 反映整个过程的接触保持，final
contact 反映终态仍被夹持；headroom 是规划裕量而不是成功证据。补充校准：本例 `10 mm`
headroom 的主要作用是补偿 finite-stiffness arm servo 的末端 tracking deficit，客观上也为
后续滑移留下高度空间，但不能保证或替代 measured relative slip、object lift 与 contact
retention。S11.5 Learning Mastered。

## S11.6a — 持物限速转移并下降到支撑（Engineering Complete）

### Problem → Why

本课保持闭爪，将自由物体从约 `(-0.447,0.198)` 水平转移到已知放置中心
`(-0.30,-0.10,0.03) m`，再下降到 ground 支撑。检查 command/actual joint speed、相对
滑移、最终物体位置误差、支撑接触和双侧手指接触；不释放、不撤离。

### Intuition and core concepts

流程分 horizontal transfer 与 descent 两段。水平阶段保持高度；下降目标根据转移后的实测
`object-to-gripper` offset 计算，让预测的物体中心落在 `z=0.03 m`。速度更慢会减小瞬时
动态负担，但更长持物时间也会积累重力蠕滑，所以 duration 是 speed margin 与 retention
time 的权衡。

- analytic cubic `qdot` 是 command speed；`data.qvel[arm_dof]` 是 actual speed，均为 rad/s。
- drop evidence 使用各阶段最小 relative-z change，并要求 bilateral fraction ≥95%。
- support 用命名 pair `ground–known_object_collision` 判断；最终仍要求左右 finger–object，
  表示尚未 release。

### Mathematics → code

用实测 held offset 将 desired object pose 转成 gripper target：

\[
p_{G,xy}^{*}=p_{O,xy}^{*}-p_{O/G,xy}^{measured},\qquad
p_G^{*}=p_O^{*}-p_{O/G}^{measured}.
\]

成功条件包括

\[
\max|\dot q_{cmd}|\le0.5\ \mathrm{rad/s},\quad
\min\Delta z_{rel}\ge-0.015\ \mathrm{m},
\]

最终 object position error norm ≤`0.015 m`，support/bilateral 均为真。

### Minimal experiment

[transfer_and_descend.py](../examples/12_pick_place/transfer_and_descend.py)：

```bash
python examples/12_pick_place/transfer_and_descend.py
python examples/12_pick_place/transfer_and_descend.py --transfer-duration 2
```

### Expected / actual result

2026-10-04 在核验后的 WSL2、`mujoco` conda、Python 3.12.14、MuJoCo 3.14.0 运行：

- `T=3 s`：peak command/actual `0.289467/0.284544 rad/s`，最差 relative-z
  `-0.009989 m`；最终 object `[-0.298030,-0.100276,0.029361] m`，error norm
  `0.002090 m`。
- `T=2 s`：peak command/actual `0.434200/0.418027 rad/s`，最差 relative-z
  `-0.006908 m`；最终 object `[-0.298125,-0.100151,0.029341] m`，error norm
  `0.001994 m`。
- 两组 transfer/descent bilateral fraction 都为 `1.000`；下降阶段分别有 231/220 steps
  检测到 support，终态同时包含 ground–object 与左右 finger–object contacts。
- 两条命令退出 0，速度、drop、contact、support 和 pose error 断言通过；未 release。

额外 `T=5 s` 实验虽然速度降到约 `0.174 rad/s`，但 relative-z 超过 `-0.015 m` 而失败。
尝试 finger `kp=200/300/400` 未稳定消除长时间蠕滑，最终保留与 Stage 7 一致且直接的
`kp=200`，用 2 s 作为 Modify。bilateral contact 一直存在也不能排除指间慢滑。

### Explanation and failure cases

- 只看最终 pose 会漏掉途中掉落后落到支撑面的情况，必须监测 relative drop 与 contacts。
- 更长 duration 降低速度但可能增加重力滑移；更短 duration 会增加速度/加速度负担。
- support contact 不表示已安全释放；本课终态 fingers 仍闭合接触。
- measured relative offset 能补偿已有滑移，但不是在线视觉或闭环 object control。
- 最终约 2 mm error 包含 servo tracking、soft contact、slip 与支撑约束影响。
- release、finger separation、retreat 与低 object velocity 属于 S11.6b。

### Robotics context and interview capsule

真实 transfer 同时受速度、加速度、retention、collision 与目标容差约束；支撑确认应先于
release。面试时可概括：我分段执行限速 transfer 与 supported descent，监测 analytic command
speed、actual qvel、relative drop、bilateral contacts、object pose error 和 ground support。
3 秒方案约 `0.29 rad/s`、10 mm slip、2.1 mm final error；5 秒更慢却因持物更久而失败，
说明 contact persistence 与 slow motion 都不能单独证明 no-slip。

### Must remember

1. speed limit 与 retention time 存在权衡，慢不自动等于安全。
2. bilateral contact 保持不代表物体没有在手指间滑动。
3. support detected 与 release complete 是两个阶段。
4. object pose error 应从 actual object state 计算。

### Run / Modify / Explain handoff

- **Run**：运行默认 3 s，确认速度、relative drop、bilateral、support、pose error 和 PASS。
- **Modify**：运行 `--transfer-duration 2`，确认 peak speed 上升但低于限制，持物时间和
  relative slip 减小，最终 support/pose 仍通过。
- **Explain**：回答：
  1. 为什么 5 s transfer 更慢却可能比 3 s/2 s 累积更多 relative slip？
  2. 为什么 bilateral fraction=1 仍不能单独证明没有掉落趋势？
  3. ground support 与 finger bilateral 同时为真分别说明什么？为何还不能算 place 完成？
  4. 为什么下降目标使用 measured object-to-gripper offset，而不是固定 gripper xyz？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已运行并验证 3 s 默认与 2 s 修改实验。Explain 确认：relative slip 会随持物时间累积，
不只取决于瞬时速度；bilateral contact 只说明左右手指仍接触物体，不能说明 object 没有相对
gripper 滑动；完整 place 要求机器人释放并退出后物体仍能在支撑面稳定保持；grasp 后
object-to-gripper offset 会受接触、滑移和 tracking 影响，下降目标应使用实测值。补充证据
拆分：ground support 表示物体已由支撑面承载，finger bilateral 表示夹爪尚未释放；还需
S11.6b 验证张爪、手指接触消失、撤离、最终位姿和低速度。S11.6a Learning Mastered。

## S11.6b — 支撑后释放并撤离（Engineering Complete）

### Problem → Why

S11.6a 终态有 ground support，但 fingers 仍夹着物体。本课完成顺序约束：先确认支撑，再
张爪；确认左右 finger–object contacts 都消失后，才沿 local `-z_G`（本例 world `+z`）
撤离。最终联合检查物体位姿、支撑、无手指接触、开口、撤离距离和低线/角速度。

### Intuition and core concepts

“碰到桌面”不是 place success。释放太早会掉落，接触未消失就撤离可能拖动物体，只看最终
位置又可能漏掉仍在运动。事件顺序为：

```text
supported + bilateral -> open -> supported + no finger contact -> retreat -> final checks
```

- release event 需要 ground–object 存在且左右 finger–object 都不存在。
- retreat 期间 finger-contact steps 和 unsupported steps 都必须为 0。
- freejoint 的 linear `(m/s)` 与 angular `(rad/s)` qvel 分开检查，避免混合单位。
- 最终 opening 至少 `0.07 m`，证明实际张开而不只是写入 open `ctrl`。

### Mathematics → code

top-down approach axis `a_W=R_WG e_z=(0,0,-1)`，所以 retreat direction
`r_W=-a_W=(0,0,1)`。实际撤离投影为

\[
d_{retreat}=r_W^T(p_{G,f}-p_{G,release}),
\]

要求不低于 requested distance 减 `0.01 m` tracking allowance。最终要求

\[
\|p_{O,f}-p_O^*\|\le0.015\ \mathrm{m},\quad
\|v_O\|\le0.01\ \mathrm{m/s},\quad
\|\omega_O\|\le0.1\ \mathrm{rad/s}.
\]

### Minimal experiment

[release_and_retreat.py](../examples/12_pick_place/release_and_retreat.py)先调用 S11.6a helper
重建并断言 supported/closed state：

```bash
python examples/12_pick_place/release_and_retreat.py
python examples/12_pick_place/release_and_retreat.py --retreat-height 0.12
```

### Expected / actual result

2026-10-04 在核验后的 WSL2、`mujoco` conda、Python 3.12.14、MuJoCo 3.14.0 运行：

- 两组先通过完整 S11.6a preconditions；张爪第 5 step、仿真时间 `10.010 s` 检测到
  supported 且 finger contacts 消失，当时 opening `0.042585 m`。
- requested retreat `0.08/0.12 m`，actual projection `0.076992/0.116675 m`。
- 两组 retreat 中 finger-contact steps=0、unsupported steps=0。
- 最终 object `[-0.297783,-0.100246,0.029363] m`，position error norm `0.002320 m`；
  opening `0.080000 m`。
- 最终 linear speed 约 0、angular speed 约 `1e-9 rad/s`；contact set 仅 ground–object。
- 两条命令退出 0，全部断言通过。

### Explanation and failure cases

- release 前不确认 support，会把放置退化为空中松爪。
- 只写 open ctrl 不证明释放；必须等待 actual opening 与 finger contacts 消失。
- contact 消失前 retreat 可能拖拽或弹射物体，因此有明确 event gate。
- final pose 合格但速度大，物体可能随后离开容差；必须检查低末速度。
- support-only contact 证明载荷已转移，但只针对当前 nominal 参数，不证明 robustness。

### Robotics context and interview capsule

真实系统常用支撑力或末端载荷变化确认 load transfer，再开启夹爪和执行 retreat。面试时可
概括：我用事件门控保证 support-before-release、contact-separation-before-retreat，最后联合
检查 object pose、support-only contact、finger clearance 和低 twist，避免把瞬时接触或
正确位置但仍在运动误判为成功。

### Must remember

1. support、release、retreat、settled final state 是四个顺序阶段。
2. `ctrl=open` 不等于实际释放，必须检查 opening 与 contact identity。
3. final pose 与 final velocity 必须联合判断。
4. nominal 完成不等于 robustness；S11.7 才做固定-seed 小范围变化。

### Run / Modify / Explain handoff

- **Run**：运行默认命令，确认 support precondition、release step、80 mm retreat、无重新接触、
  final pose/support/opening/velocity 和 PASS。
- **Modify**：运行 `--retreat-height 0.12`，确认物体终态基本不变而 clearance 增大。
- **Explain**：回答：
  1. 为什么必须在张爪前确认 ground support？
  2. 为什么写入 open ctrl 后还要等待 finger contacts 消失才能 retreat？
  3. 为什么 final pose error 很小仍必须检查 object linear/angular velocity？
  4. 最终只有 ground–object contact、无 finger contact，分别排除了哪些失败模式？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已完成默认 `0.08 m` 与修改 `0.12 m` retreat 实验并分析数据。Explain 确认：张爪会
撤销夹持力，必须先确认环境支撑以避免自由落体或失稳；open command 发出不代表物理动作
完成，必须等 actual contacts 消失；pose 描述当前状态，velocity 描述后续运动趋势，二者要
联合判断；ground–object contact 排除悬空、失去支撑和弹离等失败。补充证据拆分：无
finger contact 独立排除夹爪仍夹持、单侧挂住或 retreat 时拖拽物体；二者联合证明载荷已从
机器人转移到环境。S11.6b Learning Mastered，完成一次 nominal known-pose pick-and-place。

## S11.7 — 固定 seed 小范围鲁棒性实验（Engineering Complete）

### Problem → Why

单次 nominal 成功不能说明对小建模误差是否敏感。本课对 initial object xy、sliding friction
和 mass 做有限范围变化，固定 seed 运行 20 次完整 S11.1–S11.6b 流程，统计 success rate、
failure stage 和成功 trial 的 final position error。实际 sampled xy 仍提供给 planner，所以
仍是 known-pose manipulation，不引入感知误差。

### Intuition and core concepts

固定 seed 使参数序列可复现。每个 trial 完整执行抓取、抬升、transfer、descent、release、
retreat；失败按 `TRANSFER_DESCENT` 或 `RELEASE_RETREAT` 分类。

默认范围：initial x/y 各自在 nominal 周围 `±0.005 m`，friction `[1.8,2.2]`，mass
`[0.045,0.055] kg`，20 trials，seed `20261004`。

### Mathematics → code

\[
\hat p=N_{success}/N_{trials},\qquad
\bar e=\frac{1}{N_{success}}\sum_i e_i,\qquad e_{max}=\max_i e_i,
\]

其中 `e_i=||p_{O,f}^{(i)}-p_O^*||` 只对成功 trial 统计。失败 trial 进入 stage counter，
不混入成功误差均值。`20/20` 是样本观测，不是总体成功概率必为 100% 的证明。

### Minimal experiment

[robustness_trials.py](../examples/12_pick_place/robustness_trials.py)：

```bash
python examples/12_pick_place/robustness_trials.py
python examples/12_pick_place/robustness_trials.py --seed 7
```

### Expected / actual result

2026-10-04 在核验后的 WSL2、`mujoco` conda、Python 3.12.14、MuJoCo 3.14.0 运行：

- seed `20261004`：20/20，rate `1.000`，failure stages `{}`；successful final error mean
  `0.002299761 m`，max `0.002344323 m`。
- seed `7`：20/20，rate `1.000`，failure stages `{}`；mean `0.002302795 m`，max
  `0.002346447 m`。
- 每个 trial 打印 sampled parameters、PASS/FAIL、failure stage 或 final error；两条命令
  均退出 0，accounting、至少一次成功和 finite error 断言通过。

### Explanation and failure cases

- 空 failure-stage dict 只表示本批次无失败，不表示分类覆盖所有未来故障。
- 更换 seed 改变样本序列，但仍来自同一参数分布。
- planner 使用 sampled xy，因此未测试 pose estimation/calibration error。
- 未变化 orientation、尺寸、控制延迟、噪声或外部扰动。
- 20 个样本不足以对低失败概率给出强统计保证，也没有做最坏情况搜索。

### Robotics context and interview capsule

工程上可先用小范围 fixed-seed sweep 建立可复现回归，再逐步扩大 uncertainty model。面试时
可概括：我对 known xy、friction、mass 做 20 次完整任务 sweep，按 pipeline stage 记录失败
并统计成功终态误差。两个 seed 都 20/20、mean error 约 2.3 mm；结论严格限定在当前范围，
不称为复杂 domain randomization 或 sim-to-real 证明。

### Must remember

1. 固定 seed 保证可复现序列，不保证结果普适。
2. success rate 必须同时报告 trials、参数范围和成功判据。
3. failure stage 比单一失败总数更利于定位 pipeline 薄弱环节。
4. known-pose variation 与 perception uncertainty 是不同问题。

### Run / Modify / Explain handoff

- **Run**：运行默认 20 trials，确认参数范围、20/20 accounting、failure stages 和误差统计。
- **Modify**：运行 `--seed 7`，比较参数序列和 error statistics，说明为何仍是同一分布。
- **Explain**：回答：
  1. 为什么 fixed seed 有助于回归调试，却不能证明普适鲁棒性？
  2. 为什么 `20/20` 不能解释为真实成功概率必然是 100%？
  3. 为什么 sampled xy 必须传给 known-pose planner？本课因此没有测试什么误差？
  4. failure stage 与 final error 分别回答什么？为何失败 trial 不混入成功误差均值？

### Status

- Engineering: [x] Code [x] Experiment [x] Docs
- Learning: [x] Run [x] Modify [x] Explain — Mastered

本人已完成默认 seed 与 seed 7 两组 handoff。Explain 确认：fixed seed 解决同一实验能否
重复，而 robustness 还取决于覆盖范围；20/20 只是有限样本的 empirical success rate，用于
估计而不等于真实概率必为 100%；sampled xy 传给 planner 是让系统针对已知真实位置重新
规划，从而测试 manipulation，而不测试 pose estimation、标定或观测噪声；failure stage
回答 pipeline 在哪里失败，final error 回答成功后放得多准，部分失败 trial 根本没有有意义
的 final place error，不能混入成功精度均值。S11.7 Learning Mastered。
