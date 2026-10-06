# Stage 14 — Motion Planning

S14.1 已实现：[代码](../examples/15_motion_planning/collision_checking.py)、[运行入口](../examples/15_motion_planning/README.md)。
Engineering / Learning 唯一状态见[根 README](../README.md#stage-14--motion-planning)。S14.2已实现，见[Configuration Space / Edge Checking](17_2_configuration_space.md)；S14.3及以后未开始。

## Problem → Why

IK 收敛只说明末端能够到达目标，并不检查机械臂、环境或手中物体是否相交。
本课构建 `valid(q, phase, held)`：候选姿态是否符合关节范围和接触政策？
每次查询要隔离实际执行状态；否则规划器试错会把正在执行的机器人“瞬移”。

## Intuition → Core Concepts

想象把机器人复制到一张计算用草稿纸上：在副本上摆姿态，再询问哪些几何体接近或相交。
共享只读 `MjModel`，每次创建独立 `MjData`。本课有意用3个滑动关节+1个yaw关节的小夹爪，
球体的穿透可手算，避免UR5e mesh/exclusions掩盖判定逻辑；后续S14.7才接入UR5e。

- self：base–palm 等非同体机器人几何碰撞。
- robot_environment：机器人与ground/obstacle。
- robot_object：包括pad–object，也包括绝不放行的palm–object。
- object_environment：物体与ground/obstacle；带物时必须更新物体再检查。

| phase | pad–object | object–ground | 其他触碰 |
| --- | --- | --- | --- |
| approach | 拒绝 | 允许浅接触（待抓物体可在地上） | 拒绝 |
| grasp | 允许浅接触 | 允许浅接触 | 拒绝 |
| transfer | 允许浅接触 | 拒绝 | 拒绝 |
| place | 允许浅接触 | 允许浅接触 | 拒绝 |

允许某对接触仅表示该碰撞政策接受，不表示双侧抓稳、足够支撑力或摩擦稳定。
本课place允许浅支撑接触，但不会自动开放整个object或robot类别。

## Mathematics：意义、形状、单位、frame

`q=[x,y,z,yaw]`，shape `(4,)`，前三项是world轴平移m，末项是绕world +z的rad。
工具初始world位置 `[0,0,0.5] m`；前三个joint range为 `[-.5,.5],[-.5,.5],[-.49,.5] m`，
yaw为 `[-3.14,3.14] rad`。这不是UR5e六个rad关节。

固定抓持假设：

```text
T_GO = [I, [.06,0,0]ᵀ; 0,1]     # O→G，物体中心位于工具+x 60 mm
T_WO(q) = T_WG(q) T_GO
p_WO = p_WG + R_WG p_GO
```

矩阵shape `(4,4)`，rotation无量纲，translation米，结果在world frame。
本课物体是球，旋转不改变它的碰撞外形，但仍正确更新完整freejoint姿态。
没有weld，也不模拟真实抓持；固定T_GO忽略滑移和感知误差。

对接触记录i，`d_i=contact.dist` 是有符号表面距离m：正值分离，零触碰，负值穿透。
非许可对要求 `d_i>0`；许可对要求 `d_i >= -δ`（δ默认6 mm）。
有效姿态还须满足所有关节范围。深度门限是教学几何政策，非安全余量或硬件推荐。
球–球的独立核对式：`d=||c1-c2||-(r1+r2)`；球–地面：`d=z_center-r`。

## Math-to-Code / API

按代码顺序阅读 `build_model → allowed_pair → check_configuration → main`。

| API / 字段 | 输入与输出/原地更新 | 本课作用 |
| --- | --- | --- |
| `MjModel.from_xml_string(xml)` | MJCF文本→编译后的MjModel | 保存几何、关节地址/范围与碰撞设置 |
| `MjData(model)` | model→独立可写状态与cache | 与live隔离；本课只复制snapshot.qpos，速度/control不用于几何查询 |
| `query.qpos` | 本模型shape `(11,)`：4关节+freejoint的3位置/4四元数 | 设置候选关节；物体坐标以world米表示，四元数wxyz无量纲 |
| `mj_forward(model, query)` | 返回None；原地更新FK、contacts及动力学cache | 第一次获得工具frame；持物更新后第二次重建碰撞；不推进time |
| `site_xpos/site_xmat` | `(nsite,3)` / `(nsite,9)`，world位置/行优先旋转 | 获得工具的T_WG |
| `mju_mat2Quat(out, mat)` | `(9,)`旋转→写入 `(4,)` wxyz buffer | freejoint旋转赋值 |
| `query.contact` | 当前ncon条记录；geom ID、dist等 | 按确切pair与phase解释几何结果 |

`qpos`直接赋值只用于查询副本。这里没有ctrl或mj_step执行轨迹，也没有接触力验证。
query.time为0；live.time保留1.25 s，live.qpos/qvel/xpos都保持不变。

**接触列表有模型边界**：本课geom margin/gap均10 mm，因此能看到正距离记录，
不同组合的有效margin由MuJoCo确定，不将其当作触碰。真实模型可能因contype/conaffinity、
同体/父子过滤或exclude而不生成某些pairs。不要用“ncon=0”断言所有物体间有全局安全距离。
政策拒绝只看实际生成的接触；不做全局最近距离或continuous collision detection。
MuJoCo mesh碰撞通常使用凸包，外形近似也影响结果。
以上语义参见[官方Collision Detection](https://mujoco.readthedocs.io/en/stable/computation/#collision-detection)
和[官方Simulation](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)。

## Minimal Experiment → Expected / Actual Result

从项目根目录、已激活mujoco环境执行运行入口中的两条脚本命令。
2026-10-06，DESKTOP-781D67A现场验证：Python3.12.14、NumPy2.5.3、MuJoCo3.14.0。
13个确定性用例；默认接受6/13，δ=1 mm接受1/13，两命令exit0，无import error。

| 用例 | 默认6 mm | 改为1 mm | 几何原因 |
| --- | --- | --- | --- |
| clear | 接受 | 接受 | 无接触 |
| pads_approach / pads_grasp | 拒绝 / 接受 | 拒绝 / 拒绝 | pad穿透5 mm，phase与深度分别起作用 |
| self_collision | 拒绝 | 拒绝 | base–palm穿透60 mm |
| robot_obstacle | 拒绝 | 拒绝 | palm–obstacle穿透30 mm |
| held_clear / held_rotated | 接受 / 接受 | 拒绝 / 拒绝 | 物体跟随工具；pad穿透5 mm |
| held_obstacle | 拒绝 | 拒绝 | object–obstacle穿透20 mm；pad与障碍仍分离 |
| ground_transfer / ground_place | 拒绝 / 接受 | 拒绝 / 拒绝 | ground–object穿透0.5 mm；1 mm时pad先不合格 |
| deep_place | 拒绝 | 拒绝 | object穿透10 mm，同时palm触地 |
| joint_limit | 拒绝 | 拒绝 | x=.6 m，查询前拒绝 |
| near_obstacle | 接受 | 拒绝 | object–obstacle分离5 mm；1 mm时pad不合格 |

JSON包括每条contact的pair、类别、signed distance、许可与violation，能定位拒绝原因。
所有产物在ignored tmp目录，不需要Matplotlib、OpenCV、Menagerie或RL包。
3.11兼容目标未在3.11执行；没有GUI/动力学/UR5e/连续路径验证。

## Explanation → Failure Cases

- 少检查held object：夹爪看似绕过障碍，物体仍可能撞。`held_obstacle`正是这个反例。
- 白名单过宽：允许抓取接触不应放行palm–object、finger–ground或object–obstacle。
- 只看ncon：正margin生成的分离记录不是穿透；可接受的pad接触也让ncon非零。
- 只看pair：同一pair浅接触可许可，过深仍拒绝；更严格门限可能把所有持物姿态判为不可行。
- 不重新forward：qpos修改后旧FK/contacts不代表新姿态。held更新需要第二次forward。
- 共享data/修改model：规划查询会污染执行或其他候选；本课每次新建data。
- 把valid点当valid边：两端不撞不保证中间不撞，S14.2再学习分辨率与edge checking。
- 浮点边界：精确相切易受roundoff影响，门限附近要制定一致政策；本课不声称鲁棒物理安全。

## Robotics Context → Interview Capsule

应用：装配接近、抓取接触、携物转移、放置支撑。同一几何接触因任务阶段具有不同意义。
实际UR5e整合需审计collision geometry/exclusions，并明确环境和持物估计；本课尚未做这一步。

**30秒**：碰撞检查是候选配置的可行性谓词。我用独立MjData写入q并forward，
根据具体几何pair、阶段和穿透深度决定接受。持物配置还需T_WG T_GO更新物体。
单点通过不保证路径、动力学或抓持稳定。

**2分钟**：先检查输入shape/有限值/关节范围，再复制几何状态并获得FK。
transfer/place按固定工具到物体变换更新freejoint后重算contacts。signed distance区分
分离与穿透；精确pair白名单允许pad–object和place支撑，同时限制许可穿透。
用自碰撞、机器人撞障碍、仅物体撞障碍、phase切换与严格深度反例验证。
数据隔离防止候选查询影响live；过滤和几何近似决定可检测范围，零接触不是全局证明。
后续规划器才会重复调用此类谓词检查边，并另外检查实际执行状态。

## Must Remember → Run / Modify / Explain

必须记住：完整机器人+物体几何；具体pair+phase+depth；独立data；forward更新cache；单点≠路径。

**Run**：运行默认命令，查看`held_obstacle`中物体负距离而pad–obstacle正距离。

**Modify**：先预测`--allowed-depth-m 0.001`影响，再运行对照。
解释为什么ground_place也失败，尽管ground–object仅穿透0.5 mm。

**Explain（五问）**：
1. 为什么候选查询需要独立MjData？共享MjModel有什么前提？
2. mj_forward与mj_step分别改变什么？持物查询为何forward两次？
3. dist为正/零/负各意味着什么？为什么ncon非零仍能接受？
4. T_WO=T_WG T_GO各变换的方向是什么？漏掉held object会怎样？
5. 为什么允许pad–object不证明抓稳，而valid起终点不证明路径可行？

**My Verification**：2026-10-06 本人明确确认实验与预测完成，并正确回答五项Explain，Learning Mastered；状态只在根README维护。
解释覆盖独立状态、forward与积分区别、持物两次cache刷新、signed distance、O→G变换和抓持/路径边界。
共享MjModel的前提是查询期间保持模型只读；许可接触仍受本课穿透深度门限约束。
