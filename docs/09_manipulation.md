# What I Need to Understand

S7.1 于 2026-09-29 开始：先在最小场景中辨认碰撞 geom 和一条接触记录，
不引入夹爪、控制器或抓取逻辑。

# Key Concepts

MuJoCo 的 `geom` 提供碰撞形状。接触是两个可碰撞 geom 在当前状态下满足接触条件时，
由碰撞检测生成的数据，不会新增 body、joint 或 actuator。

拟创建水平 plane 和半径 0.05 m 的自由球体。球心初始世界高度为 0.20 m，
球体最低点为 0.15 m，高于 z=0 的地面。重力使球体下落；球心接近 0.05 m 时，
球面最低点接近地面，预计开始产生接触。

`data.ncon` 是当前状态的接触数量。每次 `mj_step(model,data)` 推进时间并更新状态、
运动学、碰撞和接触；接触可通过 `data.contact[i]` 读取。`geom1`/`geom2` 是两个
geom 的整数 ID，可用 `mj_id2name` 转为名称；`pos` 是世界坐标系接触位置，单位 m。
接触数量和顺序可随状态变化，不应假定某对 geom 永远是 contact[0]。

# Experiments

预测题：初态是否已有接触；大约在球心何种高度首次接触；接触是否给模型增加关节。
本人正确预测初态 ncon=0、球心约 0.05 m 时首次接触、接触不会增加关节。

助手新增 [contact_scene.xml](../examples/08_contact_and_grasping/contact_scene.xml) 和
[main.py](../examples/08_contact_and_grasping/main.py)。XML 含 z=0 plane、半径 0.05 m/
质量 0.1 kg 的自由球体，初始球心 z=0.20 m，步长 0.002 s、重力 -9.81 m/s²。
脚本首次在本课使用：`MjModel.from_xml_path` 加载固定模型；`MjData` 保存动态状态和接触；
`mj_forward` 原地刷新派生量但不推进时间；`mj_step` 原地推进一步；`mj_id2name` 将 geom ID
转为名称。接触 `pos` 形状 (3,)，是世界位置 m；`dist` 是 geom 的有符号距离 m。

2026-09-29 核验 WSL2 Ubuntu 24.04.5，在同一 shell 激活并核验 mujoco 环境，
Python 3.12.14、MuJoCo 3.13.0。运行
`python examples/08_contact_and_grasping/main.py` 退出 0，无导入错误。
初态 time=0、ball_z=0.2 m、ncon=0。第 88 步 time=0.176 s 首次发现接触，
ball_z=0.049789280 m；geom 对为 ground/ball_collision，世界接触位置约
(0,0,-0.00010536) m，有符号距离 -0.00021072 m。轻微负值表示离散步进时已出现
小量几何穿入；接触位置位于两表面之间的接触参考位置，未必严格等于地面 z=0。
模型尺寸始终 nq=7、nv=6、njnt=1；freejoint 的姿态含 3 个位置和 4 元数共 7 个 qpos，
速度含 3 个线速度和 3 个角速度共 6 个 qvel。

以上是助手运行观察。待本人解释 `ncon` 是否固定、接触为何不改变 nq，以及首次球心
为何略低于 0.05 m 后完成 S7.1。未运行 GUI、未新增依赖。

本人正确回答 `data.ncon` 随当前状态变化；对接触不改变 nq 暂不清楚；将球心略低于
0.05 m 归因于数据精度误差和形变量。补充讲解：body/joint/freejoint 属于加载后固定的
`MjModel` 拓扑，决定 nq/nv/njnt；contact 是碰撞检测在当前 `MjData` 中生成的临时约束，
出现或消失都不会创建 joint。首次检测时的主要差异不是浮点舍入：仿真以 0.002 s
离散推进，球体可从上一个无接触状态跨到轻微穿入状态；MuJoCo 的软接触求解也允许
有限穿入来产生接触力。geom 仍是刚性几何，不能把该数值描述成已模拟真实材料形变。
待本人确认这两点后再勾选 S7.1。

本人确认 contact 是 MjData 中随状态变化的临时约束、不改变 MjModel 关节拓扑；
球心略低于半径主要来自离散步进和软接触允许的少量穿入。S7.1 已勾选。

## S7.2 最小平行夹爪（2026-09-29，关节与命令手算开始）

拟建立两个对称手指，各由一个 `slide` 关节沿相反的世界 x 方向向外移动。
slide 的 qpos/qvel 单位分别为 m、m/s，不是 rad、rad/s。两个关节范围均为
[0,0.04] m；q=0 表示各手指位于最内侧，q 增大表示向外打开。

设两手指最内侧表面的基础间距为 0.02 m，则开口宽度为
`opening = 0.02 + q_left + q_right`。两个位置执行器分别给各 slide 关节设置目标位移，
ctrl 单位为 m；修改 ctrl 不会瞬间修改 qpos，必须经过 mj_step 由执行器动力学响应。
本课先让两个目标相等以保持对称，不引入 tendon、夹持物体或接触逻辑。

待本人手算：q_left=q_right=0.01 m 时开口；从该状态把两个 ctrl 都设为 0.03 m 时，
命令是打开还是闭合；step 前 qpos 是否立即变为 0.03 m。之后再创建最小模型。
S7.2 未勾选，未运行 Python/MuJoCo/GUI。

# What I Learned

# Interview Questions

1. geom、body 和 contact 分别表示什么？
2. `data.ncon` 是模型固定参数还是随状态变化的数据？
3. 如何把 contact 中的 geom ID 转为可读名称？
