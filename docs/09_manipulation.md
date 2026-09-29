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
待本人回答后创建最小 XML 和读取脚本。尚未运行 Python/MuJoCo/GUI，S7.1 未勾选。

# What I Learned

# Interview Questions

1. geom、body 和 contact 分别表示什么？
2. `data.ncon` 是模型固定参数还是随状态变化的数据？
3. 如何把 contact 中的 geom ID 转为可读名称？
