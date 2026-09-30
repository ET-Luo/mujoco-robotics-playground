# What I Need to Understand

S8.1 于 2026-09-29 完成：把已有平面两关节 Reach 写成任务数据接口，先明确
observation、action、reward、terminated 与 truncated，不安装 Gymnasium 或训练算法。

# Key Concepts

本课 observation 定义为长度 8 的一维数组：
`[q1,q2, qvel1,qvel2, ee_x,ee_y, target_x,target_y]`。
关节角单位 rad、速度 rad/s、末端与目标位置 m。固定目标理论上可不重复观察，但显式
包含 target 能让接口以后支持目标变化。混合单位暂不归一化；这是后续工程问题。

action 定义为长度 2 的关节速度命令 `(qdot1_cmd,qdot2_cmd)`，单位 rad/s，每项限制在
[-0.2,0.2]。假定控制周期 0.02 s，几何更新为 `q_next=q+action*0.02`；它仍不是已验证的
MuJoCo 实际 qvel 跟踪。

reward 先用最小密集奖励 `reward=-||target_xy-ee_xy||_2`，为单个标量；距离越小，reward
越接近 0。暂不加入成功 bonus、动作惩罚或碰撞惩罚，以便先检查方向是否正确。

结束条件分开：`terminated = distance < 0.01 m` 表示任务成功；`truncated = step_count >= 100`
且尚未成功，表示时间上限。成功是任务状态，截断是外部预算，不能混为一项。

# Experiments

本人完成三项纸面检查：正确写出 observation 形状 `(8,)`，并将 q、qvel、末端/目标
位置的单位依次写为 rad、rad/s、m、m；距离 0.006 m 时正确得到 reward=-0.006、
terminated=True；第 100 步距离 0.012 m 时正确得到 terminated=False、truncated=True。
尚未实现接口、运行 Python/MuJoCo 或安装 RL 依赖；S8.1 已勾选。

S8.2 于 2026-09-29 开始。已说明 Gymnasium 在本步只提供标准 `Env`、`Box` 和 seed
处理，将其加入 requirements 并安装到核验后的 `mujoco` 环境。`reach_env.py` 已包含
固定初态、平面 FK、8 维 observation space 和 reset 状态更新；observation 的四段数组
拼接由本人正确填写。2026-09-29 在核验的 `mujoco` 环境运行 reset 检查：返回
`[0,0,0,0,0.7,0,0.5,0.2]`，shape `(8,)`、dtype float64，space 包含检查通过；
人为修改 q/qvel/step_count 后再次 reset 也恢复初态。待本人解释 reset 返回值与为何
末端位置应由 FK 重算；尚未实现 step 或训练算法。

本人随后正确解释 observation 是智能体实际看到的输入，info 是额外信息；补充说明
policy 通常不使用 info。本人也正确解释 `end_effector_xy` 不是独立状态，而是由当前 q
通过 FK 得到的派生量，不能沿用上一回合的值。S8.2 已勾选。

S8.3 开始。本人正确预测动作 `(0.3,-0.1)` rad/s 被限幅为 `(0.2,-0.1)` rad/s，
并得到一步后的 `q=(0.004,-0.002)` rad；也正确区分第 100 步未成功时的截断与
同一步成功时的任务终止。本人随后正确填写动作限幅、速度状态、角度积分、距离奖励、
`terminated` 和 `truncated` 表达式，助手将其集成到 `reach_env.py`。

2026-09-29 在 WSL2 与核验后的 `mujoco` 环境运行 Gymnasium 1.3.0：环境检查通过，
仅提示 observation space 使用无穷上下界；边界动作实测 `qvel=[0.2,-0.1]` rad/s、
`q=[0.004,-0.002]` rad，observation 仍在 space 内。短随机动作回合在第 100 步以
约 0.295914 m 距离得到 `terminated=False, truncated=True`。另将目标设为当前末端
并从第 99 步推进，实测第 100 步为 `terminated=True, truncated=False`，验证成功优先。
本人正确解释随机回合用于检查 `step` 逻辑，并不要求随机策略完成任务；也正确指出真正
的 `data.qvel` 是物理引擎根据动力学计算的状态，会受执行器、惯性、重力、接触、约束
和数值积分等因素影响，而这里的 `self.qvel` 只是直接赋值的几何命令。S8.3 已勾选。

S8.4 本人比较 `-d` 与 `-(d^2)`：在 0.02/0.20 m 时正确得到线性奖励
-0.02/-0.20、平方奖励 -0.0004/-0.04；从 0.20 m 改善至 0.02 m 时，两者分别提高
0.18 和 0.0396。两者都偏好更小距离。平方形式使远/近奖励绝对值之比从 10 变为 100，
但因当前距离小于 1 m，其奖励绝对值及本次改善量反而更小，不能只凭“平方”断言惩罚
绝对更强。新增 `reward_comparison.py`，在核验后的 `mujoco` 环境运行，打印值与四项
单调性/数值断言均通过。S8.4 已勾选；未训练策略或运行 MuJoCo/GUI。

S8.5 先梳理 PPO 数据流：当前策略与环境交互收集 rollout，其中保存 observation、action、
reward、value、旧动作概率和结束标志；固定这批数据后计算 critic target/advantage，再用
有限轮次的小批量更新 policy 与 critic。本人对非终止单步样本 `r=-0.05`、`gamma=0.99`、
`V(s)=-0.20`、`V(next)=-0.10`，正确得到 critic target=-0.149、advantage=0.051，
并解释正 advantage 会推动动作概率提高。

本人也正确完成两种 PPO 裁剪手算。正 advantage=0.051 时，概率 0.20→0.26 得到
ratio=1.30、clipped ratio=1.20、未裁剪/裁剪目标 0.0663/0.0612，最终取 0.0612。
负 advantage=-0.05 时，概率 0.20→0.14 得到 ratio=0.70、clipped ratio=0.80、
未裁剪/裁剪目标 -0.035/-0.04，最终取 -0.04。本人正确解释动作表现较差时应降低其
概率，但裁剪不让单批旧数据持续奖励越界的大幅更新。S8.5 已勾选；PPO 实现保持 TODO，
未安装训练依赖、运行 Python/MuJoCo/GUI 或声称策略已训练。

S8.6 梳理 SAC 的 replay、双 critic 与 entropy。本人对非终止样本 `r=-0.05`、
`gamma=0.99`、两个 target Q 为 -0.20/-0.10、`alpha=0.10`、`log_pi=-0.50`，
正确得到较小 Q=-0.20、entropy 修正后的下一状态值 -0.15，以及 critic target=-0.1985。
本人正确解释 actor 会利用 critic 偶然高估的动作，取两个 critic 的较小值用于保守约束，
不是挑选“更好的 critic”。

本人也正确区分：SAC 是 off-policy，可从 replay buffer 反复利用旧策略 transition；PPO
基本是 on-policy，旧 rollout 与当前策略差异过大后不能长期复用。`alpha` 增大表示更重视
entropy，通常鼓励探索并使动作分布更分散。S8.6 已勾选；SAC 的 replay buffer、网络、
优化器和训练循环均保留 TODO，未安装训练依赖或运行 Python/MuJoCo/GUI。

# What I Learned

# Interview Questions

1. observation 为什么需要包含目标，混合单位有什么后续问题？
2. action 是关节位置、速度还是力矩，单位与范围是什么？
3. terminated 与 truncated 分别表示任务事件还是时间预算？
4. 为什么随机动作回合超时不能说明任务或学习算法实现错误？
5. 几何积分中的命令速度与动力学仿真的实际关节速度有什么区别？
6. PPO rollout 需要保存哪些量，为什么更新期间把它视为固定的旧策略数据？
7. advantage 的正负如何影响动作概率，clipped objective 如何限制单次变化？
8. SAC 为什么能复用 replay 中的旧数据，而 PPO 通常不能长期复用旧 rollout？
9. 双 critic 的较小值和 entropy 系数分别解决什么问题？
