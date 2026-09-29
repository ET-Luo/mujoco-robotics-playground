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

# What I Learned

# Interview Questions

1. observation 为什么需要包含目标，混合单位有什么后续问题？
2. action 是关节位置、速度还是力矩，单位与范围是什么？
3. terminated 与 truncated 分别表示任务事件还是时间预算？
4. 为什么随机动作回合超时不能说明任务或学习算法实现错误？
5. 几何积分中的命令速度与动力学仿真的实际关节速度有什么区别？
