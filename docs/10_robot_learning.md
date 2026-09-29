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

# What I Learned

# Interview Questions

1. observation 为什么需要包含目标，混合单位有什么后续问题？
2. action 是关节位置、速度还是力矩，单位与范围是什么？
3. terminated 与 truncated 分别表示任务事件还是时间预算？
