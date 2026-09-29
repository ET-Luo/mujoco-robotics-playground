# What I Need to Understand

S8.1 于 2026-09-29 开始：把已有平面两关节 Reach 写成任务数据接口，先明确
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

待本人完成三项纸面检查：列出 observation 形状与各段单位；距离 0.006 m 时算 reward
并判断 terminated；第 100 步距离 0.012 m 时判断 terminated/truncated。尚未实现接口、
运行 Python/MuJoCo 或安装 RL 依赖，S8.1 未勾选。

# What I Learned

# Interview Questions

1. observation 为什么需要包含目标，混合单位有什么后续问题？
2. action 是关节位置、速度还是力矩，单位与范围是什么？
3. terminated 与 truncated 分别表示任务事件还是时间预算？
