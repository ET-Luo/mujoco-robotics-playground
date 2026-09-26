# Stage 1 — Task 1: UR5e simulation pipeline

状态：示例已实现；本人已提供观察并通过核心概念问答。Headless 通过，GUI 正常退出仍待确认，整个 Task 暂不勾选。

## Concepts

- `MjModel` 是 XML 编译得到的世界设定：结构、质量、关节、执行器、重力、timestep。
  正常 step 不会自动改变这些设定，但程序可以显式修改其中允许修改的参数。
- `MjData` 是这一份仿真的当前状态与计算工作区：time、qpos、qvel、ctrl、力等。
  类比：model 是游戏规则，data 是当前存档；同一 model 可以配多份独立 data。
- UR5e 的 `qpos` 是六个关节角度（rad），`qvel` 是六个角速度（rad/s）。
- `ctrl` 是执行器命令。此模型自带位置伺服；默认零命令不是关闭执行器。
- `mj_step(model, data)` 计算物理作用并积分，原地更新 data：`t → t + dt`。
  `dt = model.opt.timestep`；它不是执行代码耗费的墙钟时间。

## Code

入口：[simulation_pipeline.py](../examples/02_ur5e_basics/simulation_pipeline.py)。
旧 `main.py` 保留作模型/执行器检查参考，本次先读这个简化入口。

| 首次用到的 API | 输入 → 输出或修改 | 为什么需要 |
| --- | --- | --- |
| Menagerie `get(...).xml("scene")` | 模型标识 → 官方 XML 缓存路径 | 获取机器人与地面模型及依赖资产 |
| `MjModel.from_xml_path(...)` | XML 路径 → MjModel | 编译物理世界定义 |
| `MjData(model)` | model → 独立 MjData | 分配与此模型匹配的状态 |
| `model.joint(id)` | 关节 ID → 该关节的模型视图 | 读取名称，对照状态输出 |
| `viewer.launch_passive(model, data)` | model/data → 窗口句柄 | 显示世界，物理仍由 Python 推进 |
| `mj_step(model, data)` | model/data → 原地更新 data，无新状态返回值 | 推进一个时间步 |
| `viewer.sync()` | 当前状态与窗口输入双向同步 | 显示运动；也可能接收控制面板修改 |

```text
官方 XML → MjModel → MjData
                        ↓
                mj_step → 打印 → sync
                    ↑               ↓
                    └── 下一次循环 ──┘
```

脚本每 250 步打印一次，另打印初始、第一步与最终状态。`sleep` 只让显示接近实时。
不使用 home keyframe，不赋值 ctrl；观察实验期间不要修改 GUI Control 或拖拽施力。

## Experiment

先激活 `mujoco`，检查 `echo $CONDA_DEFAULT_ENV` 和 `which python`，然后从仓库根目录运行：

```bash
# 默认 GUI，10,000 步，约 20 秒仿真时间；关闭窗口可提前结束。
python examples/02_ur5e_basics/simulation_pipeline.py
# 单独核对物理，不打开窗口：
python examples/02_ur5e_basics/simulation_pipeline.py --headless --steps 1000
```

助手实测（2026-09-26）：Python 3.12.14、MuJoCo 3.14.0、Menagerie 2026.9.1。
环境检查 PASS；headless 1,000 步退出码 0。`dt=0.002`，time 从 0 增长到 2.000 s；
初始 qpos/qvel/ctrl 均为零，末尾 ctrl 仍为零。`shoulder_lift_joint` 的角度约
0.025024 rad，角速度约 0.00000184 rad/s。其余状态也正常打印，无导入错误。
这是助手的运行记录，不代表本人已完成观察或理解。

GUI 单独运行：`timeout --kill-after=3s 20s python -u
examples/02_ur5e_basics/simulation_pipeline.py --steps 1000`。
完成 2.000 秒仿真和最终状态打印后，退出码 139，输出
`timeout: the monitored command dumped core`。这是进程异常终止，不是超时退出码 124；
未出现 Python traceback。故障时序指向 native viewer 退出路径，但不能据此确定是
MuJoCo、GLFW/OpenGL 还是 WSLg 的具体问题。没有新增图形依赖或 workaround。
用户此前确认 GUI 可见仍有效，本次没有视觉确认新窗口；正常退出尚未通过。

请先观察再解释，记录自己的结果：

1. 第一条推进后的 time 是多少？每隔 250 步的打印间隔是多少模拟秒？
2. 六个 qpos 是否都变化相同？挑一个关节比较初始、0.5 秒和 2 秒。
3. qvel 先变大还是先变小？接近零时，qpos 一定等于零吗？
4. ctrl 保持零，机器人是否完全静止？为什么不能直接称为“无驱动自由下落”？
5. 终端变化很小、肉眼看不明显，能否据此说没有仿真？

## What I Learned

以下根据本次学习对话记录，不是助手预填答案。

本人最终总结（原话）：

> 这次实验没有给 `ctrl` 赋值，但 `qpos` 仍然变化，是重力、执行器作用等共同影响的结果

本人已回答：第一步 time 为 0.002 s，因为 dt 为 0.002 s；MjModel 描述配置，
MjData 描述状态，mj_step 修改 MjData；角速度接近零不代表角度接近零。
本人观察第一个关节角度先变大再变小，提供的一个时刻 `qpos[0]` 为
`1.58300432e-07 rad`。单个快照不足以证明完整运动过程或振荡。

经问答纠正后，已确认：当前 UR5e 的 ctrl 是目标角度而不是角度增量；
希望保持 -0.2 rad 时，目标应为 -0.2 rad，而不是零。
正负方向依据关节轴定义，不能直接等同屏幕中的左右。

核心概念问答已通过；新的 GUI 正常退出证据尚未提供，不据此宣称程序验证全部通过。

## Common Misunderstandings

- model/data 分开不意味着 model 永远不可修改；它们承担的职责不同。
- `ctrl=0` 不等于关掉位置伺服，也不保证 `qpos=0`。
- `MjData(model)` 不会自动选择模型的 home keyframe。
- `viewer.sync()` 和 `sleep()` 不代替 `mj_step()`；当前脚本空格不暂停仿真。

## Interview Questions

1. MjModel 与 MjData 各存什么，为什么要分开？
2. UR5e 的 qpos 与 qvel 的单位和意义是什么？
3. mj_step 的两个输入是什么？它返回新状态还是修改原状态？
4. timestep 与程序墙钟耗时有什么区别？
5. 为什么不设置 ctrl，也可能出现运动和执行器作用？

验收：程序运行 + 本人观察 + 能解释上述核心对象/API，全部满足后才更新学习 checkbox。
下一小 Task（本任务验收后）：先预测再比较 500 与 1,000 步的时间和同一关节状态；尚未实施。
