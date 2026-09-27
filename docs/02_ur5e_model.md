# What I Need to Understand

S1.3：关节名称如何对应 qpos/qvel，nq/nv/nu 分别计数什么。
状态：2026-09-27 预测、实际读取与解释均完成，S1.3 已勾选；GUI 状态不因本实验改变。

S1.4：2026-09-27 元素问答和六组执行器—关节映射均完成，已勾选。
本步为官方 XML 阅读与预测，没有修改模型或运行新实验。

# Key Concepts

- joint 定义允许的相对运动；actuator 把控制输入转换为驱动力，本身不增加关节自由度。
- 官方 XML 的 `<general class="size3" name="shoulder_pan" joint="shoulder_pan_joint"/>`
  定义执行器 shoulder_pan，并通过 joint 属性连接 shoulder_pan_joint；不要靠相似名称推断映射。
- `<general>` 的行为由参数决定。此模型 size3 继承固定增益与仿射偏置配置，形成位置伺服，
  因而 ctrl 是目标角度。当前只识别配置含义，不推导控制公式，也不实现控制器。
- 没有执行器的关节仍可能受重力或其他物理作用运动；此处 actuator 不是关节本身。

- body 是刚体及其局部坐标系；joint 定义相对父刚体的运动自由度；geom 描述附着的形状。
- site 是固定在所属 body 上的参考位置和方向，不增加自由度，也不参与碰撞。
- 本模型 `attachment_site` 属于 `wrist_3_link`，`pos="0 0.1 0"` 是相对该刚体
  局部坐标系的位置，单位米；不是固定的世界坐标。`quat` 定义局部朝向，本步不推导。

- nq、nv、nu 分别是位置坐标、速度坐标、执行器控制输入数量。
- `model.joint(joint_id).name` 用关节 ID 读取名称。
- `model.jnt_qposadr[joint_id]` 返回该关节在 qpos 中的起始索引。
- `model.jnt_dofadr[joint_id]` 返回该关节在 qvel 中的起始索引。
- 以上读取均不改变仿真。当前 UR5e 每个关节有一个角度和角速度，单位为 rad、rad/s。
  此模型中 ID 与地址碰巧相同，但代码应读取地址；更一般的关节可能占多个元素。
- 控制输入不能仅按同序关节推断映射，本次不扩展到执行器控制。

# Experiments

在 mujoco 环境运行：

```bash
python examples/02_ur5e_basics/simulation_pipeline.py --headless --steps 1000
```

[代码](../examples/02_ur5e_basics/simulation_pipeline.py)现在打印名称与两种状态地址，
并在最终状态逐关节打印角度和角速度。
助手验证（2026-09-27）：检查 conda/解释器后运行，退出 0，无导入错误；
time=2.000 s、nq=nv=nu=6，六个关节映射及状态均打印。
未复测 GUI，也未修改物理循环或控制命令。

| 关节 | qpos 索引 | qvel 索引 |
| --- | --- | --- |
| shoulder_pan_joint | 0 | 0 |
| shoulder_lift_joint | 1 | 1 |
| elbow_joint | 2 | 2 |
| wrist_1_joint | 3 | 3 |
| wrist_2_joint | 4 | 4 |
| wrist_3_joint | 5 | 5 |

本人提供的 elbow_joint 最终输出（2026-09-27）：qpos 索引 2，qvel 索引 2；
`qpos[2]=6.66674796e-03 rad`，`qvel[2]=-4.20118884e-06 rad/s`。
本人正确解释其角度为正、此刻角度正在减小。未报告新问题；本次助手仅记录反馈，未重跑。

# What I Learned

S1.4 首步本人已正确回答：多个 visual geom 不代表多个关节，它们随所属刚体运动；
删除例子中 shoulder_1 的 visual geom 不会删除 shoulder_pan_joint，只减少显示外观。
这是基于原 XML 的阅读预测，并未实际删除模型元素。

site 问答（2026-09-27）：本人正确回答增加 site 不增加关节；所属刚体运动时，
site 相对刚体的局部位置不变，世界位置可能改变。此为概念预测，尚未做位置读取实验。

本人已正确预测：nq 是位置坐标数量；UR5e qvel 的单位为 rad/s；
只有六个转动关节、三个控制输入的假设模型中 nq=6、nv=6、nu=3。
本人读取实际映射后回答（原话）：

> 位于角度的正方向，此刻角度在减小

助手反馈：正确区分了位置符号与速度符号；当前是在朝零角度靠近，
但不能仅由此推断之后一定越过零点。S1.3 完成。

actuator 基础问答（2026-09-27）：本人正确辨认 shoulder_pan 是执行器、
shoulder_pan_joint 是关节；删除执行器不删除关节自由度。本人解释无此执行器时
仍可能“因为其它关节执行器的指令，被带动”。助手确认这是可能原因；需区分
刚体在世界中随上游运动与该关节自身相对角度变化，前者不必然意味着后者。
S1.4 的元素基础问答已通过；本人随后正确列出全部六组映射：

| 执行器 name | joint 属性指向的关节 |
| --- | --- |
| shoulder_pan | shoulder_pan_joint |
| shoulder_lift | shoulder_lift_joint |
| elbow | elbow_joint |
| wrist_1 | wrist_1_joint |
| wrist_2 | wrist_2_joint |
| wrist_3 | wrist_3_joint |

依据为官方 XML 的 name/joint 属性，不将名称相似当成通用映射规则。
阅读任务完成，没有新增运行或 GUI 验证结论。

# Interview Questions

1. nq/nv/nu 和 qpos/qvel/ctrl 分别表示数量还是具体值？
2. 为什么不能直接把 joint ID 当成所有模型的状态索引？
3. jnt_qposadr 和 jnt_dofadr 分别解决什么问题？
4. 六个转动关节是否必然有六个控制输入？
