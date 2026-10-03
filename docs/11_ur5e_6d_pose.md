# UR5e 6D Pose

## Concept

S10.1 从已经完成的 UR5e site 世界位姿读取继续，只比较 position 与 orientation，
不发送控制命令。S10.2 再单独学习 orientation error；S10.3 扩展到 full Jacobian。

## Prediction

开始 S10.1 实验前由本人回答：

1. `site_xpos` 的 shape、单位和参考坐标系是什么？
2. `site_xmat` reshape 后的 shape 是什么？
3. 旋转矩阵的三列各自表示什么？
4. `site_xpos` 与 `site_xmat` 分别相对哪个坐标系描述 site？
5. 如果末端只改变朝向、不发生平移，site 原点的 position 是否一定改变？为什么？

2026-10-02 本人回答与校准：

- 正确指出 `site_xpos` 的 shape 为 `(3,)`、单位为 m，并在 world frame 中表示。
- 第一次将 `site_xmat` reshape 后的 shape 回答为 `(9,)`。这里需区分：
  `data.site_xmat[site_id]` 的原始存储 shape 是 `(9,)`；执行 `.reshape(3, 3)` 后是 `(3, 3)`。
- 正确解释 3×3 旋转矩阵的三列分别是 site 局部 +x/+y/+z 轴在 world frame 中的方向，
  这些列是无量纲单位向量。
- 正确指出 `site_xpos` 和 `site_xmat` 都描述 site 相对于 world frame 的位姿分量。
- 正确指出 position 不必因 orientation 改变而改变；但单个转动关节通常会同时改变两者，
  固定 site 原点的纯朝向变化一般需要满足机械臂运动学约束的多关节协同。

最小实验前，本人正确预测：`mj_forward` 不推进 `data.time`；模型中的局部 `site_pos`
不变；世界朝向会变；世界位置是否改变取决于 site 原点相对旋转轴是否存在垂直偏移。
若原点在轴上，转动可改变 orientation 而不改变 position。

## My TODO

- [x] 回答 S10.1 五个问题，并给出每项的 shape / unit / frame；reshape shape 经一次校准。
- [x] 运行前预测 `wrist_3_joint +0.1 rad` 时 position、orientation、局部定义与时间变化。

## Experiment

已新增 [`main.py`](../examples/09_ur5e_6d_pose/main.py)，复用
[`site_pose.py`](../examples/04_forward_kinematics/site_pose.py) 的 UR5e 加载、home reset、
`mj_forward` 和 `attachment_site` 查找方式。脚本比较 home 与直接增加 wrist_3 qpos
`0.1 rad` 后的完整世界位姿，并检查 joint axis 与 site 原点的局部几何关系；不引入 IK、
控制或 `mj_step`。尚待在核验环境后运行。

## Result

2026-10-02 助手在 WSL2 中激活并核验 `mujoco` 环境后运行：

```bash
python examples/09_ur5e_6d_pose/main.py
```

使用 `/home/lucas/miniconda3/envs/mujoco/bin/python`，Python 3.12.14、MuJoCo 3.14.0，
退出码 0，无导入错误。静态局部量为 wrist_3 axis=`(0,1,0)`、joint→site=`(0,0.1,0)` m，
site 原点到旋转轴的垂直距离为数值零。home 与 wrist_3 `+0.1 rad` 后的 `site_xpos` 都是
约 `(-0.133997825, 0.491999298, 0.488000367)` m，差为零；`site_xmat` 改变，局部
`site_pos` 不变，`data.time` 均为 0 s。脚本所有断言通过。

home 时 site +x/+y/+z 约朝 world +x/-y/-z；更新后 +x 与 +y 的世界方向改变，+z
基本保持 world -z。该结果待本人解释后再判断 S10.1 学习完成度；未运行 GUI 或动力学。

## Failure / uncertainty

本实验只验证一个特殊的 wrist_3 轴上 site 和一个 `0.1 rad` 位姿变化。它不能说明任意
关节转动都保持末端位置，也没有验证 actuator tracking、速度、受力或 6D 控制。

## What I learned

2026-10-02 本人解释：site 原点位于 wrist_3 旋转轴上，joint→site 向量与轴平行，
因此转动时原点不沿圆周运动，世界位置保持不变。旋转矩阵三列是 site 三根局部轴的
世界方向；与实际旋转轴对齐的 site 轴方向保持不变，另外两轴绕它旋转。不能只看到
joint XML 的局部 `axis=(0,1,0)` 就直接断定 site 的哪一列不变，必须考虑 joint frame
与 site frame 的坐标关系。

本人也正确限定：该结果依赖 attachment site 恰在 wrist_3 旋转轴上的特殊几何，不能
推广为任意 UR5e 关节转动都只改变 orientation。至此完成 S10.1，README 已勾选。

## S10.2 — Orientation Error（已开始）

目标是把 current orientation 到 target orientation 的差表示成一个三维 rotation vector，
而不是把两个 3×3 矩阵逐元素相减。先统一本课约定：`R_WC` 和 `R_WT` 分别把 current
与 target frame 中表示的向量转换到 world frame。相对旋转的乘法顺序决定误差向量在哪个
frame 中表达，本课先构造 world-frame error：

```text
R_error_world = R_WT * R_WC^T
```

它满足 `R_error_world * R_WC = R_WT`。随后把这个相对旋转写成 axis-angle：单位轴给出
世界系中的纠正方向，角度给出还需旋转多少 rad；二者相乘得到 shape `(3,)`、单位 rad 的
rotation vector。代码与 MuJoCo API 留到手算完成后。

### Prediction / hand calculation

令 current orientation 为单位矩阵，target orientation 为绕 world `+z` 旋转 `+90°`：

```text
R_WC = I
R_WT = [[ 0, -1, 0],
        [ 1,  0, 0],
        [ 0,  0, 1]]
```

请本人计算或判断：

1. `R_error_world = R_WT * R_WC^T` 等于什么矩阵？
2. 对应 axis-angle 的单位轴和角度分别是什么？
3. rotation vector 是什么，shape、单位和参考坐标系分别是什么？
4. 为什么 `R_WT - R_WC` 得到的 3×3 数组不能直接作为角速度式的三维 orientation error？

2026-10-02 首次回答：本人把 `R_error_world` 误答为单位矩阵；正确识别旋转轴为
world `+z`、角度为 90°，也正确给出 rotation vector 的 shape `(3,)`、单位 rad 和
world frame，但尚未写出三个分量。本人指出旋转矩阵不是普通三维向量，方向正确但需补全
矩阵差不表示旋转复合、不是合法旋转矩阵，且九个元素不能直接作为三维角速度式误差。

校准：因为 `R_WC=I`，所以 `R_WC^T=I`，从而
`R_error_world=R_WT I=R_WT`，不是单位矩阵。对应角度应以 rad 写作 `pi/2`；结合 world
`+z` 单位轴，rotation vector 应由本人补写为具体三维分量。S10.2 尚未完成。

2026-10-03 本人正确补充 rotation vector 为 `[0,0,pi/2]` rad（world frame），并正确
指出矩阵差不表示旋转复合、通常不是合法旋转矩阵；旋转矩阵虽有九个元素，但 SO(3)
约束使其只有三个自由度，九个差值不能直接作为三维角速度式误差。手算要求已完成。

新增 [`orientation_error.py`](../examples/09_ur5e_6d_pose/orientation_error.py) 最小骨架。
为保留核心推理，脚本尚有两处本人 TODO：

1. 用 `current_rotation` 和 `target_rotation` 写出 world-frame `relative_rotation`。
2. 从相对旋转矩阵写出顺序为
   `[R32-R23, R13-R31, R21-R12]` 的 NumPy 三维数组。

骨架随后用 `trace(R)` 计算角度、用 `2*sin(angle)` 恢复单位轴，再返回 axis×angle。
本课明确排除接近 0 与 pi 的数值分支；TODO 完成前不运行，也不勾选 S10.2。

本人随后正确提交两个核心表达式：`R_WT @ R_WC.T`，以及用 NumPy 零基下标提取
`[R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]`。助手仅将变量名对应到骨架的
`target_rotation`/`current_rotation`；待运行验证和结果解释后再判断 S10.2 完成度。

2026-10-03 助手在 WSL2 中激活并核验 `mujoco` 环境后运行：

```bash
python examples/09_ur5e_6d_pose/orientation_error.py
```

解释器为 `/home/lucas/miniconda3/envs/mujoco/bin/python`、Python 3.12.14。脚本退出 0，
得到 world-frame error=`[0,0,1.570796327]` rad、shape `(3,)`、norm=`1.570796327` rad，
与手算 `[0,0,pi/2]` 一致，断言通过。本脚本为 NumPy 几何检查，没有加载 MuJoCo 模型、
运行 `mj_step` 或 GUI。待本人解释乘法顺序、正号和 0/pi 数值边界后完成 S10.2。

本人最终解释：`R_WT @ R_WC.T` 定义 current→target 的 world-frame 相对旋转，所以本例
为绕 world +z 的 +90°，rotation vector 为 `[0,0,pi/2]`；标准轴提取含
`1/(2*sin(angle))`，在 angle 接近 0 或 pi 时分母趋近零，必须用特殊数值分支。
推理、核心 TODO、运行结果与边界解释均完成，S10.2 已勾选。

## Interview questions
