# What I Need to Understand

S4.4 于 2026-09-28 开始：位置 Jacobian 的行列、单位和世界参考系，
用 UR5e 单个 hinge 的有限差分核对一列。先理解和手算，不实现 IK 或控制器。

# Key Concepts

FK 给出 p=f(q)，位置 Jacobian 描述当前姿态附近的位置变化率。
对本例六个 hinge：Jp[i,j]=∂p_i/∂q_j，行对应世界 x/y/z，列对应关节速度自由度。
Jp 形状为 (3,nv)=(3,6)，不是按照执行器数 nu 构造；各列单位 m/rad。
小角度位移满足 Δp≈Jp*Δq；瞬时线速度满足 v_site=Jp*qvel，单位 m/s。
Jp 依赖姿态，不是旋转矩阵，不能用它直接计算 p=Jp*q，也不能把局部近似用于任意大变化。

只改变第 j 个关节：Δp≈Jp[:,j]*ε。某个元素为负表示该关节正向微小变化时，
对应世界位置分量减小；不表示关节本身在负转。
拟用 ε=1e-6 rad，分别读取 home 与 q_j+ε 的世界 site 位置，
用 (p_after-p_before)/ε 核对 home 处一列，称为前向有限差分。
不调用 mj_step，保持其他 qpos 不变；只做几何计算。ε 太大有线性近似误差，
太小会受浮点相减误差影响，不预先宣称越小越准确。

API 计划：先 mj_forward(model,data) 刷新，再分配 jacp=np.zeros((3,model.nv))，调用
`mujoco.mj_jacSite(model, data, jacp, None, site_id)`。
输入为模型、当前状态、位置 Jacobian 输出数组、可选旋转 Jacobian 输出数组及 site ID；
None 表示跳过旋转部分。函数原地填写 jacp，Python 返回 None，不推进时间。
以 model.jnt_dofadr[joint_id] 定位列，以 model.jnt_qposadr[joint_id] 定位待扰动角度；
这两个索引概念不同，不依赖它们恰好相同。
依据：[官方 mj_jacSite 与 mj_jac](https://mujoco.readthedocs.io/en/3.13.0/APIreference/APIfunctions.html#mj-jacsite)。

# Experiments

概念手算题（假设数值，不是 UR5e 实测）：某列为 (-0.5,0.2,0) m/rad，
只让对应关节增加 0.001 rad，预测末端世界位移的三个分量及单位。
本人正确给出 (-0.0005,0.0002,0) m，起初误说 x 增大；
解释增量符号后，本人正确计算原 x=0.3 时新 x=0.2995 m。

## 助手实验（2026-09-29）

新增 [main.py](../examples/05_jacobian/main.py)，读取 home 的完整 Jp，
只扰动 shoulder_pan_joint 的 qpos +1e-6 rad，forward 后计算位移与差分列，
最后恢复 qpos 和派生量。jacp 在 home 计算，比较时不覆盖为扰动后的 Jacobian。
检查 pwd/git status，保留已有修改；WSL2 Ubuntu 24.04.5，自动激活并核验 mujoco，
解释器 /home/lucas/miniconda3/envs/mujoco/bin/python，Python 3.12.14、MuJoCo 3.14.0。

```bash
python examples/05_jacobian/main.py
```

退出 0，无导入错误；Jp 形状 (3,6)，本关节 qpos 地址和 DOF 列索引均为 0。

```text
Jacobian column (m/rad): [-0.491999298, -0.133997825, 0]
Finite difference (m/rad): [-0.4919992314, -0.1339980713, 5.551115123e-11]
Predicted displacement (m): [-4.919992984e-7, -1.339978255e-7, 0]
Recomputed displacement (m): [-4.919992314e-7, -1.339980713e-7, 5.551115123e-17]
Max column difference: 2.458e-7 m/rad
Time: 0 -> 0 s
```

列差绝对容差 1e-6 m/rad 检查通过，恢复位置的 1e-12 m 检查和时间不变检查通过。
差分近似与浮点计算使结果不必完全相等；z 的极小量不应解释为显著物理位移。
这是助手实现和执行的运动学核对，未调用 mj_step、GUI 或安装依赖；未实现 IK。
本人正确回答真实位移 x/y“均减小”，不除以 ε 时单位为“m”，“不能”直接与列比较。
S4.4 核心练习完成，已勾选；随后本人追问 Jacobian 列如何得到，补充推导如下。
链接与 git diff --check 检查通过。

# What I Learned

本人手算：“Δx≈-0.0005 m，Δy≈0.0002 m，Δz≈0 m”，单位 m。
对原 x=0.3 m 的后续题回答“0.2995 m”；真实实验判断 x/y 均减小，
知道位移 m 不能直接与 m/rad 的 Jacobian 列比较。

## 补充：Jacobian 列的来源

数学上对 FK 关于一个关节角求偏导，其他关节角保持不变。
例如平面单杆 p=(L*cos(q),L*sin(q),0)，对应列为 (-L*sin(q),L*cos(q),0)。
几何上，对目标点上游的 hinge：Jp[:,j]=a_W×(p_W-o_W)。
a_W 是关节正向单位轴，o_W 是轴上的关节锚点，p_W 是 site 位置，全部使用世界表示。
× 为向量叉乘；结果方向沿点绕轴运动的圆的切线，大小为点到轴的垂直距离，
对应每弧度的位移变化率。有限差分是用两个相近 FK 结果估计该导数，用于核对。

2026-09-29 助手在同一 shell 激活并核验 mujoco 后进行额外几何核对，未改脚本：
读取 data.xaxis[j]（世界单位轴）、data.xanchor[j]（世界锚点，m），
并用 np.cross 计算叉乘（两个形状 (3,) 向量输入，返回 (3,) 向量）。
home 的 shoulder_pan 轴为 (0,0,1)，锚点 (0,0,0.163) m，
site-锚点约 (-0.13399783,0.49199930,0.32500037) m。
叉乘为 (-0.49199930,-0.13399783,0)，与 mj_jacSite 该列在 1e-12 绝对容差内一致。
临时 Python 检查退出 0，未调用 mj_step/GUI；这是助手补充验证，不冒充本人推导。
随后用户选择进入 S4.5，见下节。

## S4.5：姿态与奇异性（已完成）

2026-09-29 开始。先用已学的平面两连杆 L1=0.4、L2=0.3 m 解释，
基座在世界原点，只考虑世界 xy 位置任务。因此使用 2×2 的 J_xy，
不将平面模型恒为零的 z 行误判为异常，也不等同于 UR5e 完整位姿奇异性。

对 FK 求导：

```text
J_xy = [[-L1*sin(q1)-L2*sin(q1+q2), -L2*sin(q1+q2)],
        [ L1*cos(q1)+L2*cos(q1+q2),  L2*cos(q1+q2)]]
```

两种教学姿态（解析代入，不是新运行结果）：

- 伸直 q1=0、q2=0：J_xy=[[0,0],[0.7,0.3]] m/rad。
- 弯折 q1=0、q2=π/2：J_xy=[[-0.3,-0.3],[0.4,0]] m/rad。

每列表示单个关节对末端瞬时运动的影响；列能张成的方向数决定可独立产生的
瞬时末端速度方向数。伸直时两列共线，秩为 1；弯折时两列独立，秩为 2。
本例任务空间平时可达秩为 2，降到 1 的姿态称为奇异姿态。
伸直时一阶 Δx 为零，不代表有限角度变化后 x 永远不变：弯曲可产生二阶向内变化。
关节仍能转动，奇异不等于卡死、动力学不稳定或代码报错。

本人正确回答第二关节 +0.001 rad 时，一阶预测 Δx/Δy“分别是0 0.0003”，单位 m。

### 指标与助手实验（2026-09-29）

在本例统一角度单位、欧氏范数下，奇异值描述关节速度到末端速度的主方向缩放。
最小奇异值 σ_min 越小，最难运动方向的响应越弱；σ_min=0 时丢失瞬时方向。
条件数 κ=σ_max/σ_min，无量纲；本例越大表示方向间的响应差异越悬殊。
这不是“所有方向都不动”，也不是动力学稳定性指标。非零但很小不等于数学上已降秩。

np.linalg.svd(J,compute_uv=False) 接收 (2,2) 矩阵，返回降序的 (2,) 奇异值数组，
不修改输入。本步不推导 SVD 算法，不计算逆或 IK。
来源：[NumPy SVD](https://numpy.org/doc/stable/reference/generated/numpy.linalg.svd.html)、
[Modern Robotics 可操作性](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/)。

新增 [planar_singularity.py](../examples/05_jacobian/planar_singularity.py)，
比较弯折和近伸直姿态，另列完全伸直作为极限参考。
检查 pwd/git status，保留已有修改；WSL2 Ubuntu 24.04.5，自动激活并核验 mujoco。
解释器 /home/lucas/miniconda3/envs/mujoco/bin/python，Python 3.12.14、NumPy 2.5.3。

```bash
python examples/05_jacobian/planar_singularity.py
```

退出 0，无导入错误，q1 均为零：

| q2（rad） | σ_max（m/rad） | σ_min（m/rad） | 条件数 |
| --- | --- | --- | --- |
| π/2，弯折 | 0.538902538 | 0.222674772 | 2.42013288 |
| 0.001，近伸直 | 0.761577216 | 0.000157567713 | 4833.33293 |
| 0，伸直参考 | 0.761577311 | 0 | inf |

两组手算矩阵核对通过；三姿态 det(J)=L1*L2*sin(q2) 在 1e-14 绝对容差下通过。
伸直位移预测输出 [0,0.0003] m，与本人手算一致。
此处 inf 表示零分母对应的无限条件数，脚本正常退出，不是运行错误。
纯 NumPy 几何实验，未调用 MuJoCo、GUI、未新增依赖。链接与 git diff --check 通过。
本人结果解释记录：

- 对比较哪组更接近奇异，首次回答“完全伸直，最小奇异值”，选了极限参考组。
  助手澄清题目比较前两组：近伸直的最小奇异值更接近零。
- 对方向响应，正确回答“某个方向响应很弱”。
- 对近伸直且 σ_min>0 属于哪种情况，正确回答“接近奇异”。

本人完成预测与基本指标解释，S4.5 已勾选，Stage 4 五项学习练习完成。
代码与数值实验由助手完成，不推断本人独立推导 SVD 或实现奇异性检测。
下一小任务可选 S5.1：位置误差、步长和停止条件，手算一次简化 IK 更新；尚未开始。
本轮仅同步文档、检查链接和 git diff --check，未重跑数值实验或 GUI。

本步复述题：

1. 弯折和近伸直哪组更接近奇异，依据哪个数值？
2. σ_min 变小表示所有方向都难运动，还是某个方向的响应变弱？
3. 完全伸直时为何关节仍能转动，但末端丢失一个瞬时运动方向？
4. 为什么非零的小 σ_min 与 σ_min=0 需要区分？

# Interview Questions

1. 为什么 UR5e 的位置 Jacobian 是 3×6，行和列分别表示什么？
2. 单列乘一个微小角度，为什么输出单位是 m？
3. 为什么有限差分扰动后要先 forward 再读 site_xpos？
4. 为什么 Jacobian 乘角度增量只是局部近似，而不是完整 FK？
