# S17.1 — Cartesian virtual spring

约0.5～2h。前置：[S16.6 Jᵀ](19_6_jacobian_transpose.md)、[S16.7 motor mapping](19_7_force_mapping_integration.md)。
[代码](../examples/18_compliant_control/cartesian_spring.py) / [示例入口](../examples/18_compliant_control/README.md)。
Engineering / Learning 唯一状态来源：[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

工具偏离目标后，怎样让它像弹簧一样往回拉？本课不用IK重新设置位置，
而是在每个physics步读取实际位置，计算恢复力并用motor执行。
虚拟弹簧没有实体弹簧：软件把位移乘以刚度，驱动器产生对应的力。
偏得越远，拉得越大；经过目标时力为零，但之前积累的速度还在，所以会越过目标。
本课只教弹性恢复，S17.2才加入速度反馈阻尼。

## Core Concepts

fixture是两个嵌套的正交slide，q=[q_x,q_y]，单位m；姿态由结构固定，不靠姿态控制器。
工具site位置 x_W=[q_x,q_y,0]m，目标 x_d_W=[0,0,0]m。
初始 q=[.04,−.03]m，qvel=0；只在初始化设置qpos，循环中只通过mj_step运动。
没有重力、contact、joint stiffness、passive damping、速度反馈或外力脉冲。
X滑动时两块1kg质量一起移动；Y只移动工具，因此M=diag(2,1)kg。

## Mathematics：含义 / shape / unit / frame

数学按列向量写，代码向量为NumPy一维array。

| 量 | 数学shape / 代码shape | 单位 / 坐标 |
| --- | --- | --- |
| x、x_d、e=x_d−x | 3×1 / (3,) | m，world |
| K=kI₃ | 3×3 / 标量k实现 | N/m，world各轴同刚度 |
| F_cmd=Ke | 3×1 / (3,) | N，驱动器期望施加给机器人的恢复力 |
| J_p | 3×2 / (3,2) | slide为m/m；world工具线速度映射 |
| τ_cmd=J_pᵀF_cmd | 2×1 / (2,) | 本课为N；hinge项才是N·m |
| U=½(x−x_d)ᵀK(x−x_d) | 标量 | J，虚拟势能 |

固定目标下 F_cmd=−∂U/∂x，链式法则给出 τ_cmd=−∂U/∂q=J_pᵀF_cmd。
这与S16.7抵抗外载荷不同：这里F_cmd已经是要施加的恢复力，不能再额外加负号。
功率一致：τ_cmdᵀqdot=F_cmdᵀv_W，且v_W=J_p qdot。
本fixture J_p=[[1,0],[0,1],[0,0]]，J_r=0；没有Z运动自由度。
固定姿态与低DOF简化了映射，仍调用真实mj_jacSite核验。

未饱和时 M qdd=−kq；每轴连续解 q_i(t)=q_i(0)cos(√(k/m_i)t)，
周期 T_i=2π√(m_i/k)。初始虚拟势能为.125J（k=100）；
真实动能 E_k=½(2 qdot_x²+qdot_y²)。连续理想系统E_k+U守恒，故不会自行停止。

## Math-to-Code / APIs

```python
force = stiffness * (target - position)  # world N
requested = jp.T @ force               # joint generalized force
data.ctrl[:] = requested / GEAR        # gear=[2,1]
mujoco.mj_step(model, data)
```

MjModel.from_xml_string(XML)返回编译模型，包括质量/执行器/步长；MjData(model)
返回对应可变状态与计算buffer，qpos/qvel/ctrl均为(2,)。
mj_forward(model,data)不推进时间，原地更新FK/site位置、速度和动力学；返回值不是新状态。
mj_jacSite(model,data,jp,jr,site_id)把site原点的world线/角Jacobian写入两个(3,nv)buffer。
调用前先forward以获得当前q对应的运动学。本课jr恒零、site_xmat恒I。
mj_fullM(model,data,dst)把当前惯量展开到(2,2)dst，核验diag(2,1)kg。
mj_step(model,data)原地积分实际qpos/qvel/time；本课每步1ms，无IK或位置重置。
这些API的原地语义见[官方Python说明](https://github.com/google-deepmind/mujoco/blob/main/doc/python.rst)。

motor固定gain=1，scalar output与gear相乘得到joint force。
X的scalar forcerange为±10，gear=2→joint ±20N；Y为±20、gear=1。
ctrl本身未限幅，force由MuJoCo限制。独立probe请求[40,−40]N，actual=[20,−20]N；
两组运行轨迹均无饱和。一般饱和后不能把actual force当成未限幅弹簧梯度。

## Minimal Experiment / Expected → Actual

在仓库根目录核验mujoco环境后运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/cartesian_spring.py
python examples/18_compliant_control/cartesian_spring.py --stiffness 200
```

Expected：zero不移动；spring朝目标加速、越过目标、持续振荡；增加k会增加力并缩短周期。
Actual（2026-10-08，Zero，headless，两命令exit0，无import error）：

| k N/m | 初始F_xy N | 理论T_x/T_y s | 连续解最大位置差 | 总能量最大相对偏差 |
| --- | --- | --- | --- | --- |
| 100 | [−4,+3] | .888577 / .628319 | .152162mm | .398242% |
| 200 | [−8,+6] | .628319 / .444288 | .218812mm | .575907% |

每case2001个pre-step样本，t=0…2s；zero全程保持初始偏移。
势能数值梯度误差≤2.19e−11N；每步J/姿态/功率/gear/限幅/零bias/passive/contact检查通过。
CLI只支持k=100/200；非法nan/0/inf拒绝，exit2。
CSV保存position、velocity、command force、requested/actual joint force与能量，
JSON保存参数/metrics，PNG展示位移、力、动能/虚拟势能交换；全部在ignored tmp/s17_1_*。

## Explanation / Failure Cases

同刚度却不同周期，是质量不同，而非Jacobian或gear改变了正确执行后的期望力。
MuJoCo Euler在这里是半隐式：先更新速度，再用新速度更新位置；
因此数值总能量有小振荡，不能声称精确守恒。见[官方积分说明](https://github.com/google-deepmind/mujoco/blob/main/doc/computation/index.rst)。
zero图中的U只是该位置对应的候选弹簧势能，zero控制器不执行这条弹簧律。
wrong-sign静态probe验证F与位移点积为正，即向外推；本课未运行发散动态轨迹。
毫米误当米会把力放大1000倍；混frame或忘gear会产生错误驱动力。
提高k不能消除无阻尼振荡；较大dt/饱和会改变理想模型，不能由当前两组推断通用稳定性。

## Robotics Context / Interview Capsule

这是接触顺应控制的弹性部分：设定偏移对应的力，比无限追求位置误差归零更能表达柔软程度。
这里F_cmd是软件计算的驱动力，不是contact sensor measurement；未验证碰撞、实际接触或UR5e/硬件。

30秒：读取工具world位移，用F=k(x_d−x)产生恢复力，再用Jᵀ转换为joint广义力，
经gear换算到有界motor。纯弹簧储能但不耗能，所以越过目标后仍振荡。

2分钟：从U=½eᵀKe推导F=−∂U/∂x与τ=−∂U/∂q，说明world frame、
slide/hinge的单位差异及功率一致性。例子中M=diag(2,1)kg、J为XY嵌入，
k=100初始F=[−4,+3]N；k翻倍力翻倍、周期乘1/√2。用连续余弦解与势能梯度
交叉核验，同时检查actual motor force。无阻尼不能settle，dt与限幅限制结论。

## Must Remember / My Verification — Run / Modify / Explain

记住：力朝目标；Jᵀ保持虚功/功率；广义力单位取决于joint；弹簧储能，阻尼耗能。
工程已完成；2026-10-08 本人明确确认实验与预测完成，并正确回答全部五项Explain，
Run/Modify/Explain全部完成，Learning Mastered；状态见根README。
解释补充：三维推广U=½(x−x_d)ᵀK(x−x_d)，K需为对称刚度矩阵，
τ=−∂U/∂q=JᵀF；本fixture有效质量X=2kg、Y=1kg，故T_x/T_y=√2。
实际驱动力是qfrc_actuator广义力；一般不能仅凭它反推出唯一的工具接触力。

Run：执行默认命令并查看tmp/s17_1_k100/response.png，观察工具过零时速度仍非零。
Modify：先预测，再仅把k从100改成200；核对初始力×2、周期×1/√2、仍不停止。
Explain：

1. 为什么用x_d−x？怎样从虚拟势能推导恢复力与Jᵀ映射？
2. 这课τ的单位为什么是N？换成hinge后是什么单位，J列的单位怎样变化？
3. 为什么同k下X比Y周期长？为什么加大k不能让纯弹簧自行停止？
4. gear=2时请求−4N应写多少ctrl？请求40N时actual是多少，限幅发生在哪里？
5. F_cmd、actual joint force与contact measured force有何区别？此次验证支持哪些结论？

现场环境：WSL2 Ubuntu24.04.5；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2，
metadata/真实module paths/APIs已核验，无安装或requirements变动；3.11兼容目标未在3.11运行。
本课不需要GUI，未做GUI验证。本人Run/Modify/Explain已完成；下一小任务S17.2等待明确请求。
