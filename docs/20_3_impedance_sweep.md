# S17.3 — Stiffness / damping sweep：增益、dt 与限幅一起看

约0.5～2h；前置：[S17.2](20_2_cartesian_impedance.md)。
[代码](../examples/18_compliant_control/impedance_sweep.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning 状态只见[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

怎样选择恢复快、振荡小、执行器能实现的 K/D？上一课只比较两个D，本课做有限参数对照。
弹簧更硬会加速恢复，同时增加请求力；阻尼太少会越过目标，太多会拖慢运动。
控制器每隔dt才重算力：即便连续模型耗能，过粗的更新也可能把刹车变成下一步的过度加速。
执行器裁剪后，实际动力学也不再是原来的线性弹簧阻尼系统。

## Core Concepts

复用cartesian_spring的XML、INITIAL、GEAR与Agg；依赖链仅mujoco/numpy/matplotlib，requirements不变。
world XY固定姿态slide，M=diag(2,1)kg，q₀=[.04,−.03]m，v₀=0，reference固定在原点。
无gravity/contact/passive damping。K=100/400N/m × ζ=.5/1/2，共六组；
每组D_i=2ζ√(m_i K)，不是同一D。默认dt=1ms、joint force cap=20N。
另两组分别只改cap到2N、只改dt到80ms；相对基准K400/ζ1保持其他参数一致。
没有随机采样或seed；每组用独立MjData，从同样初值开始。

## Mathematics：意义、shape、unit、frame

world位置/速度(3,)为m、m/s；Jp为(3,2)，qvel为(2,)m/s。
F_W=−Kx_W−Dv_W，v_W=Jp qvel；τ_req=JpᵀF_W为(2,)slide force N。
这里只控制XY，D为两轴对角增益N·s/m，K为相同的标量N/m；Z力为0。

未饱和连续模型每轴：m qdd+d qdot+kq=0，ω_n=√(k/m)，ζ=d/(2√(mk))。
K翻四倍且保持ζ，ω_n与D翻倍、初始弹簧力翻四倍；不是只改K不改D的实验。
固定初值v₀=0，初始阻尼力为0；所有组peak position包含初始偏移，故另记录overshoot。

本fixture零joint damping、Euler、每physics step显式更新ctrl，其递推为：

\[
v_{n+1}=v_n+\frac h m\operatorname{clip}(-kq_n-dv_n,-c,c),\qquad
q_{n+1}=q_n+h v_{n+1}.
\]

MuJoCo的Euler使用半隐式位置更新，joint damping另有隐式处理；这里阻尼由motor控制律计算，
没有joint damping，不能当作隐式阻尼。[官方积分说明](https://mujoco.readthedocs.io/en/stable/computation/)。

未饱和时，以[q,v]ᵀ组成状态，单轴离散矩阵：

\[
A=\begin{bmatrix}1-h^2k/m&h(1-hd/m)\\-hk/m&1-hd/m\end{bmatrix}.
\]

谱半径ρ(A)是特征值绝对值的最大值，无量纲；ρ<1表示这个线性递推渐近稳定。
令a=h²k/m、b=hd/m，特征多项式λ²−(2−a−b)λ+(1−b)=0。
对k,d,h>0，二阶单位圆判据给出a+2b<4。
这是本离散模型的推导，不是通用机器人稳定性判据；饱和后不能再用A准确预测整段轨迹。
K400/ζ1/h=.08时X/Y谱半径2.134/4.275，局部未饱和线性模型已失稳；
实际轨迹由20N限幅约束加速度，但仍不能settle。不把有限4s振荡称为已证明的极限环或无限发散。

## Math-to-Code / APIs

```python
velocity = (jp @ data.qvel)[:2]
force = -stiffness*position-damping*velocity
requested = jp.T @ np.r_[force, 0.]
data.ctrl[:] = requested/GEAR
mujoco.mj_step(model, data)
```

MjModel.from_xml_string返回编译模型；设置model.opt.timestep（秒）同时改变physics与controller周期。
本课没有分离controller与physics频率。MjData返回可变状态buffer。
mj_forward原地更新FK/动力学、不推进时间；mj_jacSite把world site线/角J写入(3,nv)buffer；
mj_step原地推进qpos/qvel/time。API不返回新状态，执行期间不重设qpos/qvel。
model.actuator_forcerange为(nu,2)，改为±cap/gear；gain1、gear=[2,1]，
qfrc_actuator=gear×actuator_force=clip(requested,±cap)。ctrl本身不裁剪。
不需要增加新API框架，关键表达式全部保留可读。

## Minimal Experiment / Expected → Actual

仓库根目录：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/impedance_sweep.py
python examples/18_compliant_control/impedance_sweep.py --cap 5
```

Expected：欠阻尼越过目标；过阻尼不越过但恢复慢；提高K与匹配D提高速度与力需求。
低cap使实际力小于请求；粗dt即使ζ=1也可能振荡。
两个命令各8case：CSV pre-step t=0…4s，细dt为4001×9、粗dt为51×9；JSON指标与sweep.png。
settling：首次两轴|位置|<1mm且|速度|<2mm/s，并在剩余记录中始终满足。
只检查采样点，到4s为止；不声称采样间或无限未来保证，粗dt尤其需谨慎。
overshoot为目标另一侧最大位移/该轴初始偏移；crossings只计|q|>1mm样本的符号变化，
忽略门限内细小反转；peak speed/force是所有轴与记录采样的最大绝对值。
饱和数为joint-sample数，不能直接当作step数或持续时间；tail是最后.5s最大轴位置误差。

Actual（2026-10-08，DESKTOP-781D67A，两命令exit0）：

| K N/m | ζ | settling s | overshoot X/Y | peak speed m/s | peak actual N |
| --- | --- | --- | --- | --- | --- |
| 100 | .5 | 1.381 | 16.23% / 16.20% | .164106 | 4 |
| 100 | 1 | .976 | 0 / 0 | .110550 | 4 |
| 100 | 2 | 1.988 | 0 / 0 | .065684 | 4 |
| 400 | .5 | .723 | 16.16% / 16.11% | .328666 | 16 |
| 400 | 1 | .547 | 0 / 0 | .221483 | 16 |
| 400 | 2 | 1.164 | 0 / 0 | .131612 | 16 |

六组无饱和，ρ<1；本有限候选中ζ1最快，不推出所有目标/门限下都最优。

| K400/ζ1对照 | settling s | 请求峰值N | 实际峰值N | 饱和joint-samples | tail error m |
| --- | --- | --- | --- | --- | --- |
| 1ms / 20N | .547 | 16 | 16 | 0 | 1.14e−20 |
| 1ms / 2N | .640 | 16 | 2 | 246 | 3.71e−20 |
| 1ms / 5N（Modify） | .574 | 16 | 5 | 95 | 1.60e−20 |
| 80ms / 20N | 未settle | 77.509888 | 20 | 87 | .0654503 |

粗dt峰值速度1.053184m/s，overshoot X/Y62.65%/294.85%，显著过零12/19次。
低cap并不必然引起振荡：这里2/5N均恢复且无overshoot，但比20N慢。
工程检查：world J、actual cap、qacc=actual/M、零bias/passive/contact、finite shape/time；
另用独立解析clip半隐式递推逐点核对q/v/actual，不调用MuJoCo积分作参考。
两命令未改变的七组CSV逐元素一致；default PNG已目视检查；CLI cap0/nan均exit2。
无安装；WSL2 Ubuntu24.04.5，Python3.12.14、MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2，
同shell核验mujoco/interpreter、metadata/真实import/module paths与API。未在Python3.11执行。
产物仅ignored tmp/s17_3_cap2与tmp/s17_3_cap5；本次不运行GUI。

## Explanation / Failure Cases / Robotics Context

过阻尼的慢模态拖长恢复；欠阻尼的反复越过使持续门限晚满足。更硬也要求更大力，
受限执行器无法实现所指定的理想刚度/阻尼。保持ρ<1也只说明局部未饱和离散模型。
连续耗能E_dot=−vᵀDv不能直接套用粗dt或饱和；限幅后实际力改变，原能量等式不成立。
增加D在连续系统中可抑制振荡，但在显式反馈递推中增大hd/m也会缩小稳定dt范围。
因此不能用“多加阻尼”无条件修复粗采样，更不能把限幅当稳定性证明。

机器人增益选择还受配置相关有效质量、重力、传感噪声、延迟与接触刚度影响。
本课只建立自由空间参数比较方法；不含contact/UR5e/ROS/硬件，也未验证通用安全或接触稳定性。
接触过渡属于下一小任务S17.4，尚未实现。

## Interview Capsule

30秒：我在同XY装置上扫描六组K/ζ，再只改力限幅或dt。
ζ1在本候选中最快，ζ2更慢；提高K也提高力与速度。
显式motor阻尼配粗dt会离散失稳，实际力限幅不能保证收敛。

2分钟：写F=−Kx−Dv、v=Jqdot与Jᵀ映射；说明有效质量决定各轴D。
从半隐式递推得到A与ρ，区分连续ζ与离散稳定性；记录requested/actual、overshoot、
峰值与持续位置/速度门限。讲清六组、低cap与粗dt各只比较什么，最后说明接触和硬件边界。

## Must Remember / My Verification — Run / Modify / Explain

K/D与dt/质量/限幅必须一起记录；请求力不是实际力；连续临界阻尼不保证任意dt稳定。
Engineering已完成；2026-10-08 本人确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部完成，Learning Mastered；状态见根README。
解释精度补充：本课settling要求从候选时刻一直保持到4s记录末尾，而非只保持任意短窗口；
粗dt的谱半径对应未饱和局部线性递推，不能直接预测整段受限轨迹，
自由空间恢复不证明接触稳定。

Run：运行默认命令，查看tmp/s17_3_cap2/sweep.png与results.json，找到粗dt失败对照。
Modify：先预测只将limited的cap从2N提高到5N会怎样改变初始请求/实际力、饱和样本与settling，
再运行--cap 5；其余七组不变。记录预测与实际差异。
Explain：

1. K翻四倍而保持ζ时，D、自然频率与初始请求力各改变多少？为什么两轴D不同？
2. 为什么ζ2比ζ1恢复慢？为什么峰值位移不能直接表示overshoot？
3. 连续ζ1为什么在80ms仍振荡？A的谱半径适用于哪一段动力学？
4. cap2N时初始请求16N、实际2N分别说明什么？限幅为何不等于稳定性保证？
5. settling为何同时检查位置、速度和后续窗口？这组自由空间结果能证明接触稳定吗？
