# S16.5 — MuJoCo Contact Force / Wrench

[代码](../examples/17_force_dynamics/contact_wrench.py) · [阶段入口](19_force_dynamics.md) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

S16.4会变换已知wrench，现在从MuJoCo接触求解器读取真实仿真接触wrench。
P1曾把第一分量normal force相加作事件门限；这不能说明完整方向、摩擦矩或作用点。
像桌子托住箱子：四个接触点各有力，合力托住重量；每个点对箱体质心还可能产生力臂矩。
球形指端贴墙时，法向不一定是world z，摩擦还可能抵抗滑动、扭转和滚动。
本课测量与解释，不实现force controller、UR5e、Jᵀ或完整抓取。

## Core Concepts / Physical Contract

两个独立fixture：

- box_support：自由箱体，半尺寸[.04,.03,.025]m，质心初始z=.08m；world gravity[0,0,−9.81]m/s²，水平plane，condim3，质量1或2kg。
- finger_wall：自由球形指端，半径.03m、质量.2kg，重力0；静止竖直plane，其朝向自由空间的法向n=[√3/2,.5,0]。施加已知world力与关于指端COM的力矩，无actuator。

指端载荷先在合成坐标系A定义：F_A=[−2,.3,.1]N，M_A=[.002,.001,.001]N·m；
R_WA=Rz(30°)，x_A=n，写入world `xfrc_applied=[R_WA F_A; R_WA M_A]`。
它是持续外部载荷，不是contact读数；无position hold反馈。指端fixture使用condim6、
friction=[1,.05,.02]；滑动系数无量纲，扭转与滚动系数单位m，不能统一叫无量纲摩擦系数。
这是理想几何与软摩擦接触，未建真实指腹形变/触觉。

| 字段 / 数学量 | shape / 单位 / 含义 |
| --- | --- |
| data.ncon / data.contact[i] | 当前接触数量 / 第i个记录，index每次forward后可能变化 |
| contact.geom[0:2] | 两个geom ID；geom1/geom2是旧命名，新代码用geom数组 |
| contact.pos / dist | (3,) world接触点m / signed gap m，负值表示穿透 |
| contact.frame | (9,)→(3,3)，**行**为normal、tangent1、tangent2在world中的表达 |
| raw from mj_contactForce | (6,), [Fn,Ft1,Ft2,Mn,Mt1,Mt2]；前三N、后三N·m，矩关于contact.pos |
| data.xipos[body] | (3,) world COM位置m，不一定等于body origin |
| summed F / M_COM | 各(3,), world N / 关于目标body COM的N·m |
| xfrc_applied[body] | (6,), world [force;torque]，torque关于body COM；持续外载荷 |
| qpos / qvel | 本自由关节(7,)/(6,)；平移m+四元数，速度world线速度m/s+body角速度rad/s |

contact frame的**x**轴才是normal，不是z。
normal从geom[0]指向geom[1]，API读出的wrench作用于geom[1]；geom[0]受同点反号wrench。
必须按body/geom身份选择sign，不能排序geom名字后丢失方向。
本例目标body在geom[1]得到sign+；另测world/support body验证sign−分支。
依据[官方 contact结构](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjcontact)、
[contact模型](https://mujoco.readthedocs.io/en/stable/computation/index.html#contact)。

condim1：仅normal force；3：加两切向force；4：加normal轴扭转moment；6：再加两滚动moment。
API输出总是六分量，不活跃moment分量为零。本箱体condim3单contact矩为零，
但point force关于COM的力臂矩仍可能非零；不能据此说合力矩总是零。
检测到接触不一定产生力，efc_address=−1可能表示不进入求解；本课记录该字段，
未单独注入gap/inactive-contact场景。API可提取零输出，勿把ncon直接当“有负载”。

## Mathematics → Math-to-Code

设A=`contact.frame.reshape(3,3)`，其行是contact轴在world的表达；
所以world→contact用A，contact→world用A.T。选目标body的s=+1或−1：

```text
F_i_W = s * A.T @ raw[:3]
M_i_contact_W = s * A.T @ raw[3:]
M_i_COM_W = M_i_contact_W + (contact.pos - COM_W) × F_i_W
F_sum_W = sum_i F_i_W
M_sum_COM_W = sum_i M_i_COM_W
```

复用S16.4：旧点contact→新点COM，位移是contact−COM。
先统一到同body、同world轴、同COM再累加；不同contact的原始力不能直接当world向量加。

每步独立检验Newton平动方程：
`m a_COM_W = F_contact_sum_W + F_external_W + m gravity_W`。
本模型仅一个自由刚体，body origin=COM，无其它运动关节/被动项；
其freejoint前三qacc是world平动加速度。一般有偏置COM或多body时不能照搬此捷径。

箱体静止尾段a≈0、无外力：Fz≈mg，Fx/Fy≈0，COM合矩≈0。
落下阶段没有contact，撞击时a≠0，支撑力瞬间可大于mg，不是求解器错误。
指端接触合力≈−external force，关于COM合矩≈−external torque；
**净力/矩为零只说明没有继续加速，不能证明速度为零或不滑移。**
代码记录两种速度，不把软摩擦下的持续小运动称作静止。

作用反作用验证先读取support body sign−，再统一到world原点O：
`M_contact_on_moving_about_O = M_COM + (COM−O)×F`，
它应等于support受力矩的负值。直接对不同body COM的合矩取负通常不成立。

## APIs：输入、输出与原地变化

| API | 行为 / 本课用途 |
| --- | --- |
| MjModel.from_xml_string(xml) | MJCF→compiled model，决定惯量/碰撞/friction/solver |
| MjData(model) | model→mutable q/v/contact/solver arrays，nq7/nv6/nu0 |
| mj_forward(model,data) | 原地刷新接触、求解器力、qacc，不推进time，返回None |
| mj_contactForce(model,data,index,result) | 输入当前contact index，原地填(6,)float64缓冲，返回None；不返回新向量、不做world变换 |
| mj_step(model,data) | 读取持续外力，原地积分一个1ms dt并推进time |

API依据[官方 mj_contactForce](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-contactforce)。
代码`measure`返回world F、COM moment与逐contact记录；frame、position、raw都来自当前forward。
先写external→forward→读wrench/state→step；pre-step记录t0–2s共2001行。
积分implicitfast，elliptic cone，iterations100、tolerance1e−12；solref=.02/1。
参数是本fixture设置，未做硬度/步长/摩擦稳定范围扫描。

## Minimal Experiment / Dependencies

仅requirements已声明的mujoco/numpy/matplotlib，无local helper/Menagerie/ROS、新依赖或安装。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/contact_wrench.py
python examples/17_force_dynamics/contact_wrench.py --mass 2
```

产物仅ignored tmp/s16_5_mass1/与mass2/：两份2001×18 CSV、results.json、contact_response.png。
JSON保存最终逐contact geom、sign、frame、raw六分量、point、distance、转换前后moment。
图左为箱体support与COM高度，右为指端world合力与COM合矩。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；conda mujoco所属Python3.12.14。
MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2 metadata/import/module paths/API核验；
无安装/requirements变更，3.11兼容目标未用3.11执行。

| Case | final contact F_W (N) | final COM moment (N·m) |
| --- | --- | --- |
| box mass1 | [0,0,9.810000000002] | [0,0,0] |
| box mass2 | [0,0,19.620000000005] | [0,0,0] |
| finger（两命令相同） | [1.882050808,.740192379,−.1] | [−.001232051,−.001866025,−.001] |

箱体最终4个contact，均condim3；normal总和≈mg，不要求任意复杂场景每点均分。
末.5s force error≤5.21e−12N，linear speed≤1.74e−15m/s；COM z=.024946997m，
比几何半高.025m低约53µm，体现soft contact小穿透，不是零穿透硬接触。
每步Newton平衡最大误差≤5.99e−12N（含碰撞），初始ncon0分支通过。

指端最终1个condim6 contact；raw约
`[2,−.1,.3,−.002,−.009969748,−.001989916]`，六分量全部非零。
contact-point moment world约[−.002727009,.000723318,−.009969748]N·m，
换到COM后才与external torque反号相等，说明不能漏力臂矩。
尾段force error≤6.93e−14N、moment error≤4.81e−13N·m；
max linear速度分量.002197249m/s、max body角速度分量.060501533rad/s，**仍在小幅运动**。
速度指标是最大绝对分量，非向量norm。受力平衡不等于静止或真实stick contact。

最终两命令均exit0；frame正交/反变换、normal非负、sign两分支、统一点reaction、
所有step Newton及尾段load balance通过。倾斜墙面使误用A而非A.T的force错误被检出，
漏COM力臂矩也被检出。初版额外“finger已静止”验收失败；观察soft摩擦持续运动后，
改为如实报告速度并仅将箱体作为静态支撑验收，未将初版失败计PASS。
无GUI、实际触觉/硬件、UR5e、闭环力控制或一般摩擦稳定性验证。

## Failure Cases

- raw[0]当world Fz：它是contact normal分量，竖直墙上normal近水平。
- 将frame当列基直接乘A：MuJoCo此字段是行基，local→world用A.T。
- 不核对geom顺序就取正力：可能把机器人→环境当环境→机器人。
- 只记录前三项：condim4/6可能有非零接触moment。
- 只加contact moment不加力臂矩：关于COM预算错误。
- 把contact检测/穿透量当力传感器：需要solver输出且模型参数影响结果。
- 接触冲击时强求Fz=mg：漏掉加速度；先检查m a。
- 力平衡就声称不滑：必须同时看速度/位移，软摩擦模型可能有持续运动。
- 将contact index跨step缓存：接触数量与顺序可变，必须重新识别geom/body。

## Robotics Context / Interview Capsule

完整contact wrench帮助解释夹爪支撑、切向滑动、软指端扭转/滚动阻力；
它是仿真模型输出，真实机器人仍需传感器、标定、滤波与接触参数辨识。
本课建立读数→方向→world→COM→平衡链路；Jᵀ映射留S16.6，force control留Stage17。

**30秒：** mj_contactForce填contact frame下的[force;moment]，normal是x轴。
frame行基需转置到world，按geom顺序取目标body符号，再以contact−COM叉乘force换点求和。
静态支撑约mg，动态用Newton方程，受力平衡不代表静止。

**2分钟：** 从箱体condim3/球指端condim6对照开始，明确body、轴、点、单位与API原地输出。
说明sign和A.T，再写COM力矩公式。用质量翻倍支撑力翻倍、斜墙六分量与reaction核对支持结果，
指出冲击峰值与软摩擦持续小运动，最后限定headless模型实验，不外推真实触觉或闭环稳定。

## Must Remember / My Verification / Run → Modify → Explain

normal=x_contact；frame行基；API六分量force在前；geom顺序决定符号；
统一点轴再相加；contact torque不等于COM torque；Fz≈mg只适用近静态；平衡不证明静止。
Engineering与Learning状态见根README。2026-10-08 本人确认实验与预测完成，
五项Explain准确覆盖API/坐标与符号、contact矩与COM力臂矩、动态支撑、
独立质量对照以及平衡/静止与reaction参考点的边界；Run/Modify/Explain全部完成，Learning Mastered。
术语对照：旧geom1/geom2分别对应当前contact.geom[0]/geom[1]，代码按geom_bodyid判body归属。
本次仅同步文档，runtime沿用2026-10-08 Zero工程验证，未重跑仿真或GUI。

**Run：** 运行默认命令，对照JSON finger raw与world force；在box图找无接触、撞击和尾段支撑。

**Modify（只一项）：** 先预测只把箱体mass1改2对支撑力、接触力矩、finger实验有什么影响，
再运行`--mass 2`；验证尾段support而不假定冲击峰值或settling时间必然按某比例改变。

**Explain（五问）：**

1. mj_contactForce的输入/输出、分量顺序、单位、坐标轴与参考点是什么？
2. 为什么local→world用frame.T？如何按geom/body身份选择受力符号？
3. condim3的raw后三项为零，为什么关于COM的单点力矩仍可能非零？怎样合成总wrench？
4. 箱体支撑为什么尾段≈mg，撞击时却可大于mg？质量翻倍时finger为何不变？
5. 为什么指端load balance通过不能说它静止或真实测力正确？reaction moment比较为何先统一点？
