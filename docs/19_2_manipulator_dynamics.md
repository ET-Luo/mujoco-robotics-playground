# S16.2 — Manipulator Dynamics Equation

[阶段入口](19_force_dynamics.md) · [代码](../examples/17_force_dynamics/manipulator_dynamics.py) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

已知 actuator 输出力矩，怎样预测关节加速度？位置控制最终也服从力矩平衡，
所以必须知道驱动力矩有多少用于克服重力、耗散运动、改变速度。
想象一个悬挂摆：释放后重力让它朝最低点加速；阻尼抵抗运动；惯量决定同样净力矩能改变速度多快。
本课只拆解动力学，不设计新controller，不移植UR5e。

## Core Concepts：形状、单位、方向

固定world +y hinge，右手正向，q=0时质心在转轴正下方；重力world −z。
q>0时质心朝world −x偏移，重力实际力矩为负。所有力矩均关于hinge轴。

| 数学 / MuJoCo | 本例 shape / 单位 / 含义 |
| --- | --- |
| q / qpos | (1,), rad，关节角 |
| v / qvel；a / qacc | (1,), rad/s；rad/s² |
| M / dense inertia | (1,1), kg·m²，关于关节轴的有效惯量 |
| b / qfrc_bias | (1,), N·m，方程左侧的重力与速度bias |
| tau_passive / qfrc_passive | (1,), N·m，本例黏性阻尼 |
| tau_act / qfrc_actuator | (1,), N·m，本例gear1 motor实际力矩 |
| tau_ext / qfrc_applied | (1,), N·m，人为施加的关节广义外力矩 |
| tau_constraint / qfrc_constraint | (1,), N·m，求解器约束反力矩；本例为0 |

一般M为(nv,nv)，广义力为(nv,)；hinge对应N·m，slide对应N，不能给所有元素统一单位。
ball/free joint有四元数时nq不等于nv，也不能直接把qpos逐元素求导当qvel。
本例没有接触、限位、frictionloss或equality；constraint=0经过ncon/nefc核验，
只验证了无约束分支。constraint还可来自限位/equality/干摩擦，并不只指contact。
未施加xfrc_applied；一般body外部wrench还需映射进广义力，不能只读取qfrc_applied。

## Mathematics

MuJoCo一般写作`M vdot + b = tau_act + tau_passive + tau_ext + tau_constraint`。
本fixture是刚体绕固定轴旋转，因此：

```text
I_COM = 0.02 * inertia_scale                 [kg m²]
M = I_COM + m l²                            [kg m²]
b = m g l sin(q)                            [N m]
tau_passive = -d v                          [N m]
a = (tau_act + tau_ext - d v - m g l sin(q)) / M
```

m=1kg，l=0.3m，d=0.1N·m·s/rad，g=9.81m/s²。
平行轴定理解释为什么质心惯量不是关节惯量。bias放在左边，所以实际重力力矩是−b。
此单轴固定惯量模型没有速度bias；一般机械臂`b(q,v)=C(q,v)v+g(q)`，
不能将运动中的qfrc_bias一概叫gravity，且C矩阵的表示并非唯一。
静态v=a=0时`tau_act + tau_ext = b`；这只是同状态力矩平衡，不证明扰动后稳定恢复。

公式与字段依据[官方 computation 文档](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework)。

## Math-to-Code / APIs

| API | 输入、输出或原地改变 | 用途 |
| --- | --- | --- |
| MjModel.from_xml_string(xml) | MJCF字符串→compiled model | 定义惯量、质心、轴、重力、motor；nq=nv=nu=1 |
| MjData(model) | model→mutable state/derived arrays | 每个probe独立q/v/ctrl/force状态 |
| mj_forward(model,data) | 原地刷新M、bias、forces、qacc，不推进time，返回None | 同时刻读数与解析核对 |
| mj_fullM(model,data,dst) | 原地填充预分配(nv,nv) float64矩阵，返回None | 将内部packed qM展开成数学上的M |
| mj_step(model,data) | 原地推进dt、qpos/qvel/time，返回None | 自由释放2s |

本机MuJoCo3.13.0真实binding是`mj_fullM(model,data,dst)`；旧资料常见其它签名，
应检查当前`mujoco.mj_fullM.__doc__`，勿照搬。依据[官方 API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-fullm)。
代码用`matrix @ data.qacc + bias - rhs`核对残差，同时以独立刚体公式核对每项与加速度；
单靠残差小并不能证明惯量/力方向设置正确。
每行先forward再记录、然后step，是pre-step样本；初始qpos赋值后不再覆盖运动。
Euler默认隐式处理joint damping，因此第一步
`v1=dt*(-b0)/(M+dt*d)`，`q1=q0+dt*v1`；
不能误用`v1=v0+dt*qacc0`当精确验收。qacc是当前连续时间方程的加速度。

## Minimal Experiment

仅需requirements已声明的mujoco/numpy/matplotlib；无local helper或新依赖。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/manipulator_dynamics.py
python examples/17_force_dynamics/manipulator_dynamics.py --inertia-scale 2
python examples/17_force_dynamics/manipulator_dynamics.py --gravity 0
```

产物仅ignored `tmp/s16_2_I*_g*/`：release.csv（2001×11）、results.json、release.png。
JSON probes列顺序同CSV header；图依次为角度、bias/passive、机械能。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；conda mujoco，
Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2，metadata/module paths/APIs核验。
代码以3.11兼容语法编写，未在3.11解释器执行。无安装或requirements修改。

| Case | M (kg·m²) | q=.5,v=0释放a (rad/s²) | 最大力矩残差 (N·m) |
| --- | --- | --- | --- |
| 默认 | .11 | −12.826812365 | 3.33e−16 |
| inertia scale2 | .13 | −10.853456616 | 3.26e−16 |
| gravity0 | .11 | 0 | 0 |

以上三条命令均exit0、无import error；每个样本核验解析M/bias/passive/a及平衡，
三种同状态probe核验time0/静态a0；Euler首步检查通过。
默认moving_loaded：q=.5rad,v=.4rad/s，actuator=.2、external=.1、passive=−.04N·m，
bias=1.410949360N·m，所以a=(.2+.1−.04−1.410949360)/.11=−10.463176001rad/s²。
静态probe有external=.1N·m，actuator=1.310949360N·m，a=0。
scale2保持mass/COM/gravity不变：bias与静态所需力矩不变，释放加速度之比=.11/.13≈.846154。
默认机械能由.360274520降至.064439805J；离散能量不是精确连续守恒证明。
gravity0且初始v0、motor0时保持q=.5：阻尼没有将静止摆恢复到q0的能力。
无GUI或真实硬件验证，也未验证多关节Coriolis/耦合、接触或motor饱和；理想无限幅motor仅供分项实验。

## Failure Cases

- 把bias当右侧实际重力：恢复方向写反；正确右侧是−bias。
- 只改mass便期望加速度减半：重力负载也随mass改变；需要明确哪些量保持不变。
- 忽略ml²：错把质心惯量翻倍等同于关节惯量翻倍。
- 把passive与bias合并、把constraint算成motor输出：丢失施力来源。
- 直接将packed qM reshape为dense矩阵：多DOF内部存储不是普通矩阵。
- 混合step前后state/force、误判Euler首步：记录时序或积分语义错。

## Robotics Context

同一套力矩预算用于机械臂负载估计、重力补偿和接触分析；运动时还要考虑多关节惯性耦合。
P1的qfrc_applied=bias等于添加理想外力，绕开驱动能力；S16.3再通过有界motor处理UR5e gravity。
本课到此停止。

## Interview Capsule

**30秒：** 关节加速度由净广义力除以有效惯量决定。MuJoCo把gravity/速度bias放左侧，
右侧分actuator/passive/external/constraint；固定摆M=I_COM+ml²，gravity实际力矩是−mgl sin(q)。

**2分钟：** 先说明world+y hinge与q0向下，列出状态/力矩单位。
用平行轴定理求M，解释bias符号和阻尼耗散；将moving probe数字代入方程，
说明同状态静态平衡并非稳定性。再讲fullM、forward无时间推进和pre-step采样，
最后以惯量单变量对照及解析加速度检查支持结论，限定无约束单DOF范围。

## Must Remember / My Verification / Run → Modify → Explain

M关于关节轴；bias移到右侧要反号；passive独立于motor；constraint独立于actuator；
瞬时平衡不等于闭环稳定；连续qacc不必等于离散速度差/dt。
Engineering与Learning状态见根README。2026-10-08 本人确认实验与预测完成，
五项Explain准确说明bias符号、平行轴定理、阻尼方向、瞬时平衡与稳定性、
一般bias及forward/step区别；Run/Modify/Explain全部完成，Learning Mastered。
补充精度：阻尼系数d单位为N·m·s/rad，阻尼力矩单位为N·m；
本例constraint=0仅验证无约束分支，非接触反力验证。
本次仅同步文档，runtime沿用2026-10-08工程验证，未重跑仿真或GUI。

**Run：** 运行默认命令，查看JSON moving_loaded的六项力矩预算与release.png。

**Modify（只一项）：** 先预测`--inertia-scale 2`对M、bias、静态motor torque、释放加速度的影响，
再运行；核对加速度比是否为.11/.13，而不是1/2。

**Explain（五问）：**

1. 为什么这里q>0时bias为正，释放加速度却为负？
2. 为什么M不是I_COM？质心惯量翻倍为何没有让加速度减半？
3. 对moving_loaded列出完整力矩方程，passive的符号和单位是什么？
4. static_balance的a=0为什么不证明闭环恢复？constraint=0又验证了哪些边界？
5. 一般机械臂的bias为什么不只是gravity？mj_forward和mj_step有何区别？
