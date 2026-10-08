# S16.4 — Force / Torque / 6D Wrench

[代码](../examples/17_force_dynamics/wrench_frames.py) · [阶段入口](19_force_dynamics.md) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

同一负载，腕部传感器与工具尖端读到的力矩为什么不同？一个wrench必须同时注明
作用主体、坐标轴、力矩参考点和分量顺序，否则数字无法正确用于机器人。
像用扳手拧螺母：同样的力，力臂改变，转动效果改变；单纯换观察坐标轴则不改变物理负载。
本课只做已知载荷的几何变换，不测接触、不驱动UR5e、不计算joint torque或IK。

## Core Concepts / Shape / Units / Frames

约定wrench为`w=[Fx,Fy,Fz,Mx,My,Mz]`，shape(6,)；前三项N，后三项N·m。
F是合力，M是关于明确参考点的合力矩；多个载荷必须先换到相同点、相同轴再相加。
力矩是右手向量；只有关于某个轴的分量才是绕该轴的有符号torque。
不要把六个Cartesian wrench分量当六个关节力矩，也不要把混合单位的6-vector普通范数当物理“力大小”。

| 本例数据 | 值 / 含义 |
| --- | --- |
| W | world右手坐标，所有几何位置先用W表示 |
| P | [.2,0,0]m，环境作用于机器人刚性工具的载荷施加点 |
| F_W | [0,0,−10]N，环境→机器人；反作用需反号 |
| c_W | [.1,.2,.3]N·m，另加的纯力偶矩，不是第二个point force |
| O | [0,0,0]m，world原点 |
| Q | [.05,.1,0]m，工具原点/新的力矩参考点 |
| T axes | 相对W绕+z转+90°，x_T=y_W，y_T=−x_W，z_T=z_W |

这是合成刚体载荷，c为独立纯力偶，其矩不随参考点改变。
单个point force在其作用点P的矩为0；本例关于P的总矩是c。
机器人→环境的reaction wrench只有换到相同点与坐标轴后才可逐分量取负。

## Mathematics

同一坐标轴、由旧参考点A换到新点B：

```text
F_B = F_A
M_B = M_A + (p_A - p_B) × F_A
```

理由：每个作用点P关于B的力臂是`P-B=(P-A)+(A-B)`。
所以增加项的位移方向必须从**新点B指向旧点A**。
位移平行F时叉积为0；F=0的纯力偶对任意参考点都有相同矩。

同一参考点、换轴：

```text
F_new = R_new_old @ F_old
M_new = R_new_old @ M_old
```

R_WT列是T轴在W中的表达，向量T→W用R_WT；W→T用R_TW=R_WT.T。
本课保留已经学过的列向量坐标约定。轴旋转不会改变F和同点M各自的范数；
改变参考点可以改变M的范数。不能将力当位置用`R F + translation`。
工具原点Q同时换点换轴，由W原点O的wrench得到：

```text
F_T = R_TW @ F_W
M_Q_T = R_TW @ (M_O_W + (O_W - Q_W) × F_W)
```

等价6×6形式（只用于对应数学，不在代码隐藏步骤）：
`w_Q_T = [[R,0],[R [O-Q]_cross,R]] @ w_O_W`，R=R_TW。
这里`[r]_cross F = r×F`；不是把4×4位姿矩阵直接乘6-vector。

概念来源：[Modern Robotics作者的wrench课程](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-4-wrenches/)。
该教材采用moment在前的顺序，与本项目force在前不同；不能直接复制其块矩阵排列。

## Math-to-Code / APIs

`shift_reference(F,M,p_old,p_new)`直接返回`M + np.cross(p_old-p_new,F)`；
所有输入是同轴(3,)向量，positions为m，force为N，moment为N·m。
`rotate_axes(R,F,M)`返回两个新数组，不修改输入；R shape(3,3)，F/M shape(3,)。
NumPy `cross`返回(3,)叉积，`@`做矩阵/向量乘法，`concatenate`只打包(6,)不改变物理量。
函数面向本课确定输入，非带全面验证的通用SDK；CLI只允许已验证lever .2/.4。
本课不调用MuJoCoAPI；MuJoCo不同字段的空间向量顺序需各自核查，不能由本项目约定推断。

额外一致性检查是刚体瞬时功率（未引入controller或完整twist教程）：
`v_Q = v_O + omega × (Q-O)`，`Power = F·v_reference + M_reference·omega`。
速度必须在**同一个力矩参考点**；换点后不能保留旧点线速度。
取v_O=[.1,.2,−.1]m/s、omega=[.2,−.3,.4]rad/s，用两个点和两组轴算得同功率W。

## Minimal Experiment

只需requirements已声明的NumPy与Matplotlib；无local helper，不需MuJoCo/Menagerie运行或新依赖。
仍在项目规定的mujoco环境执行数值实验。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/wrench_frames.py
python examples/17_force_dynamics/wrench_frames.py --lever 0.4
```

产物仅ignored `tmp/s16_4_lever0.2/`与`tmp/s16_4_lever0.4/`：results.json、
lever_sweep.csv（41×4）和wrench_geometry.png。CSV扫P_x，F/c/Q/轴不变。
图左为world x-y俯视，P的×标记表示力向−z进入页面；图右为关于Q的world矩随力臂变化。
Modify移动的是**载荷施加点P**，不是仅换表达方式；因此物理力矩改变。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2，conda mujoco所属Python3.12.14。
NumPy2.5.3、Matplotlib3.11.2 metadata/import/module paths/API核验，无安装/requirements变更。
3.11兼容语法目标未用3.11执行。

| P_x (m) | M_O_W (N·m) | M_Q_W (N·m) | M_Q_T (N·m) | 功率 (W) |
| --- | --- | --- | --- | --- |
| .2 | [.1,2.2,.3] | [1.1,1.7,.3] | [1.7,−1.1,.3] | .48 |
| .4 | [.1,4.2,.3] | [1.1,3.7,.3] | [3.7,−1.1,.3] | −.12 |

两命令exit0，无import error，force均[0,0,−10]N。
解析分量、直接P→Q与P→O→Q、换轴/换点两路径、往返、旋转范数、纯力偶、
沿force方向平移reference、同点reaction及三种表示功率检查通过，atol1e−12。
负功率表示该载荷在所选瞬时刚体运动上吸收机械能，不是负的力大小。
漏换点造成moment误差范数1.118034N·m，位移符号写反造成2.236068N·m；
两组均主动检出。增大P_x仅改变My_W及其换轴对应分量，c与F不变；
总moment不会全量翻倍，因为还有独立c和固定Q偏移。
这些是代数/几何实验，无仿真动力学、传感器、contact、GUI或硬件验证。

## Failure Cases

- 只写“tool wrench”不注明参考点/受力主体：即使坐标正确也无法判断物理含义。
- 只旋转腕部moment当尖端moment：漏掉力臂项。
- 写成(new−old)×F：符号反了；用直观已知力臂验算。
- 混用world位移和tool force叉乘：叉乘两输入必须同轴。
- 沿用位置4×4变换：力不加translation；moment的平移项是位移×力。
- 混用[force;moment]与[moment;force]：矩阵块顺序/下游接口错误。
- 比较不同参考点的reaction逐项反号：先统一点和轴。
- 把moment范数变化理解成负载变大：先检查是否只是换参考点。

## Robotics Context / Interview Capsule

腕部六维力传感器、工具尖端接触与payload重力需要换点换轴才能比较。
机器人关节力矩映射留S16.6；接触求解器的contact frame与force方向留S16.5。
一个力矩参考点变了，并不是力的作用点移动了；Modify则故意移动作用点检验力臂。

**30秒：** wrench是同轴表达的合力及关于指定点的力矩。本项目force在前。
同点换轴两向量都乘R；换点力不变，矩加(old−new)×F；reaction需同点同轴才取负。

**2分钟：** 先声明environment→robot、W/T与P/O/Q、N和N·m。
从(P−Q)×F+c推导shift公式，解释R_TW方向；给出默认三组moment。
说明两路径/往返与功率一致性检查，再用错误符号和漏平移的非零误差证明测试能检出错误。
最后区分换表示与移动负载，限定合成几何实验，不外推接触或真实传感器。

## Must Remember / My Verification / Run → Modify → Explain

wrench必须有方向、轴、点、顺序、单位；同轴才能叉乘；新→旧位移；
纯力偶矩不随点变；同点moment旋转只换分量；同载荷换表示不改变一致计算的功率。
Engineering与Learning状态见根README。2026-10-08 本人确认实验与预测完成，
并准确解释wrench物理契约、换点叉积与数值、逆旋转、纯力偶/reaction及功率一致性；
Run/Modify/Explain全部完成，Learning Mastered。
本次仅同步文档，runtime沿用2026-10-08 Zero工程验证，未重跑数值实验/GUI。

**Run：** 默认命令，找出JSON关于O/W、Q/W、Q/T三种wrench，并手算Q/W的Mx和My。

**Modify（只一项）：** 先预测只把P_x从.2改为.4后哪些F/M分量变化，再运行
`--lever 0.4`；解释为什么总力矩不全量翻倍，Q和c保持不变。

**Explain（五问）：**

1. 一个6D wrench需要注明哪些信息？本项目顺序与各分量单位是什么？
2. 为什么从O换到Q用(O−Q)×F？本例M_Q_W的Mx为何是+1.1？
3. 为什么W→T用R_WT.T？同点换轴与换参考点分别改变什么？
4. F=0纯力偶为何换点不变？reaction wrench什么时候可以直接取负？
5. 功率检查为何需要换点后的线速度？本实验为何不能证明真实接触测力正确？
