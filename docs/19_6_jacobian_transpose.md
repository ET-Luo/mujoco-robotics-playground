# S16.6 — Jacobian Transpose: Wrench → Joint Torque

[代码](../examples/17_force_dynamics/jacobian_transpose.py) · [阶段入口](19_force_dynamics.md) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

已会读取/变换工具wrench，怎样知道它给各个关节带来多少外力矩？
复用Stage10的UR5e geometric Jacobian，这次把运动映射转为力映射。
每个关节微小转动都会让工具产生微小平移/转动；外载荷在这些运动上做功，
其功对关节角的系数就是该关节的广义外力矩。这就是J转置，不是求J逆或再做IK。
本课只验证几何与虚功映射，不让机器人运动、不实现force controller。

## Core Concepts / Physical Contract

UR5e Menagerie home，attachment_site原点O，工具轴T由当前site_xmat给出，world轴W。
方向固定为**environment→robot**；`tau_ext=J.T w_ext`是外载荷的广义力矩，
不是motor command。若静态且仅重力与该外载荷，忽略其它项，驱动需求是
`tau_act=g(q)−tau_ext`；实际还要检查反馈、passive、constraint与motor限幅。
若希望robot→environment产生某wrench，必须先统一作用方向再讨论驱动平衡。

| 量 | shape / 单位 / frame、参考点 |
| --- | --- |
| q、虚拟qdot | (6,), rad / rad/s，name→id→qposadr/dofadr确定列对应关节 |
| Jp、Jr | (3,nv)，平移m/rad、角速度rad/rad；world表达，平移速度在site O |
| J=[Jp;Jr] | (6,nv)，geometric Jacobian，输出[v_O_W;omega_W] |
| wrench=[F;M_O] | (6,), N / N·m；与J同world轴，moment关于O |
| tau_ext | (nv,), 本例全hinge所以N·m；列对应DOF而非任意actuator顺序 |
| R_WT | (3,3)，列为工具轴在world的表达，T→W用R_WT |
| point P / offset | world点m / tool位移m，P固连工具末端body |

wrench的6个分量不是六个joint torque；即使两者都是(6,)也不能直接相等。
MuJoCo geometric Jacobian的角速度块不是Euler角速度。
本课J下半是world角速度，上半是O点速度；不要直接套moment-first教材的矩阵排列，
也不要混用空间twist中不同线速度参考点的Jacobian。

载荷在T定义F_T=[4,−3,8]N、纯力偶M_T=[.2,−.1,.3]N·m，
world表达用R_WT分别旋转；R取模型当前home，不人为假设与world平行。
四种case：site_force、site_couple、site_wrench（两者叠加）、offset_force（P处纯力，无独立力偶）。
默认P−O在tool表达[.1,.04,−.02]m；Modify翻倍该偏置，姿态/F/couple不变。

## Mathematics → Math-to-Code

运动关系与瞬时功率：

```text
v_O_W = Jp_W qdot
omega_W = Jr_W qdot
Power = F_W.T v_O_W + M_O_W.T omega_W
      = (Jp_W.T F_W + Jr_W.T M_O_W).T qdot
tau_ext = Jp_W.T F_W + Jr_W.T M_O_W
```

任意虚拟速度都应满足两侧功率相等，因此得到该映射；不需要J可逆。
每个joint也可用虚位移理解：`delta_work = F·delta_p + M·delta_theta_W = tau·delta_q`。
为独立核验，代码分别对六个关节做±1e−6rad FK差分，得到每列的work/rad，
而不是只重复`J.T w`。旋转差分用`dR R.T`的skew提取world微转动，避免Euler角近似混淆。

偏置点P的纯力必须先换点，或使用P点Jacobian：

```text
r_W = P_W - O_W
M_O_W = r_W × F_W
tau_ext = Jp_O_W.T F_W + Jr_O_W.T M_O_W
        = Jp_P_W.T F_W
```

同一点换轴时同时旋转J的行块与wrench：
`Jp_T=R_WT.T Jp_W`，`Jr_T=R_WT.T Jr_W`；
`F_T=R_WT.T F_W`，`M_O_T=R_WT.T M_O_W`，所得joint torque不变。
工具轴下的Jp仍描述同一物理O点，不能只换wrench而保留world J。

J转置在奇异姿态仍定义良好，不表示所有wrench都能被独立控制；
其核空间可能把非零wrench映射成零joint torque。驱动能力与接触可行性是另一个问题。
推导依据[Modern Robotics作者的静力学课程](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/)。
该教材的受力方向与moment-first排列应先转为本课约定，再比较公式。

## APIs / Inputs / Outputs / Mutations

| API | 输入、输出/原地改变 | 用途 |
| --- | --- | --- |
| mujoco_menagerie.load(name) | 编译模型→MjModel，依赖已有缓存或下载 | 复用既有UR5e结构 |
| MjData(model) | model→mutable q/v/derived arrays | actual快照与有限差分probe独立 |
| mj_resetDataKeyframe(model,data,key_id) | 原地初始化home状态/time，返回None | 不求IK选配置 |
| mj_forward(model,data) | 原地刷新pose等derived arrays，不推进time，返回None | 每个配置下求J与FK |
| mj_jacSite(model,data,jacp,jacr,site_id) | 原地填两个预分配(3,nv)数组，返回None；world轴 | O点速度映射 |
| mj_jac(model,data,jacp,jacr,point_W,body_id) | world点固连指定body，原地填(3,nv)数组 | P点直接Jacobian交叉核验 |
| mj_applyFT(model,data,F_W,M_point_W,point_W,body_id,target) | 原地**累加**到(nv,)target，返回None；F/M/world点均world，moment关于该point | API对照Jᵀw |

API依据[官方 functions](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft)。
`mj_applyFT`收到本课单独的零数组，只计算映射，不写actual qfrc_applied/xfrc_applied。
同缓冲调用两次会得到两倍结果；不要误以为覆盖或已执行动力学。
计算point force时传P、couple=0；若传O就必须同时传已经换到O的moment。
本课不调用mj_step，time0、actual qvel0、外力数组0均检查；虚拟qdot只是功率代入值。
采用既有full Jacobian写法而保留API可见；无旧课local helper导入，避免带入IK依赖链。

## Minimal Experiment / Dependencies

仅requirements已声明mujoco/numpy/matplotlib/mujoco-menagerie；无新依赖、ROS、IK求解器。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/jacobian_transpose.py
python examples/17_force_dynamics/jacobian_transpose.py --offset .2
```

产物仅ignored tmp/s16_6_offset0.1/与offset0.2/：results.json、jacobian_world.csv（6×6）、
joint_torques.csv（4×6）、torque_mapping.png。J上三行为Jp、下三为Jr；
torque CSV的行顺序在header给出，各列按JOINTS顺序。
图对照四类外力矩，不是actuator的实际输出图。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2，conda mujoco所属Python3.12.14。
MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2、Menagerie2026.9.2 metadata/import/module paths/API核验。
无安装/requirements改动；代码3.11兼容目标未用3.11执行。

| Case | joint external torque（按pan/lift/elbow/wrist1/2/3，N·m） | 功率（W） |
| --- | --- | --- |
| site_force | [−2.370013,4.910988,3.635988,.499999,−.4,≈0] | −.743800527 |
| site_couple | [−.299999,−.200001,−.200001,−.200001,.1,.3] | .041000275 |
| site_wrench | [−2.670011,4.710987,3.435987,.299997,−.3,.3] | −.702800252 |
| offset .1 point force | [−1.910008,4.650990,3.375990,.240000,.48,−.46] | −.670000362 |
| offset .2 point force | [−1.450004,4.390992,3.115992,−.019998,1.36,−.92] | −.596200197 |

虚拟qdot=[.1,−.2,.15,−.1,.05,.12]rad/s；功率两侧一致atol1e−12，actual robot没有运动。
两命令均exit0；applyFT/累加语义、同点world/tool axes、P点J与换点矩核对atol1e−12通过。
FK有限差分虚功最大误差≤1.78e−9N·m，独立验收atol2e−8。
漏偏置力矩造成joint torque误差范数1.183385489/2.366770979N·m；
混world J与tool wrench造成12.358167870N·m误差，主动检出两种错误。
纯力在attachment_site作用时wrist3 torque≈0，沿用其site位于该轴的几何性质；
加入力偶或偏置后该关节也可有明显torque，不能从纯力case推断wrist3永远不受载荷。
偏置翻倍只让offset_force−site_force这一附加项翻倍，不使整个joint torque向量翻倍。

无时间积分、motor执行/限幅、contact、负载恢复、GUI或真实硬件验证。
这些数值验证同模型几何映射，不能证明机器人能产生所需接触wrench。

## Failure Cases

- 把environment wrench映射值直接当抵抗负载的motor command：方向需反号并纳入gravity等预算。
- J world而w tool：矩阵shape正确但物理意义错。
- wrench moment关于P却用O点J：先换点或改P点J。
- `[moment;force]`与`[linear;angular]`直接配对：行顺序错。
- 忽略couple或偏置力臂矩：末端腕部扭矩估计失真。
- 以功率恒等式通过就宣称实现正确：同一错误J两侧也可能一致，需API与独立FK虚功核验。
- applyFT不清零target反复调用：累加外力矩而非覆盖。
- 想用J逆处理力映射：不需要逆；奇异性影响可控性，不取消虚功关系。

## Robotics Context / Interview Capsule

外力矩估计、接触load预算、笛卡尔力控制都用这个关系；真正执行还须motor映射/限幅、
反馈、接触可行性和稳定性。S16.7再将gravity、load与bounded hold串成Integration。

**30秒：** Jacobian映射joint速度到同点Cartesian速度；功率一致性给出tau_ext=Jᵀw_ext。
J/w必须同轴、同点、同顺序。环境负载的映射不是抵抗负载的motor command，后者还需完整动力学预算。

**2分钟：** 声明O/W/T与environment→robot，列Jp/Jr/F/M单位和shape。
从F·v+M·omega推导tau·qdot，再解释P点力的r×F换点，给出world/tool轴一致结果。
说明applyFT单独缓冲、有限差分虚功与功率检查的区别，最后指出无mj_step/驱动能力验证。

## Must Remember / My Verification / Run → Modify → Explain

力映射用J转置；同点同轴同顺序；纯力偏置产生矩；外负载和抵抗力矩不同号；
API累加不等于施加到actual；映射正确不代表执行可行。
Engineering见根README；Learning三项待本人明确报告。

**Run：** 默认命令，对照JSON四类joint torque、两侧power和virtual-work误差；查看图。

**Modify（只一项）：** 先预测偏置从[.1,.04,−.02]变成[.2,.08,−.04]m后哪些case不变，
再运行`--offset .2`；比较offset_force−site_force，解释为何总力矩不整体翻倍。

**Explain（五问）：**

1. 从功率/虚功如何推导tau=Jᵀw？为什么无需J逆？
2. 本课Jp/Jr与wrench的shape、单位、轴与参考点是什么？
3. P处纯力用O点J时为何加入(P−O)×F？偏置翻倍让哪一项翻倍？
4. 环境→机器人wrench得到的tau_ext与静态抵抗它的motor torque有什么符号关系？gravity如何加入？
5. mj_applyFT改变了哪个数组？功率恒等式、FK虚功与API对照分别证明什么，不能证明什么？
