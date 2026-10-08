# Stage 16 — Force & Dynamics Foundations

P2核心问题：**How does a robot physically interact with the environment?**
本阶段从actuator的物理输入/输出走到接触wrench和Jᵀ；既有UR5e运动学/PD不重复。
[审查与规划](p2_plan.md) · [例子README](../examples/17_force_dynamics/README.md) ·
[状态唯一来源](../README.md#stage-16--force--dynamics-foundations)。

## 阶段任务 skeleton

每项0.5～2小时；S16.1 Engineering Complete + Learning Mastered（2026-10-08 本人确认），S16.2 Engineering Complete + Learning Mastered（2026-10-08 本人确认）；S16.3 Engineering Complete、Learning待本人验证；S16.4起未实现、未验证。

| Task | 主要概念 / 最小实验计划 | 验收设计与Integration |
| --- | --- | --- |
| S16.1 Position vs torque control | 同mechanics同load，servo/显式motor feedback/open torque | ctrl→scalar actuator output→joint torque，gear/限幅/恢复；完整包见下 |
| [S16.2 Manipulator dynamics](19_2_manipulator_dynamics.md) | 先单hinge，`M(q) qdd + b(q,qdot) = tau_act + tau_passive + tau_ext + tau_constraint` | M/inertia、bias、passive分项/单位；质量或重力单变量对照，静态平衡与加速度 |
| [S16.3 Gravity Compensation Integration](19_3_gravity_compensation.md) | UR5e motor替换/映射，zero torque vs gravity feedforward vs hold反馈 | `qfrc_bias(q,0)=g(q)`；运行态bias还含速度项；按motor gear与限幅施加，不用无限qfrc_applied掩盖能力 |
| S16.4 Force / torque / 6D wrench | 已知world point force与moment，换轴/换参考点 | `[F; moment]` shape6，N/N·m；`moment_new=moment_old+(p_old-p_new)×F`，方向/点明确 |
| S16.5 MuJoCo Contact Wrench | box-plane与一个finger contact fixture | mj_contactForce全6分量、contact frame、geom作用方向、world合成；静态support≈mg与action/reaction |
| S16.6 Jacobian Transpose | 复用UR5e site J，已知工具wrench | `tau=J_p.T F+J_r.T moment`，虚功/功率与点force交叉核验；不是J逆/IK |
| S16.7 Force Mapping Integration | UR5e静态tool小外力/简单接触 | gravity+motor+tool load/contact→joint balance与有界hold；测力/施力分离，不实现Cartesian impedance |

S16.4–.7每课再按下方Sprint格式展开API/实验/Actual/失败分析，不预填数值或PASS。

# S16.1 — Position Command vs Torque Command: same physics, different interfaces

## Problem

原有UR5e `ctrl` 是position target，而S3 motor的 `ctrl` 是torque。
P1还额外写 `qfrc_applied=qfrc_bias`。这三种写入究竟控制了哪种物理量？
本课只回答第一条边界：一个位置servo最终也必须产生joint torque；
选择motor接口后，反馈律由谁计算？扰动来临时输出和状态如何变化？

## Why

接触时只看target和qpos会漏掉驱动力矩、力矩上限以及外部负载。
P2后续要主动控制力，必须先分清command、actuator输出和外力。
这不是重新推导PD：沿用已掌握的PD式，验证两种actuator在同物理条件下的关系。

## Intuition

同一个可旋转的转子，初始正好在0.3rad。
位置servo在模型内部根据偏差算恢复力矩；motor feedback在Python里算同一个反馈力矩。
open-loop motor保持零torque，只在初始无负载时恰好平衡。
1–2秒外部施加+0.8N·m，会把转子推向正角度；两种反馈产生负torque抵抗它。
2秒撤去外力，反馈回到target；零torque无法知道应该回哪一个角度。

## Core Concepts / Physical meaning / Frames / Units

| 量 | 本模型shape / 单位 / 含义 |
| --- | --- |
| qpos / qvel | (1,)；rad / rad/s；绕固定world +z按右手规则转动 |
| position ctrl | (1,)；rad，gear=1 servo的角度reference，不是torque |
| motor ctrl | (1,)；本理想hinge transmission的scalar actuator torque coordinate，N·m；gear=2时joint torque是它的2倍 |
| actuator_force | (1,)；scalar output p；本hinge模型按N·m解释，尚未乘gear；通用actuator不可统一叫N |
| qfrc_actuator | (1,)；joint实际actuator torque，N·m，含force clamp与transmission映射 |
| qfrc_applied | (1,)；外部直接施加的generalized joint torque，N·m；本课为正负脉冲 |
| Kp / Kd | N·m/rad / N·m·s/rad；对同一joint的恢复与active damping |
| passive damping b | 0.1N·m·s/rad；模型自带黏性阻尼，独立于controller Kd |

本fixture没有gravity、contact、joint limit、其它actuator；固定轴使torque方向不含糊。
角度是连续hinge坐标，open loop可转过一圈以上，不是UR5e安全关节范围。
`qfrc_applied`人为造负载，**不是mj_contactForce测量、传感器读数或Cartesian wrench**。

## Mathematics

已知PD，目标速度为0：

```text
requested_tau = Kp (q_target - q) - Kd qdot
position servo (gear 1): p = clip(requested_tau, -1.5, +1.5), tau_act = p
motor feedback (gear 2): ctrl = requested_tau / 2
                         p = clip(ctrl, -0.75, +0.75), tau_act = 2 p
open-loop motor:          ctrl = 0, tau_act = 0
```

gear=2不是免费增加物理能力：这里同时把motor scalar force range减半，
使两个实现具有相同joint ±1.5N·m上限。力矩command可能超过上限；data.ctrl不等于clamped输出。
本模型只有0.04kg·m²固定惯量与passive damping，平衡时qdot/qdd均为0：

```text
0 = -Kp (q - q_target) + tau_ext
q - q_target = tau_ext / Kp
```

+0.8/20=+0.04rad，位置稍偏正向才能生成−0.8N·m恢复力矩。
“位置控制”不保证负载下零误差；本课没有积分或外力feedforward。
撤去负载后两feedback逐渐恢复。open loop即使最后速度趋零，也没有恢复角度的弹簧。
完整manipulator动力学、gravity/bias留S16.2–3。

MuJoCo position shortcut的gain/bias、motor transmission和限幅依据
[official XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position)与
[actuation model](https://mujoco.readthedocs.io/en/stable/computation/index.html#actuation-model)；
本机3.14.0的compiled参数与同状态force读数已现场核验。

## Math-to-Code / API

代码：[actuator_semantics.py](../examples/17_force_dynamics/actuator_semantics.py)。
`build_model`生成同一机械fixture，只变actuator；不导入旧课helper。

| API / array | 输入与输出/状态变化 | 本课为什么需要 |
| --- | --- | --- |
| MjModel.from_xml_string(xml) | 输入MJCF字符串，返回compiled model；nq=nv=nu=1 | 检查真实gain/bias/gear/force ranges，不靠actuator名字猜 |
| MjData(model) | 输入model，返回mutable data；含qpos/qvel/ctrl/forces | 每模式独立状态、同initial condition |
| mj_forward(model,data) | 无新状态返回；原地刷新当前state的力与其它derived quantities，time不推进 | 初始化、同状态force核对、pre-step记录对齐 |
| mj_step(model,data) | 读取当前command，原地推进1ms、更新time/qpos/qvel，返回None | 动力学响应；全程不在loop里覆盖qpos |
| model.actuator_trnid/gear/gainprm/biasprm | compiled arrays；此例1个hinge与1个drive | 核验servo与motor实际配置/映射 |
| data.actuator_force/qfrc_actuator | forward求得的scalar/joint outputs | 直接核对tau=gear*p、夹断后actual torque |

模型用Euler dt=1ms；两feedback同一步以当前q/v算力。
若换成implicitfast，内置servo的velocity feedback与外部Python显式反馈可能有不同数值处理；
不能把这次等价结果推广为任意integrator/dt都逐点一致。
本课为了透明比较用小dt，未证明Euler高刚度接触稳定。

CSV全是pre-step sample：先写ctrl/load→mj_forward→读forces/state→mj_step。
记录t=0至4秒共4001行，外力在1≤t<2秒，恰好1000步；最后一行不再step。
不把旧step的force和新step的q混成同一个测量。

## Minimal Experiment / Dependencies

依赖只需requirements已声明的mujoco/numpy/matplotlib；无安装、无ROS2/Menagerie/OpenCV。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/actuator_semantics.py
python examples/17_force_dynamics/actuator_semantics.py --kp 40
```

产物：ignored `tmp/s16_1_kp20_load0.8/` 与 `tmp/s16_1_kp40_load0.8/`，
每组三份CSV、results.json、response.png。CSV的ctrl不能跨模式直接用同一种单位解释。
图第一行包含open loop全幅，第二行放大feedback偏移，第三行actual actuator与external torque。

## Expected / Actual Result

2026-10-07 / DESKTOP-781D67A，WSL2 Ubuntu24.04.5、Python3.12.14、
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2；metadata、module paths、相关API已核验。
代码3.11兼容目标，此次未用3.11解释器。

| Case | 负载末段平均q−target（rad） | peak abs actuator torque（N·m） | 末0.5s max error（rad） |
| --- | --- | --- | --- |
| Kp20 position/feedback | +0.0400000455 | 0.969718 | 3.32e−11 |
| Kp40 position/feedback | +0.0199999990 | 1.055229 | 1.03e−11 |
| Kp20 load−0.8 position/feedback | −0.0400000455 | 0.969718 | 3.32e−11 |

三条命令均exit0、engineering_checks_passed=True，无import error；
第三条为`python examples/17_force_dynamics/actuator_semantics.py --external-torque -0.8`。
servo与motor feedback最大q/v元素差：default8.66e−16、Kp40 2.55e−15、反向1.01e−15。
open loop final q=8.280090rad（负向−7.680090rad）；Kp对其无影响。
末0.5s仍有max speed0.173460rad/s，不能把它描述为4秒已静止。

static两个probe保持time0：未饱和requested−0.2N·m，servo p=−0.2、gear2 motor p=−0.1；
饱和requested+3N·m，servo p=+1.5、motor p=+0.75，两者joint torque均+1.5。
动态脉冲没有达到饱和；**饱和验证仅来自同状态probe**，不伪称已测试饱和接触稳定性。
图已目视核验，无GUIviewer测试。独立CSV/数值/输入检查见交接记录。

## Explanation / Stability

位置servo本身是闭环torque发生器，显式motor反馈在相同物理增益/限幅下可以做同一件事。
“torque control”描述输入接口，不意味着open loop，也不意味着力已准确传到environment。
Kp翻倍，稳态偏移约减半；Kd固定时动态阻尼比也改变，peak torque在这次实验反而略增。
稳态误差改善不能推导出所有transient更好；本课没做完整stiffness/damping sweep。
本fixture正Kp/Kd/passive damping帮助耗散，自由空间小dt实测收敛；
contact stiffness、delay、coupled mass或离散步长改变后，须重新测峰值与振荡。

## Failure Cases

- 把servo ctrl=0.3当0.3N·m，或者把gear2 motor ctrl直接当joint torque：单位/映射错。
- 忘记gear时，motor施加2倍torque；复制forcerange也会改变joint物理上限。
- 想仅提高Kp消掉稳态负载误差：有限Kp仍需非零偏移，且可能先饱和。
- 看到qvel变小就认为open loop回到target：passive damping只耗散运动，不提供恢复角度。
- 混记post-step q和pre-step torque，或者初始化之外覆盖qpos：误判物理响应。
- 外力方向写反、阻尼正反馈、加大dt：可能增加偏移/振荡；这些未在本课作失稳扫描。
CLI仅允许已验证Kp20/40与load±0.8；nan、无穷值、其它数值拒绝，避免把受限验收称通用参数测试。

## Robotics Context

工业position接口隐藏内置servo；torque接口允许自行设计力相关反馈，但仍需要模型、
映射、限幅与稳定性分析。P1额外generalized bias补偿不经过motor torque cap；
P2将明确驱动可提供的torque与外界施加的力，本轮不改旧集成。
本课是actuator接口实验，无真实motor电流/带宽/摩擦辨识，非力控制或contact操作能力证明。

## Interview Capsule

**30秒：** ctrl语义由actuator定义。这里position servo输入角度，内部产生PD力矩；
gear2 motor需把joint torque除以2再输入。同负载下两个feedback一致，零torque open loop不回位。

**2分钟：** 从1DOF机械fixture和world+z方向开始，说明q/qdot与gain单位，写出PD与tau=gear*p。
力矩限幅在scalar output上，所以motor与servo的forcerange需要按gear换算。
静态负载要求恢复torque抵消外力，有q−target=tau_ext/Kp的偏移；load撤去后反馈回位。
用同状态probe验证饱和，动态数据验证恢复；区分position/motor command、actual joint torque、
external torque，最后限定Euler小dt自由空间结果，不外推contact或hardware。

## Must Remember

ctrl不是固定物理量；actuator_force不是一律N；joint torque要经过transmission；
motor接口也可以闭环；有限刚度允许偏移；初始化之外不要用qpos赋值替代控制。

## My Verification / Run / Modify / Explain

Engineering结果见上；Learning状态仅根README。2026-10-08 本人明确确认实验与预测完成，
并正确回答全部五项Explain：ctrl语义/gear/限幅、负载平衡方向、饱和输出、
open loop与被动阻尼、单DOF自由空间验证的适用边界。Run/Modify/Explain完成，Learning Mastered。
本次仅更新文档；runtime沿用2026-10-07工程验证，未重跑实验或GUI。

**Run：** 运行默认命令，查看results.json和三行图，找出1秒开始施力、2秒撤力的行为。

**Modify（只一项）：** 先预测把Kp20改成40后负载末段偏移、恢复过程、peak torque与open loop会怎样变，
再运行`--kp 40`，以loaded_mean_offset、peak torque和图核对，Kd保持不变。

**Explain（五问）：**

1. position servo和gear2 motor中ctrl分别表示什么？为何它们可以产生相同joint torque？
2. 正外力矩作用时，q偏向哪边、恢复torque朝哪边？负载稳态偏移为何是tau_ext/Kp？
3. requested torque=3N·m时，两个actuator的scalar output和joint output分别是多少？
4. open-loop torque为何撤去外力后仍不回target？速度减小证明了什么、没有证明什么？
5. 这次自由空间等价结果为何不能直接证明UR5e接触稳定性？改变integrator/dt时应重查什么？

完成三项后由本人明确报告；本轮到此停止，不自动实现S16.2。
