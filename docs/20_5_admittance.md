# S17.5 — Admittance：测力 → 虚拟质量 → 运动参考

约0.5～2h，前置：[S17.4](20_4_contact_transition.md)。
[代码](../examples/18_compliant_control/admittance.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning状态只维护在[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

如果工具测到有人沿+Y推它，怎样生成一个可控、有限的让步运动？
前几课阻抗由位置/速度误差产生motor力；本课外层先把测力送进一个软件里的质量、阻尼和弹簧，
算出它应怎样运动，再把这个运动作为机器人inner loop的reference。
虚拟质量越大，同一净力下虚拟加速度越小；虚拟弹簧让reference有回到基准的趋势；
没有弹簧时，微小持续测力偏置也可能导致reference一直漂移。

## Core Concepts / Scope

本课用合成测力，避免把contact求解、触碰状态机与新算法同时混进来。
输入是理想world环境→机器人+Y测力通道，不是真实传感器或mj_contactForce读数。
此输入**只驱动软件外层，不同时施加为物理外力**；实际MuJoCo装置没有contact/外力。
因此验证的是参考生成与自由空间跟踪，不是实际受推机器人或闭环接触交互。

复用cartesian_spring XML/DT/GEAR/CAP，两个slide固定orientation，真实M=diag(2,1)kg。
本课实际初值q/v=0，X固定为零，Y跟踪生成reference；零gravity、contact和passive damping。
依赖链只有requirements已声明的mujoco/numpy/matplotlib，helper用Agg，无新依赖。
虚拟M_v默认1kg、D_v4N·s/m、K_v20N/m；Modify只将M_v改2kg，K_v/D_v不变。
M_v是软件参数，不是修改model.body_mass，也不是实际惯量整形保证。

| 控制层 | 输入 | 输出 | 本课参数 |
| --- | --- | --- | --- |
| admittance外层 | F_measured,W（N） | z / u，即位移/速度reference（m、m/s） | M_v、D_v、K_v |
| motor跟踪内层 | reference−actual（m、m/s） | world force → Jᵀ → ctrl | K_inner400N/m，D_inner[56.568542,40]N·s/m |

```mermaid
flowchart LR
    F[合成 world +Y 测力] --> V[虚拟质量 阻尼 弹簧]
    V --> B[可选 限速 限位]
    B --> R[位置与速度 reference]
    R --> C[inner motor 跟踪]
    C --> M[MuJoCo 实际动力学]
    M --> C
```

外层六个有限case：±.4N脉冲（.5…2.5s）、.2N恒定bias+弹簧、
.2N bias无弹簧/无限制、同bias无弹簧/有界、4N强脉冲/有界。
无限制case是漂移与解析比较对照；不能把所有case都称为有界controller。

## Mathematics：意义 / shape / unit / frame

这里只学习world Y标量，基准x₀=0，z=x_ref−x₀（m），u=zdot（m/s）：

\[
M_v\dot u+D_vu+K_vz=F_{measured,Y},\qquad \dot z=u.
\]

F为环境→机器人world +Y分量N；M_v kg、D_v N·s/m、K_v N/m，各项都是N。
正输入从静止起得到正加速度，参考沿+Y；负输入反向。输入符号错会朝相反方向让步。
未限幅连续虚拟能量E_v=½M_vu²+½K_vz²，E_dot=F u−D_vu²。
它是软件模型的能量，不等于机器人机械能；有界投影后的递推也不完全满足这个等式。

恒定输入且K_v>0：稳态u=0，z=F/K_v；.2N/20N/m=.01m有限偏移，不能称为零误差回原点。
力撤除后参考趋回基准。K_v=0时没有恢复项：

\[
u(t)=\frac F{D_v}(1-e^{-D_vt/M_v}),\quad
z(t)=\frac F{D_v}\left[t-\frac{M_v}{D_v}(1-e^{-D_vt/M_v})\right].
\]

本课bias=.2N，最终速度趋向.05m/s，位置随时间漂移；有阻尼并不保证位置有界。
K_v=0撤力后速度会衰减，但通常留下永久位移；本课bias一直保留，未另做撤力case。

未限幅半隐式更新（h=.001s）：

\[
a_n=(F_n-D_vu_n-K_vz_n)/M_v,\quad
u^*_{n+1}=u_n+h a_n,\quad z_{n+1}=z_n+h u^*_{n+1}.
\]

有界case先clip u*到±.05m/s，再clip候选z到±.04m，最终存储
u_{n+1}=(z_{n+1}−z_n)/h。到位移边界时不继续积累隐藏的向外速度，
参考停住；边界处向内净力仍可使其离开。边界不是力平衡点。
限位步骤可能产生速度/加速度突变，未增加加速度或jerk界，不能当硬件安全轨迹生成器。
这些是reference界，**不保证实际robot位置/速度严格同界**；实际跟踪误差必须单独记录。

inner loop：v_W=Jp qvel，F_inner=K_inner(x_ref−x)+D_inner(v_ref−v)，τ_req=JpᵀF_inner。
x、v为world (3,)m、m/s，Jp(3,2)，τ_req(2,)slide force N，gear=[2,1]、joint cap20N。
实际自由空间M qdd=τ_actual；本课不把合成测力加到右侧，也不做F_measured的motor抵消。

## Math-to-Code / APIs

```python
acceleration = (force-D_V*velocity-stiffness*z)/mass
trial_velocity = velocity + DT*acceleration
limited_velocity = np.clip(trial_velocity, -V_MAX, V_MAX)
next_z = np.clip(z + DT*limited_velocity, -Z_MAX, Z_MAX)
next_velocity = (next_z-z)/DT
```

上段仅有界case启用clip，无界case保留原递推。
virtual z/u是独立标量；MjData.qpos/qvel始终是实际状态，执行中不重设它们。
本步先由当前virtual z/u算inner ctrl，再mj_step；当前测力产生的next_z/u供下一时刻使用。
CSV是pre-step：同一行的测力、参考、actual与inner force；raw_virtual_accel为投影前计算，
不能据它直接推有界更新后速度。speed_limit_next/position_limit_next对应这一行到下一行的更新。
末行保存候选flag但不执行更新，metrics只计前10000次更新。

MjModel.from_xml_string(XML)返回编译模型；MjData(model)返回可变物理状态buffer。
mj_forward原地更新site pose/动力学、不推进时间；mj_jacSite写world线/角J到(3,nv)buffer，
不返回J；mj_step原地更新qpos/qvel/time。所有controller均每1ms更新。
J、gear、actuator cap与物理加速度均按实际API buffer核验；没有新增框架或高层solver。

## Minimal Experiment / Expected → Actual

仓库根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/admittance.py
python examples/18_compliant_control/admittance.py --virtual-mass 2
```

每命令六case，10001×13 CSV（0…10s）、results.json与admittance.png。
Expected：正负脉冲镜像；有弹簧bias稳态10mm；无弹簧bias漂移；
bounded reference遵守40mm/50mm/s；改变虚拟mass影响瞬态而不改变F/K稳态偏移。
质量翻倍、D/K固定时ζ=D/(2√(M_vK_v))降低，不能把改变M_v称为同ζ实验。

Actual：本次工程结果见下表；图表展示测力、reference/actual和reference速度。

2026-10-08，DESKTOP-781D67A；默认与--virtual-mass 2均exit0，无import error：

| case | 默认M_v1：峰值 / 末reference mm | M_v2：峰值 / 末reference mm | 行为 |
| --- | --- | --- | --- |
| positive | 24.1494 / ≈0 | 27.0143 / −.00978 | 正向响应，撤力后趋回基准 |
| negative | 同幅反号 | 同幅反号 | virtual/actual/force逐点镜像 |
| bias_spring | 12.0747 / 10.0000 | 13.5072 / 10.0001 | 持续偏置导致有限offset |
| bias_drift | 487.550 / 487.550 | 475.050 / 475.050 | 无弹簧reference持续漂移 |
| bias_bounded | 40 / 40 | 40 / 40 | 持续偏置下卡在位移界，末reference速度0 |
| strong_bounded | 40 / ≈0 | 40 / −.01511 | 先限速/限位，撤力后释放并趋回 |

positive峰值reference速度.051449/.041721m/s（M_v1/2）；未启用限速，不声称其≤.05。
质量翻倍后峰值位移反而更大：D/K固定使ζ从.447214降到.316228，瞬态overshoot增大。
这并不违背同一净力下加速度更小；不能把“质量更大”推成“所有位移都更小”。
持续无弹簧bias两组末reference速度均约.05m/s，终端速度F/D与M_v无关，M_v影响过渡。

strong_bounded参考峰值速度.05m/s、位移.04m，两组speed limit更新1356/1308次、
position limit1195/1188次；bias_bounded限位8955/8742次、未触发限速。
边界采样的浮点误差约1e−15m/s，脚本按1e−12容差检查。
strong_bounded实际峰值速度.056794/.056737m/s，明确高于reference限速；
最大实际/reference位置差.922834/.922793mm。其余case tracking error最大<.901mm，motor全组无饱和。
峰值motor力全组≤2.010896N；position projection没有让真实qpos瞬移。

检查：J/Jr、motor cap与qacc、零bias/passive/contact/external、shape/clock/finite、
真实半隐式q/v递推与reference位移速度一致性通过。
未限幅四case对照独立连续阶跃解（脉冲用两个延迟阶跃相减），两mass最大reference误差≤.050mm。
独立CSV虚拟递推、inner控制/实际动力学/virtual能量、限位限速flag核对通过。
额外对±位移边界做outward拒绝、inward释放静态probe通过；不是完整负向动态饱和实验。
CLI mass0/nan各exit2，默认PNG目视检查、本地文件链接与git diff --check通过。
产物仅ignored tmp/s17_5_m1与m2，临时日志/tmp；无需GUI。

现场WSL2 Ubuntu24.04.5/kernel6.6.87.2，Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2；
当前conda hook激活mujoco，同shell检查interpreter、distribution metadata/真实module paths。
脚本实际运行核验了MuJoCo API；无安装/requirements改动，未在Python3.11运行。

## Explanation / Failure Cases / Robotics Context

Admittance定义的是测力到运动reference的映射；inner loop仍可用阻抗式motor控制。
不能仅因最终仍输出torque，就把整个外层称为直接impedance。
物理跟踪并非理想位置源，finite bandwidth导致滞后、速度超调；虚拟M_v不等于真实运动的有效质量。
本合成测力不来自actual位置/环境，没有验证环境反馈造成的耦合稳定性。

K_v>0的测力bias变成稳态offset；K_v=0的bias变成长期漂移。限位只能阻止reference继续出界，
没有消除sensor bias，也没有继续满足原来的无界虚拟微分方程。
在边界继续累积未使用速度会导致释放时异常运动，本课存储投影后实际reference速度避免隐藏状态。
速度参考不是对真实速度的硬限幅；实际安全还需独立监测、停止策略与轨迹约束。
错误力符号、frame混用、传感器bias、粗dt、延迟、噪声都可能改变行为；本课没有噪声滤波/去偏/接触注入。
应用是人工拖动、协作让步或力驱动运动接口；接触任务仍需验证environment/inner/outer整体闭环。
无GUI、ROS、UR5e、真实测力/物理外力/硬件验证，也不证明通用稳定性或passivity。

## Interview Capsule

30秒：Admittance先由测力解虚拟M a+D v+K z=F，产生运动reference，再由motor内层跟踪。
与位置误差产生力的impedance区分输入输出。持续偏置下有K停在F/K，无K持续漂移；
限位限速约束reference，并不保证真实机器人同样有界。

2分钟：说明world环境→机器人力符号和各单位，区分软件z/u与MjData实际qpos/qvel。
写出半隐式积分与速度/位置投影，解释边界速度回写与inward释放。
指出虚拟质量翻倍但D/K不变会降低ζ，峰值位移可能增加；偏置稳态F/K与质量无关。
用正负脉冲、bias offset/drift、有界强输入及actual误差作证，最后说明合成输入没有验证接触整体稳定性。

## Must Remember / My Verification — Run / Modify / Explain

测力→运动reference是admittance；位移/速度误差→力是impedance。
虚拟质量不是物理质量；有阻尼不等于位置有界；reference界不是actual安全保证。
Engineering Complete；Learning Run/Modify/Explain等待本人完成，状态只见根README。

Run：运行默认命令，查看tmp/s17_5_m1/admittance.png与results.json，定位offset、drift和boundary三种行为。
Modify：预测只把virtual mass1→2kg如何改变脉冲初始加速度、ζ、峰值位移/速度、bias稳态偏移，
再运行--virtual-mass 2；保持physical model、D/K、输入与bounds不变，比较预测与实际。
Explain：

1. Admittance与impedance的输入/输出分别是什么？为什么admittance仍需inner tracking loop？
2. M_v与MuJoCo真实质量怎样区别？只将M_v翻倍而D/K不变，初始加速度与ζ怎样改变？
3. 为什么有K的持续测力bias产生F/K偏移，而无K仍会漂移？阻尼为何不足以固定位置？
4. 到reference位移边界时为什么要回写投影后的速度？reference限速能保证actual速度不超界吗？
5. 正负力在world Y上应产生怎样的让步？本课合成测力验证能否证明真实接触/硬件闭环稳定？

下一小任务S17.6 Normal force control等待明确请求，不自动实现。
