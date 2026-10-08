# S16.7 — Force Mapping Integration: Known Tool Load + Bounded Hold

[代码](../examples/17_force_dynamics/force_mapping_integration.py) · [阶段入口](19_force_dynamics.md) ·
[Engineering / Learning 唯一状态](../README.md#stage-16--force--dynamics-foundations)。

## Problem → Why → Intuition

现在把gravity、工具wrench、Jᵀ、motor gear/cap、实际mj_step串成一个力矩预算。
S16.6仅算映射，S16.3已经驱动有界motor；本课验证“施加外载荷后怎样保持home”。
重力前馈托住手臂重量，已知外载荷前馈抵消工具负载，PD处理位置/速度偏差；
若符号错或驱动不足，完整方程仍可成立但目标保持失败。
本课选Stage16计划中的**UR5e工具小外载荷**分支，未加入接触场景。
不实现impedance、force feedback、IK/trajectory、gripper或ROS。

## Core Concepts / Physical Contract

复用S16.3裸UR5e home与private motor模型、armature/passive、name/address映射；
contact和joint limits仍显式关闭。world gravity=[0,0,−9.81]m/s²，
前三关节cap±150N·m、后三±28N·m；shoulder-lift gear2，其余gear1。
力矩范围是模型配置，非此次验证的厂商硬件能力。

O为attachment_site原点，T为当前工具轴；载荷施加点P固连末端body，
P−O在T为[.1,.04,−.02]m，world位置随actual q变化。
用初始R_WT将F_T0=[4,−3,8]N、couple_T0=[.2,−.1,.3]N·m旋转到world，
之后**固定world载荷方向**，不跟随工具轴重旋转。
完整载荷F_W≈[4.000040,3.000015,−7.999974]N，couple_W≈[.200001,.100000,−.299999]N·m。

load envelope a(t)：0–.5s为0，.5–1s线性升至1，1–3s保持1，3–3.5s线性降至0，随后0。
力/力偶一起乘a(t)，三方案使用相同环境载荷定义、target、PD与cap。
由于actual姿态不同，工具偏置点位置/J和映射后的joint load可不同，不能要求各轨迹tau_load相同。

| 量 | shape / 单位 / frame、点 |
| --- | --- |
| actual q、v、qacc | (6,), rad / rad/s / rad/s²；按joint地址读取 |
| g、bias、passive | (6,), N·m；g来自独立零速度probe，actual bias含速度项 |
| F_W、couple_W | (3,), N / N·m；环境→机器人，力偶关于P |
| M_O_W | (3,), N·m；换点后的矩关于site O |
| Jp、Jr | (3,nv)，world表达、Jp速度点O，分量顺序linear/angular |
| tau_load / qfrc_applied | (nv,), 本模型N·m；唯独实际环境载荷进入该通道 |
| requested / actual motor torque | (6,), N·m；actual经scalar force cap与gear映射 |
| ctrl | (6,), motor scalar coordinate N·m，requested/gear，ctrl不限幅 |
| residual | (nv,), N·m；完整动力学方程残差，不是位置误差 |

controller使用已知合成载荷作为**oracle feedforward**，没有传感器或接触估计。
“计算控制需要的load”与“环境实际施加载荷”两条路径分开：手写Jᵀ供控制，
mj_applyFT写qfrc_applied供仿真，并逐步核对两者相等。
这里写qfrc_applied是允许的真实外扰模型，未向它写gravity/补偿力来绕过motor上限。

## Mathematics → Math-to-Code

```text
P_W = O_W + R_WT(q) offset_T
M_O_W = couple_P_W + (P_W-O_W) × F_W
tau_load = Jp_O_W.T F_W + Jr_O_W.T M_O_W
feedback = Kp*(target-q) - Kd*v

gravity_pd:        requested = g(q) + feedback
load_compensated:  requested = g(q) - tau_load + feedback
wrong_load_sign:   requested = g(q) + tau_load + feedback

ctrl = requested / gear
actual = gear * clip(ctrl, -joint_cap/gear, +joint_cap/gear)
M(q) qacc + bias(q,v) = actual + passive + tau_load + constraint
```

本fixtureconstraint=0、xfrc_applied=0、body_gravcomp=0，检查无隐藏施力来源。
当前g来自独立probe，actual q/v从不在控制loop覆盖。
Kp=[120,120,100,30,30,20]N·m/rad，Kd=[25,25,20,6,6,4]N·m·s/rad，沿用S16.3。
1ms physics/control，同步implicitfast；已知负载的前馈不是computed torque，未抵消C(q,v)v。

近静态、无饱和/passive耗散项时：

```text
gravity_pd:       Kp*(q-target) ≈ tau_load
load_compensated: q-target ≈ 0
wrong_load_sign:  Kp*(q-target) ≈ 2*tau_load
```

这解释有限位置刚度承受负载需偏移，符号错误会放大偏移；
不能保证wrong_sign偏移严格等于baseline两倍，因为tau_load随姿态变化。
动态时不能只算actual+load−g，应检查M qacc与完整bias/passive。

低cap=.1使shoulder-lift只有±15N·m：home无工具载荷时已需约−15.857N·m，
满载且正确补偿时需约−20.308N·m，均不可达。
已知load补偿正确也不能凭模型知识突破motor cap。

## APIs / Inputs / Outputs / Mutations

本例唯一local helper是[gravity_compensation.py](../examples/17_force_dynamics/gravity_compensation.py)：
导入build_model、gravity、JOINTS/KP/KD/DT；helper需要mujoco/numpy/matplotlib/mujoco-menagerie，
无更深local导入。保持关键J、外力、ctrl、forward/step调用在本课主循环可见。

| API | 行为 / 本课用途 |
| --- | --- |
| build_model(cap_scale) | 返回private compiled MjModel、q/v/actuator地址、gear与joint caps；其load API见S16.3 |
| MjData(model) | 返回mutable actual/probe，状态互不修改 |
| mj_resetDataKeyframe(model,data,home_id) | 原地初始化q/v/ctrl/time，返回None；随后清除旧position ctrl |
| mj_forward(model,data) | 原地刷新pose/forces/bias/qacc，不积分、不推进time |
| mj_jacSite(model,data,jp,jr,site) | 原地填world轴(3,nv)几何Jacobian，返回None |
| mj_applyFT(model,data,F_W,couple_P_W,P_W,body,target) | 原地累加(nv,)target，返回None；本课target就是actual qfrc_applied |
| mj_fullM(model,data,dst) | 当前binding原地填(nv,nv)惯量矩阵，用于完整预算 |
| mj_step(model,data) | 原地推进1ms、actual q/v/time，返回None |

每step清零qfrc_applied后再applyFT，避免累加旧step载荷；不重复写xfrc_applied。
先forward获取当前pose/J→构造载荷→写external/ctrl→forward→记录同刻预算→step。
CSV全部pre-step，t0–5s共5001行；loaded评估2.5≤t<3s，tail4.5–5s。
源：[官方applyFT](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft)、
[完整动力学](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework)。

## Minimal Experiment / Dependencies

只需requirements已声明mujoco/numpy/matplotlib/mujoco-menagerie；无安装或新依赖。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/force_mapping_integration.py
python examples/17_force_dynamics/force_mapping_integration.py --cap-scale .1
```

产物仅ignored tmp/s16_7_cap1/与cap0.1/：三份5001×68 CSV、results.json、load_hold.png。
CSV包含q/v/g/bias/passive/external/requested/actual/ctrl/qacc/完整残差，按JOINTS分组列。
图为最大joint error、shoulder-lift偏移、correct方案的gravity/load/requested/actual预算。

## Expected / Actual Result → Explanation

2026-10-08 / Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2，conda mujoco所属Python3.12.14；
MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2、Menagerie2026.9.2 metadata/import/module paths/API核验。
无安装/requirements改变，3.11兼容目标未用3.11执行。

| cap1 | loaded max error (rad) | peak error (rad) | tail max error (rad) |
| --- | --- | --- | --- |
| gravity + PD | .037185621 | .038389110 | 4.451e−5 |
| correct load feedforward + PD | 3.434e−18 | 3.454e−18 | 3.458e−21 |
| wrong load sign + PD | .074422293 | .076600913 | 8.729e−5 |

三方案默认均无饱和，载荷撤去后恢复；最大tail速度≤.001398rad/s。
正确方案在home、零速度、同模型oracle负载下几乎精确抵消，极小误差不表示真实硬件精度或鲁棒性。
正确方案loaded shoulder-lift：g≈−15.857087、load≈+4.450989、actual≈−20.308076N·m；
actual+load≈g。gravity_pd靠约+.037184rad偏移提供额外抵抗；wrong sign偏移约+.074420rad。
loaded静态预算误差correct≤1.12e−16N·m，baseline≤.001071、wrong≤.001846N·m，
后两者仍有小速度，静态预算只作近似描述。

低cap正确补偿：8262个饱和joint-samples、peak误差3.527149rad，tail误差3.048211rad、
速度2.994673rad/s；未恢复、未静止。低cap验收PASS表示限幅与保持失败被检出，不是性能成功。
六个case全部每步Jᵀ与applyFT一致误差≤2.67e−15N·m，功率差≤1.25e−14W，
完整动力学残差≤6.40e−14N·m；两条命令最终exit0，无import error。
已目视默认图，未进行GUI、真实传感器/接触、硬件、payload辨识或高增益/延迟稳定扫描。

## Failure Cases

- 把环境load映射项加到motor：本例放大偏移，应从gravity预算中减去。
- 正确前馈近零误差就声称鲁棒：外力来自精确oracle，没有噪声/未知扰动。
- qfrc_applied未清零：applyFT逐step累加载荷，制造假负载。
- 外力同时写qfrc_applied与xfrc_applied：双算环境力。
- 在外力通道添加controller缺少的补偿：绕过真实motor cap，掩盖不可达保持。
- 只看requested不看actual：饱和后动力学预算错误。
- 把动态静态预算的非零值当仿真故障：漏M qacc、速度bias或passive。
- 从失控低cap掉落fixture推断硬件安全：本模型关闭contact/limits，仅用于可重复反例。

## Robotics Context / Interview Capsule

已知工具重力/外载荷补偿与PD保持是常见力矩预算；真实load通常需传感器或模型估计，
需留驱动裕量。接触合力可使用S16.5的测量，但本课未将contact solver接入UR5e闭环。
Stage16工程知识链已串起，Learning仍需本人Run/Modify/Explain；不自动启动Stage17。

**30秒：** 在偏置工具点施加已知环境wrench，换点后Jᵀ得到load torque；
motor输出g−tau_load+PD并经过gear/cap。逐步核对外力映射、实际驱动力矩和完整动力学，
比较有限刚度偏移、正确前馈与错误符号，以及低cap失败。

**2分钟：** 声明environment→robot/world/P/O与单位，写r×F和Jᵀ，再写完整动力学预算。
解释外力由applyFT写实际外扰，motor只能走ctrl，gravity来自独立probe。
用−15.857+抵抗4.451→−20.308N·m说明符号；引用loaded/tail/saturation结果，
最后区分精确oracle近零误差与真实鲁棒控制，并限定无contact/硬件验证。

## Must Remember / My Verification / Run → Modify → Explain

load和motor通道分开；J/w同点同轴；补偿减external torque；actual力矩用于预算；
动态不是静态平衡；正确模型也不能突破cap；oracle前馈不是接触测力闭环。
Engineering见根README。2026-10-08 本人确认实验与预测完成，Run/Modify完成；
本人随后补齐第2项，准确区分载荷World-fixed与施力点tool-fixed，
说明力臂与Jacobian随q变化导致tau_load改变；五项Explain完成，Learning Mastered。
本课具体载荷约定：
初始化时以initial_rotation把F/couple转到World，之后两向量World-fixed；
P固连工具，用当前R_WT更新world位置，因此姿态改变时moment/J/tau_load仍会改变。
本次仅更新文档，runtime沿用2026-10-08 Zero工程验证，未重跑仿真或GUI。

**Run：** 默认命令，查看三种偏移与load撤去后的恢复；手算loaded shoulder-lift预算。

**Modify（只一项）：** 先预测cap缩至.1对home初始平衡、满载驱动需求、饱和和恢复的影响，
再运行`--cap-scale .1`；保持load/PD/gear不变，比较requested/actual与tail状态。

**Explain（五问）：**

1. 为什么正确控制律是g−tau_load+PD？本例shoulder-lift满载力矩预算如何计算？
2. 偏置P点的force/couple怎样换到O再映射？载荷方向为什么不会随工具重新旋转？
3. qfrc_applied与ctrl各代表哪种施力主体？本课为什么允许applyFT写前者，却不允许补偿力写前者？
4. gravity_pd为何有loaded偏移，错误符号为何通常更大？正确前馈近零误差证明了什么、没证明什么？
5. 动态完整预算与静态预算有什么区别？低cap预算仍正确为何保持失败？
