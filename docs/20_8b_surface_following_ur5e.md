# S17.8b — Surface Following Integration / UR5e

约0.5～2h；前置：[fixture扫描](20_8a_surface_following_fixture.md)、[UR5e motor](19_3_gravity_compensation.md)、[Jᵀ](19_6_jacobian_transpose.md)。
[代码](../examples/18_compliant_control/ur5e_surface_following.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning 状态以[根README](../README.md#stage-17--compliant--contact-rich-control)为准。

## Problem → Why → Intuition

fixture中切向和法向是两条独立slide；UR5e由六个旋转关节共同产生工具运动。
怎样保持上一课“沿表面往返30mm、压住2N”的任务，又真正通过关节motor执行？
直觉：外环仍按工具处的测力修正normal参考；内环改成Cartesian力/姿态力矩，经当前Jacobian转成六关节力矩。
工具位置与关节角的关系是非线性的，惯量也随姿态变化，不能把fixture的独立轴结论当作机械臂保证。

## Core Concepts / scope

裸UR5e+固定球形probe，半径20mm、质量50g；仅probe-plane接触，condim1无摩擦。
probe球心是`scan_tip`，不把球心误称为几何接触点。所有原有arm碰撞mask置零；不提供自碰撞/环境避碰保证。
probe刚性连接在wrist_3_link，body局部位置[0,.14,0]、quat[-1,1,0,0]；
attachment_site原本在[0,.10,0]且同quat，因此球心沿attachment的local +Z偏置40mm。
plane水平，world z=.22m。surface原点[-.45,.20,.22]m，t=world+X、n=world+Z、b=world−Y。
工具local Z朝下，姿态目标R_WT=diag(1,−1,−1)。先用既有DLS IK求球心[-.45,.20,.25]m的初始q。
IK只在private MjData中运行，初始化actual q一次；不验证home→initial运动，不在扫描中用IK写qpos。
初始normal坐标30mm，球面间隙10mm；approach速度10mm/s，独立于force-loop参考速度cap20mm/s。
执行全程motor torque + mj_step；存在真实gravity、关节armature与姿态相关M、bias。

复用S17.8a的quintic往返与S17.6的force_velocity、参考界；状态为approach→load→scan→hold/failed。
不同于二维fixture，binormal位置也需保持0，姿态需保持固定；切向与binormal由位置任务控制，normal只由力反馈生成reference。
当前模型需要requirements已声明mujoco/numpy/matplotlib/mujoco-menagerie；
helper链包含surface_following/normal_force/contact_transition/cartesian_spring、pregrasp_motion、gravity_compensation，均只导入这四包。
无ROS/新依赖/CUDA/高层controller框架，Agg headless。

## Mathematics：shape / meaning / unit / frame

```text
R_WS = [[1,0,0], [0,0,-1], [0,1,0]]  # columns t/n/b in world
x_S = R_WSᵀ (x_W − p_WS)             # (3,) m; nonzero origin matters
v_S = R_WSᵀ Jp_W qvel                # (3,) m/s
ω_W = Jr_W qvel                      # (3,) rad/s
f_n = (R_WSᵀ F_env_W)[1]              # scalar N; environment pushes upward
v_ref_n = clip(−.006(2−f_n), −.02, .02) m/s
n_ref_next = clip(n_ref + dt*v_ref_n, .005, .06) m
x_ref_S = [quintic_tangent, force_generated_normal, 0]
F_cmd_S = K_S (x_ref_S−x_S) + D_S (v_ref_S−v_S)
e_R_W = log(R_target_W R_current_Wᵀ)∨ # (3,) rad; reused world rotation-vector helper
M_cmd_W = K_R e_R_W − D_R ω_W         # (3,) Nm; moment about scan_tip
τ_task = Jp_Wᵀ R_WS F_cmd_S + Jr_Wᵀ M_cmd_W
τ_req = qfrc_bias + τ_task             # (6,) Nm, simulated g(q)+c(q,qvel)
ctrl[actuator_ids] = τ_req / gear
```

K_S=[1000,400,1000]N/m、D_S=[80,40,80]N·s/m，K_R60N·m/rad、D_R12N·m·s/rad。
现有force_velocity的gain.003在UR5e中乘2并重新clip，等效gain.006m/(N·s)，速度界保持20mm/s；
controller结构复用，增益根据本arm实验调整，不假定fixture参数可直接泛化。
Jp/Jr均(3,6)，每列对应dof地址，单位分别m/rad与rad/rad；位移m与旋转rad不能共用同一数值增益假装同单位。
完整bias feedforward来自当前仿真状态，含gravity及速度项；它不含真实contact reaction，不减掉`qfrc_constraint`。
这属于理想模型补偿，不声称实际机器人能够精确得到该项。
真实动力学：M qacc+bias = actuator+passive+constraint（本课无外力输入）。
虚功：τ_taskᵀ qvel = F_cmd_Sᵀ v_S + M_cmd_Wᵀ ω_W。

normal球面接触力沿球心法线，condim1没有摩擦或接触力偶，因此球心处contact moment为0，
`qfrc_constraint = Jp_at_sphere_centerᵀ F_env_W`。这依赖本课几何；一般偏置/摩擦接触需含力臂与力偶。
姿态控制仍需要Jr项，不能由“球心contact moment=0”推断姿态无需控制。

## Math-to-Code / APIs and mapping

`mujoco_menagerie.get(...).xml('ur5e')`返回本机缓存XML路径；不复制资产入Git。
`MjSpec.from_file(path)`返回可修改的模型描述，`add_body/add_geom/add_site`添加刚性probe和plane，`compile()`返回MjModel。
`MjData(model)`返回实际或IK状态对象；状态对象之间分开，几何IK写入不影响动态执行中的actual状态。
按joint name→joint id→`jnt_qposadr/jnt_dofadr`取地址；按actuator name取id，再核验`actuator_trnid`连接的joint。
不能把site id、joint id、qpos地址、dof地址和actuator id互换，即使本模型数组顺序恰好相同。
编译general位置servo改为无动态、固定gain1、零bias motor；gear=[1,2,1,1,1,1]。
保留真实actuator force限制，并除gear变换成对应范围，使广义力矩cap仍是原model cap。

`mj_forward(model,data)`返回None，原地刷新位姿/接触/加速度，不推进时间；
outer input取previous ctrl的fresh solve，设置current ctrl后再solve记录current reaction。
`mj_contactForce(model,data,index,raw)`返回None、写(6,) contact-frame force/moment，
contact.frame的行是world轴，geom顺序决定受力符号；不能直接拿raw[0]作为world Z向量。
`mj_jacSite(model,data,jp,jr,site)`返回None，写world线/角Jacobian(3,nv)，site选新probe球心。
`mj_fullM(model,data,mass)`返回None、展开(nv,nv)惯量，用于完整动力学预算。
`mj_step(model,data)`返回None，更新实际qpos/qvel/time；dt1ms、implicitfast，失败hold也继续物理推进。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/ur5e_surface_following.py
python examples/18_compliant_control/ur5e_surface_following.py --leg-duration 1
```

默认每段2s，5…9s往返；Modify仅改每段1s，5…7s往返，模拟均至12s。
无注入nominal检查扫描窗口切向误差<1mm、力误差<.05N、contact100%；末11…12s切向误差<.1mm、力误差<.05N。
扫描及末尾binormal误差<1mm、orientation误差<.005rad，整轮site/joint速度在预算内且无motor饱和。
T2预期task True；T1作为预期跟踪失败对照，仍完整运行用于评价，不能把engineering PASS等同所有task True。
`scan_complete`只表示参考路径时段结束，不代表actual扫描达标。轨迹/力误差门限是离线验收；与超力10N/速度等在线退出guard不同。
loss case在6s合成将plane下移100mm，预期contact_lost→failed，锁存actual位置和姿态，停止2N外环及后续扫描。
force>10N、actual joint速度>1rad/s、actual site norm速度>.1m/s、joint range越界、requested torque越cap，也可触发failed。
测量速度guard是退出条件，不是使actual速度永不越界的硬限速。保持关节模型范围约束不等于做了避碰。

## Actual Result / Explanation

2026-10-09，Zero：WSL2 Ubuntu24.04.5/kernel6.18.33.2；通过当前conda hook激活mujoco，
同执行shell核验环境与所属Python3.12.14。MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2、
Menagerie2026.9.2的distribution metadata、真实imports/module paths/API通过。无安装或requirements变更；未在3.11执行。

两条最终命令各nominal/loss两case、12001×65 CSV，exit0；输出位于ignored tmp/s17_8b_T2/T1。
初始化IK20updates，位置误差.164044μm、旋转误差4.605e−8rad；time0，不计home到起始点的动态验证。

| nominal指标 | 每段2s | 每段1s |
| --- | --- | --- |
| 切向最大误差 | .352200mm | 1.462038mm |
| 法向力最大误差 | .048350N | .281013N |
| contact保持 | 100% | 100% |
| binormal最大误差 | .116618mm | .501542mm |
| orientation最大误差 | .656481mrad | 2.336228mrad |
| actual site速度norm峰 | 28.9359mm/s | 63.2352mm/s |
| actual joint速度最大分量峰 | .068096rad/s | .152785rad/s |
| 末1s平均normal反力 | 1.999749N | 1.999989N |
| 末1s最大normal误差 | .000565N | .000024N |
| motor饱和sample | 0 | 0 |
| task_passed | True | False：切向/法向过程指标超限 |

两组均1.005s touch；T2于9s、T1于7s走完参考；T1虽最终回到起点且恢复约2N，中途仍不合格。
参考峰速度28.125→56.25mm/s，峰加速度43.3013→173.2051mm/s²；actual误差并不按同样比例缩放。
切向误差约4.15倍、法向误差约5.81倍，展示真实arm动态耦合与固定增益的能力边界；不把误差变化归因为单一已证明因素。
两组完整approach/input峰6.400016N、current峰5.235051N，低于10N预算。
nominal motor峰分量≤18.095460N·m；关节caps前三150、后三28N·m。保留joint ranges/armature .1kg·m²，passive damping0。

loss两组均6s注入、同sample contact_lost，reference位置/姿态锁存、参考速度清零，无scan_complete；继续physics至12s。
末尾法向力0N，actual切向约14.927569/31.127775mm，不回起点；T1发生在去程结束附近、T2在去程中点附近。

每步actual motor/gear/cap、contact Jᵀ映射、wrench功率和完整动力学通过；max residual≤3.161e−12N·m。
`python tmp/s17_8b_validation/check.py`独立复核四CSV的quintic/力参考积分/inner控制/限幅和failed锁存；
每case246个50ms及事件landmark快照重建FK/J/bias/当前与previous-ctrl测力/加速度，
用actual接触点mj_applyFT独立核对constraint，逐快照再mj_step核对下一行实际q/v，通过。
`python tmp/s17_8b_validation/jacobian_checks.py`复核整条actual q/v递推、sphere质量/半径/contact masks/range，
初始/6s的probe与attachment 40mm偏置关系、数值差分Jp/Jr及独立真实motor饱和probe，通过。
这里无passive damping时观察到的implicitfast递推，不推广成所有implicitfast模型的公式。
CLI duration0/nan各exit2；两PNG目视与本地Markdown links/git diff --check通过。

初版照搬30mm/s approach输入反力约19.6N，触发force_budget_exceeded；改10mm/s与10mm起始间隙。
较大初始间隙导致5s load_not_ready；更早touch后，原gain.003的T2扫描最大力误差.069206N仍不合格。
最终将P gain调为.006、保留20mm/s速度界和所有验收/退出门限，T2达标，T1保留为失败对照。
这些准备实验不计最终PASS；没有对所有姿态/模型参数的稳定性保证。
本轮未动态注入超速/关节越界/motor不足/搜索超时，不把独立motor静态限幅probe称为整臂饱和后的恢复验证。
无GUI、ROS、摩擦、自碰撞/其他arm碰撞、未知曲面或硬件验证。

## Failure Cases / Robotics Context

错surface轴或遗漏origin会把normal反馈/切向参考送错方向；用旧attachment_site Jacobian会漏掉probe偏置。
姿态误差与角速度必须同为world frame。bias补偿不能消除真实contact反力，否则改变当前任务物理意义。
多关节耦合使切向运动影响法向与姿态；实际力与路径需要分别验收，不能只复用fixture的数值结论。
本课对应已知平面擦拭/抛光控制的最小原型；没有未知曲面估计、摩擦、真实力传感器、硬件或ROS接口。
失败hold仍可能运动/失去载荷，不自动恢复、退让或完成剩余路径。ROS监控与worker生命周期留S17.9。

## Interview Capsule

30秒：我把固定姿态二维fixture的法向测力外环与往返路径移植到UR5e，新增binormal与姿态保持。
用probe球心的world Jp/Jr把Cartesian力/力矩映射为关节motor torque，加模型bias补偿；actual运动只来自mj_step。

2分钟：解释surface非零origin、+Z反力与欠力时负normal参考；说明qpos/dof/actuator地址和gear限幅。
给出平移与旋转的独立单位和Jᵀ式，区分IK初始化和动态控制，说明bias不含contact。
用扫描误差/contact/force/姿态/速度/饱和评价，再用中途loss展示partial失败，不把任务退出称为硬件停机。

## Must Remember

- 世界坐标位置转surface坐标要先减origin。
- 新probe点必须使用对应Jacobian；球心、attachment与接触点不同。
- bias是模型项，contact是环境反力；target不等于actual force。
- IK只准备起始q，扫描必须由motor和physics执行。
- selector分轴不保证一般机械臂动力学解耦。

## My Verification / Run / Modify / Explain

2026-10-09 本人明确确认实验与预测完成，并准确回答五项Explain；Run/Modify/Explain完成，Learning Mastered，状态见根README。
解释精度：gear=2、固定gain=1且零bias时，ctrl=τ_req/2；150N·m joint cap对应actuator force的forcerange±75。
本课actuator_ctrllimited=False，限制施加在actuator输出，不能把forcerange称为ctrlrange。
Jp随所选点改变；同一刚性body上的Jr相同，球心偏置40mm的额外贡献由Jp体现。
Run：执行默认命令，查看tmp/s17_8b_T2/results.json、ur5e_surface.png及nominal/loss事件与指标。
Modify：只将每段2s减成1s；先预测参考峰速度×2、加速度×4，再观察实际轨迹/法向力/姿态和力矩是否符合预算。
Explain：

1. 从fixture移植到UR5e后，为什么需要binormal和姿态保持？为什么只用六关节PD保持初始q不能完成扫描？
2. surface法向和原点是什么？world位置与force转surface时为什么只有位置需要减origin？
3. 为什么Jp/Jr必须在scan_tip求？Jᵀ输出的单位是什么，gear=2的motor该如何下发和限幅？
4. IK、bias feedforward、contact force分别承担什么作用？为什么扫描过程中不持续用IK覆盖qpos？
5. T减半时哪些参考量严格缩放？实际normal力是否必须不变？loss后局部控制是否仍保持2N或代表硬件停止？

下一小任务S17.9等待明确请求，不自动实施。
