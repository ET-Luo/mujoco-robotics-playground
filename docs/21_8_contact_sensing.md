# S18.8 — Touch/contact sensing：接触对象、合力与区域读数

约0.5～2h；前置：[手模型](21_6_hand_fixture.md)、[Jacobian](21_7_fingertip_jacobian.md)、[接触wrench](19_5_contact_wrench.md)。
[代码](../examples/19_teleoperation_dexterous/contact_sensing.py) / [示例入口](../examples/19_teleoperation_dexterous/README.md)。
Engineering/Learning状态唯一来源：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

一根手指有touch读数，究竟是碰到另一指、碰到环境，还是碰到目标物体？一个区域读数为0是否能证明整根手指没接触？
本课保留接触双方的geom名字，再分别统计接触、每指world合力、法向力之和以及两个区域传感器。
直觉：touch是某块区域收集到的标量法向载荷，不是一根手指完整的三维力，也不自动说明接触对象。

## Core Concepts / scope

复用hand.xml和hand_fixture mapping；在内存XML副本中添加six touch sensors，原hand模型不改。
三case：no_contact正常闭合、self_contact过闭合、fixed_probe正常闭合时f0碰到固定sphere探针。
探针固定在world [.024,0,.09]m，半径10mm；不是自由物体、抓取或抗扰动任务。
每case4s、4000physics步/4001pre-step样本；实际关节由mj_step推进。依赖已有mujoco/numpy/matplotlib与hand_fixture helper，无新依赖。

每指distal body上两个site区域：
- small：sphere半径8mm，local位置[0,0,.03+tip_offset]m，默认中心在tip。
- wide：ellipsoid半轴[12,12,25]mm，local中心[0,0,15]mm，覆盖本例distal接触。

site只定义传感器区域，不是碰撞geom，也不改变质量/接触形状。Modify将small向local+Z移30mm，wide不变。
这些touch只匹配**与site同body的geom**；本课位于distal body，不等于整指proximal/distal都能被它检测。
每指normal/合力统计则覆盖该指两个geom，通过具名geom id→finger owner映射。

[MuJoCo官方touch定义](https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor-touch)：输出是非负标量法向力之和；接触点位于区域内，或相关normal ray与区域相交时纳入。
所以不能仅用contact.pos是否在site内复刻touch；normal re-projection也可能纳入点在区域外的接触。
本课不实现自制ray筛选器，也不把区域传感器称为高分辨率触觉皮肤。

## Mathematics：shape / unit / frame / sign

每个active接触读取raw(6,)=[normal,tangent1,tangent2,moment...]，前三项N、后三项N·m。
contact.frame reshape(3,3)的**行**为contact normal/tangent轴在world中的方向。

```text
F_on_geom1_W = frame.T @ raw[:3]              # (3,) N，作用在contact.geom[1]
F_on_geom0_W = −F_on_geom1_W                  # (3,) N，对方反作用力
F_finger_W[i] = Σ 本指接触的带符号world力      # 每指(3,)，三指(3,3)
N_finger[i] = Σ max(0, raw[0])                # (3,) N，法向标量和
touch_zone = Σ zone纳入的raw[0]              # 每sensor scalar N
force_qualified[i] = N_finger[i] > .05N       # (3,) bool，教学阈值
```

candidate count是几何接触记录数；active count要求efc_address≥0；force-qualified还要求法向和超过阈值。
active constraint不保证力足够大，threshold bool也不保留载荷大小。当前场景未专门制造inactive geometric candidate，不能把该防御分支称为注入验证。
Fn sum不是net force norm，也不是压力Pa；它不含切向分量或方向信息。
一根指参与两接触时不同方向力相加可能抵消，法向标量仍相加；不要把所有sensor标量相加当作物体net force。
指间自接触应同时计入两指，力相反；整个hand内部接触force总和为0，不代表每指没受力。

独立符号校验：将接触点P的world力投影到其body的点Jacobian，`τ_contact = Jp(P).T F + Jr(P).T moment`。
这不是tip Jacobian：不同作用点的平移Jacobian不同。`mj_jac`在contact point写world Jp/Jr(3,nv)。
本例condim3的contact moment为0，但记录raw完整六项；此例不验证condim6扭转力矩。
qfrc_constraint还含soft joint limits，不能直接把它全部当contact torque；代码仅取contact对应efc rows与efc_force进行比较。
当前小模型使用dense efc_J；pyramidal condim3每contact四行。该检查是force sign/frame验证，不是新增控制器。

## Math-to-Code / APIs

`build_model(case,tip_offset)`解析原MJCF，在副本添加site/touch/可选fixed probe，再`MjModel.from_xml_string`返回编译模型。
`MjData(model)`返回可变actual state/physics/sensor buffers；`mapping`分别解析joint state地址与actuator id。
`mj_forward(model,data)`原地更新FK/contact约束解和当前touch sensordata，不推进time；写ctrl后显式刷新，CSV记录同一pre-step状态。
`mj_contactForce(model,data,index,raw)`无新值返回，原地写contact-frame force/moment；efc_address<0时不读取active力。
`read_touch(...)`先具名解析sensor id，核验sensor_type与dim1，再用sensor_adr读sensordata切片。
六sensor故意顺序f2/f0/f1；canonical f0small/wide、f1small/wide、f2small/wide地址为[3,2,5,4,1,0]。
此处全scalar使id与adr恰好相同，但一般多维sensor时并不相同，仍要用sensor_adr。
`mj_jac(model,data,jp,jr,point,body_id)`输入world point(3,)m和body id，原地写world Jp/Jr(3,nv)，用于接触点投影校验。
`mj_step`原地推进actual qpos/qvel/time1ms；sensor值从未作为ctrl或外力输入，因此没有触觉反馈控制。

JSONL保留每个active pair的geom名字、owner、contact frame/point/raw与world force，供独立重建；不把固定探针或其他手指误标为抓取目标。
输出CSV每sample含candidate/active counts、normal sums、每指world F、small/wide touch与threshold bool；q/qvel用于验证Modify不改physics。

## Minimal Experiment / Expected → Actual

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/contact_sensing.py
python examples/19_teleoperation_dexterous/contact_sensing.py --tip-offset .03
```

两命令各三个case；CSV/JSON/JSONL/PNG只ignored tmp/s18_8_offset0与offset0.03。
预期无接触全零；self_contact出现具名指间pair；fixed_probe只f0测到探针；small区域不保证覆盖全部distal接触。
Modify改变small读数，但不改变q/qvel、接触力、wide读数。所有case最终张开后不再有active contact。

2026-10-10 DESKTOP-781D67A：WSL2 Ubuntu24.04.5/kernel6.6.87.2；同shell conda mujoco所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API/helper路径通过，无安装/requirements改动。
两命令exit0，各3×4001×40CSV，两PNG已目视。独立校验和links/diff通过，3.11语法通过但未在3.11执行；未运行GUI。

| case | 具名pair | 峰法向和N | 默认small峰N | offset30mm后small峰N | wide峰N |
| --- | --- | --- | --- | --- | --- |
| no_contact | 无 | 全指0 | 全指0 | 全指0 | 全指0 |
| self_contact | f0/f1、f0/f2、f1/f2的dist_geom | 每指2.818674 | 每指2.818674 | 全指0 | 每指2.818674 |
| fixed_probe | f0_dist_geom/probe_geom | f0 1.395928，其他0 | 全指0 | 全指0 | f0 1.395928，其他0 |

self_contact有4980active pair记录（同sample三pair），fixed_probe有1729记录。
默认small漏读force-qualified finger-samples：self0、probe1721；移位后self4974、probe1721。
这些是阈值.05N下**finger-sample**计数，不是“抓取失败次数”或sensor延迟。
所有case中wide逐sample等于本指normal sum，说明它覆盖本场景的distal接触，不证明所有未来姿态或proximal接触均覆盖。

1.5s self_contact每指normal sum约2.244829N、world合力norm约1.944079N；三指world合力相加为0。
probe在1.5s f0 world F≈[.868680,0,.938380]N，norm1.278734N、normal1.278342N；切向分量使二者也不相同。
两个offset的q/qvel/candidate/active/normal/world F/wide/binary全部列逐sample完全相同，只有small读数可能不同。
contact点world force投影与contact efc generalized torque残差≤2.776e−17N·m，建立符号与坐标语义。

独立`tmp/s18_8_validation/check.py`读取JSONL用frame三行线性组合重建force，按owner/sign汇总normal/active/world F，核对CSV与threshold；
验证内部反作用力抵消、probe仅f0、wide覆盖以及两offset physics全trace一致，六case通过。
CLI offset nan/−1/.01各exit2，builder nonfinite/negative拒绝；3.11语法、local links/status/git diff --check通过。

## Explanation / Failure Cases

- touch>0却对象未知：self_contact也可有载荷，要读geom pair才能区分指间/fixture/未来object接触。
- small touch=0却本指有force：区域未覆盖对应接触；本probe对照直接展示，不能用零读数断言整指无接触。
- 把scalar touch当三维力或净力norm：丢掉方向与切向载荷，多接触方向可抵消。
- 不处理geom顺序：对第一/第二geom作用力相反，错误sign会使world force与约束力矩不符。
- 将joint limit误算为接触力：qfrc_constraint混合多种约束，需按contact来源核对。
- 图上通道重合并不说明没有测量区别：默认self的小/大区域确实读到相同contacts，probe与Modify才暴露覆盖边界。
- 所有touch正仍不证明object稳定：本课没有自由物体、lift、保持摩擦或抗扰动；不推进S18.9。

## Robotics Context

逐指touch/force用于接触事件、负载分配和抓取状态判断的输入；“触到了谁”及测量覆盖是控制前必须明确的接口。
接触测力还需要现实sensor噪声、偏置、带宽/标定与安装位置；本课使用理想仿真读数，未注入这些硬件误差。

## Interview Capsule

**30秒：**保留geom pair，把contact-frame力用frame.T转world并按geom顺序选sign，汇总每指net force与normal sum。
touch是同body/site区域内或normal-ray纳入的法向标量和；区域零读数不代表整指无接触，自接触正读数不代表抓取。

**2分钟：**说明candidate、active和threshold binary；给force转换与touch shape/unit，比较self与fixed probe；
用移位区域且physics完全相同的实验说明sensor覆盖，用内部力抵消说明scalar与vector区别，最后界定自由物体与真实sensor验证边界。

## Must Remember

- 接触双方名字、作用对象、frame先于任何“抓住了”的判断。
- touch单位N，不是压力Pa或三维力；sensor_adr用于读值。
- candidate / active / force threshold三个证据分别看。
- site区域不改变碰撞几何；测量覆盖不等于整指覆盖。

## My Verification — Run / Modify / Explain

Learning仅由本人报告，状态只在根README。
Run：默认执行，查看touch_comparison.png与JSONL，解释probe有force但small为0、自接触没有object却有touch。
Modify：仅`--tip-offset .03`，先预测small/wide/contact/actual state哪些改变，再对照。
Explain：
1. candidate、active contact、force-qualified binary和touch值分别是什么？
2. world接触力为什么用frame.T并按geom顺序取sign？每指normal sum为何不等于合力norm？
3. small touch=0能否证明整指无接触？site区域匹配哪一个body，normal ray有何影响？
4. sensor id与sensor_adr有什么区别？移动site为什么能改读数而不改physics？
5. 自接触总world合力为0且touch为正，是否矛盾？为什么这些结果不能证明物体被稳定抓取？
