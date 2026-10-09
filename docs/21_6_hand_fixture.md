# S18.6 — Lightweight multi-finger hand：模型与驱动映射

约0.5～2h；前置：[S16.1 actuator语义](19_force_dynamics.md)、[S18阶段入口](21_teleoperation_dexterous.md)。
[模型](../examples/19_teleoperation_dexterous/hand.xml) / [代码](../examples/19_teleoperation_dexterous/hand_fixture.py)。
Engineering / Learning唯一状态：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

六个ctrl如何准确驱动三根手指的六个关节？关节目标是否等于实际角度？手指互相碰撞时是否仍能达到目标？
本课建立可读的小模型，分别观察编译后的索引、position servo和实际开闭动力学。
直觉：ctrl向某个actuator发目标，actuator通过transmission作用于具名关节；它不把关节直接瞬移到目标。

## Core Concepts / geometry

固定palm没有joint/freejoint，world位置[0,0,.04]m、top Z=.046m；三指根在半径60mm的圆上，方位0/120/240°。
每指proximal长40mm、mass30g；distal长30mm、mass20g；capsule半径4mm。finger沿local+Z伸出，local+X朝palm中心。
两hinge均绕各自body的local+Y；正角度将local+Z朝local+X弯曲，即向中心闭合。
第二关节轴随proximal姿态转动，不是固定world+Y。初始q=0时三指竖直，不含真实人手的拇指对掌自由度。
每指末端有具名tip site供后续任务使用；本课仅记录位置/几何图，不推导FK/J。

模型nq=nv=nu=6；六个independent position actuator，无tendon/equality联动。
actuator刻意打乱顺序，canonical joint顺序如下：

| joint | joint id / qpos address / dof address | actuator id | target range rad |
| --- | --- | --- | --- |
| f0_prox | 0 / 0 / 0 | 1 | 0–.9 |
| f0_dist | 1 / 1 / 1 | 4 | 0–1.2 |
| f1_prox | 2 / 2 / 2 | 5 | 0–.9 |
| f1_dist | 3 / 3 / 3 | 2 | 0–1.2 |
| f2_prox | 4 / 4 / 4 | 3 | 0–.9 |
| f2_dist | 5 / 5 / 5 | 0 | 0–1.2 |

前三种索引在此全hinge模型恰好相同，API概念仍不同；加入free/ball后不能继续作这个假设。
具名joint→jnt_qposadr/jnt_dofadr，具名actuator→actuator_trnid核对transmission；不靠文件顺序猜索引。

## Mathematics：意义 / shape / unit

```text
q_raw = close_scale · envelope(t) · [.55,.65,.55,.65,.55,.65] # (6,) rad
q_target = clip(q_raw, joint_lower, joint_upper)            # (6,) rad
ctrl[actuator_ids] = q_target                              # gear1 position servo
τ_req = .25 · (q_target−q_actual) − .02 · qvel_actual       # (6,) N·m
τ_act = clip(τ_req, −.08, +.08)                            # actuator torque cap
```

kp=.25N·m/rad、kv=.02N·m·s/rad、gear1；position actuator的ctrl是target rad，不是motor torque。
每joint passive damping=.002N·m·s/rad，armature=.00002kg·m²为附加关节惯量；无重力，无外加力。
真实动力学是`M(q)qacc+bias = τ_actuator+τ_passive+τ_constraint`，M为(6,6)kg·m²，qacc(6,)rad/s²。
同一指的两关节存在惯性耦合，prox单独动作时dist可能短暂偏离0；“独立actuator”不等于动力学解耦。

joint range、ctrlrange、forcerange是三个层次：
- 应用层target clip和actuator ctrlrange限制输入目标，不直接限制实际q。
- joint range由软约束模拟，可小幅越界；不能宣称q永远精确位于range内。
- forcerange在gear1下限制单actuator关节力矩±.08N·m，不限制碰撞反力或所有constraint torque。

本模型专用joint limit参数`solreflimit=.005 1`、`solimplimit=.99 .99 .001`；它们不同于geom contact的solref/solimp。
关节限位参数与软限位含义见[MuJoCo官方XML文档](https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-joint)。
contact使用geom默认solref=.01 1、solimp=.95 .95 .001、friction=[.7,.005,.0001]、默认condim3。

## Collision policy

palm与prox capsule在安装pivot处有4mm几何重叠，XML显式exclude这三对body；prox与同指dist的邻接body按MuJoCo默认父子过滤。
不同手指间碰撞保留；distal与palm也未关闭。不能为使目标成功而关闭所有self-collision。
默认关闭幅度1没有self contact；幅度2将raw目标[1.1,1.3]rad裁剪到[.9,1.2]rad，三根distal在中心互相阻挡。
这只是手自身的几何/动力学实验，没有object、抓取、touch sensor或force closure；对象接触/抓取留后续任务。

## Math-to-Code / MuJoCo APIs

`MjModel.from_xml_path(path)`读MJCF并返回编译模型；包含joint/actuator/body/site ids、range与transmission，不是实际状态。
`MjData(model)`返回可变qpos/qvel/ctrl与动力学/contact buffers，四case各独立实例。
`model.joint(name).id`与`model.actuator(name).id`分别解析名字，读取jnt_qposadr/jnt_dofadr定位state，读取actuator_trnid核对实际joint。
`bounded_target(raw,lower,upper)`返回clip后的新(6,)数组；nonfinite或错误shape拒绝。
`mj_forward(model,data)`原地刷新FK、actuator force和constraint结果，不推进time；写ctrl后actual qpos仍保持当前值。
`mj_fullM(model,data,inertia)`原地展开(nv,nv)真实惯量，用于逐sample核对动力学残差，不做惯量整形。
`mj_step(model,data)`原地推进真实hinge qpos/qvel/time1ms，implicitfast积分；运行中不设qpos/qvel。
因此不使用先前Euler专属递推作为本课积分检查；本课检查独立servo公式与完整力平衡。
`data.contact`读取geom pair、dist与efc_address，只确认active碰撞与几何穿透；本课不实现每指接触测力课。

`static_checks`在独立MjData中验证ctrl写入不移动q、正确f0_prox产生.0625N·m、错误ctrl[0]实际驱动f2_dist，以及实际actuator cap。
静态probe目标100rad故意绕过应用clip，MuJoCo内部ctrlrange/forcerange仍生效；此probe不推进physics也不用于正常运行。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/hand_fixture.py
python examples/19_teleoperation_dexterous/hand_fixture.py --close-scale 2
```

两命令各4case：single_0/1/2只将该指prox目标推到.25rad；cycle三指一起开闭。
0–1s线性闭合；1–2s保持；2–3s线性张开；3–4s保持0。每case4000physics步、4001个pre-step sample。
expected：具名映射正确；single case其他两指不动，同指dist允许惯性耦合；cycle1跟踪目标无contact；cycle2目标裁剪并遇自碰撞，实际可停在目标前；两组都能张开。
CSV/JSON与response/geometry PNG只ignored tmp/s18_6_scale1/scale2。
geometry PNG为body/site中心线与palm轮廓，不是capsule表面或MuJoCo GUI渲染，不能从线条间隙推断表面无接触。

## Actual Result / Explanation

2026-10-09 DESKTOP-781D67A：WSL2 Ubuntu24.04.5/kernel6.6.87.2；conda mujoco所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata、imports/module paths/API通过；无安装或requirements改动，无local helper。
两命令各exit0、4×4001×48CSV；static probes、独立CSV/compiled servo/transmission检查通过，CLI0/nan/−1各exit2。
3.11语法通过但未在3.11执行；四PNG目视、links/status/git diff --check通过，无GUI运行。

| cycle指标 | scale1 | scale2 |
| --- | --- | --- |
| 2s target prox/dist rad | .55 / .65 | .9 / 1.2 |
| 2s actual prox/dist rad | .54999977 / .64999951 | .76546438 / 1.20024128 |
| 2s最大tracking error rad | 4.9013e−7 | .134536 |
| 被裁剪joint samples（六joint累加） | 0 | 7548 |
| active contact samples | 0 | 1660 |
| 单sample最大active contacts | 0 | 3 |
| 最大几何穿透 | 0 | .254095mm |
| 最大joint range越界rad | 0 | .00193948 |
| actuator饱和joint samples | 0 | 0 |
| 末.5s最大open error rad | .000163679 | .000327395 |
| 末.5s最大speed rad/s | .00192270 | .00384584 |

scale2三对observed geom均为不同finger的dist_geom。接触阻止prox达到.9rad，不能把tracking error称为映射失败。
三single case其他两指位置全程0；2s选中prox≈.24999988rad，dist≈2.05e−8rad；无contact。
palm pose全程固定；max dynamics residual scale2约6.73e−16N·m。两幅度均完成重新张开。

开发失败与修正：首版缺palm/prox排除，初始三对安装接触dist≈−4mm，逐指隔离失败；明确排除安装pair后通过。
默认joint限位参数下scale2最大越界.0165633rad，超过预设.005rad实验容差；调整专用限位参数后降到.00193948rad。
首次误将geom的solref/solimp名字用于joint，schema拒绝；核对官方文档改为solreflimit/solimplimit后编译与运行通过。
这些失败不计PASS；最终model与两完整实验重新验证，未修改actual q来掩盖越界。

## Failure Cases / Robotics Context

- 把canonical joint顺序直接写ctrl数组：此模型ctrl[0]驱动f2_dist，不是f0_prox；即使数组shape都是6也会控制错指。
- 把ctrl当actual q：servo是有力矩上限的动力学执行器，contact会阻挡实际运动。
- 把joint/ctrlrange当硬墙：目标精确clip，actual仍受软限位/惯性/碰撞约束影响。
- 把固定palm和独立actuator当真实完整手：模型无移动腕部、人手组织、tendon耦合或object，不能证明握持能力。
- 为通过tracking而关self-collision：抹掉本课要观察的接触阻挡；应先检查装配接触与允许pair。

此fixture为后续多指几何、每指接触及抓取提供透明入口；本课止于结构、mapping、bounded目标和开闭实验。

## Interview Capsule

**30秒：**固定palm上三根两hinge手指，六个gear1 position servo接收rad目标。
名字分别解析joint state地址和actuator id，核对transmission；ctrl不是qpos。目标clip、motor torque cap与joint软限位各自限制不同量。

**2分钟：**解释body树、local hinge轴与正向闭合、state/control三种索引；
用乱序actuator与single finger probe证明映射；给servo力矩公式与单位，解释惯性耦合和self-contact阻挡；
用scale2目标.9rad/actual.765rad说明实际动力学，最后界定软限位与无object/GUI验证边界。

## Must Remember

- 三种索引分别解析，即使恰好相同也不要混用。
- position ctrl是目标rad；qpos实际rad，qvel实际rad/s，actuator广义力矩N·m。
- target limit、torque limit、soft joint limit是三个层次。
- 动力学耦合不是映射错；自碰撞阻挡不是抓取成功。

## My Verification — Run / Modify / Explain

Learning仅由本人报告，状态只在根README。
Run：默认运行，查看mapping JSON、single case CSV与geometry/response图，确认驱动哪一指。
Modify：仅`--close-scale 2`，先预测target裁剪、self-contact、actual tracking与张开恢复，再对照结果。
Explain：
1. 为什么joint id、qpos address、dof address和actuator id必须分别解析？ctrl[0]实际驱动谁？
2. 固定palm怎样由MJCF表达？hinge axis在哪个frame，正角度为何向中心弯曲？
3. position servo的ctrl是什么单位？kp/kv/gear与actual torque是什么关系？
4. target clip、forcerange和joint range各限制什么？为什么actual仍可能小幅越界或达不到target？
5. 单指prox动作时dist可暂时移动，为什么不必是映射错误？self-contact成功检测为何不能证明抓住物体？
