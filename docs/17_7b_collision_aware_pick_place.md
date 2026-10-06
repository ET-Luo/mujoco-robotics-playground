# S14.7b — Collision-Aware Pick-and-Place

[代码](../examples/15_motion_planning/collision_aware_pick_place.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。复用[S13.3b执行](16_3b_vision_pick_place.md)与[S14.7a规划接口](17_7a_ur5e_obstacle_planning.md)。

## Problem → Why

裸臂不撞，不代表夹爪或手中物体不撞；抓取需要接触，运输却不能允许物体落地。
本课把带物几何与phase contact policy接入规划，并独立检查actual取放与重复试验。
输入是已知物体pose，集中学习planning，不重做PnP。没有weld或执行qpos重置。

## Intuition → Core Concepts

规划草稿上假设物体固定在夹爪中，随每个候选q移动；执行时物体完全由接触力与重力运动。
两个模型的差异可能产生滑移、碰撞或抓持失败，所以名义几何通过不是实际成功。
运输还需要抓持姿态/高度约束，不能只问“当前姿态有没有撞”。

| 阶段 | 允许的具体pairs | 几何/执行补充 |
| --- | --- | --- |
| transit/pre/approach | object–ground | 夹爪打开，物体静态已知pose |
| close/lift | left–object、right–object、object–ground | lift开始允许离开初始支撑 |
| lift_hold/transfer/transfer_hold | left–object、right–object | 物体落地是失败，不能放行 |
| descend/support | left–object、right–object、object–ground | 需要持续有力支撑才可释放 |
| release | left–object、right–object、object–ground | 等待持续分离 |
| release_hold/retreat/final | object–ground | 不允许finger–object再接触 |

所有允许pairs仍有6mm最大penetration；robot/self/obstacle与finger–ground不放行。
原阶段检查、95%双侧接触、每个held运动阶段15mm relative-change规则保留。
累计相对变化另报告，不是15mm累计保证，也不是路径长度。

## Mathematics：shape、unit、frame

q shape `(6,)` rad；两个finger slide和object freejoint translation为m，freejoint rotation为wxyz。
T_OG是G→O，名义T_GO=inverse(T_OG)是O→G，均shape `(4,4)`，translation米：

```text
T_WO(q) = T_WG(q) T_GO
p_WO = p_WG + R_WG p_GO
R_WO = R_WG R_GO
```

第一次forward获得工具FK，写入完整object freejoint后第二次forward刷新碰撞。
query中的物体仅是固定抓持**假设**；actual object pose只用于评价，不反馈到目标/名义offset。

初始object center=[−.45,.20,.03]m，box全尺寸40/30/60mm；destination=[−.30,−.10,.03]m。
固定球障碍center=[−.375,.05,.09]m、r=.025m，位于持物直连附近。
名义held运输tool tilt≤.15rad；transfer的名义object center z≥.085m。
这些是显式任务约束，不是通过碰撞检查自动推导的抓稳保证。
名义障碍surface distance≥8mm，actual≥2mm；signed distance capped上界.1m，非连续安全证明。

树的rad Euclidean metric/η=.2rad/h=.025rad、局部joint box（端点范围±.35rad，clip compiled limits）；
shortcut h=.01rad，全部输出边复查h=.01rad。transit与transfer使用RRT-Connect+shortcut+cubic；
reference v≤.3rad/s、a≤.8rad/s²，waypoint零端速，C1一般非C2。
其他lift/descend/retreat保留原Cartesian cubic进度/私有IK/关节插值参考，逐样本查名义几何；
不能把其旧插值参考也宣称为新增双约束cubic时间化。

## Math-to-Code / APIs

`run_trial`构建同一场景，生成known-pose grasp candidate，再定义geometry queries、motion planner、actual observer。

- `build_model`只新增可选obstacle参数；旧默认模型不变。保留原gripper与4个wrist–finger exclusions。
- `run_pipeline`新增可选motion_planner/geometry_observer与close_ramp_seconds；默认None/0保持旧执行。
  回归运行原S13.3b CLI通过；新增钩子只由本课启用。
- `queries`独立MjData，候选arm q及名义finger target写入副本，未持物时object是初始/命令放置pose；
  持物时用T_GO更新freejoint完整pose，actual对象位置不用于planning。
- `policy_report`解释每个contact的具体pair与signed dist，允许pair也拒绝过深penetration。
  用`mj_geomDistance`逐对查询所有active非plane collision geoms到obstacle，包含box与gripper。
  返回值为m、上截断.1m；不是contact力。模型过滤/凸几何近似边界见前课。
- `plan_connect`复用六维callbacks；失败在当次phase停止，保存partial tree。
  所有reference样本再次检查，同名reference数组保存于NPZ，方便独立复查。
- `descend`只保留从起点连续有效的名义参考prefix，避免将会撞finger–ground的末端规划点放行。
  **实际**下降仍以连续50steps/0.1s object–ground force>.05N支撑事件停止，未用truth位置纠正目标。
  若截断后没有实际支撑，support gate失败，绝不强行释放。
- close用1s cubic finger **ctrl目标** ramp，然后1s settle，避免servo step瞬态；不直接赋actual finger qpos。
- `mj_step`积分并推进time，`mj_forward`刷新积分后cache，`mj_contactForce`原地填6维contact-frame wrench。
  wrench首轴为contact normal，用于bilateral/support force证据；API详见S13.3b。
- arm-only qfrc_bias前馈使用理想广义外力接口，物体仍受重力，不能据此证明硬件扭矩能力。
- observer在trace追加之后检查actual geometry，因此失败那一步也保存；无对象姿态修正或动力学weld。

## Minimal Experiment → Expected / Actual Result

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/collision_aware_pick_place.py
python examples/15_motion_planning/collision_aware_pick_place.py --trials 1 --close-target 0.014
```

依赖已声明NumPy/MuJoCo/Matplotlib/Menagerie/OpenCV headless；cv2来自复用执行模块imports，
本课没有实际图像/PnP处理。无新依赖/安装。
2026-10-06 DESKTOP-781D67A：Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2、
Menagerie2026.9.1、OpenCV distribution4.14.0.94/import4.14.0。3.11兼容目标未执行。

默认固定场景，planner seeds7/8/9，3/3 placement_passed；只改变planner seed，非物理随机化。

| seed | sim time | final position error | min actual obstacle distance | transfer phase relative change |
| --- | --- | --- | --- | --- |
| 7 | 21.888s | .758213mm | 8.967720mm | 11.512518mm |
| 8 | 22.122s | .929616mm | 15.624921mm | 12.713926mm |
| 9 | 22.478s | .807125mm | 22.355081mm | 13.794382mm |

seed7：lift52.910115mm，transfer planner48扩展/6shortcut接受，transfer reference3.198s；
三held运动phase均双侧接触fraction1.0，transfer无ground contact。累计relative change最大约24.61mm，
不能将phase15mm通过误当累计15mm通过。final orientation error=.008746°，retreat79.974mm；
最后支撑/分离/低物体速度检查通过。

close-target=.014m代表每根finger slide目标，名义opening=20+2×14=48mm，宽于本课夹持尺寸；
1trial在close拒绝（持续双侧contact/force未建立），不执行lift/transfer。
`--trials 1 --max-nodes 2`在transfer node_budget失败：transit直连仍可通过，但持物绕行无法扩展；
保留close/lift实际轨迹，**不执行transfer**，更没有最终placement结果。

评价CLI三种情况均exit0：表示全部尝试已评价，须看successes/status/failure_phase；非法参数exit2。
成功率分母包含全部attempted；successful_final_errors_m只包含成功，不能代表所有尝试的平均误差。
3/3不证明普遍100%可靠性，样本极小且场景/控制/物理固定；未做稳定wall性能benchmark。

工程调试的失败证据（不是当前默认结论）：一步close target跳变曾穿透约6.11mm，
保持6mm门限、改ramp后解决；较弱夹紧/无运输高度限制的raw RRT路径曾使物体落地；
加入名义运输姿态/高度约束后避免触地，但未shortcut的路径仍超过phase滑移门限。
最终collision/姿态/高度checked shortcuts缩短运输，保留15mm门限通过；未放行transfer–ground或finger–ground。
历史诊断在ignored tmp/s14_7b_diagnostics/，不是已维护的旧策略CLI回归入口。

产物ignored `tmp/s14_7b_pick_*/evaluation.json`与每trial的summary.json/trace.csv/pipeline.npz/pipeline.png。
NPZ保存名义T_GO/T_OG、各phase参考与实际trace，actual_qpos供独立几何回放；失败保存partial trace。
原trace shape `(steps,33)`及phase ID含义沿用S13.3b；actual_qpos shape `(steps,model.nq)`。
无GUI、真实视觉、连续碰撞或硬件验证；palm visual没有active collision，arm/base等coverage/exclusions仍受模型限制。

## Explanation → Failure Cases

- 漏掉held object：planner仅查机器人外形会漏物体障碍；每个query必须更新object后forward。
- 名义固定offset不等于实际无滑移：actual会相对运动，保持独立retention/placement评价。
- 白名单过宽：允许抓取接触不能允许object–obstacle、finger–ground或transfer–ground。
- collision-free但抓持失败：工具倾斜、reference高度降低、长时间运输都可能使held假设失效。
- solver软接触：允许pair也可能瞬态穿透过深，平滑ctrl目标与放宽门限不是同一行为。
- descent只查终点：真实support事件可能提前出现；须持续力证据，并拒绝无支撑release。
- partial failure当成功：node budget在执行中阶段失败时，前面可能已有运动；不能称time=0/无执行。
- 统计误解：同场景3个planner seed只是当前路径策略观察，不是物理/视觉鲁棒性试验。

## Robotics Context → Interview Capsule

实际搬运需同时满足robot/environment/object几何约束、抓持保持和阶段接触语义。
本课展示planner的固定held几何如何与实际contact mechanics衔接；未引入ROS或硬件。

**30秒**：候选q先做工具FK，再按名义T_GO移动物体并重算碰撞。按phase放行具体接触对，
但始终拒绝障碍和finger-ground。规划通过后actual只用controls推进，另查几何、接触力与滑移。
重复试验包含失败分母，固定held假设不是实际抓稳证明。

**2分钟**：我复用取放执行器，只新增可选规划/观察钩子。transit与held transfer接双树、
checked shortcut、cubic参考，其余Cartesian动作逐样本验证。held查询使用完整pose链，
而执行物体保持freejoint靠接触与重力。工具倾斜/高度政策避免无碰撞但不适合运输的路径，
actual阶段检查仍可否决名义通过。支撑force持续后才release，分离后才retreat。
初版穿透/落地/滑移失败被保留，没有放宽接触白名单或slip门限。三seed只说明固定场景，
模型覆盖、离散分辨率、理想arm前馈限制了结论。

## Must Remember → Run / Modify / Explain

必须记住：完整T_GO/两次forward；名义≠actual；pair+phase+depth；force/support事件；失败停止并保留trace；全部attempts统计。

**Run**：默认3trial，核对每trial transfer计划、actual距离、slip、支撑/释放和最终误差。

**Modify**：先预测`--trials 1 --close-target 0.014`的opening与首个失败phase，再运行对照。

**Explain（五问）**：
1. T_GO/T_OG各是什么方向？held查询为什么forward两次？actual为什么不能按T_GO强制摆物体？
2. 哪些接触在close/place允许、transfer却不允许？为什么允许pair还要限制深度？
3. 为什么名义collision-free不证明抓稳？工具姿态/高度约束、actual slip分别解决什么？
4. 为什么支撑力持续才release、分离才retreat？planner budget在transfer失败时哪些动作已执行？
5. 3/3能说明什么？如何区分全部attempts与成功条件误差、phase slip与累计变化？

**My Verification**：本人明确确认实验与预测完成，Explain覆盖T_GO/T_OG与两次forward、分阶段pair/深度、名义held与实际摩擦滑移、持续支撑/分离事件、失败前已执行阶段和小样本统计边界，Learning Mastered；状态只在根README维护。

精度补充：累计relative change是相对close结束基准的最大位置偏离，不是各phase滑移之和或轨迹长度。名义高度约束只约束预测物体姿态；实际离地、保持与支撑仍需独立证据，不能由名义几何通过推认。
