# S17.8a — Surface Following Integration / fixture

约0.5～2h；前置：[S17.7 Hybrid](20_7_hybrid_position_force.md)。
[代码](../examples/18_compliant_control/surface_following.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning 状态只维护在[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

保持2N法向载荷的同时，工具沿已知平面走30mm再返回；如果中途失去接触或能力不足，不能继续把剩余路径当成成功。
终点回到起点并不证明往返途中准确，也不证明一直压住表面。本课把已有法向反馈和切向位置跟踪串成有阶段、有失败出口的实验。
直觉：切向负责“走哪里”，法向负责“压多重”，状态机负责“什么时候可以走、什么时候必须结束任务”。

## Core Concepts / scope

同S17.7 XY slide，固定orientation，surface t/n/b=world X/Y/Z，原点重合，R_WS=I。
球半径20mm，site在球心；平面y=0、法向+Y，condim1无摩擦，零gravity/passive/external。
M=diag(2,1)kg，inner K400N/m、D=[56.568542,40]N·s/m，dt1ms，motor广义力cap20N。
法向target2N、gain .003m/(N·s)、参考速度cap20mm/s、normal reference5…60mm。
approach30mm/s独立于force-loop参考速度界。依赖仅mujoco/numpy/matplotlib；helper链
surface_following→hybrid_control/normal_force→contact_transition→cartesian_spring，Agg headless，无新依赖。

| phase | 行为 / 转移 |
| --- | --- |
| 0 approach | 接近；active contact且反力>.05N→1；3s未接触→4 |
| 1 load establishment | 法向反馈；5s检查力误差≤.05N才进入2，否则4 |
| 2 scan | 30mm去程+回程；两段完成→3 |
| 3 final hold | 切向0，继续法向反馈保持2N |
| 4 failed local hold | 锁存失败当刻actual site位置，reference速度清零，不恢复扫描 |

触碰后丢失gate、输入测力>10N、requested motor超当前cap，均锁存phase4。
10N是教学预算；guard读取当前状态、previous ctrl的fresh forward测力，当前ctrl施加后的力另外记录。
没有对未来力峰或一个采样间的力作保证。phase4继续inner damping和mj_step，不是冻结qpos或硬件制动。
锁存actual位置不继续保持2N任务；丢失表面时仍可能残余运动，motor不足时尤其明显。

## Mathematics：meaning / shape / unit / frame

每段时长T，距离L=.03m，u∈[0,1]：

```text
h(u)=10u³−15u⁴+6u⁵
outward: x_d=L h(u), v_d=L/T h'(u)
return:  x_d=L(1−h(u)), v_d=−L/T h'(u)
normal: v_ref=clip(−.003(2−f_n), −.02, .02) m/s
        n_ref_next=clip(n_ref+dt*v_ref, .005, .06) m
        v_ref_actual=(n_ref_next−n_ref)/dt
x_ref_S=S_p [x_d,0,0]ᵀ+S_f [0,n_ref,0]ᵀ
F_cmd_S=K(x_ref_S−x_S)+D(v_ref_S−v_S)  # XY only
τ_req=Jp_Wᵀ R_WS F_cmd_S
ctrl=τ_req / gear
```

S_p=diag(1,0,0)、S_f=diag(0,1,0)，互斥正交投影，3×3无量纲。
x/v/F为(3,) m、m/s、N；Jp为(3,2)，slide列m/m；τ_req为(2,) N。
R_WS列为surface轴在world中的方向；x_S=R_WSᵀx_W，v_S=R_WSᵀJp qvel，f_n=(R_WSᵀF_env_W)[1]。
法向反力+Y，欠力时参考向−Y深入，因此负号必要。
两段的端点速度与加速度为零，连接处位置/速度/加速度连续；jerk不要求连续。
参考峰速度1.875L/T；峰加速度(10/√3)L/T²。T减半：参考速度翻倍、加速度四倍，距离与target不变。
实际跟踪误差由闭环动力学决定，不能套用同一个比例。

## Math-to-Code / APIs

`scan_reference(time,duration)`返回切向位置m和速度m/s；失败后planned goal仍用于诊断，applied reference已锁存。
`build_model()`复用`MjModel.from_xml_string(str)`，返回编译模型；`MjData(model)`返回可变状态与缓存。
`mj_forward(model,data)`返回None，原地计算位姿、约束力和加速度，不推进时间；每步先测力、再给ctrl、再求当前反力。
`measure(...)`调用`mj_contactForce(model,data,index,raw)`，后者原地写(6,)接触frame力N/力矩N·m；
按contact.frame行轴旋转、geom顺序取符号，helper返回环境→球world force(3,)、active数、depth m。
`mj_jacSite(model,data,jp,jr,site)`原地写world线/角Jacobian(3,nv)，返回None；用于site速度和Jᵀ广义力。
`mj_fullM(model,data,inertia)`原地展开(nv,nv)惯量供动力学校验。
`mj_step(model,data)`返回None，推进实际qpos/qvel/time一个dt；执行中不覆盖实际位置/速度。
限幅施加在motor actuator force，gear=[2,1]把它映射成slide广义力；记录requested与actual区分饱和。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/surface_following.py
python examples/18_compliant_control/surface_following.py --leg-duration 1
```

默认扫描5…9s（每段2s），Modify扫描5…7s（每段1s），均模拟到12s。
每条命令四case：nominal；6s移除平面的loss；6s平面向工具跳2mm的overforce；6s切向cap降为.01N的saturation。
这些是合成故障注入，不是运动表面动力学模型。三个失败对照应拒绝任务、锁存reference、取消后续扫描、仍推进physics。
nominal验收扫描切向误差<1mm、法向误差<.05N、contact100%；末11…12s切向误差<.1mm且contact/force合格，无饱和。
工程通过包含预期失败实验通过，不能把所有case的task_passed称为True。
CSV/JSON/PNG仅ignored tmp/s17_8a_T2与T1。

## Actual Result / Explanation

2026-10-09，Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；同shell核验mujoco与所属解释器。
Python3.12.14，MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2 metadata/实际imports/module paths/API通过；
无安装/requirements改动；源码保持Python3.11兼容写法，未在3.11运行。

| nominal指标 | T=2s | T=1s |
| --- | --- | --- |
| 往返扫描时间 | 5…9s | 5…7s |
| 最大切向误差 | .204596mm | .740687mm |
| 最大法向力误差 | .035781N | .035781N |
| 扫描contact | 100% | 100% |
| actual切向峰速度 | 28.6304mm/s | 59.3115mm/s |
| X motor峰值 | .090772N | .380159N |
| 末1s平均反力 | 1.999964N | 1.999964N |
| saturated samples | 0 | 0 |

两条最终命令各四case exit0；nominal task True，三故障 task False。
loss和overforce均6s退出；超力input峰22.500547N/current峰20.563787N，退出后末尾仍有约.801186N反力。
saturation在T2于6.048s、T1于6s退出：cap降低不必立即使requested越界。
T2低cap组继续physics至12s，actual切向末尾约97.63mm、5953个饱和sample；
这直接展示“reference锁存”不能保证工具就地停止。T1饱和sample1655，末位置约30.605mm。

每步frame/J/motor cap/功率/M/动力学与零bias/passive/external、完整实际半隐式q/v递推检查通过。
独立`python tmp/s17_8a_validation/check.py`复核八CSV的controller、限幅、动力学、递推，
quintic边界、normal反馈积分、失败reason/reference锁存，以及初始/touch/扫描/注入/力峰/末尾actual状态恢复后的直接接触力。
两nominal normal相关列差≤1e−10，属于这个无摩擦恒定惯量装置的结果。
CLI duration0/nan各exit2；默认PNG目视、本地Markdown链接/git diff --check通过。
初版T2因把饱和退出误要求为注入同刻而assert失败；改为检测实际requested越界后的退出窗口，未放宽正常扫描门限。
搜索超时/load-not-ready本轮未注入；没有通用稳定性、GUI或硬件停止验证。

## Failure Cases / Robotics Context

接触丢失后继续积分法向reference会追逐不存在的表面；继续planned tangent路径会隐藏partial failure。
超力退出不等于接触反力瞬间归零；局部位置保持也不等于维持原target或回到起点。
饱和检测比较requested与cap，失败hold可能仍饱和，因此还要看actual位置/速度。
本fixture切向无摩擦、恒定对角惯量；分轴选择不能证明一般机器人的动力学解耦。
应用对应擦拭/抛光时的路径与载荷双目标；未验证摩擦、未知曲面、GUI、UR5e、ROS或硬件。
UR5e移植留S17.8b，本轮不实施。

## Interview Capsule

30秒：我用法向力误差生成有界normal参考，用quintic往返生成tangent参考，经selector合成后由Jᵀ motor loop执行。
验收整个扫描窗口的轨迹、反力、contact和饱和；失败锁存actual位置并终止剩余路径，physics仍继续。

2分钟：解释surface frame的+Y环境反力和欠力向−Y的符号；区分outer reference速度与actual site速度。
给出quintic、force-reference积分、inner impedance和gear映射；按approach/load/scan/hold/failed讲状态机。
用6s故障说明planned goal与applied reference的差异；再用T减半说明速度/加速度预算及动态跟踪限制。
最后明确固定无摩擦fixture与真实抛光机器人之间的边界。

## Must Remember

- 整条路径验收；终点相同不等于扫描成功。
- position/force任务在同一frame内分轴，target force不等于measured force。
- planned goal、applied reference、actual motion是三种不同量。
- 失败退出任务与暂停仿真不同；reference界不保证actual安全界。

## My Verification / Run / Modify / Explain

Engineering见根README；Learning Run/Modify/Explain均等待本人明确报告。
Run：运行默认命令，读results.json和surface_following.png，确认nominal True、三个故障False以及事件顺序。
Modify：仅将leg-duration从2s减为1s；先预测参考峰速度/加速度和往返完成时刻，再比较实际误差、力与饱和。
Explain：

1. 为什么回到起点还不足以证明表面扫描成功？扫描窗口应检查什么？
2. 欠力时normal参考为什么向−Y？selector如何合成两个任务？
3. planned goal、applied reference、actual位置分别是什么？故障后哪些仍随时间变化？
4. phase4为什么继续mj_step？锁存actual位置是否意味着实际速度立即为零或继续保持2N？
5. T减半时参考速度和加速度怎样变化？为何不能据此直接推断actual误差也同比例变化？

下一小任务S17.8b等待明确请求，不自动实现。
