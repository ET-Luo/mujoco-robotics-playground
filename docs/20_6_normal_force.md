# S17.6 — Normal force control：用实际反力误差调整法向参考

约0.5～2h；前置：[S17.4](20_4_contact_transition.md)、[S17.5](20_5_admittance.md)。
[代码](../examples/18_compliant_control/normal_force.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning唯一状态：[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

S17.4固定参考产生约2N，但如果目标改成4N，固定参考不会自动知道。
本课读取真实MuJoCo接触反力，计算目标与测量之差，再移动法向参考：
力不足就向墙内移动，力过大就向外退；内层motor继续跟踪参考。
这是运动改变接触、接触反力反馈运动的闭环，区别于上一课的合成测力输入。
接触不存在时，0N不意味着应无限向前追赶；力反馈必须门控，先有限搜索，丢失后退出并本地hold。

## Core Concepts / Fixture / States

复用contact_transition.build_model/measure，依赖链到cartesian_spring：仅mujoco/numpy/matplotlib，
requirements不变。平面y=0、normal +world Y、球半径.02m、site在球心，
XY slide固定姿态，真实M=diag(2,1)kg，初始球心[0,.06]m、速度0。
condim1无摩擦；gravity/passive damping/external均零。
K_inner400N/m、D_inner[56.568542,40]N·s/m、dt1ms、gear=[2,1]、joint cap20N。
solref=[.01,1]、solimp=[.95,.95,.001,.5,2]、solver iterations100/tolerance1e−12。

| state | 行为 | 转移 |
| --- | --- | --- |
| 0 approach | yd=max(.015,.06−.03t)，限范围搜索；不启用力反馈 | active pair且反力>.05N → 1；3s未触碰 → 2 |
| 1 contact | closed/loss case使用力误差移动reference；fixed case固定.015 | gate失效 → 2 |
| 2 stopped | 锁存当时actual球心为reference，vd0，仍算motor并mj_step | 不自动恢复搜索 |

stopped是本地位置/阻尼hold，不是仿真pause或立即物理停止；没有清零actual qvel。
force_loop_enabled字段区别phase/state与实际控制分支；fixed有接触也不启用反馈。
无噪声/延迟滤波或去抖，本简化门控允许一次失效就锁存退出；不是通用接触管理器。

## Mathematics：frame / unit / shape

normal n_W=[0,1,0]，f_meas=n_WᵀF_environment→probe,W（N，正值）。
目标f_d为正反力幅值；probe→environment施力相反，不能混用符号。
力误差e_f=f_d−f_meas：

\[
u_y=-g_f e_f,\quad g_f=.003\ \mathrm{m/(N\,s)},\qquad
\bar u_y=\operatorname{clip}(u_y,-.02,.02),
\]
\[
y_{d,new}=\operatorname{clip}(y_{d,old}+h\bar u_y,.005,.06),\qquad
v_{d,y}=(y_{d,new}-y_{d,old})/h.
\]

力不足e_f>0，需要负Y运动参考向墙压；力过大e_f<0，正Y参考退让。
只在state1、gate有效、非fixed启用。gate无效时上述更新不执行，不用0N追目标。
reference范围5…60mm是球心参考范围，非actual位置安全保证；contact force-loop参考速度≤20mm/s，
approach速度30mm/s使用单独策略，不能称整个任务都限速20mm/s。

没有额外PI控制器I状态；但y_d确实积分了速度型力误差反馈，不能说“系统没有积分”。
位置范围投影直接回写唯一reference状态，vd由实际增量回写，避免隐藏的越界积分继续累积。
不另维护未裁剪reference；这不是完整PI anti-windup教程，也不证明遇到motor饱和时能正常恢复。

内层world F=K_inner(x_d−x)+D_inner(v_d−v)，v=Jp qvel，τ_req=JpᵀF，ctrl=τ_req/gear。
x/xd/v/vd为world(3,)m、m/s；Jp(3,2)、qvel(2,)m/s；τ_req(2,)slide force N。
X参考0，无切向扫描或hybrid selector，重点只在Y法向力。
真实法向动力学：m_y ydd=F_motor,y+f_env。稳态近似：

\[
f_{env}\simeq K_{inner}(y-y_d),\quad
 y_d\simeq y-\frac{f_d}{K_{inner}}.
\]

y≈.02m，目标2/4N的reference应分别靠近.015/.01m，软接触会带来小修正。
反馈根据测量修正reference，而不是预先把目标除以K就宣称已完成力控制。

## Math-to-Code / APIs / Measurement timing

```python
gate = bool(active and sensed[1] > .05)
requested_velocity = np.clip(-GAIN*(target-sensed[1]), -.02, .02)
reference = np.clip(reference + DT*requested_velocity, .005, .06)
velocity = (reference-previous_reference)/DT
```

上段更新仅force_loop分支执行；else分支在源码中清楚显示搜索/固定reference/停止hold。
MjModel构造返回编译模型，MjData返回actual可变状态。mj_forward原地更新FK/动力学/contact，
不推进time；mj_jacSite写world Jp/Jr到(3,nv)buffer；mj_fullM写(nv,nv)物理惯量。
mj_step原地推进qpos/qvel/time，全程不将actual状态设为reference。

measure复用S17.4：mj_contactForce(model,data,index,raw)原地填(6,)contact-frame
[force;moment]，前三N/后三N·m，关于contact.pos；frame行是world表达的接触轴，
frame.T转换到world，并按probe属于geom[1]/geom[0]取正/负号。
本课核验Jᵀcontact=qfrc_constraint、M qacc=actual+constraint，不能把motor力当测力。

采样pre-step：先用previous ctrl做fresh mj_forward，得到input_measured用于outer feedback/gate；
再设置current ctrl、forward重求解，current_reaction用于本步积分和误差验收。
两次求解同q/v、不同ctrl，测量来自仿真约束求解，没有硬件传感器/滤波或理想连续反馈。
目标改变不通过同一步求解器隐式消除误差；没有把测量设置为目标值。

## Minimal Experiment / Expected → Actual

仓库根目录：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/normal_force.py
python examples/18_compliant_control/normal_force.py --target-force 4
```

每命令四case，各8001×17 CSV（0…8s）、results.json、force_control.png。

| case | 实验问题 |
| --- | --- |
| closed | 正常接触后跟踪目标2N或4N |
| fixed | 仍固定yd=.015，不用力误差，说明测量本身不构成反馈 |
| loss | 前半同closed，3s把平面移到y=−.1m，验证gate退出 |
| absent | 平面从开始就在y=−.1m，搜索范围内无墙，3s超时 |

loss/absent改变固定geom位置是合成几何撤走场景，不模拟有质量/有限速度的移动墙。
实际球心/速度不瞬移；loss3s当步先修改geometry再fresh forward，controller发现gate失效后退出。
Expected：closed达到目标；fixed仍约2N；loss不继续追0N误差，absent不启动力反馈。
force tracking qualified要求7…8s每个采样有效接触、|current_reaction−target|<.05N、|vy|<2mm/s。
仅末1s采样窗口，不证明无限未来；峰值/全程误差另外记录。
peak absolute force error含approach无接触阶段，不能把它当稳态误差；同时报告current/input两个反力峰。
失联case预期force_tracking_qualified=False，工程PASS表示成功检出并验证退出，不表示失联仍跟踪目标。

Actual（2026-10-08，DESKTOP-781D67A，最终两个命令均exit0）：

| case / target | 末1s平均反力N | 末1s最大误差N | current反力峰N | 最后reference mm | tracking qualified |
| --- | --- | --- | --- | --- | --- |
| closed / 2N | 1.997421 | .004202 | 4.663290 | 14.999035 | 是 |
| closed / 4N | 3.994886 | .008332 | 4.899475 | 9.998001 | 是 |
| fixed / 2N | 1.998002 | .001998 | 7.130667 | 15 | 是（本目标恰巧匹配） |
| fixed / 4N | 1.998002 | 2.001998 | 7.130667 | 15 | 否 |
| loss / 2N或4N | 0 | 2或4 | 同相应closed | 19.995778 / 19.991544 | 否，明确退出 |
| absent / 2N或4N | 0 | 2或4 | 0 | 15 | 否，搜索超时 |

closed/fixed/loss触碰均1.334s；input测量峰6.358974N，与current峰不同。
loss在3s同时记录wall_removed/contact_lost；1666个force-loop采样后禁用，reference保持锁存值；
absent在3s记录search_timeout、force-loop始终0；两类末1s反力0，未把失败报告为tracking成功。
closed末1scontact100%、速度<2.3e−8m/s；全case motor无饱和。
参考限速/限位保护分支存在；本动态参数未触发它们，不称为动态饱和恢复验证。
静态符号probe：欠力→负Y、过力→正Y、零误差→0；超大误差映射到±.02m/s通过。

初版6s实验的4N目标未通过：5…6s最大误差.070953N，contact未丢失。
保留慢收敛结论，最终统一8s、末1s验收；没有放宽.05N门限。
reference最小值改5mm，使4N稳态reference≈10mm位于范围内部，避免范围边界混入本课力跟踪问题。
不把初版失败算PASS；8s结果不保证所有目标都能收敛。

每步J/Jr/M、motor cap、contact Jᵀ映射、动力学budget/零bias/passive/external通过；
shape/clock/finite及实际半隐式q/v递推逐点通过。
独立CSV复算inner law、outer law/范围投影与vd增量、gate使能、力平衡/depth；
恢复初始/峰值/撤墙时刻/末尾状态重求解contact反力通过。
两命令fixed轨迹/控制/测力完全相同（只有target字段改变）；说明未接入力反馈。
CLI target0/nan各exit2；4N PNG目视核验；本地文件链接/git diff --check通过。
产物只ignored tmp/s17_6_f2/f4，临时日志/tmp。无GUI、UR5e、ROS、真实测力/硬件验证。
WSL2 Ubuntu24.04.5/kernel6.6.87.2，Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2，
当前conda hook激活mujoco，同shell核验python、metadata、真实module paths；脚本真实API执行通过。
无安装/requirements改动，未在Python3.11执行。

## Explanation / Failure Cases / Robotics Context

力误差反馈能适应目标变化，fixed虽测量反力却不使用它，因此4N目标下仍约2N。
在2N目标上fixed误差更小只是本fixture/reference恰好匹配，不能仅凭这一组宣称它具有力反馈能力。
力闭环有有限收敛时间；初次触碰仍有速度/动能，峰值不是目标自动限幅。
本课没有超力急停，也不把目标2/4N当碰撞峰值预算。

接触丢失时直接继续积分e_f会向前追赶不存在的墙；门控与有限搜索限制该行为。
退出时reference切到当前actual且vd0，但仍可能有非零速度；inner damping通过动力学刹车，
而不是清零qvel或暂停mj_step。此退出方式未证明硬件停止安全。
启动/触碰/退出的速度reference可跳变；本课没有速度连续轨迹或未知表面自动恢复。
零误差时reference速度为零，不代表motor力为零：保持力靠reference与actual偏差持续存在。

错误反力符号会把不足当过大；高增益、延迟、噪声、contact刚度、motor/reference限幅都可能改变稳定性。
仅验证无噪声低DOF平面/球、指定参数与2/4N目标；未验证通用稳定性、未知曲面、动态饱和或PI抗饱和。
应用为恒力压紧、擦拭与装配接触；切向position/法向force组合留下一小任务S17.7。

## Interview Capsule

30秒：把world环境→工具法向反力作为正测量，计算e=f_d−f_meas。
欠力时以负Y参考速度向墙压，过力时退；有界积分reference再由inner motor跟踪。
只在有效接触时启用，丢失或搜索超时退出并本地hold，分别报告末窗口误差与峰值。

2分钟：说明球心/平面/normal frame，写outer速度律、reference投影与inner阻抗/Jᵀ映射。
区分测量反力、motor力和目标；用m ydd=F_motor+F_env解释动态与稳态。
描述previous/current ctrl两次求解的时机；固定reference在2N恰巧成功但4N失败，闭环两目标均通过。
讲清gate、退出后继续physics、reference积分但无额外PI状态，最后限制在指定fixture与参数。

## Must Remember / My Verification — Run / Modify / Explain

力目标不是峰值上限；有测量不等于闭环，必须让误差影响控制。
力反馈需要contact gate；参考积分需范围投影；stopped hold不等于立即物理停止。
Engineering Complete；2026-10-08 本人确认实验与预测完成，并提交五项Explain，
反馈逻辑/固定reference对照/积分与动态停止的核心解释正确，Run/Modify/Explain完成，Learning Mastered。
状态只见根README。精度补充：本人用假设的X法向说明符号，实际本fixture是world Y，
墙对工具为+Y、欠力时reference向−Y；本课零gravity，静态motor Y力平衡接触反力，
一般机器人还可能需要重力/bias补偿，hinge才对应torque单位。
本gate具体为active pair且反力>.05N；search timeout与reference bounds是独立保护逻辑。
reference积分数学上也累积力误差，但用途/状态与另加PI补偿不同；边界直接回写reference，
本课未验证动态motor饱和或PI抗饱和恢复。

Run：运行默认命令，看tmp/s17_6_f2/force_control.png与JSON，找到loss/absent的退出事件。
Modify：先预测只将target2→4N对closed最终reference/反力、fixed反力与误差、loss后使能的影响，
再运行--target-force 4；其他参数不变，对照预测与实际。
Explain：

1. f_meas正方向是什么？欠力/过力时reference应向哪边移动，为什么速度律有负号？
2. 为什么fixed在2N看似成功，却不能跟踪4N？测量接触力与力反馈控制有什么区别？
3. 哪些条件启用force loop？接触丢失后若继续用0N误差积分会怎样？
4. 为什么误差接近0时仍需非零motor力？用法向动力学解释motor力与反力的关系。
5. reference积分与PI积分状态怎样区别？范围投影做了什么，退出hold为什么不等于清零真实速度？

下一小任务S17.7 Hybrid position-force等待明确请求，不自动实现。
