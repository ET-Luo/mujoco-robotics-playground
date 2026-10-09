# S18.5 — Teleoperation Integration：接触任务与输入恢复

约0.5～2h；前置：[S18.1](21_1_incremental_teleoperation.md)–[S18.4](21_4_force_feedback.md)。
[代码](../examples/19_teleoperation_dexterous/teleoperation_integration.py) / [示例入口](../examples/19_teleoperation_dexterous/README.md)。
Engineering / Learning唯一状态：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

将单项映射、clutch、接触控制和力显示接起来后，输入中断不能让本地控制器停摆，恢复也不能追赶旧手部位移。
直觉：输入是操作者的新意图；本地反馈始终照顾机器人当前状态。
本课选择中断时保持最后参考位置，恢复第一帧只重建master锚点，再从下一帧继续接收增量。

## Core Concepts / scope

复用Stage17固定姿态XY slide/球/墙模型，接触normal +world Y、condim1（仅法向，无切向摩擦）。
world anchor=[0,.06,0]m；workspace X±25mm、Y15–90mm、Z±20mm。
50Hz synthetic master、1kHz local feedback与physics，13s simulation；没有网络、ROS、多线程或硬实时保证。
clutch/recenter复用S18.2 `map_sample`，仅增加keyword lower/upper参数供此fixture使用，旧默认不变。
S18.4 `map_force`在每步提供虚拟device-on-hand信号，gain.5/cap1.5N，不写任何实际设备或robot force。
依赖requirements已有mujoco/numpy/matplotlib；local helper链为前四课与cartesian_spring/contact_transition，无新依赖。
keyboard可选，本课采用可重复script，不引入GUI。

三个case：nominal无中断、dropout_rebase正确恢复、dropout_wrong_recovery遗漏恢复rebase。
错误对照保留其他所有处理，专门观察“速度有界但动作语义错误”。
任务阶段按script时间安排，不是自主状态机；touch事件与实际contact连续性单独核验，不能把phase名称当接触证据。

## Mathematics：frame / shape / unit

映射和控制仍为：

```text
Δx_req_W = s R_WM (m_M[k]−a_M[k−1])     # (3,) m，R(3,3)，s无量纲
Δx_bounded = norm_limit(Δx_req_W, .02m/s × .02s)
x_ref = world_box_projection(x_ref + Δx_bounded)
F_W = 400N/m · (x_ref−x_actual) − D · v_actual
τ_req = Jp.T @ F_W                       # (2,) N，本fixture均为slide
ctrl = τ_req / gear                     # gear=[2,1]，实际广义力cap20N
```

world Jp(3,2)=[[1,0],[0,1],[0,0]]，v=Jp@qvel单位m/s。
真实有效质量M=diag(2,1)kg，D_xy=[56.568542,40]N·s/m。ZOH position reference、vd=0。
接触球半径20mm、refY15mm；实际Y约19.995mm时弹簧−2N与环境+2N平衡。
切向X运动沿无摩擦墙面，因此X滑动不直接改变Y反力；不是一般摩擦表面扫描验证。

有效packet要求：finite issued、`0≤now−issued≤.1s`、sequence为严格增加整数、master有限(3,)米。
本课timestamp/now都来自同一个simulation clock；sequence当前进程从0起，无跨进程epoch/重启协议。
拒绝包不能改变accepted sequence、last_issued、master anchor或reference；新包必须通过验证才能刷新lease。
`stale = now−last_issued > .1s`，代码含1e−12浮点容差。
无新有效packet时reference立即保持，即使lease尚未过期；stale flag用于声明状态及恢复rebase。

| 状态/事件 | reference处理 | master anchor | physics/local controller |
| --- | --- | --- | --- |
| 正常packet | 应用scaled bounded增量 | 更新当前读数 | 每1ms继续 |
| clutch断开 | 保持 | 每有效packet更新 | 每1ms继续 |
| 重新接合 / recenter | 消费该事件样本，保持 | 重建当前读数 | 每1ms继续 |
| missing / rejected input | 保持 | 不更新 | 每1ms继续 |
| stale后首有效packet | 保持，不补旧增量 | rebase | 每1ms继续 |

保持的是旧reference，不是把reference改成actual位置。本策略可以持续保持接触负载，不是卸力或物理急停。
恢复首帧丢弃gap及该样本位移，所以任务行程会变短；不是把丢失命令排队补执行。

## Math-to-Code / APIs

`packet_reason(...)`返回空字符串表示接受，或拒绝reason；它只验证，不修改状态。
`map_sample(..., lower=LOWER, upper=UPPER)`返回新ref、新anchor、diagnostics；输入验证、scale/norm/box仍复用S18.1/2。
每physics step先检查packet/lease，再`mj_forward`更新实际site，`mj_jacSite`写world Jp/Jr(3,nv)，读取site_xpos与qvel。
设ctrl后再次`mj_forward`重解当前motor/contact，`measure`用`mj_contactForce`的(6,)contact-frame输出及frame.T/geom sign得到environment-on-probe force。
`MjModel`由MJCF编译返回；`MjData`为可变实际state/buffers；`mj_forward`不推进time，Jacobian/contact函数原地写buffer。
`mj_step`原地推进qpos/qvel/time1ms，只在初始化设qpos，运行中不写qpos/qvel。
CSV为pre-step state/current command与contact；previous-command反力仅另记诊断，峰取两者最大。
物理检查：`M*qacc=actual_motor+Jp.T*reaction`，Euler半隐式递推`v_next=v+dt*a`、`q_next=q+dt*v_next`。

## Minimal Experiment / Expected

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/teleoperation_integration.py
python examples/19_teleoperation_dexterous/teleoperation_integration.py --dropout-duration 1.2
```

每命令三个case×13s；CSV/JSON/PNG只ignored tmp/s18_5_gap0.6与gap1.2。

| simulation时间s | script | 应观察 |
| --- | --- | --- |
| 0–3 | −master X15mm/s，scale1 | world Y60→15mm，actual检测touch |
| 3–4 | 静止 | contact约2N |
| 4–5 | −master Y20mm/s，scale.5 | 沿world+X滑动，保持normal contact |
| 5–5.6 | clutch断开，移动手柄 | reference保持，master可重定位 |
| 5.6 | 重接合rebase | 不应用旧位移 |
| 5.8 | master坐标加[.3,−.2,.1]m并recenter | reference无事件跳变 |
| 6–8 | 再沿+world X滑动 | 包含6.4s开始的输入中断 |
| 8–11 | +master X15mm/s、scale1 | world Y退回60mm并分离 |
| 11–13 | 静止 | final tracking/velocity恢复，无contact |

packet在20ms末记录此前区间master运动，scale按当前script阶段选择；8s边界切为scale1，该帧仍包含上一滑动区间的master增量。
这是明确离散事件顺序，不假定所有阶段端点都等于连续积分结果；clutch及rebase也消费事件样本。
nominal X最终触25mm box，超出部分丢弃；gap case在恢复后不补走，所以末X更小。

## Actual Result / Explanation

2026-10-09 DESKTOP-781D67A：WSL2 Ubuntu24.04.5/kernel6.6.87.2，conda mujoco所属Python3.12.14核验；
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/真实imports/API/helper路径通过，无安装或requirements改动。
两命令exit0，各三个case13000×32CSV；两PNG目视。工程PASS包含错误恢复的明确语义失败，不能称所有task通过。

| 量 | nominal | 正确gap .6 | 正确gap1.2 |
| --- | --- | --- | --- |
| physics/local feedback次数 | 13000/13000 | 13000/13000 | 13000/13000 |
| accepted packets | 649 | 619 | 589 |
| rejected packets | 0 | 2 | 2 |
| stale local samples | 0 | 519 | 1119 |
| 最终ref X mm | 25 | 23.8 | 17.8 |
| workspace裁剪samples | 24 | 0 | 0 |
| 过期开始s | 无 | 6.481 | 6.481 |
| 恢复s | 无 | 7.0 | 7.6 |
| 恢复首帧ref增量 | 无 | 0 | 0 |

所有case反力峰3.158396N，4–8s平均1.998002N且实际contact连续；motor无饱和。
末1s tracking<1µm、speed<.001mm/s，无contact；referenceY最终60mm。
输入中断最后有效issued=6.38s；6.48s age=.1s尚有效，6.481s才stale。
6.6s注入1s旧timestamp拒绝，6.62s注入duplicate sequence拒绝；二者都不刷新lease。
错误恢复两gap均产生+X .4mm额外reference位移，最终X24.2/18.2mm；限速仍通过、最终tracking也通过，但`recovery_no_motion_contract_passed=False`。
错误原始增量被限幅后仅应用.4mm，剩余丢弃，不持续追赶整个gap位移。
stale期间actual X仍变化约.629/.630mm，随后趋于旧ref；再次证明reference保持不等于实际位置冻结。

独立`tmp/s18_5_validation/check.py`读取CSV核对feedback/gear cap/动力学/Euler、无有效包时ref/issued不变、rebase/clutch不动、
lease边界、两拒绝reason、恢复delta、virtual force、实际contact保持与退回分离；六case通过。
共享map_sample默认/.5 S18.2回归各exit0；packet freshness边界/future/重复/非法seq/nonfinite guard通过；CLI duration0/nan/−1各exit2。
两源码3.11语法通过，未用3.11解释器执行；links/status/git diff --check通过。

## Failure Cases / limits

- rejected包刷新lease：通信看似恢复，但没有新的有效运动意图。
- 恢复直接沿旧anchor求差：将gap期间的手柄位移当作新指令；速度有界仍会错误动作。
- stale时停mj_step或保持旧ctrl：停止feedback/physics不等于物理制动，丢掉本地稳定响应。
- 自动把ref切到actual：改变接触弹簧负载；本课选择旧ref保持，不隐式卸力。
- clutch/recenter事件顺序不明：可能误把coordinate jump映为运动；真实协议还需epoch/显式状态及设备时间处理。
- 本课没有自动超力停止、接触丢失恢复或多故障重试。contact门槛和末尾checks是实验验收，不是在线硬件保护。
- 单次synthetic gap不证明跨机时钟、网络延迟/抖动、设备reset丢事件、hardware safety或双边passivity。

## Robotics Context

机器人端持续反馈、外部低频意图带有效期，是遥操作与远程控制常见职责划分。
恢复应说明哪些运动被丢弃、何时需操作者重建意图；这是控制接口语义，不只是通信重连。
本例限XY法向接触/无摩擦滑动，不拓展UR5e或下一阶段hand。

## Interview Capsule

**30秒：**用增量mapper与clutch/recenter生成有界reference，1kHz本地阻抗持续跟踪。
有效输入有sequence与issued lease；过期/拒绝保持旧ref，恢复首帧rebase以丢弃旧master增量。
nominal/两个gap都完成接触与退回；遗漏恢复rebase即使通过速度与tracking，也违反动作语义。

**2分钟：**分别说明master、ref、actual、ctrl以及input/feedback/physics时钟；
给出stale状态表、接触弹簧平衡、clutch/reset/recovery的锚点操作，解释gap使行程缩短；
最后用.4mm错误恢复说明tracking成功不等于意图正确，界定simulation lease与真实硬件协议的差别。

## Must Remember

- 只有有效包才能更新lease/anchor；拒绝不是成功通信。
- reference hold仍继续physics和feedback，也可以继续保持接触力。
- recovery rebase消费第一帧，丢弃历史运动；不自动补走。
- 验收意图正确、tracking、contact三项要分别观察。

## My Verification — Run / Modify / Explain

状态只在根README；助手验证不代替本人学习。
Run：默认执行，查看integration.png与JSON events，核对6.481s过期、7s恢复以及.4mm错误对照。
Modify：只把`--dropout-duration 1.2`，先预测stale时长、恢复时刻、末X、contact力和physics/feedback计数，再核对。
Explain：
1. missing input、stale input、rejected packet分别如何影响ref和lease？为什么拒绝包不能刷新lease？
2. stale时为何保留旧ref并继续反馈？为什么它不是物理停止，也不是卸力？
3. clutch重接、recenter和stale恢复分别为何要消费事件样本并rebase？
4. 错误恢复限速和末tracking都通过，为什么仍失败？被拒绝增量会继续补走吗？
5. gap从.6变1.2，为何末X变小、normal力和physics计数基本不变？本课lease验证有什么现实边界？

2026-10-09 本人明确确认实验与预测完成，并提交五项Explain；根README Run/Modify/Explain全部完成，Learning Mastered。
解释精度：错误恢复将gap累计master位移误当新增量，本例限幅后仅额外应用.4mm，其余丢弃，不持续补走整个历史位移。
Rebase保留当前robot reference，重建master差分锚点；不是重设actual robot state。
本次只同步文档，未重跑数值/GUI；runtime沿用2026-10-09工程验证。
