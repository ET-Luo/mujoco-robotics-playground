# 项目经验与进度交接

最后整理：2026-09-30（S8.7 CPU 烟雾测试完成，待本人解释）。供新会话快速恢复上下文。
先读根目录 [AGENTS.md](../AGENTS.md)，再读本文；执行前重新检查实际环境。

## 当前目标与边界

用户已选择 S5.3。讲解每轮重算 FK/Jacobian、最多接受 20 次更新、最后一次也检查误差。
教学关节范围均为 [-π,π] rad，候选越界则拒绝并停止；不据此判定目标不可达。
本人正确回答候选 q2=3.15 rad、不能接受，以及用尽次数后应未成功停止。
本人正确填写 examples/06_inverse_kinematics/iteration.py 的成功判断、任一关节越界判断
和候选接受三处表达式。2026-09-29 核验 WSL2 Ubuntu 24.04.5，激活核验 mujoco 与
所属 Python 3.12.14 后运行，退出 0，无导入错误。误差由 0.003 m 逐次约减半，
第 5 次更新后为 0.000093753023 m，首次满足 1e-4 m 容差并打印 SUCCESS。
本次候选均合法，未实际触发限位失败分支；未运行 MuJoCo/GUI。
本人正确解释第 0 条是初态，因此 5 次更新对应 6 条记录；程序因满足容差停止；
本次未触发越界，不能证明限位失败分支正确。S5.3 已勾选。
S5.4 本人正确预测 target=(0.8,0) m 的几何最小残差 0.1 m，并判断误差下降但未达容差
不能成功。iteration.py 增加目标 xy 参数；在核验的 mujoco/Python 3.12.14 环境先复跑
默认目标，仍于第 5 次成功。不可达目标运行退出 0，误差从 0.5 m 曾降至约 0.10186 m，
之后因接近伸直奇异姿态及无阻尼直接求逆而反复增减，第 20 次为约 0.165764 m，
以 `FAILED: update limit reached` 结束；未触发关节限位或求解异常。
本人正确回答终止原因为次数上限，并用完全伸直仍无法触及解释几何不可达。
补充澄清次数上限本身不能证明一般目标不可达。S5.4 已勾选，Stage 5 完成。
S6.1 本人正确回答：UR5e ctrl 为 rad 的关节位置目标，不能直接写入末端位移；
位置误差用世界系和 m，Jacobian 映射输出 delta_q 为 rad。S6.1 已勾选。
S6.2 平面两连杆单步限制：本人首次比例/矩阵乘法有误，经讲解后确认 scale=0.4、
受限 delta_q=(0,0.004) rad、线性位移=(-0.0012,0) m，并正确理解这是更新量限制。
新增 examples/07_cartesian_control/main.py。2026-09-29 核验 WSL2 Ubuntu 24.04.5，
激活核验 mujoco 与所属 Python 3.12.14 后运行退出 0，无导入错误。FK 真实位移约
(-0.0011999968,-0.000002399997) m，误差从 0.003 降至 0.0018000048 m；
限制与误差减小断言通过。本人正确解释缩放导致未一步到达、最大分量满足限制，
并指出 FK 圆弧几何产生微小 y 位移。S6.2 已勾选。
S6.3 开始：control_dt=0.02 s、max_joint_speed=0.2 rad/s，由两者计算每轮最大增量；
本人正确算出每轮 0.004 rad、示例速度 0.15/-0.2 rad/s 并判断满足限制；
对 dt 减半的影响首次误答为增量变大，已解释 step_max=speed_max*dt，新限制为 0.002 rad。
本人确认后新增 examples/07_cartesian_control/tracking.py。2026-09-29 核验 mujoco/Python
3.12.14 后运行退出 0；3 次更新使误差从 0.003 m 降至 0.000000590488 m，假定时间
0.06 s。第二关节前三轮命令速度为 0.2/0.2/约 0.098206 rad/s，速度限制断言通过。
这是几何命令速度，没有 mj_step/执行器动力学，不能当作实际 qvel。本人正确解释
初态使 3 次更新有 4 条记录，后两项暂不清楚；已讲解前两轮因原始增量超限而饱和在
0.2 rad/s、第三轮原始增量较小故 scale=1，以及命令速度与 data.qvel 的区别。
本人随后确认限幅饱和和几何命令速度，S6.3 已勾选，Stage 6 完成；不宣称实际 qvel。
S7.1 开始：拟创建半径 0.05 m、球心 z=0.20 m 的自由球体落向 z=0 plane，先预测
初态接触、首次接触高度和接触是否增加关节。本人正确答 ncon=0、约 0.05 m、不增加。
新增 contact_scene.xml/main.py；2026-09-29 核验 mujoco 环境，Python 3.12.14、MuJoCo
3.13.0，运行退出 0。第 88 步 t=0.176 s 首次接触，球心 z=0.049789280 m，geom 对
ground/ball_collision，接触 z≈-0.00010536 m、dist=-0.00021072 m；离散步进下有轻微穿入。
nq=7、nv=6、njnt=1 前后不变。待本人解释动态 ncon、不改变 nq 和球心略低于半径，
本人正确解释 ncon 随状态变化；对 nq 暂不清楚，并将轻微穿入归因于精度/形变。
已讲解 contact 是 MjData 临时约束、不改变 MjModel 拓扑；轻微穿入主要来自 0.002 s
离散推进和软接触求解，geom 仍是刚性几何。待本人确认后完成 S7.1。
本人确认上述两点，S7.1 已勾选。
S7.2 开始两个对称 slide 关节的夹爪：范围 [0,0.04] m，基础开口 0.02 m，
opening=0.02+q_left+q_right；位置执行器 ctrl 单位 m。待本人手算开口、命令方向和
step 前 qpos 是否变化。本人首次答 0.02 m/打开/不会；打开与不会正确，开口漏加两侧
位移，应为 0.04 m。已澄清 ctrl 只改目标、不推进动力学，qpos 需经 mj_step 逐步响应。
本人确认后新增 gripper.xml/gripper.py。2026-09-29 核验 mujoco 环境，Python 3.12.14/
MuJoCo 3.13.0 后运行退出 0。两执行器正确映射左右 slide；初态 qpos 各 0.01 m、开口
0.04 m，写 ctrl 各 0.03 m 后 step 前 qpos 不变，1000 步/2 s 后 qpos 各 0.03 m、
开口 0.08 m；目标与对称断言通过。待本人解释开口变化、映射及是否证明抓取能力后
完成 S7.2。本人首次解释把变化归因于基础开口、把映射表述为执行器决定运动距离，
并只提抓取位姿；已澄清变化量为左右各 0.02 m 之和，映射是 actuator 目标传给 joint
而实际响应由动力学决定，且无物体/接触/摩擦/保持实验不能证明抓取。待本人确认。
本人确认上述三点，S7.2 已勾选。
S7.3 开始固定位置 Reach 判据：世界系三维欧氏距离严格小于 0.01 m 才成功；恰好
0.01 m 不成功，超时也不是成功。本人正确算 e=(0.006,0.002,0) m、d≈0.006325 m
并判断成功/边界失败。新增 reach_criterion.py；首次测试因 `numpy.bool_ is True` 身份比较
错误退出 1，修正显式 bool 和稳定边界构造后复跑退出 0：内部/边界/外部为
0.006324555 True、0.010000000 False、0.010001000 False。只验证判据，不含控制/仿真。
待本人解释 Reach 与抓取、超时关系后完成 S7.3。未运行 GUI或新增依赖。详见 09 笔记。
本人正确解释 Reach 不等于稳定抓取，且 0.012 m 残差在超时时属于未成功；S7.3 已勾选。
S7.4 开始：夹爪初始开口 0.08 m，中心物体宽 0.04 m，双指目标各 0.005 m（空载开口
0.03 m）。理想接触时双指 q 各约 0.01 m；以左右各有物体接触作为本步成功现象，
不等同稳定抓取。本人正确答 q 各 0.01 m、接触不等于夹紧；反力解释基本正确，已澄清
并非定位精度主导。新增 gripper_contact.xml/close_contact.py。2026-09-29 核验环境后
运行退出 0；第 27 步/t=0.054 s 双侧接触，qpos 各约 0.00901662 m、ctrl 各 0.005 m、
开口 0.038033237 m，物体仍在原点。差异来自接触阻挡及软接触穿入；未验证保持/重力/
扰动/抬升。待本人解释结果后完成 S7.4。未运行 GUI、无新依赖。详见 09 笔记。
本人正确解释软接触穿入及稳定抓取需保持/夹紧证据；对 qpos 未达 ctrl 首次误认为进入
成功容差。已澄清脚本没有关节位置容差，而是在双侧接触时返回；差异来自接触阻挡及
继续运行时的执行器—接触力平衡。待本人确认后完成 S7.4。
本人确认接触反力阻挡手指，S7.4 已勾选。
S7.5 开始预设夹持状态的小幅抬升：夹爪基座目标 +0.05 m；定义物体 z 增量至少
0.04 m 且相对夹爪竖直位置变化不超过 0.01 m 为本课成功现象。待本人判断夹爪上移但
物体只升 0.002 m，以及物体升 0.048 m/相对变化 0.002 m 两组结果，再创建重力场景。
本人正确判 A 失败/B 成功并指出必须检查物体。新增 gripper_lift.xml/lift.py；重力下先
稳定夹持，再将基座目标斜坡到 +0.05 m。首次 kp=500/阶跃只抬物 0.016736 m，退出 1；
第二次 kp=2000/斜坡抬物 0.036966 m，退出 1；均保持原判据，失败源于有限位置伺服刚度
导致重力稳态下垂。最终 lift kp=5000 运行退出 0：基座升 0.047886057 m，物体升
0.040620966 m，相对 z 变化 -0.007265091 m，满足 >=0.04 m 和 <=0.01 m 两项判据。
待本人解释稳态下垂、两项证据和负相对变化后完成 S7.5。未运行 GUI。详见 09 笔记。
本人正确解释两项判据及负方向，基座误差仅答重力；已补充有限 kp 位置伺服需靠误差
产生支撑力，因此重力下有稳态下垂，负相对 z 表示物体相对夹爪向世界 -z 滑动。
本人确认上述解释，S7.5 已勾选。
S7.6 开始：从持物抬升状态按下降→确认物体/地面支撑→张爪→夹爪撤离→复查物体顺序
做放置释放。成功需物体接近目标且由 ground 支撑、手指不再接触、末段速度小；先张爪
再落地属于掉落。本人正确解释提前下坠并排对顺序；释放后只提检查夹爪再接触，已补充
还需 ground 支撑、目标位置和末段速度。待本人确认完整证据后实现。未运行新 MuJoCo/GUI。
详见 09 笔记。
本人确认完整证据。gripper_lift.xml 升降范围扩至 -0.06 m，新增 place_release.py。
2026-09-29 先复跑 lift.py 退出 0，物体升 0.042733988 m、相对变化 -0.007265089 m。
place_release.py 运行退出 0：下降至 object_z=0.029914602 m 检测 ground 支撑后才张爪；
撤离稳定后物体位置约 (0,0,0.029362824) m、速度 1.36e-10 m/s，ground=True、
finger contact=False、xy/低速检查 True，Place success=True。待本人解释支撑触发、软接触
z 与四项证据，再完成 S7.6 和组合任务清单。未运行 GUI。详见 09 笔记。
本人正确解释需等实际支撑以免未稳定释放，以及软接触允许少量穿入；四项证据仅笼统答
排除未稳定，已拆分讲解：地面支撑排除悬空/下落、无手指接触排除未释放、xy 排除错位、
低速排除滑动/弹跳。待本人确认后勾选并写清单。
本人逐项确认，S7.6 已勾选，Stage 7 完成。09 笔记已写 11 步组合清单，从 reset、Reach、
对齐、闭爪、保持、抬升、转移、下降、支撑、释放/撤离到终检；仅为清单，未实现状态机。
S8.1 开始为平面 Reach 定义接口：observation 8 维（q/qvel/末端 xy/目标 xy），action 为
±0.2 rad/s 两关节速度命令，dt=0.02 s；reward=-distance；distance<0.01 terminated，
第 100 步未成功则 truncated。本人正确写出 observation 形状 `(8,)` 及四段单位
rad/rad/s/m/m；距离 0.006 m 时正确得到 reward=-0.006、terminated=True；第 100 步
距离 0.012 m 时正确得到 terminated=False、truncated=True。S8.1 已勾选；未安装
Gymnasium/RL 库或实现接口。下一小任务为 S8.2：先明确最小接口与依赖，再只实现
reset 和 observation。
S8.2 开始：本人确认 reset 后 q=(0,0) 时末端为 (0.7,0) m，因为 reset 恢复初始关节
状态，末端再由 FK 得到。已说明 Gymnasium 用于标准 Env/Box/seed 接口，将其加入
requirements，并在核验的 mujoco 环境安装 Gymnasium 1.3.0。新增 reach_env.py 骨架，
包含固定状态、FK、observation space 和 reset；本人正确填写四段 observation 拼接。
2026-09-29 在核验的 mujoco 环境运行：reset 返回预期 8 维 float64 数组，space 包含
检查通过；人为修改状态后再次 reset 正确恢复 q/qvel/step_count。待本人解释 reset
返回值和 FK 重算语义；尚未实现 step 或训练算法。
本人正确解释 observation 是智能体看到的状态输入、info 是额外信息；经补充明确 policy
通常不使用 info。本人正确解释末端 xy 是当前 q 的 FK 派生量，不能保留上一回合的值。
S8.2 已勾选，下一小任务为 S8.3：加入 step，区分 terminated/truncated，并用短随机
动作回合检查接口。
S8.3 本人正确完成动作限幅、几何速度积分、距离奖励及终止/截断表达式。助手集成
`reach_env.py` 并在核验后的 mujoco 环境运行 Gymnasium 1.3.0 环境检查、动作边界、
100 步随机回合与第 100 步成功优先检查，均通过。随机回合以约 0.295914 m 残差截断；
第 100 步成功案例为 terminated=True、truncated=False。环境检查仅提示 observation
无穷边界。本人正确解释随机回合只检查接口而不要求随机策略成功，并指出直接赋值的
几何命令速度不同于物理引擎综合执行器、惯性、重力、接触、约束和积分后得到的
`data.qvel`。S8.3 已勾选；未运行 MuJoCo 或 GUI。下一小任务为 S8.4：纸面对比两种
距离奖励的数值和偏好，再做最小数值检查，不开始 PPO/SAC。
S8.4 本人正确计算 `-d` 与 `-(d^2)` 在 0.02/0.20 m 的四个值，以及从远到近的
奖励改善 0.18/0.0396；理解两者均偏好更小距离，以及平方形式在小于 1 m 时相对比例
更大但绝对尺度更小。新增 reward_comparison.py，在核验后的 mujoco 环境运行退出 0，
打印值与单调性/数值断言通过。S8.4 已勾选；未运行 MuJoCo、GUI 或训练算法。
下一小任务为 S8.5：只解释 PPO 的 rollout 与更新数据流并手算一个小样本目标，保留
实现 TODO，不安装或运行训练框架。
S8.5 本人正确计算单步 critic target=-0.149、advantage=0.051，并正确完成正 advantage
下 ratio 1.30→1.20、最终 objective 0.0612，以及负 advantage 下 ratio 0.70→0.80、
最终 objective -0.04 的裁剪手算。本人解释正负 advantage 对动作概率的方向，以及
裁剪限制一次策略变化。PPO README 已记录 rollout→target/advantage→有限轮更新的数据流；
S8.5 已勾选。未安装训练依赖或实现网络/优化器。下一小任务为 S8.6：解释 SAC 的
replay、双 critic 和 entropy，并手算一个简化 target；保留实现 TODO。
S8.6 本人正确得到双 critic 较小值 -0.20、entropy 修正值 -0.15 和 critic target
-0.1985，并解释较小值用于约束过高估计。本人正确区分 SAC off-policy replay 复用与
PPO on-policy rollout，也正确解释增大 alpha 更强调 entropy、通常使策略更分散。
S8.6 已勾选；SAC README 已记录数据流，训练实现保持 TODO。未安装训练依赖或运行
Python/MuJoCo/GUI。S8.7 按路线要求需用户单独授权后，才能选择一种算法做 CPU 短运行。
用户已单独授权 S8.7 并选择建议的 PPO。安装 Stable-Baselines3 2.9.0；默认解析曾开始
下载带 CUDA 13 依赖的 PyTorch，发现后中止且未安装，改为 PyTorch 2.14.0+cpu，
requirements 已固定 CPU index/版本。新增 train_smoke.py，以 64 步 rollout、2 update
epochs、256 总步和 `[32,32]` 网络在 CPU 运行退出 0，耗时约 0.132 s；训练日志正常。
训练前/后确定性距离约 0.286/0.301 m，均在 100 步截断，不能证明收敛或失败。密集网格
进一步验证每关节 ±0.4 rad 的回合可达角范围内最小距离约 0.1478 m，当前 0.01 m 目标
事实上不可达。待本人解释烟雾测试证明了什么/未证明什么，以及动作、dt、horizon 如何
造成不可达后再勾选 S8.7。未运行 MuJoCo/GUI，未保存模型。

当前接续 S5.2：本人正确提交 error、delta_q、q_new 三行 NumPy 表达式，
助手原样集成 examples/06_inverse_kinematics/main.py。重新核验 WSL2 Ubuntu 24.04.5，
激活核验 mujoco 与所属 Python 3.12.14 后运行该脚本，退出 0，无导入错误。
FK 重算误差长度从 0.003 m 降至 0.001500010937 m，仍高于 1e-4 m 成功容差。
本人正确判断误差减小但未成功；起初因半步误选线性预测作为真实误差，
助手解释 q_new 已含 alpha 及圆弧与切线近似差异后，本人正确确认完整增量也须重算 FK。
详见 [IK 笔记](07_inverse_kinematics.md)。S5.2 已勾选。
本轮保留已有修改，仅同步五份文档，检查链接与 git diff --check；上述运行证据来自
2026-09-29 前一轮，本轮未重跑 Python/MuJoCo/GUI，无循环或新依赖。
下一小任务为 S5.3：先讲解有上限的迭代、每轮刷新 Jacobian 和关节限制，再由本人实现。
以下为此前已完成任务的记录，其中旧的“下一小任务”以本段为准。

S4.1 最新反馈：本人起初将 +90° 后世界位置误答为 (0.4,0,0.5) m，
助手用随杆尺子解释正确位置 (0,0.4,0.5) m。本人随后明确理解相对局部系没动、
相对世界系在动，不再标为概念未理解。本人已正确完成杆中点 p_J=(0.2,0,0) m 手算：
R_WJ*p_J=(0,0.2,0) m，p_W=(0,0.2,0.5) m。S4.1 已勾选。
示意图与矩阵由助手提供，不宣称本人独立推导矩阵或掌握齐次变换。
S4.2 已开始：基座在世界原点，L1=0.4/L2=0.3 m，q2 为相对第一杆的角度。
本人正确回答 (q1,q2)=(0°,0°)/(90°,0°) 末端为 (0.7,0,0)/(0,0.7,0) m。
本人已正确判断 (90°,90°) 时第二杆朝 -x、位移 (-0.3,0,0)，末端 (-0.3,0.4,0) m。
随后截图提供正确公式 x=L1*cos(q1)+L2*cos(q1+q2)、y=L1*sin(q1)+L2*sin(q1+q2)。
本人已提交正确 NumPy x/y 两行，助手原样集成 examples/04_forward_kinematics/main.py。
检查 WSL2 Ubuntu 24.04.5，在同一 shell 激活并核验 mujoco、对应 Python 3.12.14 后，
运行 `python examples/04_forward_kinematics/main.py` 退出 0，无导入错误。
两姿态分别输出 [0.7,0,0] / [-0.3,0.4,0] m，绝对容差 1e-12 m 检查通过。
S4.2 已勾选；核心公式由本人编写，包装与执行检查由助手完成。纯 NumPy 计算，未运行 MuJoCo/GUI。
用户已选择 S4.3：静态读取 attachment_site，位于 wrist_3_link、局部 pos=(0,0.1,0) m。
讲解 site_xpos 世界位置、site_xmat 按行存储的 3×3 世界朝向，矩阵列是 site 各轴世界方向。
本人正确预测局部不变、世界位姿可能变、forward 不推进时间。
新增 examples/04_forward_kinematics/site_pose.py；重新核验 WSL2 Ubuntu 24.04.5，
激活检查 mujoco、Python 3.12.14/MuJoCo 3.14.0 后运行退出 0，无导入错误。
home 世界位置约 (-0.133998,0.491999,0.488) m，shoulder_pan qpos +0.1 rad 后
约 (-0.182446,0.476164,0.488) m；局部位置 (0,0.1,0) 与 time=0 保持不变，朝向改变。
本人正确读取 home 朝向矩阵，回答 site +y/+z 轴分别指向世界 -y/-z，S4.3 已勾选。
本人完成预测和方向解释，读取代码与执行由助手完成。本轮仅同步五份学习文档，
检查 pwd/git status、链接与 git diff --check，未重跑仿真。
用户已选择 S4.4：开始 Jp 的世界 x/y/z 行、速度自由度列、m/rad 单位和 Δp≈JpΔq。
本人完成假设列乘角度增量手算，纠正 x 增减判断，正确算出 x=0.2995 m。
新增 examples/05_jacobian/main.py：home 处 mj_jacSite，shoulder_pan +1e-6 rad 前向差分。
2026-09-29 核验 WSL2 Ubuntu 24.04.5、激活核验 mujoco 后运行退出 0；
Python 3.12.14、MuJoCo 3.14.0。Jp=(3,6)，该列约 (-0.491999298,-0.133997825,0) m/rad。
最大列差 2.458e-7 m/rad，通过 1e-6 绝对容差；恢复位置与时间不变检查通过。
本人正确解释 x/y 均减小，位移单位 m 不能直接与 Jacobian 列比较，S4.4 已勾选。
本人追问列如何得到；补充 FK 偏导和 hinge 的 a_W×(p_W-o_W) 几何来源。
助手激活核验 mujoco 后临时读取世界轴/锚点，叉乘与 mj_jacSite 列 1e-12 容差核对通过，
退出 0，未调用 mj_step/GUI。详细数值和解释见 06 笔记；不宣称本人已独立推导。
用户已选择 S4.5：先用平面两连杆 L1=0.4/L2=0.3 的 2×2 xy Jacobian 讲解，
伸直 (0,0) 时 [[0,0],[0.7,0.3]]，弯折 (0,π/2) 时 [[-0.3,-0.3],[0.4,0]]。
本人正确给出伸直时第二关节 +0.001 rad 的位移预测 (0,0.0003) m。
讲解最小奇异值和条件数后新增 examples/05_jacobian/planar_singularity.py。
2026-09-29 核验 WSL2 Ubuntu 24.04.5、激活核验 mujoco 后运行退出 0，
Python 3.12.14、NumPy 2.5.3。弯折/近伸直 σ_min 约 0.222675/0.000157568 m/rad，
条件数 2.42013/4833.33；完全伸直参考 σ_min=0、κ=inf。手算矩阵和行列式核对通过。
本人首次选了完全伸直的参考组，澄清比较前两组后，正确理解“某个方向响应很弱”，
并正确回答 σ_min>0 的近伸直属于“接近奇异”。S4.5 已勾选，Stage 4 五项学习练习完成。
用户已选择 S5.1：用两连杆 q=(0,π/2)、p=(0.4,0.3) m，目标 (0.397,0.3) m，
J=[[-0.3,-0.3],[0.4,0]] m/rad，手算 e、Jδq=e 及 q_new=q+0.5δq。
解释 α 为无量纲步长比例，位置容差 1e-4 m；更新后须 FK 重算误差，迭代上限不等于成功。
本人初答误差符号和半步小数有误，δq=(0,0.01) 正确；助手纠正后本人正确预测 x=0.3985 m。
2026-09-29 助手激活核验 mujoco 后以临时 Python 代入 FK，退出 0；
新 p≈(0.39850000625,0.299996250008) m，误差长度≈0.001500010937 m。
本人正确判断误差尚未满足成功容差，不能宣布成功；S5.1 已勾选。
下一小任务为 S5.2：手写单次 Jacobian 更新并检查误差变化，尚未开始；无新增代码文件/IK 循环。
已同步文档并检查链接和 diff，未运行 MuJoCo/GUI。详见 07 笔记。
本轮仅同步五份文档并检查链接与 git diff --check，未重跑实验；不推断本人掌握 SVD 推导。
未运行物理仿真/GUI、不实现 IK，命令和结果详见 06 笔记。
未调用 mj_step/GUI，不进入 IK/控制器。详见 FK 笔记。
本轮仅同步五份学习文档，检查 pwd/git status、文档链接与 git diff --check，未运行仿真。

最新任务：用户已选择 S4.1。沿用 Stage 3 单杆几何讲解 W/J/E 与 p_W=R_WJ*p_J+t_WJ。
本课 J 原点在世界 (0,0,0.5) m、轴随杆旋转；E 位于杆端、轴与 J 平行，均为教学定义。
助手演示 q=0 平移，给出绕 +z 旋转 +90° 的矩阵；本人通过后续杆中点题完成手算。
详见 [FK 笔记](05_forward_kinematics.md)。S4.1 已勾选，尚未实现 FK 或修改模型；未运行 Python/GUI。
本轮工作树编辑前干净；链接与 git diff --check 检查通过。以下为 Stage 3 历史过程。

2026-09-28 用户再次请求继续 Stage 4：核对最新 README 后接续 S4.1 同一手算题，
Stage 3 保持已完成。补充局部/世界坐标的直观解释，待本人回答 p_J 与旋转后 p_W；
没有新增代码或运行仿真，本轮仅更新本交接与 FK 笔记并检查链接和 diff 格式。

最新进度（2026-09-28）：本人给出 S3.1 首次答案，两项误差和 D 项正确，
P 项误算为 1 N·m，因此总力矩写为 0.8 N·m；本人请求详细讲解。
助手解释 P=10×0.2=2 N·m、总力矩=1.8 N·m，以及单位和 D 项符号。
后续 q=q_target、v=+0.1 的练习中，本人正确回答 P=0，但漏掉 D 项负号；
助手解释后，本人正确判断 v=-0.1、目标速度为零时 D 项朝正方向。
本人随后正确复算 P=2、D=-0.2、总力矩=1.8 N·m，并用尚未到目标角度、
实际速度为 +0.1 而目标速度为零解释。S3.1 已勾选，未实现控制器。
S3.2 已开始：本人正确判断固定力矩不会自动归零，并指出 P 项随位置变化；
误差数值回答正确，变量误差表达式由助手讲解。本人独立写出正确公式
`torque = kp*position_error+kd*velocity_error`。
现已新增 examples/03_pd_control/single_joint.xml 与 main.py：无重力的单 hinge，
motor gear=1；初版三个 PD 表达式留作 TODO，完成过零力矩基线验证。
本次在 WSL2 Ubuntu 24.04.5 激活并核验 mujoco，解释器为
/home/lucas/miniconda3/envs/mujoco/bin/python（3.12.14），MuJoCo 3.13.0。
运行 `python examples/03_pd_control/main.py` 退出 0，无导入错误；nq=nv=nu=1，
1,000 步后 time=2 s、q=0.3 rad、v=0、目标误差=0.2 rad，符合零力矩静止预测。
本人现已正确填写三个表达式并提供运行截图；助手保留本人公式，清理过时 TODO 注释
与零占位提示。重新激活并核验 mujoco 后复跑同一命令退出 0，无导入错误，与截图一致：
首步力矩 2 N·m；0.5 s 时 q=0.487531、v=0.074872、ctrl=-0.025058；
2 s 末态 q=0.499998 rad、v=0.000010 rad/s、误差=0.000002 rad（打印精度）。
本人已正确解释 D 项制动强于 P 项；最初不清楚 UR5e 区别，经讲解后正确回答
此处 ctrl=0.5 是 +0.5 N·m 力矩，不是停在 0.5 rad。S3.2 已勾选。
S3.3 本人正确预测首步力矩 A=2、B=8 N·m，B 初始加速更强。
助手保留 PD 公式，新增 --kp 10/40、逐步采样、指标和 --plot PNG/CSV。
本轮重新检查 WSL2 Ubuntu 24.04.5，激活并核验 mujoco 与解释器，运行两组均退出 0。
固定 Kd=2、同初值 q=0.3/v=0、同目标 0.5 rad，1,001 点覆盖 0～2 s。
首次达到 0.48 rad：A=0.422 s、B=0.094 s；A 采样未见超调，B 最大角度
0.511036763 rad（0.156 s），超调约 0.01104 rad。数据形状、时间、初值、有限值检查通过。
单组 PNG/CSV 位于 logs/pd_kp10、logs/pd_kp40，助手另生成 pd_kp_comparison.png 并目视检查。
命令与指标定义见示例 README。本人正确回答 B 更快且有超调，不能只凭更快选择 B，
并请求再次解释超调；助手说明峰值减目标约 0.01104 rad，以及到达目标时仍可能有速度。
补充初始加速度大本身不保证超调。S3.3 已勾选。
用户已选择 S3.4：固定 Kp=40，拟比较 Kd=0.5/2；同初值 q=0.3、v=0、目标 q=0.5、v=0。
本人正确计算 D=-0.05/-0.2，最初误判首步力矩不同；解释 v=0 时 D=0 后，
本人正确判断初始角加速度相同。main.py 新增 --kd、--steps、末段 0.5 s 误差/速度指标。
本轮重新核验 WSL2 Ubuntu 24.04.5，激活并核验 mujoco 解释器后，两组 2 s 均退出 0；
A 末段仍有残余振荡，因此两组从初态延长至 4 s，均退出 0，无导入错误。
超调 A/B 约 0.11745/0.01104 rad；3.5～4 s 最大绝对误差约 2.298e-8/4.441e-16 rad，
最大绝对速度约 8.252e-7/8.882e-15 rad/s。CSV 形状、有限值、时间、初态、前缀重现检查通过。
logs/pd_kd_comparison.png 已目视检查；命令和详情见示例 README 与 PD 笔记。
本人正确回答 A 振荡更明显、较大 Kd 制动更强；对经过目标但速度仍大的情况，
回答不能认为稳定，可能仍有残余振荡。助手补充速度仍大本身已说明尚未静止。
S3.4 已勾选，Stage 3 四项学习练习完成；不推断本人独立实现采样绘图或完成 GUI 验证。
下一小步为 S4.1 世界/关节/末端坐标系与简单刚体变换，尚未开始，不自动实现后续算法。
本轮仅同步五份学习文档，链接与 git diff --check 通过；未运行 Python/GUI 或新增依赖。
详见 [PD 笔记](04_pd_control.md) 与[示例说明](../examples/03_pd_control/README.md)。
保留已有修改，同步两份 README、路线、笔记和交接；未新增依赖。
以下为此前学习过程记录，待答状态以本段为准。

2026-09-27 当前请求已进入 S2.1：只理解 shoulder_pan 的目标角度单位、gear 和范围。
助手静态核对本地缓存 XML 与官方参数文档；ctrl 单位 rad，gear 默认第一项为 1，
ctrlrange 为 ±6.2831 rad。详细来源与回答见 [03 Joint Control](03_joint_control.md)。
本人正确回答 rad、赋值不立即改变实际角度、7 rad 超出范围；起初不理解 gear，
解释 l=gq 后正确计算 gear=2、目标 0.4 rad 时 ctrl=0.8。S2.1 已勾选。
用户随后选择进入 S2.2：home 理解与预测已通过。本人正确回答只设 qpos 而 ctrl=0
时目标仍为零；home 目标增加 0.05 rad 后 ctrl=-1.5208，step 前实际角度=-1.5708 rad。
本人随后提供 2 秒运行输出：shoulder_pan 目标保持 -1.5208 rad，实际角度从
-1.5708 到 -1.52080194 rad，最终 qvel=9.90050996e-6 rad/s。
助手按打印精度计算末态误差约 1.94e-6 rad；其他关节角度也有变化。
详见 03 笔记的 S2.2。本人能用误差小、角速度近零判断末态；此前误将
“只改一个目标，只有对应关节会运动”判为成立，现已纠正，原话：
“不能，受重力和伺服影响，qpos仍然会有微小变动”。助手补充一般情况下变化不保证微小。
S2.2 已勾选。本次未设具体误差容忍阈值。
用户已选择进入 S2.3：讲解在目标修改后采集 t=0，再逐步记录 time/ctrl/qpos，
拟在现有 main.py 增加最小采样绘图。本人正确预测目标为水平线；初次漏计初始点，
解释后正确回答 3 步有 4 个样本，时间为 0、0.002、0.004、0.006 s，采样理解已通过。
用户明确授权非 mujoco 环境（包括 base）可自动激活，已修改 AGENTS.md。
已激活并检查 mujoco，Python 属于该环境；main.py 新增 headless --plot，保存 PNG/CSV。
1,000 步运行退出 0，1,001 样本覆盖 0～2 s；CSV 形状、时间间隔、固定目标与初值检查通过，
已目视检查图。详见 03 笔记的 S2.3 验证；未验证 GUI、未新增依赖。
本人正确解释后段变平表示角速度趋近零，但认为橙线越过目标。
助手在激活并核验 mujoco 后复查已有 CSV：1,001 点中 actual > target 的点数为 0，
max(actual-target)=-1.9371748432e-6 rad，采样数据未显示超调。未重跑仿真或 GUI。
进一步解释超调及负数比较后，本人正确判断实际 -1.53 rad 尚未到目标 -1.5208 rad。
采样与读图理解练习完成，S2.3 已勾选；图与 CSV 由助手生成，本人完成观察与问答，
不宣称本人独立运行或编写绘图代码。本轮仅同步文档，链接与 diff 格式检查通过，未重跑。
用户已选择 S2.4：拟各自从 home 出发，对比 shoulder_pan 的 +0.02/+0.05 rad，
均运行 1,000 步，比较目标、末态误差及其他关节变化；不实现控制器。
本人正确给出 -1.5508/-1.5208 rad 并判断均在范围内。已为 main.py 增加 --delta，
输出末态误差及逐关节角度变化；越界目标在步进前拒绝，不自动裁剪或改方向。
激活并核验 mujoco 后，两组 headless 1,000 步均退出 0；PNG/CSV 和数据检查通过。
末态误差分别 7.837648406e-7 / 1.937174843e-6 rad；shoulder_lift 两组均变化约 0.008079 rad。
--delta nan/10 拒绝检查通过（退出 2），未测试 GUI。完整命令与数据见 03 笔记。
本人正确回答 +0.02 组末态绝对误差更小、其他关节实际角度会变化、范围内不保证零误差。
助手补充：本次以 shoulder_lift 首尾差约 0.008079 rad 为依据，未记录该关节逐步数据，
不据此断言每一步都变化。S2.4 已勾选，Stage 2 四项学习练习完成。
用户已选择 S3.1：开始解释 e_q=q_target-q、e_v=v_target-v、tau=Kp*e_q+Kd*e_v，
单位分别 rad、rad/s、N·m。待本人手算 q_target=0.5、q=0.3、v_target=0、v=0.1、Kp=10、Kd=2。
下一小步是读取手算结果并解释 D 项符号；S3.1 未勾选，未实现控制器或运行仿真。
概念、边界与题目见 04_pd_control.md；当前 UR5e 的 ctrl 仍是角度目标，不能直接填力矩。
本轮仅记录回答并同步五份文档，保留已有修改，链接与 git diff --check 通过；未重跑仿真或 GUI。
本轮重新检查环境仍为 base、Python 为 /home/lucas/miniconda3/bin/python，未执行 Python。
本轮仅记录用户输出、同步文档并检查链接与 git diff --check，没有重跑或验证 GUI。
现有 main.py 已有 home 初始化和单目标实验，本步没有新增代码。
本轮环境仍为 base，未运行 Python；保留已有五份文档修改，检查链接与 diff 格式。
本轮仅记录回答并同步五份学习文档；pwd/git status 确认并保留已有未提交修改，
相对链接目标与 git diff --check 检查通过，没有运行 Python 或新增运行证据。
本次检查 pwd/git status，保留原有未提交修改；WSL2 Ubuntu 24.04.5，终端为 base，
Python 路径 /home/lucas/miniconda3/bin/python，因此没有执行 Python、仿真或 GUI。
本次仅更新学习文档，检查相对链接与 git diff --check；不新增运行验证结论。
下文 Stage 1 描述为此前过程记录，其中“下一项建议 S2.1”已由本次请求推进。

用户已正式开始 Stage 1 Task 1，只学习基础 simulation pipeline。新增
`examples/02_ur5e_basics/simulation_pipeline.py`：显式加载官方 XML、创建 model/data、
默认打开 viewer、循环 step 并打印状态。不设置 ctrl，不选 home keyframe，保留原有代码。
讲解已先于实现完成；没有引入控制器或新依赖。本人已提供观察并通过核心概念问答。
原先把零 ctrl 理解为保持任意当前姿态，经问答已纠正，能回答保持 -0.2 rad 要设 -0.2 rad。
最终原话“这次实验没有给 ctrl 赋值，但 qpos 仍然变化，是重力、执行器作用等共同影响的结果”
已记录到 `01_mujoco_basics.md` 的 What I Learned。不要重复将理解状态写为未验收。
README 的整个 Task 1 暂不勾选：headless 通过，但 GUI 正常退出仍待确认。

本会话用户明确授权 Codex 在执行 shell 主动激活 mujoco；激活后必须再次检查环境和路径。
实测 Python 3.12.14、MuJoCo 3.14.0、Menagerie 2026.9.1；环境检查 PASS。
新脚本 headless 1,000 步退出 0：dt=0.002、time=2.000，ctrl 始终零，qpos/qvel 有变化。
GUI 1,000 步完成最终打印后退出 139，提示 `timeout: the monitored command dumped core`。
用户此前能看到 GUI 与本次退出异常分开记录；确切根因未知。命令与结果见
[本次实验笔记](01_mujoco_basics.md)。

S1.2 已完成：用户先正确预测 500/1,000 步分别为 1/2 秒，角度不一定翻倍，
再提供 qpos[1]/qvel[1] 的两组结果，正确用很小的角速度判断该关节接近静止。
数据和原话见 `01_mujoco_basics.md`；本次仅记录用户实验，没有助手重跑。
README S1.2 已勾选。S1.3 已开始：用户正确回答 nq 是数量、qvel 单位 rad/s，
六个转动关节与三个控制输入对应 nq=nv=6、nu=3。
最小 pipeline 已补充 jnt_qposadr/jnt_dofadr 映射和最终逐关节状态输出；
2026-09-27 激活并检查 mujoco 后 headless 1,000 步退出 0、time=2.000 s，未复测 GUI。
2026-09-27 用户已正确读取 elbow_joint：两种索引均为 2，qpos=6.66674796e-03 rad，
qvel=-4.20118884e-06 rad/s，并解释角度为正、正在减小。S1.3 已勾选完成。
S1.4 的 body/joint/geom 问答已通过：用户正确理解多个 geom 随 body 运动、
删除例子中的 visual geom 不会删除 joint。没有实际修改 XML。
site 基础问答已通过：用户正确回答不增加关节、局部位置不变、世界位置可能改变。
依据 wrist_3_link 内 attachment_site 的官方定义进行阅读预测，没有位置读取实验。
当前 actuator 小步已开始：只读官方 XML 的 shoulder_pan 执行器，
通过 joint="shoulder_pan_joint" 识别映射，区分运动自由度与驱动力来源。
解释该模型 general 的位置伺服语义，但不推导控制公式。用户已正确辨认执行器与关节，
并理解删除执行器不删除自由度，其他执行器的作用仍可能通过连接影响运动。
2026-09-27 本人正确列出全部六组 name → joint 映射，记录于 `02_ur5e_model.md`；
元素基础问答与映射练习均完成，README S1.4 已勾选。
S1.5 已开始：用户正确预测 forward 不推进时间、step 后时间为 0.002 s。
新增 `forward_vs_step.py`，直接将 shoulder_lift qpos 设为 -0.2 rad，打印旧缓存，
forward 刷新 site 世界位置，再 step 一次。2026-09-27 在已检查的 mujoco 环境运行退出 0。
forward 后 time=0、angle=-0.2；step 后 time=0.002、angle 约 -0.199676。
用户随后正确反馈：直接改 qpos 后 site 缓存未变，forward 后 site 位置改变但时间不变，
直接赋值不是物理运动。S1.5 已勾选；本次仅记录反馈，未复测 GUI，未引入控制算法或新依赖。
S1.1 已完成：本人预测 nq=nv=1、nu=0，辨认 hinge 约束；运行基础示例后提供
qpos 0→3.05127557 rad、qvel 0→-2.41849323 rad/s、time=2.000 s，
并正确用 qpos 变化说明杆绕铰链转动。本次仅记录反馈，未修改代码或重跑。
S1.1～S1.5 学习练习均已勾选；Task 1 的 GUI 正常退出问题独立保留，不宣称全部工程验证完成。
下一项建议 S2.1：读一个 UR5e 执行器的 ctrl 单位、gear 和范围；待用户选择，不自动推进。
本次笔记见 `02_ur5e_model.md`，不进入控制算法。
GUI 退出异常独立跟踪，不阻塞 headless 学习；完整 Task 勾选仍须正常运行证据。
此前框架和路线评估见 [索引](learning_roadmap.md)。

### 最新用户确认（2026-09-26）

用户确认此前无法看到 GUI 的问题已解决，现在能看到 GUI 画面。
画面显示不再是当前阻塞项，不要继续把 `WARN:COPY MODE` 当作未解决问题。
具体修复步骤未提供，不推断是哪项操作生效；视角/控制交互和本次退出状态未单独确认。
本次仅根据用户反馈更新文档，没有重新运行 GUI。

### 历史运行证据（2026-09-24）

当次排查中用户曾明确允许主动激活 `mujoco` 环境；这是一条历史记录，后续按当前用户指令和环境规则执行。
激活后确认 Python 为 `/home/lucas/miniconda3/envs/mujoco/bin/python`，版本
3.12.14，MuJoCo 3.13.0，Menagerie 2026.9.2；没有安装或升级依赖。
`python examples/02_ur5e_basics/main.py --headless` 完成 1,000 步，退出码 0，
模块缺失未复现。`timeout --kill-after=3s 30s python -u
examples/02_ur5e_basics/main.py --viewer --steps 2500` 完成 5.000 秒仿真，
退出码 0，此次未复现历史 GUI 退出崩溃；桌面画面及交互仍待用户确认。
另已启动 `--viewer --steps 150000` 供用户观察，其完成结果尚未验证。
用户随后确认只看到 `WARN:COPY MODE`，没有仿真画面，因此 GUI 显示仍未通过。
现场 WSLg 版本为 1.0.73.2；`/mnt/wslg/weston.log` 启动日志记录
`rdp_allocate_shared_memory: Failed to open "/mnt/shared_memory/{...}" with error: Input/output error`，
随后 `use_gfxredir = 0`、`enable_copy_warning_title = 1`。MuJoCo 窗口已登记到 WSLg，
长时仿真进程检查时仍存活且未输出新错误。证据指向 WSLg 共享内存/窗口传输问题，
但尚未验证恢复方法。建议保存工作后在 Windows PowerShell 执行 `wsl --shutdown`，
重开 WSL 后复测；若仍失败，再按微软文档更新 WSL。未代为关闭 WSL，以免中断其他工作。
当时建议重启 WSLg 后确认画面；此显示待办已由 9 月 26 日用户反馈关闭。
此前退出码 0 仅代表当次进程测试通过。

这是面向 Robotics Software Engineering、Embodied AI、Robot Learning 和机械臂操作的
个人学习与面试准备仓库。采用官方 MuJoCo API，CPU 优先，一次学习一个概念。
用户当前重点：先理解 → 再实现 → 再实验 → 再总结；GUI 画面已确认可见。

已完成基础搭建和 UR5e 加载/检查演示。没有实现自定义控制器、PD、IK、Cartesian
control、轨迹规划、RL 或任务环境。后续阶段仅有文档占位；不要自行推进高级算法。
代码完成不代表用户已经掌握对应知识。

## 进度与入口

| 内容 | 状态 | 入口 |
| --- | --- | --- |
| 仓库结构、环境规则、检查脚本、最小依赖 | 已完成 | [根 README](../README.md)、[环境检查](../scripts/check_env.sh) |
| 单铰链基础仿真 | 无界面验证通过 | [示例 01](../examples/01_basic_simulation/README.md) |
| 官方 UR5e 加载、关节/执行器检查、单目标微调 | 无界面验证通过 | [UR5e 示例](../examples/02_ur5e_basics/README.md)、[代码](../examples/02_ur5e_basics/main.py) |
| GUI 显示 | 9 月 26 日用户确认可见，显示问题已解决 | 上方最新用户确认 |
| GUI 交互与退出 | 9 月 24 日进程退出正常；最新交互与退出未单独确认 | 下文历史记录 |
| Joint control 及后续阶段 | 未实现 | [学习路线](learning_roadmap.md) |

学习顺序：`01_basic_simulation` → `02_ur5e_basics` → `02_joint_control`。
两个 `02_` 目录是有意保留的，不要仅为统一编号重命名。

## 环境与启动

此前实测环境（不是新会话的自动保证）：

- Ubuntu 24.04 / WSL2，Miniconda 环境名 `mujoco`，VS Code 连接 WSL。
- Python 路径：`/home/lucas/miniconda3/envs/mujoco/bin/python`。
- 9 月 24 日记录：Python 3.12.14、MuJoCo 3.13.0、Menagerie 2026.9.2。
  9 月 22 日历史记录为 MuJoCo 3.14.0 / Menagerie 2026.9.1；以重新检查为准。
- 代码兼容目标为 Python 3.11，但尚未在 3.11 运行验证；不要擅自替换现有环境。
- 依赖仅有 `mujoco`、`numpy`、`matplotlib`、`mujoco-menagerie`，版本未锁定。

用户在新终端的启动流程：

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
```

只有确认环境为 `mujoco` 且 Python 属于该环境后，才执行：

```bash
python --version
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
python examples/02_ur5e_basics/main.py --headless
```

2026-09-27 用户明确授权并更新 AGENTS.md：当前环境不是 mujoco（包括 base 或未激活）时，
Codex 可自行激活已有 mujoco 环境，无需再次确认。在执行所用的同一 shell 中重新检查
环境名和 Python 归属，通过后继续；仅激活或检查失败时停止报告。不要在 base/system Python 安装或运行。
此前已安装 Menagerie；新会话先检查，不要每次重复安装或升级依赖。

## 已验证的模型经验

- 基础示例：一个被动铰链，没有执行器；`nq=nv=1`，1,000 步推进到 2.000 秒。
- UR5e 使用 `mujoco_menagerie.load("universal_robots_ur5e")` 返回 `MjModel`，
  然后创建 `mujoco.MjData(model)`。默认加载包含地面的 scene。
- 包名是 `mujoco-menagerie`，Python 导入名是 `mujoco_menagerie`。
- 首次加载联网下载该模型，默认缓存于 `~/.cache/mujoco_menagerie`；
  不需要复制整个 Menagerie 仓库或把网格资产提交到本项目。
- 实测 UR5e：`nq=6`、`nv=6`、`nu=6`，6 个关节、6 个执行器、8 个 body（含 world）。
- 关节名称为 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、
  `wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`；对应执行器名称去掉 `_joint`。
- `qpos` 是实际广义位置，`qvel` 是实际广义速度；`ctrl` 的含义取决于执行器配置，
  不能对任意模型都理解为目标角度或力矩。
- 此 UR5e 用 `general` 执行器实现位置伺服。代码检查 transmission、gain/bias、
  dynamics、gear 和范围后才把命令作为目标角度，不能仅凭名称判断。
- 从 `home` keyframe 同时初始化姿态和控制值，避免只设置 `qpos` 而让目标仍为零。
- 用 `actuator_trnid` 查执行器对应关节，再通过 `jnt_qposadr` 找位置地址；
  不要假设 actuator ID 就是 qpos 索引。
- 只将 `shoulder_pan` 目标由 -1.5708 改为 -1.5208 rad，并检查关节/控制范围。
  1,000 步后时间为 2.000 秒、状态有限，实际角度变化约 0.049998 rad。
  其余控制值未改变，但其他关节仍可能因重力和动力学耦合而运动。

以上来自此前真实运行；详细版本、模型 revision 和输出摘要见 UR5e 示例 README。

## GUI 操作经验与限制

在确认环境后运行以下命令可延长窗口操作时间：

```bash
python examples/02_ur5e_basics/main.py --viewer --steps 150000
```

当前 timestep 下约为五分钟仿真时间，实际墙钟时间可能更长。默认 1,000 步约两秒
就结束；自动关窗不一定是模型加载失败。9 月 24 日已启动长时命令，但完整运行结果
未验证，当时用户只看到 `WARN:COPY MODE`。9 月 26 日用户确认画面已可见；
具体交互效果仍未单独确认。

| 操作 | 方法 |
| --- | --- |
| 旋转 / 平移 / 缩放视角 | 左键拖动 / 右键拖动 / 滚轮 |
| 显示或隐藏左 / 右面板 | `Tab` / `Shift+Tab` |
| 查看快捷键 | `F1` |
| 调整执行器目标 | 展开右侧 `Control`，小幅修改一个目标 |

建议先把 `shoulder_pan` 从约 -1.52 调到 -1.47 rad，观察对应关节。
这是操作建议，不是已验证的用户交互结果。GUI 控制修改由 `viewer.sync()` 同步；
当前循环不会每步覆盖目标。手动操作后不再满足“只改变一个命令”的自动演示条件。

当前采用 `launch_passive`，仿真由 Python 的 `mj_step` 循环推进。
**脚本没有暂停回调，按空格不会暂停它**；不要把 managed viewer 的暂停行为套用过来。
拖动刚体施加扰动也不等于实现末端目标控制或 IK。

操作依据：[官方 viewer 文档](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)、
[官方快捷键](https://mujoco.readthedocs.io/en/stable/programming/samples.html#shortcuts)。

## GUI 历史问题（保持与 headless 结果分开）

以下为 9 月 22 日历史错误；9 月 24 日未复现退出崩溃，但出现画面传输问题，见上文。

1. 基础示例：仿真到达 2.000 秒后出现 `GLXBadDrawable` / `X_GLXSwapBuffers`，
   随后测试超时，退出码 124。
2. UR5e 示例：完成全部仿真和最终状态打印，但退出时发生以下错误，退出码 134：

```text
Fatal glibc error: pthread_mutex_lock.c:450 (__pthread_mutex_lock_full): assertion failed: e != ESRCH || !robust
timeout: the monitored command dumped core
```

此前诊断发现 `DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`、WSLg 1.0.66，
没有 `glxinfo`。第一类错误疑似 GLX/WSLg 或关闭路径问题；第二类表现在 native
GUI/线程退出路径。历史错误的确切原因未确认；9 月 24 日未复现退出崩溃，
9 月 26 日用户确认显示问题已解决。保留这些记录供复发时参考，不作为当前显示故障。
没有安装额外图形依赖或设置渲染后端 workaround。

若用户要求诊断 GUI，先保留准确报错、退出码、是否打印最终状态，分别复测 headless
和 GUI。不要因关闭时报错就回退已通过验证的模型加载，也不要仅凭异常名称断言根因。
历史测试命令和完整记录见 [MuJoCo notes](mujoco_notes.md) 与 UR5e 示例 README。

## 下次如何继续

1. 读 `AGENTS.md` → 本文 → 与当前请求有关的示例 README/代码。
2. 检查 `git status --short`；9 月 26 日本次编辑前工作树干净。新会话必须现场核对，保留已有修改。
3. 按用户当次目标推进；核心问答、S1.1～S1.5 已完成，下一项可选 S2.1 的执行器参数阅读；不要重复旧验收或自动实现下一任务。
4. 用户若要求改善交互，可在这个示例中增量增加暂停/持续运行功能；这些目前尚未实现。
5. 完成工作后更新本文的状态和实测结果；同步相关 README，避免把建议写成已完成事项。

本次新增最小示例并完成 headless/GUI 分开验证；详见顶部当前任务记录。
