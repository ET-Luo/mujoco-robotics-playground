# P3 — Learning-Based Robot Manipulation

核心问题：**How can a robot learn manipulation behavior from demonstrations?**
2026-10-10：用户确认 P0/P1/P2 完成；本轮静态审查已有代码与状态，未重跑历史阶段。
状态唯一来源：[README](../README.md#p3--learning-based-robot-manipulation)。

## P2 审查与复用

| 已有能力 | 可复用入口 | 限制 / P3 所需改动 |
| --- | --- | --- |
| MuJoCo/UR5e、6D FK/IK/J、trajectory | examples/09–11；12_pick_place/pregrasp_motion.py 的 build_model/solve_pregrasp_ik/cubic_reference | 不重教；private IK state 与实际 simulation state 保持分离 |
| known-pose manipulation | 12_pick_place/transfer_and_descend.py 的 run_reference/run_to_support；release_and_retreat.py | 直接复用模型与接触判据；循环需增加逐 policy tick 的记录，旧汇总日志不能直接当训练集 |
| perception/planning/ROS2 | 14_vision_manipulation/vision_pick_place.py；15_motion_planning；16_ros2_integration | P1 固定场景工程已完成；ROS pipeline 未集成障碍RRT。P3先 state-based，不再搭 ROS 或视觉encoder |
| torque/wrench/dynamics | 17_force_dynamics 的 gravity_compensation、wrench_frames、contact_wrench、jacobian_transpose | 复用单位、frame、actuator cap与诊断；不假设真实力传感器或硬件控制能力 |
| force/impedance/admittance/hybrid | 18_compliant_control/cartesian_spring.py XML；contact_transition.build_model/measure；hybrid_control.run；surface_following.run；ur5e_surface_following | XY是小实验主线；UR5e扫描可作扩展expert；run函数目前不是任意policy step接口 |
| teleoperation | 19_teleoperation_dexterous/scaling_clutch.map_sample；teleoperation_integration.run | mapper/clutch/lease可复用；已有输入为scripted master，无真人键盘/设备示范；旧日志有32列且混合1kHz/50Hz，需显式重采样与action提取 |
| tactile/contact sensing | contact_sensing.contact_measure/read_touch；multi_contact_grasp.object_contacts | touch是区域标量，contact是仿真pair力，不是高分辨率触觉图像；normal gate不可等同稳定抓取 |
| multi-contact/friction/closure/stability | hand_fixture.mapping；multi_contact_grasp.build_model；friction_cone；force_closure；grasp_stability.run_trial | free cylinder/有限窗口，force closure为理想平面证书；低摩擦/缺指可能未ready，所有attempt保留分母 |
| PPO/SAC基础 | Stage8记录与 environments/rl | 用户已有概念；不重复RL、不把BC改为reward优化 |

结论：P2工程与学习已按用户记录完成；README首页和roadmap简介有过时措辞，本轮修正。
接触、ROS、GUI、真实硬件能力不因历史headless通过而扩大。旧controller可以作expert，
但dataset recorder、统一reset/step/evaluator是P3新工作，不能宣称旧CSV天然就是可训练数据。

## 顺序与资源

Demonstrations → episode dataset → split/normalization → BC → closed-loop failure →
recovery → action chunking → ACT → diffusion → contact-aware integration。
每个表中Task均0.5～2小时学习规模，超预算继续拆分；只实现当前获授权Task。
Ryzen 7 7840HS / WSL2 Ubuntu24.04：CPU PyTorch，首选2层32–64宽MLP、
1–2层/2–4 heads/64宽Transformer、几十条短episode、batch16–64、1–4 CPU threads。
这些是待计时的起始预算，不是收敛保证；训练时间/采样延迟必须实测。
无大型视觉encoder、image-based ACT、大型Diffusion、VLA或World Model。
S19.1仅需已有NumPy/MuJoCo/Matplotlib helper；Torch从BC任务起核验CPU版本，不提前安装。

## 共同 benchmark 合同（规划，尚未实现）

A：xy_goal_reach_v1作为契约/BC起点；B：同XY fixture上下绕障碍双路径任务，供BC/ACT/Diffusion统一比较；
C：contact surface following，最后选一个小三指保持/扰动任务。它们是不同benchmark轨道，不能把跨task数字排优劣。
每次新增轨道，三类policy都必须在该轨道重训/重测；固定model/schema版本、obs、action语义、
50Hz policy/1kHz inner loop、reset分布、episode IDs/split、normalization、horizon、成功阈值与evaluation seeds。
保留独立test episode/reset seeds；val仅用于选择模型，test不调参。OOD friction/mass/noise/latency单列。
训练数据只用train拟合normalizer；sequence/window不跨episode，padding mask不能计入loss。
政策可有不同历史/预测horizon，作为明确的实验变量报告；相同可观测通道与action执行边界。
报告多training seeds、全部attempt success/timeout/failure、位置误差、速度/平滑度、接触力峰值/滑移/掉落、
wall train time、推理均值/p95与simulation time。error同时报告全分母及成功条件统计。
Offline validation loss独立报告，不代替closed-loop MuJoCo rollout；专家baseline、BC、ACT、Diffusion都走同一scorer。
锁存全过程失败，末帧恢复不能洗掉失败；恢复数据按来源episode分组，防止train/val泄漏。

## 四阶段与小任务

详见 [Stage19](22_learning_from_demonstrations.md)、[Stage20](23_act_temporal_policy.md)、
[Stage21](24_diffusion_policy.md)、[Stage22](25_contact_aware_robot_learning.md)。
ACT的action chunking/temporal ensembling及生成式sequence结构依据[原论文](https://arxiv.org/abs/2304.13705)。
教学tiny state ACT保留CVAE posterior/prior、KL项与Transformer chunk decoder；去掉视觉分支。
只做deterministic chunk Transformer时明确命名baseline，不能冒充ACT。
Diffusion依据[原论文](https://arxiv.org/abs/2303.04137)：条件去噪动作序列，逐轮执行短前缀并重新观测。
本仓库拟用小MLP denoiser作为CPU教学实现，不承诺复现原论文性能。

## Robot Learning 的五个关键差异

Offline loss衡量expert分布上的预测；rollout衡量policy诱导状态分布上的任务结果。
误差改变下一状态，造成covariate shift并沿长horizon累积；低MSE仍可能失败。
时间相邻样本相关，随机按row split会泄漏；动作chunk需尊重episode边界。
同一observation可能允许多个绕行模式，平方回归均值可能落入障碍；条件生成也需检验模式覆盖和连贯性。
Generalization必须区分held-out同分布与未见物理参数/噪声/延迟；有限seed通过不证明通用操作能力。

本轮截止S19.1。下一个小任务S19.2等待用户请求。
