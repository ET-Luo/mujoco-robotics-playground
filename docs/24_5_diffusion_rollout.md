# S21.5 — Tiny state Diffusion Policy / closed-loop rollout

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [代码](../examples/22_diffusion_policy/diffusion_rollout.py)
前置S21.4；0.5～2小时。主概念：采样完整chunk，执行短前缀，再用actual state重新规划。
Engineering Complete表示接口、实验与验证完成，不表示策略成功或Learning Mastered。

## Problem → Why → Intuition

S21.4在expert状态生成局部chunk，尚未承受自身动作改变状态后的反馈。闭环中必须从实际q/qvel再采样，而不能继续读expert观测。
直觉：预测10个reference并不意味着必须执行10个；只执行E个后重新看机器人，舍弃旧计划剩余槽。
条件生成加receding horizon的应用依据：[Diffusion Policy论文](https://arxiv.org/abs/2303.04137)。这里仍是CPU H10 MLP教学版本，无视觉输入。

## Core Concepts / shapes / units / frames

| 量 | 含义 |
| --- | --- |
| actual o_t | (6,)，world XY位置[m]、速度[m/s]、goal[m] |
| normalized o | (1,6)，train-only normalizer；不是expert状态 |
| sampled chunk | (1,H10,2)，K100步DDPM输出的归一化reference |
| decoded plan | (10,2)，world绝对XY reference[m]，不是ctrl/下一qpos |
| E | 执行前缀长度，默认5/Modify1；50Hz下重规划间隔.1/.02s |
| mask | (1,10)，h<150-anchor，固定时间预算决定，不读未来success |
| physical state | qpos/qvel实际积分；只在reset直接初始化 |

固定S21.1 B轨道paired split：2validation reset21104/21108；test只保留seed、不读取或执行。
冻结S21.4 H10/K100 checkpoint，离线validation loss已选模；本节不使用rollout调参/挑seed/重选checkpoint。
默认没有--checkpoint时用当前环境解释器准备同一S21.4训练包；显式checkpoint路径可跳过准备。
每reset预定义单个采样stream，seed=sample_seed+reset_seed×1000+anchor；默认sample_seed215。
不同E的共同anchor使用同seed，但actual observation一般不同，不要求共同anchor产生相同计划。

## Mathematics / control contract

```text
anchor = floor(t/E)*E
plan = decode(DDPM(o_actual[anchor], mask_from_remaining_budget, seed(anchor)))
a_t = plan[t-anchor]                     # 前缀，其余舍弃
F_requested = 400*(a_t-q) - D*qvel      # N，D=2*sqrt(mass*400)
ctrl = F_requested / gear
```

reference每.02s更新、期间保持；反馈每.001s重算，MuJoCo motor实际力限幅±20N。
mass=[2,1]kg、gear=[2,1]；总150reference tick/3000physics step/3simulation秒。
actuator力有限不代表位置有界：本fixture slide关节无workspace限位，持续错误reference仍能产生很大运动。
没有新增workspace投影、吸附goal、reference裁剪或mode筛选，否则会改变冻结策略的实验问题。

成功=每个actual1kHz运动segment都不进入半径10mm圆盘 AND 最后10观测每个位置误差<1mm且速度<2mm/s。
圆盘仅几何禁入区，没有contact反力；碰撞锁存，末态到达不洗掉失败。为观察完整失败，碰撞后仍继续积分固定时长。
无碰撞并不代表到达目标、满足workspace或硬件安全；本评分没有workspace约束，本节如实记录发散。

## Math-to-Code / APIs

`load_checkpoint`核验task/H/K/架构、窗口数据hash、paired split、normalizer与noise schedule；返回eval模型、保存参数、normalizer与groups。
只加载train/validation的窗口审计文件，不读test。checkpoint保存的旧data bytes与协议一致不等于跨环境重新采集必逐值一致。
`DiffusionPlanner.infer(normalized_actual_o)`返回(1,10,2)归一化计划；由调用次数×E得到anchor，构造剩余预算mask和采样seed。
复用task.rollout的`kind='act'`仅是其chunk接口分支，模型实际是DDPM，不调用ACT网络。
`seq.sample`完整100步、相同observed state条件；不读expert route/phase/未来状态或真实epsilon。
返回后task.rollout按train normalizer变回米、执行前缀；invalid尾槽NaN不会执行，保存mask供审计。

`MjModel.from_xml_string(XML)`编译返回模型；`MjData(model)`创建独立可变状态。
`mj_forward(model,data)`原地刷新派生状态，不推进时间；`mj_step`原地推进qpos/qvel/time，不返回新的机器人状态。
rollout每tick读取env.observe中的实际qpos/qvel与固定goal；没有执行阶段赋值qpos。
每次采样同步阻塞：采样wall time内simulation暂停，不模拟真实机器人在等待期间继续运动。

依赖已有NumPy/CPU Torch/MuJoCo/Matplotlib，无新安装；helpers sequence_diffusion→conditional_denoiser/noising/windows与task→episode_contract→cartesian_spring。
TorchCPU1thread；`subprocess.run`用sys.executable准备S21.4，不切到system Python。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/diffusion_rollout.py
```

新ignored tmp/s21_5/run_*，默认training/内准备S21.4包；顶层protocol/results/trials.csv、每reset expert_upper/expert_lower/diffusion轨迹NPZ与rollout.png。
无需历史tmp；显式--output目录必须尚不存在。
预期接口finite、计划与actual观测对应、专家成功；Diffusion胜负以实测为准。

## Actual Result / Explanation

2026-10-12 Zero；WSL2 Ubuntu24.04.5/kernel6.18.33.2，mujoco内Python3.12.14；NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.13.0/Matplotlib3.11.2 metadata/真实paths/API通过，无安装。
默认命令加`--output tmp/s21_5/default`，随后Modify命令见下，两组exit0。
也验证`--checkpoint tmp/s21_4/final/chunk.pt --output tmp/s21_5/checked`，E5轨迹与default逐值一致（不含wall latency）。
默认准备的weights与S21.4逐值一致，selected epoch146、offline audit .089860；本节未根据失败重新训练或挑模。

| 模式 | success | collision | tail到达 | 两reset final error [mm] | 最小actual净空 [mm] |
| --- | --- | --- | --- | --- | --- |
| expert upper/lower（各自） | 2/2 | 0/2 | 2/2 | 均<.001 | 15.268 |
| Diffusion E5 | 0/2 | 0/2 | 0/2 | 4.797 / 32879.495 | 13.617 |
| Diffusion E1 | 0/2 | 0/2 | 0/2 | 76499.156 / 56791.234 | .850 |

E5一次绕行后未稳定到达，另一次远离目标发散；E1两次发散。发散是finite的错误动作，不是import异常或NaN。
E5 reset21108在tick31首次位置norm>100mm，max绝对reference64.681m；E1两reset tick22/41越过100mm，max reference83.214/61.489m。
这说明低offline loss没有阻止闭环误差累积；状态离开专家分布与无界外推可能参与发散，但没有因果干预定位，不能声称唯一原因。
E1反馈更频繁仍失败，不构成“短前缀总更好/更差”的普遍结论。

E5/E1共60/300次采样，每trial30/150次；batch1 sampler latency median8.869/7.962ms、p9517.067/14.610ms、max22.754/23.888ms。
仅包括DDPM及内部trace分配，不含normalization/decode/physics；对照simulation重规划间隔100/20ms不是硬件实时保证。
E1本轮有最大延迟超过20ms，且仿真暂停等待，不能声明完整20ms控制周期已满足。

两组12trial独立重算36000个Euler步、1kHz运动段几何/scorer；全部360个actual anchor重采样核对、mask/seed/计划槽/aggregate通过。
checkpoint/default准备逐值一致、invalid execute CLI/错误task checkpoint拒绝、Python3.11语法、links/状态/git diff --check通过（actual3.12）。
默认PNG已查看：发散subplot使用真实坐标尺度不裁掉失败。无test/GUI/OOD预定义benchmark/接触反力/硬件验证；单training seed、两reset、单采样stream。

## Failure Cases / Robotics Context

- 读expert状态重新规划：得到离线teacher forcing而非闭环。
- 错用旧chunk或槽索引：必须记录anchor/offset、只执行有效前缀。
- 从未来success构造mask：部署信息泄漏；这里只看固定预算。
- frequent replanning随机切换：可能打断序列一致性，但本轮没有专门mode-switch归因。
- 限力被误认为限位：无界slide可发散；没有workspace不代表所有无碰撞轨迹安全。
- 只报0collision：两次未到达或发散都必须计失败。
- 采样耗时小于sim间隔就称实时：完整延迟、等待期间物理行为和系统调度尚未验证。

机器人应用：学习策略负责reference计划，低层阻抗负责实际跟踪，receding horizon用新actual反馈修正计划；各层承担不同责任。
本节失败保留为后续S21.6的诊断输入，不自动添加救援控制器或扩展训练。

## Interview Capsule / Must Remember

30秒：冻结chunk Diffusion模型，从actual state采样10步reference，只执行E步再观测。50Hz reference下低层1kHz限力反馈，完整actual运动段检查安全与tail到达。离线loss通过不意味着闭环任务成功；本例E5/E1均0/2且有发散。

2分钟：画actual observe→normalize→100-step DDPM→decode→prefix→ctrl→mj_step反馈环；说明H/K/E/t和mask来源、seed合同、reference与力输入；解释固定offline checkpoint与真实闭环评分、碰撞锁存；报告失败和sampling latency范围，区分同步仿真与真实实时；最后提出需要后续诊断验证的假设，不直接宣称Diffusion优于ACT。

Must Remember：actual反馈；只执行前缀；预算mask；reference不是ctrl；无碰撞不等于success；Engineering Complete不等于策略可靠。

## My Verification / Run → Modify → Explain

Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），状态见根README。
解释覆盖H/K/E与规划次数、actual反馈与丢弃槽位、低层控制与限力边界、全过程安全与tail稳定、offline误差及端到端实时证据。
精度补充：本节robot tick指50Hz reference tick，内部physics/feedback为1kHz；ctrl=F_requested/gear后由actuator限力。策略E5/E1均0/2的实验结论保留。
Run：运行默认命令，观察两个实际轨迹与失败；检查results中的success/collision/tail与latency。
Modify：先预测E5→E1的重规划次数、反馈频率与可能行为，冻结Run生成的checkpoint（替换路径）：

```bash
python examples/22_diffusion_policy/diffusion_rollout.py \
  --checkpoint tmp/s21_5/default/training/chunk.pt --execute 1
```

现场验证命令额外用了`--output tmp/s21_5/execute1`。保持H/K/weights/sample_seed/reset不变，不能根据两reset挑E作为已证最佳。
Explain（五问）：
1. H、K、E、robot tick分别是什么？E5/E1每trial为何采样30/150次？
2. 为什么必须读取actual state？前缀执行后旧计划哪些槽被丢弃？
3. reference如何变为ctrl与实际运动？限力为何不能防止本例位置发散？
4. 0collision为什么仍失败？为什么检查实际运动段及tail位置/速度，而不是仅reference或末态？
5. 为什么低offline loss、更短前缀或本轮sampling latency都不能直接证明闭环可靠与实时满足？

下一小任务S21.6 multimodal/latency diagnosis等待明确请求；不自动实施。
