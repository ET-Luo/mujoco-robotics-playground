# S20.2 — Action chunking baseline / replan

[状态唯一来源](../README.md#stage-20--act--temporal-policy) · [代码](../examples/21_act_temporal_policy/chunk_baseline.py)
前置：S20.1的window/mask与S19低层controller；0.5～2小时学习规模。

## Problem → Why → Intuition

一次预测H步，机器人究竟执行多少步才重新看环境？逐tick预测可能不断把动作序列的第一帧拿来执行；
执行较长前缀则沿着原先计划推进，却更晚利用新状态。
本课把**预测长度H**和**执行长度E**分开：固定一个checkpoint比较E=1、5、H，再用H20做Modify。
像导航先规划一段路线，走E步后根据实际位置重新规划；重新规划不等于凭空消除模型误差。

## Core Concepts / Mathematics

输入o_t∈R^6=(x,y,vx,vy,gx,gy)，world frame；位置/goal [m]、速度 [m/s]。
动作a∈R^2是world XY绝对位置reference [m]，不是力、ctrl或下一帧qpos。

```text
z_t = (o_t - train_mean_o) / train_scale_o
MLP(z_t) -> normalized predictions (H,2)
A_t = predictions * train_scale_a + train_mean_a
execute: a_(t+k) = A_t[k], k=0,…,E−1; 1≤E≤H
next anchor: t+E, using actual o_(t+E)
```

末段实际只执行min(E,T−t)步，不越过固定T100/2s。
policy reference每.02s更新一次；预测调用频率为50/E Hz（最后不足E步除外）。
E1/5/10/20对应名义50/10/5/2.5Hz，100tick需要100/20/10/5次调用。
**E步内是高层计划的开环前缀，低层阻抗反馈始终闭环运行：**

```text
F = clip(K*(reference-qpos) - D*qvel, -20N, 20N)
K=400 N/m; D=2*sqrt(M*K) N*s/m; M=[2,1] kg
ctrl = requested_force / gear
```

复用S19 `env.advance`，每个reference保持20个1ms physics tick，反馈每tick重新计算。
只有reset写qpos/qvel，执行状态由mj_step更新。

训练：S20.1 `make_windows(rows, history=1, chunk=H)`；当前观测输入，不增加历史输入变量。
MLP为6→32→32→2H，Tanh隐藏层，CPU单线程；H10有1940参数，H20有2600参数。
loss是S20.1的有效动作标量MSE，Torch通过boolean选择保留有效位置梯度；padding梯度为零。
Adam lr=.001、batch64、200epochs、seed202；epoch0也允许作为候选，按validation masked MSE选择。
每个batch按有效标量求平均；整组offline分数按全部有效标量重新计算。
不同H改变目标长度、重叠标签权重和输出参数量，所以H10/H20不是只改变执行频率的纯消融。
同一H内E比较才固定weights、normalizer、reset和所有训练细节。

## Math-to-Code / APIs

`make_policy(H)`返回torch.nn.Sequential，不是ACT：没有Transformer、CVAE、latent或KL。
`policy(x)`返回(B,2H)，`reshape`成为(B,H,2)。`loss(pred,target,valid)`返回可反传标量Tensor；
`backward()`写入参数梯度，`optimizer.step()`原地更新参数。训练只使用train，validation选checkpoint，test数组不打开。
`torch.inference_mode()`停用梯度记录；`predict`返回反归一化(H,2)的NumPy数组。

`MjModel.from_xml_string(XML)`编译旧XY模型并返回MjModel；`MjData(model)`返回独立可变状态。
`mj_forward(model,data)`原地刷新派生状态，不推进time；`mj_step`原地积分状态/time，无返回新状态。
`rollout`每次获取真实o_anchor，保存整段预测，再依次执行plan[k]；E结束丢弃未执行尾部并replan。
不重复plan[0]，不把同一个plan从头循环播放，也不从expert未来标签读取动作。
checkpoint包含weights、normalizer与split hash；保存后weights_only/CPU重载并验证预测完全一致。

## Minimal Experiment / Expected Result

依赖requirements已有NumPy、torch==2.14.0+cpu、MuJoCo、Matplotlib；
local helper为sequence_dataset与behavior_cloning→episode_contract→cartesian_spring。无新增包。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/chunk_baseline.py --prepared tmp/s19_3/verified193
```

prepared为S19.3输出；缺失时按[Stage19 README](../examples/20_learning_from_demonstrations/README.md)的S19.2→3重建并替换路径，
不需要旧BC checkpoint，也不需要先保存S20.1的窗口产物。
输出新ignored tmp/s20_2/run_*：protocol.json先写；policy.pt、loss.csv、offline prediction NPZ、
所有validation trial NPZ、results.json、experiment.png。显式--output必须新目录。
每组20trial：4reset×(3种E+expert+constant)，expert每policy tick重新观测，constant始终使用train平均reference。

先预测：固定H，E增加会减少预测调用、增加高层反馈等待；offline指标必须完全相同，因为执行方式不改变checkpoint。
成功率不保证单调；需闭环实验回答。成功判据沿用S19末10观测全部位置<1mm且速度<2mm/s。

## Actual Result / Explanation

2026-10-11 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；
mujoco环境Python3.12.14、NumPy2.5.3/MuJoCo3.14.0/Matplotlib3.11.2/Torch2.14.0+cpu真实模块与API核验；CUDA build None，无安装。
默认命令加`--output tmp/s20_2/default`；同输入加`--chunk 20 --output tmp/s20_2/chunk20`，均exit0。

| H | selected epoch | validation chunk RMSE / first-action RMSE | train wall（循环） |
| --- | --- | --- | --- |
| 10 | 136 | .762 / .781 mm | 2.485s |
| 20 | 172 | .543 / .691 mm | 2.587s |

训练wall包含每epoch整组train/validation forward、selection；不包含window构造、checkpoint存储或rollout。
没有单次推理延迟benchmark，本课prediction call数量不是速度测量。

| H | E | calls/trial | success / attempts | 全分母final mean error |
| --- | --- | --- | --- | --- |
| 10 | 1 | 100 | 1/4 | 3.585mm |
| 10 | 5 | 20 | 1/4 | 1.845mm |
| 10 | 10 | 10 | 2/4 | .873mm |
| 20 | 1 | 100 | 1/4 | 2.036mm |
| 20 | 5 | 20 | 2/4 | .867mm |
| 20 | 20 | 5 | 4/4 | .357mm |

两组expert4/4、constant0/4（终态均误差10.895mm）。不能凭终态均值代替末10帧/速度成功判据。
PNG左侧loss，右侧仅第一个validation reset的goal error，不是全部trial统计。

解释：同一个模型的E1总是执行索引0，较长E会使用后续索引。各索引输出由不同最后一层权重产生，
可能具有不同偏差；频繁重新预测也会改变访问状态分布。因此“更频繁观测必然更成功”不成立。
本次较长E表现更好与该机制相容，但未单独分离逐offset偏差、状态分布变化等根因，不能声称已证明因果归因。
预测性能来自有限4reset/单training seed/名义固定动力学；未施加扰动或做新的test泛化评价。
H20/E20 4/4不是通用稳定性保证，也不与S19旧BC构成完全匹配架构的优劣结论。

## Validation / Failure Cases / Robotics Context

40保存trial独立重算80000个Euler physics步，来源reset/clock/scorer/hash通过；
chunk anchor/offset、实际状态输入重预测、执行动作与plan[k]逐项核对通过。
同H E组首预测一致，E5/EH前5动作一致；checkpoint/normalizer不变、CSV argmin与offline masked分数核对通过。
Torch masked loss有效值/梯度与padding梯度零通过；T13/E5最后只执行3步通过。
五种CLI非法/缺输入/已有目录均exit2；损坏prepared hash拒绝；3.11仅语法核验，实际runtime3.12；两PNG已查看。
未做GUI、OOD、扰动恢复、完整ACT或真实硬件实验。

- 混淆H/E会把输出结构变化误当执行频率变化。
- 重复plan[0]等于保持单reference，不等于执行chunk。
- 使用预测状态而非actual观测replan会脱离机器人真实反馈。
- 在前缀执行过程中低层反馈仍必需；high-level开环不等于停掉阻尼/力限幅。
- 路径突变、外扰或接触变化可能要求更快replan；本课名义结果不能代替该验证。
- 改normalizer或用rollout选本课checkpoint会改变协议；test不参与。

应用：示范学习的短期动作计划、机器人高层策略与快速反馈controller分频；后续ACT也需要同样的执行时间契约。

## Interview Capsule / Must Remember

**30秒：**小MLP由当前状态预测H步动作，执行前E步后用真实状态重新规划；H控制预测范围、E控制反馈间隔。
训练排除padding，动作仍交给快速低层controller；offline loss与闭环成功分别评价。

**2分钟：**解释6→2H、训练/推理数据隔离、normalized→米单位reference→force/gear→mj_step。
固定checkpoint比较E才能隔离执行方式；较小E更快利用新状态，但可能反复使用有偏第一动作，不能保证成功率提升。
H变化同时改变模型与监督，需报告边界；末尾截断前缀并保存anchor/offset验证时间对齐。
MLP chunk是deterministic baseline，尚非ACT。无需把后续Transformer概念提前搬入本课。

## My Verification / Run → Modify → Explain

README Engineering状态与Learning状态分开；2026-10-11 本人确认实验与预测完成，并正确回答五项Explain；Run/Modify/Explain全部完成，Learning Mastered。
精度补充：本课低层是XY slide的阻抗力反馈；E1始终执行chunk首动作。动作跳变/抖动是一般可能机制，本轮没有单独验证其根因。
本次仅同步文档，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-11工程验证。

**Run：**运行默认命令，核对三种E的prediction_calls、offline指标、success与末帧均误差；查看experiment.png。

**Modify：**先预测，再增加`--chunk 20`运行：输出维度/参数量/调用次数怎样变化？
比较两组内E1与EH；区分“同weights改E”与“H变更后重训”。不要假定更长H或更小E必然更好。

**Explain（五问）：**

1. H10/E5时第5个执行tick（t=4）用哪个anchor、哪个offset？t=5为何需要新观测？
2. E10的高层前缀开环时，低层反馈为何仍是闭环？ctrl、reference、qpos分别是什么？
3. 同一checkpoint改变E，为何offline loss不变而rollout可能改变？
4. 为何更频繁replan未必更好？本次结果能否证明长前缀普遍更鲁棒？
5. H10改H20为什么需要重训？它与固定H只改E的比较有哪些变量差异？

本轮止S20.2；S20.3 Necessary Transformer concepts等待明确请求。
