# S20.7 — BC / chunk baseline / ACT 统一评估

[状态唯一来源](../README.md#stage-20--act--temporal-policy) · [代码](../examples/21_act_temporal_policy/compare_policies.py)
前置S19.8与S20.1–6；0.5～2小时。主概念：用统一执行合同区分预测、任务结果与计算成本。

## Problem / Why / Intuition

S19与S20各自报告过BC、MLP chunk和ACT结果，不能只拼接历史数字就判断模型优劣。
本节把冻结模型放进同一task/reset/controller/scorer，成对比较；同时承认训练预算与模型容量不相同。
直觉：让不同驾驶员驾驶同一辆车、从同一位置出发，用同一个到达标准；考试公平不代表训练过程相同。

不训练、不调参；不新增障碍/视觉/接触。采用A轨道xy_goal_reach_v1，不冒充尚未实现的双路径B轨道。

## Core Concepts / Mathematics

观测o=(x,y,vx,vy,gx,gy)∈R^6，world frame，位置/goal[m]、速度[m/s]。
动作a∈R^2是world XY绝对位置reference[m]，不是力ctrl或直接写qpos。
BC预测单动作；chunk/ACT预测H10，执行E1或E5；ACT uniform每tick预测并平均对齐候选。
所有模式只读当前真实观测，history1。ACT部署prior z=0，posterior完全禁用。

```text
offline current-slot RMSE = sqrt(mean_(i,channel) (a_hat_i[0]−a_expert_i)^2)
  # BC/expert/constant没有chunk维；同一批专家状态与当前动作标签，单位m
closed-loop state transition: o_(t+1) = simulation(o_t,a_t)
success = AND_(i in last10 observations) (||q_i−goal||<.001m AND ||v_i||<.002m/s)
reference step RMS = sqrt(mean_t ||a_t−a_(t−1)||²)     # m，未除dt
```

离线误差用固定expert状态分布；闭环由策略影响后续状态，形成policy分布。
只比较当前槽，不能把BC单步RMSE与ACT整个H10 masked RMSE直接排名；代价是当前槽指标不评价完整chunk。
不同执行E的同模型离线当前槽值相同，闭环结果却可不同。

全分母final mean包含失败trial；成功条件均值只对成功trial算，无成功返回null，不能用0表示。
固定时长的所有trial均truncated=True，success由独立判据决定；未满足尾窗判据记time_limit_criterion_not_met。
本XY合同没有碰撞/掉落判据，不声称全过程安全；日后接触轨道需要独立失败锁存。

## Math-to-Code / APIs

`compare_policies.py`读S19.3 prepared，校验split/normalizer/hash；复用S19.5 test校验器。
`load_chunk`校验H10、split与normalizer；ACT loader校验architecture与hash；BC loader校验task/horizon与统计值。
`torch.load(...,weights_only=True,map_location='cpu')`返回checkpoint字典；`load_state_dict`原地更新模型参数。
`eval()`设置推理模式；`inference_mode()`禁用autograd开销，均不会训练或读取未来expert标签。

部署分别调用旧BC、chunk、ACT、ensemble rollout，最后统一由`summarize_trace`评分并与旧scorer核对。
每个rollout有独立`MjData`与reset：`MjModel.from_xml_string`返回编译模型，`MjData(model)`返回可变状态；
`mj_forward`原地刷新派生量，无时间推进；`env.advance`保持reference20步、每步计算阻抗反馈并调用`mj_step`原地积分。
50Hz reference/1kHz feedback与force cap保持一致；只在reset写qpos/qvel。

协议写在执行前，包括checkpoint/XML/split/normalizer hash、全部模式、horizon、评判与test状态。
结束核验checkpoint与protocol没变；保存每个trial、offline当前动作、CSV、结果与图像。

CPU latency：batch1实际归一化→网络→反归一化，warmup20后测400个专家状态，报告mean/p95。
BC返回2channel，chunk/ACT返回20channel；测量不含ensemble融合、physics、IO，不能当完整实时系统延迟。
E5减少调用频率，不改变单次网络预测成本；expert/constant记录的是规则调用，并非神经网络推理。
训练耗时只引用带hash的历史sidecar，明确未与weights加密绑定，本次不训练、不计新训练时间。

## Minimal Experiment / Expected Result

只需requirements已有NumPy/CPU Torch/MuJoCo/Matplotlib；helpers为tiny_act/chunk_baseline/temporal_ensemble、
attention_basics/cvae_latent/sequence_dataset与S19 behavior_cloning/evaluate_rollout/episode_contract→cartesian_spring。
无新依赖或安装。输入缺失时按[Stage19 README](../examples/20_learning_from_demonstrations/README.md) S19.2→3→4、
[S20.2](23_2_chunk_baseline.md)和[S20.5](23_5_tiny_act.md)重建并替换路径。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/compare_policies.py \
  --prepared tmp/s19_3/verified193 \
  --bc-checkpoint tmp/s19_4/default/policy.pt \
  --chunk-checkpoint tmp/s20_2/default/policy.pt \
  --act-checkpoint tmp/s20_5/default/act.pt
```

输出新ignored tmp/s20_7/run_*：protocol.json、results.json、summary.csv、两组offline NPZ、64trial NPZ、comparison.png。
包含expert/constant/BC/chunk E1,E5/ACT E1,E5/uniform，共8mode×4reset×2group。
预期validation重现旧结果；test排名未必一致；更低offline loss与更平滑reference都不能保证成功。
test已在S19.5暴露，validation用于历史模型选择；本次为公开冻结复测，不称盲测，不根据test选择E或ensemble。

## Actual Result / Explanation

2026-10-12，DESKTOP-781D67A，默认`--output tmp/s20_7/default`，T100/2s：

| 模式 | validation成功 | test成功 | val final mean [mm] | test final mean [mm] | val/test当前动作RMSE [mm] |
| --- | --- | --- | --- | --- | --- |
| expert | 4/4 | 4/4 | <.001 | <.001 | 0 / 0 |
| constant | 0/4 | 0/4 | 10.895 | 11.874 | 8.002 / 9.586 |
| BC | 0/4 | 0/4 | 7.242 | 11.909 | .734 / 1.532 |
| chunk E1 | 1/4 | 0/4 | 3.585 | 4.485 | .781 / 1.724 |
| chunk E5 | 1/4 | 1/4 | 1.845 | 3.592 | .781 / 1.724 |
| ACT E1 | 2/4 | 2/4 | .875 | 2.149 | .737 / 1.943 |
| ACT E5 | 4/4 | 2/4 | .499 | 2.081 | .737 / 1.943 |
| ACT uniform | 2/4 | 3/4 | 1.151 | 2.016 | .737 / 1.943 |

BC参数1346/best79、chunk1940/best136、ACT31146/best20；历史trainloop约1.730/2.485/25.443s，含各自训练评分循环，
来自各自2026-10-11历史sidecar；训练epochs/seed为BC200/194、chunk200/202、ACT100/205（以源protocol为准）。
本次batch1 p95，val/test：BC .024/.027ms、chunk .026/.029ms、ACT .305/1.060ms。
受CPU调度等影响，测量不是硬实时保证；全部E1/uniform模式100calls，E5模式20calls。完整wall与sim time在CSV中分别记录。

解释：BC test offline更低但闭环更差，当前槽误差没有捕捉policy诱导分布与多步执行效果。
ACT E5 validation4/4不代表test4/4；uniform test3/4也不足以宣称胜出。这里只有单seed与四reset/group。
不能归因“Transformer一定更好”：模型容量、标签horizon、CVAE目标、训练seed/预算均不同。

补充命令追加`--duration-scale 2 --output tmp/s20_7/duration2`（T200/4s）：各模式两组success次数均不变；延长时间不是自动恢复策略。
两组共128trial独立Euler重算384000步；scorer/全分母aggregate/actual状态预测/执行索引与uniform动作对齐通过。
离线当前槽误差从保存NPZ重算、expert标签、三个CLI拒绝与chunk split mismatch通过；3.11语法通过，实际Python3.12.14。
默认comparison.png已查看；图仅展示每组首个episode，统计使用全部四个reset。

## Failure Cases / Robotics Context

- 改scorer、reset或低层频率：数字失去可比性。
- 不同prediction horizon离线loss混比：标签集合不同；本节只对齐当前槽。
- 用test选E后再称盲测：泄漏；本节所有预定模式都报告，不挑胜者。
- 只看成功trial均值：删除困难失败；同时保留全分母和null。
- latency遗漏范围：网络p95不是physics/通信总延迟，少调用也不自动提高任务成功。

机器人应用：部署前在相同任务、控制接口与资源边界下比较策略；需要扩充多seed、物理变化、接触与新holdout后才能支持更强结论。

## Interview Capsule

30秒：统一benchmark固定数据、reset、动作含义、控制与scorer，分别报告offline当前动作误差、闭环任务结果和计算成本。离线低误差不保证闭环成功，公开test不能用来调参后声称盲测。

2分钟：本例在同XY任务比较BC/MLP chunk/tiny ACT，冻结weights与train-only normalizer，采用成对reset。E1/E5和uniform属于显式执行变量，持续用actual状态反馈。离线只对齐当前槽标签，闭环检查最后10观测的位置与速度，所有失败保留分母，成功条件统计单列。CPU batch1 normalize/predict/decode成本与model calls分别记录，训练历史耗时标明来源。单seed、四reset、容量/训练预算不同，因此结论限定为这些冻结模型的描述性结果，不是架构因果证明。

## Must Remember / My Verification

固定执行合同；对齐指标定义；失败保留分母；成本标明范围；test不参与选择。
根README管理Engineering/Learning状态；助手执行不计本人Run。

- Run：运行上面命令，查看summary.csv与comparison.png。
- Modify：追加`--duration-scale 2`，先预测延长时间是否必然改善所有模式，再运行比较；不据此选模型。
- Explain：①为何当前槽RMSE比BC与ACT整chunk loss更可比？②为什么BC test RMSE更低但闭环可能更差？③哪些量必须固定，哪些作为显式实验变量？④为什么公开test上的3/4不足以选uniform并声称盲测胜出？⑤单次推理p95、calls、simulation time与wall time各表示什么？

本人已确认实验和预测完成，并提交五项Explain；Run/Modify/Explain全部完成，Learning Mastered（2026-10-12）。
精度补充：离线固定expert轨迹与样本，闭环固定reset/执行合同，actual trajectory由策略产生，不能固定为同一轨迹。
评价时长在同一对比中固定，另开实验可作为显式变量。RTF=simulation time/wall time，必须声明wall计时范围；本例rollout wall不包含文件保存与作图。
不自动进入S21.1。
