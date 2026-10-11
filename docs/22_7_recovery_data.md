# S19.7 — Recovery Data / DAgger Intuition

## Problem → Why → Intuition

问题：怎样补充BC真实访问状态上的正确动作，以及离开专家轨迹后的恢复示范？
S19.6已经看到expert-state与BC-state误差不同。本课不只诊断：让冻结BC访问状态，
查询同一当前状态的expert标签，再让expert接管恢复，将新增train来源数据聚合并重训一次。
这是一轮教学实验，不是完整多轮DAgger、不提供原论文的no-regret保证。

[DAgger原论文](https://proceedings.mlr.press/v15/ross11a.html)的核心是按当前策略访问的状态查询专家、
聚合数据并迭代更新策略。本课只收集冻结父策略的一批数据并更新一次，额外加入expert恢复后缀；
没有在更新策略上继续采集、没有多轮行为混合schedule或收敛证明。

## Core Concepts

| 数据 / 行为 | 含义 / shape / unit / frame |
| --- | --- |
| original expert rows | 原train16episode×100=1600，world XY absolute reference标签 |
| learner-state relabel | 冻结BC执行前2s，查询这100个实际状态的expert动作，16×100=1600标签 |
| executed action | 真正推进下一状态的reference；BC前缀与expert后缀分别执行 |
| expert takeover recovery | 第100tick起，在同一MjData中换expert控制，继续2s；1600个执行过的expert动作 |
| observation | 完整采集(201,6)，[x,y,vx,vy,gx,gy]，m/m·s⁻¹/m |
| executed_actions / expert_actions | 各(200,2)，world reference m；前缀两者通常不同，后缀相同 |
| source group | 由原episode ID继承train；所有衍生行/恢复后缀保留同一来源组 |

不从validation/test采集训练标签；normalizer保持原train统计。不要把S19.6的validation诊断文件拿来训练。
失误前缀也保留；标签来自expert的当前状态决策，不是把另一条成功轨迹同tick的动作粘过来。
标签正确仍依赖expert能力和任务合同；本例是无接触XY可达goal，小范围恢复，不证明真实复杂任务专家总能纠正。
expert接管恢复成功不等于新learner学会恢复。

## Mathematics → Math-to-Code

原数据D₀={(o^E,a^E)}；新增D₁={(o^π,π_E(o^π))} ∪ {(o^recovery,π_E(o^recovery))}。
动作标签仍是absolute XY reference，不是“纠正量”、reward或实际qpos。
训练目标：L=(L_original+λ L_new)/(1+λ)，每部分都是normalized action MSE，按rows/两轴平均。
默认λ=1使original/new各占50%损失权重；new内部1600relabel/1600recovery等量，期望各25%。
这不是对全部4800行均匀抽样（那会使new占2/3）。λ=0完全忽略新增标签梯度。

`collect`：reset一次→100tick frozen BC actual execution+expert query→同一状态100tick expert takeover。
`expert_action(o)`仅用当前位置和goal：x+clip(goal−x,±5mm)；`predict`用冻结BC与原normalizer。
`env.advance`保持400N/m阻抗、实际速度反馈、gear、±20N motor cap及1kHz物理；policy50Hz。
MuJoCo API：`MjModel.from_xml_string`返回编译模型，`MjData`返回独立可变状态；
`mj_forward`原地更新派生量不推进time，`mj_step`在advance内原地推进1ms（每tick20次）。
接管时没有reset/赋值qpos/qvel，也没有teleport到expert理想轨迹；仅切换reference生成者。

`relabel_pairs.npz`用observations[:-1]对应expert_actions，并保存source_episode_id/source_group/phase。
重要：前缀expert_actions是counterfactual建议，并未执行，不能用它重放并期待重现原BC观测。
完整原轨迹重放必须用executed_actions；未来ACT sequence也不能把这些逐状态建议假装成真实执行的expert action chunk。
专家恢复后缀才是实际执行的expert时序轨迹。

## Matched-Update Ablation

两支从同一个S19.4 checkpoint(epoch79)的权重开始，均新建Adam(.001)，500update、batch64、seed194。
continued_bc仅原数据；aggregated_bc原+新增标签。两支抽取同一original batch序列；
对照支仍抽取但不使用new索引，避免随机序列不同影响配对。新增支有额外forward/backward计算，
相同优化步数不是完全相同FLOPs/wall预算；本课不声称排除所有计算预算差异。
每10步及最后一步检查原validation expert-state loss，包含step0候选，deepcopy最小loss checkpoint。
没有使用validation rollout选模型；所有原验证reset/goal/horizon/scorer保持一致；test未加载。
选checkpoint是原有离线规则，未为了“聚合看起来成功”改为另一评价目标。

## Minimal Experiment

依赖requirements已有CPU Torch2.14.0+cpu/NumPy/MuJoCo/Matplotlib；
helper为behavior_cloning、covariate_shift.predict/expert_action、episode_contract、cartesian_spring，无新增包。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/recovery_data.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt
```

使用本机匹配的prepared/checkpoint；缺ignored产物时按旧S19.2→3→4流程重建并记录本机结果。
每次打印新ignored tmp/s19_7/run_*：16collection NPZ、relabel_pairs.npz、results.json、
continued/aggregated checkpoint与loss CSV、16validation rollout NPZ、recovery.png。
原dataset/split/normalizer/父checkpoint不覆盖。source ID都属于train，派生行不会重新随机划分。

## Expected / Actual Result

2026-10-11 DESKTOP-781D67A / WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco所属Python3.12.14。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2/Torch2.14.0+cpu metadata/import/API/helper路径核验；
Torch CUDA build None/tensor CPU，无安装/requirements改动。

```bash
python examples/20_learning_from_demonstrations/recovery_data.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --output tmp/s19_7/default
python examples/20_learning_from_demonstrations/recovery_data.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --new-weight 0 --output tmp/s19_7/no_new
```

两命令exit0；采集各16×4000=64000physics步，validation4reset×4controller×2000=32000步。
两组采集数组完全一致；原train1600、policy-state relabel1600、executed recovery1600。
采集阶段frozen BC前缀成功1/16；同实际状态expert接管后成功16/16（全部attempt分母，不删失败前缀）。

| 默认λ=1 | continued_bc | aggregated_bc |
| --- | ---: | ---: |
| 500update最后original train MSE | .00144615 | .00304508 |
| 500update最后new-label train MSE | .03990877 | .01021791 |
| 最后validation MSE | .00725983 | .01802053 |
| 最低validation选出的step | 310 | **0** |
| selected validation MSE | .00711188 | .00759851 |
| training wall（不含采集/rollout/save） | .301s | .336s |

新增label loss从初始.04362707下降到.01021791，但原validation loss变差；
按冻结的选模型规则，聚合支保留step0的父策略权重。不能把最后500步loss改进说成selected策略改进。
默认validation expert4/4、frozen_bc0/4、continued_bc0/4、aggregated_bc0/4。
Modifyλ=0时continued/aggregated两个训练CSV与selected weights相同，均best_step310，rollout仍0/4。
no_new wall约.234/.225s；小样本wall差异不作算法速度结论。

## Explanation → Failure Cases

已证实：train来源的policy-state可以查询正确expert reference；expert能在连续实际状态上恢复；
新标签参与训练并降低该训练集合loss。但没有证实一次聚合改善所选learner的闭环成功。
可能存在new/old分布权重与表示取舍，以及原expert-state validation标准不反映恢复状态表现。
本课没有逐因素消融或更换选模型规则，不把这些候选解释宣称为已定位根因。
collection取自父策略；新策略访问分布可能再变化，正是多轮聚合的动机，不能从单轮得到DAgger完整保证。

- expert_actions与executed_actions混淆，会把错误行为或counterfactual动作当实际transition。
- 对派生rows随机split会泄漏同一源episode；必须继承source group，不能按新文件名独立分组。
- expert接管16/16只说明expert，不能标learner恢复完成。
- 更多labels/训练loss下降不保证success；保留step0是合法validation选择，不是训练没发生。
- λ=0没有new-label梯度，但仍收集/保存数据用于学习对照；不能称没采集。
- test已在S19.5查看；本课不用它选模型，不新增OOD/最终盲测证据。

验证：32collection+32validation保存轨迹独立重算192000Euler步、raw同状态expert标签与实际执行动作/scorer/hash检查通过。
聚合row-source/phase/shape、无val/test来源、原normalizer不变、CSV argmin/checkpoint重载通过。
λ=0两支weights精确一致；默认selected step0与父权重精确一致；父checkpoint SHA不变。
非法new-weight nan/negative、steps0、已有output、split错配均exit2；Python3.11语法解析（未实际3.11执行）及两PNG目视通过。
links/anchors/git diff --check通过；无GUI、第二轮聚合、真人teleop、接触任务或test调参。

## Robotics Context / Interview Capsule

应用：经典controller纠正学习策略、遥操作接管示范、policy-induced state标注与数据质量审查。
30秒：DAgger思路把专家标签补到策略自己访问的状态；本课只做一轮，并保留来源分组与真实闭环验收。
2分钟：区分执行动作与专家建议、BC前缀与连续expert恢复；解释D₀∪D₁、λ加权、matched-update对照、
train来源继承与normalizer冻结，再用新增loss下降/selected step0/learner0of4说明学习数据与任务成功的区别。
Must Remember：同状态查询；执行与标签分开；派生来源不跨split；一次聚合不保证成功。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。Learning三项待本人完成。
Run：执行默认命令，核对三类row计数、source_group、collection成功分母、selected_step与validation rollout。
Modify：先预测只将new-weight1→0会改变哪些训练结果、哪些采集数据不变，再运行：

```bash
python examples/20_learning_from_demonstrations/recovery_data.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --new-weight 0
```

Explain（五问）：
1. Learner-state expert label与executed action有什么区别？为什么不能直接重放前者复现原轨迹？
2. Expert接管恢复如何保持真实动力学连续？16/16为何不等于learner成功？
3. 为什么新数据必须继承train source group、保留原normalizer，不能从validation诊断文件补训练？
4. λ=0对照与matched updates帮助隔离什么？为何新增label loss下降仍选step0且闭环0/4？
5. 本课体现DAgger的什么核心思路？还缺什么才能称完整多轮DAgger及证明泛化改善？

下一小任务S19.8 BC integration benchmark，等待明确请求；本轮停止。
