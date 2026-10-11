# S19.8 — BC Integration Benchmark

## Problem → Why → Intuition

问题：如何把示范、预处理、模型、动作执行和评分连接成一个可复核的机器人学习结果？
低offline loss不是最终验收；benchmark要问“相同任务条件下，实际执行是否完成任务”。
本课固定已有XY fixture，不重训、不换任务，不以跑出成功为理由修改阈值。
学习规模0.5～2小时；Engineering状态与本人Learning三项唯一见[README](../README.md#stage-19--learning-from-demonstrations)。

直觉：先写实验合同，再让每个policy从同一reset出发。记录全部尝试，即使都是失败。
将结果分成expert分布上的动作预测、policy自身闭环任务结果和CPU执行成本，避免混为一个“分数”。
S19.5原test已经公开；本课是固定权重的描述性复测，不是新的独立盲测。

## Core Concepts — 固定什么、报告什么

| 项目 | 本课合同 |
| --- | --- |
| Task/model | xy_goal_reach_v1；Stage17无接触XY slide fixture，不是UR5e抓取 |
| Observation | (6,) world XY位置m、速度m/s、目标XY位置m |
| Action | (2,) world XY绝对位置reference，m；不是ctrl或qpos |
| Dataset | S19.3同一split：16train / 4validation / 4test episode；统计量仅train拟合 |
| Checkpoints | S19.4 BC；可选S19.7聚合checkpoint，全部先冻结，不据本次结果选模型 |
| Baselines | expert、训练action均值constant；都走同一controller与MuJoCo动力学 |
| Frequency | policy50Hz；reference保持期间controller/physics1kHz |
| Horizon | 默认100tick/2s；Modify全policy统一200tick/4s，另报协议 |
| Success | 最后10个observation每个位置误差<1mm且速度<2mm/s；阈值不变 |
| Failure | 达到time limit而未满足判据；仍计全部attempt；success也可同时truncated |
| Outputs | protocol.json先写；results.json、summary.csv、每次闭环NPZ；只放ignored tmp |

Validation与test分别报告，不能拼成8个“独立盲测样本”。没有重新划分episode、
拟合normalizer或训练模型。加载checkpoint使用CPU、weights_only，核验split hash和保存统计量。
额外模型在结果中叫additional_0，其实际path/hash记录在protocol。
只有一个既有训练seed、每组4个reset；没有足够证据排序算法总体能力。

## Mathematics — Shape / Unit / Meaning

Offline动作RMSE：`sqrt(mean((pred_action−expert_action)^2))`，对N×2元素平均，单位m。
它是expert observation分布上的单步预测误差，不是末端任务误差。

闭环位置误差：`e_t=||position_t−goal||₂`，单位m；速度`v_t=||velocity_t||₂`，m/s。
Success是尾部10个采样都满足两条件，不只检查最后一帧。
成功率`sum(success)/attempts`，所有完成的trial都在分母里。
全分母mean final error与仅成功trial的mean final error分开；没有成功时后者为null，不写0。

动作变化：`sqrt(mean((a_t−a_(t−1))^2))`，对(T−1)×2元素平均，m。
它是本协议下reference变化代理，不是实际机械加速度、jerk或安全证书。
constant变化为0仍可任务失败；不能把“更平滑”自动解释为“更好”。

Controller继续`F=K(reference−q)−D*qvel`，K=400N/m，D=2sqrt(MK)，M=[2,1]kg；
gear把F转换为ctrl，每轴力限幅20N，实际状态由mj_step推进。

## Math-to-Code / APIs

[integration_benchmark.py](../examples/20_learning_from_demonstrations/integration_benchmark.py)：
`bc.load_prepared`检查train/validation与normalizer；`load_evaluation`检查冻结checkpoint和test。
`offline_prediction`用当前观测归一化→CPU MLP→动作反归一化。
`bc.rollout`对每个reset/policy创建独立模型和状态，逐tick读取actual observation，再预测和执行。
只在初始化赋qpos/qvel；不使用示范action replay替代学习策略闭环。

MuJoCo复用API：`MjModel.from_xml_string(XML)`返回编译的模型，`MjData(model)`返回独立可变状态；
`mj_forward(model,data)`原地更新派生动力学量不推进时间；
`mj_step(model,data)`原地推进1ms物理状态，每个policy tick20次。均由既有helper保持可见。

先保存protocol、权重/模型/数据hash与分组，再执行两组paired trials，保存完整N+1观测/N动作。
结束确认checkpoint未变，核对初态与scorer；工程检查PASS和机器人success分别报告。
若运行发生异常，命令失败，不能把已完成的部分当作完整benchmark或静默排除失败trial。

## Minimal Experiment — Run

依赖requirements已有NumPy/MuJoCo/Matplotlib/Torch CPU；无新增包。
先核验conda环境，使用本机实际的prepared/checkpoint路径：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/integration_benchmark.py \
  --prepared tmp/s19_3/verified193 \
  --checkpoint tmp/s19_4/default/policy.pt \
  --additional-checkpoint tmp/s19_7/default/aggregated_bc.pt
```

若ignored输入不存在，按S19.2→S19.3→S19.4重建，并将打印路径代入。
额外checkpoint可省略；要复测聚合模型则先按S19.7生成。跨电脑不假设tmp自动跟随Git。

## Expected / Actual Result — 2026-10-11 DESKTOP-781D67A

预期：expert达到任务条件；离线准确的BC仍可能闭环失败；聚合选step0则行为与父模型一致。
默认命令及`--duration-scale 2`均exit0，含四policy×两组×四reset=32trial/命令。

| Group / Policy | Offline RMSE mm | 2s success | 全attempt平均终态误差mm | 4s success | 4s平均终态误差mm |
| --- | ---: | ---: | ---: | ---: | ---: |
| Validation / Expert | 0 | 4/4 | ≈0 | 4/4 | ≈0 |
| Validation / Constant | 8.002 | 0/4 | 10.895 | 0/4 | 10.895 |
| Validation / BC | 0.734 | 0/4 | 7.242 | 0/4 | 7.470 |
| Validation / Aggregated | 0.734 | 0/4 | 7.242 | 0/4 | 7.470 |
| Exposed Test / Expert | 0 | 4/4 | ≈0 | 4/4 | ≈0 |
| Exposed Test / Constant | 9.586 | 0/4 | 11.874 | 0/4 | 11.874 |
| Exposed Test / BC | 1.532 | 0/4 | 11.909 | 0/4 | 12.000 |
| Exposed Test / Aggregated | 1.532 | 0/4 | 11.909 | 0/4 | 12.000 |

CPU1thread、batch1；先20次warmup，然后每组400条expert观测单次测量。
包含normalization、forward、反归一化与Python调用；不含controller、MuJoCo、IO，也不是实际BC访问状态的计时。
默认test BC mean/p95约0.0217/0.0252ms；微秒级测量受调度与计时开销影响，不宣称硬实时保证。
Rollout wall每policy每组4次约0.428～0.471s，包含动力学与已有逐步assert，不是纯推理时间。
Train时间从历史sidecar单列：S19.4约1.730s、S19.7聚合约0.336s，仅各自训练循环；
不是本次实测，也非完整数据采集成本；sidecar有路径/hash但未与权重加密绑定，不提供强来源证明。
若无匹配sidecar则为null。两次协议duration不同，不能用来选模型后仍称独立评价。

验证：64保存trial，独立重算192000 Euler物理步、评分/动作变化/全分母、hash、相同reset；
4s与2s前缀完全相同，聚合与原BC全部轨迹相同。CLI非法duration、已存在output、缺checkpoint及人为checkpoint-split mismatch均exit2（后者在rollout前拒绝）。
Python3.11仅语法检查；实际Python3.12.14。无GUI、接触/UR5e/OOD或硬件测试。

## Explanation / Failure Cases / Robotics Context

结果支持“本模型offline预测比constant好，但任务闭环未达标”；没有定位唯一失败根因。
S19.6显示策略状态访问与误差传播不同，S19.7新增label拟合下降但所选权重未改善。
本课把这些边界放进可复核报告，保留失败基线供后续同task/split/metrics的时序策略比较。
ACT尚未实现，本课不凭BC失败推断ACT一定成功。

常见错误：统计只保留成功、NULL写成0、误把wall当simulation time、对不同horizon混排名、
用test调整threshold/checkpoint、把constant低动作变化当高质量、把四次成功当鲁棒性证明。
实际机器人要另外评估接触、质量/摩擦变化、噪声/延时和未见任务；这些留在相应后续阶段。

## Interview Capsule / Must Remember

30秒：BC benchmark要固定数据和执行合同，把offline动作拟合与closed-loop任务成功分开。
专家和学习模型同reset/低层控制/判据，保留失败分母；报告成本和test是否已使用。

2分钟：说明episode split、train-only统计、checkpoint冻结与hash，预测输出反归一化成reference后
经反馈controller进入MuJoCo；同reset配对比较任务成功/终态误差，同时测CPU batch1推理。
低offline RMSE不能保证策略诱导状态分布上准确；没有成功时条件均值为空，不把训练loss或工程PASS当成功。
说明四reset/单seed/公开test复测的证据边界，新的独立泛化测试要另行冻结协议。

Must Remember：协议先于结果；全部attempt入分母；时间要注明测量边界；复测不自动恢复盲测独立性。

## My Verification — Run / Modify / Explain

Engineering完成；Learning三项待本人确认，状态唯一来源为根README。
Run：运行上文命令，查看protocol、summary.csv与results中的training provenance，区分工程PASS与task success。
Modify：先预测只加`--duration-scale 2`会改变哪些结果，再运行；
检查offline RMSE/weights/split不变、所有policy horizon一起延长，2s轨迹是4s轨迹的相同前缀。
不要从此结果重选模型或调成功阈值。

Explain：
1. 为什么protocol必须在rollout前冻结？哪些变量必须共享，哪些差异必须明确报告？
2. 全attempt平均误差与success-conditioned平均误差各回答什么？0个success为何是null而非0？
3. Offline RMSE较低、reference变化较小，为何都不能代替闭环任务成功？
4. Train wall、batch1推理延迟、rollout wall与simulation time各包括什么？小p95为何不是实时保证？
5. 为什么本课Test是公开复测？怎样设计下一次独立泛化评价，避免边看结果边选模型？

本轮停S19.8；Stage19 Engineering全部完成不等于本人Learning全部完成。
下一小任务S20.1 sequence dataset / boundaries / mask，等待明确请求，不自动实施。
