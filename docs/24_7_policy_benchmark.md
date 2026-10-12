# S21.7 — Same-task BC / ACT / Diffusion benchmark

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [代码](../examples/22_diffusion_policy/policy_benchmark.py)
前置S21.6；0.5～2小时。主概念：共同实验合同与多training seeds，让policy比较有明确证据范围。
Engineering Complete不代表所有policy成功；Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）。

## Problem → Why → Intuition

S21.1与S21.5/6的数字来自不同训练次数与采样分母，不能直接宣布算法胜负。
直觉：比赛要固定场地、起点、规则，再列清不同装备和计算成本。相同task/data不意味着相同优化预算，更不意味着结果可外推到所有机器人。
本节完成[P3共同benchmark合同](p3_plan.md)的B轨道比较：冻结checkpoint/配置后首次评价独立test；后续复测不再是未见过的holdout。

## Core Concepts / common contract

| 项目 | 合同 / shape / units / frame |
| --- | --- |
| task | xy_two_route_disk_v1，同XY动力学与半径10mm几何禁入圆盘；无contact反力 |
| observation | (6,)，actual world XY位置[m]、速度[m/s]、goal[m]；history1；无route/phase标签 |
| action | (2,)，world绝对XY reference[m]；不是ctrl或下一qpos |
| timing | reference50Hz、inner feedback/physics1kHz、150tick=3simulation秒 |
| data | 8train/2validation/2test paired reset；同reset上下路同组；train16episode/2400行 |
| normalization | 按train拟合mean/std；三模型raw rows与normalizer逐值核对 |
| success | 全部actual运动段clearance>0 AND 最后10观测每个error<1mm/speed<2mm/s |
| split use | validation用于各模型offline选模；test reset21101/21111只在冻结后评价 |

三组training seeds：BC211/212/213，ACT212/213/214，Diffusion214/215/216。它们是独立训练重复，数字相同不表示跨架构随机流相同。
Diffusion每个模型/reset使用sampling streams215…218，生成seed=stream+reset×1000+anchor；Modify225…228。
确定性BC/ACT每模式分母3模型×2reset=6；Diffusion每模式3模型×2reset×4stream=24。不重复确定性策略四次制造额外证据。
24trial只有两种reset条件，不是24个独立世界。比较成功比例时，各模型/reset同权，再对该条件下sampling streams平均；不把这些小样本当普遍成功率。

## Models / training budget

| 模型 | 参数 | 输出 | 优化预算与checkpoint选择 |
| --- | --- | --- | --- |
| BC | 1346 | 当前reference | 100epoch/batch64，最低validation单步MSE |
| tiny ACT | 31146 | H10 chunk | 60epoch/batch64，masked reconstruction+.01KL；最低validation zero-prior chunk MSE |
| chunk Diffusion | 23956 | H10 epsilon | 150epoch/batch128，K100；最低固定validation noise masked epsilon MSE |

CPU1thread/Adam lr.001，架构、batch、epoch及目标不同；这是既定学习实现的成本/行为比较，不是等预算架构因果实验。
ACT部署固定prior z=0，禁止调用posterior；Diffusion预声明DDPM100与eta0 DDIM25，两者均E5，未按S21.6胜负挑最佳sampler。
确定性模式BC E1、ACT E1/E5；E是额外执行变量，不把ACT E5与E1差异全部归因于架构。

## Mathematics / shared metrics

```text
current_slot_RMSE_mm = 1000 * sqrt(mean_(draw,row,XY) (reference_hat-target_reference)^2)
F_requested = 400*(reference-q) - D*qvel
ctrl = F_requested / gear                  # 实际actuator限力±20N
success_fraction = successful_attempts / all_attempts
reference_step_RMS_mm = 1000 * sqrt(mean_t ||a_(t+1)-a_t||²)
```

offline只在固定validation expert状态评价共同当前槽，不混比BC单步loss、ACT chunk/CVAE loss与Diffusion epsilon loss。
Diffusion在每个固定状态从prior抽四段chunk，mask由该expert anchor的固定剩余预算决定，评分只取槽0。
alternate合法模式可能与配对expert标签不同，因此该RMSE是reference标签一致性，不是生成质量或安全的完整度量。
offline状态来自expert；closed-loop状态由各policy自己的动作与动力学产生，必须分别评价。
报告全分母final error；success-only error只在成功trial上定义，无成功时为undefined，不填0。
碰撞锁存：uniform mean即使tail到达也失败。BC无碰撞但未到达，同样失败。有限力不限制无界slide的位置。

## Math-to-Code / APIs

`prepare(cohort)`现场采集同B数据，依次训练九个模型；BC/ACT复用multi.train新增可选seed，默认211/212保持；Diffusion用sequence_diffusion新增--train-seed，默认214保持。
两处只开放训练seed参数，不改变原任务默认算法；checkpoint记录实际seed。
`restore`核验cohort/data/checkpoint SHA、task/schema、paired split、normalizer与跨模型raw rows，返回eval模型和训练记录。
cohort.json存选好的全部weights与校验值；protocol.json在offline/rollout前写入模式、seeds、评分与checkpoint SHA，评价后再确认未变。
`--models-from`直接复用cohort，不重训、不改weights。生成的数据、cohort和日志仅在ignored tmp内。

MuJoCo复用task.rollout：MjModel.from_xml_string编译返回model，MjData创建可变状态；只在reset写qpos/qvel，mj_forward刷新派生量不推进时间，mj_step原地积分qpos/qvel/time。
chunk接口kind='act'用于ACT或Diffusion adapter，不表示Diffusion调用ACT网络。
Diffusion使用S21.6 Planner/draw，实际观测与budget mask重新规划；ACT infer只有状态输入，未来动作标签仅用于训练。
依赖已有NumPy/torch==2.14.0+cpu/MuJoCo/Matplotlib。helpers为multimodal_paths/TinyACT/BC/windows/CVAE/attention、sampling_diagnostics及S21.4/5/task链；无新库或安装。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/policy_benchmark.py
```

默认新tmp/s21_7/run_*，准备三组训练模型，再评价validation与已经暴露的B test；不读test训练标签或拟合test normalizer。
显式--output目录须不存在；--models-from可省去准备阶段。
输出cohort/数据及九个checkpoint、protocol、12份offline预测NPZ、每组72trial NPZ（共144）、trials.csv/results、latency.npz、benchmark.png。
预期专家成功/mean碰撞；学习策略实际结果不预设。所有模型、seeds和失败attempt保留，不以rollout挑模型。

## Actual Result / Explanation

2026-10-12 Zero；WSL2 Ubuntu24.04.5/kernel6.18.33.2，mujoco内Python3.12.14；NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.13.0/Matplotlib3.11.2 metadata、真实paths/API通过，无安装。
默认准备命令加`--output tmp/s21_7/default`成功，完成九模型训练与初轮validation。
完善offline预算mask与统一整段decode计时后，用冻结cohort运行最终命令：

```bash
python examples/22_diffusion_policy/policy_benchmark.py \
  --models-from tmp/s21_7/default/cohort --output tmp/s21_7/with_test
python examples/22_diffusion_policy/policy_benchmark.py \
  --models-from tmp/s21_7/default/cohort --sample-seed 225 --output tmp/s21_7/test_seed225
```

两组exit0。共同合同检查要求独立test，最终with_test首次评价B test；模型/配置/seeds已冻结。Modify是公开复测，不能调参或挑sampling seed。

| 模式 | validation success / collision | test success / collision | val/test全分母final mean [mm] |
| --- | --- | --- | --- |
| expert upper/lower（各自） | 2/2 / 0 | 2/2 / 0 | 均<.001 |
| uniform mean | 0/2 / 2 | 0/2 / 2 | 均<.001，但全过程失败 |
| BC E1 | 0/6 / 0 | 0/6 / 0 | 22.414 / 17.886 |
| ACT E1 | 1/6 / 0 | 0/6 / 1 | 4.708 / 3.409 |
| ACT E5 | 6/6 / 0 | 6/6 / 0 | .255 / .261 |
| Diffusion DDPM100 E5 | 0/24 / 4 | 0/24 / 3 | 3696.944 / 4360.369 |
| Diffusion DDIM25 E5 | 0/24 / 6 | 0/24 / 6 | 3671.058 / 2044.638 |

ACT E1 test有一次tail到达，但途中碰撞导致success仍0；末态不能掩盖全过程失败。
ACT E5每training replica val/test均2/2。validation成功路线总体upper3/lower3，是不同训练模型/状态的汇总，不证明同一模型同一状态覆盖两个模式。
Diffusion均有大误差/发散；低epsilon loss没有转化为本例可靠执行。原因仍未因果定位，不凭此声称Diffusion本质不适合机器人。

offline当前槽RMSE（三training seed范围）：BC .187–.201mm，ACT .400–.495mm，DDPM2.621–2.845mm，DDIM25 3.346–3.685mm。
BC标签误差低却无成功；说明离线指标与闭环任务之间不能直接互推。

| 模式 | val/test tail最大速度的trial中位数 [mm/s] | val/test reference step RMS的trial中位数 [mm] | val/test success-only final error [mm] |
| --- | --- | --- | --- |
| BC E1 | 24.051 / 15.186 | .872 / .869 | undefined / undefined |
| ACT E1 | 8.015 / 7.859 | 1.111 / 1.241 | .704 / undefined |
| ACT E5 | .499 / .505 | .943 / .946 | .255 / .261 |
| DDPM100 E5 | 34.301 / 41.173 | 5.838 / 5.600 | undefined / undefined |
| DDIM25 E5 | 34.213 / 35.640 | 7.045 / 6.717 | undefined / undefined |

上表从保存trial/trace独立计算，中位数不能代替每trial速度阈值；成功条件仍按每trial最后10观测逐值判断。

| 模式 | batch1 pipeline median / p95 [ms] |
| --- | --- |
| BC E1 | .058 / .097 |
| ACT E1 | .374 / .637 |
| ACT E5 | .328 / .592 |
| DDPM100 E5 | 7.357 / 17.585 |
| DDIM25 E5 | 1.704 / 3.013 |

每模型3warmup+20计时，各模式汇总60次，轮换顺序。范围为cached observation normalize→整个predict/sample→整段decode→finite检查，不含physics/传感/I/O。
ACT E1/E5调用相同infer，耗时差是这轮测量差异，不是E改变网络结构。同步仿真不模拟等待期间物理运动，不能据延迟宣称硬实时。
BC/ACT/Diffusion trainloop三模型分别1.409–1.537s、13.826–15.155s、2.843–2.902s；不含数据/采样/绘图。不同epoch/batch/目标，不当等预算胜负。
selected epochs：BC96/96/99、ACT60/43/56、Diffusion146/150/150。

Modify225…228：确定性与控制baseline轨迹逐值不变；Diffusion仍全0/24，DDPM val/test碰撞5/2、DDIM8/10，错误量随随机流变化。weights/cohort SHA保持不变。
最终两组288trial独立864000Euler步/geometry/scorer，全部13680actual预测anchor与计划槽、offline预测/指标、分组aggregate/per-replica分母/latency重算通过。
seed214 Diffusion weights与原S21.6逐值一致；metadata seed/hash校验、tampered checkpoint拒绝、两非法seed CLI、Python3.11语法、链接与git diff --check通过（实际3.12）。benchmark PNG已查看。
保留全部起始未提交文件。无GUI/接触反力/OOD/硬件验证；test现已公开，只有两test reset、三training seeds，不做统计显著性或算法普遍优越性声明。

## Failure Cases / Robotics Context

- 混用A轨道到达任务与B轨道绕障数据：跨task数字不可排名。
- 比不同loss单位/标签horizon：这里只用共同当前槽reference指标，选模loss各自独立。
- 重复确定性策略当独立采样，或把四sampling seed当四training seed：虚增证据。
- 只报success-only error、删碰撞/发散或选最好seed：改变分母与真实失败面貌。
- 看到test结果后挑E/model/sampler：污染holdout；本节已暴露test，未来修改须新holdout或明确公开复测。
- 用小任务ACT E5全成功证明架构普遍优越：预算、实现、状态覆盖和任务尺寸都有限。

机器人应用：策略评估要同时看学习标签一致性、actual执行、全过程约束和计算成本。选择候选实现是工程决策，还需要更广任务/分布/控制验证。

## Interview Capsule / Must Remember

30秒：固定task/data/normalizer/reset/control/scorer，对三组训练模型评价离线当前槽误差和真实闭环；Diffusion额外报告sampling seeds及分母。冻结后首次test，全部attempt保留。本例ACT E5 val/test6/6，BC与Diffusion失败，结论仅限这组实现与任务。

2分钟：先写共同合同，再列不同模型容量/预算/输出horizon与E；解释offline loss不混比、training/sampling seeds及分组分母；描述checkpoint SHA、先冻结再test、collision锁存与success-only统计；列统一pipeline计时范围；最后解释失败和小样本限制，不把已暴露test用来择优。

Must Remember：same task不等于equal budget；offline不等于闭环；training/sampling seed分开；所有attempt；先冻结再test；成功学习不等于可靠策略。

## My Verification / Run → Modify → Explain

Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），根README为状态唯一来源。
解释覆盖共同合同与预算、training/sampling seeds及分母、不同loss与当前槽RMSE边界、全过程成功判据、test冻结与有限样本可靠性。
第四问精度补充：ACT E1 test的tail到达trial因途中collision而失败，本任务没有额外姿态要求；不把假设的超时或姿态问题当作本例实际原因。
Run：运行默认命令，查看benchmark.png、分组/per-replica成功率、collision与tail、训练成本和计时范围。当前test是公开复测。
Modify：先预测冻结模型换sampling streams后哪些结果不变，替换Run打印的cohort路径：

```bash
python examples/22_diffusion_policy/policy_benchmark.py \
  --models-from tmp/s21_7/default/cohort --sample-seed 225
```

不重训、不改E或sampler，不按test挑更好的seed。
Explain（五问）：
1. 公平比较需要固定哪些合同？为什么same data不等于equal training budget？
2. 三training seeds与四sampling seeds分别检验什么？为什么BC/ACT分母6、Diffusion24？
3. 为什么不能直接排名BC/ACT/Diffusion训练loss？共同当前槽RMSE仍有哪些局限？
4. 为什么ACT E1 test tail到达仍失败？为什么成功条件误差不能替代全分母失败统计？
5. test冻结/首次评价/公开复测有何区别？为什么ACT E5 6/6与低延迟仍不足以证明普遍优越或硬实时？

本人已完成当前Run/Modify/Explain，Stage21学习项全部Mastered；下一步复盘或S22.1需明确请求，不自动进入Stage22。
