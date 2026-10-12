# S22.9 — Final Learning-Based Manipulation Benchmark

[入口](../examples/23_contact_aware_learning/final_benchmark.py) · [示例README](../examples/23_contact_aware_learning/README.md) · [状态唯一来源](../README.md)

## Problem → Why → Intuition

P3最终问题不是“哪个网络loss最低”，而是：在给定任务、数据和执行合同下，冻结策略是否能在实际闭环中完成操作，代价和失败边界是什么？
benchmark把各阶段实验整理成可审查的证据。统一的是实验纪律与指标定义；不同物理任务仍各自评分，不把不同成功率相加成为算法总排名。

本课复用既有算法，不新增恢复训练、大模型、视觉encoder或硬件控制。A审计已有产物；B本机缺失历史cohort时按S21.7原预算重新准备并公开复测；C按S22.4原预算准备，冻结后首次评价预留test；D复用冻结三指BC，首次评价预留test。
“首次”只指第一次S22.9执行。产物一经查看，后续Run/Modify都是公开复测，即使使用相同reset仍不能称新盲测。

## Core Concepts：四轨道合同

| 轨道 | 任务 / 可观测输入 | action / 单位 | 固定执行与评分 | 已有算法 |
|---|---|---|---|---|
| A | XY目标到达；6维世界位置/速度/goal | 2维绝对XY参考[m] | 50Hz/1kHz，2s；末10观测位置<1mm、速度<2mm/s | BC、MLP chunk、ACT；Diffusion未训练 |
| B | XY双路径绕几何禁入圆盘；6维 | 2维绝对XY参考[m] | 3s；实际每1ms运动segment净空>0，且末10观测到达/低速 | BC、ACT、DDPM100/DDIM25 Diffusion |
| C | 两slide表面负载跟随；full10维位置/速度/goal/力/touch | 2维actual-q锚定reference delta[m] | 3s；0.6s后接触、任意力≤5N、末250ms位置/力/速度稳定 | BC、ACT、DDIM25 Diffusion |
| D | 三指自由物体保持/扰动；38维因果状态/法向力/past ctrl/time/phase | 6维绝对关节参考[rad] | 4s；50ms接触gate、ready、全过程failure latch与末250ms恢复 | BC；ACT/Diffusion未训练 |

所有轨道低层动力学/限幅各自保持原合同。算法间固定同轨道数据、normalizer、reset、scorer和可观测通道；E、采样预算、模型容量是显式变量，不宣称相同训练预算。
C/D输入含仿真物体/接触真值，D含干净monitor phase；不是已验证的真实传感系统。
B禁入区是几何判据，没有障碍接触反力。C扰动是表面geom高度jump，不是经过动力学控制的实体运动平台。

| 轨道 | split与训练seed | test分母与暴露状态 |
|---|---|---|
| A | 原group split与原单seed模型 | 每模式4reset；S19.5已暴露，本课仅历史产物审计 |
| B | 8train/2val/2test paired reset；BC211–213、ACT212–214、Diff214–216 | 确定性6、Diff24（3模型×2reset×4采样流）；S21.7已暴露 |
| C | 16train/4val/4test reset；两direction同组；每类seed223–225 | expert8、BC/ACT各24、Diff各48；先冻结九模型再用test22400–22403 |
| D | 12train/3val/3test reset；BC226–228 | 每pulse expert3、BC各3，三BC合计9；test22700–22702首次使用 |

采样流不是新的reset或新的训练模型。报告`per_training_seed`；不能将相关attempt当作全部独立样本做显著性声明。
缺少对应任务模型时写“未评价”，不能伪装为0%失败，也不能搬用另一轨道checkpoint或结果。

## Mathematics：指标意义、shape、单位

对一个固定 `(track, mode, scenario)`，全部计划attempt数为N：

\[
 p=\frac{1}{N}\sum_{i=1}^{N}\mathbf1[\mathrm{success}_i].
\]

保留失败、未ready、未收到pulse的attempt；不只取成功或有效抓持的子集。
全分母误差\(\bar e=N^{-1}\sum_i e_i\)和成功条件误差\(\bar e_{succ}=\sum_i I_i e_i/\sum_i I_i\)含义不同；没有成功时后者为null，不能填0。
A/B位置误差与C切向误差为mm；D drift为m、slip为m/s、旋转为rad；接触力为N，推理耗时为ms。跨轨道误差不直接平均。
C的recovery要求先实际观察到fault，再在restore后连续100ms满足接触/负载带；它不清除failure latch，因此恢复后仍可success=False。
D `ready`在2.5s冻结，pulse只有ready时送达；基础.1N向下載荷在2s后HOLD生效，未pulse不等于未加载。

## Math-to-Code / APIs

`final_benchmark.py`用单独Python子进程运行C/D，避免不同轨道的模块导入改变训练source inventory。
`contact_worker`复用S22.4 prepare/restore/rollout/scorer：只改变评价reset为预留test，训练与normalizer只用原train/val。
新入口SHA记录在evaluation protocol中，不加入原训练helper集合；checkpoint provenance保持原样。
九模型task/schema/seed/H/K/source/normalizer与manifest/data SHA均核验，协议写入后才开始test。
ACT部署通过`infer`使用z=0，Planner禁止posterior；Diffusion每次规划只接收当前观测和horizon mask，不接收未来标签。

`dexterous_worker`复用S22.6/7的cohort校验器和任务，原baseline/Fx±/Tx±五pulse都报告。
`checked_historical`核对A协议/轨迹SHA、B保存的物理trace，再把实际输入文件SHA登记到最终evidence。
最终results保存分轨道contracts、coverage与证据索引，summary.csv/benchmark.png仅展示成功计数；offline、训练成本、误差、latency和失败细节保留在各轨道results中。

MuJoCo API仍直接出现在旧任务中：`MjModel`编译模型；`MjData`创建独立可变状态；`mj_forward`原地刷新当前状态的接触/传感器而不推进时间；`mj_step`原地推进1ms。
策略reference经原低层actuator和动力学执行，评分不暂停仿真，末帧不能洗掉全过程失败。
NPZ保留N个动作、N+1策略观测、完整1kHz轨迹；B为`physics_xy`，C/D为`physics`，汇总adapter必须尊重各轨道字段语义。

## Minimal Experiment / Expected Result

依赖requirements已有mujoco/numpy/matplotlib/torch==2.14.0+cpu与Stage19–22 helper，无新依赖。
默认需要A S20.7产物和D S22.6/7 cohort；未指定B/C时生成本机原协议模型，所有产物仅ignored tmp。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/final_benchmark.py --output tmp/s22_9/my_run
```

如果已有B/C，可以加`--b-results <S21.7目录> --c-models <S22.4 cohort目录>`；A/D可用`--a-results`、`--d-models`指定。
output须新目录。本机本课工程运行已暴露C/D test，后续运行应加`--repeat`，不能因换机器或输出目录恢复盲测。A/B test仍是公开证据；C/D第一次执行会暴露预留test，之后始终按公开复测解释。
预期不同轨道排名不同、low offline loss与任务成功不一致；不预设所有策略成功。

## Actual Result / Explanation

2026-10-12 DESKTOP-781D67A。A沿用并重新审计S20.7历史产物，不是fresh rollout；B本机按原预算重建并公开复测；C/D冻结后首次预留test。

| 轨道 | 同任务test结果 |
|---|---|
| A | expert4/4，BC0/4，chunk E1/E5为0/4、1/4；ACT E1/E5均2/4，uniform3/4；单training seed |
| B | 两路线expert各2/2，mean0/2碰撞；BC与ACT E1均0/6，ACT E5为6/6；DDPM100/DDIM25均0/24 |
| C nominal | expert7/8，BC23/24，ACT E1/E5为6/24、7/24；Diff E1/E5为0/48、6/48 |
| D 五pulse合计 | expert15/15ready/success；三BC共0/45ready/success，均无pulse；BC226每pulse一条drop |

C专家reset22401、direction−1在0.6s接触截止触发failure latch；最后tail已达标仍失败。不能看到test后延长截止或删除这一reset。
C的loss与overforce任务success全部0；loss的expert/BC/ACT E1/E5分别恢复8/8、24/24、24/24、24/24，但不清除锁存。Diff E5 loss observed43/48、recovered31/48；overforce observed43/48、recovered12/48。
C Diff E1几乎未建立接触，不能只按planned pulse判断实际故障已发生；event_observed单独记录。
C validation current-slot RMSE：BC .357–.380mm、ACT .550–.626mm、Diff .855–.899mm。它们不是同一loss，也不能替代test闭环。

冻结C权重将采样流224/225改为226/227后，Diff E1 nominal为1/48、E5为13/48；E5 loss恢复34/48、overforce恢复12/48，任务仍全0。保留两组全部结果，不挑较好stream，变化不证明算法改善。
240条确定性C与60条D轨迹/模型输出配对不变；计时和未使用的sampling seed元数据另计。
S22.7物理域和S22.8噪声/延迟仍是validation诊断，不纳入新test或跨任务排名。

## Failure Cases / Robotics Context

- 同名ACT/Diffusion不表示同task、同数据或同预算；B成功不能直接移作D抗扰证据。
- pooled平均受分母和场景比例影响；小样本排名改变不等于算法普遍优劣。
- 只报告成功误差会掩盖所有失败；只报告末态会掩盖collision、超力与drop。
- 同一模型多个Diffusion采样流不等于多个独立训练seed；不能挑最成功stream。
- 看到test后再换checkpoint/E/scorer就不能继续声称独立test。
- 推理时间必须标明normalize/infer/decode、warmup、batch与physics/I/O范围。A历史/B专门计时/C实际规划/Dpolicy调用范围不同，不做统一实时排名。

应用：机器人学习项目交付时建立可复查的实验合同、覆盖矩阵、全分母统计与失败分析，决定下一步应修数据、控制、观测还是模型。
这是一套CPU state-based教学证据，不是通用抓放、真实视觉、硬件安全或实时部署认证。

## P3 Reasoning Summary / Interview Capsule

主线：因果示范记录 → 按episode/reset切分 → train-only normalization → BC → 闭环分布偏移 → chunk/ACT → 条件Diffusion → contact反馈 → 物理域和传感链诊断 → 冻结benchmark。
学到的核心能力是解释“训练目标、策略输入、动作执行、物理成功”之间的区别，并用可重建的轨迹验证。

30秒：冻结模型与数据，在同任务执行合同下比较闭环结果和成本，按轨道保留所有attempt；test曝光、缺少模型、失败与恢复都显式报告。

2分钟：先列任务/单位/obs/action/控制/scorer，再列split、normalizer、training与sampling seeds、E和采样预算；说明offline只能评价专家分布。
展示至少一条末态恢复但全过程失败的轨迹、一条未ready无pulse的轨迹。解释成功条件统计的选择偏差、跨任务平均的分母问题与模型覆盖缺口。
最后给出能复查的SHA/协议/数据和真实部署未验证范围。

## Must Remember / Run → Modify → Explain

必须记住：同任务合同；全部attempt；锁存失败；train-only统计；先冻结后test；缺失不等于失败；不跨task排算法总名次。

Run：本机C/D test已在工程运行中暴露，使用冻结模型并加`--repeat`；执行入口并查看四轨道contracts、summary.csv、benchmark.png和C/D具体失败轨迹，识别每行分母与test暴露状态。

Modify：冻结同B results、C cohort与D cohort，先预测换Diffusion采样流后哪些轨迹应不变，再做公开复测，不能按结果挑更好的流：

```bash
python examples/23_contact_aware_learning/final_benchmark.py \
  --b-results tmp/s22_9/my_run/B \
  --c-models tmp/s22_9/my_run/C/cohort \
  --sample-seed 226 --repeat --output tmp/s22_9/my_stream226
```

B本次作为固定证据复用；C Diffusion改流，其余C与D的物理轨迹和模型输出应逐值不变（wall推理计时、未使用的采样seed元数据另计）。若首Run复用了外部B/C目录，相应替换路径。

Explain：
1. 四轨道哪些合同必须固定，哪些预算可以不同？为什么缺少A Diffusion/D ACT不能记成0%？
2. 为什么C Diffusion分母48、ACT分母24？这些attempt是否都是独立样本？
3. offline误差、全分母误差与成功条件误差有什么区别？为什么success-only没有样本时不能写0？
4. C已经恢复负载、B末态已到达、D未drop，为什么都仍可能任务失败？
5. C/D第一次预留test评价与后续Modify有何区别？为什么轨道排名/低延迟不证明通用算法优越或硬实时？

My Verification：本人确认实验与预测完成，并准确解释五项核心问题；Run/Modify/Explain全部完成，Engineering Complete + Learning Mastered（2026-10-12），状态以根README为准。P3后续复盘或新任务等待本人选择，不自动进入P4。

## Engineering Verification（2026-10-12，DESKTOP-781D67A）

WSL2 Ubuntu24.04.5/kernel6.6.87.2，mujoco环境/所属Python3.12.14；MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2、Torch2.14.0+cpu版本/import/module paths核验，实际API运行通过，无安装。
本次明确使用独立生成B结果；C/D worker由入口调度，首次汇总字段adapter和计时变量覆盖修正后，使用`--reuse-rollouts`从已完成的轨迹重建报告，不修改评分、原协议或模型。

```bash
python examples/22_diffusion_policy/policy_benchmark.py --output tmp/s22_9/B
python examples/23_contact_aware_learning/final_benchmark.py --b-results tmp/s22_9/B --output tmp/s22_9/final
python examples/23_contact_aware_learning/final_benchmark.py --reuse-rollouts --b-results tmp/s22_9/B --output tmp/s22_9/final
python tmp/s22_9/verify_ab.py
python tmp/s22_9/verify_final.py tmp/s22_9/final
python tmp/s22_9/verify_final.py tmp/s22_9/modify
python tmp/s22_9/verify_d_scores.py
python tmp/s22_9/verify_pairs.py
```

checker仅本机ignored临时独立审计，不是共享运行入口。第二命令初次在汇总失败，C/D评价完整完成；修正后的第三命令成功，不把中间失败标PASS。
Modify采用内部C/D worker分开执行，再审计重建报告，效果对应上面的冻结采样流练习；未重新训练或改E/scenario阈值。
A64/B144轨迹560000 Euler步独立重建；C train-only normalizer/episode边界/末端mask检查通过。
默认与Modify各1824588 physics行、91788因果观测、576首末DDIM重建、5条保存参考/载荷物理重积分通过，全部独立审计exit0。两组合计3649176 physics行、183576观测、1152 DDIM重建和10条重积分。
两组D120条全过程failure/ready/pulse/tail独立重算、300条确定性配对、所有报告计数/reference step RMS通过。
已测试已有output、负sample seed、改流未冻结三CLI拒绝（exit2）；Python3.11语法通过，实际3.12。两最终PNG已查看，547个本地文件链接与git diff --check通过。
A历史/B公共/C-D新test证据各自保留；报告重建版本与原worker版本各自记录SHA。成本范围不同，部分执行并行，不作为跨轨道计时排名或硬实时证明。
无GUI/真人/真实传感器、硬件或未训练算法迁移验证。Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）。

学习核验覆盖固定同轨道合同、未评价与0%区别、训练seed/reset/采样流的相关性、三类误差与成功选择效应、全过程failure latch及test曝光边界。计数补充：C每scenario的ACT为3训练seed×4reset×2方向=24，Diffusion再乘2采样流=48。P3课程工程与学习完成；现有模型覆盖和部署验证范围不因此扩大。
本次仅同步七份文档，核验本地链接与git diff --check；运行证据沿用2026-10-12工程验证，未新增Python或GUI运行。
