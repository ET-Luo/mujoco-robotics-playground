# S19.5 — Closed-Loop Rollout / Offline vs Rollout

## Problem → Why → Intuition

问题：一个在expert数据上预测动作较准的BC，独立控制实际MuJoCo时是否完成任务？
S19.4已完成训练/validation模型选择。本课冻结该checkpoint和train normalizer，第一次使用独立test集合评价。
不训练、不refit、不根据test挑模型，也不修改成功门限。主概念是**预测误差与闭环任务结果是不同测量**。
直觉：offline每次给模型“专家走到的位置”；closed-loop每次给模型“它自己走到的位置”。

## Core Concepts

| 测量 | 输入分布 / 输出意义 / unit |
| --- | --- |
| offline action RMSE | test expert observations上预测reference与记录expert action差异；m，平均rows/XY两轴 |
| normalized MSE | 同一差异除train action_scale后平方平均；无量纲，channel加权与m单位不同 |
| rollout final position error | 实际MuJoCo末帧工具到goal的world XY欧氏距离；m |
| rollout success | 末10观测全部distance<1mm且speed<2mm/s；episode-level判据 |
| expert disagreement on rollout | 在policy实际状态上计算专家reference与实际policy动作差异；m；回顾性诊断，不是训练数据 |
| attempted / successful | 每policy全部4个test reset为分母；仅成功条件误差另外报告，0成功用null而非0 |

test包含4个独立episode/400个expert动作样本，从S19.3固定IDs取reset与goal。
checkpoint是S19.4由validation选出的epoch79，train seed194；不比较多个checkpoint后选test最好者。
expert、constant、BC各用fresh MjData，相同初态/goal、action语义、50Hz/1kHz、motor cap与scorer。
source是classical expert仿真truth state；当前没有视觉、接触、物体抓取或真实sensor。

## Mathematics → Math-to-Code

Offline：L=mean_{test rows,j}[(πθ(o)_j−a*_j)²]，RMSE=√L（m）。
这里π包含train normalization与action反变换；不能把normalized网络输出直接当位置。
Closed-loop：a_t=πθ(o_t)，s_(t+1)=F_MuJoCo(s_t,a_t)，o_(t+1)=observe(s_(t+1),goal)。
每a_t是world XY绝对reference，物理F=K(a_t−qpos)−D*qvel，再ctrl=F/gear；motor广义力限幅±20N。
reach success=∀t∈末10帧：||x_t−g||₂<.001m AND ||v_t||₂<.002m/s。
Offline是各坐标RMSE，reach是位置向量norm；二者的量纲虽同为m，却不是同一个量，不可互换阈值。

`load_evaluation`读weights-only CPU checkpoint，核验split hash/task/horizon与normalizer逐字段一致。
只读normalizer/test，检查test file hash、row IDs、timestep顺序/shape/finite/归一化。
`offline_prediction`分别计算expert/constant/BC在固定test rows上的动作；expert对原始标签误差0是数据合同检查。
`bc.rollout`每tick读取actual observation再生成动作，复用S19.4可见MuJoCo循环，没有replay expert观测。
`MjModel.from_xml_string(XML)`返回编译模型，`MjData(model)`返回独立可变状态；
`mj_forward`原地计算、time不推进；`mj_step`在env.advance内原地积分1ms，每policy action执行20步。
只有reset赋qpos/qvel；执行中不修改实际状态去贴expert trajectory。
退化constant使用train raw action mean；不加入额外控制裁剪或改模型，以免比较协议不一致。
评价末尾核验checkpoint SHA未改变；程序无optimizer/backward/step/refit。

## Minimal Experiment

依赖requirements已有torch==2.14.0+cpu/numpy/mujoco/matplotlib；helper链
behavior_cloning→episode_contract→cartesian_spring同依赖，无新安装或ROS。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/evaluate_rollout.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt
```

使用你的S19.3 prepared与同一次S19.4生成的policy.pt；两者split hash必须匹配。
其他电脑没有ignored产物时按S19.2→S19.3(seed193)→S19.4默认流程重建，并替换目录；
新机器得到的训练结果需现场记录，不能保证与本机数字精确一致。
输出新ignored tmp/s19_5/run_*，含results.json、offline_predictions.npz、12个rollout NPZ和rollout.png。
NPZ observation(T+1,6)、action(T,2)、time(T+1)、goal_error/speed(T+1)、expert_disagreement(T,2)。
图显示4个test episode的实际goal error随simulation time变化，1mm线只代表位置门限，仍须speed判据。
本课固定4test reset的图布局；不是大规模benchmark工具。

## Expected / Actual Result

2026-10-11 DESKTOP-781D67A / WSL2 Ubuntu24.04.5 / kernel6.6.87.2；mujoco环境所属Python3.12.14，
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2/Torch2.14.0+cpu metadata/import/API/helper路径核验。
Torch CUDA build None/tensor CPU；无安装、依赖改动、模型重训或normalizer refit。
现场命令：

```bash
python examples/20_learning_from_demonstrations/evaluate_rollout.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --output tmp/s19_5/default
python examples/20_learning_from_demonstrations/evaluate_rollout.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --duration-scale 2 --output tmp/s19_5/long
```

| 指标 | expert | constant | BC |
| --- | ---: | ---: | ---: |
| Test offline action RMSE | 0 | 9.585531mm | 1.532095mm |
| Test normalized MSE | 0 | 1.159639 | .0309306 |
| 2s task success | 4/4 | 0/4 | 0/4 |
| 2s mean final position error（全分母） | .000008568mm | 11.873576mm | 11.909423mm |
| 2s max tail position error | .000325126mm | 20.575873mm | 17.387335mm |
| 2s max tail speed | .00409241mm/s | 约0 | 1.922612mm/s |
| 4s task success | 4/4 | 0/4 | 0/4 |
| 4s mean final position error | 约0 | 11.873576mm | 11.999548mm |
| 4s max tail position error | 约0 | 20.575873mm | 17.731416mm |
| 4s max tail speed | 约0 | 约0 | .114500mm/s |

两命令exit0表示evaluation工程检查通过，BC任务success保持0/4。
2s每命令4reset×3policy×2000physics步=24000步；4s为48000步，全部attempt保存。
两个条件offline输入始终原test100tick，所以offline指标相同；延长执行并不增加离线示范样本。
Modify只改horizon100→200，三个policy都延长；同seed前100action/101obs完全一致。
验证：保存后24条rollout独立Euler重算、因果policy/action、reset/schema/clock/scorer/aggregate/hash检查通过；
共同前缀与checkpoint未改变通过；CLI duration0/nan、已有output、split不匹配、损坏test拒绝。
Python3.11语法解析通过（未实际3.11执行）；两rollout.png已查看，links/anchors/git diff --check通过。
每个条件内部task/split/metrics相同，但2s/4s是不同duration协议，不能把两组当作同horizon算法排名。
criterion仍为末10帧，所以评价窗从1.82…2s移到3.82…4s，不扩大位置/速度容差。

## Explanation → Failure Cases

BC离线明显优于constant，闭环平均最终位置误差却未明显优于constant；不能从action RMSE推出任务成功。
在本例4个BC test trial中，2s/4s速度门限通过、位置门限未通过；仅“几乎静止”不能说明到达goal。
4s速度下降而位置误差仍约12mm，说明此次延时未恢复成功；可疑原因包括goal附近policy bias、
访问状态与expert示范不同、动态输入误差。没有干预/扰动/训练覆盖比较，不能断言每个原因的贡献或全局稳定性。
实际state上的expert disagreement只是诊断，不是DAgger、恢复数据采集，也不是分布shift的完整因果证明。

- 把离线expert observations逐帧喂给policy却称closed-loop会掩盖真实反馈效应。
- 用test重选checkpoint/split、修改成功阈值会让本次test结果失去独立评价意义。
- 当0成功时只报告success-conditioned error=0会误导；本例明确用null。
- constant很慢/最终speed≈0仍可能目标错误；成功同时需要位置与速度。
- 4个固定test episode只覆盖当前分布，不证明OOD/friction/noise/latency鲁棒性。
- test本轮已查看；后续不能假装它仍是从未见过的最终盲测。保持此固定benchmark用于公开比较，
  若需要新的最终独立证据，应在调参前预先定义fresh evaluation seeds/协议，另行授权实施。

## Robotics Context / Interview Capsule

应用：模仿学习部署验收、paired controller-policy评价、经典expert与learned policy比较。
30秒：offline loss测expert-state动作拟合，rollout测policy-induced state上的物理任务表现；所有attempt统计同一scorer。
2分钟：解释冻结checkpoint/normalizer与paired resets，逐tick实际观测→动作→MuJoCo，位置/速度联合判据，
用1.532mm offline/0of4 closed-loop说明指标差异；最后解释延时对照和test已使用的边界。
Must Remember：模拟真实执行；相同task/split/metrics；loss下降不是success；观察到失败不等于已完成因果归因。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。2026-10-11 本人确认实验与预测完成，并准确回答五项Explain；Run / Modify / Explain全部完成，Learning Mastered。
Run：执行默认命令，核对offline指标、全部attempt成功率、位置与速度通过标记，查看rollout.png。
Modify：先预测只把duration翻倍是否一定改善位置成功，再运行：

```bash
python examples/20_learning_from_demonstrations/evaluate_rollout.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --duration-scale 2
```

Explain（五问）：
1. Offline动作RMSE和closed-loop位置误差各测什么，为什么不能直接互换？
2. 为什么expert/constant/BC需要相同reset、goal、horizon、inner controller与成功判据？
3. 如何确认程序实际闭环执行，而非replay示范或把reference直接赋qpos？
4. BC接近静止、位置未达标，延长到4s仍失败，能得出什么结论，哪些原因尚未证实？
5. test已经评价后，为什么不能用它继续选模型却仍宣称独立泛化验证？

下一小任务S19.6 covariate shift / compounding error，等待明确请求；本轮停止。

本人Explain准确区分expert-state动作拟合与真实闭环任务误差、paired条件、公平scorer、action replay和直接qpos赋值。
结论边界：当前四reset延时2→4s仍失败，只说明该延时对照未满足判据，不证明更长时间必然失败或已找到根因。
首次冻结模型/normalizer/协议后test评价仍是独立评估；之后若依据其结果调参，则需新的独立评价集。
Learning Mastered不改变BC任务0/4。本次仅同步文档，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-11工程验证。
