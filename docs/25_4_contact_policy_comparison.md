# S22.4 — Contact ACT / Diffusion comparison and recovery

[状态唯一来源](../README.md#stage-22--contact-aware-robot-learning) · [比较代码](../examples/23_contact_aware_learning/contact_policy_comparison.py) · [序列模型](../examples/23_contact_aware_learning/contact_sequence.py)
前置S22.3；0.5～2小时。Engineering Complete（2026-10-12）；Learning Run/Modify/Explain待本人完成。

## Problem → Why → Intuition

S22.3表明增加接触通道不保证提升BC成功率。本节在同一接触任务上重训BC、ACT和Diffusion，比较单步反馈与chunk执行。
直觉：一口气计划数步可以保持序列一致性，但扰动发生后，旧计划可能不再合适；更频繁地生成随机计划也不必然更稳定。
接触恢复只说明重新建立某种负载，不说明目标位置已到达，更不能撤销历史超力或接触丢失。

复用S22.3的球/平面、反馈专家、分组数据、10维观测、delta动作、position servo和原success scorer。
全部为controller expert，真人0。没有加载B轨道旧权重；三种模型都在C接触轨道重新训练。未进入S22.5三指任务。

## Core Concepts / common contract

| 项目 | 固定定义 |
| --- | --- |
| task / schemas | tip_surface_follow_v1 / tip_force_goal_observation_v3 / actual_q_anchored_reference_delta_v1 |
| observation | (10,)，q(2)[m]、qdot(2)[m/s]、x_goal[m]、F_target[N]、world F(3)[N]、touch[N]；history1；全部模型用full |
| action | (2,) [m]，相对执行时actual q的位置reference偏移；不是实际Δq、torque或接触力命令 |
| execution | delta先限±[6,8]mm，再把q+delta投影到reference box；ctrl为absolute reference[m]，position servo限力±10N |
| clocks / horizon | 50Hz policy、1kHz physics；150tick=3sim秒，151observations、3001physics snapshots |
| dataset | 16train/4validation reset groups，正负方向同组；32/8episode，4800/1200 transition-start pairs；40专家全部成功 |
| normalization | 同一train-only mean/std，常量scale=1；不拟合val/test，不将terminal当action-label起点 |
| test | 4reset seeds仅保留，未采集或评价；只有validation作为本节实验范围 |
| reproducibility | training seeds223/224/225；Diffusion sampling streams224/225，独立命名空间、各anchor重新取Gaussian |

reference box限制目标，不限制actual q。actual q在扰动后可能出界；box投影后的实际reference偏移也可能超过先前的delta cap，clipped_ticks如实统计。
不能把上述边界当硬关节限位或硬件安全保证。force仍是仿真具名pair真值proxy；touch为pad法向标量，并非真实触觉图像。

## Mathematics / H, E, K and masks

```text
H = 10 future action slots
E = 1 or 5 executed slots before discarding the suffix and replanning
K = 100 training diffusion indices; eta0 DDIM uses 25 model calls
r_(t+j) = box_clip(q_actual_(t+j) + delta_clip(plan[j]))
L_ACT = masked_mean((prediction-normalized_target)^2) + .01 * KL(q(z|o,A) || N(0,I))
L_diff = masked_mean((epsilon_hat-epsilon)^2)
a_k = sqrt(alpha_bar_k)*a_0 + sqrt(1-alpha_bar_k)*epsilon
x0_hat = (a_k - sqrt(1-alpha_bar_k)*epsilon_hat)/sqrt(alpha_bar_k)
a_s = sqrt(alpha_bar_s)*x0_hat + sqrt(1-alpha_bar_s)*epsilon_hat
```

DDIM最后式使用原训练索引k→较小s，不把25次调用冒充训练K=25；最后s=0且alpha_bar_0=1。
eta=0只去掉每个reverse step的新噪声，初始Gaussian仍随机。未裁剪生成的normalized action；仅共同执行层限制reference。
E5执行plan[0:5]，每个slot都锚定到**当时actual q**，不会累加delta成虚构q轨迹，也不把所有槽固定锚定到预测时q。
网络只在replan看新的10维观测；当前q仍每tick用于reference换算，position servo/physics仍1kHz，E5不等于低层反馈停止。

按episode连续构造H10 windows；train4800window，有效46560槽/padding1440槽；val1200window，有效11640/padding360。
loss只计有效slot×2通道，不能把padding当真实action。生成mask只用固定剩余episode预算，不读取未来success。
ACT的无效future output不用于执行；Diffusion每步把无效normalized槽置0；raw padding不作为物理有效预测，须结合planning_valid读取。

## Math-to-Code / model and budget

| 模型 | 本节实现 / 参数 | 训练预算与选模 |
| --- | --- | --- |
| BC full | 两层32宽Tanh、当前delta；1474 | 100epoch/batch128/Adam .001，最低val normalized action MSE |
| contact ACT | 复用S20.5真实TinyACT，obs projection从6→10；D32/2heads/latent4、三组单层Transformer、H10；31274 | 60epoch/batch128/Adam .001，训练CVAE，最低val zero-prior masked action MSE |
| contact Diffusion | joint H10 masked MLP，41→128→128→20、SiLU；24468 | 150epoch/batch128/Adam .001，最低fixed-val masked epsilon MSE |

CPU1thread；同seed各模型用独立generator维持相同batch行顺序，epoch数、参数量、架构和目标不同。
这是这些既有学习实现的行为/成本比较，不是等预算架构因果胜负。
ACT posterior在训练允许读取未来action标签，部署只调用infer(o)，固定prior z=0；Planner将posterior替换为抛错函数，防止监督泄漏。
Diffusion日程linear beta=.0001… .1，K100，部署固定DDIM25；sampling seed=stream*100000000+reset*1000+direction_offset+anchor。
不同stream不会复用另一stream的下一tick Gaussian；同模式跨scenario及E1/E5共同anchor的初始噪声可匹配，但轨迹状态会不同。

`prepare_cohort`调用S22.3 bc.prepare现场采集同C数据，分别训练九个模型；没有根据closed-loop结果挑模型/seed或调整预算。
`offline`对同1200个expert validation状态计算共同slot0 raw delta RMSE[mm]；Diffusion每状态两draw。
三种selection loss有不同含义，不能直接比较数值；共同RMSE仍是标签一致性，不是完整closed-loop质量指标。
`Planner`记录actual planning observations、chunks、mask、seed、anchor与执行offset，BC每tick预测一次，ACT/Diffusion各150或30次/episode。
`rollout`保持官方MuJoCo接口可见：build_model编译MjModel，MjData创建可变状态；reset才写qpos/qvel，后续ctrl→mj_forward刷新→mj_step积分。
每physics步读取world contact force/touch、实际actuator/passive/qacc，核对0.1kg*qacc=actuator+passive+contact。
physics新增第16列为真实使用的surface height，便于独立回放扰动。policy observation不含该隐藏参数。

模型checkpoint保持原SHA及训练source记录；loader检查所有model/task/training helper的source hash，评价driver允许独立修订精度，当前driver hash写protocol，不重写训练 provenance。
训练采集solver tolerance1e−12/iterations100；最终所有评价统一1e−14/iterations200，物理模型/动作/评分不变，原因见后文。
依赖已有mujoco/numpy/matplotlib/torch==2.14.0+cpu，无新包；helpers含contact_bc/task/observation、TinyACT的attention/CVAE/window/BC/episode/cartesian_spring导入链及noise_schedule。
旧Stage20/21 main与旧rollout不运行，旧课程源码不改。

## Disturbance → recovery → success

1s开始临时修改plane geometry，默认持续0.2s，然后恢复原高度：loss向下15mm，overforce向上30mm。
这是合成geometry jump，不是具有真实速度/惯量/能量预算的移动表面或硬件施力。训练数据不含这些扰动。
先用expert验证实际事件；不能仅凭scenario名称宣布发生超力。更改几何先于当前tick观测，因此当前replan可感知扰动，仍在执行旧chunk的slot则不能重新规划。

沿用S22.3评分：任意Fz>5N，或0.6s起Fz<.05N，历史失败锁存；末250ms每点x error<1mm/force error<.25N/speed<2mm/s；成功=无锁存 AND tail稳定。
失败继续运行至3s用于分析，没有安全停止controller，也没有自动expert接管或隐式救援。

**Observed event：** loss必须扰动前一physics sample仍Fz≥.05N，且pulse期间出现Fz<.05N；overforce必须先前Fz≤5N，且pulse期间实际超过5N。
已经没有接触的trial不假装发生了“接触丢失”。已有失败仍保留全分母，首次失败可能早于1s。
**Load recovered：** event确实observed，且restore后Fz在[.05,5]N、|Fz−target|<.25N连续101sample，跨越100ms。
报告确认时间=连续窗口末端time−restore time，而非窗口开始时间；没有有效event或没有恢复时为None。
负载恢复不检查x位置/速度，tail_reached另检查；二者都不能清除failure latch。nominal的event/recovery为None，不能把汇总0解释为恢复失败。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/contact_policy_comparison.py --output tmp/s22_4/my_run
```

output须为新目录；默认重新采集/训练九模型，评价三scenario。输出data/window、cohort/checkpoint/loss、共同offline预测、protocol、528trial NPZ、results.json与comparison.png，只ignored tmp。
若已有本节cohort，可加`--models-from <原run目录>/cohort`省去训练；validation数据从该cohort父目录data读取并检查SHA。
预期nominal专家可行，扰动可触发锁存；策略成功、负载恢复和tail表现均不预设。
每scenario BC/ACT分母3training×4reset×2direction=24，Diff另2streams=48；expert为8。
528trial并不代表528个独立环境，只来自4个validation reset groups及相关重复，不给出普遍算法可靠性结论。

## Actual Result / Explanation

2026-10-12 Zero；WSL2 Ubuntu24.04.5/kernel6.18.33.2；mujoco所属Python3.12.14；
MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2/Torch2.14.0+cpu metadata、真实module paths/API核验通过，无安装。
现场完整准备：同脚本`--output tmp/s22_4/final`；之后保持该cohort原SHA，最终严格数值评价：

```bash
python examples/23_contact_aware_learning/contact_policy_comparison.py \
  --models-from tmp/s22_4/final/cohort --output tmp/s22_4/verified
python examples/23_contact_aware_learning/contact_policy_comparison.py \
  --models-from tmp/s22_4/final/cohort --pulse-duration .1 --output tmp/s22_4/short_verified
```

两组最终命令exit0，以下只引用verified/short_verified。test未使用。

| 模式 | nominal成功 | loss observed / load recovered | overforce observed / load recovered |
| --- | --- | --- | --- |
| expert | 8/8 | 8/8 / 8/8 | 8/8 / 0/8 |
| BC E1 | 23/24 | 24/24 / 24/24 | 24/24 / 21/24 |
| ACT E1 | 9/24 | 24/24 / 24/24 | 24/24 / 24/24 |
| ACT E5 | 9/24 | 24/24 / 24/24 | 24/24 / 24/24 |
| Diffusion E1 | 0/48 | 1/48 / 0/48 | 2/48 / 0/48 |
| Diffusion E5 | 16/48 | 45/48 / 37/48 | 45/48 / 17/48 |

两种fault scenario所有模式任务success均0，虽然许多trial恢复负载。BC loss tail18/24；ACT E1/E5 loss tail0/24、4/24；Diff E5 loss tail3/48，恢复与稳定到达有明确差别。
nominal ACT E1/E5均无接触锁存，但末段位置/速度/部分force条件失败；Diff E1有47/48在0.6s未ready锁存，全部末段speed失败；E5有4/48在0.6s锁存。
这些是失败分类，不是对根因的因果定位；频繁采样与跨chunk一致性可能相关，未单独隔离证明。

专家在overforce默认3s窗口内未恢复。示例reset22350/−1：restore时qz=.000203m、final qz=−.027095m，球底world z=2.905mm而表面0.636mm，final Fz=0；尚未重新接触，不把nominal可行当扰动恢复保证。
30mm几何jump幅度较大，故障组结果不能外推到小扰动、真实运动表面或所有接触任务。

共同offline delta RMSE：BC .357–.380mm、ACT .550–.626mm、Diffusion .855–.899mm。
ACT best epochs3/3/4，Diffusion150/143/150；均为预声明offline选模，不挑closed-loop成功模型。
train loop BC1.427–1.719s、ACT19.593–21.950s、Diff4.837–5.398s/model；不含采集/保存/评价，预算不同不作架构因果胜负。
实际计划pipeline median/p95[ms]：BC .092/.259，ACT E1 .417/1.062、E5 .514/1.166，Diff E1 1.692/3.981、E5 1.889/4.267。
对应10800/10800/2160/21600/4320次实际调用，含首调用，无独立warmup剔除；不含physics、sensor refresh、I/O或网络。同步仿真在推理时暂停，不能证明硬实时。

Modify0.1s：nominal全部逐值不变；fault task success仍全0。Diff E5 loss负载恢复41/48（原37），tail5/48（原3）；Diff E1 loss1/48（原0），但此前已失败不撤销。
ACT E1 loss tail6/24（原0），E5为0/24（原4）；缩短扰动没有保证所有终点表现单调改善。权重、sampling profiles、评分阈值均未改。

## Verification / Failure Cases

初候选2mm抬面未实际超过5N，10/20mm也未在全部expert reset产生超力；按expert实际力检查校准30mm，最终expert8/8观测到超力。未根据learner胜负调幅度。
初0.1s评价在ACT223/reset22351/+1摩擦恢复瞬态，solver1e−12的Newton残差2.686e−7N，预算断言失败，未计PASS。
同checkpoint/reset/pulse，1e−14/200iterations残差降至2.13e−14N；没有放宽1e−8N预算断言，九模型checkpoint字节保持不变，两组全部重新严格评价通过。
恢复定义检查补上扰动前接触条件；sampling seed使用分离namespace，避免不同stream错位复用Gaussian。最终训练权重与初轮同seed逐值一致。

独立核验：分组/train-only统计、H10全部4800/1200window边界与mask、九模型最佳epoch、ACT padding NaN不变性通过。
两组1056trial/3169056physics snapshots时间/积分/Newton/geometry、全部99360plan anchor与槽/seed/mask/reference、success/recovery/per组汇总与latency重算通过。
对每模型/E/scenario的固定reset/方向、Diff stream224，共96trial独立stored-reference重积分，与原observer全151观测一致。
另外144个Diffusion plan逐25步用NumPy reverse式与独立MLP epsilon前向核对；全部共同offline RMSE独立重算，nominal跨Modify逐值一致。
checkpoint篡改/core helper source mismatch拒绝；非法CLI×2、empty mask与NaN padding、DDIM非相邻oracle和Python3.11语法通过（actual3.12）。最终PNG已查看。
local文件links/状态/git diff --check通过。无GUI/test、真人/实际传感器、mass/friction域扫描、noise/latency或硬件安全验证。

## Robotics Context / Interview Capsule / Must Remember

应用：表面跟随、插装与抓持的序列策略，需要区分几何目标、接触负载、恢复过程和全过程安全证据。
**30秒：** 同一接触任务上重训BC/ACT/Diffusion，共享观测、分组、normalizer、执行层与scorer。H10执行E1/E5，DiffusionDDIM25。正常BC23/24、ACT各9/24、Diff E1/E5为0/48与16/48；扰动即使负载恢复，历史失败仍保留。
**2分钟：** 解释actual-q锚定delta、CVAE训练posterior与部署z0的边界、masked diffusion与三个时间轴，说明预算与分母差异。再展示接触丢失前置条件、restore后连续100ms负载恢复、末段稳定和锁存success的区别。最后补充几何jump不等于真实移动表面，small reset样本/test未用与数值求解器预算的证据范围。
Must Remember：chunk的delta按执行时actual q换成reference；E不是K；恢复不清除失败；更多重规划不保证生成策略稳定；比较必须同轨道重训，不能用B旧权重。

## My Verification / Run → Modify → Explain

学习状态仅根README记录；助手执行不算本人Run。本人于2026-10-12确认实验、预测及以下五项Explain完成，Run/Modify/Explain全部完成，Learning Mastered。

精度补充：部署z=0是本实验的确定性prior选择，不代表所有ACT必须如此；冻结pulse时应比较轨迹/计划值，wall-clock latency不要求逐值一致。
**Run：** 执行my_run，核对九模型、nominal与两个fault组，查看comparison.png；在results中找一个load_recovered=True但success=False的trial。
**Modify：** 先预测哪些结果必然不变、哪些可能变化，再用冻结cohort只缩短pulse：

```bash
python examples/23_contact_aware_learning/contact_policy_comparison.py \
  --models-from tmp/s22_4/my_run/cohort --pulse-duration .1 \
  --output tmp/s22_4/my_short
```

比较observed_events、load_recovered、tail_reached、success，解释为什么更短扰动不自动撤销失败或让所有指标更好。
**Explain（五问）：**
1. H10、E1/E5、训练K100与DDIM25分别表示什么？每个future delta为何锚定执行时actual q？
2. ACT训练posterior为什么可以用future标签，部署为什么不能？Diffusion eta0为什么仍有随机性？
3. BC/ACT分母24、Diffusion48如何组成？哪些合同相同，哪些预算不同，为什么不证明普遍算法优劣？
4. observed event、连续100ms load recovery、tail_reached与latched success分别测什么？为什么恢复后仍失败？
5. 冻结权重缩短pulse时，哪些nominal数据应不变？为什么test未用、offline误差与inference p95都不能代替实际任务泛化/硬实时证明？

下一小任务S22.5三指free-object专家可行性，等待本人明确请求，不自动实施。
