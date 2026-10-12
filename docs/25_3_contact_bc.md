# S22.3 — Contact-aware BC / observation ablation

[状态唯一来源](../README.md#stage-22--contact-aware-robot-learning) · [训练代码](../examples/23_contact_aware_learning/contact_bc.py) · [任务代码](../examples/23_contact_aware_learning/contact_task.py)
前置S22.2；0.5～2小时。Engineering Complete（2026-10-12）；Learning Run/Modify已完成，Explain待补第①⑤项（2026-10-12 本人反馈）。

## Problem → Why → Intuition

接触通道提供额外信息，但是否帮助学到更可靠的动作，必须在同一任务里重训并闭环检验。
本节让球形指端建立平面接触、沿x移动到目标，并保持指定法向力；比较state、state+force、state+touch、full四种BC。
直觉：给驾驶者更多仪表，不保证他会更好地使用它们；离线模仿准确也不保证自己的动作造成的新状态仍可控。

S22.2仅有两条合格脚本轨迹，控制器未按接触反馈调节，不能直接当本节训练/validation数据。
本节保留旧文件，复用球/平面模型、位置执行器与force/touch定义，另采集feedback-controller expert。
输入来源不是真人遥操作，human demonstrations仍为0；未训练ACT/Diffusion、不进入S22.4。

## Core Concepts / frozen task contract

| 项目 | 定义 / shape / units / frame |
| --- | --- |
| task | `tip_surface_follow_v1`，两slide球指端、水平平面、无重力，摩擦系数0.5 |
| observation schema | `tip_force_goal_observation_v3`，(10,)，q(2)[m]、qdot(2)[m/s]、x_goal[m]、F_target[N]、F_W(3)[N]、touch[N] |
| ablation input | state=前6槽；state_force=前9槽；state_touch=前6+touch；full=全部，维度6/9/7/10 |
| action schema | `actual_q_anchored_reference_delta_v1`，(2,) [m]，相对**当前actual q**的位置reference偏移 |
| actuator command | r=q+bounded_delta，再做reference box限幅；ctrl=r[m]，position servo kp=200N/m，限力±10N |
| bounds | delta上限±[6,8]mm；reference x∈[−25,25]mm，z∈[−42,0]mm；不是实际关节硬限位 |
| clocks / budget | policy50Hz，每action保持20次1kHz物理步；150action/151obs，3simulation秒 |
| reset distribution | plane height∈[−3,3]mm，initial qx∈[−2,2]mm，qz∈[−27,−25]mm，qdot=0；F_target∈[.8,1.6]N，x_goal=±8mm |
| grouping | 同reset seed的±direction共享height/initial/force target，必须同split |
| train / validation / test | 16/4/4 reset groups；test只保留seed，不采集标签或运行test rollout |

qz是位移，球心world z=0.04+qz，10mm半径球约在qz=height−0.03接触。
目标第二槽从S22.2的米制task goal改为牛顿制F_target，action也从absolute reference改为delta；因此必须新schema，不能混用旧normalizer/标签。
force是环境作用于tip的world合力，来自仿真具名pair真值；touch是MuJoCo pad法向力标量，非压力、非触觉图像。
本水平单接触场景touch=Fz，二者部分冗余；Fx携带摩擦信息，Fy为常量0。
height、phase、teacher action、未来观测、reset seed均不进入policy。q/velocity可能已经提供接触模式的代理信息，不假设state-only必然失败。

## Mathematics / expert and BC

```text
error_x = x_goal - qx
if Fz < .05 N: delta_x = 0; delta_z = -.0003 m         # search, no tangent scan
else:
  delta_x = clip(.6*error_x - .05*vx + .5*Fz*tanh(error_x/.001)/200, ±.006)
  delta_z = (2*vz - F_target + .3*(Fz-F_target))/200
r = clip(q + clip(delta, ±[.006,.008]), lower, upper)
a_label = r - q                                       # executed, bounded delta [m]

z_o = (o - mean_train_o) / scale_train_o
z_a = (a_label - mean_train_a) / scale_train_a
L_BC = mean_(row,XZ) (MLP(z_o[mode_indices]) - z_a)^2
```

dx中的.6为无量纲位置增益，.05单位s、乘vx变成m；.5是摩擦系数，Fz/200由N换成m；tanh平滑正反方向。
dz中的2单位Ns/m，抵消slide damping；200单位N/m。
−F_target/200给位置servo施加向下预载；测到Fz过大时，.3(Fz−F_target)/200给向上修正，减少压入。
这是本学习模型的简单反馈专家，不是硬件通用力控制器。source=`feedback_controller_expert`，不使用隐藏height来计算action。

动作标签来自执行后保存的r−当前q，仍与**执行前o_t**配对，不把执行后force写回输入。
预测的delta不是实际q变化：还要经历执行器限力、质量、阻尼、摩擦和接触约束。
训练输出可越界；统一执行层显式限delta/ref，统计clipped_ticks。限reference不等于硬件安全或实际位置不越界。

## Math-to-Code / APIs / data

`contact_task.rollout`用S22.1 build_model编译MjModel，创建MjData；qpos/qvel仅在reset赋值。
position servo沿用原模型；本任务显式设置solver tolerance=1e−12，全部expert/BC/干预相同。
每个policy边界先`mj_forward`刷新当前state/contact/sensor，再读观测，生成delta并设置ctrl=r。
随后每physics步刷新当前命令下force、记录physics row并`mj_step`原地积分，不能用记录的qpos替代执行。
`mj_contactForce`向(6,)数组原地写contact-frame wrench；frame.T换到world，根据geom受力对象取符号。
每个tick边界的o_t读取上一保持ctrl；physics trace记录新命令下的force，命令变化时二者力可不同，但q/qdot/time相同。
所有151观测保留terminal，训练只取前150观测对应150动作。

`prepare`现场采集32train+8validation expert episode，所有attempt保存NPZ/manifest；仅successful expert用于BC标签，失败排除清单显式记录。
本次40/40 expert成功，无排除。train4800行、validation1200行；镜像方向按reset group隔离，不随机行split。
normalizer逐通道只拟合train transition-start rows的population mean/std，std<1e−8则scale=1；四模型共享同一完整normalizer再切输入槽。
val不参与统计。test seeds22400…22403只存协议；本节完全未生成test数据。
train reset22300…22315；validation22350…22353；training seeds223/224/225。

`train`：CPU1thread、两层32宽Tanh、输出2维，Adam lr=.001，100epoch/batch128，无早停。
每seed各模式使用相同训练行与由独立generator产生的相同batch顺序；不同输入维度导致初始化/参数数目不同，不宣称完全同容量因果实验。
checkpoint由最低validation normalized action MSE选择（含epoch0），不按closed-loop成功率选模或调参。
`restore`核对task/obs/action schema、source/version/hash、mode/seed，weights_only=True恢复权重；模型切eval，无posterior或未来标签输入。
默认data、loss CSV、12checkpoint/offline NPZ、cohort、protocol、104trial NPZ、results与PNG全部ignored tmp。
依赖requirements已有NumPy/MuJoCo/Matplotlib/torch==2.14.0+cpu；helpers仅contact_task→observation_contract。没有新包、ROS、RL或键盘采集。

## Full-trajectory scorer

每条trial含3001个1kHz采样点，记录time、q/qvel、force/touch、实际actuator广义力、qacc、passive阻尼力。
核对world Fx/Fz与两slide Newton预算：0.1kg*qacc=actuator+passive+contact。
积分关系/时间、touch=Fz、动作与held_ctrl均核验。

**全过程锁存失败：** 任意时刻Fz>5N；或t≥0.6s仍Fz<.05N（未ready或随后接触丢失）。
锁存只用于评分，机器人仍运行至3s，不把它冒充物理制动或安全停止controller。
**末250ms稳定到达：** 每个physics观测|x−goal|<1mm、|Fz−F_target|<.25N、速度模长<2mm/s。
成功要求无锁存失败 AND 末段全部通过。末帧恢复不能抹掉早先失败。
整个扫描期间的force误差不要求始终<.25N；该精度只用于末段，过程力要求接触且≤5N，不能误读成全程精确跟力。
finite异常与不合法action会报错中止，不把失败数值检查计PASS。没有硬件/连续时间或步间无限带宽保证。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/contact_bc.py --output tmp/s22_3/my_run
```

输出目录须不存在。预期先验证expert可行性，再按共同协议训练四种输入配置；BC结果不预设，offline loss与rollout分别看。
四模式每个分母3training seed×4reset×2direction=24；expert4×2=8。
24不是24个独立环境：只有4reset groups，方向与训练重复相关，不能把小样本成功比例当普遍可靠性。

## Actual Result / Explanation

2026-10-12 Zero；WSL2 Ubuntu24.04.5/kernel6.18.33.2；mujoco所属Python3.12.14；
MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2/Torch2.14.0+cpu metadata、真实import paths/API核验通过，无安装。
最终命令：上述脚本`--output tmp/s22_3/final`，然后冻结cohort执行
`--models-from tmp/s22_3/final/cohort --contact-input mean --output tmp/s22_3/mean_final`，均exit0。

| 模式 | 输入维 / 参数数 | offline delta RMSE(mm)，3seed范围 | measured成功 | frozen mean-input成功 |
| --- | --- | --- | --- | --- |
| expert | 明确反馈规则 | 不定义 | 8/8 | 8/8 |
| state | 6 / 1346 | .365–.396 | 24/24 | 24/24 |
| state_force | 9 / 1442 | .353–.363 | 20/24 | 13/24 |
| state_touch | 7 / 1378 | .349–.357 | 24/24 | 11/24 |
| full | 10 / 1474 | .357–.380 | 23/24 | 12/24 |

measured各seed成功state=8/8/8，force=7/5/8，touch=8/8/8，full=8/7/8（每seed8trial）。
mean-input为state=8/8/8，force=4/5/4，touch=7/0/4，full=4/4/4。
train loop每模型1.472–1.721wall秒，不含采集、serialization与rollout；未测完整pipeline实时latency。
measured BC峰force最大2.947803N；两组全部trial均无实际contact-loss/overforce锁存。
默认五次失败都有末段x误差>1mm，其中两次也速度>2mm/s；force末段误差都<.25N，不能把失败统称为失去接触。
state_force某失败x误差3.384mm，full某失败2.572mm/速度5.512mm/s；其根因未做因果定位。

**两种ablation的解释不同：**
四模式比较是移除通道并重训；mean-input是同一checkpoint在推理时把接触槽换成对应train mean（标准化后0），真正force/scorer仍读取actual simulation。
mean-input改变了训练时的通道关系，可能造成输入分布偏移；下降证明这些模型依赖接触通道，不能证明加入通道优于重新训练的state模型。
state-only可利用q/qdot和F_target生成预载反馈，本简单水平面任务不要求它必然失败。
touch与Fz冗余、小reset分布和优化差异都限制结论；不能宣称接触信息没有价值或touch普遍胜过force。
offline RMSE衡量expert状态上action-label一致性，不评价policy自己造成的新状态，不能代替全程scorer。

## Verification / failure-analysis draft

先用过小dx cap的专家抽查，大载荷下x停滞；训练前补摩擦项与接触gate，随后40条expert全部成功。
初轮measured完成，但mean输入干预在一条摩擦瞬态的Newton断言失败：默认solver tolerance1e−8残差约1.681e−5N，contact→广义力映射残差0。
同reset/checkpoint/干预重现并把solver容差改1e−12后，残差降至约4.44e−16N；统一精度后重新采集、训练与两组评估。
未放宽1e−8N预算断言；初轮失败的mean实验不计PASS，最终结果只引用final/mean_final。

独立核验：40条expert来源/分组/标签对齐；train-only统计；12checkpoint最佳val loss、offline NumPy MLP/指标重算通过。
两组208trial的624208physics snapshots时间/积分/Newton、全部31200policy输出与reference限幅、scorer与aggregate/per-seed分母独立重算通过。
另选固定reset/direction，对每模型及expert共26trial用原S22.1 observer与保存reference重新积分，全部151观测一致。
冻结state/expert对应轨迹逐值不变；人为早期6N峰与0N接触丢失均保持失败，即使terminal tail合格。
checkpoint SHA篡改拒绝、两个非法CLI与nonfinite action拒绝、Python3.11语法检查通过（actual3.12）；默认PNG已查看。
本地文件链接/状态/git diff --check通过。未读test、无GUI/真实传感器、OOD mass/friction/noise/latency或硬件安全验证。

## Robotics Context / Interview Capsule / Must Remember

应用：插装、表面接触和抓持的策略输入设计；先检查通道物理定义和部署可得性，再谈网络表现。
更多传感器可能提高可辨识性，也会带来冗余、噪声、标定和分布偏移；本节理想仿真force并不等于现实力计。

**30秒：** 我在同surface task上按reset group隔离数据，train-only归一化，用四种输入重训BC，三个training seeds共享预算和全过程scorer。state/touch均24/24，force20/24，full23/24；增加通道没有保证更高成功率。

**2分钟：** 解释目标force[N]和q锚定reference delta[m]，如何通过position actuator执行；说明expert只用可观测接触反馈、test未使用、1kHz接触/超力锁存与末段稳定条件。区分重训消融和冻结mean-input干预，后者含接触模型降到13/11/12，但不能据此宣布比state-only更好。补充通道冗余、4reset groups的小样本限制、offline loss与closed-loop的区别，以及求解器预算错误如何复现和统一精度修正。

Must Remember：schema包含物理语义；split隔离共享reset；mean-input干预≠移除后重训；低offline loss≠稳定闭环；全分母保留；test未用≠已证明test泛化。

## My Verification / Run → Modify → Explain

状态仅根README记录；2026-10-12 本人确认实验与预测完成，Run/Modify已记录。
Explain第②③④项通过，第①⑤项仍需本人给出本节具体定义/判据，不以“需核对实现”替代回答。
补充：goal第二槽为F_target[N]；delta是相对actual q的reference偏移[m]，ctrl为限幅后absolute reference[m]，actual Δq由物理积分产生。
normalizer使用4800条successful train expert的transition-start rows，排除terminal/val/test。
score无额外“扫描完成”状态：无全过程锁存失败且末250ms所有采样满足x/force/velocity阈值才成功。

**Run：** 运行my_run，确认40条expert、四模式/三seed与test_used=False；看contact_bc.png、results.json和任一失败trial。

**Modify：** 先预测哪些模型会改变、为什么state不会，再用原checkpoint做推理干预：

```bash
python examples/23_contact_aware_learning/contact_bc.py \
  --models-from tmp/s22_3/my_run/cohort --contact-input mean \
  --output tmp/s22_3/my_mean
```

比较成功分母、末段x/force误差和clipped_ticks。不要重训、调阈值或从test选模型。

**Explain（五问）：**

1. 本节goal第二槽与S22.2有什么不同？delta action、absolute ctrl和actual q变化的单位/关系分别是什么？
2. 为什么同reset的正负方向必须同split？normalizer用了哪些行，为什么不包括terminal、val或test？
3. state-only成功24/24、full23/24说明什么、不能说明什么？为什么offline误差更小仍可能闭环失败？
4. 移除通道后重训与冻结模型把contact换成train mean有什么区别？后一实验下降能支持什么结论？
5. 本节成功由哪两类条件组成？0.6s gate、早期超力锁存与末段稳定怎样配合，为什么最后恢复也不能洗掉失败？

下一小任务S22.4 contact ACT/Diffusion同任务比较，等待本人明确请求，不自动实施。
