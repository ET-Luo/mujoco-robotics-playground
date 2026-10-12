# S22.7 — Friction / mass / domain generalization

[根README状态](../README.md#stage-22--contact-aware-robot-learning) · [扫参入口](../examples/23_contact_aware_learning/domain_generalization.py) · [共享物理循环](../examples/23_contact_aware_learning/dexterous_task.py)。学习时间0.5～2小时。

## Problem → Why → Intuition

S22.6只在mass=.03kg、μ=.7的三指free-object场景采集示范。相同初始状态与策略遇到未见摩擦／质量时会怎样？
本课学习**物理域偏移的实验合同**：冻结训练结果，一次只改一个物理因素，用专家和learner共同区分物理任务边界与学习失败。

直觉：更小μ允许的切向接触力更小；改变质量也改变惯性和接触响应。但不同参数不自动推出单调成功率。
本课BC已经在nominal域失败，OOD同样失败不能证明泛化良好，也没有成功率下降空间可用于排名。

## Core Concepts / Protocol

| Domain | mass [kg] | object-contact μ | 角色 |
| --- | ---: | ---: | --- |
| nominal | .03 | .7 | 训练域同分布诊断 |
| friction_03 | .03 | .3 | held-out physics，摩擦单因素 |
| friction_01 | .03 | .1 | held-out physics，摩擦单因素 |
| friction_003 | .03 | .03 | held-out physics，摩擦单因素 |
| mass_0015 | .015 | .7 | held-out physics，质量单因素 |
| mass_006 | .06 | .7 | held-out physics，质量单因素 |
| mass_009 | .09 | .7 | held-out physics，质量单因素 |

网格预先声明；没有μ/m组合、noise/latency、contact输入消融、domain randomization或OOD训练。
训练／offline选checkpoint／normalizer只用nominal。权重、归一化、action box/rate/servo、scorer与horizon全部冻结。
质量和μ不作为policy输入；不把真实参数值当privileged条件提示。

同12train／3validation reset groups，test3组仅预留。评价每个物理域都配对同3个validation resets，
保留object xyz/yaw、finger q的相同初始值；改物理参数而不同时换随机初态。
这些reset已用于S22.6 validation，本次是**公开的physics-held-out诊断，不是新的独立盲测**。
OOD结果不能用来重选checkpoint／seed、重新拟合normalizer后仍声称原冻结泛化实验。

默认7 domains×3resets×（expert＋3个BC seeds）=84attempt。默认pulse=baseline；
Modify给同整个网格施加Fx+默认幅值.2N／.2s半正弦（仅ready触发），不是质量×摩擦多因素组合。

## Mathematics：物理意义、shape、unit、frame

### 摩擦约束

滑动接触的直觉边界：`||F_t|| ≤ μ F_n`，F[N]，μ无量纲，n/t属于接触局部frame。
这是容量直觉，不能仅凭这一个理想式计算MuJoCo动态抓持成功；软接触、三维几何和速度都参与。
condim=3，只包含法向和两个切向力，不引入spin/roll接触力矩。

MuJoCo动态接触在geom同priority时逐分量取较大摩擦；只把物体μ设小，手指仍为.7，会掩盖降μ实验。
本课把object_geom priority固定为1（其它geom为0），使物体接触有效μ采用物体值；所有域都同样设置priority。
手指自身／非物体接触仍保留原.7，不把整只手的摩擦同时改变。
object与finger的solref/solimp相同，nominal中priority修改不改变实际求解参数；已用旧轨迹核对数值行为。
规则依据：[MuJoCo官方 Contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters)。
脚本在保存的若干state上重求解并核验实际`contact.friction[:2]`；无active物体接触时明确记录null，不假造摩擦测量。

### 质量、惯量与载荷

圆柱半径r=.02m、总高L=.05m：
`I_transverse = m(3r²+L²)/12`，`I_axial = mr²/2`，[kg·m²]，惯量为COM处主轴下的三元素。
nominal为9.25e−6、9.25e−6、6e−6kg·m²（编译主轴排序可能不同）；相同几何时质量改变，惯量须同比例改变。
通过XML `geom mass`重新编译，让MuJoCo同时计算mass/inertia；不只写body_mass而留下旧惯量／派生常量。

world COM平衡：`m*a_W=F_contact_W+F_applied_W`，shape(3,)，m[kg]、a[m/s²]、F[N]。
共享rollout现用`model.body_mass[object]`核对Newton，而不是固定.03常量；断言仍<1e−6N。

**全局gravity=0，HOLD物体2s后向下载荷固定.1N，不随m改变。**
所以质量实验观察固定外力下的惯性／接触响应，不是F=mg随质量增加的重量实验。
若改为m*g，还改变了施加载荷，不能把结果称为本协议下的纯质量比较。
positive solref软约束的接触响应与质量／等效惯性有关，不能只用静态摩擦容量来解释全部漂移。

### 合同保持

50Hz策略、1kHz物理/scorer，4s；38维因果观测（q/qdot、object pose/velocity、上一ctrl下normal、past ctrl/time/phase）。
200action／201obs；6维absolute reference[rad]，box+每20ms .02rad rate cap，servo±.08N·m。
2s anchor；2.5s ready；全1kHz contact/slip/pose/drop/support失败锁存及末250ms速度恢复，与S22.6相同。
未ready不施加pulse仍留分母；HOLD后的.1N载荷不依赖ready，所以无pulse不等于无载荷。

## Math-to-Code / APIs

- `domain_model(mass_kg,friction)`：复制hand.xml、加free cylinder、设置mass/priority/friction；`MjModel.from_xml_string`返回编译模型，不改旧hand.xml。
- `model.body_mass[id]`为kg标量，`model.body_inertia[id]`为(3,)主惯量[kg·m²]；只读核验。
- `rollout(...,domain_model=model)`：新增可选模型输入，复用同一因果闭环／scorer；默认模型路径仍为S22.6 nominal。
- `MjData(model)`新建trial；`mj_forward`原地更新力/加速度、无时间推进，`mj_step`原地推进qpos/qvel/time；质量只在编译时改变，不在episode内跳变。
- `contact_parameter_probe`在保存state下重建`MjData`并forward；检查`contact.dim==3`及`contact.friction[:2]`真实值，返回active probe计数／范围。探针不进入policy输入，也不改变原轨迹。

为了保留S22.6已冻结模型，loader登记该历史task/training-driver的两个准确SHA；其它helper哈希仍须一致。
当前新训练source必须完全匹配当前source，未知旧source、checkpoint/data SHA错配仍拒绝。
训练provenance保持原值，evaluation source另记录；没有把旧cohort元数据改写成新训练版本。

## Minimal Experiment / Run

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/domain_generalization.py --output tmp/s22_7/my_run
```

output须新目录；无外部下载。默认重新采集nominal的15expert、训练三BC，然后冻结评估网格。
可复用已有S22.6 cohort，省掉名义域重训：

```bash
python examples/23_contact_aware_learning/domain_generalization.py --models-from tmp/s22_6/my_run/cohort --output tmp/s22_7/my_frozen
```

所有data/cohort/rollouts/protocol/results/PNG只ignored tmp；协议先写文件再开始OOD评价。
主要看results.json中每域planned/ready/success/drop和failure_first_times；domains.png同时画ready与success，
没有删掉未ready来美化success率。

## Expected / Actual / Explanation

预期：nominal专家可行；较低μ可能无法抵抗固定载荷；质量影响不是简单“越重越失败”。
BC的nominal成功率已为零，需报告地板效应及实际有没有建立接触／收到载荷／pulse。
2026-10-12 / DESKTOP-781D67A：默认命令及冻结同cohort的Fx+命令均exit0，每组完成全部84attempt。

| 参数域 | baseline expert ready / success | Fx+ expert ready / success | BC（三seed合计） |
| --- | --- | --- | --- |
| nominal .03kg/μ.7 | 3/3、3/3 | 3/3、3/3 | 两组均0/9 ready/success |
| μ.3 | 3/3、3/3 | 3/3、0/3 | 两组均0/9 |
| μ.1 | 3/3、3/3 | 3/3、0/3 | 两组均0/9 |
| μ.03 | 0/3、0/3 | 0/3、0/3 | 两组均0/9 |
| .015kg | 3/3、3/3 | 3/3、3/3 | 两组均0/9 |
| .06kg | 3/3、3/3 | 3/3、3/3 | 两组均0/9 |
| .09kg | 3/3、3/3 | 3/3、3/3 | 两组均0/9 |

每组expert21attempt，baseline18/21→Fx+12/21；BC63attempt，均0/63。
μ.03专家曾进入HOLD，在2s后固定载荷下loss/slip/drop并未ready；Fx+未施加，不是抗扰成功。
μ.3/.1专家baseline通过，Fx+发生contact_loss/slip（部分rotation，μ.1另有drift），
说明这个固定专家承受nominal载荷和横向扰动的边界不同；不证明任何控制器在低μ都不可能成功。
质量三组成功不代表重量泛化或所有质量都可行：只有固定外力、三个离散参数、三个reset和4s。
BC每域8/9 close_timeout，seed226/reset22652一条曾HOLD后失败；在.09kg该条未drop但仍因loss/slip/drift/tail失败。
每组drop次数expert3、BC6；未drop不是成功。两组BC无pulse，逐值相同，不能宣称抗域偏移／抗扰。

新nominal示范15/15成功，2400/600行；三training seeds/best91/84/110及offline RMSE与S22.6逐值一致。
实际质量与惯量按编译模型核对，实际contact μ而非只看XML；Newton阈值不放宽。
所有protocol/NPZ/JSON/PNG及本地独立checker仅ignored `tmp/s22_7/`。
环境：WSL2 Ubuntu24.04.5/kernel6.6.87.2，mujoco所属Python3.12.14；MuJoCo3.14.0、NumPy2.5.3、
Matplotlib3.11.2、Torch2.14.0+cpu metadata/真实imports/paths/API通过，无安装。
验证命令`python tmp/s22_7/verify_domains.py` exit0（本地独立checker不提交）：

- 全7模型的几何、关节/servo、solref/solimp、priority等不变；仅object μ或mass及其同比例惯量改变，field逐项比较。
- nominal15条采集NPZ、2400/600行、normalizer与S22.6逐值一致；三模型state_dict／normalizer逐值一致，offline best epoch核对。
- 两组168trial/672168physics行，完整Newton／servo／box/rate／held ctrl／平移积分、pulse／load和scorer/first times独立重算；最大Newton残差8.11e−11N。
- 33768个policy观测在保存state＋previous_ctrl下独立重建，7318个含active finger接触的输入核验实际μ；mass/μ不作为输入。
- 两组prefix和未ready轨迹逐值一致；24个nominal评价轨迹与旧S22.6 baseline/Fx+逐值一致。
- 六条不同参数／成功／drop／timeout代表的保存ctrl与载荷从初始状态独立重积分，qpos/qvel逐值一致；两条原observer/monitor实际action回放obs/physics逐值一致。
- 当前和登记的旧cohort均可加载；未知source、checkpoint篡改拒绝；零/负mass、零/NaN μ拒绝；Fx+无冻结模型、非法pulse、已有output均CLI exit2。
- 三文件Python3.11语法检查通过（实际3.12）；两PNG已查看，Markdown links和`git diff --check`通过。

没有新依赖或系统配置修改。已有S22.6脚本仍能加载已登记的旧模型，training provenance保持原SHA，未知旧source不自动放行。
无三test reset组、GUI、真人、真实sensor、domain randomization、ACT/Diff比较或硬件验证。

## Failure Cases / Robotics Context

- 只改object friction而忽略混合规则，实际contact μ仍为.7；必须检查实际contact字段。
- 改mass不改inertia，或者改质量同时让载荷随之增加，却把结果称为同一种单因素实验。
- 用OOD选模型/调参，再把同一域称为从未见过的独立test；本课三test reset组仍未使用。
- ID失败→OOD也失败，没有证据证明鲁棒；未接触的trial根本没有激发该接触摩擦条件。
- 低μ未ready、未施加载荷/pulse，若删除这些trial，分母就不再反映整个抓持任务。
- 每域3expert／9BC是3个reset×3个training seeds；同reset相关，不冒充9个独立环境seed或统计置信区间。
- 单一网格不是domain generalization定理，也不证明真实对象材质／质量的可迁移性。

应用：机器人sim-to-real前的参数敏感性诊断，先辨明任务物理边界、策略ID能力和真正的OOD表现；
将来扩展训练域须重新定义训练合同、匹配预算并保留新的未见物理域，而不是偷偷加入本次失败样本。

## Interview Capsule

30秒：我把训练域固定为30g/μ.7，冻结BC和统计，逐一改变物体接触μ或质量，并用同reset与同低层控制／scorer评价。质量与惯量一同编译，实际contact μ用priority控制并检查。BC在ID已经失败，所以不能从OOD同样零分声称泛化；专家用于展示物理任务边界。

2分钟：说明ID新reset与OOD新物理参数的区别；画训练与验证来源组、物理网格和独立test保留的边界。
解释为什么降低geom friction可能不等于降低contact friction，为什么mass和inertia需一致；
指出本课固定.1N、零重力，与重量扫描不同。最后展示planned/ready/success/drop与失败锁存，
说明地板效应、未ready无pulse和不能按OOD结果选模型的限制。

## Must Remember / Run / Modify / Explain

Modify：先预测低摩擦／不同质量下Fx+如何影响ready和success，再冻结同cohort加扰动：

```bash
python examples/23_contact_aware_learning/domain_generalization.py --models-from tmp/s22_7/my_run/cohort --pulse Fx_plus --output tmp/s22_7/my_fx
```

比较同域同reset同controller的baseline与Fx+，检查2.5s前轨迹相同，未ready全轨迹不变。
Fx+只改变外部pulse；网格、训练、权重、normalizer及固定.1N载荷不变。

Explain：

1. 新validation reset与未见μ/m分别属于什么泛化？本次为什么不是独立blind test，OOD结果能否选checkpoint？
2. 为什么只减object geom friction可能扫参失效？priority如何控制有效μ，怎样验证实际接触参数？
3. 为什么mass变化必须伴随inertia变化？本课是否模拟重量增加，Newton核验为何不能继续写固定.03？
4. 为什么要配对reset、分别改μ与m、冻结normalizer？把低摩擦失败加进训练后还能叫同一冻结泛化实验吗？
5. BC在ID和OOD都失败能否证明抗域偏移？planned／ready／success／drop各告诉你什么，未ready与无pulse是否表示无载荷？

My Verification：本人已确认实验与预测完成，并准确解释五项核心问题；Run/Modify/Explain全部完成，Engineering Complete + Learning Mastered（2026-10-12），状态以根README为准。

验证理解：新reset与未见动力学参数分别检查初始状态与物理域泛化；有效摩擦须核对contact.friction；质量与惯量同比缩放，零重力下固定外载荷不是重量测试；配对reset与冻结权重、train-only归一化保持比较一致；BC在nominal已失败，不能用OOD零成功率判断泛化能力。本次是validation参与过选模的诊断，独立test未用。
本次学习状态更新仅核验文档；运行结果沿用2026-10-12工程验证，不新增运行或GUI验证。
下一小任务S22.8等待明确请求，不自动实施noise/latency。
