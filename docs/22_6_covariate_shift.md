# S19.6 — Covariate Shift / Compounding Error

## Problem → Why → Intuition

问题：为什么BC在专家状态上误差小，却在自己访问的状态上表现变差？一次或持续动作偏差会怎样传播？
S19.5已观察失败，但尚未归因。本课冻结BC，使用validation做受控诊断；不再把已查看test当盲测。
直觉：动作改变下一状态，下一状态又改变动作；每一步误差都发生在这个反馈链中。
主概念是policy-induced state distribution和误差传播，不训练恢复policy、不采集DAgger训练集。

## Core Concepts

| 内容 | meaning / unit / frame |
| --- | --- |
| d_E / d_π | 同reset/goal下expert或BC在时间上实际访问的状态分布；经验轨迹不是完整分布估计 |
| expert-state BC error | 将冻结BC查询在expert访问状态上，与该状态expert action比较；world XY reference m |
| BC-state BC error | 查询在BC访问状态上，再比较同状态expert action；不是与同tick另一轨迹动作硬配对 |
| intervention | policy reference上额外+world X 2mm；不是实际位置偏移、外力或观测噪声 |
| single / repeated | 0.5s即tick25一次20ms偏置；或tick25…44连续20次，覆盖.5… .9s |
| matched nominal delta | 干预轨迹与**同一controller同reset的nominal**位置差；m，不能与goal error混为一谈 |
| horizons | 每trial实际连续4s/200action；分别在2s/4s原末10观测窗口评分，不另reset |

expert和BC使用相同model/reset/goal/底层阻抗/gear/20N cap/成功阈值；干预按同tick施加。
重复偏置与单次相同幅度，但总暴露次数20倍，不声称两组总输入量相同。
4个validation reset、两controller、三condition，共24trial/命令；所有attempt保存。

## Mathematics → Math-to-Code

训练最小化E_{o~d_E}[||πθ(o)−π_E(o)||²]；执行需要E_{o~d_π}[同一误差]。
哪怕目标函数相同，输入分布改变后期望也可能改变。这是本课covariate shift问题。
令s为适当尺度化后的实际状态，a为尺度化的reference，在局部Lipschitz假设下可以写误差传播界：

```text
e_(t+1) ≤ L_cl * e_t + L_a * ε_t
ε_t = ||π(o_t^π) − π_E(o_t^π)||
e_T ≤ L_cl^T e_0 + L_a Σ_(k=0…T−1) L_cl^(T−1−k) ε_k
```

L_cl包含环境动力学与expert反馈影响，不是只看网络层数；本课没有估计这些常数或证明全局稳定。
L_cl<1可衰减，≈1可能累加，>1可能放大；方向、反馈与非线性也可使误差抵消或饱和。
“compounding”不是每一步必然变大，也不是无条件O(T²)或指数增长定律。
同一scale下训练误差小，并不为BC实际访问状态上的ε_t提供上界。

低层仍是F=400(reference−qpos)−D*qvel，ctrl=F/gear；附加δa_x初始请求力变化400δa_x N，
2mm对应.8N、4mm对应1.6N；随后实际运动与闭环反馈改变输出，不能把reference偏置当作qpos立即平移。
`run`每tick取实际observation→expert/BC raw action→加bias→env.advance执行20×mj_step。
`MjModel.from_xml_string`返回编译模型，`MjData`返回独立可变状态，`mj_forward`原地算派生量不推进time；
`mj_step`原地积分1ms。qpos/qvel只在reset赋值；不在干预时瞬移或冻结状态。

## State Coverage Proxy

只用train rows构造f(o)=[x−gx,y−gy,vx,vy]/saved_obs_scale[:4]。
D(o)=min_{train row i}||f(o)−f(o_i)||₂；单位是归一化坐标的距离。
mean减法在两点差中抵消，不需要refit；这是诊断feature，不替换原网络6维输入。
使用goal-relative位置减少不同goal的平移影响，但忽略绝对goal依赖；不能证明完整6维状态/条件覆盖。
最近邻距离受尺度、样本数和稀疏性影响，不是密度、confidence、OOD判据或support证书。
同样，BC-state误差变大与几何距离变大共同支持状态访问变化，但不独立证明全部失败根因。

## Minimal Experiment

依赖requirements已有torch==2.14.0+cpu/numpy/mujoco/matplotlib；helper链behavior_cloning→episode_contract→cartesian_spring。
读取train/validation/normalizer，核验prepared hashes和checkpoint split、normalizer逐字段一致；不读test、不refit。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/covariate_shift.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt
```

路径替换为本机匹配的S19.3 prepared与S19.4 checkpoint；缺ignored产物时按旧任务流程重建并记录本机结果。
输出新ignored tmp/s19_6/run_*：results.json、24个NPZ和shift.png。明确记录raw action、附加bias、applied action。
图左比较三condition的mean goal error，灰区标 repeated exposure；右比较nominal两种状态的coverage proxy与BC误差norm。
图的mean action-error norm与JSON的scalar per-axis RMSE不同，不能直接读成同一数字。

## Expected / Actual Result

2026-10-11 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco所属Python3.12.14，
NumPy2.5.3/MuJoCo3.14.0/Matplotlib3.11.2/Torch2.14.0+cpu metadata/import/API/helper路径核验，无安装/依赖改动。
CPU tensor/CUDA build None。最终命令：

```bash
python examples/20_learning_from_demonstrations/covariate_shift.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --output tmp/s19_6/verified2
python examples/20_learning_from_demonstrations/covariate_shift.py --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --bias-mm 4 --output tmp/s19_6/verified4
```

两命令exit0，各24trial×4000physics步=96000步；每种controller/condition在2s与4s各4attempt。
所有expert condition两horizon均4/4；所有BC condition均0/4，工程PASS不等于BC任务成功。

Nominal同一前100tick（2s）比较，避免混入长时settling帧改变平均权重：

| 指标 | expert访问状态 | BC访问状态 |
| --- | ---: | ---: |
| BC对同状态expert action的RMSE | .734121mm | 3.728322mm |
| mean nearest-train proxy | .199058 | .755913 |

同一冻结BC在不同访问状态上误差不同；expert-state .734121mm复现原validation offline误差。
4s nominal统计相应为.6131/3.55966mm和约.099/.617；不是网络改善，而是时间权重改变。

| intervention | Expert最大paired位置偏离 | BC最大paired位置偏离 |
| --- | ---: | ---: |
| single2mm | .20791mm | 1.85646mm |
| repeated2mm | 1.95238mm | 20.31803mm |
| single4mm | .41581mm | 3.39372mm |
| repeated4mm | 3.90475mm | 26.70684mm |

BC nominal 2s/4s全分母mean final goal error7.24174/7.47038mm。
repeated2mm为8.19989/7.84598mm；repeated4mm为6.68853/7.45671mm。
更大bias导致更大峰值paired偏离，但末帧goal error未单调增长；干预可能暂时抵消原始偏差。
这不是恢复policy成功：所有BC仍未通过1mm/2mm·s⁻¹联合criterion。

## Explanation / Failure Cases

受控干预使相同reset/controller仅在指定reference上不同，干预前obs前26帧完全一致；
随后paired轨迹差异可归于该干预及其闭环传播。无扰动BC与expert访问差异则涉及policy本身及动力学耦合。
expert可从这些偏置恢复、独立积分核验通过，支持当前fixture/执行链可正常工作；不证明所有控制问题都不存在。
无法仅靠本课拆分网络goal偏置、数据覆盖、局部反馈增益对原BC0/4的贡献，需要额外对照；不宣称完整根因定位。

- 把expert不同状态的动作直接当BC-state标签会错配；必须查询同一实际状态的expert。
- 把同一时间的干预delta当goal error，会混淆干预效应与baseline本来已有的失败。
- 单次与20次注入不是相同累计输入，不可仅把峰值比解释成系统增益。
- NN距离不是证明support外，也不检验真实sensor可得性或跨friction/mass泛化。
- 长时均值可能被settling帧改变，因此JSON同时报告100/200tick对齐统计。
- 本课validation诊断不训练；不把保存的expert查询诊断当已完成recovery/DAgger dataset。

验证：48保存trial独立重算192000个Euler physics步，逐tick raw/bias/applied action、success窗口、paired delta、
NN选定帧brute-force、artifact hash与aggregate通过；干预前prefix一致、两个幅度nominal全部数组一致、checkpoint SHA不变。
非法bias nan/0/>10、已有output、split不匹配拒绝exit2；Python3.11语法解析（未实际3.11运行）与两PNG目视通过。
links/anchors/git diff --check通过；无GUI、新训练、test重选模型、recovery/DAgger或接触任务。

## Robotics Context / Interview Capsule

应用：模仿学习闭环诊断、disturbance恢复能力评估、下一课恢复示范来源选择。
30秒：BC只在expert分布上拟合，动作误差可改变下一观测；同状态专家查询与paired干预有助分析误差传播。
2分钟：说明d_E与d_π、同状态action error、局部传播界，再解释单次/持续reference偏置与相同physics；
用.734→3.728mm说明访问状态差异，用非单调最终误差说明反馈与抵消，最后陈述NN与根因证据边界。
Must Remember：shift是访问分布变化；propagation不是必然发散；控制变量、对齐时间窗、报告全分母。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。Learning三项待本人完成。
Run：执行默认命令，读nominal的两种状态误差、paired最大偏离、2s/4s结果与shift.png。
Modify：先预测bias2→4mm会如何改变峰值与末帧误差，再运行：

```bash
python examples/20_learning_from_demonstrations/covariate_shift.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s19_4/default/policy.pt --bias-mm 4
```

Explain（五问）：
1. 为什么expert-state误差小不能限制BC-state误差？本例如何对齐同状态action比较？
2. 最近邻coverage proxy测什么，为什么不是OOD或密度证明？
3. reference偏置怎样通过controller影响实际状态，为什么不能直接改qpos？
4. 一次偏置、持续偏置及同controller nominal配对分别帮助说明什么？为什么末帧误差不一定单调？
5. 本课对shift/propagation已验证哪些证据，还缺哪些证据才能完整定位BC失败或证明恢复能力？

下一小任务S19.7 recovery data / DAgger intuition，等待明确请求；本轮停止。
