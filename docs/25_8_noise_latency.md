# S22.8 — Noise / latency robustness

[代码](../examples/23_contact_aware_learning/noise_latency.py) · [示例README](../examples/23_contact_aware_learning/README.md) · [状态唯一来源](../README.md)

## Problem → Why

冻结S22.7 nominal BC，检查法向力噪声和传感观测延迟如何改变输入、动作与闭环行为。
本课只诊断，不补数据、不重新训练、不根据结果选择checkpoint。三个validation reset已参与此前选模，独立test未用，因此不是最终盲测。
BC在clean条件已经失败，成功率存在零值地板；需同时观察同状态动作敏感度和全过程失败原因。

## Intuition → Core Concepts

相机/编码器/力传感器给策略的是估计与历史，不是即时物理真值。噪声改变读数；延迟让读数对应过去的状态。
本课只给BC加合成观测通道：物理仿真、接触监视器和评分始终使用真值。
原专家保持干净监视器，独立运行三条作为流程参照；不能以专家成功证明它对噪声鲁棒，也不能声称BC的所有在线信息均已受污染。

38维输入中，`0:28`是关节q/qvel、物体位置/四元数、世界角/线速度和三指法向力；这些通道共同延迟。
`28:38`是过去已执行ctrl、当前决策时间、在线phase，保持当前值。phase来自干净1kHz monitor，是本课明确保留的理想信息。
只有`25:28`三个法向力读数加噪声，单位N。位置单位m、关节位置rad、线速度m/s、角速度rad/s；四元数无量纲。
延迟整条历史四元数而非逐分量加噪声，因此不会引入非单位四元数。
动作仍是6维绝对关节目标角度rad，限幅/rate cap/servo扭矩限制与原课相同。

预先固定六条件：clean；力噪声σ=.05/.15N；观测延迟1/3tick（20/60ms）；σ=.05N + 1tick组合。
相同reset使用`228000 + reset_group`随机seed，跨训练seed和条件使用匹配的随机序列。
这只控制外生噪声；不同策略访问的真实状态与截断后噪声误差仍可能不同。

## Mathematics：意义、shape、单位与时间

令真值观测为 \(o_k\in\mathbb R^{38}\)，决策间隔 \(\Delta t=.02\mathrm{s}\)。
采样时噪声 \(\epsilon_k\sim\mathcal N(0,\sigma^2I_3)\)，单位N；力传感代理为

\[
 c_{k,25:28}=\max(0,o_{k,25:28}+\epsilon_k).
\]

其余capture分量等于真值。非负截断使零力附近的误差具有正偏；不能将截断后的读数误差称为零均值高斯。
对整数延迟d，\(j_k=\max(0,k-d)\)：

\[
 \tilde o_{k,0:28}=c_{j_k,0:28},\qquad
 \tilde o_{k,28:38}=o_{k,28:38}.
\]

启动阶段重复首帧，而非读取负索引或未来帧。实际样本年龄为\((k-j_k)\Delta t\)，启动时小于设定延迟。
先在capture加噪声，再延迟整个capture；因此同一旧样本重复送达时，其噪声也相同。
这模拟固定传感样本延迟，不是action执行延迟、计算耗时、wall-clock sleep、丢包或乱序。

\[
 a_k^{raw}=\pi(\tilde o_k),\quad a_k=\operatorname{bounded}(a_k^{raw},ctrl_{k-1}).
\]

归一化使用checkpoint中的train-only参数，不重新拟合带噪/延迟轨迹。
在同一条实际访问轨迹上，离线计算干净输入反事实 \(a_k^0=\pi(o_k)\)：

\[
 E_a=\sqrt{\frac1{200\cdot6}\sum_{k,r}(a_{k,r}^{raw}-a_{k,r}^{0})^2}\quad[\mathrm{rad}].
\]

它测量该轨迹上的直接输入敏感度。不是动作标签误差、限幅后执行误差或任务鲁棒性；反事实动作不执行，不能据此推断干净策略会访问相同未来状态。
`force_input_rmse_N`比较收到的力与当前真值力；延迟条件下它含状态随时间变化造成的误差，不等于噪声σ。

## Math-to-Code / APIs

`ObservationChannel.__call__`在原rollout调用policy的位置接收当前pre-action观测，保存capture、源tick和实际policy input，返回6维raw action。
`np.random.default_rng(seed).normal(0,sigma,3)`生成独立三通道合成噪声；`np.maximum`实现非负截断；源tick显式满足`source <= tick`。
Python列表是本课短轨迹的历史缓冲；每条trial新建channel，防止状态和随机数跨轨迹串联。
`bc.restore`检查checkpoint SHA、schema、训练seed、登记source并恢复冻结MLP与normalizer。
`read_cohort`同时核对分组、训练manifest和数据SHA。训练provenance保留原始来源，评价source单独记录。

MuJoCo仍由原任务直接调用：`MjData(model)`创建可变仿真状态；第一次`mj_forward(model,data)`在上一ctrl下原地刷新接触/速度等派生量，不推进时间，返回值不作为观测。
policy调用后才写新ctrl，再`mj_forward`计算post-action审计量；`mj_step`原地积分1ms。
本课wrapper从前一次求解形成的观测取历史，从不使用同动作后的接触力。噪声不写回qpos、qvel、ctrl或接触参数。
真值obs有201帧，action/capture/policy input各200帧；末帧没有动作与新传感样本，不参与策略评估。

## Minimal Experiment / Expected Result

已有S22.7 cohort可直接复用；无`--models-from`时只采集原nominal训练/验证数据并训练原三seed BC，再冻结评价。
依赖mujoco/numpy/matplotlib/torch==2.14.0+cpu与原S18.11 helper链，均已在requirements声明；无新依赖。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/noise_latency.py --models-from tmp/s22_7/nominal_pipeline/cohort --output tmp/s22_8/my_run
```

也可省略`--models-from`生成自包含数据/cohort。output必须是新目录。
每次54条BC（6条件×3训练seed×3reset）+3条干净专家参照，所有attempt包含失败均保留。
协议在评价前保存；NPZ包含真值物理轨迹、200行capture/noise/received/source、反事实raw。
预期clean输入和raw反事实逐值相同；delay只取过去；noise不会直接改变仿真力；BC失败可能掩盖鲁棒性差异。

## Actual Result / Explanation

2026-10-12 DESKTOP-781D67A，复用S22.7 nominal cohort，默认运行exit0。干净专家3/3 ready/success；54条BC全部未ready、无pulse、success0。

| 条件 | BC ready/success | drop | 平均同状态raw RMSE(rad) | 平均force输入RMSE(N) |
|---|---|---:|---:|---:|
| clean | 0/9 | 1 | 0 | 0 |
| σ=.05N | 0/9 | 1 | .000860 | .036150 |
| σ=.15N | 0/9 | 1 | .002343 | .108520 |
| delay20ms | 0/9 | 0 | .001984 | 0 |
| delay60ms | 0/9 | 0 | .002712 | 0 |
| σ=.05N+20ms | 0/9 | 0 | .002501 | .035987 |

clean与两个noise条件各8条close_timeout、1条曾HOLD后loss/drift/drop/support失败；σ=.15N另触发rotation。
两个delay与组合条件各9条close_timeout，法向力始终为零，因此force输入RMSE为零；关节/物体状态的延迟仍会影响raw动作。
未drop不表示更稳定，可能只是没抓住、未加载，也没有收到pulse。
动作敏感度与σ不是严格线性；clip、非线性MLP、不同访问轨迹和执行限幅都会影响关系。
噪声翻倍Modify同cohort运行exit0，BC仍全部0/9ready/success；三个非零noise条件平均raw RMSE分别为.001615、.004824、.003207rad，force输入RMSE分别为.072419、.219250、.071974N。clean与纯delay预期不变，配对核验结果见工程验证。
默认最大Newton残差3.93e−15N，通过原1e−6N阈值；物理与评分规则未放宽。

## Failure Cases / Robotics Context

- 对physics或真值obs加噪后再评分会把观测误差混成物理失败。本课保持两者分离。
- 延迟包含当前动作或post-action反馈会泄漏未来；负索引会错误读到后面的历史。
- 每次送达重新加噪不是“采样后延迟”；startup重复样本需要明确约定。
- 当前phase仍揭示干净接触监视器的结果；这是局部传感链诊断，非完整部署误差模型。
- 无pulse不等于无.1N外载荷：进入HOLD后可在2s加载，却在2.5s前失败而未ready。
- 仿真proxy不等于真实传感器。真实系统还要校准bias、带宽、时钟、异步采样、丢包与执行延迟。

应用：辨别力反馈模型、视觉状态策略面对陈旧输入的敏感性，为后续是否需要历史模型、滤波或训练增强提供证据；本课不实施这些后续方案。

## Interview Capsule

30秒：冻结策略，把观测通道和物理真值分开；先对力capture加噪，再以因果索引延迟物理通道，保持执行历史和phase当前；用真值全轨迹评分，并报告同状态动作敏感度。

2分钟：解释38维通道的单位、20ms样本延迟与启动规则；非负截断产生偏置，当前metadata和过去physical channels形成有意异步输入。
说明权重、归一化、reset和噪声seed固定；真值monitor与干净专家具有理想信息。
BC原域已经失败时零成功率无法排名鲁棒性，同状态反事实RMSE也仅量化直接输入影响；最终独立test与真实硬件验证仍需另行开展。

## Must Remember / Run → Modify → Explain

必须记住：传感噪声不是物理扰动；观测延迟不是动作延迟；同状态raw变化不是成功率；干净monitor限制结论范围。

Run：执行上面命令，检查protocol、results和channels.png，辨认clean零误差及两个延迟的源tick。

Modify：先预测σ翻倍后截断偏置、动作敏感度与任务成功率是否必然翻倍，再保持同cohort/reset/延迟运行：

```bash
python examples/23_contact_aware_learning/noise_latency.py --models-from tmp/s22_7/nominal_pipeline/cohort --noise-scale 2 --output tmp/s22_8/my_noise2
```

比较非零noise条件；clean与纯delay条件应逐值不变。不能看到结果后换checkpoint或拟合归一化。

Explain：
1. 噪声为什么在capture时加入再延迟？启动三tick如何选source，怎样证明不读未来？
2. 哪些输入延迟，哪些保持当前？干净phase和专家参照如何限制鲁棒性结论？
3. 为什么clip后的噪声不再零均值？force RMSE为什么可能不同于σ，尤其纯delay条件？
4. 同状态raw action RMSE与两条闭环动作差异分别说明什么？限幅会如何影响解释？
5. BC在clean与受污染条件都未ready，能否说明鲁棒？未pulse、未drop能否说明抗扰，是否必然没有.1N载荷？

My Verification：本人已确认实验与预测完成，并准确回答五项Explain；Run/Modify/Explain全部完成，Engineering Complete + Learning Mastered（2026-10-12），状态以根README为准。下一小任务S22.9等待明确请求。

## Engineering Verification（2026-10-12，DESKTOP-781D67A）

WSL2 Ubuntu24.04.5，mujoco环境/所属Python3.12.14；MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2、Torch2.14.0+cpu版本/import/module paths/API核验通过，无安装。

```bash
python examples/23_contact_aware_learning/noise_latency.py --models-from tmp/s22_7/nominal_pipeline/cohort --output tmp/s22_8/default
python examples/23_contact_aware_learning/noise_latency.py --models-from tmp/s22_7/nominal_pipeline/cohort --noise-scale 2 --output tmp/s22_8/noise2
python tmp/s22_8/verify_channels.py tmp/s22_8/default
python tmp/s22_8/verify_channels.py tmp/s22_8/noise2
```

以上均exit0；checker是本机ignored临时独立审计，非共享运行入口。
两组共114attempt；21600策略输入/432108 BC physics行重建noise/capture/历史source/current metadata/raw/反事实/limiter与真值scorer。
两组24条clean BC/专家参照与S22.7 nominal Fx轨迹逐值回归；四条代表轨迹保存ctrl+载荷重积分qpos/qvel逐值一致。
27条clean/pure-delay配对轨迹除wall推理耗时外全部数组逐值不变，确认noise-scale只影响预定条件。
非法σ/delay被拒；CLI已有output、nan scale、Modify无frozen各exit2；Python3.11语法通过（actual3.12），两张PNG已查看，文档本地链接/git diff --check通过。
未重新运行fresh-training分支；本次执行明确复用已核验S22.7 cohort。无GUI/真实sensor/硬件/完整链延迟验证。
Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），不自动实施S22.9。

学习核验覆盖capture噪声与因果延迟、当前metadata及干净phase的结论边界、非负截断偏置与force RMSE、同状态raw敏感度与闭环差异、ready/drop/pulse及基础载荷的区别。本人明确完成Run/Modify/Explain；本次仅更新学习状态与文档，不新增运行或GUI验证，工程证据沿用2026-10-12。
