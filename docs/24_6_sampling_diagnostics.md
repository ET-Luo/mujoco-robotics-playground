# S21.6 — Multimodal path / sampling latency diagnostics

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [代码](../examples/22_diffusion_policy/sampling_diagnostics.py)
前置S21.5；0.5～2小时。主概念：冻结模型，通过采样对照区分局部多样性、成功路线覆盖与计算成本。
Engineering Complete表示实验与诊断可复查，不表示模型已经可靠；S21.7比较其他policy等待明确请求。

## Problem → Why → Intuition

S21.5单个随机stream失败，不能说明其他stream是否选另一条路，也不能用少数延迟推断少步采样的效果。
直觉：多抽样不等于多条合法路线；更快生成错误动作仍然失败。需要固定模型、任务、reset、执行前缀和随机合同，完整报告结果。
少步推理使用[DDIM论文](https://arxiv.org/abs/2010.02502)的eta=0更新，依据[作者实现](https://github.com/ermongroup/ddim/blob/main/functions/denoising.py)核对公式；只复用数学，不安装原项目或使用CUDA。

## Core Concepts / contract

| 量 | 含义、shape、单位与frame |
| --- | --- |
| H=10 | 一次生成10个动作槽，reference50Hz，覆盖.2s控制区间 |
| training K=100 | 训练noise schedule索引，保持不变 |
| S | 推理网络调用数，DDIM100/25/10；不是重新训练一个不同K的模型 |
| E=5 | 每次执行前5个reference，固定.1s重规划间隔 |
| o | 当前actual状态(1,6)，world位置[m]、速度[m/s]、goal[m]，train-only归一化 |
| A | normalized(1,10,2) → decoded absolute world XY reference[m] |
| mask | 剩余150tick预算决定，末尾不足10槽置零/排除；无未来success oracle |
| stream seed | 215…218，每reset每anchor seed=stream+reset×1000+anchor |

冻结S21.4 H10 MLP/23956参数/epoch146、same paired split2validation reset；不执行test。
默认四个config：DDPM100 baseline，以及DDIM100/25/10 eta=0。100→25→10是同一DDIM家族的调用数对照；DDPM100与DDIM100的差别还包括随机更新规则，不能只归因于步数。
没有裁剪reference、救援控制器、挑最好sample、训练或rollout选模。
无--checkpoint时，用已验证环境的sys.executable准备同一S21.4包，保证默认无需历史tmp。

## Mathematics / correct skipping

保留训练alpha_bar[k]。选择严格下降的原始索引k→s（s可小于k−1），eta=0：

```text
A0_hat = (Ak - sqrt(1-alpha_bar[k]) epsilon_theta(Ak,k/K,o,mask))
         / sqrt(alpha_bar[k])
As = sqrt(alpha_bar[s]) A0_hat + sqrt(1-alpha_bar[s]) epsilon_theta(...)
```

s=0时alpha_bar[0]=1，得到A0_hat；不添加新的z。随机性仍来自初始AK~N(0,I)，所以确定性去噪不代表所有seed输出相同。
`timestep_grid(S)`用round(linspace(100,1,S))选S个原始索引，再附clean0；如S10：100,89,78,67,56,45,34,23,12,1,0。
每个被选k仍输入k/100，不能重标为本次第几次调用/S。调用少，不改变训练噪声含义。
S1是100→0一次A0估计；API边界验证支持它，但实验只跑预声明的100/25/10/Modify50。
不能在旧DDPM相邻公式中任意跳过k。DDIM允许跳步，是因为更新显式使用目标alpha_bar[s]。
本节eta固定0；DDPM baseline继续使用posterior variance和k1不加噪。无debug trace的新draw与旧seq.sample DDPM输出逐值一致。
训练末端alpha_bar100≈.005619，标准高斯prior仍是近似，少步公式正确不保证模型生成质量。

## Three diagnostic levels / math-to-metrics

1. **固定初始观测的局部倾向**：每config每reset64个初始噪声样本，batch64。mean_h(reference_y)>1mm记upper hint，<−1mm记lower，否则center。
   各槽y先用±1mm deadband变为−1/0/+1，去掉0后数相邻符号变化。它描述局部抖动，不判定完整路线，正负样本也不证明密度具有两个统计模式。
2. **实际穿越方向**：完整rollout第一actual x>=0处，y>10mm为upper，<−10mm为lower，中间center；从未穿越为unresolved。沿用task scorer；方向本身不代表无碰撞或到达。
3. **成功模式覆盖**：对同reset四个stream，仅收集success=True的upper/lower路线。两种都出现才算该reset覆盖成功上下模式；结果以“覆盖reset数/2”报告，不用hint代替。

每config2reset×4stream=8trial，每trial150tick/30次重规划。全部32trial和4expert attempt保存。
闭环chunk_hint_changes是每次规划mean-y倾向之间的非零符号变化；这是“倾向切换”代理，不是经过因果定位的完整mode-switch计数。
reference step RMS=1000*sqrt(mean_t ||a_(t+1)-a_t||²)[mm]；本代码aggregate保存其trial均值，个别发散会主导均值，应同时看trials.csv各trial。
额外diverged_over100mm检查全过程actual position norm>100mm，仅作为诊断标记，不修改原success合同。
成功仍是全过程1kHz运动段圆盘净空>0，且最后10观测每个位置误差<1mm、速度<2mm/s。

## Math-to-Code / APIs

`ddim_step`输入Ak、预测epsilon、原k/目标s、训练alpha_bar，返回新As；不操作物理状态。
`draw(...,record=False)`从高斯prior调用模型S次，返回normalized chunk；record=True额外返回全部状态、epsilon与z，方便独立公式检查。
`Planner.infer`只读取actual normalized o和预算mask，用相同seed规则生成plan；复用task.rollout chunk接口，kind='act'表示接口分支，实际没有ACT网络。
`chunk_hint`返回−1/0/+1，输入为米单位reference与valid mask；`summarize`分别汇总失败、穿越方向和成功路线覆盖。

expert/MuJoCo执行复用已有API：MjModel.from_xml_string编译模型，MjData保存可变状态；mj_forward刷新派生量不推进时间，mj_step原地积分qpos/qvel/time。
低层每ms计算400(reference−q)−D*qvel，再除gear写ctrl，实际力限幅±20N。无界slide关节及几何圆盘无反力的限制沿用S21.5。
依赖已有NumPy/torch==2.14.0+cpu/MuJoCo/Matplotlib；helpers为diffusion_rollout.load_checkpoint、sequence_diffusion及其collect/noising/windows/task链，无新依赖或安装。

## Latency scope

CPU1thread/batch1，固定初始actual观测，先3轮warmup，再20轮计时。每轮轮换config顺序以减小长时间分块测量的偏差；保存全部原始延迟。
范围：cached raw observation归一化→预算mask→draw（不保存debug trace）→反归一化→finite检查。
不含传感器读取、physics、I/O、系统调度；不是整个机器人端到端周期。固定状态基准也未证明发散/OOD状态下的尾延迟。
报告median、p95、max及超过名义E5=100ms的样本数；20个样本中的零超时不能证明实际deadline miss rate为零。
同步仿真采样时物理时间暂停，不模拟真实机器人等待期间继续运动。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/sampling_diagnostics.py
```

自动新ignored tmp/s21_6/run_*；显式--output须尚不存在。
输出training/（默认准备时）、protocol/results、trials.csv、512个局部样本的8份probe NPZ、32trial+4expert NPZ、latency.npz、diagnostics.png。
预期同weights同seed重现、DDIM降低调用数通常降低计算成本；安全/覆盖实测，不预设更优。

## Actual Result / Explanation

2026-10-12 Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2，mujoco内Python3.12.14；NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.13.0/Matplotlib3.11.2 metadata、真实paths/API通过，无安装。
默认命令加`--output tmp/s21_6/default`；Modify加`--output tmp/s21_6/steps50`，均exit0。

| sampler | success | collision | actual upper/lower/center/unresolved | >100mm发散 | 成功上下覆盖reset |
| --- | --- | --- | --- | --- | --- |
| DDPM100 | 0/8 | 2/8 | 2/3/2/1 | 1/8 | 0/2 |
| DDIM100 | 0/8 | 4/8 | 2/1/4/1 | 1/8 | 0/2 |
| DDIM25 | 0/8 | 4/8 | 2/2/3/1 | 1/8 | 0/2 |
| DDIM10 | 0/8 | 4/8 | 2/2/3/1 | 1/8 | 0/2 |

expert上下各2/2。初始probe upper/lower/center汇总128样本：DDPM32/59/37，DDIM100 45/54/29，DDIM25 45/52/31，DDIM10 44/48/36。
64样本中含槽间非零符号变化的数量（两reset分别）：DDPM44/48、DDIM100 48/41、DDIM25 47/40、DDIM10 42/38。
因此局部倾向多样且抖动，实际穿越也有上下，但没有成功路线覆盖证据。
闭环倾向切换总数14/24/22/18；reference step RMS的trial中位数5.838/7.364/7.191/6.863mm（独立从trial表计算）。
发散导致其trial均值704.498/694.738/726.768/778.650mm；不能把该均值直接解释为通常抖动幅度。

| sampler | pipeline median [ms] | p95 [ms] | max [ms] | >100ms /20 |
| --- | --- | --- | --- | --- |
| DDPM100 | 8.349 | 11.351 | 26.090 | 0 |
| DDIM100 | 7.164 | 13.819 | 14.468 | 0 |
| DDIM25 | 2.038 | 3.258 | 3.657 | 0 |
| DDIM10 | .899 | 1.684 | 2.541 | 0 |

少步降低本机median，不宣称尾延迟严格单调或实时满足；DDIM100 p95高于DDPM100也说明小样本测量有抖动。
Modify DDIM50仍0/8、collision4/8、路线2/1/4/1、覆盖0/2；median3.025ms/p955.392ms/max10.890ms，倾向切换24。
其余DDPM100/DDIM100/DDIM10的probe与完整轨迹逐值一致；重新测量latency不会逐值一致。
少步未修复可靠性；全部失败且只有两reset、单training seed，不能根据这些数值选“最佳sampler”。四stream是采样seed，不是四次独立训练。

两组72trial/expert独立重算216000个Euler步与运动段几何/scorer；全部1920actual anchor重新采样核对输入、seed、mask、执行槽。
所有probe反向转移、反归一化/hints/指标/成功coverage/latency重算通过；DDIM oracle恒等式、grid全部S1…100、debug capture不变性、masked tail、DDPM旧sampler/S21.5 stream215回归通过。
Modify不变配置逐值核验、非法steps0/重复steps/负seed CLI拒绝、3.11语法、links与git diff --check通过（实际3.12）。默认PNG已查看。
无test/GUI/接触反力/硬件实时、统计密度多模态证明或模式切换因果定位；产物只在ignored tmp。

## Failure Cases / Robotics Applications

- 删掉DDPM步而不改目标alpha_bar：错误跳步公式。
- 将训练K改为S或错误重标k：模型收到错误噪声等级。
- eta0被误认为无多样性：初始prior仍随机。
- 上下符号/穿越次数当成功覆盖：必须保留全程安全与稳定到达。
- 看到hint变化就断定失败原因：这是代理统计，不是因果实验。
- 选最好seed/best-of-N却只报告成功：改变策略与分母，本节保留全部attempt。
- 小延迟或零名义超时当硬件deadline保证：测量范围与样本量不支持。

机器人语境：采样器是质量/计算成本取舍；可靠执行还依赖动作一致性、训练分布、约束与控制反馈。诊断先区分“计算快”“局部多样”“合法完成”，再决定后续研究方向。

## Interview Capsule / Must Remember

30秒：冻结noise model，用eta0 DDIM的目标累计噪声系数支持跳步，比较100/25/10次调用。分别测局部chunk倾向、实际穿越和成功路线覆盖，再测明确范围的batch1延迟。本例少步更快但全部0/8，没有成功覆盖。

2分钟：写A0_hat与As公式，说明原k与新S的区别、eta0和prior随机性；描述paired resets/four sampling streams与全attempt；解释hint不等于homotopy/成功路线，碰撞与tail必须独立计分；给延迟scope、warmup、p95与名义超时边界；最后说明对照只改变sampler/步数，不能凭小样本挑优或宣称已定位失败原因。

Must Remember：跳步需新公式；训练K不变；eta0仍可多样；成功覆盖不是y符号；固定模型全分母；延迟范围明确。

## My Verification / Run → Modify → Explain

Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），根README为状态唯一来源。
解释覆盖训练K与推理S、正确DDIM跳步与eta0随机性、局部倾向/actual路线/成功覆盖、因果边界与training/sampling seeds、计时范围与硬实时限制。
精度补充：本节pipeline具体包含cached observation归一化、mask、噪声初始化、网络与显式更新、decode及finite检查，排除physics/传感/I/O；0/8成功与0/2成功覆盖结论保持不变。
Run：运行默认命令，查看局部probe、实际route和success coverage三个层级，以及latency原始值。
Modify：先预测25→50次调用的成本与可能质量变化，冻结Run的checkpoint（替换路径）：

```bash
python examples/22_diffusion_policy/sampling_diagnostics.py \
  --checkpoint tmp/s21_6/default/training/chunk.pt --ddim-steps 100 50 10
```

只替换中间DDIM调用数，保持其他sampler、模型、H/K/E/reset/seed不变；不能只报告最好的配置。
Explain（五问）：
1. training K与inference S有什么区别？为什么正确DDIM可跳步而旧DDPM相邻公式不能直接删步？
2. eta=0为何仍会有样本多样性？本实验初始noise如何配对，DDPM100与DDIM100还改变了什么？
3. chunk hint、actual crossing route、successful mode coverage各说明什么？为什么前两者都有上下仍可能覆盖0/2？
4. hint切换/reference抖动为什么不能直接证明失败原因？四sampling seed是否等于四training seed？
5. latency测了哪些环节？p95、max与20次零名义超时为什么不足以保证真实机器人实时？

下一小任务S21.7同task BC/ACT/Diffusion比较等待明确请求；不自动实施。
