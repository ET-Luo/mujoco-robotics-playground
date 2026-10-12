# S21.3 — Conditional diffusion / tiny denoiser

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [实验代码](../examples/22_diffusion_policy/conditional_denoiser.py)
前置S21.2；0.5～2小时。主概念：学习epsilon_theta(a_k,k,o)，让去噪依赖噪声等级和当前状态。
本节H=1，只训练单步reference的噪声预测；S21.4再处理chunk/mask/反向采样，S21.5再执行生成策略。

## Problem → Why → Intuition

S21.2使用已知epsilon恢复动作，但部署没有oracle。现在用网络从带噪动作推断epsilon。
当前状态告诉模型动作应与什么任务状态相容；k告诉模型信号被破坏了多少。
直觉：同一模糊动作数值，在不同状态或不同噪声等级下，需要不同解释。
条件生成的机器人语境见[Diffusion Policy论文](https://arxiv.org/abs/2303.04137)，噪声预测目标见[DDPM](https://arxiv.org/abs/2006.11239)。这里是CPU教学MLP，不是论文视觉模型的复现。

## Core Concepts / data contract

| 量 | shape / meaning / units / frame |
| --- | --- |
| raw o_t | (6,)，world x,y[m]、vx,vy[m/s]、goal x,y[m] |
| raw a_0 | (2,)，当前tick的world XY绝对位置reference[m]，不是ctrl或下一qpos |
| normalized o / a_0 | (B,6)/(B,2)，train-only按通道mean/std变换后，无量纲 |
| k | (B,)，每行独立均匀抽取整数1…100；网络输入k/K为(B,1) |
| epsilon / a_k | (B,2)，标准高斯标签 / 归一化带噪动作 |
| epsilon_hat | (B,2)，网络输出，无量纲；不直接送到机器人 |

下标a_0指干净数据，不是episode第0个动作；t才是机器人时间索引。
网络输入总共9维：2维带噪动作+1维k/K+6维状态。MLP 9→64→64→2，SiLU激活，4930参数。
标量k/K是最简单明确的时间条件；后续可研究embedding，本节不增加复杂结构。

复用S21.1 B轨道`xy_two_route_disk_v1`：同seed上下两路同组，8train/2validation/2test reset。
现场生成16train与4validation episode（2400/600行），不依赖旧tmp、不重训BC/ACT。test仅保留seed。
route/phase/timestep/pair_id仅用于数据审计，不作为网络输入。
初始上下路同观测，状态并不能消除所有模式歧义；后续y/vy可提供已走路线的信息。
所有行含goal hold，指标是行平均，未另做运动区间或初始歧义专项评估。

## Mathematics

```text
alpha_bar[k] = product_(i=1..k)(1-beta[i])
a_k = sqrt(alpha_bar[k]) a_0 + sqrt(1-alpha_bar[k]) epsilon
loss = mean_(batch,channel) (epsilon - epsilon_theta(a_k,k/K,o))²
```

保持S21.2的K100、linear beta=.0001… .1。每个训练batch重新采样k和epsilon。
标签由程序生成；网络只接收a_k、k/K、o，a_0和真实epsilon仅参与造训练样本与计算loss。
MSE最优预测是E[epsilon | a_k,k,o]。这仍是条件均值，但对象是每个噪声等级的epsilon；完整动作分布生成还需要反向过程，不能直接用一次预测证明多模态覆盖。

离线诊断使用已知a_0造a_k，再计算：

```text
a_0_hat = (a_k - sqrt(1-alpha_bar[k]) epsilon_hat) / sqrt(alpha_bar[k])
a_0_hat-a_0 = -sqrt((1-alpha_bar[k])/alpha_bar[k]) (epsilon_hat-epsilon)
raw_error = normalized_error * train_actions_scale
RMSE_mm = 1000 * sqrt(mean(raw_error²))
```

RMSE按样本/XY通道平均，不是每个点的欧氏距离均值。恢复可很不稳定，不clip输出掩盖误差。
高k时a_k本身接近epsilon，所以低epsilon MSE不等于动作恢复简单；小信号系数仍放大残差。

## Math-to-Code / APIs

`collect(output)`直接调用已有task.reset/references/rollout，返回arrays、normalizer、paired groups。
`MjModel.from_xml_string(XML)`编译返回模型，`MjData(model)`创建独立可变状态。
示范rollout只在reset写qpos/qvel；`mj_forward`原地刷新派生量不推进时间，`mj_step`原地积分qpos/qvel/time。
每ms控制F=K(reference-q)-D*qvel，requested/gear写ctrl，限幅±20N。禁入圆盘是几何约束，无contact反力。
本节仅用expert采集，不将denoiser输出执行到MuJoCo。
local helpers：dual_path_task→episode_contract→cartesian_spring（XML/动力学）；noising_basics提供schedule，依赖NumPy/Matplotlib。
执行依赖requirements已有NumPy、torch==2.14.0+cpu、MuJoCo、Matplotlib；无新库/安装。

`noising`按k gather alpha_bar，系数(B,1)广播到两通道。独立NumPy公式核验用于捕捉行索引/广播错误。
`TinyDenoiser.forward`拼接输入，返回epsilon_hat，不改机器人状态。
`nn.Linear`建立可训练仿射层；`loss.backward()`原地累积参数梯度，`optimizer.step()`更新参数；zero_grad清旧梯度。
`model.eval()`切换评价模式，`torch.inference_mode()`禁用梯度记录；本MLP没有dropout或BatchNorm。
`--conditioning none`用零替代归一化状态，保留相同层尺寸、初始化、训练排列/k/epsilon，隔离状态输入。

CPU1thread/seed213/Adam lr.001/batch128/100epoch；epoch0保留未训练参考。
selection固定validation每行3份k/epsilon（seed21301）以减少选模指标抖动；取最小MSE checkpoint。
audit同一validation reset使用另一噪声seed21302，共1800case，选模后才评价。
独立noise不是新holdout机器人状态；它只检查同validation状态上的新加噪样本。
checkpoint保存权重/日程/normalizer/protocol，reload预测逐值一致；本节不提供跨任务加载/部署接口。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/conditional_denoiser.py
```

打印新ignored `tmp/s21_3/run_*`：train/validation/normalizer NPZ、protocol.json、denoiser.pt、loss.csv、audit.npz、results.json、denoising.png。
显式`--output`目录必须尚不存在，以免覆盖已有产物。
预期训练改善epsilon误差；状态干预/wrong-k用于检查输入作用，效果大小以实测为准。

## Actual Result / Explanation

2026-10-12 Zero；WSL2 Ubuntu24.04.5/kernel6.18.33.2，mujoco内Python3.12.14；NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.13.0/Matplotlib3.11.2 metadata、真实paths/API通过，无安装。
命令`python examples/22_diffusion_policy/conditional_denoiser.py --output tmp/s21_3/state_checked`；
再`--conditioning none --output tmp/s21_3/none_checked`，均exit0。

| 配置/诊断 | audit epsilon MSE | 恢复reference RMSE [mm] |
| --- | --- | --- |
| state训练模型 | .039117 | 5.109 |
| state未训练 | 1.015598 | 85.746 |
| zero epsilon baseline | 1.007732 | 85.516 |
| state训练模型 + 状态循环错配137行 | 4.246114 | 24.305 |
| state训练模型 + 全部k/K错设.5 | .295785 | 44.117 |
| none重新训练 | .307293 | 17.436 |

state/none best epoch100/99、selection MSE .038669/.310398，trainloop1.132/1.141s（不含采集/audit/绘图）。
两组同数据/normalizer与audit clean/o/k/epsilon/noisy逐值一致；state错配是冻结模型干预，none是相同训练预算重新训练，不能混称一种ablation。
none模型打乱状态结果逐值不变，符合忽略状态的实现。
state分k区间epsilon MSE：1–25 .148594、26–50 .006420、51–75 .003061、76–100 .003142。
低噪声时epsilon被弱化，识别具体噪声更难；高噪声epsilon易从a_k猜到。该解释与forward系数相符，不构成全部优化行为的因果证明。
打乱状态破坏了状态/动作对应关系，可能离开联合数据分布；不能把该数值当真实机器人故障率。

首轮训练完成后float32/float64公式检查失败：低噪声处1-alpha_bar相减导致sqrt系数差异，最大约1.913e-6。
把NumPy交叉检查绝对容差4e-7调到3e-6后完整重跑；首轮不计PASS，oracle恢复另设5e-6。
独立重算全部audit指标/forward、train-only normalizer/paired split、匹配对照输入、none不变性、reload一致与3.11语法通过。
默认PNG已查看；runtime实际为3.12，未声称3.11运行。非法epoch0/负seed CLI拒绝；链接与git diff --check通过。
未生成test、未做反向采样/生成策略rollout/碰撞率/模式覆盖/GUI；单training seed、两validation reset不支持泛化排名。

## Failure Cases / Robotics Context

- 输入真实epsilon或干净a_0：泄漏答案，部署不可用。
- 用validation拟合normalizer、按行拆 paired reset：数据泄漏。
- 固定训练噪声：可能记忆有限标签；本节每batch重新抽样。
- 验证噪声每epoch任意变化：影响checkpoint比较；固定selection噪声并另设audit。
- 认为高k低noise loss证明恢复准确：分母放大与单位换算不可忽略。
- 认为state一定提供route oracle：同初始状态仍有两种合法选择。
- 把epsilon_hat当动作：它是无量纲噪声估计；还需要采样、反归一化与控制合同。

机器人应用：状态约束动作分布，使计划与当前位置/速度/目标相容；后续chunk保持时间关系，闭环执行再处理反馈和安全。

## Interview Capsule / Must Remember

30秒：Conditional denoiser输入带噪动作、噪声等级和机器人状态，预测训练程序注入的epsilon。训练每次重新加噪，以MSE监督；网络不看干净动作或route标签。离线loss通过后，还要验证反向生成和真实执行。

2分钟：先写forward公式与epsilon MSE，再解释归一化、k与机器人时间、9维输入；说明paired split和fresh训练噪声、fixed selection与独立noise audit；对照关闭状态、错配状态与wrong-k，解释不同实验含义；用A0误差放大式说明低loss为何不足；最后划清单步denoiser、chunk sampler、closed-loop policy三个阶段。

Must Remember：输出是噪声；k提供噪声语境；o提供任务语境；监督答案不能进网络；audit新噪声不等于新状态holdout。

## My Verification / Run → Modify → Explain

Engineering Complete，Learning待本人验证，根README为唯一状态来源。
Run：运行上方默认命令，查看loss.csv/denoising.png和results.json的四个k区间。
Modify：先预测关闭状态条件的结果，再保持seed与epoch不变重训：

```bash
python examples/22_diffusion_policy/conditional_denoiser.py --conditioning none
```

比较trained epsilon MSE及恢复RMSE；两次输出路径不同，保留全部结果。
Explain（五问）：
1. 网络9维输入分别是什么，输出是什么？为什么a_0和真实epsilon不能作为输入？
2. 为什么需要k和o？当前state是否一定能确定上下路线？
3. fresh训练噪声、fixed validation噪声、独立audit噪声各解决什么问题？audit是否是新状态test？
4. 为什么高k的epsilon MSE很低，却不能直接证明动作恢复或安全生成成功？
5. 关闭状态重新训练与冻结模型打乱状态有什么区别？这次Modify说明什么、不能说明什么？

下一小任务S21.4 action-sequence diffusion / sampling等待明确请求；不自动实施。
