# S21.2 — Diffusion basics / forward noising

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [代码](../examples/22_diffusion_policy/noising_basics.py)
前置S21.1；0.5～2小时。本节只学习forward noising、schedule与epsilon prediction目标；训练模型在S21.3，反向采样在S21.4。

## Problem → Why → Intuition

S21.1说明模式平均可能不安全。要学习动作分布，先定义一个可控制的信息破坏过程，再让模型学习从受损样本识别噪声。
直觉：逐步降低动作信号、提高随机噪声，最终使不同动作接近共同的高斯分布。
加噪不是对机器人下达随机指令，而是训练时处理归一化数据。
依据：[DDPM论文](https://arxiv.org/abs/2006.11239)及[作者实现](https://github.com/hojonathanho/diffusion)。

## Core Concepts / shapes / units

| 符号 | 含义 |
| --- | --- |
| A0 | 干净归一化动作chunk，(H=8, Da=2)，无量纲；本节合成数组，无实际米单位映射 |
| h | chunk内动作槽，未来执行顺序 |
| k | diffusion index，信息破坏程度；不是h，也不是MuJoCo simulation time |
| beta[k] | 第k次转移加入的噪声方差，无量纲，0<beta<1 |
| alpha[k] | 1-beta[k]，单步信号方差比例 |
| alpha_bar[k] | 前k个alpha的乘积，累计信号方差比例 |
| epsilon | 独立标准正态数组，与A0同shape；闭式公式中的累计噪声 |

代码用长度K+1数组：k=0干净，beta[0]=0、alpha_bar[0]=1；有效转移k=1…K。
本节固定K=100、linear beta从.0001到.1，突出可见的信息衰减。这是教学日程，不是推荐policy超参数。
真实任务应使用train-only动作normalizer；不同量纲不能直接混入单位高斯噪声。

## Mathematics

单步Markov转移（z_k相互独立）：

```text
A_k = sqrt(alpha_k) A_(k-1) + sqrt(beta_k) z_k
z_k ~ N(0,I)
```

递推条件均值为sqrt(alpha_bar_k)A0；条件方差v_k=alpha_k v_(k-1)+beta_k=1-alpha_bar_k。
独立高斯线性组合仍是高斯，因此可直接跳到任意k：

```text
q(A_k | A0) = N(sqrt(alpha_bar_k) A0, (1-alpha_bar_k) I)
A_k = sqrt(alpha_bar_k) A0 + sqrt(1-alpha_bar_k) epsilon
SNR_k = alpha_bar_k / (1-alpha_bar_k)  # k=0无噪声，SNR无限
```

sqrt是标准差/幅度系数，不可直接用方差代替。
闭式epsilon不是最后一步的z_k，而是所有独立增量的加权合成噪声除以sqrt(1-alpha_bar_k)后得到的标准高斯变量（k>0）。
直接采样和逐步采样具有相同条件边缘分布，不要求同seed时逐元素相同。
图中各k复用一个epsilon以隔离schedule效果；这组快照不是Markov轨迹。

训练目标准备：随机选k∈{1,…,K}、epsilon~N(0,I)，构造Ak；预测epsilon_hat，最小化mean((epsilon-epsilon_hat)^2)。
训练标签可得，因为epsilon由训练程序自己生成；部署时没有真实epsilon可读。
k=0仅用于边界检查，不作为本节讨论的噪声训练样本。
真实模型依赖Ak、k，条件策略还依赖观测；本节只比较oracle、zero和固定偏差预测，不训练网络。

```text
A0_hat = (Ak - sqrt(1-alpha_bar_k) epsilon_hat) / sqrt(alpha_bar_k)
A0_hat - A0 = -sqrt((1-alpha_bar_k)/alpha_bar_k) (epsilon_hat-epsilon)
```

噪声预测MSE相同，在不同k对应的干净动作误差不同。高噪声时除以小信号系数会放大误差。
这是A0代数估计，不是完整反向采样器；oracle恢复不能证明从未知噪声生成动作。

## Math-to-Code / APIs

`noise_schedule(K,beta_end)`返回beta和alpha_bar，均(K+1,)；检查范围并用`np.cumprod(1-beta)`累计乘积。
`np.random.default_rng(seed)`返回独立随机数生成器；`standard_normal(shape)`返回同shape标准高斯样本。
`forward_noise(clean,epsilon,alpha_bar[k])`返回新Ak数组，输入不被修改。
`recover_clean`返回A0估计；oracle传入实际生成的epsilon仅验证代数。
NumPy广播允许标量系数作用于(H,Da)每个槽/通道。`np.savez`写可重算数组；Matplotlib Agg写PNG，无GUI。
依赖仅requirements已有NumPy/Matplotlib；无local helper、Torch训练、MuJoCo调用、数据集或test标签。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/noising_basics.py --output tmp/s21_2/verified
```

输出`arrays.npz`、`results.json`、`noising.png`；均位于ignored tmp。
预期alpha_bar单调下降；k0等于干净数组；oracle恢复到浮点精度；epsilon+.1误差符合解析式。
另用50000份独立逐步Markov样本检查末端条件均值/方差，均值容差6个标准误差，方差容差10个近似标准误差。
有限样本检验支持分布推导，不是分布完全一致的证明。

## Actual Result / Explanation

2026-10-12，Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；mujoco环境Python3.12.14，NumPy2.5.3/Matplotlib3.11.2真实import路径均在环境内，无安装。
上面默认命令及下方Modify命令均exit0，checks_passed=True。

| beta_end | alpha_bar[100] | oracle最大绝对误差（所选k） | epsilon偏差+.1的末端A0 RMSE |
| --- | --- | --- | --- |
| .1 | .005618761 | 2.443e-15 | 1.330320 |
| .02 | .363563248 | 2.220e-16 | .132308 |

两组epsilon预测偏差MSE均.01；zero预测MSE为1.463380（16个元素的有限样本，不要求等于总体期望1）。
Markov末端均值最大误差.008711/.006394，方差最大误差.011303/.012501，全部在预设容差内。
默认信号幅度仍为sqrt(.005619)≈.075，并非完全无信号；.02日程幅度≈.603，不能称末端接近纯噪声。
默认PNG已目视检查。未运行robot rollout、GUI、denoiser训练或反向采样；不据此判断碰撞/模式覆盖。

## Failure Cases / Robotics Applications

- 把alpha当alpha_bar：忽略累计衰减。
- 用beta当标准差：方差错误，噪声日程含义改变。
- 混淆k/h：去噪迭代次数不等于执行动作数或物理时间。
- 对米、速度、角度直接同尺度加噪：需要先按训练数据归一化。
- 截断/clip加噪数组：改变高斯forward分布；安全约束需在后续生成/执行阶段单独验证。
- 把oracle恢复当生成：部署没有真实epsilon，预测误差特别在高噪声处不能忽略。
- 极端schedule使alpha_bar数值下溢：A0恢复分母不稳定；本实验只验证上述两组日程。

机器人语境：以后对完整动作chunk加噪，让去噪模型利用各槽关系和当前观测生成一致计划；真实安全性仍取决于闭环执行。

## Interview Capsule / Must Remember

30秒：Forward diffusion把归一化动作按已知日程逐步变成高斯噪声。累计乘积alpha_bar给出任意k的闭式采样，训练模型预测程序生成的epsilon。k不是机器人时间。

2分钟：先说多模态动作均值的局限；写单步高斯转移并推导累计均值/方差；解释sqrt系数、alpha_bar与SNR；说明epsilon标签如何得到、k为什么是输入；再写A0估计及误差放大式。最后说明oracle只验证数学，模型训练、反向采样和碰撞评价需要后续分别验证。

Must Remember：累计乘积；标准差取平方根；累计epsilon不同于最后增量；归一化；oracle不等于生成。

## My Verification / Run → Modify → Explain

Engineering已完成，Learning状态见根README。2026-10-12 本人确认实验与预测完成：Run/Modify完成；五项Explain均通过，Learning Mastered。解释覆盖累计信号与平方根、k与h、累计噪声与单步增量、训练标签与部署oracle边界、恢复误差放大与末端信号残留。

Run：运行上述命令，查看JSON和PNG，用arrays.npz对照k=0/50/100。

Modify：保持seed、K和干净数组不变，仅减小beta_end。先预测信号残留和误差放大方向，再运行：

```bash
python examples/22_diffusion_policy/noising_basics.py --beta-end .02 --output tmp/s21_2/beta002
```

Explain（五问）：
1. 为什么累计信号用alpha_bar而非alpha？为什么乘sqrt？
2. k=50和h=5分别表示什么？是否对应50个机器人控制tick？
3. 闭式epsilon与单步z_k有什么区别？为什么两种采样只要求分布相同？
4. 训练时为什么有epsilon标签，部署时为什么不能用oracle恢复？
5. epsilon预测MSE相同，为什么不同k的A0误差不同？beta_end减小后有哪些实测变化？

完成后停止于S21.2，下一小任务S21.3 conditional tiny denoiser等待明确请求。
