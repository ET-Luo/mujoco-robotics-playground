# S21.4 — Action-sequence diffusion / sampling

[状态唯一来源](../README.md#stage-21--diffusion-policy) · [代码](../examples/22_diffusion_policy/sequence_diffusion.py)
前置S21.3；0.5～2小时。主概念：从高斯噪声经反向过程联合生成一段动作，不再依靠干净动作造推理输入。
S21.5才执行生成策略；本节不运行闭环或比较成功率。

## Problem → Why → Intuition

单步denoiser能预测噪声，却不能直接给出一段可执行计划。需要把动作窗口作为联合随机变量，学习整段epsilon，再从随机噪声逐步生成chunk。
直觉：每次模型都看全部槽位，能利用时间关系；但没有结构或损失保证输出必然平滑。
依据：[DDPM论文](https://arxiv.org/abs/2006.11239)与[作者的posterior/sample实现](https://github.com/hojonathanho/diffusion/blob/master/diffusion_tf/diffusion_utils.py)。本节采用posterior variance、epsilon参数化，无clip，索引k0为干净状态。

## Core Concepts / contract

| 量 | shape / meaning / units / frame |
| --- | --- |
| o_t | (B,6)，归一化当前world XY位置、速度、goal；history=1 |
| A0 | (B,H=10,Da=2)，当前tick起的未来reference，归一化、无量纲 |
| Ak / epsilon_hat | (B,10,2)，带噪chunk / 联合噪声预测 |
| valid | (B,10) bool，真实动作槽；False是episode末尾padding |
| k | 每batch样本独立1…100，整段共用一个k；不是动作槽h或机器人时间t |
| raw A | (B,10,2)，反归一化world XY绝对位置reference[m]；不是ctrl或qpos |

H10对应50Hz下10个reference槽，覆盖.2s控制区间；K100是生成迭代数。
MLP把chunk flatten成20维，加k/K1维、state6维、boundary mask10维，共37维；37→128→128→20，23956参数。
输出reshape回10×2；不是逐槽独立模型，也没有使用Transformer/外部scheduler框架。

复用S21.3 collect重新生成同B轨道2400train/600validation行，paired reset split8/2/2；test不生成。
用pair_id与side构造episode_id，只为窗口分组；不传入网络。S20.1 make_windows只在同episode取未来动作，normalized padding=0。
train总24000槽，23280有效、720padding；每episode最后9个anchor有padding。
mask是明确的episode边界元数据，不是route指令；尾部mask会透露剩余长度。本节离线probe知道边界。
部署时应根据真实任务剩余预算构造mask，通常完整chunk全True；不能从未知未来成功时刻读取mask。

## Mathematics

```text
Ak = sqrt(alpha_bar[k]) A0 + sqrt(1-alpha_bar[k]) epsilon
L = sum_(b,h,d) valid[b,h] * (epsilon_hat-epsilon)²
    / (Da * sum_(b,h) valid[b,h])
```

代码以boolean selection求有效scalar均值。padding既从损失排除，也从带噪输入和反向输出置零；mask作为输入明确告诉模型哪些槽不存在。
不同于用padding0作为真实标签：归一化0表示train mean，并非物理零reference，更不是不存在动作。
每batch新的k/epsilon；linear beta .0001… .1、alpha=1-beta、alpha_bar累计乘积。

从AK~N(0,I)开始，k=100…1：

```text
mu_theta = (Ak - beta[k]/sqrt(1-alpha_bar[k]) * epsilon_theta(Ak,k,o,mask))
           / sqrt(alpha[k])
posterior_variance[k] = beta[k] * (1-alpha_bar[k-1]) / (1-alpha_bar[k])
A_(k-1) = mu_theta + sqrt(posterior_variance[k]) z_k    # k>1
A0 = mu_theta                                        # k=1
```

z_k独立标准高斯；最终k1无新增噪声，posterior variance[1]=0。
这是完整DDPM相邻转移，不能只跳过循环步数而保留原相邻公式；少步采样需另推正确日程，留后续ablation。
mu公式等价于高斯posterior均值中用A0_hat替代A0。验证中用已知A0构造epsilon核对该恒等式；实际sampler不读真实A0/epsilon。
alpha_bar[100]=.005619，末端仍有少量信号；标准高斯prior是近似而非精确q(AK)。
不clip generated或A0_hat：保留原始误差；归一化范围也不天然是[-1,1]。

```text
raw_A[h,d] = generated_normalized[h,d] * train_scale[d] + train_mean[d]
```

有效槽返回米单位reference，无效槽标NaN用于保存/可视化，禁止把它送给控制器。
部署执行前缀需要显式检查有效mask与finite值；本节不执行。

## Math-to-Code / APIs

`ChunkDenoiser.forward`屏蔽padding、拼接Ak/k/o/mask，返回(B,10,2)epsilon。
`forward_noise`把每样本alpha_bar gather成(B,1,1)，广播到整段；同段噪声各scalar独立。
`masked_loss`返回有效scalar MSE、拒绝全空mask。
`reverse_step`返回新的A_(k-1)，不推进物理时间；`sample`固定观测/边界，返回最终chunk及全部101个状态、100个epsilon预测和增量。
`torch.Generator`隔离采样随机流；`torch.inference_mode`禁用梯度；`model.eval`进入评价模式。
Adam/backward与S21.3相同，CPU1thread、seed214、lr.001、batch128、150epoch。
selection固定validation noise21401选最低loss（含epoch0）；audit noise21402仍是同validation状态，非新状态holdout。

依赖requirements已有NumPy/torch==2.14.0+cpu/MuJoCo/Matplotlib，无新安装。
helpers：conditional_denoiser.collect→dual_path_task→episode_contract→cartesian_spring，以及noising_basics与sequence_dataset.make_windows。
expert采集仍创建MjModel/MjData；mj_forward刷新派生量，mj_step原地积分实际状态。采样过程完全不调用mj_step。
checkpoint包含task/H/K、数据hash、架构、train normalizer、beta/alpha_bar与weights；加载先核验，不把S21.3 H1模型当chunk模型。

## Minimal Experiment / Expected Result

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/sequence_diffusion.py
```

自动新目录`tmp/s21_4/run_*`，显式--output必须尚不存在。
输出数据/normalizer/window/protocol、chunk.pt、loss.csv、audit.npz、samples.npz、results.json、sampling.png。
固定probe：4validation episode × anchor t0/30/60/140/149 ×4sample=80，所有attempt保留，不挑最好sample。
同初始状态上下episode相同，但配对expert标签不同；样本RMSE只是相对配对示范的距离，不是生成质量/模式覆盖的完整评分。
预期loss改善、采样finite、同seed同权重逐值重现、不同seed可变；不预设多模态覆盖或平滑成功。

## Actual Result / Explanation

2026-10-12 Zero，WSL2 Ubuntu24.04.5/kernel6.18.33.2；mujoco环境Python3.12.14、NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.13.0/Matplotlib3.11.2 metadata/真实paths/API通过，无安装。
命令`python examples/22_diffusion_policy/sequence_diffusion.py --output tmp/s21_4/final`；
再`--checkpoint tmp/s21_4/final/chunk.pt --sample-seed 215 --output tmp/s21_4/final215`，均exit0。
150epoch，best146，selection .082275（epoch0 1.005013）；audit masked epsilon MSE .089860，zero baseline .999722。
seed214/215各80sample，相对配对expert scalar RMSE3.685/3.713mm；weights和audit指标一致，Modify不重训。
最终final trainloop3.191s、batch80采样.030845s；seed215 batch.033227s，均是本机观察、不含data/绘图，不是batch1实时延迟或控制预算证明。
最终命令实际时间见各results.json；性能数字不作严格复现要求。

全部100步用独立NumPy公式重算，两组窗口每anchor逐值核验episode边界、mask/标签；独立重算audit/sample指标与反归一化通过。
posterior均值解析恒等式(k1/2/50/100)、k1无噪声、empty mask拒绝、padding loss/input不变性、reload/同seed重现、冻结Modify与3.11语法通过。
首轮图横轴自动排序使k100在右，已改invert_xaxis并完整重跑，时间方向从左100到右0；不把旧图方向计正确。
默认图已查看；初始状态sample明显抖动且可跨上下符号。这是模型未产生稳定序列的实际现象，不据低loss掩盖。
可能与有限模型、数据/训练、初始歧义或sampler近似有关，未因果定位。不同seed改变输出不等于覆盖两个合法全路径。
无test/GUI/闭环执行/碰撞率/完整路线覆盖验证；H10只覆盖局部未来，不能据此判绕过整个圆盘。

## Failure Cases / Robotics Context

- 窗口跨episode或把padding当动作：标签污染，必须检查边界和mask分母。
- k/时间h错位：整段共同加噪等级与各槽未来时间是两个轴。
- 只使用A0_hat一次：不是从prior进行完整反向生成。
- 最后一步仍注入噪声、posterior variance误用beta、跳步不改公式：采样算法改变。
- clip/选择最好sample让图好看：隐藏误差，本节全部attempt保存。
- mask来源不合法：不能用未来终止oracle构造部署mask。
- 输出NaN padding被执行或把reference当ctrl：执行接口错误。
- 低epsilon loss被当安全证据：实际sample已有抖动，后续还需闭环轨迹验证。

机器人应用：联合生成局部动作计划，再执行短前缀、重新观测与重采样。H、K与执行前缀E分别控制计划长度、计算迭代和反馈频率；E尚未执行。

## Interview Capsule / Must Remember

30秒：把动作chunk作为联合随机变量加噪，以mask排除episode末尾padding，条件MLP预测整段epsilon。推理从高斯prior迭代DDPM相邻反向步骤，最后不加噪，再按train normalizer变回reference米单位。离线生成不是闭环成功。

2分钟：说明H/K/t三个轴；画(B,H,Da)到flatten模型的映射；写mask loss和posterior mean/variance；解释k1边界、seed reproducibility与有限prior近似；说明反归一化、padding/finite检查、short prefix和闭环反馈。最后承认生成抖动及尚未验证完整模式覆盖/安全。

Must Remember：联合chunk；合法边界mask；posterior variance；k1无噪声；反归一化才是reference；sampler不读真实A0。

## My Verification / Run → Modify → Explain

Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain），根README为唯一状态来源。
解释覆盖H/K/t、masked scalar分母、相邻DDPM更新、生成监督边界、reference与ctrl以及seed实验的证据范围。
精度补充：本节k1 posterior variance为零，所以最后不注入噪声；生成A0并不保证是准确或安全动作。归一化padding0对应train mean，不是不存在动作的物理命令。
Run：运行默认命令，检查loss/sample图、samples.npz的trace(101,80,10,2)与mask。
Modify：先预测换seed会改变什么，把下面checkpoint路径换成Run打印的实际目录，冻结weights：

```bash
python examples/22_diffusion_policy/sequence_diffusion.py \
  --checkpoint tmp/s21_4/final/chunk.pt --sample-seed 215
```

预期audit和weights不变，生成sample变化；不从两组挑最好seed作为默认策略。
Explain（五问）：
1. H、K、机器人tick t分别代表什么？为何joint chunk与单步denoiser不同？
2. padding为何需要mask？有效scalar分母是什么？归一化0为何不能作为不存在动作的物理命令？
3. reverse mean和posterior variance各做什么？为什么k1不加噪？为何不能任意跳过k？
4. 生成时为什么不用真实A0/epsilon？反归一化后的量是什么，与ctrl有什么区别？
5. 换seed验证了什么？为什么局部sample RMSE/低noise loss不足以证明平滑、完整模式覆盖与闭环安全？

下一小任务S21.5 closed-loop Diffusion Policy等待明确请求；不自动实施。
