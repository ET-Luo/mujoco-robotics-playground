# S20.3 — Necessary Transformer concepts

[状态唯一来源](../README.md#stage-20--act--temporal-policy) · [代码](../examples/21_act_temporal_policy/attention_basics.py)
前置：S20.1 window/mask、S20.2 chunk；约0.5～2小时。本课只做手算与小前向，不训练ACT。

## Problem → Why → Intuition

MLP把输入向量直接映射为动作chunk；Transformer让每个token按内容从其他token读取信息。
理解ACT前，先回答：token如何带时间信息？attention在什么维度上归一化？缺失帧怎么屏蔽？

Q像当前token提出的检索请求，K像候选token的索引，V是实际取回的信息。
Q/K的相似度决定权重，V负责被加权的内容。它们都是网络学出的表示，不是机器人世界坐标中的向量。
本课先用Q=K=V的小矩阵做算术，再用随机线性投影验证真实PyTorch多头实现。

## Core Concepts / Mathematics

**Token**是序列中的一个向量。本课一个token代表一个时刻/槽位的特征，不是每个标量通道一个token。
真实o=(x,y,vx,vy,gx,gy)中位置与goal的单位为m，速度为m/s；先按train统计归一化，再由Linear(6,D)嵌入。
本实验直接定义无量纲token，不加载真实示范，不把random output解释为米单位动作。
B=batch数，S=输入token数，H=未来槽位数，D=embedding宽度，h=head数，d_k=D/h。

```text
X: (B,S,D)
Q=X W_Q, K=X W_K, V=X W_V   # self-attention，同输入、不同投影
scores = Q K^T / sqrt(d_k)
P[i,j] = softmax(scores[i,:])[j]   # 行是query，列是key；沿key归一化
output = P V
```

通常Q/K/V各投影到D，再拆成(B,h,S,d_k)，每头权重(B,h,S,S)。
拼接各头得到(B,S,D)，再乘W_O。不同头可学不同关系，但不保证某头有可命名的物理含义。
除sqrt(d_k)用于控制点积随维度增长的尺度，避免softmax过于饱和；不是物理单位换算。

**手算：**三个token X=[[1,0],[0,1],[1,1]]，Q=K=V=X，d_k=2。
第一query的分数=[1/√2,0,1/√2]；softmax=[.401112,.197776,.401112]。
输出=.401112[1,0]+.197776[0,1]+.401112[1,1]=[.802224,.598888]。
softmax每行和为1；输出是V行的加权组合，不能把attention权重直接当动作或因果解释。

**位置编码：**不带位置或结构mask的共享self-attention具有置换等变性：改变token顺序，输出只随之换序，
缺少“哪个时刻”的信息。加P_pos可把内容和所在槽位结合：

```text
PE[p,2i]   = sin(p / 10000^(2i/D))
PE[p,2i+1] = cos(p / 10000^(2i/D))
input = embedded_content + PE
```

p是离散索引，不是秒或米；代码要求D为正偶数。固定PE、仅交换内容，通常不再等同于简单交换输出。
本课用固定sin/cos编码，不训练位置参数。后续可用learned query embedding，当前先展示必要机制。

## 三种 mask 与 Math-to-Code

| mask | shape示例 | 目的 | 本课布尔含义 |
| --- | --- | --- | --- |
| dataset valid | (B,S)/(B,H) | 标记真实观测/标签 | True=真实 |
| MHA key_padding_mask | (B,S) | 不从padding key读取 | True=屏蔽，因此取~valid |
| MHA causal attn_mask | (S,S) | query i不能读j>i | True=屏蔽上三角 |
| SDPA attn_mask | 可广播到(B,h,L,S) | 控制可读query-key pair | True=允许 |
| action loss mask | (B,H) | 只对真实动作标签监督 | 本仓库True=计loss |

API的布尔极性依据[PyTorch MultiheadAttention文档](https://docs.pytorch.org/docs/stable/generated/torch.nn.MultiheadAttention.html)
与[SDPA文档](https://docs.pytorch.org/docs/stable/generated/torch.nn.functional.scaled_dot_product_attention)。

NumPy `attention(q,k,v,allowed)`返回(output,weights,scores)，不修改输入。
屏蔽分数设−∞，再沿key行softmax；为了数值稳定，先减每行最大值。
每query必须至少有一个可见key；全屏蔽行在手写实现明确拒绝，不依赖各API对空行的不同处理。
左padding加causal时可能产生全屏蔽的早期query，要跳过/另行处理这些无效query，不能盲目softmax。
mask也不能修复已含NaN的Q/K/V；此函数拒绝非finite输入。

`nn.MultiheadAttention(D,h,batch_first=True)`返回模块；调用query/key/value返回新output及weights，
内部做Q/K/V投影和输出投影；调用参数不是已经投影好的Q/K/V。
`average_attn_weights=False`返回逐head weights(B,h,L,S)；默认平均head时会隐藏head维度。
`F.linear(tokens,in_proj_weight)`显式展开QKV投影，结合NumPy每头计算与concat/W_O，核对MHA结果。
`F.scaled_dot_product_attention`直接接受已投影并拆head的Q/K/V，返回各头加权输出；不负责W_Q/K/V或W_O。
两者本课dropout=0、float64、CPU，以便核对数值。

key padding屏蔽的是**列**，不自动清除padding query的**行**：第三query仍会读前两个key，输出[.5,.5]。
本课block后显式将invalid query的encoded row设0；cross-attention仍保留key mask。
loss mask只屏蔽损失，不能阻止padding参与前向信息混合。

## Tiny forward / Robotics Context

代码用B1/S3/D4/h2，小型post-norm block：

```text
h = LayerNorm(X + MultiHeadAttention(X,X,X))
encoded = LayerNorm(h + FFN(h))
FFN = Linear(4,8) -> ReLU -> Linear(8,4)
queries = PE(未来槽位0…H−1)             # 不读取未来expert动作
decoded = CrossAttention(queries, encoded, encoded)
action_slots = Linear(4,2)(decoded)
```

残差保留token输入，LayerNorm沿每token的D个通道归一化并有可学习仿射参数，FFN逐token共享非线性变换。
这些步骤保留序列长度；FFN不混合不同token，attention负责token间信息交互。
self-attention的Q/K/V来源相同；cross-attention的Q来自输出槽位，K/V来自观测memory，L可以不同于S。

| 张量 | 默认shape | slots7 |
| --- | --- | --- |
| input/encoded | (1,3,4) | 相同 |
| Q per-head（self） | (1,2,3,2) | 相同 |
| self weights | (1,2,3,3) | 相同 |
| future queries | (1,5,4) | (1,7,4) |
| cross weights | (1,2,5,3) | (1,2,7,3) |
| action_slots | (1,5,2) | (1,7,2) |

parallel action槽位query仅提供位置，不偷看未来真实标签，所以不因槽位代表未来就必须加autoregressive causal mask。
本课causal是独立教学probe；当前cross-query之间没有self-attention，也没有完整decoder堆叠。
这些结构展示将来ACT如何把memory映射到动作槽位，但random weights输出没有policy性能含义，也没有CVAE。

## Minimal Experiment / Expected and Actual Result

依赖requirements已有NumPy、torch==2.14.0+cpu，无local helper/MuJoCo/Matplotlib导入、新安装或旧tmp输入要求。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/attention_basics.py
```

新ignored tmp/s20_3/run_*包含attention.npz/results.json（矩阵、shapes、probe结果与source/artifact hash）。
显式--output必须新目录。终端会打印手算矩阵，便于逐行对照。

2026-10-11 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；Python3.12.14/mujoco环境，
NumPy2.5.3与Torch2.14.0+cpu metadata、真实模块路径与MHA/SDPA API通过，CUDA build None，无安装。
命令：默认命令加`--output tmp/s20_3/default`；加`--slots 7 --output tmp/s20_3/slots7`，均exit0。

| 预期probe | 实际（两组相同） |
| --- | --- |
| 改padding值，有效query输出不变 | max change=0 |
| 不使用padding mask会污染输出 | max change=1000.598888 |
| causal阻止未来token影响过去输出 | earlier max change=0 |
| 无位置时置换等变 | 数值核对通过 |
| 固定位置仅换内容改变关系 | max difference=.807034 |
| 手写每头计算匹配MHA | max error2.776e−17 |
| SDPA allow-mask匹配手写结果 | 1e−12容差通过 |

独立scalar loop重算保存scores/softmax/output、hash、所有weights行归一化和invalid query清零通过；
slots5→7保留前5槽输出与所有encoder结果（1e−15容差），三CLI非法/已有目录均exit2；
全空/shape错/非bool三mask拒绝通过。Python3.11仅语法核验，实际3.12。
slots增加只扩展query数；Linear(4,2)参数共享，参数shape不随slots改变，和S20.2的2H输出层不同。
无训练、示范数据、动作反归一化、MuJoCo rollout、GUI或ACT性能实验。

## Explanation / Failure Cases / Must Remember

- softmax沿query或feature维度做，会得到不同算法；每query的key权重应和为1。
- True的意思取决于API；不要从变量名mask猜极性。
- padding key mask不抹去padding query输出，attention mask也不代替loss mask。
- 无PE不等于输出完全一样：输入置换，输出随之置换；不能把“等变”说成“不变”。
- causal保护过去不读未来，与不从padding读取是两个不同约束；parallel chunk prediction不等于自回归。
- 随机前向验证shape/公式/API；不能证明学习了机器人时间关系、可安全执行或已完成ACT。

**30秒 Interview：**Transformer把序列表示成token，通过Q/K相似度得到沿key归一化的权重，再加权V。
位置编码提供槽位信息，多头学习不同关系，mask限制信息可见性；需要区分padding、causal与loss mask。

**2分钟 Interview：**从(B,S,D)→(B,h,S,d_k)→(B,h,L,S)→concat/W_O讲形状与计算，
解释残差/LayerNorm/FFN及self/cross来源区别；未来槽位query不输入expert标签。
指出PyTorch mask极性差异、invalid query处理与全屏蔽行；本课只有random forward而非ACT策略。

## My Verification / Run → Modify → Explain

Engineering与Learning状态以根README为准；2026-10-11 本人确认实验、预测与五项Explain，Run/Modify/Explain全部完成，Learning Mastered。
解释覆盖QKV与缩放、self置换等变与固定query的cross memory置换不变、mask极性与query输出、逐head形状及并行槽位无标签泄漏。
精度边界：本课cross query彼此独立；其他结构允许query间交互也不自动等于未来标签泄漏，需检查实际输入与任务约束。
本次仅同步文档，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-11工程验证。

**Run：**运行默认命令，手算第一query分数、softmax、输出，对照results.json与打印矩阵。

**Modify：**先预测再加`--slots 7`运行；哪些维度变化、哪些参数shape不变？前5槽输出为何应保留？
本课cross-attention query独立，没有query间self-attention，所以增加后续槽不会改变前面槽的输出。

**Explain（五问）：**

1. Q/K/V各做什么？为什么softmax沿key归一化，为什么要除sqrt(d_k)？
2. 无位置编码时token换序，输出是什么关系？位置编码如何提供时间槽位信息？
3. dataset valid、MHA key_padding_mask、SDPA布尔mask的True分别意味着什么？key mask为何不自动清零query？
4. self/cross-attention的Q/K/V分别来自哪里？B1/h2/H5/S3时cross weights为何是(1,2,5,3)？
5. 残差、LayerNorm、FFN分别起什么作用？未来action query为何不必因“未来”就加causal mask？

本轮止S20.3；S20.4 ACT CVAE latent / KL / prior等待明确请求。
