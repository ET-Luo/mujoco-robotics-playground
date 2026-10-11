# S20.5 — Tiny state-based ACT / closed-loop

[状态唯一来源](../README.md#stage-20--act--temporal-policy) · [代码](../examples/21_act_temporal_policy/tiny_act.py)
前置：S20.1–4；0.5～2小时学习规模。主概念：把CVAE训练和Transformer chunk预测接入真实闭环。

## Problem → Why → Intuition

S20.3只有随机Transformer前向，S20.4用MLP验证latent机制；本课组合成可训练的tiny state ACT。
训练posterior看expert chunk，推理只用实际当前状态和prior z=0，decoder并行输出短动作计划。
计划仍要经过reference解码与低层反馈，下一次预测必须读取机器人实际状态。

直觉：posterior像训练辅助老师，Transformer decoder像规划学生；上机执行时老师看不到未来答案，学生只能根据当前状态规划。
保留CVAE、KL、Transformer encoder/decoder和chunk；不增加视觉、UR5e、接触或RL。
本课是低维state教学变体，不是ACT原论文完整性能复现。算法与zero-prior依据[作者实现](https://github.com/tonyzhaozh/act/blob/main/detr/models/detr_vae.py)。

## Core Concepts / Mathematics

当前o∈R^6=(x,y,vx,vy,gx,gy)，world frame，位置/goal [m]、速度 [m/s]；
a∈R^2是world XY绝对位置reference [m]。S19.3训练集normalizer冻结；网络内部均用归一化、无量纲输入/输出。
B=batch，H=10，D=32，heads=2（每头16），latent=4，history=1。

```text
posterior q_phi(z|o,A,valid):
  [CLS, embed(o), embed(a_0)…embed(a_9)] + fixed positions
  -> TransformerEncoder -> CLS feature -> mu/logvar (B,4)
z = mu + exp(0.5*logvar)*epsilon
memory = TransformerEncoder([embed(o), embed(z)] + learned type embeddings)
queries = 10 learned slot vectors (B,10,32)
A_hat = Linear(32,2)(TransformerDecoder(queries,memory))
```

CLS表示汇总token，始终真实；posterior序列(B,12,32)，仅action padding的key被屏蔽。
先清理padding action，再嵌入；token position提供序列槽位信息。不能只在loss屏蔽padding而让encoder读取垃圾。
memory序列(B,2,32)的两个token分别来自o、z，用learned type向量区别。
decoder先做query self-attention，再cross-attention读取memory，再逐token FFN；每部分包含残差与LayerNorm。
本课三个stack各1层，FFN宽64、ReLU、post-norm、dropout0，共31146可训练参数。

| 张量 | shape |
| --- | --- |
| posterior tokens / key padding mask | (B,12,32) / (B,12) |
| mu/logvar/z | 各(B,4) |
| observation+latent memory | (B,2,32) |
| learned queries / decoder output | (B,10,32) |
| query self / cross attention weights（概念形状） | (B,2,10,10) / (B,2,10,2) |
| action chunk / label valid | (B,10,2) / (B,10) |

未来槽位queries不含真实未来动作，query之间双向attention不会自动泄漏标签；本课不加causal mask。
与S20.3的独立cross-query不同，这里queries可交互；不能套用“增加query不影响旧query”的结论。

训练目标沿用S20.4：masked normalized MSE + .01 KL，KL sumlatent/meanbatch，logvar限[-10,10]。
重参数化使梯度传入posterior Transformer；训练posterior与部署prior路径明确分开。

## Math-to-Code / APIs

`TinyACT`构造投影、posterior encoder、memory encoder、decoder、action head与learned queries。
`nn.TransformerEncoderLayer(32,2,64,batch_first=True)`构造self-attention+FFN+norm层；
`TransformerEncoder(layer,1)`返回单层stack，调用src/optional key mask返回同长度的encoded Tensor。
posterior调用`src_key_padding_mask=True`代表忽略对应key；CLS与状态位置始终False。

`nn.TransformerDecoderLayer`构造target self-attention、memory cross-attention与FFN；
`TransformerDecoder(layer,1)(tgt,memory)`返回(B,H,D)，长度取自tgt，不是memory。
API组成与输入说明见[PyTorch官方文档](https://docs.pytorch.org/docs/2.14/generated/torch.nn.TransformerDecoderLayer.html)。
`register_buffer`保存固定位置编码，不把它当可训练参数；learned queries/type/CLS为Parameter，通过梯度训练。

`posterior(o,A,valid)`返回mu/logvar；`decode(o,z)`返回normalized chunk；`infer(o)`只用z=0，不能接受A。
`predict`只对actual观测归一化，调用infer并反归一化成(H,2)米单位reference。
`load_checkpoint`用weights_only/CPU重载，验证architecture、split hash、normalizer hash与全部统计值。

`MjModel.from_xml_string(XML)`编译旧XY模型；`MjData(model)`为每次rollout创建独立状态。
只在reset写qpos/qvel；`mj_forward`原地刷新派生状态，不推进时间；`mj_step`在env.advance中原地积分状态和time。
每次reference保持20个1ms tick，阻抗反馈每tick更新：F=K*(a−q)−D*qdot，经force cap与gear变成ctrl。
reference不是ctrl，也不是下一帧qpos。

高层执行plan[0…E−1]，然后读取actual o_(t+E)，丢弃旧尾部重新预测；
H10/E5预测调用10Hz、reference50Hz、inner feedback1kHz。E1则每个reference tick预测（50Hz）。
最后不足E步时只执行剩余tick，时间不超T100/2s；不做temporal ensembling，留S20.6。

## Minimal Experiment / Expected Result

依赖requirements已有NumPy/CPU Torch/MuJoCo/Matplotlib；local helper为attention_basics.positions、
cvae_latent数学函数、sequence_dataset与chunk_baseline→behavior_cloning→episode_contract→cartesian_spring。
无新依赖或安装。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/tiny_act.py --prepared tmp/s19_3/verified193
```

prepared是S19.3输出；缺失时按[Stage19 README](../examples/20_learning_from_demonstrations/README.md) S19.2→3重建并替换路径。
不需要S20.4 checkpoint。新ignored tmp/s20_5/run_*打印输出路径，含protocol先写、act.pt、loss.csv、
offline.npz、12trial NPZ、results.json、rollout.png（四个validation reset的ACT轨迹）。
显式--output必须新目录；没有test数组加载。CLI每20epoch打印prior validation MSE。

训练CPU1thread/seed205/100epochs/batch64/lr.001；posterior evaluation noise独立seed2051固定，
**checkpoint按validation prior z=0 masked MSE选**，符合本课实际部署路径；不按posterior或rollout成功选。
训练中每epoch分别记录train/validation posterior R、KL、R+.01KL与prior MSE。
epoch0可被选中；早期checkpoint可能优于更久训练，本课不强迫选最后epoch。

预期：部署全过程不读取expert未来标签；expert/constant/ACT共用同reset/controller/scorer。
执行E变化时weights、offline指标不变，closed-loop访问状态可能改变。

## Actual Result / Explanation

2026-10-11 DESKTOP-781D67A；WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco环境Python3.12.14。
NumPy2.5.3/Torch2.14.0+cpu/MuJoCo3.14.0/Matplotlib3.11.2 metadata、真实模块路径与API通过，CUDA build None，无安装。
默认命令加`--output tmp/s20_5/default`；Modify实际为：

```bash
python examples/21_act_temporal_policy/tiny_act.py --prepared tmp/s19_3/verified193 \
  --checkpoint tmp/s20_5/default/act.pt --execute 1 --output tmp/s20_5/execute1
```

两命令exit0；第二命令**不训练**，--epochs在加载checkpoint时不使用；training_wall_s为null，无新loss.csv。
使用本机默认命令实际打印的输出路径替换checkpoint参数，不假设别的机器有tmp/s20_5/default。

| 指标 | 默认训练 / E5 | 固定checkpoint / E1 |
| --- | --- | --- |
| best epoch | 20（从0…100按prior MSE选择） | 同20 |
| prior-zero / posterior-mean validation RMSE | .829188 / .829245 mm | 完全相同 |
| validation KL nats/sample | .00158380 | 相同 |
| ACT success/attempts | 4/4 | 2/4 |
| 全分母ACT final mean error | .499317mm | .874740mm |
| 每trial预测调用数 | 20 | 100 |
| train loop wall | 25.443s | 本轮未训练 |

train wall包括epoch评分/selection，不含数据构造、checkpoint或rollout；未测batch1推理latency。
expert两组4/4（终态均误差约9.89e−9mm），constant两组0/4（10.895mm）。
成功要求**末10观测每帧**位置<1mm、速度<2mm/s；终态均误差小于1mm不等于所有trial成功。
PNG展示位置误差/1mm线；速度约束仍需看保存NPZ/results，不从位置图单独判成功。

E1反复使用首动作，E5执行多个槽位；改变动作序列又改变访问状态，因此同offline不同成功数是合理的。
具体失败根因未独立分离。更多replan不自动更好，本次结果不证明E5普遍更鲁棒。
H10、单seed、4个既有validation reset、固定动力学/无扰动；没有test/OOD、多seed或多模式评价。
4/4是本次成功，不是对通用ACT能力或真实硬件安全的保证。
本课没有匹配预算的BC/MLP/ACT比较，不能拿历史数字证明算法优劣；统一benchmark留S20.7。

## Validation / Failure Cases / Robotics Context

24保存trial独立48000个Euler步核对actual状态，reset/clock/scorer/hash与plan anchor/offset/action匹配通过。
按每个实际plan anchor重新调用prior infer核对保存chunk；两组首plan相同；offline/weights/normalizer完全一致，expert/constant轨迹相同。
checkpoint保存重载精确一致；CSV argmin/目标组合、offline来源/预测/统计通过；NumPy/Torch MSE核对atol2e−9。
部署全过程将posterior替换为抛异常函数，ACT rollout仍通过；posterior padding改NaN统计不变。
独立reconstruction backward检查posterior encoder/latent stats/memory encoder/decoder/action head均有梯度，padding标签梯度0。
empty posterior labels拒绝；T13/E5最后截断到3步；checkpoint-split mismatch拒绝；四CLI非法/已有output/缺checkpoint均exit2。
Python3.11仅语法检查，实际3.12；两PNG已查看。无GUI验证。

常见失败：漏掉位置/slot区别导致输出难区分；用posterior标签参与infer；normalizer/split错配；
重复plan[0]而不递增offset；replan用预测状态代替actual状态；按rollout挑checkpoint后再称独立结果；
只看终态不检查tail速度；把架构存在与学习成功/泛化保证混在一起。

应用：低维示范学习中的chunk策略，CVAE辅助训练与并行Transformer解码；真实控制仍需把网络输出接入合适快速反馈与约束。

## Interview Capsule / Must Remember

**30秒：**tiny state ACT训练posterior Transformer看观测/动作chunk，估计latent后重建动作并加KL；
部署只用actual状态与prior z=0。Transformer learned queries并行输出10个reference，执行5步后replan，低层持续反馈。

**2分钟：**讲CLS posterior(B,12,32)、mu/logvar(B,4)、state/latent memory(B,2,32)、query(B,10,32)与action(B,10,2)。
说明padding key mask/重建mask、reparameterization与KL、train/infer隔离；prior validation选模，z=0闭环用actual反馈。
E比较固定checkpoint；offline相同不代表rollout相同。保留单seed/有限reset与state变体边界，不声称复现原论文性能。

## My Verification / Run → Modify → Explain

README为Engineering/Learning状态唯一来源；2026-10-11 本人确认实验、预测与五项Explain，Run/Modify/Explain全部完成，Learning Mastered。
解释覆盖CLS posterior与mask、query/memory形状、prior部署与实际动力学、选模路径及固定checkpoint的E消融边界。
精度补充：代码先由requested force除gear得到ctrl，actuator force cap在MuJoCo中生效；独立动力学重算使用等价的clipped generalized force。
本次仅同步文档，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-11工程验证。

**Run：**运行默认训练，核对best_epoch、prior指标、4个ACT trial及expert/constant，查看rollout.png。

**Modify：**先预测E5→E1的调用次数/哪些指标不变，再使用默认生成act.pt加`--checkpoint ... --execute 1`运行。
不重新训练，核对training_wall_s=null、offline与weights不变而轨迹可变。

**Explain（五问）：**

1. posterior的CLS/状态/action tokens如何得到mu/logvar？为什么padding mask只屏蔽action key？
2. decoder的query、memory分别是什么？为什么这是Transformer CVAE chunk policy而不是MLP或纯deterministic Transformer？
3. 部署时z从哪里来？为什么禁用posterior仍应能rollout？normalized输出怎样变成ctrl与actual qpos？
4. 为何按prior validation MSE选checkpoint，且不能用posterior重建或本轮rollout成功代替独立验证？
5. 同weights E5→E1，哪些不变、哪些变化？4/4和2/4分别能说明什么，不能说明什么？

本轮止S20.5；S20.6 Temporal ensembling / ablation等待明确请求。
