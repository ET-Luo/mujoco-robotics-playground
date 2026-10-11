# S20.1 — Sequence dataset / boundaries / mask

状态唯一来源：[根README](../README.md#stage-20--act--temporal-policy)。前置：S19.3 split/normalization与S19.8。
本课约0.5～2小时；[代码](../examples/21_act_temporal_policy/sequence_dataset.py)，仅依赖已声明NumPy。

## Problem → Why → Intuition

BC每次学习 o_t→a_t；短动作序列模型需要 o 的历史→未来动作chunk。
困难先在数据契约：过去不能偷看未来，未来标签不能串入下一条episode，缺失帧不能冒充真实零动作。
把每条episode想成独立书籍；窗口可在书内滑动，到封面只能补空白，不能读下一本。
本课构造样本与损失反例；尚不训练策略，也不执行chunk。

## Core Concepts / Mathematics

令episode有T个动作，anchor t∈{0,…,T−1}，历史长度L，预测长度H。

- O_t=[o_(t−L+1),…,o_t]，shape (L,6)，只包含过去及当前观测。
- A_t=[a_t,…,a_(t+H−1)]，shape (H,2)，未来expert动作是训练标签。
- observation_valid[j]=(t−L+1+j≥0)；action_valid[k]=(t+k<T)。True代表真实token。
- 不存在的token在**归一化空间**填0，索引记−1。0对应训练均值，不表示物理reference为零。

原始o=(x,y,vx,vy,gx,gy)，world frame，位置/目标单位m，速度m/s；a为world XY绝对reference [m]。
归一化后各通道无量纲，使用S19.3保存的train mean/scale，绝不按窗口重新拟合。
50Hz tick相隔.02s；L=4覆盖当前与之前.06s，H=10的reference供未来10tick使用（.2s执行时段）。

对batch B，预测和标签shape (B,H,2)，M为(B,H)：

```text
loss = sum_(b,k,d) M[b,k] * (prediction[b,k,d] - target[b,k,d])²
       / (2 * sum_(b,k) M[b,k])
```

代码先boolean选择valid token再求差/mean，因此padding含NaN也不会污染有效loss。
空mask拒绝，不能除以零。此处是每个有效动作标量等权；不是每条episode或每个anchor等权。
重叠窗口让一个动作作为多个anchor的标签，是训练权重选择，不是新的独立示范。
历史mask用于标明有效输入；未来Transformer的attention mask与loss mask职责不同，不能直接假定同一布尔极性。

## Math-to-Code / APIs

`load_prepared(Path)`返回split metadata及NumPy arrays，检查hash、split不重叠、shape、归一化一致性与连续timesteps；
只打开normalizer/train/validation，不打开test数组。没有MuJoCo/Matplotlib/Torch helper导入。
`make_windows(rows,L,H)`先按episode ID分组，再按timestep排序；返回全新窗口字典，不修改输入。
`np.arange`生成相对时间索引；boolean indexing取有效帧；`np.zeros`建立padding；
`np.savez_compressed`将数组写入NPZ，无返回的新状态。`masked_mse`返回一个float。

每个action tick保留一个anchor，episode结束处不丢样本；history左padding，actions右padding。
S19.3已把原始T+1观测中的terminal o_T排除，本课anchor最后是T−1，最后标签a_(T−1)。
o_T存在并不意味着a_T存在。未来动作只作监督，推理时不能当输入；不同于后续CVAE训练posterior。

## Minimal Experiment / Expected and Actual Result

先在仓库根目录核验环境，再运行：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/sequence_dataset.py --prepared tmp/s19_3/verified193
```

prepared参数是S19.3输出目录。其他机器若没有该ignored目录，依照[S19示例README](../examples/20_learning_from_demonstrations/README.md)
的S19.2→S19.3重建，替换为实际打印路径；不需要BC checkpoint。
每次新建ignored tmp/s20_1/run_*，保存train.npz、validation.npz、results.json（参数、输入/输出hash、反例）。
显式--output要求目录不存在。

2026-10-11，DESKTOP-781D67A，WSL2 Ubuntu24.04.5；mujoco环境Python3.12.14/NumPy2.5.3真实模块路径核验，无安装。
实际命令为上面命令加`--output tmp/s20_1/default`；第二组加`--chunk 20 --output tmp/s20_1/chunk20`，均exit0。

| 条件 | train窗口/actions shape | train有效/padding动作token | validation有效/padding |
| --- | --- | --- | --- |
| L4/H10 | 1600 / (1600,10,2) | 15280 / 720 | 3820 / 180 |
| L4/H20 | 1600 / (1600,20,2) | 28960 / 3040 | 7240 / 760 |

train history shape(1600,4,6)、有效6304；validation (400,4,6)、有效1576；两组相同。
预期样本数不变、H增加导致尾部padding增加；实际吻合。每条T100/H10有效动作数955，H20为1810。
短合成episode长度3/2，L4/H4且输入rows打乱：A最后chunk=[102,0,0,0]；错误flat slice=[102,200,201]跨入B。
有效位置误差固定1时masked MSE=1；padding预测改1000，错误unmasked MSE=550000.45，正确loss仍1；改NaN仍1。
这些是合成数字，不是米单位机器人误差。
另对两组共4000保存窗口逐token独立核对source ID/timestep/value/padding通过；
H=1与H>T、empty mask拒绝、四种CLI非法/缺输入/已有output拒绝exit2通过；Python3.11语法检查通过。

## Explanation / Failure Cases / Robotics Context

mask使“没有标签”与“标签恰好为归一化零”分开。增大H提供更多未来监督，也增加尾部无效槽与重叠标签；不自动提升闭环成功。

- 先平铺再切窗口会跨episode；先随机row split会泄漏同一轨迹，本课冻结episode split。
- 历史从t+1开始会泄漏未来观测；首个动作必须对齐a_t。
- padding先填物理0再归一化会变成其他值；本课直接填normalized 0且明确mask。
- 按B×H×2除会让padding多的batch loss看似更小；必须只统计有效标量。
- 只mask loss而不处理历史padding，不保证后续attention忽略空输入。

机器人序列预测需要这份时间契约：reference依然交给低层controller；预测H个动作与实际执行多少个动作是不同参数，后者留S20.2。
本课不重跑物理、GUI或策略性能评价，无ACT成功率结论。

## Interview Capsule

**30秒：**我按episode分组，以每个动作时刻为anchor，构造只含过去/当前的观测历史与从当前开始的未来动作标签。
边界处补零并保存有效mask；归一化只复用train统计，loss只平均有效动作标量。

**2分钟：**解释(B,L,6)/(B,H,2)和两个mask；o_t对应a_t，terminal观测无额外动作。
历史左补、动作右补，缺失索引−1；0可是真实训练均值，所以不能从值推断valid。
按episode split之后再生成重叠窗口，保持来源隔离；更长H改变标签重复与padding比例，不等于更好的rollout。
推理只获取真实历史，未来expert动作不可见；loss mask也不等于attention API的mask。

## Must Remember / My Verification

Engineering与Learning状态见根README；助手执行不计本人Run。Runtime实际Python3.12，3.11仅语法检查。

**Run：**运行默认命令，找到results.json与两份NPZ，对照shape和padding统计。

**Modify：**先预测H10→20时样本数、历史shape、padding数量怎样变，再加`--chunk 20`运行。
用T100逐anchor累加min(H,T−t)，解释为何有效标签数并未翻倍。

**Explain（五问）：**

1. 在t=0与t=99，L4/H10各有哪些真实观测/动作索引？为什么o_100不能产生a_100？
2. normalized padding=0为什么不能证明token真实？为什么不能用数组值推断mask？
3. 两条episode平铺后切chunk有什么错误？重叠window是否代表更多独立示范？
4. masked MSE的分母为何是2×valid token数？若按完整B×H×2平均会发生什么？
5. 训练的未来动作标签与推理输入有什么差别？预测H步是否意味着必须开环执行H步？

本人Run/Modify/Explain待报告；下一小任务S20.2等待明确请求。
