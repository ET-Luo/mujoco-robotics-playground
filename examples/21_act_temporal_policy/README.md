# Stage 20 — ACT / Temporal Policy

当前实现[S20.1 sequence dataset / boundaries / mask](../../docs/23_1_sequence_dataset.md)、[S20.2 chunk baseline](../../docs/23_2_chunk_baseline.md)、[S20.3 attention basics](../../docs/23_3_attention_basics.md)、[S20.4 CVAE latent](../../docs/23_4_cvae_latent.md)与[S20.5 tiny state ACT](../../docs/23_5_tiny_act.md)。
状态唯一来源：[根README](../../README.md#stage-20--act--temporal-policy)。
S20.1仅依赖requirements已有NumPy；无Torch/MuJoCo/local helper导入，无新依赖。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/sequence_dataset.py --prepared tmp/s19_3/verified193
python examples/21_act_temporal_policy/sequence_dataset.py --prepared tmp/s19_3/verified193 --chunk 20
```

prepared为S19.3目录；缺失时按[Stage19](../20_learning_from_demonstrations/README.md) S19.2→3重建并替换路径。
默认L4/H10，每个动作时刻一个anchor；history左padding/action右padding，normalized zero，valid=True表示真实token。
仅train/validation；冻结split与normalizer，不读取test数组、不训练、不推进物理。
输出新ignored tmp/s20_1/run_*：train/validation NPZ与results.json；--output必须新目录。
2026-10-11本机两组exit0：train1600/validation400窗口；H10 train有效15280/padding720，H20有效28960/padding3040。
短episode跨界与masked loss反例通过；本人已确认Run/Modify/Explain，S20.1 Learning Mastered（2026-10-11），详见学习包。S20.2见下节，S20.3见后节，S20.4见后节，S20.5见后节，S20.6见后节，S20.7及后续尚未实现。

## S20.2 — MLP chunk baseline / execute prefix / replan

[学习包与五问](../../docs/23_2_chunk_baseline.md)。依赖已有CPU Torch/NumPy/MuJoCo/Matplotlib，
复用sequence_dataset与Stage19 behavior_cloning/episode_contract/cartesian_spring，无安装。
核验上述mujoco环境后运行：

```bash
python examples/21_act_temporal_policy/chunk_baseline.py --prepared tmp/s19_3/verified193
python examples/21_act_temporal_policy/chunk_baseline.py --prepared tmp/s19_3/verified193 --chunk 20
```

当前状态6→32→32→2H，L1、H10默认，200epochs/seed202，masked loss，validation选checkpoint，不打开test。
同weights比较E1/5/H；仅执行前缀后用actual状态replan，50Hz reference/1kHz feedback不变。
新ignored tmp/s20_2/run_*含protocol/checkpoint/loss/offline预测/20trial/results/experiment.png。
2026-10-11本机H10 success1/4、1/4、2/4；H20为1/4、2/4、4/4；expert两组4/4、constant0/4。
40trial独立80000physics步、chunk对齐/checkpoint/mask梯度/末段截断及CLI通过，详见学习包。
有限4reset/单seed/无扰动或新test；本例是MLP baseline，ACT尚未实现。本人已确认实验、预测与五项Explain，Learning Run/Modify/Explain全部完成，Mastered（2026-10-11），S20.3见下节。

## S20.3 — Token / positions / QKV / attention / masks

[学习包、手算与五问](../../docs/23_3_attention_basics.md)。仅已有NumPy/CPU Torch；无local helper/MuJoCo/Matplotlib导入、新安装或历史tmp输入。
核验上述mujoco环境后运行：

```bash
python examples/21_act_temporal_policy/attention_basics.py
python examples/21_act_temporal_policy/attention_basics.py --slots 7
```

3token/2channel手算，D4/h2 MHA与显式投影/每头NumPy计算核对；padding/causal/位置编码probe，
residual/LayerNorm/FFN与future-slot cross-attention小前向。random weights，不训练策略/ACT或推进物理。
输出新ignored tmp/s20_3/run_*：attention.npz、results.json，含矩阵/shapes/hash。
2026-10-11两组exit0；MHA手写max误差2.776e−17，padding/causal有效输出变化0，slots5→7前5输出一致。
手算保存值/hash/weights归一化、query清零、CLI与invalid mask检查通过，3.11仅语法核验。
Engineering Complete；本人已确认实验、预测与五项Explain，Learning Run/Modify/Explain全部完成，Mastered（2026-10-11）；S20.4见下节。

## S20.4 — CVAE posterior / reparameterization / KL / inference prior

[学习包、手算与五问](../../docs/23_4_cvae_latent.md)。仅已有NumPy/CPU Torch与sequence_dataset helper；无MuJoCo/Matplotlib、新安装。
核验上述mujoco环境后运行：

```bash
python examples/21_act_temporal_policy/cvae_latent.py --prepared tmp/s19_3/verified193
python examples/21_act_temporal_policy/cvae_latent.py --prepared tmp/s19_3/verified193 --beta 1
```

S19.3 prepared缺失时按Stage19 S19.2→3重建。MLP CVAE/H10/latent4、100epochs/seed204，validation固定epsilon的R+beta KL选checkpoint。
posterior输入o+masked expert chunk+valid；infer仅接受o/可选z，默认z=0；不打开test、不做rollout，尚非完整ACT。
新ignored tmp/s20_4/run_*保存protocol/checkpoint/loss/validation predictions/results。
2026-10-11两组exit0；beta .01/1的KL .001552/.0000641，prior-zero RMSE .804/.819mm。
手算/Normal KL/梯度、padding隔离、infer禁用encoder检查、checkpoint/hash/CSV与CLI通过，详见学习包。
Engineering Complete；本人已确认实验、预测与五项Explain，Learning Run/Modify/Explain全部完成，Mastered（2026-10-11）；S20.5见下节。

## S20.5 — Tiny state ACT / Transformer CVAE / closed-loop

[学习包与五问](../../docs/23_5_tiny_act.md)。依赖已有CPU Torch/NumPy/MuJoCo/Matplotlib，
helper为attention_basics/cvae_latent/sequence_dataset/chunk_baseline及旧S19 controller，无安装。
核验上述mujoco环境后运行：

```bash
python examples/21_act_temporal_policy/tiny_act.py --prepared tmp/s19_3/verified193
python examples/21_act_temporal_policy/tiny_act.py --prepared tmp/s19_3/verified193 \
  --checkpoint tmp/s20_5/default/act.pt --execute 1
```

第二命令将checkpoint替换为第一命令实际打印目录下act.pt；prepared缺失按S19.2→3重建。
D32/2head/FF64/latent4，posterior/memory encoder与decoder各1层，H10、默认E5、31146参数。
100epochs/seed205，train posterior R+.01KL，validation **prior-zero** MSE选checkpoint；test不打开。
推理仅actual o/z0，reference50Hz/inner反馈1kHz；--checkpoint模式不训练，固定weights改E。
新ignored tmp/s20_5/run_*含protocol/act.pt/offline/12trial/results/rollout.png，训练另含loss.csv。
2026-10-11 best20/priorRMSE .829mm，E5 4/4、E1 2/4，expert均4/4、constant均0/4；trainloop25.443s。
24trial独立48000physics步/动作时序/hash/checkpoint/梯度与infer隔离通过；有限单seed/4validation reset，不是完整论文复现或统一算法benchmark。
Engineering Complete；本人已确认实验、预测与五项Explain，Learning Run/Modify/Explain全部完成，Mastered（2026-10-11）；temporal ensembling见S20.6。

## S20.6 — Temporal ensembling / ablation

[学习包、公式、结果与五问](../../docs/23_6_temporal_ensemble.md)。沿用S20.5依赖与checkpoint，无安装或训练。

```bash
python examples/21_act_temporal_policy/temporal_ensemble.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s20_5/default/act.pt
```

先核验mujoco环境；checkpoint替换为自己的S20.5输出。按同一执行tick对齐历史chunk，显式时间mask保留零reference。
固定weights对比E1/E5/uniform/newer/older；正age decay偏新，负偏旧；不打开test。
2026-10-12默认±.5成功2/4、4/4、2/4、3/4、1/4；追加--decay .01两方向均2/4，接近uniform。
输出新ignored tmp/s20_6/run_*，20trial NPZ/results/ablation.png。默认40000步独立Euler/actual-state预测/scorer/E1-E5精确复核通过。
Engineering Complete；Learning Run/Modify/Explain待本人验证；不自动推进S20.7。
