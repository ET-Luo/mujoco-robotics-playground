# Stage 20 — ACT / Temporal Policy（S20.1–5已实现，其余规划）

前置：S19 BC integration完成后才开始；[状态与任务](../README.md#stage-20--act--temporal-policy)。
Why / Intuition：预测短动作序列可表达时间结构，执行时仍重新观测。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| [S20.1](23_1_sequence_dataset.md) | 已实现：历史/未来window、episode边界、padding mask |
| [S20.2](23_2_chunk_baseline.md) | 已实现：MLP chunk baseline，执行前缀/replan与horizon比较 |
| [S20.3](23_3_attention_basics.md) | 已实现：token/位置编码/QKV/attention/mask，低维shape手算与小前向 |
| [S20.4](23_4_cvae_latent.md) | 已实现：MLP CVAE机制、posterior/latent/prior与KL，训练/推理输入隔离（非完整ACT） |
| [S20.5](23_5_tiny_act.md) | 已实现：state Transformer CVAE/chunk decoder训练与prior-zero closed-loop，CPU小网络 |
| S20.6 | temporal ensembling：同一执行时刻的重叠预测对齐与指数权重，ablation |
| S20.7 | BC/chunk baseline/ACT统一benchmark，离线与闭环、耗时与失败报告 |

Core Concepts / Mathematics：动作A_t∈R^(H×d_a)；训练qφ(z|o,A)、推理prior z；
loss为masked reconstruction+βKL；ensemble只组合指向同一执行tick的预测。
Math-to-Code：window/mask→posterior→latent→decoder→action反归一化→inner controller。
Minimal Experiment / Expected / Actual：S20.1 L4/H10与H20窗口构造已验证，详见学习包；S20.2 MLP chunk/replan已验证，H10/H20结果见学习包；S20.3 attention小前向已验证，S20.4 MLP CVAE机制已验证；S20.5 tiny state Transformer CVAE训练/闭环已验证，E5 4/4、E1 2/4；非论文完整性能复现。
Explanation：chunk减少逐帧决策频率，但长open-loop前缀可能降低恢复能力；ensemble可平滑也可能增加响应滞后。
Failure Cases：跨episode window、padding计loss、推理使用未来expert action、chunk错时对齐。
Robotics Applications：平滑连续操作与示范时序学习。
30秒 Interview：ACT学习动作chunk；后续ACT将保留生成latent与Transformer，用低维状态替代视觉；当前S20.2是MLP baseline。
2分钟 Interview：解释CVAE train/inference差别、chunk/replan、ensemble、同BC协议闭环评估。
Must Remember：deterministic Transformer baseline不是完整ACT。
My Verification / Run / Modify / Explain：S20.1命令与五问见学习包；本人已确认实验、预测与五问Explain，S20.1 Learning Mastered（2026-10-11）；S20.2 Engineering Complete + Learning Mastered（2026-10-11 本人确认实验、预测与五项Explain）；S20.3 Engineering Complete + Learning Mastered（2026-10-11 本人确认实验、预测与五项Explain）；S20.4 Engineering Complete + Learning Mastered（2026-10-11 本人确认实验、预测与五项Explain）；S20.5工程完成、Learning待本人验证，其余任务仍规划。
Follow-up：mask排除什么？prior如何用？何时长chunk更差？ensemble怎样对齐？
算法依据：[ACT原论文](https://arxiv.org/abs/2304.13705)。
