# Stage 20 — ACT / Temporal Policy（规划 skeleton）

前置：S19 BC integration完成后才开始；[状态与任务](../README.md#stage-20--act--temporal-policy)。
Why / Intuition：预测短动作序列可表达时间结构，执行时仍重新观测。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| S20.1 | sequence dataset：历史/未来window、episode边界、padding mask |
| S20.2 | action chunking：先MLP chunk baseline，执行前缀/replan与horizon比较 |
| S20.3 | 必要Transformer：token/位置编码/QKV/attention/mask，低维shape手算与小前向 |
| S20.4 | tiny state ACT的CVAE posterior/latent/prior与KL，训练/推理输入隔离 |
| S20.5 | Transformer chunk decoder训练与closed-loop rollout，CPU小网络 |
| S20.6 | temporal ensembling：同一执行时刻的重叠预测对齐与指数权重，ablation |
| S20.7 | BC/chunk baseline/ACT统一benchmark，离线与闭环、耗时与失败报告 |

Core Concepts / Mathematics：动作A_t∈R^(H×d_a)；训练qφ(z|o,A)、推理prior z；
loss为masked reconstruction+βKL；ensemble只组合指向同一执行tick的预测。
Math-to-Code：window/mask→posterior→latent→decoder→action反归一化→inner controller。
Minimal Experiment / Expected / Actual：预计比较不同chunk/replan；尚未实现，无工程结果。
Explanation：chunk减少逐帧决策频率，但长open-loop前缀可能降低恢复能力；ensemble可平滑也可能增加响应滞后。
Failure Cases：跨episode window、padding计loss、推理使用未来expert action、chunk错时对齐。
Robotics Applications：平滑连续操作与示范时序学习。
30秒 Interview：ACT学习动作chunk；本课保留生成latent与Transformer，用低维状态替代视觉。
2分钟 Interview：解释CVAE train/inference差别、chunk/replan、ensemble、同BC协议闭环评估。
Must Remember：deterministic Transformer baseline不是完整ACT。
My Verification / Run / Modify / Explain：实施时提供命令；未运行/修改/解释，状态留空。
Follow-up：mask排除什么？prior如何用？何时长chunk更差？ensemble怎样对齐？
算法依据：[ACT原论文](https://arxiv.org/abs/2304.13705)。
