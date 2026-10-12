# Stage 21 — Diffusion Policy（S21.1–S21.7已实现，S21.7学习待验证）

前置：BC→ACT；[状态与任务](../README.md#stage-21--diffusion-policy)。
Why / Intuition：多条合法路径的平均动作可能不合法，用条件生成建模动作序列分布。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| [S21.1](24_1_multimodal_paths.md) | 已实现：成对XY双路径/几何禁入区、条件均值碰撞反例、BC/ACT新任务重训与validation对照 |
| [S21.2](24_2_noising_basics.md) | 已实现：noise schedule、forward noising、oracle epsilon恢复与独立Markov边缘矩验证 |
| [S21.3](24_3_conditional_denoiser.md) | 已实现：H1 state/timestep条件MLP、epsilon预测与匹配none训练对照 |
| [S21.4](24_4_sequence_diffusion.md) | 已实现：H10/window/mask、DDPM100步反向采样与米单位反归一化 |
| [S21.5](24_5_diffusion_rollout.md) | 已实现：冻结H10/K100，actual反馈与短前缀，E5/E1均0/2、有发散 |
| [S21.6](24_6_sampling_diagnostics.md) | 已实现：DDPM100/DDIM100/25/10、局部倾向/实际穿越/成功覆盖与CPU延迟；均0/8 |
| [S21.7](24_7_policy_benchmark.md) | 已实现：三training seeds、共同B轨道val/test/指标、CPU成本与冻结sampling Modify |

Core Concepts / Mathematics：A_k=√ᾱ_k A_0+√(1−ᾱ_k)ε；条件噪声预测
εθ(A_k,k,o_history)，优化masked mean ||ε−εθ||²；k是去噪index，不是机器人时间。
Math-to-Code：训练动作归一化→加噪→小denoiser→逆过程→执行前缀→重新观测。
Minimal Experiment / Expected / Actual：S21.1双路径expert均2/2、uniform mean碰撞2/2；BC/ACT E1/E5成功0/2、1/2、2/2（validation）；p=.8解析mean安全2/2。S21.1–6保留的新B test已于S21.7冻结后首次评价，今后为公开复测；S21.3单步条件denoiser audit MSE .039117，none .307293，S21.4已实现H10反向采样，audit .089860，sample存在抖动；S21.5已实现闭环E5/E1均0/2且有发散；S21.6冻结采样对照均0/8、成功上下覆盖0/2，少步降低本轮median；S21.7 val/test ACT E5均6/6、BC0/6、Diffusion均0/24；结论限本教学实现与小任务，不作普遍优劣判断。
Explanation：生成样本需要序列一致性；低denoising loss不能保证不碰撞或mode coverage。
Failure Cases：错误schedule、动作尺度失配、均值轨迹撞障碍、逐tick随机切模式、采样慢于预算。
Robotics Applications：多解操作与连续动作生成；CPU小模型教学版本，不复现大型视觉系统。
30秒 Interview：Diffusion Policy条件去噪生成动作序列，并在闭环中只执行短前缀。
2分钟 Interview：从平均动作失败讲起，解释forward/noise objective/reverse sampling、条件、horizon与延迟取舍。
Must Remember：机器人时间与diffusion timestep不同；必须测试真实MuJoCo rollout。
My Verification / Run / Modify / Explain：S21.1 Engineering Complete，Learning Run/Modify/Explain全部完成，Mastered（2026-10-12 本人确认实验、预测与五项Explain）；命令/练习/五问见[S21.1学习包](24_1_multimodal_paths.md)，S21.2 Engineering Complete、Learning Run/Modify完成、Explain全部完成、Learning Mastered（2026-10-12 本人确认），命令/练习/五问见[S21.2学习包](24_2_noising_basics.md)；[S21.3](24_3_conditional_denoiser.md) Engineering Complete + Learning Mastered（2026-10-12 本人确认Run/Modify与五项Explain）；[S21.4](24_4_sequence_diffusion.md) Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）；[S21.5](24_5_diffusion_rollout.md) Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）；[S21.6](24_6_sampling_diagnostics.md) Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）；[S21.7](24_7_policy_benchmark.md) Engineering Complete、Learning待验证；不自动实施Stage22。
Follow-up：均值为何撞障碍？conditioning提供什么？如何统计mode coverage？采样steps影响什么？
算法依据：[Diffusion Policy原论文](https://arxiv.org/abs/2303.04137)。
