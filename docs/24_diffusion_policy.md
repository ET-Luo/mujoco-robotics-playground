# Stage 21 — Diffusion Policy（S21.1–S21.3已实现，其余规划）

前置：BC→ACT；[状态与任务](../README.md#stage-21--diffusion-policy)。
Why / Intuition：多条合法路径的平均动作可能不合法，用条件生成建模动作序列分布。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| [S21.1](24_1_multimodal_paths.md) | 已实现：成对XY双路径/几何禁入区、条件均值碰撞反例、BC/ACT新任务重训与validation对照 |
| [S21.2](24_2_noising_basics.md) | 已实现：noise schedule、forward noising、oracle epsilon恢复与独立Markov边缘矩验证 |
| [S21.3](24_3_conditional_denoiser.md) | 已实现：H1 state/timestep条件MLP、epsilon预测与匹配none训练对照 |
| S21.4 | action-sequence diffusion：window/mask、反向采样与反归一化 |
| S21.5 | tiny state Diffusion Policy闭环MuJoCo执行；短前缀/receding horizon |
| S21.6 | 多模态路径覆盖/碰撞/模式切换，采样steps与CPU延迟ablation |
| S21.7 | BC/ACT/Diffusion同task/split/metrics比较，多seed与失败分析 |

Core Concepts / Mathematics：A_k=√ᾱ_k A_0+√(1−ᾱ_k)ε；条件噪声预测
εθ(A_k,k,o_history)，优化masked mean ||ε−εθ||²；k是去噪index，不是机器人时间。
Math-to-Code：训练动作归一化→加噪→小denoiser→逆过程→执行前缀→重新观测。
Minimal Experiment / Expected / Actual：S21.1双路径expert均2/2、uniform mean碰撞2/2；BC/ACT E1/E5成功0/2、1/2、2/2（validation）；p=.8解析mean安全2/2。新test保留；S21.3单步条件denoiser audit MSE .039117，none .307293，尚无反向采样/生成policy，不预判优于ACT。
Explanation：生成样本需要序列一致性；低denoising loss不能保证不碰撞或mode coverage。
Failure Cases：错误schedule、动作尺度失配、均值轨迹撞障碍、逐tick随机切模式、采样慢于预算。
Robotics Applications：多解操作与连续动作生成；CPU小模型教学版本，不复现大型视觉系统。
30秒 Interview：Diffusion Policy条件去噪生成动作序列，并在闭环中只执行短前缀。
2分钟 Interview：从平均动作失败讲起，解释forward/noise objective/reverse sampling、条件、horizon与延迟取舍。
Must Remember：机器人时间与diffusion timestep不同；必须测试真实MuJoCo rollout。
My Verification / Run / Modify / Explain：S21.1 Engineering Complete，Learning Run/Modify/Explain全部完成，Mastered（2026-10-12 本人确认实验、预测与五项Explain）；命令/练习/五问见[S21.1学习包](24_1_multimodal_paths.md)，S21.2 Engineering Complete、Learning Run/Modify完成、Explain全部完成、Learning Mastered（2026-10-12 本人确认），命令/练习/五问见[S21.2学习包](24_2_noising_basics.md)；[S21.3](24_3_conditional_denoiser.md) Engineering Complete、Learning待验证；不自动实施S21.4。
Follow-up：均值为何撞障碍？conditioning提供什么？如何统计mode coverage？采样steps影响什么？
算法依据：[Diffusion Policy原论文](https://arxiv.org/abs/2303.04137)。
