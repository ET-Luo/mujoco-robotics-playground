# Stage 21 — Diffusion Policy（规划 skeleton）

前置：BC→ACT；[状态与任务](../README.md#stage-21--diffusion-policy)。
Why / Intuition：多条合法路径的平均动作可能不合法，用条件生成建模动作序列分布。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| S21.1 | 同XY双路径绕障碍示范；多模态action与regression averaging失败，BC/ACT重训对照 |
| S21.2 | diffusion basics：noise schedule、forward noising、epsilon prediction的小数组实验 |
| S21.3 | conditional diffusion：state/timestep conditioning、小MLP denoiser训练 |
| S21.4 | action-sequence diffusion：window/mask、反向采样与反归一化 |
| S21.5 | tiny state Diffusion Policy闭环MuJoCo执行；短前缀/receding horizon |
| S21.6 | 多模态路径覆盖/碰撞/模式切换，采样steps与CPU延迟ablation |
| S21.7 | BC/ACT/Diffusion同task/split/metrics比较，多seed与失败分析 |

Core Concepts / Mathematics：A_k=√ᾱ_k A_0+√(1−ᾱ_k)ε；条件噪声预测
εθ(A_k,k,o_history)，优化masked mean ||ε−εθ||²；k是去噪index，不是机器人时间。
Math-to-Code：训练动作归一化→加噪→小denoiser→逆过程→执行前缀→重新观测。
Minimal Experiment / Expected / Actual：预计学习双路径分布；尚未实施，不能预判优于ACT。
Explanation：生成样本需要序列一致性；低denoising loss不能保证不碰撞或mode coverage。
Failure Cases：错误schedule、动作尺度失配、均值轨迹撞障碍、逐tick随机切模式、采样慢于预算。
Robotics Applications：多解操作与连续动作生成；CPU小模型教学版本，不复现大型视觉系统。
30秒 Interview：Diffusion Policy条件去噪生成动作序列，并在闭环中只执行短前缀。
2分钟 Interview：从平均动作失败讲起，解释forward/noise objective/reverse sampling、条件、horizon与延迟取舍。
Must Remember：机器人时间与diffusion timestep不同；必须测试真实MuJoCo rollout。
My Verification / Run / Modify / Explain：实施时给命令，当前均未完成。
Follow-up：均值为何撞障碍？conditioning提供什么？如何统计mode coverage？采样steps影响什么？
算法依据：[Diffusion Policy原论文](https://arxiv.org/abs/2303.04137)。
