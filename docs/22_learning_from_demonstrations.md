# Stage 19 — Learning from Demonstrations

Why：从可解释expert学习状态→动作映射，先理解机器人数据，再训练。
Intuition：示范是闭环控制沿时间产生的episode，不是独立图片标签表。
状态只见[README](../README.md#stage-19--learning-from-demonstrations)。

| Task（每项0.5～2h） | 核心概念 / 最小实验 / 验收 |
| --- | --- |
| S19.1 | [observation/action/episode契约](22_1_episode_contract.md)；expert transition生成与实际MuJoCo replay |
| S19.2 | expert demonstration generation：复用controller、采样start/goal、轨迹dataset与manifest，失败episode保留 |
| S19.3 | episode-level train/val/test split；训练集normalization；泄漏与constant-channel检查 |
| S19.4 | CPU tiny MLP Behavior Cloning；MSE数学到Torch训练，held-out loss与constant baseline |
| S19.5 | closed-loop MuJoCo rollout；同reset下expert vs BC，offline loss vs任务成功 |
| S19.6 | covariate shift/compounding error；扰动注入，失败时刻与长horizon对照 |
| S19.7 | recovery demonstration；DAgger intuition与一次policy-state expert relabel，标明不等于完整DAgger |
| S19.8 | BC integration benchmark：冻结split/normalizer/test seeds，全部attempt metrics与耗时 |

Core Concepts：o_t、a_t、o_(t+1)、episode、采样频率、expert vs learned policy。
Mathematics：BC最小化 train 的 mean ||πθ(norm(o_t))−norm(a_t)||²；只拟合train统计。
Math-to-Code：episode IDs→split→training tensors→optimizer→反归一化action→同MuJoCo执行器。
Minimal Experiment：从S19.1开始，后续训练未实施。
Expected/Actual：预计expert闭环恢复；S19.1实际结果见学习包，不存在BC loss或BC success结果。
Explanation：π的动作改变后续输入，监督验证与执行分布不同。
Failure Cases：时序错位、future-state泄漏、按row split、normalizer泄漏、动作单位错误、只报告成功episode。
Robotics Applications：classical专家蒸馏、遥操作模仿、恢复行为。
30秒 Interview：BC拟合示范动作，但机器人必须闭环验证；低离线误差不保证任务成功。
2分钟 Interview：说明episode与action执行契约、按episode划分、train-only统计、expert/BC同seed比较，
再解释distribution shift和恢复数据如何补充policy实际访问状态。
Must Remember：先数据合同，再网络；任务成功由scorer定义。
My Verification：具体Run/Modify/Explain见当前学习包；未来各Task分别交接，不预标掌握。
Follow-up：为什么不能随机row split？为何MSE低仍失败？恢复数据从哪些状态采？
