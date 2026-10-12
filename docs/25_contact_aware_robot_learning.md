# Stage 22 — Contact-Aware Robot Learning

[状态与任务](../README.md#stage-22--contact-aware-robot-learning)。
S22.1已Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）；S22.2 Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）；S22.3 Engineering Complete + Learning Mastered（2026-10-12 本人确认）；S22.4 Engineering Complete（2026-10-12），Learning待本人验证；其余任务仅规划。
[示例入口](../examples/23_contact_aware_learning/README.md)。

Why / Intuition：几何相近的状态可能处于不同接触模式；force/tactile提供操作反馈。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| [S22.1](25_1_observation_contract.md) | 已实现两slide指端接触观测合同；frame、同步时间、force noise、train-only normalization与6/9/7/10维通道ablation |
| [S22.2](25_2_demonstration_recorder.md) | 已复用teleoperation mapper与指端servo记录scripted master；N/N+1、全部attempt保留与actual action replay；真人0，键盘采集仍另拆子任务 |
| [S22.3](25_3_contact_bc.md) | 已实现水平面surface-following反馈专家与BC闭环；state/force/touch/full重训与冻结输入干预，三training seeds、test保留 |
| [S22.4](25_4_contact_policy_comparison.md) | 已同contact task重训九个BC/ACT/Diffusion模型；E1/E5、接触丢失/超力与负载恢复，保留失败锁存与独立test |
| S22.5 | 小三指free-object保持/扰动task，复用gate/slip/drop scorer与专家；先验证expert可行性 |
| S22.6 | dexterous/contact-aware policy闭环，动作限幅与contact failure锁存 |
| S22.7 | friction/mass各自改变，训练域/held-out域与domain generalization |
| S22.8 | noise/latency分别扫描；观测时间对齐与历史buffer，多因素组合另列 |
| S22.9 | final Learning-Based Manipulation Benchmark：各轨道BC/ACT/Diffusion，统一split/seed/metrics与总结 |

Core Concepts / Mathematics：o=[q,qdot,goal,F_W,touch]；F单位N、touch按各sensor定义；
policy动作仍经已有低层controller执行，不把学习输出直接当无界torque。
Math-to-Code：具名contact提取→固定schema/timestamp→train-only统计→policy→controller→全程scorer。
Minimal Experiment / Expected / Actual：S22.1同state不同表面paired法向力1.355968/0N；noise .2检查通过。S22.2三条示范2/3 quality eligible，scale .5为0/3，全部保存回放通过；S22.3 contact BC验证state/force/touch/full成功24/20/24/23（各24trial），test未用；不外推通道普遍优劣。
Explanation：force/tactile能帮助辨别模式，也可能受噪声/延迟影响；仿真contact truth需标明部署可得性。
Failure Cases：sensor/action错时、privileged状态泄漏、未ready trial被删除、末帧恢复掩盖drop。
Robotics Applications：表面接触、顺应操作、有限窗口灵巧抓持与恢复。
30秒 Interview：接触学习需要正确的力方向、时间对齐与全过程稳定性判据。
2分钟 Interview：说明示范来源、低层/高层频率、通道ablation、同协议比较与OOD物理参数实验。
Must Remember：训练域内成功不等于domain generalization；有限窗口不证明硬件或长期稳定。
My Verification / Run / Modify / Explain：S22.1具体命令与五问见[学习包](25_1_observation_contract.md)，Learning已Mastered（2026-10-12 本人确认）；终点在P3，不进入P4。
Follow-up：哪些通道部署可得？延迟怎样模拟？为何保留未ready分母？如何隔离friction与mass效应？

S22.2具体Run/replay、scale .5 Modify与五问Explain见[学习包](25_2_demonstration_recorder.md)；Learning已Mastered（2026-10-12 本人确认），不自动启动S22.3。

S22.3具体Run、冻结mean-input Modify与五问Explain见[学习包](25_3_contact_bc.md)；Learning已Mastered（2026-10-12 本人确认），不自动进入S22.4。

S22.4具体Run、冻结pulse duration Modify与五问Explain见[学习包](25_4_contact_policy_comparison.md)；Learning待本人验证，不自动进入S22.5。
