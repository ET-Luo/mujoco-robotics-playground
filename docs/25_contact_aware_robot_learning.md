# Stage 22 — Contact-Aware Robot Learning（规划 skeleton）

[状态与任务](../README.md#stage-22--contact-aware-robot-learning)。
Why / Intuition：几何相近的状态可能处于不同接触模式；force/tactile提供操作反馈。

| Task（各0.5～2h） | 内容 / 最小实验 |
| --- | --- |
| S22.1 | proprioception+force+touch schema；frame、采样时间、噪声、normalization与通道ablation |
| S22.2 | 复用teleoperation mapper/controller记录示范；scripted baseline，最小键盘采集另拆子任务，不假装已有真人数据 |
| S22.3 | surface-following contact-aware BC闭环；state-only vs force/touch observations |
| S22.4 | ACT/Diffusion在同contact task重训重测；接触丢失/超力与恢复 |
| S22.5 | 小三指free-object保持/扰动task，复用gate/slip/drop scorer与专家；先验证expert可行性 |
| S22.6 | dexterous/contact-aware policy闭环，动作限幅与contact failure锁存 |
| S22.7 | friction/mass各自改变，训练域/held-out域与domain generalization |
| S22.8 | noise/latency分别扫描；观测时间对齐与历史buffer，多因素组合另列 |
| S22.9 | final Learning-Based Manipulation Benchmark：各轨道BC/ACT/Diffusion，统一split/seed/metrics与总结 |

Core Concepts / Mathematics：o=[q,qdot,goal,F_W,touch]；F单位N、touch按各sensor定义；
policy动作仍经已有低层controller执行，不把学习输出直接当无界torque。
Math-to-Code：具名contact提取→固定schema/timestamp→train-only统计→policy→controller→全程scorer。
Minimal Experiment / Expected / Actual：拟比较接触通道帮助；尚未实现，无成功率结论。
Explanation：force/tactile能帮助辨别模式，也可能受噪声/延迟影响；仿真contact truth需标明部署可得性。
Failure Cases：sensor/action错时、privileged状态泄漏、未ready trial被删除、末帧恢复掩盖drop。
Robotics Applications：表面接触、顺应操作、有限窗口灵巧抓持与恢复。
30秒 Interview：接触学习需要正确的力方向、时间对齐与全过程稳定性判据。
2分钟 Interview：说明示范来源、低层/高层频率、通道ablation、同协议比较与OOD物理参数实验。
Must Remember：训练域内成功不等于domain generalization；有限窗口不证明硬件或长期稳定。
My Verification / Run / Modify / Explain：实施时提供具体命令，当前未完成；终点在P3，不进入P4。
Follow-up：哪些通道部署可得？延迟怎样模拟？为何保留未ready分母？如何隔离friction与mass效应？
