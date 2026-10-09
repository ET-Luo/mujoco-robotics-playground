# Stage 18 — Teleoperation & Dexterous Foundations

入口：[S18.1学习包](../../docs/21_1_incremental_teleoperation.md) / [代码](incremental_reference.py)。
Engineering / Learning状态只维护在[根README](../../README.md#stage-18--teleoperation--dexterous-foundations)。

S18.1仅synthetic master translation→world增量→norm限速→box投影→Cartesian reference。
固定motion scale1，无actual robot/MuJoCo/ROS/device/clutch。依赖requirements已有NumPy/Matplotlib，无helper或新依赖。
数值例子也须使用conda mujoco，Agg headless；不在system/base Python执行。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/incremental_reference.py
python examples/19_teleoperation_dexterous/incremental_reference.py --master-amplitude 2
```

Modify改变合成master运动幅度1→2，mapper scale/20mm/s界/workspace/采样率不变。
2026-10-09 Zero：两条命令exit0，requested峰80/160mm/s，applied reference峰均20mm/s；
首次workspace裁剪3.02/2.02s，4.02s反向第一帧立即释放；unbounded baseline预期超速/越界。
常量offset/静态guards/独立CSV解析milestones验证见学习包；两PNG目视、links/git diff --check通过。
产物只ignored tmp/s18_1_a1/a2；不称actual robot速度/接触/安全验证。
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.2已按请求完成[学习包](../../docs/21_2_scaling_clutch.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.4等待明确请求。

## S18.2 — scaling / clutch / recenter

[学习包](../../docs/21_2_scaling_clutch.md) / [代码](scaling_clutch.py)。
复用S18.1 local helper；只需NumPy/Matplotlib，不新增依赖。

```bash
python examples/19_teleoperation_dexterous/scaling_clutch.py
python examples/19_teleoperation_dexterous/scaling_clutch.py --fine-scale .5
```

先按上文核验mujoco环境。2026-10-09 DESKTOP-781D67A两命令exit0：
clutch/recenter保持、scale切换无历史跳变、恢复首帧.04/.10mm、norm峰20mm/s、box裁剪10/25sample。
产物ignored tmp/s18_2_scale0.2与scale0.5；没有实际机器人或GUI验证。
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）。S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.4等待明确请求。

## S18.3 — teleoperation + impedance

[学习包](../../docs/21_3_teleoperation_impedance.md) / [代码](teleoperation_impedance.py)。
复用Stage17 XY fixture/contact与S18.1 mapper；实际状态只由mj_step推进。
依赖requirements已有mujoco/numpy/matplotlib，local helpers同三包；无需Menagerie/ROS或新依赖。
先按上文核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/teleoperation_impedance.py
python examples/19_teleoperation_dexterous/teleoperation_impedance.py --master-period .1
```

每条命令比较free_local/contact_local/contact_packet_feedback；simulation10s/10000physics步，CSV/JSON/PNG只ignored tmp/s18_3_period*。
Modify把master50Hz改10Hz，local反馈保持1kHz。目标ZOH、vd=0；不是实时多线程/IPC运行。
2026-10-09 DESKTOP-781D67A：两命令exit0，各三case、10000×20CSV；本地两频率final tracking通过，无motor饱和。
接触峰3.158/3.713N、hold约1.998N；10Hz packet-feedback末tracking失败，峰445.824N、饱和4900joint samples。
工程PASS包含这个明确失败对照；motor20N cap不等于contact反力界。
独立CSV/物理/调度检查、局部线性谱半径分析、CLI拒绝、3.11语法及两PNG通过；无GUI/硬件验证。
Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.4等待明确请求。
