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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.2已按请求完成[学习包](../../docs/21_2_scaling_clutch.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）。S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。

## S18.4 — virtual force feedback

[学习包](../../docs/21_4_force_feedback.md) / [代码](force_feedback.py)。
复用S18.3接触run的world environment-on-robot测力，离线生成master virtual device-on-hand信号。
依赖mujoco/numpy/matplotlib与S18.3既有helper链；无新安装。先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/force_feedback.py
python examples/19_teleoperation_dexterous/force_feedback.py --feedback-gain 2
```

Modify只改gain .5→2，norm cap1.5N固定；反号/80ms延迟显示对照。没有设备施力或修改机器人ctrl。
2026-10-09 DESKTOP-781D67A：两命令exit0，robot10000×20/feedback10000×23CSV、JSON/两PNG只ignored tmp/s18_4_gain*。
稳态反力1.998N→显示.999/1.5N；raw峰1.577/6.308N、饱和1/2438sample；robot两运行全trace相同。
正确/反号接近假想功率负/正，离开contact后旧显示残留80sample；独立CSV/轴/norm/physics/guards/CLI/语法通过，四PNG已目视。
只验证显示信号，非真实haptic force或双边passivity/stability；Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）。
S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。

## S18.5 — teleoperation integration

[学习包](../../docs/21_5_teleoperation_integration.md) / [代码](teleoperation_integration.py)。
S18.1–4增量/clutch/recenter/阻抗/virtual反馈合成XY接触任务；新增simulation lease与恢复rebase。
依赖mujoco/numpy/matplotlib与既有helpers，无新安装。先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/teleoperation_integration.py
python examples/19_teleoperation_dexterous/teleoperation_integration.py --dropout-duration 1.2
```

两命令各nominal/dropout_rebase/dropout_wrong_recovery，13s/13000×32CSV、JSON/PNG只ignored tmp/s18_5_gap*。
2026-10-09 DESKTOP-781D67A：六case physics/feedback/contact验证通过；contact约2N、末尾分离与tracking恢复。
过期6.481s，恢复7/7.6s；正确首帧delta0，错误对照delta .4mm（限速与末tracking仍通过，但恢复动作契约失败）。
nominal末X25mm；正确两gap末X23.8/17.8mm；expired/duplicate两包拒绝不刷新lease。
S18.2 map_sample只扩展可选bounds，旧默认/Modify回归通过；独立CSV/CLI/3.11语法/两PNG/links通过。
没有GUI/keyboard/ROS/网络/设备或硬件安全验证；simulation hold不是物理急停。
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。

## S18.6 — lightweight three-finger hand

[学习包](../../docs/21_6_hand_fixture.md) / [MJCF](hand.xml) / [代码](hand_fixture.py)。
固定palm、三指各两hinge、六个gear1 position servo；actuator故意乱序，具名mapping检查。
依赖requirements已有mujoco/numpy/matplotlib，无local helper、新安装或object。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/hand_fixture.py
python examples/19_teleoperation_dexterous/hand_fixture.py --close-scale 2
```

2026-10-09 DESKTOP-781D67A：两命令exit0，各single_0/1/2/cycle、4001×48CSV；JSON/两PNG只ignored tmp/s18_6_scale*。
默认cycle无contact、close error4.9e−7rad；scale2 target [.9,1.2]、actual [.765464,1.200241]rad，三对指间contact。
scale2裁剪7548joint sample、active contact1660sample，软限位最大越界.001939rad；两组均重新张开。
static wrong ctrl0→f2_dist、实际torque cap、固定palm、逐指隔离/servo/动力学/独立CSV/CLI/语法检查通过；四PNG目视。
mount exclusion仅排除palm/prox安装接触，指间collision保持。没有object抓取/GUI/真实手或硬件安全验证。
Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.7等待明确请求。
