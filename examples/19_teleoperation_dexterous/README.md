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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.2已按请求完成[学习包](../../docs/21_2_scaling_clutch.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）。S18.3已按请求完成[学习包](../../docs/21_3_teleoperation_impedance.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4已按请求完成[学习包](../../docs/21_4_force_feedback.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

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
S18.5已按请求完成[学习包](../../docs/21_5_teleoperation_integration.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6已按请求完成[学习包](../../docs/21_6_hand_fixture.md)，Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

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
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7已按请求完成[学习包](../../docs/21_7_fingertip_jacobian.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.7 — fingertip FK / Jacobian

[学习包](../../docs/21_7_fingertip_jacobian.md) / [代码](fingertip_jacobian.py)。
复用hand.xml/mapping和三包依赖；仅离线FK/J，不推进physics或引入IK。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/fingertip_jacobian.py
python examples/19_teleoperation_dexterous/fingertip_jacobian.py --delta .01
```

2026-10-10 DESKTOP-781D67A两命令exit0；3配置×3site，解析FK/world J、全6列中心差分/非零速度通过，time0/state未被probe改变。
FK误差2.776e−17m；δ.001/.01预测最大error .0254/2.54µm；FD ε1e−6误差1.517e−11m/rad，1e−8舍入回升。
错误frame/actuator列对照被检测；独立screw axis/JSON/CSV/rank/CLI/3.11语法/两PNG通过。
产物只ignored tmp/s18_7_delta*；没有GUI/actual tracking/碰撞可行性或硬件验证。
Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8已按请求完成[学习包](../../docs/21_8_contact_sensing.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.8 — contact / touch sensing

[学习包](../../docs/21_8_contact_sensing.md) / [代码](contact_sensing.py)。
在hand.xml副本加入distal small/wide touch zones；no_contact/self_contact/fixed_probe三个case。
固定sphere只作测量对照，无自由物体抓取；原hand模型不改。依赖mujoco/numpy/matplotlib及hand_fixture，无新增包。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/contact_sensing.py
python examples/19_teleoperation_dexterous/contact_sensing.py --tip-offset .03
```

2026-10-10 DESKTOP-781D67A：两命令exit0、各3×4001×40CSV与具名pair JSONL/JSON/PNG，只ignored tmp/s18_8_offset*。
self每指normal峰2.818674N；默认small/wide都检测，移位后small0。fixed_probe仅f0，normal峰1.395928N、small0/wide正常。
两offset q/qvel/contact/world force/wide全trace相同；内部force总和0、scalar normal正，逐pair解释见学习包。
force点Jacobian/efc投影、独立JSONL重建/CSV/区域对照、CLI/guards/3.11语法/两PNG/links通过。
没有GUI、真实sensor/自由物体保持/抓取或安全证明。Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.9已按请求完成[学习包](../../docs/21_9_multi_contact_grasp.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.9 — multi-contact grasp integration

[学习包](../../docs/21_9_multi_contact_grasp.md) / [代码](multi_contact_grasp.py)。
复用hand.xml副本/hand_fixture.mapping；依赖已有mujoco/numpy/matplotlib，无新增包。
自由圆柱、接触gate、对向两指/120°三指；2s起object-only向下负载，gravity0，无weld。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/multi_contact_grasp.py
python examples/19_teleoperation_dexterous/multi_contact_grasp.py --load .3
```

2026-10-10 DESKTOP-781D67A两命令exit0，各4case×4001×53CSV与pair JSONL/JSON/PNG，只ignored tmp/s18_9_load*。
0.1N两/三指最大加载漂移1.657/1.104mm，通过4mm有限位置保持判据；0.3N两指4.971mm失败、三指3.314mm通过。
低摩擦虽通过gate仍滑至palm；missing_finger超时不加载。末帧接触/静止不代替全过程验收。
物体contact F与constraint、Newton平衡、独立pair/CSV/COM力矩/slip/50帧gate/冻结target/load/冲量、CLI/guards/3.11语法和两PNG核验。
没有长期/任意wrench/force closure/GUI或真实手验证；两三指布局和total normal同时变化。
Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）。Run/Modify/Explain完成；S18.10a已按请求完成[学习包](../../docs/21_10a_friction_cone.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.10a — friction cone intuition

[学习包](../../docs/21_10a_friction_cone.md) / [代码](friction_cone.py)。
独立三slide球形滑台/plane单接触，防止滚动，gravity0；elliptic condim3/μ与法向负载扫描。
依赖已有mujoco/numpy/matplotlib，无local helper或新安装；不改既有hand/grasp。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/friction_cone.py
python examples/19_teleoperation_dexterous/friction_cone.py --normal-scale 2
```

2026-10-10 DESKTOP-781D67A两命令exit0，各四组33列CSV/JSON/两PNG，只ignored tmp/s18_10a_normal*。
默认理想能力.1/.2/.3/.6N，首次报告速度>0.1mm/s对应外载.1035/.2035/.304/.604N。
scale2最强组能力1.2N，扫到1N/3s未触发，但末速度.024969mm/s非零；null不等于零滑动。
实测fn/切向范数满足cone，contact F/constraint/Newton与独立CSV/动量/停止协议/重复case/CLI/guards/3.11语法和四PNG验证。
步数依停止时刻变化，1ms不变；没有GUI、真实材料、自由滚动或多接触closure证明。
Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）。Run/Modify/Explain全部完成；S18.10b已按请求完成[学习包](../../docs/21_10b_force_closure.md)，Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.10b — planar force closure intuition

[学习包](../../docs/21_10b_force_closure.md) / [代码](force_closure.py)。
纯静态平面两点接触，G/C/W显式可见；依赖已有numpy/matplotlib，无local helper、新安装或MuJoCo积分。
对向摩擦/无摩擦/同侧比较rank与正span；有限负载另加每接触normal cap1N。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/force_closure.py
python examples/19_teleoperation_dexterous/force_closure.py --mu .1
```

2026-10-10 DESKTOP-781D67A两命令exit0，各24×7CSV、矩阵/closure证书/逐负载力分配与naive对照JSON、两PNG，仅ignored tmp/s18_10b_mu*。
三布局closure True/False/False，两μ设置均不变；W rank3/1/3，同侧rank3却不能产生负Fx。
μ.5有界测试7/8、2/8、2/8；μ.1为2/8、2/8、1/8，区别closure方向覆盖与有限力能力。
独立解析1248负载、参考点平移一致变换、force/moment/friction/cap、CLI/guards/3.11语法、四PNG/links检查通过。
仅本理想平面模型证书，无实际物体动态/真实三维/GUI/硬件稳定证明。
Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）。Run/Modify/Explain全部完成；S18.11已按请求完成[学习包](../../docs/21_11_grasp_stability.md)，Engineering Complete；Learning待本人Run/Modify/Explain；下一步为本课学习验证，不自动新增阶段。

## S18.11 — disturbance / grasp stability integration

[学习包](../../docs/21_11_grasp_stability.md) / [代码](grasp_stability.py)。
复用hand_fixture与multi_contact_grasp，依赖已有mujoco/numpy/matplotlib，无安装，旧课模型/代码不改。
三条件×八pulse矩阵，world COM半正弦F/T；ready前置检查，全trial固定4000步，失败锁存。
先核验conda mujoco/interpreter：

```bash
python examples/19_teleoperation_dexterous/grasp_stability.py
python examples/19_teleoperation_dexterous/grasp_stability.py --pulse-scale 4
```

2026-10-10 DESKTOP-781D67A两命令exit0，各24×4001×73CSV/接触快照JSON/results+summary/两PNG，只ignored tmp/s18_11_scale*。
全计划通过8/24→4/24；ready都8/24，正常摩擦8/8→4/8；低摩擦/缺指未ready，取消pulse但不删分母。
强Fx±/Tx±使slip/contact/pose等失败，48trial均无motor saturation；输出峰.046850/.069297Nm。
每帧velocity API/Jacobian、COM F/T投影/Newton/servo，与独立48CSV/脉冲/gate/失败/分母/快照/线动量/CLI/guards/3.11语法/四PNG检查通过。
仅指定矩阵与有限窗口结果，无GUI/真实硬件/任意三维wrench/长期稳定性证明。
Stage18工程任务全部完成；S18.11 Learning待本人Run/Modify/Explain。Run默认、Modify pulse-scale4，Explain见学习包五问；不自动新增阶段。
