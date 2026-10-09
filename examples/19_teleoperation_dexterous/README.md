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
Engineering Complete，Learning Run/Modify/Explain待本人报告；下一小任务S18.2等待明确请求。
