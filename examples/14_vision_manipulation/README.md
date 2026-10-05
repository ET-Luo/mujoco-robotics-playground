# Stage 13 — Vision-Based Manipulation

当前仅实现 S13.1：Perception pose→base→world 的 frame/quality 接口。
[完整学习包](../../docs/16_vision_based_manipulation.md) ·
[状态唯一来源](../../README.md#stage-13--vision-based-manipulation)。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/perception_pose.py
python examples/14_vision_manipulation/perception_pose.py --rms-limit-px 0.3
```

固定 camera、合成感知结果包，验证字段/SE(3)/quality/time，再用 actual UR5e base cache
映射 T_CO→T_BO→T_WO。拒绝时不返回可用位姿；truth 只在 producer/scoring 区域。
需 NumPy、MuJoCo、mujoco-menagerie、Matplotlib（均已在 requirements 声明），不需要 OpenCV。
PNG/CSV/NPZ/JSON 在 ignored `tmp/s13_1_pose_rms*/`。

Modify 先预测 RMS 门限 1→0.3 px 对接受数和输出的影响，再运行第二条命令。
包含 low-RMS biased pose 被接受的限制演示，quality 通过不等于 accurate/reachable/graspable。
Engineering Complete；本人于 2026-10-05 确认实验、预测与五项 Explain，Learning Mastered；状态见根 README。无 image detector/PnP、IK、动力学、GUI 或抓取。
不自动实现 S13.2。
