# Stage 12 — Robot Perception Geometry

当前仅 S12.1 Camera Frames / Coordinate Transform。完整教学与实验记录见
[Learning Package](../../docs/15_robot_perception_geometry.md)；状态唯一来源为
[根 README](../../README.md#stage-12--robot-perception-geometry)。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/camera_frames.py
python examples/13_perception_geometry/camera_frames.py --camera-x -0.25
```

脚本用 NumPy 合成 optical-camera observation，读取官方 UR5e `base` 的实际世界位姿，
验证 camera→base→world 的完整 pose、点、方向和刚体逆变换。无需 GPU、图像、GUI 或新依赖。
移动相机时物体保持不动：camera 坐标变化，恢复后的 base/world pose 不变。
S12.2 及后续仅规划，尚无实现。
