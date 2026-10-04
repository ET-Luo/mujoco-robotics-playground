# Stage 12 — Robot Perception Geometry

当前实现 S12.1 Camera Frames、S12.2 Pinhole Projection 、S12.3 CPU RGB/depth acquisition 、S12.4 camera calibration 与 S12.5 PnP object pose。
S12.1 完整教学与实验记录见
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

## S12.2 — Pinhole / Intrinsic / 3D→2D Projection

[完整 Learning Package](../../docs/15_2_pinhole_projection.md)。只用 NumPy/Matplotlib，
已知 optical-camera 点与 K 投影到 pixel；区分 depth gate 与图像边界，并展示同射线深度歧义。

```bash
python examples/13_perception_geometry/pinhole_projection.py
python examples/13_perception_geometry/pinhole_projection.py --focal-scale 2
```

生成小 CSV/PNG 到已忽略的 `tmp/s12_2_pinhole_f1/`、`tmp/s12_2_pinhole_f2/`。
Modify 只改变 fx/fy，保持主点与 image size 不动，预测 pixel 相对主点偏移如何改变。
S12.2 无 calibration、渲染、PnP 或 motion。

## S12.3 — MuJoCo CPU RGB / Depth Acquisition

[完整 Learning Package](../../docs/15_3_rgb_depth_acquisition.md)。小 XML scene 用 EGL/Mesa
llvmpipe 进行软件渲染，脚本在 import 前设置 backend 并检查 renderer name，无需 GUI/GPU。

```bash
python examples/13_perception_geometry/rgb_depth_capture.py
python examples/13_perception_geometry/rgb_depth_capture.py --camera-height 1.0
```

原始 RGB uint8、米制 float32 depth 与 PNG/metadata 保存到已忽略的 `tmp/s12_3_rgbd_*/`。
默认 top/floor depth=0.74/0.80 m；Modify 变为 0.94/1.00 m，K 保持不变。
这里只核验 frame、像素方向、可见表面 axial depth，不做 calibration/PnP/point cloud。
S12.3 acquisition 本身不估计内参。

## S12.4 — Camera Calibration

[完整 Learning Package](../../docs/15_4_camera_calibration.md)。CPU OpenCV headless 用
24 个 synthetic checkerboard-corner views 标定 K/d，各 54 个内角点；8 个留出视角作
known-pose 误差检查。无图像检测或 PnP，不宣称 camera→base 外参已经标定。

```bash
python examples/13_perception_geometry/camera_calibration.py
python examples/13_perception_geometry/camera_calibration.py --noise-px 0.6
```

依赖新增 opencv-python-headless，缺失时只在核验后的 mujoco 环境安装。
JSON/CSV/NPZ/PNG 在已忽略 `tmp/s12_4_calibration_noise*/`。
Modify 只改角点噪声；同时观察 training/held-out error，并解释 board size ×2 对平移尺度的影响。
S12.4 不估计待操作物体的 PnP pose。

## S12.5 — PnP Object Pose

[完整 Learning Package](../../docs/15_5_pnp_pose.md)。已知 K/d 和 12 个有身份的非共面 metric
3D↔2D 对应，OpenCV CPU solvePnP 输出 object→optical camera 的 T_CO。复用前课投影/RMS helper，
不重新标定；分别报告 pixel residual、translation error [m] 与 rotation error [degree]。

```bash
conda activate mujoco
python examples/13_perception_geometry/pnp_pose.py
python examples/13_perception_geometry/pnp_pose.py --noise-px 0.8
```

先预测噪声增大后的变化，再比较；同时观察 focal+5% 的低 RMS / 高 pose error。
JSON/CSV/NPZ/PNG 在 ignored `tmp/s12_5_pnp_noise*_seed20261004/`。
输入为合成对应点，不包含 detector、base transform 或抓取。本人已确认实验、验证与 Explain；任务状态见根 README。
Engineering 完成后 STOP；S12.6 及后续仅规划。
