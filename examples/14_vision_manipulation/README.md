# Stage 13 — Vision-Based Manipulation

当前实现 S13.1 frame/quality 接口与 S13.2 grasp candidates / endpoint IK screening。
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
S13.2 已按本人明确请求实现，见下。

## S13.2 — Grasp Pose Generation

[完整 Learning Package](../../docs/16_2_grasp_pose_generation.md)。Upright yaw-only box，
估计 T_WO + full dimensions→四个 top-down T_WG/T_WP；先 gap/width 筛选，
再复用 P0 bounded DLS 求 pre/grasp 两端点。每个 candidate 独立 MjData/home。

```bash
python examples/14_vision_manipulation/grasp_candidates.py
python examples/14_vision_manipulation/grasp_candidates.py --size-x-m 0.12
```

先预测 dx 40→120 mm 对 width/opening/IK 调用数的影响，再比较。
PNG/CSV/NPZ/JSON 在 ignored `tmp/s13_2_grasp_x*_y*_yaw*_d*/`，不需前课产物。
无新依赖；模型复用已声明 MuJoCo/Menagerie，consumer 不读取 fixture object truth。
Engineering Complete；本人已确认 Run/Modify，Explain 待补 width/opening/slide 定量关系，状态见根 README。
不检查中间路径/碰撞、未执行闭爪/动力学，IK failed 只是 local failure；不自动实现 S13.3a。
