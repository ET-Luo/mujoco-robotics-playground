# Stage 13 — Vision-Based Manipulation

当前实现 S13.1 frame/quality、S13.2 grasp candidates，以及 S13.3a image/PnP→连续运动与 S13.3b 完整视觉取放；S13.4分别比较pose/extrinsic误差，S13.5进行seeded重复试验评价。
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
Engineering Complete；本人已确认 Run/Modify/Explain，Learning Mastered，状态见根 README。
不检查中间路径/碰撞、未执行闭爪/动力学，IK failed 只是 local failure；不自动实现 S13.3a。

## S13.3a — Vision-to-Motion

[完整 Learning Package](../../docs/16_3a_vision_to_motion.md) · [代码](vision_to_motion.py)。
CPU 彩色 landmark 图像→整图颜色 ID/质心→PnP→quality/frame/upright prior→IK/reference→
home/pre/approach 连续动力学。一次观测、固定物体，truth 仅用于图像生成与评价。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/vision_to_motion.py
python examples/14_vision_manipulation/vision_to_motion.py --noise-px 0.8
python examples/14_vision_manipulation/vision_to_motion.py --noise-px 0
python examples/14_vision_manipulation/vision_to_motion.py --rms-limit-px 0.1
```

需 NumPy/MuJoCo/Menagerie/Matplotlib/OpenCV headless（已声明）；不需 renderer/GUI/Torch。
输出 ignored `tmp/s13_3a_motion_noise*_rms*/`，默认模拟10.5 s、无 contact；
末端 tracking≈0.073 mm，但 true-target error≈4.05 mm。
noise=0 仍有 raster quantization。严格 RMS 对照在运动前拒绝。
Execution 不写 arm qpos，使用位置目标与显式 qfrc_bias 前馈；不是受真实扭矩限幅的硬件验证。
先预测 noise 0.2→0.8 对 tracking/true-target errors 的影响，再 Modify 对比。
Engineering Complete；本人于2026-10-06确认实验与预测，并回答五项Explain，Learning Mastered；状态见根README。
没有闭爪/lift/place/GUI，停止在 S13.3a。

## S13.3b — Vision Pick-and-Place

[完整 Learning Package](../../docs/16_3b_vision_pick_place.md) · [代码](vision_pick_place.py)。
复用 image/PnP/upright/candidate 与 P0 model/IK/contact concepts；free object 从 home 连续执行
approach/close/lift/transfer/descent/support/release/retreat，不在grasp初始化。
目标仅使用estimate、名义T_OG与commanded destination；object state仅用于评价，contact反馈用于阶段切换。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/vision_pick_place.py
python examples/14_vision_manipulation/vision_pick_place.py --close-target 0.014
```

预测第二条的空载开口及失败阶段后再运行；预期close failure退出1，partial trace仍保存。
默认22.228 s、lift52.728 mm、final error.714 mm、retreat79.974 mm；
逐阶段relative change≤15 mm，但累计23.846 mm，不代表无slip。
支撑持续确认后停止descent并开爪；post-release持续支撑，手指与地面接触仍属unexpected。
依赖与S13.3a相同，无新增包。PNG/CSV/NPZ/JSON在ignored `tmp/s13_3b_pick_place_close*/`。
Engineering Complete，本人已确认Run/Modify/Explain，并补正开口计算，Learning Mastered；状态见根README。
无GUI/真实相机/硬件验证，不自动开始S13.4。

## S13.4 — Perception Noise

[完整 Learning Package](../../docs/16_4_perception_noise.md) · [代码](perception_noise.py)。
固定同一image/PnP/scene/controller，7个确定性case分别改变pose或extrinsic的translation/rotation。
显式声明frame与rotation pivot；每case从home独立执行完整pipeline，不用object truth修正target。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/perception_noise.py
python examples/14_vision_manipulation/perception_noise.py --scale 0.5
```

先预测两个translation shifts的方向/长度、yaw lever shift与tilt gate，再比较task outcome。
默认1° base-origin yaw产生8.584mm origin shift；大pose偏移在close失败，6°外参tilt被prior拒绝。
scale.5的tilt通过prior但仍close失败；失败/拒绝没有final placement error，不能记作0。
post-fit pose fault保留original reported fit RMS，另存recomputed diagnostic RMS，不能混称拟合质量。
root comparison CSV/PNG/JSON及各case完整/partial trace在ignored `tmp/s13_4_noise_scale*/`。
baseline通过且7case评价完成时进程exit0；不代表所有case取放成功或统计success rate。
依赖同S13.3b，无新包；Engineering Complete；本人于2026-10-06确认实验、预测与五项Explain，Learning Mastered。
无GUI/真实camera/随机robustness验证，不自动开始S13.5。

## S13.5 — Repeated-Trial Evaluation

[完整 Learning Package](../../docs/16_5_repeated_trials.md) · [代码](repeated_trials.py)。
每trial独立采样object xy±5mm、camera xyz±20mm与image seed，从home执行完整pipeline。
准确外参、固定尺寸/朝向/物理/控制参数；target来自PnP estimate，reference collision采用simulator oracle。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/repeated_trials.py
python examples/14_vision_manipulation/repeated_trials.py --noise-px 0.8
python examples/14_vision_manipulation/repeated_trials.py --seed 7
```

预测配对noise改变对quality/失败率/条件误差的影响再运行；前两命令manifest相同。
默认6/8，seed7与noise.8各2/8；成功条件误差约.7mm，不代表全部trial都精准完成。
统计失败phase、成功率与Wilson区间、成功条件误差、wall/sim time；失败不从分母删掉。
Manifest/CSV/JSON/dashboard和紧凑sampled trace在ignored `tmp/s13_5_trials_seed*_n*_noise*/`。
无新依赖；make_image增加可选seed，前课默认观测逐字节保持一致。
Engineering Complete；Learning待本人Run/Modify/Explain；状态见根README。
无GUI/真实camera/硬件验证，不自动开始Stage14。
