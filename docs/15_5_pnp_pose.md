# S12.5 — Perspective-n-Point / Object Pose Estimation

约 1～2h。[代码](../examples/13_perception_geometry/pnp_pose.py) ·
[状态唯一来源](../README.md#stage-12--robot-perception-geometry) ·
[前课 Calibration](15_4_camera_calibration.md)。

## Problem → Why

已知 K/d 后，面对一组物体 3D 特征与图像 2D 对应，怎样求物体在相机中的位置与朝向？
PnP 将 pixel observations 转成 `T_CO`，补上 Camera→Object Pose Estimation 的几何环节。
这里仅估计 camera-frame pose；不做图像检测、camera→base 接入、grasp、RGB-D 或机器人动作。

## Intuition → Core Concepts

单个 pixel 只给一条 camera ray，无法指定深度；但多个已知相对位置、尺度和身份的 3D 点，
在同一刚体 pose 下必须同时落到各自的观测 rays。求解满足这些约束的 R/t，就是 PnP。
P 是 perspective、n 是点数；metric 3D geometry 提供尺度，不需要 depth image。

| 量 | meaning / shape / unit / frame |
| --- | --- |
| object points | `(12,3)` float64，m，在物体 O 中，已知非共面 landmark geometry |
| image points | `(12,2)` float64，pixel，在 optical image 中；每一行与同索引 3D 点对应 |
| K / d | `(3,3)` / `(5,)`，已知相机参数；K 四参数 pixel，d 无量纲 |
| rvec | `(3,1)`，rad，R_CO 的旋转向量参数；不是 Euler angles 或机器人 joint angles |
| tvec | `(3,1)`，m，object origin 在 optical camera C 中的位置 |
| T_CO | `(4,4)`，把 object coordinates 变到 optical camera coordinates |
| T_OC | inverse pose，把 camera coordinates 变到 object；其平移是 camera center in O |

OpenCV camera 约定 x右/y下/z前，与 S12.1/2 optical frame 相同。
solvePnP 返回 object→camera R/t，见[官方 PnP 文档](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)。
本课用 SOLVEPNP_ITERATIVE：对于非共面点，DLT 初值需要至少 6 个对应，再做重投影误差优化。
这不是所有 PnP 方法的共同最小点数；本课不比较 P3P/EPnP/IPPE/RANSAC。

## Mathematics：meaning / shape / unit / frame

```text
p_Cj = R_CO p_Oj + t_CO
predicted_uv_j = distort_and_project(p_Cj, K, d)
min over R_CO,t_CO: Σ_j || predicted_uv_j - observed_uv_j ||²

T_CO = [R_CO  t_CO]         T_OC = [R_CO.T  -R_CO.T @ t_CO]
       [0 0 0   1 ]                [0 0 0          1    ]
```

内参在此固定，不像 S12.4 同时拟合 K/d 和每视角 pose。
若物体原点在几何中心，tvec 就是该中心的 camera coordinates；若原点换到其他部位，
tvec 表示新的原点位置，不能不检查 object model 就把它称为物体中心。

位姿误差与 pixel residual 分开：

```text
translation_error = ||t_est - t_true||₂                  # m
R_delta = R_est @ R_true.T
rotation_error = acos(clip((trace(R_delta)-1)/2,-1,1))    # rad，报告时转 degree
reprojection_RMS = sqrt(mean(||predicted_uv-observed_uv||²))  # pixel per 2D point
```

不能用 rvec 逐元素差的范数普遍代替 SO(3) angle；相对旋转 metric 无需重教前课 orientation IK。
角度 clip 防止 floating-point trace 越界；极小误差的 acos 有精度限制，不报告无限精确的旋转。
真位姿误差在现实未知 pose 场景通常不能直接算，本课因为 synthetic truth 才能评价。

## Math-to-Code / API

代码直接暴露核心调用：

```python
success, rvec, tvec = cv2.solvePnP(
    object_points, image_points, K, distortion,
    useExtrinsicGuess=False, flags=cv2.SOLVEPNP_ITERATIVE,
)
R_CO, _ = cv2.Rodrigues(rvec)
```

不把 true pose 当初值；truth 只生成观测/评价结果。
输入需要同索引匹配的 finite contiguous float arrays；本课要求 >=6、object point rank=3。
这是本课选用非共面路线的输入限制，不代表 OpenCV 不能处理平面点。

| API | 输入 → 输出 / effects | 本课作用 |
| --- | --- | --- |
| `cv2.solvePnP` | object/image points、K/d、flags → bool,rvec,tvec | 估计 object→camera pose；不改 robot state，不估计 K |
| `cv2.Rodrigues` | rotation vector → R、Jacobian | 得到 `(3,3)` R_CO；Jacobian 不用 |
| `cv2.projectPoints` | object points、estimated rvec/tvec、K/d → pixels、Jacobian | 独立计算 fitted reprojection residual |
| NumPy rank / finite checks | 中心化的点集/输入数组 → 约束检查 | 阻止退化或非法输入进入本课求解器 |
| NumPy homogeneous inverse | R/t → T_OC | 核对 frame 方向，不把 camera center 当 tvec |

复用 S12.4 `project`/`rms_2d` helper；不运行 calibrateCamera、不加载忽略目录中的标定产物。
本课**给定准确 K/d**作受控 PnP baseline，再单独用 focal+5% 模拟错误内参。
这不等于已串联前课 estimated calibration；真实 calibration→PnP pipeline 与噪声评价留给 Stage 13。

success=True 只表示 API 返回结果；还检查 finite R/t、RᵀR=I、det=+1、全部点 Z_C>0，
并报告残差。合法 rotation/positive depth 仍不能证明输入匹配正确或达到任务精度。

## Minimal Experiment

12 个有唯一身份的 abstract non-coplanar landmarks，Object XYZ 范围约 ±0.06 m。
它们不是不透明方块所有可见角点的 detector 输出；没有渲染、遮挡推理或 landmark extraction。
已知 point identity/order 是实验前提，不是 PnP 自动发现的结果。

设定 K=`[[560,0,319.5],[0,580,239.5],[0,0,1]]`，
d=`[-0.14,0.05,0.001,-0.0008,0]`；true rvec=`[0.25,-0.35,0.15]` rad，
true tvec=`[-0.10,-0.10,0.77]` m。噪声只加在 u/v，per-axis sigma 与 fixed seed 控制。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/pnp_pose.py
python examples/13_perception_geometry/pnp_pose.py --noise-px 0.8
```

默认 σ=0.2 px、seed=20261004；Modify 只改 σ=0.8，points、pose、K/d 与标准化噪声样本不变。
既有 OpenCV headless 已满足需求，本轮不安装/升级依赖，不需要 GUI/GPU/MuJoCo renderer。

## Expected / Actual Result → Explanation

预期：无噪声恢复真 pose；较大 pixel noise 在当前 fixed seed 下会增加 pose/residual error。
不能推断每个 seed、geometry 或 σ 都严格单调。错误 K 可以让 pose 吸收部分误差，
即使 noisy reprojection RMS 很小，metric pose 仍有偏差。

2026-10-04 助手现场核验 WSL2 Ubuntu 24.04.5、mujoco env，interpreter
`/home/lucas/miniconda3/envs/mujoco/bin/python`，Python 3.12.14 / cv2 4.14.0 / NumPy 2.5.3。
代码兼容目标 3.11，未在 3.11 执行。默认、`--noise-px 0.8`、额外 `--noise-px 0` 三命令均退出 0。

| 量 | 无噪声 | 默认 σ=0.2 | Modify σ=0.8 |
| --- | --- | --- | --- |
| translation error m | <1e-8 | 0.000389763 | 0.001472155 |
| rotation error degree | <1e-4 | 0.378563 | 1.555400 |
| noisy reprojection RMS pixel | <1e-6 | 0.258050 | 1.029309 |
| clean reprojection RMS pixel | <1e-6 | 0.155588 | 0.632125 |
| wrong focal+5% translation error m | 0.038429496 | 0.038792973 | 0.039790076 |
| wrong focal+5% rotation error degree | 0.502328 | 0.855102 | 2.010723 |
| wrong focal+5% noisy RMS pixel | 0.080909 | 0.264791 | 1.026455 |
| solve + geometric-check latency ms | 13.444 | 40.425 | 10.511 |

默认 estimated tvec=`[-0.1001754,-0.1000216,0.7703474]` m；
estimated camera center in O=`[-0.1663805,-0.0884730,-0.7602398]` m，二者不是同一物理表达。
T_OC @ T_CO=I、proper rotation、positive depths、non-coplanar rank 与无噪声恢复通过。
collinear 输入在求解前被拒绝；finite/too-few-input checks 也已单独验证。
默认与 Modify raw data 固定几何与 noise ×4 的检查通过，saved pose/residual/error 独立核对通过。
CLI nan/negative noise 拒绝（退出 2）；默认 PNG 已目视检查。

错误 K 的 Modify RMS 甚至略低于正确 K（1.026455 vs 1.029309 pixel），
但 translation error 约 40 mm 而非 1.47 mm。这不是说错误 K 更好；有限 noisy data 中
pose 可以吸收系统误差，RMS 的微小差异不能替代 metric truth comparison。
forward/inverse self-consistency 同样不能认证位姿准确，它只验证表示与代数。

latency 是单次 solve + rank/geometry/assert checks，包含首次调用开销的可能影响；
未拆分 benchmark，不能当作稳定 pose solver latency 或 pipeline FPS。

产物在 `tmp/s12_5_pnp_noise*_seed20261004/`：pnp.json、pnp_data.npz、
12 行 correspondences.csv、pnp.png。仅合成点与图表，不生成真实相机图像。
无 detector、PnP-RANSAC、物体对称歧义消解、camera→base transform、IK/trajectory 或抓取验证。

## Failure Cases

| 失败 | 证据与处理边界 |
| --- | --- |
| 错误 K/d | 本课 focal+5% 对照验证低 RMS 仍可有数厘米 pose error；d 错误影响未单独测量 |
| collinear / inadequate geometry | 本课输入 rank 检查拒绝共线；平面方法有自身约束/歧义，不由此推断所有平面 PnP 无效 |
| 对应点乱序/错配 | PnP 不检测 landmark identity；本课不实施 RANSAC/outlier rejection |
| object尺寸错 | metric translation 继承 model 尺度；尺度实验已在 S12.4 完成，本课不重跑 |
| tvec 当 camera center / T_OC 当 T_CO | 读取 frame 定义；camera center in O 是 −Rᵀt |
| 数值解在 camera 后方 | positive-depth 检查拒绝；本课没有 exhaustive multiple-solution search |
| noise、poor image coverage、对称点 | 参数可敏感或歧义；本课只测一个 asymmetric labeled point set，不声称通用 robustness |
| 先 undistort pixels 却仍传原 d | 会重复畸变处理；本课输入原 distorted pixels 与对应 d，未测预去畸变链 |
| API success 就当 grasp成功 | 缺少真实匹配、base extrinsic、可达性、避障和执行证据 |

## Robotics Context

PnP 输出可以作为之后 `T_BO=T_BC@T_CO` 的 pose source，前提是外参、时间戳与 frame 约定准确。
本课只交付 T_CO；Stage 13 才接 perception→base→grasp→existing world-frame IK。
PnP 不需要 depth image，但需要可识别的 metric object geometry；任意未知物体不能直接套用本输入。

## Interview Capsule

**30 秒**：PnP 已知 K/d 和有身份的 metric 3D↔2D 对应，求最小重投影误差的 object→camera R/t。
OpenCV optical x右/y下/z前，tvec 是 object origin in camera，不是 camera center。
评估需分 pixel RMS 与 metric pose error，错误内参可以被 pose 吸收；API success 不等于精度保证。

**2 分钟**：对比 calibration 与 PnP：前者估共享内参，后者固定内参、估一个物体 pose。
用多点已知相对几何同时约束投影 rays，解释无 depth 仍可获得 metric pose，但尺度来自 model。
明确 `(N,3)` 米、`(N,2)` pixel、旋转向量 rad 与 T_CO direction。
说明本课 iterative method 从 non-planar DLT 初值开始，至少六点，不使用 truth initial guess。
求解后检查 proper rotation、positive depths、重投影 residual，再用 synthetic truth 评价平移距离与 rotation angle。
最后展示 focal+5% 的低 pixel RMS/高 metric error，说明校准与对应质量是必要前提；
指出实验没有 detector、occlusion 或 manipulation pipeline，不能扩大结论。

## Must Remember

- solvePnP 输出 T_CO，输入 3D frame 决定返回 pose 的 object frame。
- 不需 depth image，但需已知 metric geometry、K/d 与正确点匹配。
- 小 reprojection error 不保证 metric pose 精确；inverse 一致也不能认证估计准确。

## My Verification / Run → Modify → Explain

状态只在根 README；本人于 2026-10-04 明确确认实验与验证均完成，并提交五问解释。
解释覆盖已知量/未知 R/t、联合几何约束、tvec 与 −Rᵀt、内参/位姿补偿和合法性检查边界。
精确补充：metric scale 来自已知 3D model 的尺寸，多点约束求解该尺度下的位姿；
calibration 估计共享 K/d，PnP 将 K/d 作为已知输入。没有提交新的数值结果，不据此新增性能结论。

**Run**：默认命令，确认 estimated T_CO、translation/rotation/RMS 与 wrong-K 对照，查看 JSON/PNG。
**Modify**：先预测 pixel noise σ=0.8 的影响，再运行；比较 pose error 与 residual，解释单 seed 的证据边界。
**Explain**：

1. PnP 已知哪些量、估计哪些量，与 camera calibration 有何区别？
2. 为什么单 pixel 不能定深度，多点 known metric geometry 却可以在没有 depth image 时估 pose？
3. 返回的 rvec/tvec/T_CO 把哪个 frame 变到哪个 frame？camera center in O 怎么求？
4. 为什么 focal+5% 的 reprojection RMS 仍小，translation error 却很大？
5. success=True、proper rotation 与 positive depth 是否足以证明真实 pose/grasp 正确？还缺哪些证据？

Engineering 完成后 STOP，不自动开始 S12.6。
