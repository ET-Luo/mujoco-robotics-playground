# S12.4 — Camera Calibration

约 1～2h。[代码](../examples/13_perception_geometry/camera_calibration.py) ·
[状态唯一来源](../README.md#stage-12--robot-perception-geometry)。
前置：[Pinhole](15_2_pinhole_projection.md)、[RGB/depth](15_3_rgb_depth_acquisition.md)。

## Problem → Why

S12.2/3 直接给定 K；实际相机一般需要通过已知几何与观测 pixel 估计 K 和镜头畸变。
本课用多视角平面棋盘格的角点对应完成标定，评价训练误差、留出误差与 metric scale。
不实现图像角点检测、PnP lesson、robot-base extrinsic 或 hand-eye calibration。

## Intuition → Core Concepts

同一块已知尺寸的板，从不同倾斜/距离/位置观察，会产生不同 pixel patterns。
每张图的板位姿不同，但同一固定相机的 K 和畸变参数共享。联合找到一组共享参数，
使所有已知板点的预测 pixel 尽可能贴近观测，就是标定。
重复同一正面姿态通常不能像多样 tilt/coverage 一样约束参数；本课使用不同姿态和图像位置。

| 量 | meaning / shape / unit / frame |
| --- | --- |
| P_B | 标定板点，`(54,3)`，m，board frame B，平面 z=0；此处 B 是 board，非 robot base |
| pattern size | 9 columns × 6 rows **内角点**，不是格子数量；相邻内角点间距 0.03 m |
| rvec_i | board→optical-camera rotation R_CB 的旋转向量，`(3,)` 或 `(3,1)`，rad；非 Euler angles |
| tvec_i | board origin 在 camera 中的位置，`(3,)` 或 `(3,1)`，m，方向是 board→camera |
| K | `(3,3)`，fx/fy/cx/cy 为 pixel；所有视角共享 |
| d | `[k1,k2,p1,p2,k3]`，`(5,)` 或 `(1,5)`，无量纲；radial / tangential coefficients |
| image points | 每视角 `(54,2)`，pixel；OpenCV 接口使用 `(54,1,2)` float32 |
| σ | 每个 pixel coordinate 的 Gaussian noise 标准差，pixel；非二维距离 RMS |

本课 board frame 原点置于内角点网格中心，x 向列增长、y 向行增长；改变原点只改变 tvec 表达。
Camera 使用 optical x右/y下/z前；不需要 renderer frame 转换，因为输入已经是 optical projection。

## Mathematics：meaning / shape / unit / frame

先进行刚体变换，再透视除法和镜头畸变：

```text
P_C = R_CB P_B + t_CB
x = X_C/Z_C, y = Y_C/Z_C, r² = x²+y²
radial = 1 + k1*r² + k2*r⁴ + k3*r⁶
xd = x*radial + 2*p1*x*y + p2*(r²+2*x²)
yd = y*radial + p1*(r²+2*y²) + 2*p2*x*y
u = fx*xd + cx
v = fy*yd + cy
```

这是标准 pinhole + radial/tangential 模型，不是 fisheye。本课 true k3=0，并在求解中明确固定
k3=0；实际估计 fx/fy/cx/cy 与 k1/k2/p1/p2。不声称“恢复了 k3”，因为它没有参与优化。
模型和参数顺序见[OpenCV calib3d 官方参考](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)。

标定的目标是最小化全部 training views/corners 的重投影平方误差：

```text
min over K,d,{R_i,t_i}: Σ_i Σ_j || project(P_Bj,R_i,t_i,K,d) - observed_uv_ij ||²
RMS_2D = sqrt(mean(|| residual_uv ||²))                 # pixel per 2D corner
```

角点的 u/v 是两个坐标。本课 RMS 是二维距离 RMS，不是除以两个轴数的 coordinate RMS。
若已知真参数且仅有独立噪声，二维误差 RMS 约为 sqrt(2)*σ；实际 training fit 还拟合了参数，
不能把此近似当作严格上界，留出误差也没有普遍的 c*σ 上界。

为什么平面仍可标定？无畸变时每视角有 `H_i ~ K[r1_i,r2_i,t_i]`；旋转列的正交与等长约束
让多视角提供共享 K 的信息。OpenCV 做初始估计与非线性优化，本课不重写求解器。
流程参考[官方标定教程](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)。

尺度陷阱：若把全部 board points 乘 α，同时把 tvec 乘 α，则 camera XYZ 都乘 α，
X/Z、Y/Z 不变，pixel 与重投影误差不变。因此图像无法发现“方格尺寸写错了一倍”。
K/d 可以正确而 metric translation 错误；真实尺寸必须测量。

## Math-to-Code / APIs

1. NumPy 建立 9×6 平面内角点，间距 0.03 m；生成 32 个不同且在图像内的 board poses。
2. `projectPoints` 用已知 K/d 生成干净角点，再叠加 per-axis Gaussian noise。
3. 前 24 个视角 calibration，后 8 个 held-out；geometry RNG 与 noise RNG 分开。
4. `calibrateCamera` 仅接收 board geometry、training pixels、image size，不接收 true K/d/poses。
5. 独立重新投影核对 OpenCV training RMS；在未使用过的视角计算 known-pose held-out error。
6. 额外比较强制无畸变的错误模型、以及 board size ×2 的尺度歧义。

| API | inputs → outputs / effects | 本课用途 |
| --- | --- | --- |
| `cv2.Rodrigues(rvec)` | `(3,)` rad → `(3,3)` rotation、Jacobian | 检查采样板点的 camera depth；不是关节 FK |
| `cv2.projectPoints` | board points、rvec/tvec、K/d → `(N,1,2)` pixel、Jacobian | 正向畸变投影与残差计算，Jacobian 本课不用 |
| `cv2.calibrateCamera` | 每视角 object/image point lists、`(W,H)`、初始 K/d、flags/criteria → RMS,K,d,rvecs,tvecs | 联合估计共享内参和各 training board pose |
| `CALIB_FIX_K3` | 标志位；k3 固定默认零 | 避免本课引入额外高阶未知量，明确模型限制 |
| `CALIB_ZERO_TANGENT_DIST` 与 `FIX_K1/2/3` | tangential/radial 均固定零 | 错误模型对照，不当作正确相机结果 |
| `cv2.setNumThreads(1)` | 当前进程 CPU thread budget | 保持小实验资源预算；不是跨版本逐位复现保证 |

`imageSize` 为 `(width,height)`，与图像数组 `(height,width)` 不同。
`cameraMatrix=None, distCoeffs=None` 使用平面标定初始化，不把真值作为 initial guess。
接口中 object/image 点为 float32，最终 K/d/poses 为浮点数组；转换精度会留下微小无噪声 residual。
`calibrateCamera` 返回的 tvec 是 board→camera，**不是 camera→robot-base 外参**。

## Minimal Experiment / Dependency

新增 `opencv-python-headless>=4.10,<5` 到 requirements.txt。
用途仅 calib3d，不使用 OpenCV GUI；安装前 dry-run 确认现有 NumPy 满足依赖。
2026-10-04 在 mujoco 内安装 opencv-python-headless 4.14.0.94（cv2 version 4.14.0），
原 NumPy 2.5.3 未改变；没有安装到 base/system Python，没有新增模型训练依赖。
OpenCV 官方网页现场为 4.13.0，本文 API 也在实际 4.14.0 环境执行核验。

从根目录核验环境后运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/camera_calibration.py
python examples/13_perception_geometry/camera_calibration.py --noise-px 0.6
```

若重建环境且 OpenCV 缺失，仅在核验的 mujoco 下安装：

```bash
python -m pip install 'opencv-python-headless>=4.10,<5'
```

默认 σ=0.15 pixel/axis、seed=20261004。Modify 将 σ 改成 0.6，板位姿与标准化噪声样本保持一致。
只改变角点定位噪声，不同时改变棋盘几何、coverage 或镜头参数。

## Held-out Evaluation：明确实验边界

- training noisy RMS：用联合估计的 K/d 与每张图估计的 pose，比较 training noisy pixels。
- held-out known-pose clean RMS：用估计 K/d 与**合成真 pose**预测未使用过的视角，比较 clean pixels。
- held-out known-pose noisy RMS：同一预测与 held-out noisy observations 比较。

留出 poses 只用于评价，不输入 calibration；没有拟合 held-out pose，也没有调用 solvePnP。
这个 known-pose 指标隔离共享 K/d 的误差，往往比重新拟合 test pose 后的误差更严格；
不能把它称为真实未知位姿照片的端到端检测/PnP test。

数据来自已知角点对应，PNG 为 corner coverage/residual plot，**不是渲染棋盘照片**。
本课隔离 calibration 数学，不测试角点检测、subpixel extraction、遮挡、模糊、真实镜头或 sensor noise。
生成与拟合使用匹配的 OpenCV 模型；无噪声恢复不证明真实标定泛化。

## Expected / Actual Result → Explanation

2026-10-04 在重新核验的 WSL2 Ubuntu 24.04.5 / mujoco 环境运行：
Python `/home/lucas/miniconda3/envs/mujoco/bin/python` 3.12.14，cv2 4.14.0、NumPy 2.5.3。
代码目标兼容 3.11，未在 3.11 执行。三条最终命令退出 0，无 import error：
默认、`--noise-px 0.6`、额外 `--noise-px 0`。

| 量 | 无噪声 σ=0 | 默认 σ=0.15 | Modify σ=0.6 |
| --- | --- | --- | --- |
| training noisy RMS pixel | 0.000010 | 0.201785 | 0.807089 |
| held-out known-pose clean RMS pixel | 0.000043 | 1.514152 | 6.207453 |
| held-out known-pose noisy RMS pixel | 0.000043 | 1.520972 | 6.234837 |
| no-distortion held-out clean RMS pixel | 11.428897 | 11.718318 | 12.660623 |
| fx estimate pixel（truth=560） | 560.000006 | 560.284797 | 561.116486 |
| fy estimate pixel（truth=580） | 579.999998 | 580.168706 | 580.649517 |
| cx estimate pixel（truth=319.5） | 319.500000 | 319.275952 | 318.108619 |
| cy estimate pixel（truth=239.5） | 239.499959 | 238.034964 | 233.584163 |
| 主 calibration fit ms | 8.927 | 18.808 | 13.745 |

default d estimate=`[-0.1408967,0.0557273,0.0006033,-0.0010066,0]`，
truth=`[-0.14,0.05,0.001,-0.0008,0]`。k3 固定零，不是精度验证。
两组 noisy data 的 calibration 可运行，但不声称达到 subpixel known-pose test accuracy。
较大的 σ 在本 fixed-seed 试验中造成更大参数误差/留出误差；不由单 seed 推断普遍比例关系。
训练 poses 可以吸收一部分内参偏差，所以 training residual 小不代表 K 已精确恢复。
无畸变模型更差，说明本数据的非零 distortion 不应简单忽略。

board size ×2 的三组对照均保持 K/d 与 RMS 基本不变，全部 estimated tvec ×2；
这直接验证 metric scale 无法只由 reprojection RMS 认证。
源码独立 RMS tolerance=2e-5 pixel，scale K tolerance=2e-3 pixel、translation tolerance=1e-5 m；
这些是合成数值检查，不是现实标定精度承诺。

最初把 held-out error 当作 `<8σ+0.02` 的断言，默认/Modify 退出 1；
该噪声倍数上界没有推导依据，已移除，改为报告实际 held-out error 与参数真值误差。
最终 PASS 表示代码/统计/尺度实验通过，**不代表满足任意操作精度需求**。

产物在忽略目录 `tmp/s12_4_calibration_noise*_seed20261004/`：
calibration.json（K/d/errors/poses/time）、corner_data.npz、8 行 heldout_errors.csv、calibration.png。
已目视核对默认图的 coverage、残差方向与误差柱状图。
CLI 的 nan/negative sigma 拒绝（退出 2）；saved arrays/metrics 与尺度结果独立核对通过。
没有重跑 P0/S12.1–3、没有 GUI 或硬件相机验证。

## Failure Cases

| 错误 | 本课证据 / 后续未验证边界 |
| --- | --- |
| 方格尺寸错误 | ×2 实验验证：pixel RMS 不揭示 metric translation 尺度错误 |
| 忽略镜头畸变 | 固定 distortion=0 对照在留出集明显更差 |
| 只看 training RMS | noisy 实验 training 小、known-pose test 较大；不能用训练误差认证 K 精度 |
| 把 board→camera 当 camera→base | calibration poses 是各视角标定板姿态；base/hand-eye 需另做标定 |
| inner corners 与 squares 混淆 / 排序错配 | 本课已知对应；真实检测中需保证列/行与物理点顺序一致，未运行检测器 |
| 正面重复视角 / coverage 太窄 | 几何上会削弱参数约束；本课未做退化数据对照，不宣称验证了其数值阈值 |
| noise/outlier/畸变模型不匹配 | 本课仅 Gaussian corner noise 与指定模型；未验证鲁棒 outlier rejection |
| 放大分辨率后直接复用 K | 需按像素采样约定改变 focal/principal point；d 无量纲，本课未执行 resize 对照 |
| 图像生成与校准用同一模型 | 是受控数学实验，不能替代真实数据、不同模型与实际定位精度评价 |

## Robotics Context

K/d 是将来 PnP 的相机输入，像素误差会通过这些共享参数影响估计的 metric pose。
board size 的误差尤其会影响以米表达的距离；camera→robot-base 的变换仍留给后续任务。
不连接真实机器人、不引入视觉模型或 MoveIt。

## Interview Capsule

**30 秒**：相机标定用已知板几何与多视角 2D corners 联合估计共享 K/d 和各板 pose。
多样 tilt/coverage 约束参数，不能只重复正面图。评价要分 training 与 held-out residual；
这里 test pose 已知，只隔离内参误差。方格尺寸定 metric scale，低 reprojection RMS 不能发现统一尺度错误。

**2 分钟**：说明 optical frame、米制 board points、pixel observations，9×6 是内角点。
给出 R/t→normalized projection→radial/tangential distortion→K 的链与 reprojection objective。
解释 calibrateCamera 的输入/返回和 OpenCV rvec/tvec 的 board→camera 方向。
列出多视角共享 K/d，但每张图拥有不同 pose，所以训练 poses 可吸收部分参数偏差。
报告留出时使用真 pose 的评价边界，不能称为未知 test image 的 PnP 性能。
再用 board/tvec 同乘 α 而投影不变说明尺度歧义。
最后说明模型复杂度：本课固定 k3，仅估计低阶畸变；真实实践还需角点质量与独立数据检查。

## Must Remember

- 同一个固定相机共享 K/d；每个 calibration view 具有不同的 board→camera pose。
- fx/fy/cx/cy 为 pixel、d 无量纲、tvec 的单位继承 board size。
- training RMS、known-pose held-out RMS 与现实定位精度是不同证据。
- calibration 不自动得到 camera→robot-base transform。

## My Verification / Run → Modify → Explain

状态仅在根 README；不以助手运行代替本人完成。

本人验证（2026-10-04）：明确确认实验与对比分析均完成，正确列出已知板点/pixels/image size，
区分共享 K/d 与各视角 board pose，解释多样视角缓解参数混淆；正确说明 K 四参数为 pixel、
畸变无量纲，k3 固定而非估计。本人说明低 training RMS 只证明解释 training observations，
不保证 K 准确；metric scale 必须由已知尺寸注入；rvec/tvec 描述 board→camera，未得到 camera→base。
助手补充：本课 held-out 使用合成真位姿，仅用于评价，不参与 calibration 或 test-pose fitting。
board size 与 tvec 同乘尺度后，透视比值不变。Run/Modify/Explain 全部确认，Learning Mastered。
本次仅同步学习记录，不新增 runtime 证据；不自动开始 S12.5。

**Run**：默认命令；读取 K/d、training/held-out errors 与尺度对照，查看 JSON/PNG。
**Modify**：先预测增大 noise sigma 对估计误差的影响，运行 `--noise-px 0.6`；
比较训练误差、留出误差与 cx/cy，说明当前 trial 的结果不等于普遍误差比例。
**Explain**：

1. calibration 输入哪些已知量、输出哪些未知量？为什么需要不同 tilt/位置的多视角？
2. K 与 distortion d 的意义/单位是什么？为何本课的 k3=0 不算“估计准确”？
3. training RMS 很小，为什么不能断言 K 精确或真实机器人定位准确？本课 held-out pose 从哪里来？
4. square size 写成两倍，为何 pixel RMS 和 K 几乎不变，tvec 却变成两倍？
5. calibrateCamera 的 rvec/tvec 属于哪两个 frame？是否已经得到 camera→robot-base 外参？

Engineering 完成后 STOP，不自动开始 S12.5。
