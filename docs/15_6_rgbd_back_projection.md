# S12.6 — RGB-D Back Projection

约 1～2h。[代码](../examples/13_perception_geometry/rgbd_back_projection.py) ·
[采集前课](15_3_rgb_depth_acquisition.md) · [状态](../README.md#stage-12--robot-perception-geometry)。

## Problem → Why

输入对齐的 RGB、米制 depth、已知 K 和 camera→base 外参，将有效像素还原成有颜色的
camera/base 表面点云。机器人需要米制几何来描述表面位置；像素本身没有距离。
本课只做 back projection，不估物体 pose，不做 ICP、外参标定或机器人执行。没有新依赖。

## Intuition → Core Concepts

像素确定一条射线；depth 给出该射线与可见表面交点的 optical Z。
K inverse 的射线 z 分量为 1，因此乘 Z 即得到 XYZ。
射线不需要归一化：若归一化后仍乘 Z，相当于把光轴深度误当 radial range。
RGB 与 depth 必须同尺寸、同像素中心、同视角且对齐。可见表面点不等于物体中心，
单视角也无法恢复遮挡后面的完整物体。

| 量 | shape / 单位 / frame |
| --- | --- |
| depth | `(H,W)` float32 输入；m；optical Z，非 range |
| K | `(3,3)`；fx/fy/cx/cy 为 pixel；本课 ideal zero-skew、无畸变 |
| uv | `(N,2)`；pixel；u 列、v 行，整数代表像素中心 |
| valid mask | `(H,W)` bool；有限且 near < Z < far |
| points_C / points_B | `(N,3)` float64；m；optical camera / actual UR5e base |
| RGB colors | `(N,3)` uint8；RGB 0–255，和 points 使用同一 mask/order |
| T_BC | `(4,4)`；平移 m；C 坐标转换到 B，不是反方向 |

## Mathematics：meaning / shape / unit / frame

```text
q = [u,v,1]^T
r_C = K^-1 q = [(u-cx)/fx, (v-cy)/fy, 1]^T
p_C = Z r_C
X_C = Z (u-cx)/fx; Y_C = Z (v-cy)/fy; Z_C = Z
range = ||p_C|| = Z ||r_C||
T_BC = inverse(T_WB) T_WC
p_B = R_BC p_C + t_BC
```

C 为 x 右/y 下/z 前。renderer M 为 x 右/y 上/z 后，
`R_WC = R_WM @ diag(1,-1,-1)`。外参取自静态模型，是已知 truth，未做 hand-eye。
实际 UR5e `base` 的旋转本次核验为 diag(-1,-1,1)、原点为 world 原点；
因此本实验 X_B=-X_W、Y_B=-Y_W、Z_B=Z_W，不能普遍把 base 当 world。

默认 320×240、fovy=45°：fx=fy=289.705627 pixel，cx=159.5、cy=119.5。
不能混用前课合成 K，也不能对畸变图像直接使用本公式：先去畸变或使用对应反投影模型。

## Math-to-Code / API

`back_project(depth,K,near,far)` 返回新的 points、uv、mask，不修改输入。
`np.nonzero(valid)` 返回 v/u 行列；`depth[valid]` 和 `rgb[valid]` 均按相同行优先顺序提取。
代码保留显式 `np.linalg.inv(K)`，使 K inverse 与公式直接对应；不是迭代估计器。
行向量批量表示需要 `points_C @ R_BC.T + t_BC`；转置来自存储方式。

复用 `capture(renderer,data)` 的同状态 RGB/depth 采集。
`MjModel.from_xml_path(path)` 返回 compiled scene；`MjData(model)` 返回状态与 cache；
`mj_forward(model,data)` 返回 None，原地更新 cam/body world poses，不推进时间。
`Renderer(model,width,height)` 返回 offscreen context；`update_scene` 原地构建场景，
切换 depth mode 后 `render()` 返回 RGB 或 metric Z 数组；context manager 关闭资源。
`mujoco_menagerie.load('universal_robots_ur5e')` 返回既有官方机器人模型，单独 MjData 的
`xmat/xpos` 读取实际 base pose；只读 frame，不渲染或驱动机器人。
这些 API 的采集细节见前课。本课没有 mj_step，两个模型 time 均为 0。

## Minimal Experiment / Run → Modify

仓库根目录执行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/rgbd_back_projection.py
python examples/13_perception_geometry/rgbd_back_projection.py --camera-height 1.0
```

**Run**：读 summary.json、cloud.png。NPZ 保存完整点云、RGB、uv、mask、depth、K 和外参；
产物位于 ignored `tmp/s12_6_rgbd_z0.8/` 和 `tmp/s12_6_rgbd_z1/`。
PNG 仅为展示，按二维像素网格稀疏采样；不改变保存的完整数据。

**Modify**：先预测相机升高后 K、depth、base 表面高度、视野覆盖范围，再运行第二条命令。
K 不变；floor/top depth 从 0.80/0.74 变为 1.00/0.94 m，base floor/top 高度仍为 0/0.06 m。
两张点云对应不同视野和像素采样，不能要求全部点坐标逐点相等。

## Expected / Actual Result → Explanation

2026-10-05 助手现场验证：WSL2 kernel 6.6.87.2、Ubuntu 24.04.5，mujoco 环境及
`/home/lucas/miniconda3/envs/mujoco/bin/python`；Python 3.12.14、MuJoCo 3.14.0、NumPy 2.5.3。
3.11 为兼容目标，未在 3.11 执行。上述两条脚本均退出 0，无 import error；
EGL renderer 为 `llvmpipe (LLVM 20.1.2, 256 bits)`，CPU 验证成功。

| 检查 | z=0.8 | z=1.0 |
| --- | --- | --- |
| 有效点数 | 76,800 | 76,800 |
| 最大 pixel roundtrip error | 5.68e-14 px | 2.84e-14 px |
| floor patch mean world z | 4.77e-8 m | 0 m |
| orange top mean world z | 0.059999990 m | 0.060000002 m |
| 错把 Z 当 range 的 floor patch mean z | 0.126922 m | 0.158652 m |

无限地板覆盖全图，因此全图有效不代表现实传感器没有缺测。
独立手算 K=(100,100,0,0)、pixel=(1,0)、Z=2 得到 (0.02,0,2) m；
注入 NaN/inf/0/negative/near/far，全部排除；全无效输入返回 `(0,3)` 点和 `(0,2)` uv。
检查实际 floor/top 高度、base/world 链和颜色 shape/order；重新投影检验 uv。
roundtrip 只证明代数一致：错误深度也能返回相同 pixel，因此必须有独立米制表面检查。
默认 PNG 已目视检查。无 GUI、真实相机、噪声、pose estimator 或机器人执行验证。
助手执行不计 learner Run。

## Failure Cases

| 错误 | 结果与处理 |
| --- | --- |
| Z 当 range | 离轴点向 camera 收缩，floor 曲起；正确使用未归一化 ray × Z |
| mm 当 m | 点云尺度/外参平移不匹配；输入显式转换米制 |
| u/v 互换或图像再 flip | 点云轴反向/几何错位；depth[v,u]，复用前课返回图像方向 |
| far-plane 当表面 | 虚构背景点；finite 不够，需要 near/far 和传感器有效标记 |
| RGB/depth 未配准 | 颜色对应错误表面；真实设备需 registration 与匹配 intrinsics |
| 畸变或 resolution 改变 | ray 错误；使用对应 K 与 distortion model |
| T_CB 当 T_BC | frame 链错；明确输入/输出坐标系，检验独立 surface truth |
| 把表面均值当物体中心 | 受遮挡/视角/采样影响；本课不做物体中心或姿态估计 |

本课 near/far 是渲染边界，真实设备应结合 confidence、无效编码和工作范围。
外参错误不影响 camera cloud 的 pixel roundtrip，但会破坏 base cloud；不能据 roundtrip 认定外参准确。

## Robotics Context

RGB-D 点云可为后续表面匹配、场景几何与 grasp 提供输入。单帧表面还需要分割、配准、
姿态估计及执行质量检查才成为操作系统。本课在几何输入边界停止。

## Interview Capsule

**30 秒**：像素给 ray，axial depth 给 Z，`p_C=Z K^-1[u,v,1]`。
ray z=1，不归一化。用已知 T_BC 转到 base，同 mask 保持 RGB 对齐。
过滤无效深度，独立已知平面核验米制几何，roundtrip 不证明 depth/extrinsic 准确。

**2 分钟**：先交代 ideal K、pixel-center、米制 depth 与 optical axes；推导 XYZ 三式，
对比 range=Z||ray||。说明向量化 mask/uv/RGB 顺序和 row-vector rotation 转置。
解释 T_BC=inverse(T_WB)T_WC，实际 base 与 world xy 轴相反。
给出 floor/top 独立核验和 normalized-ray 失败实验，说明相机升高改变 depth/extrinsic、
不改变固定 resolution/fovy 下的 K。最后指出遮挡、畸变、RGB-depth alignment 与已知外参边界。

## Must Remember / My Verification

状态只在根 README；Learning Run/Modify/Explain 尚待本人明确报告。

- Z 是 optical-axis depth；K inverse ray 的 z=1。
- 有效 depth、pixel-center K、RGB 对齐、米制外参缺一不可。
- 表面点不等于物体中心；camera/base/world frame 必须明确。

**Explain**：

1. 为什么 K inverse ray 不需要单位化？若 depth 是 radial range，公式应如何改变？
2. 为什么 depth[v,u]，而 uv 储存 (u,v)？怎样确保 RGB 与点不打乱？
3. T_BC 怎样由 T_WB/T_WC 得到？为什么本例 base xy 与 world 相反？
4. 相机升高后哪些量变化、哪些不变？为什么不能逐点比较整张点云？
5. 为什么 pixel roundtrip 通过仍可能存在错误 depth 或外参？怎样独立验证？

Engineering 完成后 STOP；下一可选小任务 S12.7a 等待明确请求。
