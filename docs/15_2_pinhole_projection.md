# S12.2 — Pinhole / Intrinsic / 3D→2D Projection

约 1～2h。[代码](../examples/13_perception_geometry/pinhole_projection.py) ·
[状态唯一来源](../README.md#stage-12--robot-perception-geometry) ·
[前课坐标约定](15_robot_perception_geometry.md)。

## Problem → Why

S12.1 已把世界物体变到 optical camera frame；现在回答：这个 3D 点在图像哪个位置？
投影连接几何世界与像素，是 calibration、PnP、RGB-D 的共同基础。
本课输入已知 3D 点和已知 K，输出像素；不估计 K 或物体 pose。

## Intuition → Core Concepts

小孔把来自空间各点的光线汇聚。我们采用位于小孔前方的**虚拟成像平面**，
使 optical +x 对应图像右、+y 对应图像下、+z 朝前；真实孔后感光平面的倒像已由此约定处理。
相似三角形给出归一化坐标 X/Z、Y/Z。相同 X 在较近 Z 下有更大的横向角度，离主点更远。
焦距以像素表示，负责把归一化坐标缩放到像素；主点提供图像坐标偏移。

| 量 | 含义 / shape / 单位 / frame |
| --- | --- |
| p_C=[X,Y,Z] | `(3,)`，m，optical camera frame；Z 是沿光轴的深度，不是欧氏距离 |
| X/Z、Y/Z | 归一化成像平面坐标，无量纲 |
| fx、fy | 焦距的像素表示，pixel；可因像素尺寸不同而不相等 |
| cx、cy | 光轴与成像平面交点的像素位置，pixel；不保证等于图像中心 |
| K | `(3,3)` intrinsic matrix，作用于 camera 坐标的投影；不是 SE(3) |
| u、v | `(2,)`，连续像素坐标，u 向右、v 向下；不是整数数组索引 |
| width、height | 本例 640、480；图像数组通常按 `[row=v,column=u]` 索引 |

物理 focal length f（m）与像素 pitch sx、sy（m/pixel）的关系是 fx=f/sx、fy=f/sy。
本课人为设置 K，不声称是真实相机标定参数。采用无畸变、零 skew 的 ideal pinhole。
像素原点在左上，本实验使用半开区域 `0<=u<640, 0<=v<480`；主点指定为 (320,240)。
以后采样真实 raster 时需明确 pixel-center、取整/插值约定。

## Mathematics：meaning / shape / unit / frame

```text
K = [fx  0 cx]    p_C = [X]    Z [u] = K p_C
    [ 0 fy cy]          [Y]      [v]
    [ 0  0  1]          [Z]      [1]

u = fx * X/Z + cx
v = fy * Y/Z + cy
```

单位：X/Z 无量纲，fx·X/Z 与 cx 都是 pixel。`K p_C` 是齐次投影量，必须除以第三项 Z。
这不是 S12.1 的 w=1 刚体变换：从 3D 到 2D 的除法会丢失深度，不能当作可逆坐标变换。

同一射线：`p_C` 和 `α p_C`（α>0）拥有相同 X/Z、Y/Z，因此像素相同。
单个像素不能单独确定一个 3D 点。后续需深度或已知物体几何等额外信息。

本课完整链：`p_C=R_WCᵀ(p_W−t_WC)`，再投影。外参将 world→camera，内参将 camera→pixel。
不能把 world xyz 直接乘 K，或把 K 当 camera→base 的 transform。

## Math-to-Code / APIs

核心函数 `project_points(points_camera,K,width,height,near_depth)`：

- 输入 `(N,3)` camera 点（m）和 `(3,3)` K；函数不修改输入。
- 返回 `(N,2)` pixels、`(N,)` projectable mask、`(N,)` in-image mask。
- 先检查点有限且 `Z>0.05 m`，仅对通过的点做 `K@p` 与除法。
- 无效点像素保留 NaN；in-image 再检查图像边界。

`0.05 m` 是此实验数值/近距离门限，不是真实相机 near clip 参数。
projectable 与 in-image 不包含遮挡、曝光、检测器能力或物体正面检查，因此不能简称真实可见性。

`np.full((N,2),np.nan)` 创建输出占位；布尔索引选择有效点；
`(K @ points[mask].T).T` 返回 `(M,3)` 齐次量；`[:,2,None]` shape `(M,1)` 通过 broadcasting
逐行除深度。`np.savetxt` 写小 CSV；Matplotlib Agg 保存静态 PNG，无 GUI。
本课只调用 NumPy/Matplotlib，不引入 MuJoCo 新 API、OpenCV 或新依赖。

## Minimal Experiment → Expected / Actual Result

在仓库根目录，核验 mujoco 环境与 interpreter 后运行：

```bash
python examples/13_perception_geometry/pinhole_projection.py
python examples/13_perception_geometry/pinhole_projection.py --focal-scale 2
```

默认 `K=[[500,0,320],[0,520,240],[0,0,1]]`。Modify 仅把 fx/fy 乘 2，
保持主点、图像尺寸、物体和相机不动，模拟 ideal camera 焦距改变。
不是图像 resize；resize 通常还需要一起变更主点、尺寸和采样约定。

先手算 `[0.10,0,0.50]_C m`：u=500·0.10/0.50+320=420，v=240。
焦距乘 2 后 u=520，改变的是相对主点的 100→200 pixel，不是整个 u 乘 2。

2026-10-04 助手在 WSL2 Ubuntu 24.04.5、mujoco 环境、Python 3.12.14、
NumPy 2.5.3 / Matplotlib 3.11.2 实测，两命令退出 0，无 import error。
代码兼容目标 3.11，未在 3.11 执行。

| 点 p_C（m） | 默认 (u,v) pixel | focal-scale=2 | 核对意义 |
| --- | --- | --- | --- |
| [0,0,1] | (320,240) | (320,240) | 光轴投到主点 |
| [0.10,0,0.50] | (420,240) | (520,240) | 近点偏移更大 |
| [0.10,0,1] | (370,240) | (420,240) | Z 加倍，主点偏移减半 |
| [0,0.10,0.50] | (320,344) | (320,448) | +Y 对应 v 增大、向下 |
| [0.20,0,1] | (420,240) | (520,240) | 同射线、不同深度，同像素 |
| S12.1 object center [-0.10,-0.10,0.77] | (255.064935,172.467532) | (190.129870,104.935065) | 衔接原世界物体 |
| [0.64,0,1] | (640,240) | (960,240) | 正深度，可投影但不在图像内 |

11 个点中，两组都是 projectable=7、in-image=6。
behind、Z=0、Z=0.05、nonfinite 共四点被拒绝，输出 NaN。
未做 depth gate 的 behind 点 `[0.10,0,-1]` 会得到貌似在图像内的 `(270,240)`；故只看像素范围不够。
脚本核验独立手算、depth scaling、same-ray ambiguity、焦距偏移缩放与边界。
CLI 拒绝 `--focal-scale 0` / `nan`（退出 2）；CSV 核对 shape、mask 与两组 offset scaling 通过。

输出文件为已忽略的 `tmp/s12_2_pinhole_f1/` 与 `tmp/s12_2_pinhole_f2/`，
各含 projection.csv 和 projection.png；也可指定 `--output-dir`。
已目视检查 focal-scale=2 对比图：统一轴范围、v 向下、重合射线标记、主点与图像边界正确。
这是几何散点图，不是相机拍摄/渲染图像；没有 calibration 或 runtime perception 证据。

## Explanation → Failure Cases

| 失败 | 原因与处理 |
| --- | --- |
| Z<=0 仍得到 finite uv | 代数除法允许负 Z，camera 前方 gate 必须先做 |
| Z 很小导致爆大 pixel | X/Z、Y/Z 放大；设置实验近距离门限，并判断边界 |
| 正深度但落在图像外 | depth 和 image bounds 是不同条件 |
| optical y 与屏幕 y 混用 | 本课 +Y_C 向下；Matplotlib 反转 y 轴表现 v 向下 |
| 把 u 当 row | 图像访问顺序通常 `[v,u]`；本课不做 raster sampling |
| fx 单位当米或把 K 当外参 | fx/fy 是 pixel；先用 extrinsic 得到 p_C |
| mm/m 混用 | 全部 xyz 同比缩放可保持像素，看似正确却破坏 metric geometry；不能靠投影验证单位 |
| 理想投影正确而真实像素不吻合 | 后续需要畸变、标定、渲染轴与 pixel-center 约定核验 |
| 已在 image bounds 就声称检测成功 | 仍可能被其他物体遮挡；本课不测遮挡或视觉估计 |

## Robotics Context

PnP 将已知物体 3D 特征与图像 2D 特征配对，并在已知 K 下求外参/物体 camera pose；
RGB-D 给出 Z 后才能把 pixel 变回具体 3D 点。这里只建立 forward projection，
不提前实现 PnP、RGB-D、校准或操作。

## Interview Capsule

**30 秒**：ideal pinhole 用 optical camera xyz 和 K 投影到像素。
相似三角形给出 X/Z、Y/Z，再由像素焦距缩放和主点偏移得到 u/v。
需要先确认 Z 为正并足够大，然后检查 image bounds；这两个检查都不证明无遮挡。
同射线的不同深度点产生同像素，所以单个像素不包含完整 metric 3D 信息。

**2 分钟**：先指定 optical x右/y下/z前、xyz 用米、uv 用连续 pixel。
给出 K 的 fx/fy/cx/cy 与 `Z[u,v,1]ᵀ=K[X,Y,Z]ᵀ`，用相似三角形解释除深度。
说明先用 extrinsic 将 world 点变到 camera，再用 intrinsic 投影，两者职责不同。
用近远点展示 Z 加倍使主点偏移减半；焦距加倍使偏移加倍，主点仍不变。
再用同射线不同深度展示投影的多对一性质。
最后说明 negative Z 也可能产生 finite 且在图像内的 uv，必须先 gate depth；
真实感知还涉及畸变、遮挡与标定，理想几何实验只验证投影关系。

## Must Remember

- 外参换 3D frame；内参与 perspective division 将 camera 3D 投影到 pixel。
- fx/fy 为 pixel，Z 为光轴深度；放大的是相对主点的偏移。
- projection 丢失深度；projectable/in-image 不代表真实检测或抓取成功。

## My Verification / Run → Modify → Explain

状态只记在根 README；助手验证不计作本人完成。

本人验证记录（2026-10-04）：明确确认实验与预测均完成，并正确用相似三角形解释
透视除法，区分光轴深度 Z 与距离，说明 K 与外参的职责、焦距变化只缩放主点偏移，
以及同射线不同深度的 pixel 歧义。本人进一步指出投影仅计算几何映射，不比较同射线上
更近的 surface，因此不能判定遮挡。Run/Modify/Explain 已全部确认，Learning Mastered。
补充单位精确表述：fx/fy 是以 pixel 表示的焦距，cx/cy 是主点 pixel 坐标。
本次仅记录本人反馈，未重跑实验或新增 runtime 证据；不自动开始 S12.3。

**Run**：执行默认命令，读取 CSV/PNG，确认 near_x=(420,240)、far_x=(370,240)，
projectable=7/11、in-image=6/11。

**Modify**：先预测 focal-scale=2 的 near_x/down_y，再运行第二条命令；
解释为何主点不变而相对主点的偏移加倍。

**Explain**：

1. 从相似三角形解释为何投影除以 Z；Z 是什么方向的深度？
2. K 的 fx/fy/cx/cy 的意义和单位分别是什么？与外参有何区别？
3. fx 加倍时为什么 u 不直接加倍？光轴点为何不动？
4. 为什么不同深度的两点可投到同一 pixel？单个 pixel 能否恢复 metric 3D？
5. 为什么先检查 depth，再检查 image bounds？通过两项是否说明没有遮挡？

Engineering 完成后 STOP；不自动开始 S12.3。
