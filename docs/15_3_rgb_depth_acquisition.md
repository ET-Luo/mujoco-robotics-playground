# S12.3 — MuJoCo CPU RGB / Depth Acquisition

约 1～2h。[代码](../examples/13_perception_geometry/rgb_depth_capture.py) ·
[小场景](../examples/13_perception_geometry/camera_scene.xml) ·
[状态唯一来源](../README.md#stage-12--robot-perception-geometry) ·
[前课投影](15_2_pinhole_projection.md)。

## Problem → Why

S12.2 用已知 3D 点算出 pixel；现在需要真正从 MuJoCo scene 得到 RGB 和 depth 数组，
并确认数组的方向、尺寸、单位与投影约定相容。否则后续标定/PnP/RGB-D 即使公式正确，
仍可能被 camera-axis、上下翻转或 depth 单位错误破坏。
本课只做静态 acquisition，不接 UR5e 操作、标定、pose estimation 或 point cloud。

## Intuition → Core Concepts

Renderer 将几何表面投影成图像，depth test 保留遮挡关系下的近处可见表面。
RGB 描述该表面的渲染颜色；depth 描述沿 camera optical +Z 的表面距离。
方块中心在表面后面：本例 center world z=0.03 m，顶面 z=0.06 m；
camera world z=0.80 m 时，center Z=0.77 m，但看到的顶面 depth=0.74 m。

| 量 | shape / dtype / unit / convention |
| --- | --- |
| RGB | `(H,W,3)` / uint8 / channel 0–255；RGB 顺序，row 从上到下 |
| depth | `(H,W)` / float32 / m；visible-surface axial Z，非归一化灰度、非径向 range |
| M renderer camera | +X right、+Y up、视线沿 −Z |
| C optical camera | +X right、+Y down、视线沿 +Z；与 M 同原点 |
| `data.cam_xpos` | `(ncam,3)` / m，camera origin 在 W 中的位置 |
| `data.cam_xmat` | `(ncam,9)`，reshape 为 `(3,3)`，R_WM 的列是 M 各轴在 W 的方向 |

MuJoCo 相机的 −Z forward、+Y up 约定见[官方 Visualization 文档](https://mujoco.readthedocs.io/en/stable/programming/visualization.html#cameras)。
当前安装的 MuJoCo 3.14.0 `Renderer.render` 实现已现场检查：depth buffer 被转换为
metric depth，classic EGL 输出已做垂直翻转；不要再把返回图像 flipud 或再次线性化 depth。
Python API 入口见[官方文档](https://mujoco.readthedocs.io/en/stable/python.html#rendering)。

## Mathematics：meaning / shape / unit / frame

```text
R_MC = diag(1,-1,-1)       # optical C axes expressed in renderer M
R_WC = R_WM @ R_MC
p_M  = R_WM.T @ (p_W - t_WM)
p_C  = R_WC.T @ (p_W - t_WM)
     = R_MC.T @ p_M
```

这不是镜像：两个轴反向，det=+1，是绕 camera x 的 π 旋转。
本场景 camera quat 为 identity，因此 R_WM=I，R_WC=diag(1,-1,-1)，
与 S12.1 的 overhead optical camera 完全一致；不代表任意相机都可以用 world identity。

本课 perspective camera 使用 vertical fovy=45°、square pixels、ipd=0：

```text
fx = fy = (H/2) / tan(fovy/2)
cx = (W-1)/2, cy = (H-1)/2        # integer indices identify pixel centers
u = fx*X_C/Z_C + cx
v = fy*Y_C/Z_C + cy
near = model.vis.map.znear * model.stat.extent
far  = model.vis.map.zfar  * model.stat.extent
```

fovy 先由 degree 转 rad；focal 与主点为 pixel，depth/near/far 为 m。
本例 extent=1、znear=0.01、zfar=3，实际 near/far=0.01/3 m。
当前不处理 orthographic 或自定义 physical-camera intrinsics。

S12.2 的 principal point=(320,240) 是人工给定的连续坐标参数。
这里 640×480 的 symmetric renderer 在整数 pixel-center 索引约定下为 (319.5,239.5)。
边界坐标转中心索引相差 0.5 pixel；不能混用 K 与 raster sampling 约定。

depth 是 Z 而非 range。对一个 pixel，令 ray direction 为
`[(u-cx)/fx,(v-cy)/fy,1]`，径向距离为 `Z*norm(ray)`；离光轴越远越大。
本课只用一个 floor pixel 区分两种距离，不提前生成 point cloud。

## Math-to-Code / API Inputs and Effects

代码的 `capture` 只有 scene update 和两次 render；同一 MjData、没有 mj_step，
所以 RGB/depth 对应同一静态 state，simulation time 始终 0。

| API | 输入 → 返回 / 更新 | 用途 |
| --- | --- | --- |
| `MjModel.from_xml_path(path)` | XML path → compiled MjModel | 加载小场景，无 Menagerie 下载 |
| `MjData(model)` | model → 新状态/cache | 存放 camera/geom world poses |
| `mj_forward(model,data)` | 返回 None；原地刷新派生量，不推进 time | cam_pos 改变后刷新世界位姿 |
| `Renderer(model,height=H,width=W)` | model 与 pixel 尺寸 → render context | 分配 offscreen framebuffer；尺寸不能超过 XML offwidth/offheight |
| `renderer.update_scene(data,camera='overhead')` | state 和 camera name → 原地刷新 render scene | 选择 fixed model camera，非 viewer free camera |
| `disable_depth_rendering()` | 无输入，切换 render mode | 下次 render 返回 RGB |
| `enable_depth_rendering()` | 无输入，切换 render mode | 下次 render 返回 metric depth |
| `render()` | 无 out 参数时返回新数组 | RGB `(H,W,3)` uint8，depth `(H,W)` float32 |
| Renderer context manager | `with` 离开时 close native resources | 避免 GL context 泄漏；不打开 GUI |
| `GL.glGetString(GL_RENDERER)` | 常量 → renderer name bytes | 核验 CPU 软件后端；PyOpenGL 是既有 MuJoCo 依赖 |

RGB PNG 供观察；`rgb.npy` 保存原始三通道 uint8，`depth_m.npy` 保存原始 metric float32。
Matplotlib 展示用 colormap/colorbar；彩色 depth PNG 不可作为 metric depth 数据输入。
metadata.json 记录 K、axes、camera origin、near/far、backend、surface checks 和 timings。

## CPU Backend and Minimal Experiment

脚本在 import MuJoCo/PyOpenGL **之前**设置：

```text
MUJOCO_GL=egl
PYOPENGL_PLATFORM=egl
EGL_PLATFORM=surfaceless
LIBGL_ALWAYS_SOFTWARE=1
```

这是本例进程的明确软件渲染配置，不修改 shell 全局环境。EGL 本身不等于 CPU；
脚本再检查 GL_RENDERER 为 llvmpipe/softpipe/software rasterizer，否则报错，不默默使用 GPU。
使用现有 Mesa/EGL；无需 NVIDIA、GUI 或新 Python/system dependency。

从仓库根目录：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/rgb_depth_capture.py
python examples/13_perception_geometry/rgb_depth_capture.py --camera-height 1.0
```

固定 world camera xy、rotation、fovy 和图像尺寸，仅提高 camera world z。
先预测：K 不变，floor depth 0.8→1.0 m、box top 0.74→0.94 m；
物体相对主点的偏移及面积变小。不是改 focal length，也不是移动物体。

## Expected / Actual Result → Explanation

2026-10-04 助手现场核验 WSL2 Ubuntu 24.04.5、mujoco environment、
`/home/lucas/miniconda3/envs/mujoco/bin/python`，Python 3.12.14 / MuJoCo 3.14.0。
代码兼容目标 Python 3.11，未在 3.11 执行。本机 GL_RENDERER 明确为
`llvmpipe (LLVM 20.1.2, 256 bits)`；无新增/升级依赖。

| 配置 | 默认 | Modify | 尺寸核验 |
| --- | --- | --- | --- |
| camera z m | 0.8 | 1.0 | 0.8 |
| W×H | 320×240 | 320×240 | 640×480 |
| fx=fy pixel | 289.705627 | 289.705627 | 579.411255 |
| box top depth m | 0.740000 | 0.940000 | 0.740000 |
| floor depth m | 0.800000 | 1.000000 | 0.800000 |
| box top center pixel | (120.350591,80.350591) | (128.680252,88.680252) | (241.201182,161.201182) |
| box top pixel count | 768 | 450 | 2961 |
| mean static RGB/depth pair ms | 3.504 | 3.729 | 7.056 |
| median pair ms | 3.397 | 3.977 | 6.712 |
| renderer initialization ms | 120.660 | 51.949 | 63.999 |

默认/Modify 两条命令及
`python examples/13_perception_geometry/rgb_depth_capture.py --width 640 --height 480`
均退出 0，无 import error。timings 为一次 warmup 后的 5 对样本，包含 scene update、RGB render、
depth render/readback，不含保存；不能视作机器人 pipeline FPS 或稳定 performance benchmark。
初次 context creation 更慢是观察，不由少量样本推断唯一根因。

RGB/depth shape/dtype、finite positive depth、same-origin frame 转换、三个彩色表面的 predicted
pixel 与 patch 颜色/深度均通过。depth patch absolute tolerance=1e-4 m；
box silhouette pixel bounds 与投影 top corners 相差 <=1.5 pixel，验证 axis/row convention。
默认 off-axis floor depth=0.800000 m，但 radial range=0.939411 m，排除将 depth 当 range。
已目视检查默认 RGB/depth PNG：红标在右上、绿标在左下，预测十字落在各顶面。

输出在已忽略 `tmp/s12_3_rgbd_z0.8_320x240/`、`z1_320x240/`、`z0.8_640x480/`。
全部为静态场景，无 GUI、physics stepping、UR5e 动作、calibration、PnP 或 learner mastery 证据。
彩色 patch 与 depth threshold 使用已知场景 truth 进行验证，不是 object detector/pose estimator。

## Failure Cases

| 情况 | 诊断与边界 |
| --- | --- |
| CPU OSMesa 首次 probe 失败 | `AttributeError: 'NoneType' object has no attribute 'glGetError'`；ldconfig 无 libOSMesa，证据指向缺失系统 backend library；非 MuJoCo geometry 错误 |
| EGL 替代 probe | 现有 EGL + software flag 成功，llvmpipe 已核验；未安装 OSMesa，失败未静默忽略 |
| EGL/GL initialization 失败 | 保留 traceback，检查 libEGL/Mesa 与 import 前的 backend selection；headless EGL 无需 WSLg window，不优先归咎 DISPLAY |
| 图像上下颠倒 | 确认 M→C 两轴转换；当前 Renderer 已 flip 输出，不能再 flip |
| 顶面 depth 与 center Z 不一致 | 正常：看到的是顶面而非隐藏中心；本例差 0.03 m |
| 背景没命中几何 | 可能返回 far-plane depth，不能简单认为 finite depth 就是有效表面；本例 infinite floor 覆盖图像，所以所有点都命中 |
| near/far clipping | 超范围表面不被正常采样；本例明确设置并检查 clipping distances |
| 把 depth PNG 当 metric | colormap 是展示，数值使用 depth_m.npy；绝不能再除 255 |
| 改分辨率却复用旧 K | f 与主点都依赖实际输出尺寸；当前 square-pixel fovy 配置需重新计算 |
| 忘记 update_scene/forward | 渲染或 pose cache 可能旧；修改 camera position 后 mj_forward，再 scene update |

## Robotics Context

后续 calibration/PnP 消费 RGB pixel；RGB-D 消费与 RGB 对齐的 metric depth 与 K。
本课建立 acquisition 和 frame 接口，后续才引入估计器；synthetic depth 没有现实深度噪声、
缺测、滚动快门或 sensor RGB/depth 外参失配，不自动证明真实相机效果。

## Interview Capsule

**30 秒**：MuJoCo rendering camera 使用 −Z forward、+Y up，转 optical frame 要翻转 y/z 两轴。
Renderer 用同一 state 获取 RGB uint8 和 metric axial depth float32，返回图像已上到下排列。
用已知 floor/box top 核验 depth，不能把 object-center Z 当可见表面 depth。
本例 EGL + llvmpipe 软件渲染适用于无 NVIDIA 的 CPU 实验。

**2 分钟**：先说明 frame、m/pixel 与 `(H,W,C)` 数组约定，给出 R_MC 与 R_WC 的乘法。
解释 K 从 fovy 和输出尺寸推导，并明确 pixel-center half-pixel convention。
列出 mj_forward→update_scene→RGB/depth mode→render 的直接 API 流程，强调不推进 simulation time。
说明 Renderer metric depth 与 raw non-linear depth buffer 不同，不重复解码或翻图。
用 world floor z=0、camera z=0.8 推出 depth=0.8；box top z=0.06 推出 0.74，
再在离轴 pixel 比较 axial depth 与 radial range，避免语义混淆。
最后报告 backend name 和静态 pair latency；这些只验证 acquisition，不证明 pose estimation 或机器人执行。

## Must Remember

- M→C 翻转 y/z 两轴；不要把 rendering camera 当 optical frame。
- RGB 是 uint8 三通道，depth 是 float32 米；Renderer 返回 depth 已转换且图像已翻转。
- depth 对应可见表面，不等于物体中心或径向 range。
- 提高相机改变 extrinsic；相同 fovy/尺寸下 K 不变。

## My Verification / Run → Modify → Explain

状态只在根 README；助手运行不计本人完成。

本人验证（2026-10-04）：明确确认实验与对比分析均完成，正确解释反转 y/z 保持 det=+1，
是绕 X 轴 π 的旋转而非镜像；depth 为 `(H,W)` float32、单位 m、含义为 optical-axis Z；
相机看到表面而非几何中心；相机高度改变外参，resolution/fovy/model 不变故 K 不变。
本人也正确区分 mj_forward 的派生量刷新、update_scene 的场景构造与 render 的渲染，
并指出本课未调用 mj_step，因此 time 不推进。Run/Modify/Explain 全部确认，Learning Mastered。
补充解释：相机升高使 Z 增大，X/Z、Y/Z 减小，所以物体缩小并向主点靠近。
本次仅记录本人反馈，不新增 runtime 证据；不自动开始 S12.4。

**Run**：默认命令，查看 RGB/depth PNG 与 metadata，确认 backend=llvmpipe、floor=0.8 m、box top=0.74 m。
**Modify**：先预测 camera-height=1.0 的 depth、K 与图像变化，再运行；核对 floor=1.0、top=0.94、K 不变。
**Explain**：

1. 为什么 M→C 是 diag(1,-1,-1)，而不是只反转 z？
2. depth 的 shape/dtype/unit 是什么？为什么它不是到 camera origin 的径向距离？
3. 为什么 box center 的 Z=0.77，而该 pixel depth=0.74？
4. 提高相机后 K 为什么不变，物体为何缩小并向主点靠近？
5. mj_forward、update_scene、render 分别更新什么？为什么本课 time 仍是 0？

Engineering 完成后 STOP，不自动开始 S12.4。
