# S12.7a — Known-Correspondence Rigid Alignment

约 1～2h。[代码](../examples/13_perception_geometry/rigid_alignment.py) ·
[前课 RGB-D](15_6_rgbd_back_projection.md) · [状态唯一来源](../README.md#stage-12--robot-perception-geometry)。

## Problem → Why

给定同一批 landmark 在 source S 与 target T 中的米制 3D 坐标，且第 i 行明确对应，
求 source→target 的旋转 R_TS 与平移 t_TS。点云配准需要这个核心子问题：
对应关系固定后，怎样找到整体最匹配的刚体运动？本课手写 NumPy centroid/SVD，
没有 nearest-neighbor、ICP iteration、Open3D、MuJoCo execution 或新依赖。
S12.6 点云不直接作为本课输入：这里用小型、有身份的 synthetic landmarks 隔离算法。

## Intuition → Core Concepts

先让两组点的质心重合，便只剩“相对质心的形状如何旋转”。
SVD 将两组形状之间的相关方向分解；选择使整体对应距离最小的旋转，
再移动 source 质心到 target 质心。刚体变换保持距离与手性，不允许尺度变化或镜像。

| 量 | shape / unit / frame |
| --- | --- |
| source / target | `(N,3)`；m；每行是 S / T 中的点，i→i 已知 |
| mean_S / mean_T | `(3,)`；m；两组质心 |
| X / Y | `(N,3)`；m；去质心后的点 |
| H=X.T@Y | `(3,3)`；m²；未除 N 的 cross-covariance sum |
| U / V / R_TS | `(3,3)`；无量纲；R_TS 将 S 方向转到 T |
| singular values | `(3,)`；m²；H 的奇异值，按降序 |
| t_TS | `(3,)`；m；source 原点在 target 中的位置 |
| T_TS | `(4,4)`；齐次变换，S→T |

## Mathematics：objective → centroid → SVD

对列向量点，优化：

```text
min Σ_i ||R p_Si + t - p_Ti||²
subject to R.T R=I, det(R)=+1
```

对 t 求导并令其为 0，得 `t=mean_T-R mean_S`。
令 x_i=p_Si-mean_S、y_i=p_Ti-mean_T，代入后目标为：

```text
Σ ||R x_i-y_i||²
= Σ ||x_i||² + Σ ||y_i||² - 2 Σ y_i.T R x_i
```

前两项与 R 无关，因此最小误差等价于最大化 `trace(R H)`，其中
`H=Σ x_i y_i.T=X.T@Y`。做 SVD：

```text
H = U diag(s1,s2,s3) V.T       # s1 >= s2 >= s3 >= 0
Q = V.T R U
trace(R H) = trace(Q diag(s))
d = sign(det(V U.T))
D = diag(1,1,d)
R = V D U.T
t = mean_T - R mean_S
```

无 det 约束时 Q=I 达到最大 trace，对应 raw R=V U.T；它可能 det=-1。
若需要 det=+1，就在最小奇异值方向使用 −1，牺牲最少的相关性。
full-rank mirror 对照中 corrected SSE 比 raw SSE 多 `4*s3`，
代码核验 `N * corrected_RMS² = 4*s3`（raw SSE≈0）。
SVD 中 U/V 的列是 H 的左右奇异方向，不是机器人坐标轴或 joint axes。

`R.T R=I` 只保证正交；det=-1 仍正交，所以必须单独检查 det=+1。
非共面点的镜像不能由合法旋转精确拟合。本课也允许平面内三个非共线对应点：
它们已经给出两条独立方向，右手旋转约束确定第三方向；s3=0 不意味着不可解。
共线点只给出一条方向，绕该线的旋转不可观测，因此拒绝；重合点更不足。

## Math-to-Code / API

`fit_rigid(source,target)` 返回新 R、t、H 奇异值与 raw determinant，不修改输入，
没有 initial guess，也不访问 truth。truth 只在合成输入和评分中使用。

```python
H = X.T @ Y
U, singular, Vt = np.linalg.svd(H)
R = Vt.T @ D @ U.T
aligned = source @ R.T + t
```

`np.linalg.svd(H)` 输入有限 `(3,3)` 数组，返回 U、singular、Vt，重构为
`U @ diag(singular) @ Vt`；注意返回 V.T 而不是 V。
`np.linalg.svd(centered,compute_uv=False)` 只返回点集 spread 的奇异值，用于几何退化 guard；
它与 H 的奇异值形状、单位不同：centered 的 singular values 为 m。
`np.linalg.det(R)` 返回 scalar determinant；`np.mean(...,axis=0)` 返回质心。
行数组运算要 `source @ R.T + t`，而列向量公式是 `R @ p + t`。

代码拒绝少于三点、shape mismatch、nonfinite、共线与重合输入；
second spread singular <=1e-10*first 时拒绝，是数值门限，不能证明近共线观测的实际精度。
最小二乘不自动识别 outlier 或错误对应；输入合法不代表数据满足 rigid model。

## Minimal Experiment / Run → Modify

从仓库根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/rigid_alignment.py
python examples/13_perception_geometry/rigid_alignment.py --noise-m 0.004
python examples/13_perception_geometry/rigid_alignment.py --noise-m 0
```

8 个非对称非共面 landmark，truth `Rz(35°)Rx(-20°)`、t=[0.30,-0.15,0.55] m。
默认仅在 target 每轴添加 Gaussian σ=0.001 m，seed=20261005。
输出到 ignored `tmp/s12_7a_rigid_noise*_seed20261005/`，含 summary.json、
完整 alignment.npz、correspondences.csv、alignment.png。PNG 左图显示 target frame 中
拟合与观测，右图是逐对应 Euclidean residual，单位 mm；不是 RGB photograph。

**Run**：运行默认实验，核对 det=+1、残差和 truth errors，并查看 PNG 与 mirror 对照。
**Modify**：先预测噪声 σ=1→4 mm 的变化，再运行第二条命令。
固定 geometry、truth、seed 与 unit noise，仅缩放噪声幅度。比较 noisy RMS、clean RMS、
t error 和 rotation error；本次约增至四倍，不保证任意噪声样本或大扰动都线性/单调。
零噪声命令用于核对可精确恢复。

## Expected / Actual Result → Explanation

2026-10-05 助手现场核验 WSL2 kernel 6.6.87.2、Ubuntu 24.04.5；
自动激活 mujoco，同执行 shell 核验 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14、NumPy 2.5.3；代码目标为 3.11 兼容，未在 3.11 执行。
上面三条脚本均退出 0，无 import error，无新增依赖。

| 指标 | σ=0.001 m | σ=0.004 m | σ=0 |
| --- | --- | --- | --- |
| noisy correspondence RMS m | 0.001501730 | 0.006001646 | 1.23e-16 |
| clean alignment RMS m | 0.000840306 | 0.003380230 | 1.23e-16 |
| translation error m | 0.000699880 | 0.002794669 | 1.24e-16 |
| rotation error degree | 0.552240 | 2.260010 | 0 |

RMS 是 `sqrt(mean(||aligned_i-target_i||²))`，不是单坐标 RMS。
noisy RMS 是训练对应误差；clean RMS 和 truth pose error 衡量已知无噪声几何的恢复，
只能在本合成实验直接评分，真实扫描通常没有 truth。
Gaussian σ 为每轴标准差，不是 Euclidean 点噪声 RMS。

各运行内置：zero-noise truth recovery、90° rotation + [2,3,4] m 平面三点手算、
R 正交/det、inverse roundtrip；mirror raw det=-1、raw RMS=6.13e-17 m，
corrected det=+1、RMS=0.045317910 m。错对应循环错位 RMS=0.049574911 m。
五类输入 guard 拒绝；脚本 CLI 另拒绝非有限/负/超范围 noise 与 negative seed。
独立产物检查与默认 PNG 观察结果见工程交接；runtime 不计本人 Run。
没有性能 benchmark、GUI、实际 RGB-D、nearest-neighbor/ICP 或机器人执行验证。

## Failure Cases

| 错误 | 物理含义 / 处理 |
| --- | --- |
| 忘 reflection correction | 可能得到镜像且 residual 很小；检查 det=+1 |
| H 写为 Y.T@X 仍套原公式 | 变换方向颠倒；H 定义和 R 公式必须配套 |
| t=mean_T-mean_S | 忘先旋转 source centroid；正确为 mean_T-R mean_S |
| 用 source@R | 行/列 convention 混淆；应 source@R.T |
| 对应错序/outlier | SVD 仍返回合法旋转但误差大；本课无 robust estimation |
| 共线/重合/近共线 | 缺方向约束或敏感；数值 guard 之外还需看 geometry spread |
| source mm / target m | rigid fit 无 scale 参数，无法补偿；统一米制 |
| 把 rank-2 全部拒绝 | 错杀有效平面三点；非共线即有两条独立方向 |
| small fit RMS 当绝对 pose accuracy | 系统误差或 wrong model 可隐藏；需要独立几何/truth 检验 |

## Robotics Context

这个 SVD fit 是 point-to-point ICP 中“对应关系固定后”的刚体更新子问题。
S12.7b 才引入 nearest-neighbor、迭代组合、initial pose 与 partial overlap。
本课用 source/target frame 名称避免自动将 transform 当 camera→base；
实际使用时必须按数据 frame 命名，已知对应配准不是自动 hand-eye calibration。

## Interview Capsule

**30 秒**：给定 3D 对应，最小化 R/t 的平方点距离，先去质心，再对 X.T@Y 做 SVD，
R=V diag(1,1,det(VU.T)) U.T，t=mean_T-R mean_S。修正 det 避免镜像，
最小奇异方向翻转损失最小。共线不可观测，平面非共线可以；ICP 还需要寻找/更新对应。

**2 分钟**：从 objective 对 t 求导得到 centroid relation；展开中心化目标，把误差最小化
化成 trace(RH) 最大化。说明 H、U/s/Vt 的形状/单位、S→T 方向和 row-vector 转置。
指出正交群包含 reflection，SO(3) 必须 det=+1，在最小 singular direction 修正。
解释几何退化：两条非平行方向足够、共线保留绕线自由度；奇异值不能简单按 full rank 断言。
报告 noise/truth/residual 三者的区别，指出错对应仍有合法解、SVD 无 robust outlier 防护，
最后明确这里是一次 fixed-correspondence solve，尚无 ICP loop。

## Must Remember / My Verification

- 已知对应、统一单位、明确 source→target 是前提。
- 去质心求 R；恢复 t 时必须旋转 source centroid。
- Vt 是 V.T；合法旋转需 orthogonal 且 det=+1。
- planar noncollinear 可解，collinear 不可唯一确定；fit residual 不是绝对 pose accuracy。

状态仅维护在根 README。本人于 2026-10-05 明确确认实验与预测均完成，并回答五项 Explain；
Run/Modify/Explain 全部确认，Learning Mastered。
本人解释覆盖中心化消去平移、旋转 source 质心后恢复 t、Vt.T 的使用、
reflection 与 proper rotation 约束、平面非共线和共线的可观测性，以及配准残差与
truth pose error 的区别；指出完整 ICP 需要不断重新寻找未知对应关系。

补充：翻转第 j 个奇异方向使 trace 目标减少 2*s_j、平方误差增加 4*s_j，
因此选择最小奇异值方向损失最小。这里估计的是一般 source→target 刚体位姿；
只有点集 frame 确实对应相机或机器人时，才能赋予它相应的物理名称。
本次仅同步本人反馈；runtime 证据沿用 2026-10-05 工程验证，未重跑实验或 GUI。

**Explain**：

1. 为什么先减质心？为什么 t 不是两个质心直接相减？
2. H=X.T@Y 时，R 为什么是 V D U.T？NumPy 的 Vt 应怎样使用？
3. det=-1 表示什么？为什么翻转最小 singular direction，修正后残差可能增加？
4. 为什么非共线平面三点可解，而共线点不行？
5. noisy correspondence RMS 与 truth pose error 有何不同？本课和完整 ICP 还差哪一步？

Engineering 完成后 STOP；不自动开始 S12.7b。
