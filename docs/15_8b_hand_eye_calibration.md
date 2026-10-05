# S12.8b — Hand-Eye Calibration / Estimate X

约 1～2h。[代码](../examples/13_perception_geometry/hand_eye_calibration.py) ·
[前课几何](15_8a_hand_eye_geometry.md) · [状态唯一来源](../README.md#stage-12--robot-perception-geometry)。

## Problem → Why

前课验证了 truth X 满足 AX=XB，但没有从观测估计 X。
现在只给成对的 robot pose G_i=T_BG(i) 与 camera board pose H_i=T_CO(i)，
求固定安装外参 X，并检查它在没参与标定的姿态上是否仍能闭环。
使用 NumPy 分步线性估计，主要概念是**从相对约束得到外参，并独立验证**。
不重新标定 K/d、不运行 PnP 或 ICP，也不把合成 truth 输入求解器。

## Intuition → Core Concepts

旋转先决定两套运动方向如何对应；有了 R_X，平移方程成为普通线性最小二乘。
训练观测产生一个 X_est，再从训练闭环估一个固定 Y_mean。
留出姿态必须仍得到相同 Y_mean：不能在留出数据上重新平均 Y 来掩盖偏移。
近同轴训练可能拟合得很好，却把未被充分激励的方向估错，只有更广方向的留出运动才显露。

| 量 | shape / unit / frame |
| --- | --- |
| G/H | `(N,4,4)`；rotation 无量纲、translation m；hand→base / board→camera |
| X（eye-in-hand） | `(4,4)`，T_GC，camera→hand；Y=T_BO，固定板在 base |
| X（eye-to-hand） | `(4,4)`，T_BC，camera→base；Y=T_GO，板固定在 hand |
| pairs / A/B | `(P,2)` / `(P,4,4)`；P=N(N−1)/2，方向同前课 |
| K / L | `(9P,9)` / `(3P,3)`；无量纲；rotation-only / translation coefficients |
| vec_F(R) / raw | `(9,)` / `(3,3)`；按列 reshape；raw 尚非合法 rotation |
| rhs / t_X | `(3P,)` / `(3,)`；m；平移方程 RHS / X translation |
| rotation errors | scalar or `(N,)`；degree；通过相对 rotation trace 计算 |
| translation errors | scalar or `(N,)`；m；同 frame 两个 translation 的 Euclidean 距离 |

两种配置及 G/H/X 的方向可核对
[OpenCV 官方 hand-eye 文档](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)。
本课实现与求解公式是直接从前课分块方程推导；不调用 OpenCV 标定 API。

## Mathematics：Rotation → Translation

前课已经推导：

```text
eye-in-hand: A=inv(G_i)G_j, B=H_i inv(H_j), X=T_GC
eye-to-hand: A=G_j inv(G_i), B=H_j inv(H_i), X=T_BC
R_A R_X = R_X R_B
K = stack(I ⊗ R_A - R_B.T ⊗ I)
```

无噪声充分激励时 K 的 nullity=1。有噪声时一般不再存在精确零空间。
取 SVD `K=U diag(s) V.T` 最后一行 V.T，解单位向量约束问题：

```text
v = argmin ||K v||², subject to ||v||=1
M = reshape_F(v, (3,3))
```

这是对齐次 rotation-only 方程的代数近似，不直接等于受 SO(3) 约束的最优解。
v 与 −v 等价；真旋转矩阵的 Frobenius norm 为 sqrt(3)，所以 M 还含任意尺度/符号。
先令 det(M)>0（必要时 M=−M），再投影：

```text
M = U_M diag(s_M) V_M.T
R_X = U_M diag(1,1,det(U_M V_M.T)) V_M.T
```

SO(3) projection 消除正尺度并保证 R.T R=I、det=+1。
**不能跳过齐次符号选择**：将 −R_truth 直接投影到 proper rotations 会产生错误旋转。
如果 M 近奇异，本课拒绝；这些数值 guards 不构成通用可靠性判据。

随后平移方程：

```text
(R_A-I) t_X = R_X t_B - t_A
L = stack(R_A-I)
b = stack(R_X t_B-t_A)
t_X = argmin_t ||L t-b||²
```

用 `np.linalg.lstsq`，不显式构造 `(L.T L)^-1`，避免 normal equations 额外恶化条件数。
R_X 的误差会进入 RHS，所以旋转误差也会导致平移误差，单位不同但通过几何链耦合。

**方法边界**：分步估计，没有 joint nonlinear refinement、噪声协方差加权或 outlier rejection。
All-pairs 共享 absolute observations，相关且不独立；这里不把 P=66 当 66 个独立采样，
也不提供置信区间。纯平移即使可约束 R_X，仍不可观测 t_X，此完整 X 求解器拒绝。

## Mathematics：Training / Held-out / Truth

12 个训练姿态先隔离，6 个留出姿态完全不传给 solver。
训练 12 poses 生成 66 pairs；不能先把 18 poses 的 pairs 混在一起再随机拆 pair。

```text
X_est = solve(G_train, H_train)
eye-in-hand Y_i = G_i X_est H_i
eye-to-hand Y_i = inv(G_i) X_est H_i
Y_train_mean = mean_pose(Y_train_i)
held_residual_i = compare(Y_held_i, Y_train_mean)
```

`mean_pose` 对 translation 算术平均，对 rotation 矩阵算术平均后投影 SO(3)。
这是简单 chordal mean，不是 SE(3) maximum likelihood，也没有使用 Y_truth。

分别报告：

- **X truth error**：X_est translation 与 X_truth translation 距离，rotation angle error；仅合成可用。
- **Training pair RMS**：AX_est 与 X_est B 的 translation norm / rotation angle RMS。
- **Training absolute RMS**：Y_train_i 与固定 Y_train_mean 的距离 / angle RMS。
- **Noisy held-out RMS**：从未参与训练的 G/H 重建 Y，比较固定的训练 Y_mean；含 observation noise。
- **Clean held-out RMS**：只在合成评分时替换为无噪声 H，仍比较训练得到的 Y_mean；
  包含 X_est 误差和 Y_mean 偏差，并不是直接 X truth error。

每项 t/R 保持独立单位。RMS=根号下 mean(error²)，不是 angle 的算术 mean。
Relative/absolute translation residual 的 frame、杠杆臂与样本数不同，不能凭大小互称更准。
留出一致性也不能独自证明绝对 accuracy：系统偏差可能在所有样本共同成立。

## Math-to-Code / API

```python
_, _, Vt = np.linalg.svd(K, full_matrices=False)
raw = Vt[-1].reshape((3, 3), order='F')
if np.linalg.det(raw) < 0:
    raw = -raw
R = project_rotation(raw)
t, _, rank, _ = np.linalg.lstsq(L, rhs, rcond=1e-10)
```

`solve_hand_eye(G,H,setup)` 返回新 X 与训练 diagnostics，不修改输入，没有 truth、initial guess 或 held-out 参数。
复用 `relative_pairs` 的 shape/count/finite/SE(3) 检查和 `excitation` 的 K/L 构造。
本课拒绝 L 最大 singular<1e-10 或 s3/s1<1e-4、K 最大 singular<1e-10 或 s8/s1<1e-4，
以及 det(raw) 近零、translation rank<3。门限是当前数值教学选择，并不承诺其它场景精度。

`np.linalg.svd(K,full_matrices=False)` 返回 U/s/Vt；取 Vt 最后一行，别取 U 最后一列。
`np.linalg.lstsq(L,rhs,rcond=1e-10)` 返回 solution、residuals、rank、singular；
代码检查 rank，不依赖 residuals 数组长度。`project_rotation` 返回新合法 rotation。
`nuisance_poses` 返回整批 Y_i；`average_pose` 只接训练 Y_i。
`pose_errors` 返回分开的 translation 数组与 degree 数组；trace/arccos 在零附近有浮点角度下限。
不调用 MuJoCo API，无新依赖；numpy/matplotlib 已有。

## Minimal Experiment / Run → Modify

从仓库根目录：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/hand_eye_calibration.py
python examples/13_perception_geometry/hand_eye_calibration.py --noise-scale 4
python examples/13_perception_geometry/hand_eye_calibration.py --noise-scale 0
```

固定 seed=20261005，18 个 synthetic poses；12 train / 6 held-out。
`broad`：旋转轴 Gaussian 抽样后归一化，angle 均匀 [−65,65]°，motion translation 各轴±0.06 m。
`near_axis`：仅将训练轴的横向分量缩到约 2°，留出轴仍 broad；G0、X/Y 含非零旋转/平移。
各安装/coverage 复用 noise draws，Modify 只缩放它们。
本课两种安装复用 X/Y 数值但赋予不同 frame 含义，不是同一个真实相机安装。

Robot G 精确，camera H 每次添加 independent pose noise：

```text
rotation vector 每轴 σ=0.1 degree * scale，dR=Rodrigues(vector)
R_observed = dR R_clean                      # camera frame 左作用
translation 每轴 σ=0.0005 m * scale
t_observed = t_clean + epsilon               # camera coordinates
```

不是每个 pair 单独加噪声，也不是对整个 H 作 SE(3) 左扰动（后者还会旋转 t）。
σ 是 rotation-vector / translation **分量**标准差，不是总角度或 Euclidean norm 的标准差。
这是抽象 pose-level noise，不等同于 PnP 实际相关误差；不要求正深度、视野或机器人可达性。

输出 ignored `tmp/s12_8b_calibration_noise*_seed20261005/`：
calibration.npz（G/H clean/observed、X truth/estimated、Y truth/training mean）、summary.json、
absolute_residuals.csv（case/split/原始 pose index/t m/R degree）、held_out.png（六个 noisy 留出残差）。
NPZ 每组前 12 行 train、后 6 行 held；CSV clean-held-out 使用同一 index、不同观测版本。

**Run**：运行默认实验，核对两种配置 X_frame，查看 broad/near_axis 的 X error、train pair RMS 与 held RMS。
**Modify**：先预测 noise-scale 1→4 的影响，再运行第二条命令。
固定 geometry、分组和随机噪声方向；比较四组的 X error、train residual、noisy/clean held residual。
同时解释为什么近轴训练旋转 RMS 相似，X accuracy 却可能明显不同。零噪声为附加精确恢复检查。

## Expected / Actual Result → Explanation

2026-10-05 WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5；从 base 自动激活 mujoco，
同执行 shell 核验 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14 / NumPy 2.5.3；3.11 为兼容目标未执行。
默认、scale=4、scale=0 三命令退出 0，无 import error、无新增依赖。

| case | scale | X t error mm | X R error degree | held t RMS mm | held R RMS degree |
| --- | --- | --- | --- | --- | --- |
| eye-in-hand broad | 1 | 0.864328 | 0.091122 | 0.878049 | 0.195203 |
| eye-in-hand near-axis | 1 | 3.873388 | 1.191635 | 5.348452 | 0.841065 |
| eye-to-hand broad | 1 | 1.094404 | 0.108762 | 2.021907 | 0.196506 |
| eye-to-hand near-axis | 1 | 4.114071 | 0.239907 | 3.897320 | 0.263672 |
| eye-in-hand broad | 4 | 3.472723 | 0.363109 | 3.508349 | 0.780337 |
| eye-in-hand near-axis | 4 | 14.443050 | 4.671020 | 21.034590 | 3.294733 |
| eye-to-hand broad | 4 | 4.357076 | 0.435098 | 8.081522 | 0.786351 |
| eye-to-hand near-axis | 4 | 16.200056 | 0.983976 | 15.522542 | 1.068307 |

默认 eye-in-hand broad/near-axis 的 train pair rotation RMS=0.215155/0.208456°，
但 X rotation error=0.091122/1.191635°，体现训练残差与参数精度不同。
L condition broad/near=1.342061/22.177587：近轴平移约束更敏感。
scale=4 本次误差约四倍，不保证每个 seed/大扰动都线性、单调；单 seed 不是稳健性 benchmark。

Clean held-out t/R RMS 默认：eye-in-hand broad=0.288427 mm/0.080388°，
near=5.270615 mm/0.821433°；eye-to-hand broad=1.523333 mm/0.069517°，
near=3.590248 mm/0.150970°。去掉留出 observation noise 后仍可能有错误，
因为 X/Y_mean 是带噪训练得到的。

零噪声四组 X/held translation error<1e-12 m，rotation angle 残差约 0～2.4e-6°
（trace/arccos 的浮点下限），matrix truth recovery atol=1e-10 通过。
所有运行内置 clean recovery、SO(3) 与同轴/纯平移拒绝；不对任意 noisy seed 设置“必须准确”断言。
独立核对三套 NPZ/JSON 与各 96 CSV rows、general-inverse held-out loop、t truth error，
单位尺度×2（R 不变/t×2）、输入未修改、zero-noise 成对反序一致；
独立 rotation-log-vector Procrustes + translation solve 恢复同样 clean X（不是 K vec 方法）。
非法 pose 输入拒绝，CLI noise nan/−1/5 均退出 2。默认 PNG 已目视检查。

曾尝试用既有 OpenCV 作独立 cross-check，但本次环境 `import cv2` 报 ModuleNotFoundError，
与 2026-10-04 的历史记录不同；没有进行 OpenCV 核验，也没有安装依赖。
这不影响本课 NumPy script，不能据此声称当前 PnP/calibration 前课仍可运行。
最初独立 zero-noise 检查在 1e-16 m 量级使用零绝对容差失败；改为 atol=1e-12 m 后通过，
脚本没有为此改变算法。未重跑前课/P0、GUI、真实图像/硬件或抓取；PASS 仅教学工程验证。

## Failure Cases

| 现象 | 原因 / 检查 |
| --- | --- |
| raw v 符号造成错解 | 齐次 ± 等价，但 −R 不是 SO(3)；先选择 det(raw)>0 再投影 |
| vec reshape 默认 C-order | 与 Kronecker 推导不配套；用 order='F' |
| train residual 小，X 误差大 | 近同轴、系统误差或不充分激励；看 informative spectrum、独立 poses |
| 随机拆 pair 作 held-out | shared observations 泄漏；先拆 absolute poses |
| 留出重新计算 Y_mean | 吸收留出误差；使用冻结的训练 Y_mean |
| pure translation / single axis | 完整 X 不可唯一确定；拒绝，不能以 lstsq 最小范数当物理唯一解 |
| pair count 当独立样本数 | 共享噪声相关；这里只作 unweighted algebraic fit |
| 误把 clean-held residual 当 X error | 仍包含训练 Y_mean 偏差，frame/lever arm 也不同 |
| σ deg 当 σ rad 或 m/mm 混用 | 破坏噪声/尺度解释；保留明确单位 |
| PnP outliers / robot noise / 不同步 | 当前未建模；这套单 seed 合成验证不能覆盖 |

## Robotics Context

Eye-in-hand 的 object→base 链为 `T_BO=G_current X_est T_CO`；eye-to-hand 为 `T_BO=X_est T_CO`。
外参 rotation error 在远处会产生杠杆臂放大的点位置误差，t error 不是操作误差的全部。
真实标定还要处理固定 frame 定义、timestamp、metric board geometry、视野与 robot kinematics quality。
本课结束 Stage 12 工程几何路线，Stage 13 再接感知→操作；不自动开始 S13.1。

## Interview Capsule

**30 秒**：成对 robot/camera absolute poses 构造 AX=XB，先用 K 最小右奇异向量估旋转、
处理齐次符号并投影 SO(3)，再用 L t=R_X t_B-t_A 最小二乘求平移。
先拆 absolute poses 防 shared-pair 泄漏，冻结训练 Y_mean 验证留出闭环，区分 residual 与 truth accuracy。

**2 分钟**：写清两种安装 X 与 A/B，解释 vec_F、最小奇异向量、proper-rotation projection 与符号。
说明平移 solve 的输入/输出、旋转误差传播、近轴弱约束与退化拒绝。
描述 12 train/6 held，pair 相关，Y 从 train 闭环估且不从 held 重估。
用相似 train rotation RMS 但不同 X error 的实例，区分 noisy-held、clean-held 与 truth error；
说明此法是无权重分步代数估计，没有 joint ML/outlier robustness，也没有真实图像与机器人验证。

## Must Remember / My Verification

- G/H/frame/单位/时刻必须配套；X frame 随安装方式变化。
- K 的最小右奇异向量不是现成 R：处理符号、按列 reshape、SO(3) projection。
- 有 R 才求 t；保持两个误差的独立单位。
- 先拆姿态再构造 pairs；留出 X/Y_mean 均冻结。
- rank/conditioning、训练拟合、留出一致性与外参 accuracy 是不同证据。

状态仅在根 README 维护。本人于 2026-10-05 确认实验与预测完成，并正确补充
齐次 ± 符号、平移分块式与 clean-held 的 Y_mean 偏差；Run/Modify/Explain 全部确认，Learning Mastered。

反馈核对：eye-to-hand X=T_BC 与 gripper→base 输入、最小右奇异向量、SO(3) 约束、
旋转误差传播和冻结 Y_mean 防泄漏的核心判断正确。
本课 target frame 名为 O，所以 H_i=T_CO(i)；eye-in-hand 的输出仍是 T_GC。
本人补充确认：v/−v 同解，需在投影前选择 raw determinant 为正；
展开 AX=XB 的平移块得 R_A t_X+t_A=R_X t_B+t_X，
整理为 (R_A-I)t_X=R_X t_B-t_A。
先拆 absolute poses 是防止 pair 共享观测；冻结训练 Y_mean 则避免测试数据调整参考。
Clean-held 去除测试测量噪声，但仍评价 X_est 与训练 Y_mean 共同造成的闭环误差，
不是单独的 X accuracy。truth 只用于合成与评分，不传给 solver 或训练均值。
本次仅同步反馈，runtime 沿用 2026-10-05 工程验证，未重跑实验或 GUI。

**Explain**：

1. 两种安装中 solver 输入 G/H、输出 X 分别是什么方向？truth 在哪些位置允许使用？
2. 为什么选 K 的最小右奇异向量？为何要处理 ± 符号并投影 SO(3)？
3. 从 AX=XB 推导 L t=R_X t_B-t_A；旋转估计误差为何影响平移？
4. 为什么先拆 absolute poses？Y_mean 从哪里来，为什么留出时不能重新估它？
5. train pair RMS、X truth error、noisy/clean held-out RMS 各说明什么？近轴为什么可能训练拟合好但 X 不准？

本课学习验证完成后 STOP；下一可选任务 S13.1 perception pose→base→world，等待明确请求。
