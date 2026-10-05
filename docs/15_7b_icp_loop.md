# S12.7b — Nearest-Neighbor ICP Loop

约 1～2h。[代码](../examples/13_perception_geometry/icp_loop.py) ·
[前课 SVD](15_7a_rigid_alignment.md) · [状态唯一来源](../README.md#stage-12--robot-perception-geometry)。

## Problem → Why

现在只有两组米制 3D 点，source S 与 target T 的行身份不对应，点数也可能不同。
给定一个初始 S→T 位姿，怎样交替寻找对应与更新刚体变换？
本课手写小点云 CPU point-to-point ICP，复用前课 `fit_rigid`。
仅合成几何，暂不接 S12.6 的渲染点云，以便分离遮挡、深度噪声与配准算法问题。

## Intuition → Core Concepts

把 source 按当前位姿搬到 target frame，为每个点寻找最近的 target 点；
丢弃太远的对应，然后把保留的对应暂时当作已知，做一次 SVD fit。
更新位姿后重新寻找最近邻，直到变化足够小或无法继续。
像“先猜谁对应谁，再解最佳运动”：近初值容易找到正确对应，远初值可能自洽地停在错误位置。

| 量 | shape / unit / frame |
| --- | --- |
| source / target | `(N,3)` / `(M,3)`；m；S / T，N 不必等于 M |
| R_TS / t_TS | `(3,3)` 无量纲 / `(3,)` m；S→T |
| transformed / updated | `(N,3)`；m；T |
| indices / keep | `(N,)` integer / bool；source row→target row / 距离门限 mask |
| distances / gate | `(N,)` / scalar；Euclidean m，不是 squared distance |
| dR / dt | `(3,3)` / `(3,)`；无量纲 / m；当前 T 点→更新后的 T 点 |
| delta rotation | scalar rad；dR 的旋转角，summary truth error 用 degree |
| accepted fraction | scalar 无量纲；accepted source count / N，非真 overlap |

最近邻为单向 source→target，可多对一；不是 landmark identity，也没有 reciprocal matching。
使用离散小点集而非连续表面法向；这是 point-to-point，不是 point-to-plane ICP。

## Mathematics

列向量约定：`x_i = R_k p_Si + t_k`，所有距离都在 T frame 中计算。

```text
j_i = argmin_j ||x_i - q_Tj||²
I_k = {i : ||x_i - q_T(j_i)|| <= gate}
(dR, dt) = argmin Σ_(i in I_k) ||dR x_i + dt - q_T(j_i)||²
            subject to dR.T dR = I, det(dR) = +1
R_(k+1) = dR R_k
t_(k+1) = dR t_k + dt
T_(k+1) = DeltaT T_k
```

增量左乘，因为它作用于已经在 T frame 的 x_i。展开
`dR (R_k p_Si+t_k)+dt` 就得到组合公式；仅 `t += dt` 会漏掉旧平移的旋转。
这里 dt 是相对于 T 原点的增量变换平移，**不是**累计 `t_(k+1)-t_k`。
T 原点移动会改变 dt 的数值，所以绝对小更新门限是坐标约定下的工程选择。

固定对应的最小二乘应使同一批对应 RMS 不增；记录 before 与 fixed-after 核对它。
重新最近邻、重新 gate 后，参与平均的点集会变，gated RMS **不保证单调**。
本课不把两个不同点集上的 RMS 差单独作为停止条件。

```text
fixed_RMS = sqrt(mean_i ||updated_i - fixed_target_i||²)     # m
stop when ||dt|| <= 1e-7 m AND angle(dR) <= 1e-6 rad
          AND |fixed_RMS_before - fixed_RMS_after| <= 1e-9 m
```

最多 60 次；少于三对停止 `insufficient_pairs`，非共线几何不足停止 `degenerate_pairs`；
到上限为 `max_iterations`；满足上面条件为 `small_update`，它只是局部停止。
没有全局解或位姿正确性保证，也没有统一跨场景的质量接收门限。

## Math-to-Code / API

```python
squared = np.sum((transformed[:, None, :] - target[None, :, :])**2, axis=2)
indices = np.argmin(squared, axis=1)
keep = distances <= gate_m
# 对 transformed 拟合，不是再次对 original source 拟合增量。
dR, dt, _, _ = fit_rigid(transformed[keep], target[indices[keep]])
R, t = dR @ R, dR @ t + dt
```

`nearest_neighbors` 输入 T-frame 点数组，广播产生 `(N,M,3)` 差与 `(N,M)` squared distance，
返回 `(N,)` target indices 和米制距离。`np.argmin(...,axis=1)` 为每行返回最小项下标，
精确并列取第一个；复杂度与内存 O(NM)，这里只处理 N=120、M=120/60。
大型点云需要分块或空间索引，这里不引入新库。

`icp` 输入 source、target、初始 R/t、距离门限和停止参数；返回新 R/t、history 列表与 stop string，
不改输入。`fit_rigid` 返回 SVD 增量，接口与退化检查见前课。
脚本不调用 MuJoCo API，不访问 truth 来求解或停止；truth 仅用于生成与评分。
初始位姿必须有限且属于 SO(3)；点、gate、iteration/tolerance 参数有输入检查。

## Minimal Experiment / Run → Modify

从仓库根目录执行；必要时先初始化 conda shell。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/13_perception_geometry/icp_loop.py
python examples/13_perception_geometry/icp_loop.py --gate-m 0.008
# 可选：观察拒绝，而不是把空 residual 当成功。
python examples/13_perception_geometry/icp_loop.py --gate-m 0.000001
```

固定 seed=20261005。120 个 anisotropic 非对称 3D 点，范围 S 中
x±0.08、y±0.05、z±0.035 m。truth `Rz(25°)`、t=[0.18,-0.10,0.30] m。
target 每轴独立 Gaussian σ=0.0003 m，打乱行顺序。

- `near_full`：Rz(20°)、truth t+[0.004,-0.003,0.002] m 初值，120→120。
- `far_full`：Rz(160°)、truth t 初值，初始 rotation error=135°，120→120。
- `near_partial`：近初值，target 只保留原 source x 大于中位数的半边，120→60。

Partial target 使用独立噪声样本；三组为控制条件对照，不是多 seed benchmark。
Modify 两次运行只改变 gate，geometry/noise/initial poses 保持相同。
输出 ignored `tmp/s12_7b_icp_gate*_seed20261005/`：summary.json、icp.npz、
各 case 的 history.csv、icp.png。NPZ 包括原始/观测点、truth/initial/final R/t、最终对应/mask。
CSV 记录每次拟合前 pair count/fraction、固定对应前后 RMS、重新 NN/gate 的 RMS/count 与增量。
PNG 展示 refreshed gated RMS、拟合前 accepted fraction、最终 XY 投影；
目标点是淡色背景，投影图不显示 Z，不能单凭重叠判断完整 3D 精度。

**Run**：运行默认命令，查看三组 summary、CSV 和 PNG，比较 stop_reason、fraction、RMS 与 truth error。
**Modify**：先预测 gate 从 20→8 mm 对近/远初值及 partial 的影响，再运行第二条命令；
记录三组 pair count、gated RMS、平移/旋转误差。解释为什么更小 RMS 不一定代表更准确位姿。

## Expected / Actual Result → Explanation

2026-10-05 现场 WSL2 kernel 6.18.33.2、Ubuntu 24.04.5；从 base 激活 mujoco，
同执行 shell 核验 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14 / NumPy 2.5.3；代码兼容目标 3.11，未在 3.11 执行；无新依赖。
上述三条命令均退出 0、无 import error。

| gate / case | iterations / stop | final pairs/N | gated RMS mm | t error mm | R error degree |
| --- | --- | --- | --- | --- | --- |
| 20 mm / near full | 3 / small_update | 120/120 | 0.532158 | 0.045091 | 0.047650 |
| 20 mm / far full | 16 / small_update | 95/120 | 11.236907 | 7.135806 | 141.863330 |
| 20 mm / near partial | 4 / small_update | 66/120 | 4.283440 | 1.664512 | 1.475980 |
| 8 mm / near full | 3 / small_update | 120/120 | 0.532158 | 0.045091 | 0.047650 |
| 8 mm / far full | 5 / small_update | 18/120 | 4.900273 | 4.108171 | 139.446286 |
| 8 mm / near partial | 4 / small_update | 60/120 | 0.436909 | 0.093034 | 0.115058 |
| 0.001 mm / all cases | 0 / insufficient_pairs | 0/120 | null | 保留初值 | 保留初值 |

近初值找到正确局部解；噪声存在，所以不要求零误差。
远初值自洽但对应错误：缩 gate 后只保留 15%，RMS 更低，旋转仍错约 139°。
Partial 的 20 mm gate 接纳了一些无真实对应的 source，产生拉偏；8 mm 本次把这些点排除。
60/120 accepted 只是本次恰与真实 overlap 一致，不能作为一般 overlap estimator。
极小 gate 无可用对应，gated RMS=null，不能填成 0 或声称“完美拟合”。

另记录全 source NN RMS（包含非重叠点）和 clean identity RMS（需要合成 truth）：
8 mm partial 的全 source NN RMS≈35.665 mm，但 identity RMS≈0.142 mm，
因为未观测的半边没有 target，而正确位姿仍能映射整个 source 的原几何。
真实数据通常没有 truth；需要结合保留比例、空间覆盖、独立约束评估质量。

内置工程检查：非原点初值的零噪声精确恢复/组合、NN 并列、多对一、SO(3)、
对应不足、共线退化、单次 iteration cap、非法点输入、固定对应 RMS 不增。
独立核验三套 NPZ 的逐点 brute-force NN、mask/count、all-source RMS、t error 与 CSV 行数；
反转 point rows 不改变近初值解，target frame 原点平移后 R 不变/t 同步平移，输入不被修改；
非法 gate 与 reflection 初值拒绝。默认 PNG 已目视检查。
这些 PASS 只表示工程检查通过，不表示 far/partial 均估计成功。
未运行 GUI、实际 RGB-D 注册、多 seed 稳健性评估或机器人执行；未重跑前课/P0。

## Failure Cases

| 现象 | 原因 / 检查 |
| --- | --- |
| small_update 但位姿错误 | ICP 是局部优化，重复形状/坏初值使错误对应自洽；需更好初值和独立约束 |
| gate 太大，partial 被拉偏 | 不重叠区域匹配到边界；查看 count、空间分布与 truth 对照 |
| gate 太小，RMS 降而 fraction 很低 | 选择了易匹配子集，或无法更新；同时报告全部距离、保留数和停止原因 |
| 重复 target / collinear pairs | 多对一允许，但可能缺少方向约束；SVD 退化停止 |
| t += dt / 右乘 Delta | 更新 frame 混淆；展开列向量公式，使用左乘组合 |
| 只看 XY 投影或 gated RMS | 忽略 Z/删点/错误对应；报告 3D metric pose 与 coverage |
| 单向 NN 当真实对应 | NN 没有身份保证，局部密度影响配准；本课未做 robust/reciprocal estimation |
| empty gate 当 RMS=0 | 没有观测证据；输出 null 和 insufficient_pairs |

## Robotics Context

扫描配准、物体模型对可见点云的局部位姿精修、里程计都使用类似的交替思想。
相机/base frame 的命名要由实际输入定义；本课仅一般 S→T。
本实验不是遮挡表面 reconstruction，不替代 hand-eye calibration，也没有估计 scale。
真实 RGB-D 的深度、外参、非均匀采样、对称物体会进一步影响质量。

## Interview Capsule

**30 秒**：point-to-point ICP 把 source 用初值变到 target frame，单向最近邻并按距离 gate，
对保留对应做 centroid/SVD 得增量，左乘更新累计位姿，再重复。
它是局部算法：小更新不是正确性证明；RMS 要和保留比例一起看，partial overlap 与初值会造成偏差。

**2 分钟**：写出 j_i/I_k 与固定对应最小二乘，说明 `(N,M)` 距离矩阵和 O(NM) 小云实现。
展开 dR(Rp+t)+dt 推出 R_new=dR R、t_new=dR t+dt；说明行点数组用 R.T。
解释多对一和退化，列出 small_update、max_iterations、insufficient_pairs、degenerate_pairs。
区分 fixed-pair RMS 保证与刷新 gate 后 RMS 的可变样本；引用远初值 139° 错误却降低 RMS 的对照。
最后用 partial 20→8 mm 展示门限收益与过小拒绝，说明没有 global/robust 保证。

## Must Remember / My Verification

- 对应每轮重新寻找；nearest neighbor 不等于真实身份。
- 对 transformed source 拟合增量；Delta 左乘，旧 t 也需旋转。
- 距离 m、角度 rad/degree 区分；统一 frame 和 metric scale。
- 停止、工程 PASS、正确位姿、Learning Mastered 是不同判断。
- RMS、fraction、空间覆盖与独立误差共同解释结果；不把 partial 未见点强迫匹配。

状态仅在根 README 维护。本人于 2026-10-05 明确确认实验与预测均完成，回答五项 Explain；
Run/Modify/Explain 全部确认，Learning Mastered。
解释覆盖随位姿重新寻找对应、增量旋转同时作用于旧平移、固定对应拟合与刷新对应的区别、
局部小更新不保证正确位姿，以及 gate 过大保留错配/过小删除暂时较远的正确对应。

精度补充：t_TS 是 source 原点在 target frame 中的位置，增量作用于整个 T-frame 结果，
故 t_new=dR@t+dt。刷新后的 gated RMS 只评价保留子集，并非整个点云的质量保证；
必须结合 fraction/coverage。NN 可多对一；无对应时 RMS=null。
本次仅同步本人学习反馈，runtime 证据沿用 2026-10-05 工程验证，未重跑实验或 GUI。

**Explain**：

1. 为什么每轮要重新寻找对应？最近邻为什么可能多对一且不是身份对应？
2. dR/dt 对哪个 frame 的点起作用？推导累计 R/t 为什么左乘，为什么 t 不能直接加 dt？
3. 固定对应 RMS 与刷新 gate 后 RMS 有什么不同？为什么后者不能保证单调？
4. 为什么 far_full 的 small_update 不能当成功？如何解释 8 mm 时低 RMS 与 15% fraction？
5. partial 的 gate 为什么既能减偏差又可能使算法无法更新？没有对应时 RMS 应报告什么？

本课学习验证完成后 STOP；下一可选任务 S12.8a hand-eye geometry，等待明确请求。
