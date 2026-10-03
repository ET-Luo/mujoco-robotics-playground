# UR5e Full Jacobian and 6D IK

本章覆盖 S10.3～S10.7。当前 Engineering 已完成到 S10.5；迭代限制和速度限制不会在
本任务提前实现。各 Task 使用独立编号标题，可按 S10.3 → S10.4 → S10.5 导航。

## S10.3 — UR5e Full 6×6 Jacobian

### 1. Why

位置 Jacobian 只能描述末端原点怎样平移，不能描述工具朝向怎样变化。抓取、插入和手术
器械对准通常同时约束 position 与 orientation，因此需要 full Jacobian，把关节速度映射为
末端的三维线速度与三维角速度。

### 2. Intuition

每个关节对末端贡献一根“六维箭头”：前三项说明该关节正向运动会把末端原点推向哪里，
后三项说明会让末端绕世界系哪根轴转动。六列并排后，`J @ qdot` 把各关节贡献线性叠加为
当前姿态附近的末端瞬时运动。

### 3. Core Concepts

- `jacp`：site 世界线速度 Jacobian，shape `(3,nv)`。
- `jacr`：site 世界角速度 Jacobian，shape `(3,nv)`。
- `J = np.vstack((jacp, jacr))`：UR5e 中 shape `(6,6)`。
- 行 0～2 是 world x/y/z 线速度；行 3～5 是绕 world x/y/z 的角速度。
- 每列对应 velocity DOF，通过 `model.jnt_dofadr` 映射，不能假设等于 actuator ID。

### 4. Mathematics

```text
[v_W]   [Jp(q)]
[ω_W] = [Jr(q)] qdot = J(q) qdot
```

| 量 | shape | 单位 | frame / meaning |
| --- | --- | --- | --- |
| `qdot` | `(6,)` | rad/s | 六个 hinge joint 的广义速度 |
| `Jp` | `(3,6)` | m/rad | world-frame position Jacobian |
| `Jr` | `(3,6)` | rad/rad，数值无量纲 | world-frame angular Jacobian |
| `v_W` | `(3,)` | m/s | site 原点世界线速度 |
| `ω_W` | `(3,)` | rad/s | site 世界角速度 |

对单个很小关节增量 `Δq_j`：

```text
Δp_W ≈ Jp[:,j] Δq_j
Δθ_W ≈ Jr[:,j] Δq_j
```

`Δθ_W` 是小旋转向量，不是欧拉角差。六维量混合 m 与 rad，未选择尺度或权重时，不能把
普通 Euclidean norm 当成天然、无单位的“总位姿误差”。

### 5. Math → Code

```python
jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
full_jacobian = np.vstack((jacp, jacr))
twist = full_jacobian @ qdot
```

`mj_jacSite` 原地填写数组并返回 `None`。调用前，`data` 必须对应所需姿态且派生量已由
`mj_forward` 或 `mj_step` 刷新。这里的 Jacobian 和输出 twist 都以 world axes 表达。

### 6. Minimal Experiment

[实验脚本](../examples/09_ur5e_6d_pose/full_jacobian.py)加载 UR5e home，读取
`attachment_site` 的 `jacp/jacr`，默认取 wrist_3 列并计算 `J[:,j]*epsilon`。随后直接令
该关节 `qpos += epsilon`，调用 `mj_forward`，用世界位置差和 world-frame rotation vector
组成实测六维变化。这是运动学有限差分，不发送 `ctrl`、不调用 `mj_step`。

### 7. Expected Result

site 原点位于 wrist_3 轴上，因此 linear 部分应接近零。正向 wrist_3 使 site +x 朝
world -y 偏转，所以 angular 部分应主要指向 world -z。默认 `epsilon=1e-6 rad`：

```text
Δposition    ≈ [0, 0, 0] m
Δorientation ≈ [0, 0, -1e-6] rad, world frame
```

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/09_ur5e_6d_pose/full_jacobian.py
```

退出码 0。wrist_3 列约为：

```text
[0, 0, 0, 3.673e-6, 3.673e-6, -0.999999999987]
```

预测与有限差分六维变化在 `1e-9` 绝对容差内一致；`data.time` 保持 0 s。

增加 `--joint` 后同环境回归 `--joint wrist_2_joint` 也退出 0。该列约为
`[-0.1,3.67e-7,-3.67e-7, 3.67e-6,1,3.67e-6]`；与 wrist_3 不同，它具有明显的
world -x linear contribution，并主要绕 world +y 旋转。六维有限差分仍在 `1e-9` 容差内。

### 9. Explanation

linear 部分为零与 S10.1 相互印证：site 原点在 wrist_3 轴上，不作圆周运动。angular 列
主要为 world -z，符合实际旋转方向。约 `3.67e-6` 的 x/y 分量来自 home keyframe 的有限
小数角和浮点运算，远小于主分量 1，不是新的显著旋转轴。

一致结果支持该列正确描述此姿态附近的瞬时位姿变化，但不证明固定 Jacobian 能精确预测
任意大关节变化。

### 10. Failure Cases

- 未刷新 `data`：Jacobian 与 site pose 可能对应旧 `qpos`。
- 混淆 joint ID、DOF address、qpos address 或 actuator ID：会检查错误列。
- `epsilon` 太大：一阶线性化误差随高阶项增长。
- `epsilon` 太小：差分可能被浮点舍入误差淹没。
- orientation error 接近 0 或 pi：当前基础 axis-angle 提取需要稳定的特殊分支。
- 把几何预测当作实际运动：真实 `data.qvel` 还受执行器、动力学和接触影响。

### 11. Robotics Application

- Manipulation：同时约束抓取点位置和夹爪朝向。
- Dexterous hand：把多关节速度映射为指尖 spatial velocity。
- Motion/control：resolved-rate control、6D IK 和奇异性分析的局部模型。
- Medical/surgical robotics：器械尖端位置与轴向对准必须共同受控。
- Robot learning：可作为解析控制器、动作设计或安全几何先验。

### 12. Interview Capsule

#### 30 秒版本

Full geometric Jacobian maps joint velocity to end-effector spatial velocity. Its top three rows give
world-frame linear velocity and its bottom three rows give world-frame angular velocity. For the 6-DOF
UR5e it is 6×6 and depends on the current configuration. I checked one column against both position and
rotation-vector finite differences.

#### 2 分钟版本

MuJoCo's `mj_jacSite` fills `jacp` and `jacr` in world axes. Stacking them gives
`[v; omega] = J(q) qdot`; each column is one velocity DOF's instantaneous contribution. At home, the
attachment site lies on wrist-3's axis, so its column has zero linear velocity and angular direction near
world -z. A `1e-6 rad` finite difference matched `J[:,j] Δq`. This is a local kinematic check, not proof
of large-motion accuracy, actuator tracking, or dynamics.

### 13. Likely Follow-up Questions

1. Why does the Jacobian depend on the current configuration?
2. What are the shape, units, and frame of `jacp` and `jacr`?
3. Why can a revolute-joint column have zero linear but nonzero angular part?
4. What happens when finite-difference `epsilon` is too large or too small?
5. Why is `J @ qdot` not proof that the physical robot achieves that twist?

### 14. Must Remember

- `mj_jacSite` mutates `jacp/jacr`; it does not return a new Jacobian.
- Rows are world linear xyz followed by world angular xyz; columns are velocity DOFs.
- `J @ qdot` mixes m/s and rad/s.
- A Jacobian is local and must be recomputed as `q` changes.
- Finite-difference agreement validates a specific pose, column, and perturbation.

### 15. My Verification

- [x] Run: personally executed the default wrist-3 experiment.
- [x] Modify: executed the wrist-2 comparison and observed the changed linear/angular contribution.
- [x] Explain: explained the purpose, local derivative, on-axis wrist-3 result, and physical limitations.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

验证记录（2026-10-03）：本人亲自运行默认 wrist-3 与 wrist-2 对比实验，并解释 full
Jacobian 补充 orientation、轴上 site 导致 linear=0/angular≠0、Jacobian 是 FK 的局部
一阶导数，以及实际运动仍受动力学和控制误差影响。本人确认 `jacp` 为 m/rad（乘
`Δq` 后 m、乘 `qdot` 后 m/s），`jacr` 为 rad/rad（乘 `qdot` 后 rad/s）。

## S10.4 — One-step 6D IK

### 1. Why

S10.3 解决了“给定关节变化，末端瞬时怎样动”。IK 反过来解决：给定期望末端 position 与
orientation，应怎样改变关节。先做一次更新，可以把 pose error、full Jacobian 和 joint
increment 的关系单独验证清楚，再进入 S10.5 的奇异性/DLS 与 S10.6 的迭代循环。

### 2. Intuition

当前姿态附近，Jacobian 是关节空间到六维末端变化的局部线性地图。把目标位姿与当前位姿
之差写成六维 error `e`，解 `J Δq = e` 就得到“如果局部线性模型完全准确，应一步消除
误差”的关节增量。真实 FK 是非线性的，所以应用 `Δq` 后必须重新计算 pose，不能把线性
预测当作实际到达。

### 3. Core Concepts

- Target pose：`(p_WT, R_WT)`，均相对 world frame。
- Current pose：`(p_WC, R_WC)`。
- Position error：`p_WT-p_WC`。
- Orientation error：`Log(R_WT R_WC^T)` 对应的 world-frame rotation vector。
- One-step IK：只在当前 `q` 计算一次 `J` 和 `Δq`，不做循环。
- Candidate check：应用前检查有限值和 joint position limits。

### 4. Mathematics

```text
e = [p_WT - p_WC]       shape (6,)
    [Log(R_WT R_WC^T)]

J(q) Δq ≈ e
Δq = solve(J(q), e)
q_candidate = q + Δq
```

| 量 | shape | 单位 | frame / meaning |
| --- | --- | --- | --- |
| `p_WT-p_WC` | `(3,)` | m | world position error |
| orientation error | `(3,)` | rad | world rotation vector |
| `e` | `(6,)` | 前 m、后 rad | world-frame pose error |
| `J` | `(6,6)` | 前行 m/rad、后行 rad/rad | current-pose geometric Jacobian |
| `Δq` | `(6,)` | rad | one-step joint increment |

本课使用方阵且条件良好的 `J`，因此直接 `np.linalg.solve`。这不是通用 IK 方案：奇异、
近奇异、冗余或欠驱动系统需要 least-squares、DLS、约束或其他方法。

### 5. Math → Code

```python
position_error = target_position - current_position
rotation_error = orientation_error_world(current_rotation, target_rotation)
error = np.concatenate((position_error, rotation_error))
jacobian = full_site_jacobian(model, data, site_id)
delta_q = np.linalg.solve(jacobian, error)
q_candidate = q_initial + delta_q
```

`np.linalg.solve` 返回新数组，不修改 `J` 或 `e`。`mj_forward` 则原地刷新 `data.site_xpos`、
`data.site_xmat` 等派生量，返回后必须重新构造 actual error。

### 6. Minimal Experiment

[实验](../examples/10_ur5e_6d_ik/main.py)从 UR5e home 开始。为了获得小、可达且可复现的
6D target，脚本暂时施加一个已知小关节 offset 并只保存产生的 target pose，然后恢复 home。
IK 求解器不读取该 offset，只读取 target position/orientation。随后它计算一次 `J/e/Δq`，
检查候选关节限位，直接设置候选 `qpos` 并用 `mj_forward` 重新评价误差。

### 7. Expected Result

home 的 condition number 应为有限且不过大；`Δq` 应接近生成 target 的小 offset，但因
Jacobian 只是一阶近似，不必逐项完全相等。更新后 position 和 orientation norm 都应下降，
但通常不会精确为零。目标放大时，单步残差预计增大。

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/10_ur5e_6d_ik/main.py
python examples/10_ur5e_6d_ik/main.py --target-scale 5
```

两条命令均退出 0，condition number 均约 8.310681，候选满足 joint limits。

| Target scale | Position before | Position after | Orientation before | Orientation after |
| --- | ---: | ---: | ---: | ---: |
| 1 | 1.405572 mm | 1.922336 µm | 4.528686 mrad | 1.123520 µrad |
| 5 | 7.022047 mm | 47.973753 µm | 22.663208 mrad | 27.950171 µrad |

默认求得的 `Δq` 与生成 offset 仅有微小差别；两项 error 都显著下降，断言通过，simulation
time 保持 0 s。5 倍目标也改善，但留下更大的绝对 residual。

### 9. Explanation

默认目标很小，所以 home 的一阶 Jacobian 能准确描述目标方向，求得的 `Δq` 接近目标生成
offset。残差非零来自 FK 曲率：更新后机器人处在新姿态，而本次只使用旧姿态的 Jacobian。
5 倍目标跨越更大局部范围，高阶非线性项更明显，因此一次更新后的残差更大。这正是后续
迭代 IK 每轮重新计算 FK、error 和 Jacobian 的原因。

### 10. Failure Cases

- `J` 奇异或近奇异：直接 solve 失败或产生很大 `Δq`；S10.5 用 DLS 处理。
- 目标太大：固定 Jacobian 的线性化不准确，error 甚至可能增加。
- 目标不可达：一次 solve 没有能力证明可达性，迭代次数耗尽也不等于数学不可达。
- position/orientation frame 不一致：堆叠出的 `e` 与 `J` 不匹配，方向会错误。
- orientation 接近 pi：当前基础 rotation-vector 提取需要稳定分支。
- 候选违反 joint limits：必须拒绝、缩放或使用约束 IK，不能直接写入。
- 本实验直接设置 `qpos`，不代表 actuator 能瞬时实现该姿态。

### 11. Robotics Application

- Manipulation：由已知 grasp pose 计算关节修正。
- Dexterous hand：由指尖 pose error 生成多关节局部更新。
- Planning/control：为 waypoint IK 或 resolved-rate controller 提供局部步进。
- Medical/surgical robotics：对器械尖端位置与轴向同时校正，但必须额外加入速度、力和安全约束。

### 12. Interview Capsule

#### 30 秒版本

One-step 6D IK stacks world-frame position error and orientation rotation-vector error, then solves
`J(q) Δq = e` using the current full geometric Jacobian. After applying `Δq`, I recompute nonlinear FK
and measure both residuals. It is a local Newton-style update, not a complete IK solver.

#### 2 分钟版本

I represent the pose target as position plus a rotation matrix. Position error is a world vector, and
orientation error is `Log(R_target R_current^T)`, also in world axes, so it matches MuJoCo's site
Jacobian. For a nonsingular 6-DOF UR5e pose I solve the 6×6 system once. A small target reduced errors
from roughly 1.4 mm and 4.5 mrad to 1.9 micrometers and 1.1 microradians. A five-times larger target
left a larger residual, showing the Jacobian is only a local first-order model. Near singularities I would
use DLS and limits rather than an unrestricted inverse step.

### 13. Likely Follow-up Questions

1. Why must position error, orientation error, and Jacobian use compatible frames?
2. Why is a rotation vector preferable to subtracting rotation matrices?
3. Why does one-step IK leave a nonzero residual even for a reachable target?
4. What happens to `Δq` near a singularity?
5. Why is direct `qpos` assignment not equivalent to commanding a real robot?

### 14. Must Remember

- 6D error is `[world position error; world rotation-vector error]`.
- Solve uses the Jacobian at the current pose; recompute FK to evaluate the actual result.
- One successful update is not an iterative, constrained, or dynamics-aware IK solver.
- Larger targets expose linearization error; near singularities expose inverse amplification.
- Always check finite values and joint limits before accepting a candidate.

### 15. My Verification

- [x] Run: personally executed the default one-step experiment.
- [x] Modify: executed `--target-scale 5` and compared both final residuals with scale 1.
- [x] Explain: explained frames/units, local linearization residual, FK recomputation, and singularity risk.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

验证记录（2026-10-03）：本人亲自运行默认与 `--target-scale 5`，两次均退出 0，观察到
condition number 约 8.31、候选满足 joint limits，且 5 倍目标留下更大单步 residual。
本人解释 Jacobian 一阶近似与非线性 FK、重算 FK、高阶残差，并确认 6D error 的
world-frame m/rad 单位，以及奇异时 solve 失败、近奇异时巨大且噪声敏感的 `Δq`。

## S10.5 — Damped Least Squares near Singularity

### 1. Why

接近奇异姿态时，某些末端运动方向对关节变化的响应极弱。普通 inverse/pseudo-inverse 为了
修正该方向的小误差，会要求非常大的关节更新，并同时放大测量噪声和模型误差。DLS 用
阻尼牺牲部分瞬时精度，换取有限、平滑、数值更稳定的更新。

### 2. Intuition

把 Jacobian 想成沿不同 task directions 具有不同“传动效率”的地图。最小奇异值对应效率
最低的方向：普通逆解会用 `1/σ` 强行放大。DLS 像在逆解中加入软弹簧，不再坚持一步完全
消除误差；方向越弱，抑制越强。`λ` 越大越保守，但残留误差也越多。

### 3. Core Concepts

- Singular value `σ_i`：关节变化沿某个方向传到 task space 的局部增益。
- Condition number `σ_max/σ_min`：数值敏感性指标；越大表示方向性差异越严重。
- Least-squares：最小化 `||JΔq-e||²`，近奇异方向可能产生大更新。
- DLS：同时惩罚 task residual 与 joint increment。
- 本实验用 `wrist_2≈0` 制造 wrist singularity，并沿最弱 left-singular direction 给误差。

### 4. Mathematics

```text
ordinary least-squares:
    Δq = J⁺ e

DLS objective:
    min_Δq ||JΔq - e||² + λ² ||Δq||²

DLS solution:
    Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

若 `J=UΣVᵀ`，普通 pseudo-inverse 对方向 `i` 的增益是 `1/σ_i`；DLS 增益为：

```text
σ_i / (σ_i² + λ²)
```

因此当 `σ_i << λ` 时，弱方向不再被无限放大。

| 量 | shape | unit / convention |
| --- | --- | --- |
| `J` | `(6,6)` | 前三行 m/rad，后三行 rad/rad |
| `e` | `(6,)` | 前三项 m，后三项 rad，world frame |
| `λ` | scalar | 与本次数值缩放后的 singular values 配套 |
| `Δq` | `(6,)` | rad |

position 与 orientation 的数值缩放会改变 singular values、condition number 和合适的 `λ`；
这些指标不是脱离单位约定的绝对物理常数。

### 5. Math → Code

```python
task_matrix = J @ J.T + damping**2 * np.eye(6)
delta_q = J.T @ np.linalg.solve(task_matrix, error)
```

代码用 `solve` 解线性系统，不显式计算 matrix inverse。普通基线使用
`np.linalg.lstsq(J, error, rcond=None)`，即使矩阵退化也能返回最小范数 least-squares 解。

### 6. Minimal Experiment

[dls_comparison.py](../examples/10_ur5e_6d_ik/dls_comparison.py)执行：

1. reset 到 home，并把 `wrist_2_joint` 设为 `0.01 rad`，接近 `0` singular reference。
2. 对 full Jacobian 做 SVD，记录 singular values 和 condition number。
3. 沿最小 singular value 对应的 left-singular direction 构造 `1e-3` 数值 6D error。
4. 比较 least-squares、`λ=1e-3` 和 `λ=1e-2` 的 `||Δq||`。
5. 分别应用一次候选，用 nonlinear FK 评价 position/orientation residual 和 joint limits。

### 7. Expected Result

近奇异时 `σ_min` 很小、condition number 很大。普通 least-squares 的 `||Δq||` 最大；small
λ 只略微抑制；medium λ 明显减小更新但保留更多 residual。把 wrist angle 改为 `0.05 rad`
后离 singular reference 更远，condition number 和普通更新范数应下降。

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/10_ur5e_6d_ik/dls_comparison.py
python examples/10_ur5e_6d_ik/dls_comparison.py --wrist-angle 0.05
```

两条命令均退出 0，候选均有限并满足 joint limits。

| wrist₂ | condition | method | λ | `||Δq||` rad | pos after | rot after |
| ---: | ---: | --- | ---: | ---: | ---: | ---: |
| 0.01 | 768.96 | least-squares | — | 0.36816 | 3.2145 mm | 0.2562 mrad |
| 0.01 | 768.96 | DLS small | 0.001 | 0.32422 | 2.4963 mm | 0.2089 mrad |
| 0.01 | 768.96 | DLS medium | 0.01 | 0.02530 | 0.8358 mm | 0.4109 mrad |
| 0.05 | 154.02 | least-squares | — | 0.07374 | 0.1298 mm | 0.0512 mrad |
| 0.05 | 154.02 | DLS small | 0.001 | 0.07334 | 0.1284 mm | 0.0508 mrad |
| 0.05 | 154.02 | DLS medium | 0.01 | 0.04777 | 0.3207 mm | 0.1567 mrad |

默认 initial error 为 position `0.8974 mm`、orientation `0.4413 mrad`。近奇异 least-squares
和 small damping 的大步跨出局部范围，使 position residual 反而增大；medium damping 的
更新保持较小，组合数值 residual 从 `0.001` 降至约 `0.000931`。

### 9. Explanation

`wrist_2=0.01` 时 `σ_min≈0.002716`，普通弱方向增益约 `1/σ_min≈368`，所以 `1e-3`
error 对应约 `0.368 rad` 更新。`λ=0.01` 大于 `σ_min`，将弱方向增益降到约 25.3，显著
限制更新。代价是它不再尝试一步消除全部误差，orientation residual 可能更大。

当 wrist angle 增至 `0.05`，`σ_min≈0.01356`、condition≈154，普通更新降到约
`0.0737 rad`，并重新落在较可靠的局部范围。实验展示的是 stability/accuracy trade-off，
不是“λ 越大越好”。

### 10. Failure Cases

- `λ` 太小：行为接近 pseudo-inverse，仍会放大弱方向、噪声和模型误差。
- `λ` 太大：更新过小，收敛缓慢且 residual 偏大。
- 固定 λ 不适合全部姿态：实际系统常根据 singular values 或 manipulability 自适应。
- 混合单位/权重不合理：会改变 SVD、condition 和阻尼的实际含义。
- 一次 DLS 仍是局部更新：大目标、关节限位与碰撞仍需迭代和约束。
- 仿真有限更新不等于真实机器人安全；仍需速度、加速度、力矩和急停限制。

### 11. Robotics Application

- Manipulation：腕部对准抓取姿态时避免奇异附近关节跳变。
- Dexterous hand：多指 Jacobian 退化时稳定指尖修正。
- Motion/control：resolved-rate control 和 operational-space motion 的数值保护。
- Surgical robotics：器械接近退化方向时优先限制不可接受的大关节命令。

### 12. Interview Capsule

#### 30 秒版本

DLS stabilizes Jacobian IK near singularities by solving a regularized least-squares problem. Instead
of amplifying a small singular value with `1/sigma`, it uses `sigma/(sigma^2+lambda^2)`. This bounds
joint updates and noise amplification, at the cost of larger task-space residual and slower convergence.

#### 2 分钟版本

Near a singularity, at least one singular value of the geometric Jacobian is small, so a pseudo-inverse
can request a huge joint update for a tiny Cartesian correction. DLS minimizes task residual plus
`lambda^2 ||delta_q||^2`, giving `J.T (J J.T + lambda^2 I)^-1 e`. In my UR5e wrist experiment the
condition number was about 769: least-squares requested 0.368 rad, while lambda 0.01 reduced it to
0.0253 rad. Moving farther from the singularity reduced the condition number to 154. Damping improves
stability but sacrifices one-step accuracy, and its meaning depends on position/orientation scaling.

### 13. Likely Follow-up Questions

1. How does SVD explain pseudo-inverse amplification near a singularity?
2. What happens when `λ` is too small or too large?
3. Why should position and orientation weighting be chosen explicitly?
4. Why use `solve` instead of explicitly forming a matrix inverse?
5. How could damping be adapted online from singular values?

### 14. Must Remember

- Small `σ_min` makes pseudo-inverse updates and noise amplification large.
- DLS replaces `1/σ` with `σ/(σ²+λ²)`.
- Larger `λ` means smaller, safer updates but more residual/slower correction.
- Condition number and damping depend on pose and task-space scaling.
- DLS improves numerical robustness; it does not solve limits, collisions, or reachability.

### 15. My Verification

- [x] Run: personally executed the default near-singular comparison.
- [x] Modify: executed `--wrist-angle 0.05` and compared condition number and update norms.
- [x] Explain: explained weak directions, SVD gain regularization, damping trade-off, and scaling.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

本人实测记录（2026-10-03）：默认与 `--wrist-angle 0.05` 均 PASS；观察到 condition
从 768.96 降至 154.02，least-squares `||Δq||` 从 0.36816 rad 降至 0.07374 rad。
本人解释弱 Cartesian direction、λ 太小/太大的边界、scaling 对 λ 的影响，并确认 DLS
把每个 SVD direction 的增益从 `1/σ` 改为 `σ/(σ²+λ²)`；near-singular 大 `Δq` 会跨出
Jacobian 的局部有效范围，使高阶 FK 项令真实 residual 增大。

## S10.6 — Bounded Iterative 6D IK

### 1. Why

一次 Jacobian 更新只能使用当前姿态附近的一阶近似。迭代 IK 在每次接受更新后重新计算
FK、6D error 和 Jacobian，从而逐步修正非线性残差。但一个可用于实验的 solver 还必须
有限终止、限制单步、检查关节位置，并区分成功、预算耗尽和无效输入。

### 2. Intuition

把 S10.4 的“一张局部地图”改成边走边重画地图：每次只走有限一步，到新姿态后重新观察
目标方向。小步更贴近局部线性模型，但通常需要更多轮；大步可能更快，也更容易跨出局部
有效区域。停止原因是实验结果的一部分，不能把所有未成功都叫作“不可达”。

### 3. Core Concepts

- 双成功判据：position norm 与 orientation norm 必须分别低于容差。
- `max_updates`：最多接受的更新数量；初始状态是 update 0。
- `max_joint_step`：每轮 `max(abs(Δq))` 的上限，单位 rad。
- Joint-limit rejection：候选越界则不写入 `qpos`，并报告独立失败状态。
- Input validation：非有限 position 或非 SO(3) rotation 在迭代前拒绝。
- Explicit status：`SUCCESS`、`UPDATE_LIMIT`、`JOINT_LIMIT_REJECTED`、`INVALID_TARGET` 等。

### 4. Mathematics

每轮 `k`：

```text
e_k = [p_target - p(q_k); Log(R_target R(q_k)^T)]
Δq_raw = J_k^T (J_k J_k^T + λ²I)^-1 e_k
s = min(1, Δq_max / max_i |Δq_raw_i|)
Δq_k = s Δq_raw
q_candidate = q_k + Δq_k
```

只有候选满足 limits 才接受 `q_{k+1}=q_candidate`。成功要求：

```text
||e_position|| < 1e-4 m
and
||e_orientation|| < 1e-3 rad
```

`Δq_max` 是每轮角度增量，不是速度；S10.7 才通过 control period 将它转换为速度限制。

### 5. Math → Code

循环中顺序保持显式：读取 pose → 计算两个 error norm → 检查 success/budget → 重算
`mj_jacSite` → DLS → 公共比例限步 → 检查 candidate limits → 写入 `qpos` → `mj_forward`。
公共比例保留 `Δq_raw` 的方向，而逐元素 clipping 会改变关节更新方向。

### 6. Minimal Experiment

[iterative_ik.py](../examples/10_ur5e_6d_ik/iterative_ik.py)包含四种场景：

- `reachable`：由小 joint offset 生成的可达 6D target。
- `difficult`：由较大 joint offset 生成，需多轮限步更新。
- `unreachable`：把 target position 沿 world +x 移动 2 m。
- `invalid`：position 含 NaN，在进入循环前拒绝。

默认 `λ=0.01`、`max_joint_step=0.05 rad`、`max_updates=80`。

### 7. Expected Result

小目标应快速满足双容差；困难目标需要更多轮；减小单步上限应增加 accepted updates。
不可达目标应以显式非成功状态有限停止，无效目标应在 0 次更新时拒绝。所有已接受步都应
满足 `max_joint_step` 与 joint limits。

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/10_ur5e_6d_ik/iterative_ik.py
python examples/10_ur5e_6d_ik/iterative_ik.py --scenario difficult --max-joint-step 0.02
```

| Scenario | Step limit | Status | Accepted updates | Final position | Final orientation |
| --- | ---: | --- | ---: | ---: | ---: |
| reachable | 0.05 rad | SUCCESS | 2 | 0.304 µm | 0.054 µrad |
| difficult | 0.05 rad | SUCCESS | 8 | 1.314 µm | 0.949 µrad |
| difficult | 0.02 rad | SUCCESS | 18 | 8.508 µm | 6.118 µrad |
| unreachable | 0.05 rad | UPDATE_LIMIT | 80 | 1.166962 m | 7.680 mrad |
| invalid | — | INVALID_TARGET | 0 | NaN | NaN |

两条命令退出 0，所有断言通过，simulation time 保持 0 s。

### 9. Explanation

每轮重算 Jacobian 让可达目标逐渐消除高阶残差。困难目标在 0.05 rad 限步下需要 8 次；
收紧到 0.02 rad 后每步更保守，因此需要 18 次，但仍满足同一双容差。不可达场景虽然
position error 从 2 m 降至约 1.167 m，却在 80 次时仍远超容差；直接终止原因是
`UPDATE_LIMIT`。结合目标超出 UR5e 工作空间可解释本例几何不可达，但“次数耗尽”这个状态
本身不能证明任意目标不可达。

### 10. Failure Cases

- Update limit：可能是不可达、收敛慢、阻尼/步长不合适或局部极小，不应混为一类。
- Joint-limit rejection：当前简单实现停止；更完整方法可缩步、投影或做 constrained IK。
- Near singularity：DLS 有帮助，但固定 λ 仍可能过强或不足。
- Orientation near pi：当前基础 rotation-vector 分支主动拒绝。
- Collision：本课未做 self/environment collision checking。
- Dynamics：直接设置 `qpos` 不验证 actuator、速度、加速度或实际轨迹。

### 11. Robotics Application

- Manipulation：迭代求取 pre-grasp/grasp waypoint 的 joint configuration。
- Dexterous hand：多轮修正指尖 pose，同时监测 joint range。
- Motion planning：可作为 waypoint 的局部 IK，但不能替代 collision-aware planner。
- Surgical robotics：需要在本结构上增加严格 workspace、速度、力和安全约束。

### 12. Interview Capsule

#### 30 秒版本

Iterative 6D IK repeatedly recomputes FK, pose error, and the full Jacobian, then applies a bounded DLS
joint update. I use separate position and orientation tolerances, a maximum update count, per-step joint
limit, and joint-position checks. Every exit has an explicit status; exhausting iterations is not by itself
a proof that a target is mathematically unreachable.

#### 2 分钟版本

At each iteration I compute world-frame position and rotation-vector errors, solve a DLS update, scale
the whole joint vector so its largest component stays below the step bound, reject candidates outside
joint limits, and recompute FK. A small UR5e target converged in two updates and a larger target in eight.
Reducing the step bound from 0.05 to 0.02 rad increased that to eighteen updates, illustrating the
local-accuracy versus progress trade-off. A 2-meter target stopped at the 80-update budget with a large
residual, reported as UPDATE_LIMIT rather than automatically labeled unreachable.

### 13. Likely Follow-up Questions

1. Why must FK and the Jacobian be recomputed after every accepted update?
2. Why use separate position and orientation tolerances?
3. Why scale the entire `Δq` vector instead of clipping each joint independently?
4. What can cause `UPDATE_LIMIT` besides an unreachable target?
5. What additional constraints are required before using this on a real robot?

### 14. Must Remember

- Recompute pose error and Jacobian at every accepted configuration.
- Success requires both position and orientation tolerances.
- Bound each step and reject candidates outside joint limits.
- Report terminal causes explicitly; update-limit exhaustion is not an unreachability proof.
- This is geometric IK, not actuator control, trajectory generation, or collision avoidance.

### 15. My Verification

- [x] Run: personally executed all four default scenarios.
- [x] Modify: ran difficult with `--max-joint-step 0.02` and compared it with 0.05.
- [x] Explain: explained local recomputation, dual tolerances, common scaling, terminal causes, and deployment limits.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

本人验证记录（2026-10-03）：亲自完成 all-scenarios Run 和 0.02/0.05 rad step comparison；
解释每轮重算源于 nonlinear FK 与 local Jacobian、双容差对应不同物理量、公共比例缩放保留
joint-space direction，并指出 step 太小等情况也会导致 UPDATE_LIMIT。本人正确限定真实
系统还需要 trajectory、控制接口和动力学约束；补充的其他 budget-exhaustion 原因包括
阻尼过大、近奇异和不良局部收敛。S10.6 Learning Mastered。

## S10.7 — Joint Velocity Limits

### 1. Why

S10.6 的 `max_joint_step` 只限制“一轮最多走多少 rad”，没有时间尺度。机器人接口通常限制
rad/s，因此必须结合控制周期把速度上限转换成每轮角度上限。否则同一 `Δq` 在 100 Hz 与
50 Hz 下代表不同速度，无法判断命令是否安全或可执行。

### 2. Intuition

速度是单位时间内的角度变化。控制周期越短，同样的速度在一轮内允许走的角度越小：
`qdot_max` 像限速牌，`control_dt` 是每帧经过的时间，两者乘积才是这一帧允许的路程。

### 3. Core Concepts

- `qdot_command`：几何算法希望发送的 joint velocity reference，单位 rad/s。
- `control_dt`：假定的命令周期，单位 s。
- `step_limit_i=qdot_max_i*control_dt`：每关节每轮角度上限，单位 rad。
- Common scaling：用单个比例缩放整条 `Δq`，保留 DLS joint-space direction。
- `data.qvel`：MuJoCo 动力学状态中的实际 generalized velocity；不是任意局部变量的别名。

### 4. Mathematics

```text
Δq_limit,i = qdot_limit,i Δt
s = min(1, min_i Δq_limit,i / |Δq_raw,i|)
Δq_limited = s Δq_raw
qdot_command = Δq_limited / Δt
```

| 量 | shape | unit | meaning |
| --- | --- | --- | --- |
| `qdot_limit` | `(6,)` | rad/s | 每关节命令速度上限 |
| `control_dt` | scalar | s | 假定控制周期 |
| `step_limit` | `(6,)` | rad/update | 每轮最大角度变化 |
| `Δq_limited` | `(6,)` | rad | 本轮几何更新 |
| `qdot_command` | `(6,)` | rad/s | 受限几何速度命令 |

本实验中所有量都是 joint space，不涉及 world/local Cartesian frame。

### 5. Math → Code

```python
step_limits = speed_limits * control_dt
ratios = step_limits / abs(delta_q_raw)
scale = min(1.0, min(ratios))
delta_q = scale * delta_q_raw
qdot_command = delta_q / control_dt
```

实现对零分量使用 `np.divide(..., where=...)`，避免除零。每轮断言所有
`abs(qdot_command) <= speed_limits`。随后只是 `qpos += delta_q` 与 `mj_forward`，没有把命令
写入 actuator 或调用 `mj_step`。

### 6. Minimal Experiment

[velocity_limited_ik.py](../examples/10_ur5e_6d_ik/velocity_limited_ik.py)跟踪 S10.6 的 difficult
target，固定 `qdot_max=0.5 rad/s`，比较：

- `control_dt=0.02 s` → `step_limit=0.01 rad/update`
- `control_dt=0.01 s` → `step_limit=0.005 rad/update`

记录首次 raw/limited update、最大命令速度、饱和轮数、更新总数、虚拟时间和 `data.qvel`。

### 7. Expected Result

周期减半后每轮角度上限减半，因此达到同一目标大约需要两倍更新；若速度上限相同，首轮
最大 `|qdot_command|` 都应饱和在 0.5 rad/s。更新数乘周期得到的虚拟时间应大致接近。
由于没有动力学推进，`data.qvel` 不会自动等于命令。

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/10_ur5e_6d_ik/velocity_limited_ik.py
python examples/10_ur5e_6d_ik/velocity_limited_ik.py --control-dt 0.01
```

| `dt` | step limit | updates | virtual time | saturated | max command |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.02 s | 0.010 rad | 35 | 0.700 s | 34 | 0.500 rad/s |
| 0.01 s | 0.005 rad | 69 | 0.690 s | 68 | 0.500 rad/s |

两条命令退出 0，position/orientation 均满足 S10.6 容差；每轮速度断言通过，候选满足 joint
limits。两组最终 `data.qvel=[0,0,0,0,0,0] rad/s`，`data.time=0 s`。

### 9. Explanation

`dt` 减半使允许的 `Δq` 减半，所以饱和阶段的更新数近似翻倍；35×0.02 与 69×0.01 给出
相近虚拟时间。首轮 raw direction 相同，公共 scale 和 `dt` 同时减半，因此两组得到相同
首轮 `qdot_command`，最大分量都是 -0.5 rad/s。

`data.qvel=0` 不表示真实机器人会静止，只表示脚本从零速度状态开始、直接改 `qpos`，并且
`mj_forward` 只刷新运动学派生量而不积分动力学。实际 qvel 需要 actuator/controller 和
`mj_step`，并会受惯性、重力、摩擦、饱和、接触与带宽影响。

### 10. Failure Cases

- 使用错误或变化的实际周期：命令的真实 rad/s 与假定值不符。
- 只限制 `Δq` 不考虑 `dt`：无法在不同控制频率间保持相同速度约束。
- 逐元素 clipping：改变 DLS joint direction，可能降低 task-space一致性。
- 只限速度：仍可能违反 acceleration、jerk、torque、collision 或 workspace 限制。
- 把 command 当作 measured qvel：掩盖 tracking error 和动力学风险。

### 11. Robotics Application

- Manipulation：限制末端对准过程中的关节命令速度。
- Dexterous hand：避免多指 IK 产生瞬时高速关节动作。
- Motion/control：连接几何 IK 与周期性 joint-velocity controller。
- Surgical robotics：速度限制是必要安全层，但还需加速度、力和工作空间约束。

### 12. Interview Capsule

#### 30 秒版本

I convert a joint-velocity limit into a per-cycle IK step using `delta_q_max=qdot_max*control_dt`.
I scale the complete DLS update with one factor, then compute `qdot_command=delta_q/control_dt` and
verify every component. This is still a geometric command; it is not MuJoCo `data.qvel` until a controller
and dynamics actually execute it.

#### 2 分钟版本

An angular step has no velocity meaning without a control period. For each joint I compute its allowed
step from rad/s times seconds, then choose the most restrictive common scale so the DLS direction is
preserved. With a 0.5 rad/s limit, changing dt from 0.02 to 0.01 seconds halved the step limit from 0.01
to 0.005 rad and increased updates from 35 to 69, while virtual time stayed near 0.7 seconds. The script
uses direct qpos integration and mj_forward, so data.qvel remains zero and does not validate tracking.

### 13. Likely Follow-up Questions

1. Why does halving `control_dt` halve the allowed joint increment?
2. Why use a common scale instead of clipping every joint independently?
3. What is the difference between `qdot_command` and measured `data.qvel`?
4. Why are velocity limits insufficient without acceleration and torque limits?
5. How would control-period jitter affect this calculation?

### 14. Must Remember

- `Δq_max=qdot_max*control_dt` connects rad/s to rad/update.
- Recompute the limit if the actual control period changes.
- Common scaling preserves the IK joint-space direction.
- Geometric command velocity is not actual `data.qvel`.
- Real deployment also needs acceleration, jerk, torque, collision and watchdog limits.

### 15. My Verification

- [x] Run: personally executed the default `dt=0.02 s` experiment.
- [x] Modify: executed `--control-dt 0.01` and compared step limit, updates, virtual time, and max command.
- [x] Explain: explained the time-step conversion, common scaling, command/state distinction, and safety gaps.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

本人验证记录（2026-10-03）：亲自完成 dt=0.02/0.01 s 两组运行与结果对比；由
`qdot=Δq/Δt` 推导 step limit，解释 dt 减半使每步距离减半、公共比例避免改变 joint-space
direction，并区分期望的 `qdot_command` 与动力学状态 `data.qvel`。本人指出真实安全还需
acceleration、jerk、collision avoidance 等约束。S10.7 Learning Mastered。
