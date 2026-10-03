# Joint-Space Trajectory Generation

本章覆盖 S10.8a～S10.8b：先观察 linear interpolation 的端点速度跳变，再用 cubic time
scaling 满足端点零速度约束。

## S10.8a — Linear Joint-Space Trajectory

### 1. Why

IK 给出一个 `q_goal`，但不说明机器人应怎样随时间到达。若直接把位置目标从 `q_start`
跳到 `q_goal`，理想速度趋于无穷，并会激发执行器饱和、冲击和振动。trajectory generation
把起终关节配置变成带时间参数的 `q(t)`、`qdot(t)` 和 `qddot(t)` reference。

### 2. Intuition

线性插值让每个关节沿直线、以恒定速度从起点走到终点。它消除了 position command 的
瞬时大跳变，但机器人在开始时要从静止瞬间切到恒速，在结束时又瞬间停下；因此 position
连续，velocity 不连续，理想 acceleration 在端点是冲击而不是普通有限值。

### 3. Core Concepts

- Joint-space path：直接插值关节角，不保证末端在 Cartesian 空间走直线。
- Duration `T`：完成运动的总时间。
- Sample period `dt`：离散命令/记录间隔，不改变解析轨迹本身。
- Hold samples：运动前后速度为零，用于显式显示 start/stop discontinuity。
- Continuity：linear trajectory 是 C0 position-continuous，但端点不是 C1 velocity-continuous。

### 4. Mathematics

对 `0≤t≤T`：

```text
s(t) = t/T
q(t) = q_start + s(t)(q_goal-q_start)
qdot(t) = (q_goal-q_start)/T
qddot(t) = 0                      inside the open motion interval
```

运动前后假设 `qdot=0`，所以 t=0/T 存在速度跳变。理想数学模型的加速度包含 impulse；脚本
用 backward finite difference 表示为端点尖峰：

```text
qddot[k] ≈ (qdot[k]-qdot[k-1])/dt
```

| 量 | shape | unit | meaning |
| --- | --- | --- | --- |
| `q_start`, `q_goal` | `(6,)` | rad | UR5e 起终关节配置 |
| `q(t)` | `(N,6)` | rad | sampled joint-position reference |
| `qdot(t)` | `(N,6)` | rad/s | sampled velocity reference |
| `qddot(t)` | `(N,6)` | rad/s² | sampled finite-difference acceleration |

### 5. Math → Code

```python
delta = q_goal - q_start
constant_velocity = delta / duration
q[moving] = q_start + (time[moving, None] / duration) * delta
qdot[moving] = constant_velocity
qddot[1:] = np.diff(qdot, axis=0) / dt
```

NumPy broadcasting turns the scalar phase of each sample into six joint positions. Matplotlib uses the
`Agg` backend to save a PNG without GUI. The script loads the UR5e model only to obtain home, joint names,
and joint ranges; it does not create `MjData` or run dynamics.

### 6. Minimal Experiment

[linear.py](../examples/11_trajectory/linear.py) moves from UR5e home by the joint offset:

```text
[+0.30, -0.20, +0.15, -0.10, +0.20, -0.25] rad
```

It verifies `q_goal` limits, generates one hold sample before and after motion, saves all 19 columns
(`time + 6q + 6qdot + 6qddot`) to CSV, and plots three panels. Generated files live under ignored `tmp/`.

### 7. Expected Result

For `T=2 s`, shoulder_pan has the largest offset `0.30 rad`, so its in-motion speed should be
`0.15 rad/s`. Doubling `T` to 4 s should halve every joint velocity. With fixed `dt=0.01 s`, the sampled
endpoint acceleration spike should also halve because the velocity jump halves.

### 8. Actual Result

2026-10-03，WSL2、conda `mujoco`、Python 3.12.14、MuJoCo 3.14.0：

```bash
python examples/11_trajectory/linear.py
python examples/11_trajectory/linear.py --duration 4
```

| Duration | Samples | max `|qdot|` | peak sampled `|qddot|` | result |
| ---: | ---: | ---: | ---: | --- |
| 2 s | 203 | 0.150 rad/s | 15.0 rad/s² | PASS |
| 4 s | 403 | 0.075 rad/s | 7.5 rad/s² | PASS |

两条命令退出 0，`q_start/q_goal`、运动速度、端点 hold、有限值和 joint limits 检查通过。
PNG 视觉检查确认三层结构：position 线性斜坡、velocity 矩形段、t=0/T 的 acceleration spikes。

### 9. Explanation

相同 `Δq` 用两倍时间完成，根据 `qdot=Δq/T`，速度减半。由于 `dt` 不变，离散尖峰
`Δqdot/dt` 也减半。这个峰值不是轨迹具有“有限的大加速度”；解析线性轨迹在端点速度
不连续，理想加速度是 impulse，离散采样只是把它表现成依赖 `dt` 的有限数字。

直接跳到 `q_goal` 连 position 都不连续；线性轨迹改进为 position 连续，但仍不满足平滑
启停。S10.8b 将用 cubic 时间缩放满足端点零速度，使 velocity 连续并得到有限 acceleration。

### 10. Failure Cases

- Duration 太短：恒定速度过大，可能超过 joint-velocity limit。
- 端点速度跳变：要求瞬时 acceleration，真实执行器无法实现。
- `dt` 太大：采样粗糙；太小只会让离散 spike 数值更大，不会修复解析不连续。
- Joint-space straight line：末端路径可能弯曲或碰撞。
- 只检查 position limits：尚未检查速度、加速度、jerk、torque 和 collision。
- Reference ≠ tracking：生成 q/qdot/qddot 不证明 MuJoCo 或真实 UR5e 能准确跟踪。

### 11. Robotics Application

- Manipulation：在 IK waypoints 之间生成可执行的 joint references。
- Dexterous hand：协调多关节手指运动，但需要更平滑的高阶轨迹。
- Motion planning：planner 给路径，time parameterization 给速度/加速度；二者不是同一问题。
- Surgical robotics：线性插值可教学，但真实器械运动通常要求严格平滑和动态约束。

### 12. Interview Capsule

#### 30 秒版本

A linear joint-space trajectory interpolates each joint as `q=q0+(t/T)(qf-q0)`, giving constant velocity
inside the motion. It is position-continuous, but if the robot is stationary before and after, velocity
jumps at both endpoints, implying impulsive acceleration. It is better than a position step but not a
dynamically smooth trajectory.

#### 2 分钟版本

Joint-space interpolation connects two configurations without requiring Cartesian path planning. For a
fixed duration, each joint velocity is its angle difference divided by duration, so doubling duration
halves velocity. I included hold samples around the motion and plotted q, qdot, and finite-difference
qddot. The plot shows linear position, constant in-motion velocity, and endpoint acceleration spikes.
Those spike values depend on sample time and represent an underlying velocity discontinuity. A cubic or
quintic time law is needed for smooth endpoint constraints, and trajectory generation still does not prove
actuator tracking or collision safety.

### 13. Likely Follow-up Questions

1. Is a straight line in joint space also straight in Cartesian space?
2. Why does doubling duration halve joint velocity?
3. What continuity class does linear interpolation provide at the endpoints?
4. Why does sampled peak acceleration depend on `dt`?
5. What additional boundary conditions can cubic or quintic trajectories satisfy?

### 14. Must Remember

- Linear joint interpolation gives constant in-motion velocity `Δq/T`.
- It is C0 in position but has endpoint velocity discontinuities when starting/stopping at rest.
- Discrete acceleration spikes approximate impulses and depend on sample period.
- Joint-space straight paths do not guarantee Cartesian straightness or collision avoidance.
- A generated reference is not evidence of actuator tracking.

### 15. My Verification

- [x] Run: personally generated and inspected the 2-second CSV/PNG.
- [x] Modify: ran `--duration 4` and compared velocity and acceleration peaks with 2 seconds.
- [x] Explain: explained time scaling, continuity, endpoint impulse, Cartesian curvature, and tracking limits.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

本人验证记录（2026-10-03）：亲自完成 2/4 s 两组运行与数据对比；由固定位移和两倍时间
解释速度减半，指出 linear trajectory 只有 position continuous、端点 velocity 不连续，
其 acceleration 在普通函数意义下不存在并可理解为 impulse。本人也正确说明 nonlinear FK
使 joint-space 直线通常映射为弯曲 Cartesian path，且 reference tracking 还受 torque/
velocity limits、inertia、gravity 等动力学因素影响。S10.8a Learning Mastered。

## S10.8b — Cubic Joint-Space Trajectory

### 1. Problem / Why

S10.8a 的 position reference 连续，但静止 hold 与恒速段之间存在 velocity jump。真实机构
无法瞬间改变速度。现在需要连接相同的 `q_start` 和 `q_goal`，同时满足：

```text
q(0)=q_start, q(T)=q_goal, qdot(0)=0, qdot(T)=0
```

四个标量边界条件正好确定一个三次多项式的四个系数。

### 2. Intuition / Core Concepts

令归一化时间 `tau=t/T`。cubic time scaling 在开头逐渐加速、中点达到最大速度、末尾逐渐
减速。它改变沿 joint-space path 的时间规律，但 path 仍是 `q_start` 到 `q_goal` 的直线。

- `T` 决定时间尺度；`tau` 无量纲。
- position 与 velocity 在静止段连接处连续，因此是 C1。
- acceleration 在起点由 0 跳到 `6 delta_q/T²`，终点再跳回 0，因此不是 C2。
- 平滑启停并非“更慢”：相同 `delta_q` 和 `T` 下，其 peak velocity 是 linear 的 1.5 倍。

### 3. Mathematics, Shapes, Units

```text
s(tau)       = 3 tau² - 2 tau³
ds/dt        = (6 tau - 6 tau²) / T
d²s/dt²      = (6 - 12 tau) / T²

q(t)         = q_start + s(tau) delta_q
qdot(t)      = ds/dt delta_q
qddot(t)     = d²s/dt² delta_q
```

| quantity | shape | unit | frame / meaning |
| --- | --- | --- | --- |
| `tau`, `s` | scalar per sample | dimensionless | normalized time / path progress |
| `q`, `delta_q` | `(N,6)`, `(6,)` | rad | UR5e joint coordinates; no Cartesian frame |
| `qdot` | `(N,6)` | rad/s | joint velocity reference |
| `qddot` | `(N,6)` | rad/s² | joint acceleration reference |

For each joint with displacement `delta_q_j`:

```text
max |qdot_j|  = 1.5 |delta_q_j| / T       at tau=0.5
max |qddot_j| = 6 |delta_q_j| / T²        at the motion endpoints
```

Thus doubling `T` halves peak velocity and quarters peak acceleration.

### 4. Math → Code / Minimal Experiment

```python
tau = time[moving] / duration
phase = 3.0 * tau**2 - 2.0 * tau**3
phase_rate = (6.0 * tau - 6.0 * tau**2) / duration
phase_acceleration = (6.0 - 12.0 * tau) / duration**2

q[moving] = q_start + phase[:, None] * delta
qdot[moving] = phase_rate[:, None] * delta
qddot[moving] = phase_acceleration[:, None] * delta
```

[cubic.py](../examples/11_trajectory/cubic.py) uses the same UR5e home and joint offset as S10.8a.
It analytically computes both trajectories, asserts the cubic boundary conditions and peak formulas, then
saves a comparison CSV and a three-panel PNG. MuJoCo is used only to read model metadata and limits;
`MjData`, `mj_step`, actuators, dynamics, and tracking are not involved.

### 5. Expected and Actual Result

For the largest displacement `0.30 rad` and `T=2 s`, expect cubic peak velocity `0.225 rad/s` and peak
acceleration `0.45 rad/s²`. With `T=4 s`, expect `0.1125 rad/s` and `0.1125 rad/s²`.

2026-10-03 verified in WSL2, conda `mujoco`, Python 3.12.14, MuJoCo 3.14.0:

```bash
python examples/11_trajectory/cubic.py
python examples/11_trajectory/cubic.py --duration 4
```

| T | linear peak `|qdot|` | cubic peak `|qdot|` | cubic peak `|qddot|` | result |
| ---: | ---: | ---: | ---: | --- |
| 2 s | 0.150 rad/s | 0.225 rad/s | 0.4500 rad/s² | PASS |
| 4 s | 0.075 rad/s | 0.1125 rad/s | 0.1125 rad/s² | PASS |

Both commands exited 0. Cubic endpoint velocities were zero and all samples were finite; the configured
goal remained within joint limits. The plot shows linear velocity jumps versus the cubic velocity arch,
and a finite cubic acceleration ramp. The plotted linear acceleration is zero on ordinary motion/hold
intervals; its endpoint impulses are not representable as finite analytic samples in this comparison.
This is reference-generation evidence only, not actuator tracking evidence.

### 6. Explanation and Failure Cases

The cubic's zero endpoint velocities remove the impulsive acceleration required by linear start/stop.
However, its acceleration is nonzero at each motion endpoint and zero in the adjacent holds, so acceleration
still jumps. A quintic polynomial can additionally impose zero endpoint acceleration and achieve C2 joins.

- A short `T` can violate velocity and acceleration limits; check both because they scale differently.
- Cubic does not enforce torque, jerk, collision, or Cartesian-path constraints.
- Joint-space interpolation still maps through nonlinear FK to a generally curved end-effector path.
- A coarse `dt` may miss the exact midpoint peak unless the samples include `tau=0.5`.
- Analytic derivatives should be preferred here; finite differences add sampling artifacts at boundaries.
- A feasible reference does not guarantee that actuators can track it under inertia, gravity, and contact.

### 7. Robotics Context

- Manipulation: smooths joint commands between IK waypoints before actuator tracking.
- Motion planning: separates geometric joint path from its time parameterization.
- Dexterous/surgical robots: endpoint smoothness reduces shock, but higher-order and dynamic constraints
  are commonly needed.

### 8. Interview Capsule

**30 seconds:** A cubic time scaling `s=3 tau²-2 tau³` interpolates two joint configurations while making
both endpoint velocities zero. It is C1 when joined to stationary holds and has finite analytic
acceleration, but acceleration still jumps at the joins, so it is not C2. For fixed displacement, peak
velocity scales as `1/T` and peak acceleration as `1/T²`.

**2 minutes:** I used normalized time to separate a straight joint-space path from its timing. Four
boundary conditions—two positions and two zero velocities—determine a cubic polynomial. Differentiation
gives a parabolic velocity profile and linear acceleration profile. Compared with linear interpolation,
the cubic removes endpoint velocity discontinuities but reaches 1.5 times the linear constant speed for
the same duration. Doubling duration halves velocity and quarters acceleration. This only generates a
kinematic reference; collision, torque limits, and dynamic tracking remain separate validation problems.

### 9. Run / Modify / Explain

Run:

```bash
python examples/11_trajectory/cubic.py
```

Modify:

```bash
python examples/11_trajectory/cubic.py --duration 4
```

Compare the two reported cubic peaks and inspect both PNGs.

Explain:

1. Why do four endpoint constraints require at least a cubic polynomial?
2. Why is cubic peak velocity 1.5 times the linear velocity for the same `delta_q` and `T`?
3. Why does doubling `T` divide peak acceleration by four rather than two?
4. Which quantities are continuous when cubic motion is joined to stationary holds?
5. What does this experiment still not prove about execution on a real robot?

### 10. Must Remember / My Verification

- Cubic time scaling enforces endpoint position and zero velocity.
- It is C1, not C2, when joined to stationary holds.
- Peak velocity scales with `1/T`; peak acceleration scales with `1/T²`.
- Smooth timing does not guarantee a straight Cartesian path or feasible dynamic tracking.

- [x] Run: personally ran the default comparison and inspected its output.
- [x] Modify: ran `--duration 4` and compared both peak-scaling laws.
- [x] Explain: answered the five questions above.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [x] · Modify [x] · Explain [x] — Mastered

本人验证记录（2026-10-03）：亲自完成默认与 4 s 实验运行和数据对比；用四个独立边界条件
需要四个多项式系数解释最低三次阶数，并从速度曲线面积说明 cubic 为补偿前后低速而具有
更高的中点速度。本人确认 duration 翻倍使 peak acceleration 按 `1/T²` 缩放为四分之一，
且 cubic 与静止 hold 连接时 position/velocity 连续、acceleration 不连续。本人也正确指出
reference 未验证 actuator torque/velocity/acceleration capability 或 controller tracking。
S10.8b Learning Mastered。
