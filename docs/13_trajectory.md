# Joint-Space Trajectory Generation

本章覆盖 S10.8a～S10.8b。当前只完成 linear interpolation；cubic trajectory 留到 S10.8b。

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

- [ ] Run: personally generate and inspect the 2-second CSV/PNG.
- [ ] Modify: run `--duration 4` and compare velocity and acceleration peaks with 2 seconds.
- [ ] Explain: answer the handoff questions in your own words.

Engineering: Code [x] · Experiment [x] · Docs [x]

Learning: Run [ ] · Modify [ ] · Explain [ ]
