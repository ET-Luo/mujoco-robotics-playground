# Stage 16 — Force & Dynamics Foundations

本阶段追问机器人怎样施力；P0/P1 的 FK/Jacobian/DLS/trajectory 不重教。
入口：[P2 审查与规划](../../docs/p2_plan.md)、[S16.1 学习包](../../docs/19_force_dynamics.md)。
Engineering / Learning 的唯一状态来源：[根 README](../../README.md#stage-16--force--dynamics-foundations)。

S16.1 比较同一外力矩脉冲下的位置伺服、gear=2 的显式 motor feedback 和零力矩 open loop；
测量 actuator_force 与 qfrc_actuator，核对限幅和恢复。只有一个绕 world +z 的 hinge，
无重力/接触，直接施加 qfrc_applied 是外部广义力矩，不是触觉或接触力测量。
依赖 requirements 已声明的 mujoco/numpy/matplotlib，无 local helpers、新依赖或 ROS2。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/17_force_dynamics/actuator_semantics.py
python examples/17_force_dynamics/actuator_semantics.py --kp 40
```

Modify：先预测负载下偏移和恢复过程，再只把 Kp 从20改成40，Kd仍为1。
CSV/JSON/PNG仅写 ignored tmp/s16_1_*。ctrl 的单位随执行器变化；比较的是物理joint torque。
headless Euler dt=1ms 的结果不证明高刚度/接触/UR5e/硬件稳定性；不需要GUI。
2026-10-08 本人确认实验与预测完成，并正确回答五项Explain；S16.1 Learning Mastered。
S16.6–S16.7 只规划，尚无实现。

## S16.2 — Dynamics budget

[完整学习包](../../docs/19_2_manipulator_dynamics.md)：world+y单hinge悬摆，核对惯量、bias、
passive、actuator、external、constraint；无接触，约束项为0。依赖同上，无local helper。

```bash
python examples/17_force_dynamics/manipulator_dynamics.py
python examples/17_force_dynamics/manipulator_dynamics.py --inertia-scale 2
```

先按上文核验conda与解释器。Modify只改变质心惯量，mass/COM/gravity保持固定；
预测关节惯量与加速度比。产物仅ignored tmp/s16_2_*，headless，无GUI需要。
2026-10-08 Engineering Complete + Learning Mastered；本人确认实验、预测并完成五项Explain。

## S16.3 — UR5e gravity compensation

[完整学习包](../../docs/19_3_gravity_compensation.md)：motor真实限幅、独立probe求gravity，
zero/gravity-only/gravity+PD同外力矩对照。另需requirements已声明的mujoco-menagerie，无local helper。

```bash
python examples/17_force_dynamics/gravity_compensation.py
python examples/17_force_dynamics/gravity_compensation.py --cap-scale 0.1
```

先按上文核验环境。Modify只减小cap；产物仅ignored tmp/s16_3_*。
关闭contact/joint limits的裸UR5e自由空间教学模型，无GUI/硬件保证。
2026-10-08 Engineering Complete + Learning Mastered；本人确认实验/预测并补齐force限幅解释。

## S16.4 — Wrench axes and reference points

[学习包](../../docs/19_4_wrench_frames.md)：force-first wrench，换轴/换力矩参考点、
纯力偶、reaction和功率交叉核对。仅NumPy/Matplotlib，无local helper或MuJoCo运行依赖。

```bash
python examples/17_force_dynamics/wrench_frames.py
python examples/17_force_dynamics/wrench_frames.py --lever 0.4
```

按上文先核验环境。Modify只移动负载作用点P，参考点Q/force/couple保持固定。
产物仅ignored tmp/s16_4_*；这是合成几何实验，无contact/GUI/硬件验证。
2026-10-08 Engineering Complete + Learning Mastered；本人确认实验、预测并完成五项Explain。

## S16.5 — Contact wrench measurement

[学习包](../../docs/19_5_contact_wrench.md)：box-plane支撑与斜墙球形指端condim6，
读取完整contact wrench、geom符号、world转换、COM合成和Newton预算。
依赖mujoco/numpy/matplotlib，无local helper或新依赖。

```bash
python examples/17_force_dynamics/contact_wrench.py
python examples/17_force_dynamics/contact_wrench.py --mass 2
```

按上文先核验环境。Modify只改箱体质量；指端case不变，load balance不代表已静止。
产物仅ignored tmp/s16_5_*，headless，无GUI/真实触觉/力控制验证。
2026-10-08 Engineering Complete + Learning Mastered；本人确认实验、预测并完成五项Explain。
