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
后续 S16.2–S16.7 只规划，尚无实现。本课验证记录见学习包。
