# Stage 17 — Compliant control

入口：[S17.1 完整学习包](../../docs/20_1_cartesian_spring.md)。
Engineering / Learning 状态只见[根 README](../../README.md#stage-17--compliant--contact-rich-control)。

S17.1 用两条正交 slide 组成固定姿态 XY 工具，位移→world 恢复力→Jᵀ广义力→motor。
重力、接触、被动阻尼均为零；不含速度反馈。X 搬运两块1kg质量，Y只搬运工具1kg。
依赖 requirements 已声明的 mujoco/numpy/matplotlib，无 local helper、新依赖或 ROS。

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/cartesian_spring.py
python examples/18_compliant_control/cartesian_spring.py --stiffness 200
```

Modify：先预测刚度翻倍后的初始力、周期与是否停止，再运行第二条命令。
CSV/JSON/PNG 写入 ignored tmp/s17_1_k100 与 tmp/s17_1_k200；headless，不需要GUI。
2026-10-08 Zero：两组工程检查通过；无阻尼持续振荡，不称settling成功。
本人随后确认实验、预测并完成五项Explain；S17.1 Learning Mastered。
只验证本fixture、dt=1ms、K=100/200N/m，无UR5e、接触或硬件稳定性验证。

## S17.2 — Cartesian impedance

[完整学习包](../../docs/20_2_cartesian_impedance.md)：同fixture加入world site速度反馈−Dv，
比较spring与impedance。复用cartesian_spring.py的XML/常量（该helper也导入上述三包），无新依赖。

```bash
python examples/18_compliant_control/cartesian_impedance.py
python examples/18_compliant_control/cartesian_impedance.py --damping-scale .5
```

先按上文核验conda/interpreter。Modify仅把D减半，预测ζ、越过目标幅度和settling。
2026-10-08 Zero：默认ζ=1与半Dζ=.5两命令exit0；settling .976/1.381s，半D越过目标约16.2%。
CSV/JSON/PNG仅ignored tmp/s17_2_*；无contact/GUI/UR5e/硬件或动态饱和恢复验证。
工程已完成，Learning Run/Modify/Explain待本人报告；不自动推进S17.3。
