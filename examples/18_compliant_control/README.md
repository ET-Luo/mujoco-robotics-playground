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
2026-10-08 本人确认实验与预测完成，并正确回答五项Explain；S17.2 Learning Mastered。
S17.3见下文。

## S17.3 — Stiffness / damping sweep

[完整学习包](../../docs/20_3_impedance_sweep.md) / [代码](impedance_sweep.py)。
复用cartesian_spring XML/initial/gear与三包依赖；6组K/ζ、单独cap与dt对照，共8case。

```bash
python examples/18_compliant_control/impedance_sweep.py
python examples/18_compliant_control/impedance_sweep.py --cap 5
```

先核验上文conda/interpreter。Modify仅把limited组cap2N改5N，预测requested/actual、饱和与settling。
2026-10-08 DESKTOP-781D67A：两命令exit0、递推/动力学/限幅通过；六组恢复，粗dt80ms未settle。
PNG/CSV/JSON只ignored tmp/s17_3_cap2与cap5；无新依赖、contact/GUI/硬件验证。
2026-10-08 本人确认实验、预测并完成五项Explain；S17.3 Learning Mastered。
S17.4见下文。

## S17.4 — Contact Transition Integration

[完整学习包](../../docs/20_4_contact_transition.md) / [代码](contact_transition.py)。
同XY装置加入+Y平面/球法向contact；approach→touch→compliant hold。
复用cartesian_spring常量/XML与三包依赖，无新依赖。

```bash
python examples/18_compliant_control/contact_transition.py
python examples/18_compliant_control/contact_transition.py --slow-speed .06
```

先核验上文conda/interpreter。Modify只将slow接近速度.03→.06m/s，fast=.3不变。
2026-10-08 DESKTOP-781D67A：两命令exit0，contact/动力学/递推通过；
slow.03冲击6.359N通过，slow.06/fast冲击12.512/69.989N超10N教学预算；三组末1s均保持约1.998N。
工程PASS包含预期失败对照，各case task_passed单独记录；无自动超力停止。
产物只ignored tmp/s17_4_v*；无GUI/UR5e/硬件/通用稳定性验证。
2026-10-08 本人确认实验、预测并完成五项Explain；S17.4 Learning Mastered。
S17.5见下文。

## S17.5 — Admittance

[完整学习包](../../docs/20_5_admittance.md) / [代码](admittance.py)。
合成world +Y测力→虚拟M/D/K→可选有界reference→同XY fixture的实际motor跟踪。
合成测力不同时作为物理外力施加；三包依赖与cartesian_spring helper，无新依赖。

```bash
python examples/18_compliant_control/admittance.py
python examples/18_compliant_control/admittance.py --virtual-mass 2
```

先核验上文conda/interpreter。Modify只将软件M_v1→2kg，保持物理mass、D/K和输入不变。
2026-10-08 DESKTOP-781D67A：两个命令各六case exit0；正负镜像、解析reference与actual动力学通过。
有K bias末reference10mm；无K漂移487.55/475.05mm；bounded ref≤40mm/50mm/s。
strong bounded实际速度峰约56.8mm/s，reference界不等于实际安全保证。
产物只ignored tmp/s17_5_m1与m2；无GUI/contact/真实外力/硬件稳定性验证。
Learning Run/Modify/Explain待本人完成；下一小任务S17.6等待明确请求。
