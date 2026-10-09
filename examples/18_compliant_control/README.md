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
2026-10-08 本人确认实验、预测并完成五项Explain；S17.5 Learning Mastered。
S17.6见下文。

## S17.6 — Normal force control

[完整学习包](../../docs/20_6_normal_force.md) / [代码](normal_force.py)。
复用contact_transition模型/measure及cartesian_spring常量/三包依赖，无新依赖。
真实MuJoCo接触反力→P力误差reference速度→有界reference积分→motor跟踪。

```bash
python examples/18_compliant_control/normal_force.py
python examples/18_compliant_control/normal_force.py --target-force 4
```

先核验上文conda/interpreter。Modify仅把target2N改4N，比较closed与fixed的反力/reference。
2026-10-08 DESKTOP-781D67A：最终两命令exit0；7…8s closed最大力误差.004202/.008332N。
fixed始终约1.998N；loss3s接触丢失退出，absent3s搜索超时，退出后继续physics/local hold。
初版4N在5…6s误差.070953N未通过，最终延长到8s，门限不变；无通用收敛保证。
产物只ignored tmp/s17_6_f2/f4；无GUI/UR5e/硬件停止或动态饱和恢复验证。
2026-10-08 本人确认实验、预测并完成五项Explain；坐标/gate精度补充见学习包，S17.6 Learning Mastered。
S17.7见下文。

## S17.7 — Hybrid position-force

[完整学习包](../../docs/20_7_hybrid_position_force.md) / [代码](hybrid_control.py)。
复用normal_force/helper依赖链，surface切向位置与法向力生成reference经互斥selector合成。
仅一次已知平面点到点移动，不实施下一课连续扫描；无新依赖。

```bash
python examples/18_compliant_control/hybrid_control.py
python examples/18_compliant_control/hybrid_control.py --distance .06
```

先核验上文conda/interpreter。Modify仅将切向距离30→60mm，时长2s与目标2N不变。
2026-10-08 DESKTOP-781D67A：两命令exit0；运动切向误差.204596/.409191mm，法向误差.035781N/contact100%。
normal_only力合格但切向失败；错误frame/selector静态拒绝、动力学/功率与normal不变核验通过。
产物只ignored tmp/s17_7_d0.03/d0.06；无GUI/斜平面动态/UR5e/硬件或通用解耦保证。
2026-10-08 本人确认实验、预测并完成五项Explain；S17.7 Learning Mastered。
S17.8a见下文。


## S17.8a — Surface following / fixture

[完整学习包](../../docs/20_8a_surface_following_fixture.md) / [代码](surface_following.py)。
同fixture串接approach/load/往返scan/hold，失败锁存actual reference，继续physics。
复用hybrid_control/normal_force/helper与mujoco/numpy/matplotlib，无新依赖。

```bash
python examples/18_compliant_control/surface_following.py
python examples/18_compliant_control/surface_following.py --leg-duration 1
```

先核验上文conda/interpreter。Modify只把每段2s减为1s，距离30mm/目标2N不变。
2026-10-09 Zero：两命令各四case exit0；正常扫描切向最大误差.204596/.740687mm，
法向最大误差.035781N/contact100%/无饱和；loss/overforce/saturation均预期退出。
低cap2s组退出后实际漂移至97.63mm，reference锁存不是立即停机保证。
独立CSV/controller/dynamics/contact landmarks与PNG验证见学习包；产物只ignored tmp/s17_8a_T2/T1。
无GUI/UR5e/ROS/硬件验证，Learning三项完成，Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S17.8b见下文。


## S17.8b — Surface following / UR5e

[完整学习包](../../docs/20_8b_surface_following_ur5e.md) / [代码](ur5e_surface_following.py)。
裸UR5e固定sphere probe、水平无摩擦plane；初始DLS IK与actual torque执行分开。
复用surface_following/normal_force/pregrasp_motion/gravity_compensation的依赖链，
需requirements已声明mujoco/numpy/matplotlib/mujoco-menagerie，无新依赖。

```bash
python examples/18_compliant_control/ur5e_surface_following.py
python examples/18_compliant_control/ur5e_surface_following.py --leg-duration 1
```

先按上文核验conda/interpreter。Modify只改每段2→1s，30mm路径/2N目标不变。
2026-10-09 Zero：两命令各nominal/loss两case exit0；T2 nominal task True，T1预期task False：
切向max error.352200/1.462038mm，normal max error.048350/.281013N，contact100%/无饱和。
T1参考走完且末尾恢复2N，不等于中途扫描合格；loss两组均6s锁存actual位置/姿态并退出，physics继续至12s。
actual速度与caps、独立FK/J/contact/step replay、数值Jacobian验证见学习包；产物只ignored tmp/s17_8b_*。
仅probe-plane碰撞开启，不提供arm避碰；无GUI/ROS/真实force sensor/硬件验证。
Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；下一小任务S17.9等待明确请求。
