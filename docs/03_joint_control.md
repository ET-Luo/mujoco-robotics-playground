# What I Need to Understand

S2.1：读取 shoulder_pan 的 ctrl 单位、gear 与范围；只阅读参数，理解问答待本人完成。

# Key Concepts

2026-09-27 助手静态核对本地缓存 `universal_robots_ur5e-2a3464301404aba7/ur5e.xml`：

- 第 125 行：`<general class="size3" name="shoulder_pan" joint="shoulder_pan_joint"/>`。
  `size3` 继承 `ur5e` 默认类；参数不全写在执行器这一行。
- 第 10–11 行：固定 gain=2000、affine bias=`0 -2000 -400`，形成内置位置伺服语义。
  本步只辨认配置，不实现或调节控制器。
- XML 未覆盖 gear，默认 `1 0 0 0 0 0`；hinge transmission 只使用第一项。
  执行器长度坐标 l=gq（此处是角坐标），g=1 时目标与关节角度数值一致，ctrl 单位 rad。
  六个 gear 分量不是六个关节的齿轮比；也不代表真实 UR5e 减速器的机械齿比。
- ctrlrange=`[-6.2831, 6.2831]` rad，约 ±360°；关节 range=`[-6.28319, 6.28319]` rad。
  compiler 的 autolimits=true 配合已给出的范围启用相应限制；本模型没有关闭控制限幅。
  输入范围约束命令，关节范围约束实际关节运动；不能据此保证实际角度等于目标。
- forcerange=`[-150, 150]`：本例 unit-gear hinge 下对应执行器力矩限制，单位 N·m，与角度范围不同。

参数语义来源：[MuJoCo general actuator](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-general)。
角度以关节参考零位为基准，绕关节定义的轴取正负，不是末端世界朝向。

# Experiments

2026-09-27 仅阅读：用 `sed -n '1,48p'` 和 `rg -n 'shoulder_pan|actuator|gear|flag'`
核对缓存 XML 与 scene；用官方文档核对默认值。尚无本人预测或运动实验结果。
环境检查：WSL2 Ubuntu 24.04.5；`echo $CONDA_DEFAULT_ENV` 为 base，
`which python` 为 `/home/lucas/miniconda3/bin/python`，因此未运行 Python 或仿真。
后续执行前须先激活 mujoco 并重新检查。本次未验证编译后数组或 GUI。

# What I Learned

# Interview Questions

1. 为什么需要沿 class 查默认参数，而不能只读 shoulder_pan 那一行？
2. 此模型 ctrl=-0.2 表示什么？赋值后实际角度会立即变成 -0.2 吗？
3. gear 的六个数是否分别对应六个关节？第一项为 1 在此处意味着什么？
4. ctrlrange、joint range、forcerange 分别限制什么？
