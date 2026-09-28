# PD control

Status: Stage 3 learning exercises completed; learner implemented PD and interpreted the gain comparisons.

Goal: Combine position error and velocity feedback to regulate a joint.

## S3.2：把手算放进循环

先读 [single_joint.xml](single_joint.xml)，再读 [main.py](main.py) 的循环。
一个质量为 1 kg 的杆绕局部 +z 轴转动（右手定则；本模型轴也与世界 +z 对齐），
零位时沿 +x 方向。关闭重力、关节弹簧、被动阻尼和摩擦，没有接触对象。
本步不设置关节限位或力矩限幅，以隔离 PD 的作用；这不是实际机器人的完整配置。

`motor` 直接连接 hinge，`gear=1`，所以这里 `data.ctrl[0]` 数值等于关节力矩（N·m）。
它不接收目标角度，也不会自己计算位置误差。与之前 UR5e 的位置伺服不同。
依据：[官方 motor 定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-motor)。

初值 q=0.3 rad、v=0；目标 q=0.5 rad、v=0；Kp=10 N·m/rad、Kd=2 N·m·s/rad。
**注意初速度改为零，不是前面手算中的 +0.1 rad/s。**

1. 本人已填写三行误差和力矩表达式；检查它们使用每步读取的 q、v。
2. 保持“读状态 → 算力矩 → 写 ctrl → mj_step”的顺序。
3. 运行前预测第一步 P、D、总力矩以及最初运动方向。
4. 运行后比较 t=0 与最终角度、速度、误差，向助手反馈输出。

在仓库根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
# 确认环境为 mujoco、解释器属于该环境后执行：
python examples/03_pd_control/main.py
```

脚本默认 headless，运行 1,000 步（2 s），每 250 步打印步进前的状态及即将施加的力矩。
最终行单独报告步进后的状态。当前已启用 PD；此前零占位基线使杆保持静止。
本人运行结果显示角度靠近目标、速度减小，见下方记录。

## 本步 API

| 调用/数组 | 输入、输出与作用 |
| --- | --- |
| `MjModel.from_xml_path(path)` | 输入 XML 路径，返回编译后的固定模型（结构、惯性、执行器等） |
| `MjData(model)` | 输入模型，返回该模型的可变状态及计算缓存 |
| `data.qpos / qvel / ctrl` | 本模型形状各为 `(1,)`，分别是 rad、rad/s、N·m；索引 0 仅因已知只有一个 hinge 和 motor |
| `mj_forward(model, data)` | 初始化改状态后原地更新派生量，不推进时间，返回 `None` |
| `mj_step(model, data)` | 根据模型和当前控制原地更新 data，包括角度、速度和时间，推进 0.002 s，返回 `None` |

## 工程验证（2026-09-28，助手）

WSL2 Ubuntu 24.04.5；激活并核验 mujoco 环境，Python 3.12.14、MuJoCo 3.13.0。
上述命令退出 0，无导入错误；nq=nv=nu=1。
零占位骨架运行后 time=2.000 s、q=0.300000 rad、v=0、误差=0.200000 rad，符合静止预测。
以上是此前零力矩基线。本人随后填写公式并提供运行截图，助手保留公式，仅清理
TODO 注释与旧提示，并在核验 mujoco 环境后复跑同一命令，退出 0、无导入错误。
首步力矩 2 N·m；t=0.5 s 时 q=0.487531 rad、v=0.074872 rad/s、力矩=-0.025058 N·m；
末态 q=0.499998 rad、v=0.000010 rad/s、误差=0.000002 rad，与截图打印值一致。
数值按六位小数打印；稀疏采样不能证明全程无超调。未测试 GUI、未增加依赖。

S3.1 learning exercises were completed on 2026-09-28: error definitions, torque
units, hand calculation, and D-term direction. See [the lesson note](../../docs/04_pd_control.md).
S3.2 completed on 2026-09-28: the learner explained braking and distinguished torque commands
from position targets. S3.3 predictions and comparison interpretation also passed on 2026-09-28.

## S3.3：固定 Kd，只改变 Kp

本人预测 Kp=10/40 的首步力矩分别为 2/8 N·m，B 组初始加速更强，预测正确。
`--kp` 当前只接受本课的 10 或 40（默认 10）；Kd 固定为 2，其余模型和初值相同。
每次启动重新创建模型和状态，从 q=0.3 rad、v=0 开始，不接着上一组末态运行。
核验上述 mujoco 环境后，在仓库根目录运行：

```bash
python examples/03_pd_control/main.py --kp 10 --plot logs/pd_kp10.png
python examples/03_pd_control/main.py --kp 40 --plot logs/pd_kp40.png
```

CSV 列为 time_s、target_rad、angle_rad、velocity_rad_s；含初始点和每步后的状态，
共 1,001 点。PNG 显示目标与角度，输出位于被 git 忽略的 logs/。
两组 CSV 还由助手叠加为 `logs/pd_kp_comparison.png`（临时分析生成，不是脚本自动输出）。

为明确比较指标，本课用“首次达到 0.48 rad 的采样时刻”衡量接近目标的速度：
0.48=0.3+90%×(0.5−0.3)。它不是稳定时间，也不是 10%～90% 上升时间。
超调量为 max(0, 采样最大角度−0.5)，仅针对本课正向目标变化。

2026-09-28 助手运行两条命令均退出 0，无导入错误；CSV 形状、有限值、同初值、
同目标、同时间轴、0.002 s 间隔与 2 s 末时检查通过，叠加图已目视检查。

| Kp (N·m/rad) | 首次达到 0.48 rad (s) | 采样最大角度 (rad) | 超调量 (rad) |
| --- | --- | --- | --- |
| 10 | 0.422 | 0.499998388 | 0（采样中未见越过目标） |
| 40 | 0.094 | 0.511036763（t=0.156 s） | 0.011036763 |

B 组超调约为目标变化量 0.2 rad 的 5.52%。结论限于当前模型、参数和 2 s 采样窗口，
不能概括成“更大 Kp 总是更好”或所有模型都必然超调。没有运行 GUI，没有新增依赖。
本人正确辨认 B 更快且发生超调，并解释不能仅凭更快就选 B；S3.3 已勾选。
助手再次说明超调是越过目标，到达目标时速度未必为零；初始加速度大本身不保证超调。
## S3.4：固定 Kp，只改变 Kd

固定 Kp=40 N·m/rad，拟比较 Kd=0.5 与 2 N·m·s/rad。两组同样从 q=0.3 rad、v=0
开始，目标 q=0.5 rad、v=0，dt=0.002 s。脚本增加 --kd（0.5/2，默认 2）与 --steps
（正整数，默认 1000）；原来的 S3.3 命令仍有效。
固定目标下 D=-Kd*v；同一非零速度下，较大 Kd 产生更强的反向力矩。
助手预测 Kd=0.5 制动较弱，更容易出现明显的往返振荡，实际曲线支持这一预测。

先手算两组在 v=+0.1 rad/s 时的 D 项，并判断初始 v=0 时首步总力矩是否相同。
后续采样比较超调、目标两侧往返、末段角度误差和速度；不将一个末态误差直接称为稳态误差。
本模型没有重力或恒定外力矩，理想静止平衡要求 q=q_target；Kd 在 v=0 时不提供力矩。
若 2 s 末段仍在振荡，需要延长观察；不能用最后一点恰好接近目标来证明已经稳定。

### 2026-09-28 预测与实验

本人正确算出 v=+0.1 时 D 项为 -0.05/-0.2 N·m。最初误判首步力矩不同，
助手解释初始速度为零、两组 D 均为零后，本人正确判断初始角加速度相同。
两组实际首步命令均为 8 N·m，第一步后的状态也相同。

核验 mujoco 环境后先运行：

```bash
python examples/03_pd_control/main.py --kp 40 --kd 0.5 --plot logs/pd_kd05.png
python examples/03_pd_control/main.py --kp 40 --kd 2 --plot logs/pd_kd2.png
```

A 组最后 0.5 s 的最大绝对误差约 1.967e-4 rad、最大绝对速度约 0.00634 rad/s，
仍有衰减运动，因此两组都从初态重新运行 4 s：

```bash
python examples/03_pd_control/main.py --kp 40 --kd 0.5 --steps 2000 --plot logs/pd_kd05_4s.png
python examples/03_pd_control/main.py --kp 40 --kd 2 --steps 2000 --plot logs/pd_kd2_4s.png
```

| 指标 | A：Kd=0.5 | B：Kd=2 |
| --- | --- | --- |
| 超调量 (rad) | 0.117450277 | 0.011036763 |
| 2 s 单点有符号误差 (rad) | -1.844e-5 | -4.441e-16 |
| 4 s 单点有符号误差 (rad) | 4.114e-10 | -4.441e-16 |
| 3.5～4 s 最大绝对误差 (rad) | 2.298e-8 | 4.441e-16 |
| 3.5～4 s 最大绝对速度 (rad/s) | 8.252e-7 | 8.882e-15 |

助手观察：A 明显往返振荡，B 超调较小、较快平息；两组末段误差与速度均接近零，
A 的 2 s 残差不应解释为永久稳态偏差。有限时间样本不能证明误差严格为零。
末段窗口为最后 0.5 s（短于该窗口的运行则取全部样本），指标不代表已定义稳定时间。

四条命令均退出 0、无导入错误。2/4 s CSV 分别为 1001/2001 点；有限值、初态、
固定目标、时间间隔、两组相同时间轴以及延长运行前 2 s 与旧数据一致性检查通过。
助手由 4 s CSV 生成 `logs/pd_kd_comparison.png`（临时分析图，展示前 1 s 的角度和速度），
已目视检查。未测试 GUI、未新增依赖。本人正确解释 A 振荡更明显、较大 Kd 制动更强，
并指出经过目标仍可能有残余振荡，S3.4 已勾选。Stage 3 学习练习完成，
不代表本人独立编写采样/绘图或完成 GUI 验证。下一小步为 S4.1 坐标系与刚体变换，尚未开始。
