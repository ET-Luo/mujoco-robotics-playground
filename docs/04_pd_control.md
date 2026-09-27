# What I Need to Understand

S3.1 已开始：单个转动关节的位置误差、速度误差、力矩及 PD 手算。
只解释与手算，不实现控制器，不修改现有 UR5e 执行器。

# Key Concepts

角度 q 相对关节参考零位，单位 rad；角速度 v 沿该关节正方向，单位 rad/s。
本步用局部小角度差，不讨论跨整圈的角度处理。
目标为 q_target、v_target，误差定义为目标减实际：

```text
e_q = q_target - q
e_v = v_target - v
tau = Kp * e_q + Kd * e_v
```

tau 是绕关节轴的控制力矩（N·m），不是角度或角速度。
正力矩沿关节规定正方向作用，不保证当前速度为正；运动还受惯性、重力和其他作用影响。
Kp 单位 N·m/rad，Kd 单位 N·m·s/rad；两项相加前都必须是力矩。
P 是 proportional（比例），D 是 derivative（微分），连续且一致的目标下速度误差是位置误差的导数。
本步固定目标 v_target=0，所以 D 项为 -Kd*v，起阻碍当前运动的阻尼作用。
P 项像把关节拉向目标的扭转弹簧；D 项像减缓运动的阻尼。
仅有 P 项时到达目标不代表已有速度归零；位置误差为零时，D 项仍可能非零。

现有 UR5e 的 ctrl 是目标角度，不能直接把本步算出的 N·m 填进去当作力矩。
后续 S3.2 才使用最小力矩模型手写 PD；此处增益只是手算数值，不是 UR5e 调参建议。
官方位置伺服中的位置反馈与速度阻尼参考：
[MuJoCo position actuator](https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator-position)。

# Experiments

2026-09-27 手算题（待本人作答，无运行结果）：

- q_target=0.5 rad，q=0.3 rad。
- v_target=0 rad/s，v=+0.1 rad/s。
- Kp=10 N·m/rad，Kd=2 N·m·s/rad。
- 求 e_q、e_v、P 项、D 项及总力矩 tau，并解释 D 项符号。

本轮仅阅读 README/控制器占位文件、核对官方说明并更新文档。
已检查 pwd/git status、保留已有修改；链接目标和 git diff --check 检查通过。
没有运行 Python、仿真或 GUI，没有新增依赖或控制代码。S3.1 保持未勾选。

# What I Learned

# Interview Questions

1. 为什么位置误差乘 Kp 后单位是力矩？
2. 固定目标下，关节正在正向运动时 D 项为什么为负？
3. 位置已经到目标、速度仍非零时，PD 输出一定为零吗？
4. 为什么不能把计算出的力矩直接填入当前 UR5e 的 ctrl？
