# S19.1 — Observation / Action / Episode Contract

## Problem → Why → Intuition

问题：怎样把expert运行变成因果正确的机器人示范？
先统一“看到什么、发出什么、何时看到结果”，否则再小的网络也会学习错标签。
本课复用Stage17无接触XY装置；primary concept是episode transition，不重复阻抗基础。
这是manipulation policy的数据起点，当前只做reach，不宣称完成物体操作或BC。

## Core Concepts

| 字段 | Shape / meaning / unit / frame |
| --- | --- |
| observation o_t | (6,)=[x,y,vx,vy,gx,gy]；世界位置m、实际速度m/s、commanded goal m |
| action a_t | (2,)世界XY绝对位置reference m；不是delta、实际qpos或motor力 |
| next observation | 执行动作20个1ms physics step后测得o_(t+1) |
| episode | T=100 actions、T+1=101 observations、2s；独立MjData/reset seed |
| terminated / truncated | (100,)布尔；本课无提前终止，最后truncated=True，terminated全False |
| success | 末10个观测全部位置error<1mm且speed<2mm/s；独立于时间截断 |

这两条slide恰与world XY对应，qpos/qvel可以直接作为工具位置/速度；不能把这个简化推广到UR5e。
50Hz发reference，1kHz重新测量并做阻抗反馈。每个episode保存NPZ，manifest保存版本、单位、频率、seed与指标。
示范记录了expert policy的动作；未来BC要预测同一种reference，由相同低层controller执行。
这些3条episode只是合同样本；本课不划分train/val、不拟合normalizer、不训练policy。

## Mathematics → Math-to-Code

专家：a_t=x_t+clip(g−x_t,−0.005,+0.005)（每个坐标，m）。
低层：F=K(a_t−x)−Dv，K=400N/m；M=diag(2,1)kg，D_i=2√(M_iK)N·s/m。
每1ms更新ctrl=F/gear；actual generalized force按motor限幅到±20N。
position target是ZOH，desired velocity=0；没有把reference jump微分成速度。
transition：(o_t,a_t) → 20×mj_step → o_(t+1)。
`observe`复制状态，避免MuJoCo数组视图让历史记录随当前状态变。
`run_episode`在执行前生成action；`advance`只用mj_step改变执行qpos/qvel。

API：`MjModel.from_xml_string(XML)`输入XML并返回编译模型；`MjData(model)`返回独立可变状态。
`mj_forward(model,data)`原地更新派生量/动力学buffer，不推进time；本课用于校验实际motor输出和加速度。
`mj_step(model,data)`原地计算与积分，推进1ms，无新状态返回值。
本课复用cartesian_spring XML/DT/GEAR/CAP；其helper导入matplotlib，因此执行依赖
requirements已有mujoco/numpy/matplotlib，无新增安装、Torch、Menagerie或ROS依赖。

## Minimal Experiment

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/episode_contract.py
```

看`tmp/s19_1_goal0.01/manifest.json`和三个NPZ。
每个NPZ的`observations[:-1]`与actions对应，`observations[1:]`是结果，不能拿未来结果作为policy输入。
模型不含碰撞/重力；不需要viewer。输出只在ignored tmp，不提交训练数据。

## Expected / Actual Result → Explanation

预期expert恢复到goal、末帧保留、动作重放一致；不是training-loss实验。
2026-10-10 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco环境所属
Python3.12.14；MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2 metadata/import路径核验。
默认与`--goal-x -.01`均exit0，每命令3episode×2000physics步，额外等量fresh-data replay。
默认3/3、Modify3/3达标；默认tail最大error2.179e−9m、speed2.764e−8m/s；
反向最大error8.854e−9m、speed1.122e−7m/s。两命令replay max error=0。
独立重载六NPZ的schema/clock/终态标记与fresh-data replay通过；CLI nan/越界goal拒绝exit2。
Python3.11语法解析通过，但未在3.11实际执行。
这只是固定3seed/短horizon expert的结果，不证明泛化。没有GUI或learned-policy rollout。
重放是open-loop saved-action检查；原始expert每50Hz使用最新observation，才是闭环expert运行。

## Failure Cases

- 用o_(t+1)预测a_t造成未来信息泄漏；只保存100个observation会丢最后next state。
- 把absolute action当delta积分会持续漂移；把reference直接赋qpos会绕过真实动力学。
- 时间截断与成功是不同字段；最后truncated不能解释成机器人失败或任务成功。
- 本课5mm界是reference相对当前位置的逐轴界，不是actual速度/安全保证。
- 下一阶段随机按row split会把同episode相邻数据泄漏；normalization只用train。
- Offline loss只看expert访问状态；learned rollout改变状态分布，误差可能累积。

## Robotics Context / Interview Capsule

应用：classical controller蒸馏、遥操作示范、contact-aware policy的数据入口。
30秒：示范是(o_t,a_t,o_(t+1))的时间序列；action单位与执行周期必须一致，数据正确才谈BC。
2分钟：说明6维状态、2维reference、50Hz/1kHz分层、复制数组和T+1 observation；
解释terminal/truncation/success分离，expert闭环与saved replay的区别，最后指出离线loss不能替代rollout。
Must Remember：先观测→expert动作→物理执行→再观测；不把ctrl、reference和实际状态混为一谈。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)；助手实验不算学习者Run。
Run：执行默认命令，核对NPZ shapes、truncation末项和manifest成功分母。
Modify：先预测只将goal_x从+10mm改为−10mm会改变哪些字段，再运行：

```bash
python examples/20_learning_from_demonstrations/episode_contract.py --goal-x -.01
```

比较同seed的初态不变、goal/action/轨迹改变，解释为何success仍需由实际状态验收。
Explain（五问）：
1. 为什么100个action需要101个observation？训练应配哪两个切片？
2. action、ctrl、qpos各是什么单位与物理意义？
3. 为什么50Hz发动作仍需要1kHz反馈，reference保持是否意味着状态停止？
4. 最后truncated=True而success=True是否矛盾？
5. fresh-data replay一致证明了什么？为何不证明未来BC闭环或泛化成功？

下一小任务S19.2：多episode expert demonstration/dataset generation。等待明确请求；本轮STOP。
