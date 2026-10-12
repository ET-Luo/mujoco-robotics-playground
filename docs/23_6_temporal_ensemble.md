# S20.6 — Temporal ensembling / ablation

[状态唯一来源](../README.md#stage-20--act--temporal-policy) · [代码](../examples/21_act_temporal_policy/temporal_ensemble.py)
前置S20.5；本节只学习重叠chunk的时间对齐与加权执行，约0.5～2小时。

## Problem / Why / Intuition

E1每次只取最新chunk的第0槽，E5沿一个计划执行五步。能否利用多个计划对当前时刻的预测，减小reference跳动？
直觉是几份过去写下的计划同时给“现在”投票；先按执行时刻对齐，再平均。它们来自同一个模型、相近状态，误差可能相关；平均不保证更准确。

## Core Concepts / Mathematics

A_s∈R^(10×2)是时刻s读取实际观测后产生的chunk，单位m、world XY绝对位置reference。
A_s[k]指向时刻s+k。当前t只能收集s≤t<s+10的A_s[t−s]，不能平均不同时间的第0槽。
观测o∈R^6含位置[m]、速度[m/s]、goal[m]；网络内部归一化，推理固定prior z=0。

```text
S_t = {s | max(0,t−9) ≤ s ≤ t}
age_s = t−s                  # 无量纲，单位为50Hz reference tick
w_s = exp(−λ age_s) / Σ_j exp(−λ age_j)
a_t = Σ_s w_s A_s[t−s]       # (2,) world XY reference [m]
```

λ=0为均匀平均；λ>0偏重新预测；λ<0偏重旧预测。指数减去最大logit再计算，避免溢出。
本节默认λ=.5，同时报告±λ和0，不能只保留胜出结果。满窗口newer平均年龄约1.47tick、older约7.53tick：
这是预测年龄，不是通信延迟；所有候选执行时间都是当前t。旧计划仍可能反映已经过时的状态。

t=2的手算：A_0[2]=[1,2]，A_1[1]=[3,4]，A_2[0]=[0,0]，均匀平均为[4/3,2]。
零reference是合法动作，不能通过action!=0判断是否存在预测。
[ACT作者实现](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py)按旧→新排列候选并使用exp(−.01×row_index)，因此偏重旧预测。
本节用显式age消除方向歧义，并以时间索引判断有效性；负λ实现相同的旧预测优先方向。

## Math-to-Code / APIs

`combine(plans,anchors,tick,decay)`输入(N,H,2)米单位计划、(N,)anchor和当前tick，返回(2,)reference与(N,)归一化权重。
先筛选0≤age<H，再用`plans[valid,age[valid]]`沿“对角线”取动作；输出不会修改MuJoCo状态。
`ensemble_rollout`每个reference tick读取实际状态、调用S20.5 `predict`、融合，然后`env.advance`。
部署禁用posterior；训练标签不能传入推理。每个episode重新创建plan列表，避免跨episode污染。

MuJoCo接口沿用S20.5：`MjModel.from_xml_string`返回编译模型；`MjData(model)`返回独立可变状态；
`mj_forward(model,data)`原地刷新派生量且不推进时间；`mj_step`原地积分状态/time，无新状态返回。
只在reset写qpos/qvel。action为位置reference而非ctrl；低层F=K(a−q)−Dqdot，力限幅后通过gear驱动ctrl。
reference50Hz、物理/反馈1kHz不变。每次融合都重新预测，所以与E1同为100calls/2s，E5仅20calls。
日志`weights_by_age`为(T,10)，列0对应最新预测；无候选列权重0。保存全部chunk便于核对，运行候选只取最近10个。

## Minimal Experiment / Expected Result

依赖requirements已有NumPy/CPU Torch/MuJoCo/Matplotlib；复用tiny_act→attention_basics/cvae_latent/sequence_dataset/chunk_baseline→S19/controller。
无安装、新依赖或训练。prepared缺失按[Stage19 README](../examples/20_learning_from_demonstrations/README.md) S19.2→3重建；checkpoint缺失先运行[S20.5](23_5_tiny_act.md)。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/temporal_ensemble.py \
  --prepared tmp/s19_3/verified193 --checkpoint tmp/s20_5/default/act.pt
```

checkpoint替换为自己的S20.5实际输出目录。输出新ignored tmp/s20_6/run_*，含results.json、20个trial NPZ、ablation.png。
五种模式固定同weights、normalizer、四个validation reset、T100与旧success scorer；test数组不打开。
预期：uniform减小reference步间变化，但旧预测可能增加跟踪误差；平滑不代表成功，E5也可能更好。

## Actual Result / Explanation

2026-10-12，DESKTOP-781D67A，`--output tmp/s20_6/default`，固定S20.5 best20 checkpoint：

| 模式 | success | final mean error [mm] | reference step RMS mean [mm] | calls/trial |
| --- | --- | --- | --- | --- |
| E1 | 2/4 | .875 | .231 | 100 |
| E5 | 4/4 | .499 | .348 | 20 |
| uniform | 2/4 | 1.151 | .153 | 100 |
| newer λ=.5 | 3/4 | .652 | .187 | 100 |
| older λ=−.5 | 1/4 | 1.922 | .162 | 100 |

RMS定义sqrt(mean_t ||a_t−a_(t−1)||²)，再对四trial平均；未除dt，是reference步间变化而非速度/jerk。
E5变化较大却全部成功；uniform更平滑但误差更大。数据支持本场景的平滑/跟踪权衡，不能证明所有ACT都如此。
四reset、单训练seed、无扰动、无新test或GUI；不据此选最终算法。统一BC/ACT比较留S20.7。

补充同命令`--decay .01 --output tmp/s20_6/decay001`：newer/older均2/4，final mean 1.130/1.171mm，RMS .154/.153mm，接近uniform；E1/E5/uniform逐值不变。
默认20trial独立Euler重算40000步、actual-state重新预测、scorer与S20.5 E1/E5轨迹逐值一致；
手算/零reference/方向/过期与未来候选排除、13tick末段与3.11语法通过。实际Python3.12.14。

## Failure Cases / Robotics Context

- 未对齐就平均：混合不同未来时刻，reference物理意义错误。
- 用非零筛选：合法零动作消失；用显式时间有效性。
- 权重方向写反：row顺序与age方向相反；检查最旧/最新权重。
- 旧chunk状态过时或多模态：平均可能滞后，甚至指向无效/碰撞动作；XY结果不证明接触或避障安全。
- 融合后的actual trajectory不同：必须持续反馈，不能拿E1保存观测离线代替新闭环。

机器人应用：多次关节/末端reference预测之间减小切换；仍需要低层控制、约束与闭环验证。

## Interview Capsule

30秒：Temporal ensembling按执行时刻收集重叠chunk，归一化权重融合reference。它能减小跳动，但旧状态预测会带来偏差，不能把平滑等同成功。

2分钟：时刻s的第k槽对应s+k，当前t取A_s[t−s]，最多H个候选。显式时间mask保留零动作；age权重的正负决定偏新或偏旧。固定checkpoint比较E1/E5/uniform/两方向，保持reset、低层频率与success合同一致，同时报告误差、成功和reference变化。融合仍每tick预测，不自动节省计算；deployment仅实际观测和prior。少量validation结果不支持泛化结论。

## Must Remember / My Verification

同一执行时刻、显式有效性、明确方向、固定权重、平滑≠跟踪成功。
工程与学习状态以根README为准；助手运行不计本人Run。

- Run：运行上面命令，查看五模式结果与ablation.png。
- Modify：追加`--decay .01`，先预测±方向差距是否缩小，再对比reference RMS和误差。
- Explain：①t=7分别取s=2、s=6的哪个槽？②为什么零reference不能当缺失？③age正衰减偏新，作者row正衰减为何偏旧？④uniform更平滑为什么可能更差？⑤为何ensemble比E5多调用模型，而不是省计算？

本人Run/Modify/Explain均待确认；不自动推进S20.7。
