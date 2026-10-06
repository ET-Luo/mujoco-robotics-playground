# S14.4 — 手写 RRT-Connect

[代码](../examples/15_motion_planning/rrt_connect.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。前置：[S14.3](17_3_rrt.md)。

## Problem → Why

单树RRT从start逐步接近goal，可能花许多扩展才能到达另一端。
本课从start与goal各长一棵树，一侧探索后，让另一侧连续朝新节点靠拢。
只学习双树extend/connect和拼接；不做平滑、时间化、UR5e或真实执行。
算法依据：[Kuffner / LaValle, RRT-Connect 原论文](https://lavalle.pl/papers/KufLav00.pdf)。

## Intuition → Core Concepts

两组探路者从两端出发。一组先探索新方向，另一组沿相同目标连续迈步，
直到到达、遇阻或预算停止。未会合则交换探索角色。
CONNECT不是一次任意长度的直连，也不是忽略中间碰撞；它是反复EXTEND。

| EXTEND状态 | 含义 | CONNECT动作 |
| --- | --- | --- |
| TRAPPED | 最近节点到下一候选的边被拒绝 | 停止，下一轮换角色 |
| ADVANCED | 已插入一个有效新节点，但还没到目标 | 继续向同一目标EXTEND |
| REACHED | 到达目标，或已有完全相同节点 | 停止，已连接 |
| budget状态 | node/attempt/time限制触发 | 整次搜索停止，保留两棵部分树 |

两个根的身份保持固定：tree0=start，tree1=goal；只有active=0/1切换。
这让路径方向可以明确处理，不会因为交换角色而把输出颠倒。

## Mathematics：形状、单位、frame

沿用S14.2的二维XY slide模型，q shape `(2,)`，world xy米；joint bounds=[−1,1]²，
C-obstacle半径45mm。两个树各有nodes `(N_t,2)` 与parents `(N_t,)` 整数索引。
η=0.15 m限制一次EXTEND；h=0.02 m控制该边的碰撞查询间距，仍用Euclidean metric。

```text
q_rand ~ Uniform([-1,1]²)
(status, i) = EXTEND(T_active, q_rand)
如果不是TRAPPED：
    (status, j) = CONNECT(T_other, T_active.nodes[i])
    如果REACHED：两树meeting相同，回溯并拼接
否则或CONNECT遇阻：active = 1-active
```

EXTEND：nearest→限长steer→整段edge check→插入并记parent。
当目标距离≤η时直接复制target，避免浮点算式使两树会合点产生微小差异。
已有完全相同节点返回REACHED；它已有root连通路径，不需要再插入重复节点。

设P_s是start root→meeting，P_g是goal root→meeting：

```text
P = P_s + reverse(P_g)[1:]
```

reverse后才是meeting→goal，去掉第二个meeting避免重复配置。
两条已检查树链连接后构成start→goal路径。普通RRT-Connect找到首条路径即停止，无最优性保证。

## Math-to-Code / APIs

`plan_connect`内有三个主要步骤：`extend`、`connect`、`result`。
保留简单list/NumPy和显式循环；复用S14.3的steer/reconstruct及S14.2的碰撞查询。

- model只读共享，snapshot不改；独立MjData赋候选qpos并`mj_forward`。
  `mj_forward`返回None，原地更新FK/contact/cache，不推进time，相关shape/API见前课。
- 初始start→goal直接连接仍检查完整边；若通过，两根之间的checked bridge就是path，
  `direct_connection=True`，不需在任一树重复插入goal。本课默认直接连接被拒绝。
- max_nodes计两棵树**总节点数**，包括两个root；单树基线只有一个root，也计总节点数。
- 每个实际EXTEND尝试计attempts，包括CONNECT内部的每一步。外层随机探索轮数另存outer_iterations。
  不把一次长CONNECT算成“一次扩展”来夸大性能优势。
- 每步CONNECT都检查预算；time是合作式检查，单次边查询可略超限，不是硬实时保证。
- REACHED必须由有效树边到达；距离靠近不算会合。
- 失败保存两棵树，path/meeting/chains/length为空；不把某个接近点作为完整路径。
- path_score是搜索后的统一复查与圆盘解析评分，不参与任一规划器搜索。
  采样分辨率漏检的限制仍存在，不能推广圆盘解析评分到UR5e。

## Minimal Experiment → Expected / Actual Result

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/rrt_connect.py
python examples/15_motion_planning/rrt_connect.py --max-attempts 20
```

必要依赖NumPy / official MuJoCo / Matplotlib，均已声明；无新增依赖。
2026-10-06，DESKTOP-781D67A；Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2。
3.11兼容目标未在3.11执行。seeds7…14共8个，两种planner每个seed各评价一次。

**比较口径**：相同场景、start/goal、η/h、node/attempt/time预算；
单树调用S14.3但goal_bias=0，双树同样均匀采样。与S14.3默认p=.15结果不可直接混比。
相同seed不是逐样本配对：单树取bias选择随机数，且两算法消费样本和扩展的节奏不同。
此处比较实现的规划器策略，不是严格控制所有随机输入的因果实验。

| 预算 | planner | success / total（含失败分母） | mean EXTEND attempts | mean configuration queries |
| --- | --- | --- | --- | --- |
| 默认500 nodes / 2000 attempts | RRT | 8/8 | 108.625 | 872.375 |
| 同上 | Connect | 8/8 | 14.125 | 206.875 |
| 20 attempts | RRT | 0/8 | 20 | 257.625 |
| 同上 | Connect | 8/8 | 14.125 | 206.875 |
| 12 total nodes（额外工程反例） | RRT | 0/8 | 11.125 | 181.875 |
| 同上 | Connect | 0/8 | 10.750 | 179.125 |

默认seed7：RRT尝试31次，Connect16次；这些是本课关闭goal bias的新结果。
所有搜索成功path的采样复查与圆盘解析评分均通过。
默认/20attempt/12nodes三命令均exit0，表示评价全部完成，**不表示每次搜索成功**。
区别于S14.3单次planner CLI：这里预算失败是正常评价结果；任一返回path解析不合格exit1，非法参数exit2。

三组48次运行保存JSON；独立验证全部树边sampled/exact、η上限、parent顺序、
meet相同、反转/去重/回溯、长度、同seed重跑、反向起终点、输入与三类预算检查通过。
清晰直连、同点、无效端点也验证。live qpos/qvel/time/xpos未被查询修改。
图展示首个seed：灰色start树、橙色goal树、蓝色path；默认图已目视检查。
产物仅ignored `tmp/s14_4_connect_*/results.json`与`comparison.png`。

## Explanation → Failure Cases

本场景CONNECT能连续向另一树的有效节点迈步，省去许多重新随机探索的外层轮次；
结果显示更少扩展与几何查询，但不能据8个seed/一个小障碍声称普遍成功率或速度优势。
没有稳定wall-time benchmark，也没有不同障碍分布/narrow passage的统计。

- 仅查会合点：中间CONNECT仍可能穿过障碍，每一步都须edge check。
- 忘记交换角色：可能长期偏向一侧；固定root身份与切换active应区分。
- 交换后拼错：必须按root身份重建start→goal，而非active→other。
- 重复meeting：拼接去掉第二个meeting，避免零长度path段。
- CONNECT无限循环：每步都检查预算；near==target直接REACHED，防止重复插点。
- 比较只数outer loops：一次CONNECT内可做许多扩展，需比较EXTEND attempts与query数。
- 失败被剔除：成功率分母必须包括全部8次，预算失败不能从分母删除。
- 最优/执行误解：双树更快连接不等于更短路径，也没有速度、加速度、tracking或持物验证。

## Robotics Context → Interview Capsule

双树策略用于固定起终点的机械臂几何查询；反向goal树假设配置连接可以反向使用。
本课静态几何与对称边有效性满足这个前提；非可逆动力学、单向动作或时间依赖障碍需另行处理。
真实UR5e的metric、limits、phase/held碰撞模型接入留到S14.7。

**30秒**：RRT-Connect从两端各长树，一侧EXTEND一次探索，另一侧CONNECT重复朝新点EXTEND。
每段限长且检查碰撞，遇阻交换角色；会合后start链加反转goal链，去掉重复meeting。
按实际扩展与查询计数比较，预算失败不证明无解。

**2分钟**：我保留树的root身份，active只是角色。EXTEND找nearest、steer、检查边，返回
TRAPPED/ADVANCED/REACHED；CONNECT对同一target反复调用直到停止。
每一步检查node/attempt/time预算，避免greedy loop绕过限制。会合点相同后，
沿两个parent回溯，固定方向拼接并去重。与单树比较统一η/h、预算和均匀采样，
但相同seed并不意味着相同样本序列。统计包含所有成功与失败；本场景减少了查询，
有限采样依旧不是连续通用证明，且输出仅geometric path。

## Must Remember → Run / Modify / Explain

必须记住：CONNECT=重复EXTEND；每步查边和预算；root身份固定；goal链反转；meet去重；计扩展而非仅外层轮数。

**Run**：默认命令，查看首个seed两棵树与path，以及8次统计分母。

**Modify**：预测将max_attempts收紧到20对两种planner的成功数和扩展数影响，再运行第二条命令。
结果只是本次固定seed集合的观察；不能把预算失败解释为无解。

**Explain（五问）**：
1. EXTEND与CONNECT分别做什么？TRAPPED/ADVANCED/REACHED意味着什么？
2. 为什么CONNECT每一步都须查边和预算，而不能只查最后目标？
3. 角色交换后如何保持start→goal方向？为何goal链反转且meeting只保留一次？
4. 为什么比较EXTEND attempts和queries，比只比较outer iterations更合理？
5. 本课8seed的成功率能支持什么结论？为什么相同seed不是相同样本序列，path也不是trajectory？

**My Verification**：Engineering已完成；本人Run/Modify/Explain待确认，状态只在根README维护。
