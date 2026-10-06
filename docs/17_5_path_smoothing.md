# S14.5 — Collision-Checked Path Shortcuts

[代码](../examples/15_motion_planning/path_smoothing.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。前置：[S14.4](17_4_rrt_connect.md)。

## Problem → Why

RRT-Connect找到的首条路径通常包含冗余转折。如何缩短路径，同时保留起终点和碰撞政策？
本课只处理几何折线：随机选两个现有顶点，检查能否用直连替换中间子链。
不做样条曲线、时间化或UR5e整合。

## Intuition → Core Concepts

沿绕行路线走时，有些拐弯可以直接跨过，有些捷径会穿过障碍。
删除中间顶点前，必须检查整条新边，不能只检查两个已合法端点。
“Path smoothing”在本课指shortcut simplification：折线更短、顶点更少，仍可能有尖角。
几何长度变小不保证更大安全间隙、速度连续或动态可执行。

## Mathematics：形状、单位、frame

沿用XY-slide模型：`P=[q0,…,qN−1]` shape `(N,2)`，每个q为world xy米；配置距离为Euclidean。

```text
L(P) = Σ ||q[k+1]-q[k]||₂   (m)
随机选 i<j
L_old = Σ(k=i…j−1) ||q[k+1]-q[k]||₂
L_direct = ||q[j]-q[i]||₂
若 j>i+1，L_old-L_direct>ε，且edge(q[i],q[j])通过：
    P_new = concatenate(P[:i+1], P[j:])
ε = 1e-12 m，避免浮点噪声导致无实质改善的替换
```

由三角不等式 `L_direct≤L_old`；接受严格缩短时，
`L(P_new)=L(P)-L_old+L_direct<L(P)`。
其余边不变，起点与终点保留。索引每次针对**当前**path重新选，不能复用旧path的索引。
这是只在现有顶点间选shortcut的版本，无法插入新切点寻找真正最短绕行路线。

## Math-to-Code / APIs

按 `path_length → validate_path → shortcut_path → exact_score → main` 阅读。

- `main`用固定planner seed7、h=.005 m生成新路径，无需S14.4保存产物。
  shortcut seed默认17，与planner随机源分离；更改shortcut预算不改变输入path。
- `validate_path`检查有限shape `(N,2)`，调用S14.2边检查验证每一段；单顶点path也检查配置。
  非法输入被拒绝，不能把shortcut算法当作修复碰撞输入的规划器。
- `shortcut_path`先copy输入，随机取i/j，非相邻且能严格缩短才调用edge checker。
  接受后删除中间顶点；被拒绝的候选不改变路径。history记录索引、端点、前后长度与边检查。
- 最后`validate_path`重检**所有**输出边，而不是只检查最后一次shortcut；预算为0时同样复查。
- 复用官方MuJoCo的`MjData(model)`独立状态与`mj_forward(model,query)`原地更新cache。
  qpos shape `(2,)`米，query无积分，live qpos/qvel/time/xpos不改；API输入/shape/frame见S14.2。
- `exact_score`是搜索后独立圆盘解析评分，未用于shortcut选择/接受。
  `final_validation.sampled_valid`与`final_exact.exact_valid`分别保存，避免隐去漏检。
- attempts是shortcut尝试上限，包含相邻/无收益选点；最多10000次，节点不足3时提前结束。
  edge checker保留单边10000段预算；超限显式拒绝。没有单独wall-time deadline。
- 默认保留碰撞采样语义，不额外偷偷加入clearance阈值。本课正clearance由独立解析结果确认，
  不能把任何有限h推广为通用连续无碰撞保证。

## Minimal Experiment → Expected / Actual Result

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/path_smoothing.py
python examples/15_motion_planning/path_smoothing.py --attempts 0
```

只需要已声明NumPy / official MuJoCo / Matplotlib，无新依赖。
2026-10-06，DESKTOP-781D67A；Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2。
3.11兼容目标未在3.11执行。输入固定RRT-Connect seed7，13顶点/1.774460133m。

| shortcut预算 / step | 实际尝试 / 接受 | 输出顶点 / 长度 | 最小解析clearance | exit |
| --- | --- | --- | --- | --- |
| 100 / .005 m默认 | 100 / 6 | 3 / 1.612792500 m | 28.191341 mm | 0 |
| 0 / .005 m对照 | 0 / 0 | 13 / 1.774460133 m | 6.011070 mm | 0 |
| 100 / .4 m失败反例 | 7 / 1 | 2 / 1.600000000 m | −45 mm | 1 |

默认长度减少约9.11%；0预算路径逐元素保持输入。默认/0预算输入与输出的每边采样复查和解析评分通过。
默认clearance增大是本次观察，不是shortcut保证。
粗间距反例误接受start→goal，最终用**同一粗间距**重检仍通过，解析评分拒绝。
说明最终重检能查输出一致性，却不能修复碰撞oracle的系统性漏检。

独立逐步history回放/前后长度、被接受边、端点/输入保留、同seed输出、最终所有边、
密集采样正距离、输入碰撞/越界/shape/step/seed/budget拒绝均通过。
单点、两点、含重复相邻顶点的有效输入也检查。CLI负attempts退出2。
产物仅ignored `tmp/s14_5_shortcut_*/results.json`与`paths.png`，默认图已目视检查。
灰虚线=input、蓝线=shortcut path、红虚线=不查碰撞的最短直连反例，红盘=C-obstacle。
无GUI、时间化、UR5e、动力学或真实执行验证；没有多seed统计或全局最优证明。

## Explanation → Failure Cases

- **盲目删点**：合法端点不保证直连合法；直接保留起终点会穿过圆盘。
- **换成样条**：曲线可能偏离已检查折线，必须重新检查整条曲线；本课未引入样条。
- **只看长度**：更短可能更贴近障碍，不能据路径变短认定更安全。
- **没重检输出**：错误切片、索引或模型变化可能造成无效连接，最后检查所有边。
- **重检仍漏检**：相同粗oracle会重复同一个错误；明确sampled vs exact字段。
- **随机预算误解**：更多尝试对同一确定性prefix只能维持或减少长度，但不保证每次都改善；
  不同shortcut seed也不保证结果谁更好。
- **忽略阶段**：本课静态点模型没有phase/held变化；真实操作不能shortcut跨过抓取/放置事件。
- **“平滑”误解**：最终仍是折线，拐角处方向变化；需要后续时间化/停点政策才能讨论运动。

## Robotics Context → Interview Capsule

机械臂规划后可用shortcut减少冗余关节绕行，但应保留joint bounds、碰撞政策与持物假设。
新的长边可能远长于RRT扩展η，必须按实际长度重新确定检查样本。
本课不限制shortcut长度为η；η是树增长参数，不是几何连接安全保证。

**30秒**：我随机选路径上的两个顶点，用完整碰撞检查验证直连；只有严格更短时才删除子链。
保持起终点、copy输入，记录每次接受，最后重检所有边。shortcut缩短折线，但不保证最优、
更大clearance或速度连续。

**2分钟**：路径length是各段Euclidean距离之和。三角不等式保证直连不长于子链，
但不保证避障，因此先检查新边再替换。随机索引针对当前path，接受必须有超过数值容差的收益。
输入先验证，最后每段复查，预算0也不跳过验证。独立圆盘scorer不参与决策，
保留粗分辨率误接受的证据。固定planner与shortcut不同seed便于比较预算；
这里只做vertex shortcut，仍有拐角，尚无时间、速度、加速度或执行结论。

## Must Remember → Run / Modify / Explain

必须记住：更短≠避障；查完整shortcut；起终点不变；最后重检全部边；折线简化≠动力学平滑。

**Run**：默认命令，查看图与history中6次接受，确认前后长度与每边复查。

**Modify**：先预测`--attempts 0`对输入/输出顶点、长度、碰撞检查和最终评分的影响，再运行对照。
预期不执行shortcut，但仍做输入/最终复查，不能把0预算解释为不验证路径。

**Explain（五问）**：
1. 为什么直连不长于原子链，但不一定合法？
2. 为什么shortcut要检查完整新边，且不能只看两个端点？
3. 接受后哪些顶点被删除，如何保证起终点不变？
4. 为什么最后重检所有边？同一粗oracle重检能否保证修复漏检？
5. 为什么本课输出仍不是速度连续的轨迹，也不保证最短或更大clearance？

**My Verification**：本人于2026-10-06明确确认实验与预测完成，五项Explain正确覆盖三角不等式与避障独立条件、完整新边检查、删除内部顶点并保留端点、最终全边复查与粗oracle限制、非最优/clearance及动态连续性边界，Learning Mastered；状态只在根README维护。

精度补充：本实现数组拼接使用np.concatenate，而非NumPy数组的加号（后者是逐元素相加）。更细最终采样提高分辨率但仍不构成一般连续安全证明；保证需要具有明确保守边界的方法或连续几何检查。几何折线本身没有速度，拐角处若不停点而保持非零速度才会发生速度方向突变；时间化可选择停点政策，见后续S14.6。
