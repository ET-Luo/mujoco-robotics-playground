# S18.9 — Multi-contact grasp：从接触到自由物体保持

Engineering Complete（2026-10-10）；Learning Run / Modify / Explain等待本人验证，状态以[根README](../README.md)为准。
[代码](../examples/19_teleoperation_dexterous/multi_contact_grasp.py)复用[hand.xml](../examples/19_teleoperation_dexterous/hand.xml)和`hand_fixture.mapping`；本课约0.5～2小时。

## Problem → Why → Intuition

S18.8能说明“哪根手指接触了什么”，还不能说明“自由物体能否保持”。本课把六关节servo、接触力和点Jacobian整合到close→hold任务：先用对向两指，再用原120°布局三指夹住圆柱，施加向下负载，观察力分布、漂移与相对滑动。

直觉：手指的径向夹紧力可以相互抵消；保持物体需要额外的向上接触力承受向下负载。指尖有法向力、物体最后静止，都不足以判断抓取成功——物体可能已经滑到掌面，由掌面托住。

## Core concepts / 实验边界

- hand.xml原文件不改；副本添加自由圆柱：半径20 mm、半高25 mm、质量30 g，初始COM为world `[0,0,95]` mm。
- 两指case把f1移到f0对侧；f2保持张开。三指case保留原120°布局；因此比较同时改变布局和总接触载荷，不能归纳为“三指永远优于两指”。
- 固定palm、1 ms physics、gear1 position servos；gravity仍为0。2 s起在**物体COM**施加world向下0.1 / 0.3 N，不对手指施加重力。这是可控的物体负载实验，不是全系统重力下的搬运。
- 没有weld、物体位置控制或初始支架；只在初始化设置模型位姿，运行期间不写物体qpos。palm碰撞保留并单独统计。
- 接触为condim3，保留法向和滑动摩擦，不引入接触自旋/滚动力矩。关节目标冻结后servo仍重新响应实际状态，hold不等于关节完全静止或卸力。
- 本课按**物体geom**过滤contact；一般touch还可能计入自接触，不能代替物体身份检查。S18.8的touch测量边界继续适用。

| Case | 目标手指 / 摩擦 | 目的 |
|---|---|---|
| two_finger | 对向f0/f1；μ=0.7 | 最小夹持整合 |
| three_finger | 原三指；μ=0.7 | 比较载荷分布与漂移 |
| low_friction | 原三指；μ=0.03 | 通过接触gate后仍滑落的失败对照 |
| missing_finger | 要求三指，f2目标一直0；μ=0.7 | 不满足持续接触条件，close超时 |

missing_finger的张开手指仍可能被运动物体撞到；失败依据是**没有满足持续gate**，不是“永远没有f2 contact”。

## Mathematics：shape / unit / frame

自由关节增加7维qpos：world位置3 + quaternion4（wxyz）；增加6维qvel：平移3 + 旋转3。模型总`nq=13, nv=12, nu=6`。Quaternion的四个分量不是四个独立转角，不能把整段qpos差直接当qvel。

对每个active物体contact，用MuJoCo的contact-frame力`f_C ∈ R³`（N），frame三条坐标轴按行存放：

\[
f_W^{object}=sR_{CW}^{T}f_C,\qquad
s=+1\text{ if object is geom1},\ -1\text{ if object is geom0}.
\]

这里的geom0/geom1指Python `contact.geom[0/1]`，不是另造的物理正方向。每指对物体的合力是该指接触的**有方向向量和**；normal是非负法向标量和，不能把normal相加当world竖直支撑力。

统一以物体COM `p_c`（world，m）为参考：

\[
F_c=\sum_i f_i,\qquad
T_c=\sum_i(p_i-p_c)\times f_i,\qquad
m\ddot p_c=F_c+[0,0,-L]^T.
\]

`F_c`单位N，`T_c`单位N·m。condim3没有内禀接触力矩，但力臂仍能产生力矩。总和包含palm等非手指接触；每指载荷只包含该指与物体的接触。物体角度变化另外记录，**本课验收是有限时间的位置保持，不是任意方向姿态稳定性证明**。

接触处的相对切向速度：

\[
v_{rel}=(J_p^{object}(p_i)-J_p^{finger}(p_i))\dot q,
\qquad v_{slip}=\|R_{CW}[1:3,:]v_{rel}\|_2.
\]

两个点Jacobian均为world frame `(3,12)`，乘`qvel (12,)`得到m/s。含freejoint时各列单位随对应dof变化，不能把所有列都叫m/rad。只对finger contact且normal>0.05 N统计最大滑动速度；palm碰撞速度不会冒充指间滑动。

物体COM漂移为`||p(t)-p(2s)||`（m）。这是相对加载前的位置变化，不是相对初始95 mm的变化，也不是接触面的局部滑动速度。Quaternion角差为`2 acos(clip(abs(q(t)·q(2s)),0,1))`，单位rad。

## Math-to-code / MuJoCo APIs

| 表达式或API | 输入 → 输出/原地更新 | 本课用途 |
|---|---|---|
| `MjModel.from_xml_string` | MJCF文本 → 编译后的model | 在hand副本中加入自由物体 |
| `MjData(model)` | model → 新仿真状态/计算缓冲 | 每个case独立，不沿用上一case状态 |
| `jnt_qposadr / jnt_dofadr` | named joint id → 数组起点 | 六hinge与object_free分别解析；ctrl仍用named actuator id |
| `data.xfrc_applied[body]` | world `[Fx,Fy,Fz,Tx,Ty,Tz]`，N/N·m | 在指定body COM施力；body id不是joint id |
| `mj_forward(model,data)` | 原地刷新FK、接触求解、派生量；不返回新状态、不积分 | 记录当前状态及对应当前力 |
| `mj_contactForce(..., index, raw)` | 接触编号 → 原地写`raw (6,)` | 转world、决定物体侧符号 |
| `mj_jac(...,jp,None,point,body)` | world接触点/所属body → 原地写world `jp (3,nv)` | 求两物体在同一接触点的相对速度；不是tip site的J |
| `mj_step(model,data)` | 原地积分、推进time 1 ms | 自由物体和手指的真实动力学演化 |

`xfrc_applied`作用于body COM，freejoint位置与速度维度不同，见[官方Simulation说明](https://mujoco.readthedocs.io/en/latest/programming/simulation.html)；字段布局见[官方数据结构](https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html)。

状态机很小：CLOSE时按原`CLOSED=[.55,.65]×3`在1 s内线性增加目标；选定每指normal均>0.15 N连续50个1 ms采样后进入HOLD，冻结当帧目标。进入HOLD并不是验收通过。2 s仍在CLOSE则进入FAULT，冻结目标，不施加测试负载。HOLD是锁存状态；接触丢失不自动重夹，本课通过后验记录揭示失败。

加载区间为2～4 s。位置保持判据：确实加载、选定每指normal全程>0.05 N、没有非手指有效支撑（normal<1e−8 N）、最大COM漂移<4 mm。4 mm是预先选定的课堂比较标准，不是物理稳定性定理。FAULT冻结目标也不代表物理停止。

## Minimal experiment / Run

依赖requirements已声明的mujoco/numpy/matplotlib及本地hand_fixture；不需要ROS、RL或新安装。
从仓库根目录，先确认WSL2 Ubuntu24.04和conda环境：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/19_teleoperation_dexterous/multi_contact_grasp.py
```

输出到ignored `tmp/s18_9_load0.1/`：四份4001×53 CSV、逐物体接触JSONL、results.json、grasp_comparison.png。CSV为**当前状态、当前接触解**，之后才mj_step；每case实际4000步、4 s。程序PASS表示预期成功/失败模式与数值检查吻合，不能把每个case都读成抓取成功。

## Expected / Actual / Explanation

2026-10-10，DESKTOP-781D67A，WSL2 Ubuntu24.04.5；conda mujoco Python3.12.14，MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2，metadata/import/API/本地helper路径核验，无安装。
两个命令（默认和`--load .3`）均exit0。前三case的gate均为0.807 s，冻结目标每指`[.44385,.52455]` rad；missing_finger在2 s超时，未加载。

| 负载 | Case | 加载后最大漂移 | 加载后最大相对切向速度 | 位置保持判据 |
|---|---|---:|---:|---|
| 0.1 N | two_finger | 1.657 mm | 0.815 mm/s | 通过 |
| 0.1 N | three_finger | 1.104 mm | 0.543 mm/s | 通过 |
| 0.1 N | low_friction | 24.795 mm | 258.515 mm/s | 失败：滑落、palm支撑 |
| 0.3 N | two_finger | 4.971 mm | 2.444 mm/s | 失败：漂移超过4 mm，仍有指接触 |
| 0.3 N | three_finger | 3.314 mm | 1.629 mm/s | 通过 |
| 0.3 N | low_friction | 25.526 mm | 523.812 mm/s | 失败：滑落、palm支撑 |

0.1 N末帧：两指各normal约0.44962 N，径向相消、各向上约0.05 N；三指各normal约0.43690 N，径向相消、各向上约0.03333 N。两组总world接触力均约`[0,0,0.1]` N，但normal总和分别约0.89924/1.31070 N。normal是夹紧力，竖直分量才承载本次负载。

低摩擦末帧手指仍有normal约0.13691 N/指，甚至对物体向下施力；palm提供向上的补偿。因此**总world力平衡和末速度接近0仍可同时出现在失败case**。

高摩擦case也有小的持续滑动。本模型使用柔性/正则化接触与有限阻尼，不能因为净力约0就推断速度为0；Newton方程约束的是加速度。以上结论只针对2 s加载窗口。继续更久可能超过4 mm，这里没有长期静止证明。

数值验证：物体contact world合力对freejoint平移`qfrc_constraint`残差最大3.34e−16 N；Newton力平衡残差最大3.77e−8 N（missing case快速接触求解），按1e−6 N检查；六servo输出不超过0.08 N·m。
独立检查逐JSONL重建方向/每指normal/合力/COM力矩/切向速度，匹配CSV；核验连续50帧gate、冻结目标、负载时序、漂移起点和超时；离散冲量与物体动量变化残差最大3.36e−17 N·s。
两组共8个case验证，CLI非法值/输入guards/3.11语法解析通过；实际执行为Python3.12，不冒充3.11运行。两PNG目视检查，无GUI验证。

## Failure cases / Robotics context

1. 继续闭合到最终角度，可能把物体挤走；servo追踪成功不等于抓取成功。
2. 仅查contact数量：短暂碰撞、自接触、张开手指被撞均可能造成假阳性；本课限定物体身份、每指法向力与持续时间。
3. 仅查最终静止/总力平衡：掌面托住的低摩擦case会误通过；必须检查加载后的全过程与非手指支撑。
4. 固定target仍有漂移：servo不是力闭环，柔性接触也不是理想无限刚度无滑动约束；本课没有调参实现零滑移。
5. 只通过一个负载方向：不能证明force closure、任意力矩抵抗、转动保持、搬运或真实手的安全性。下一课才单独学习摩擦锥；本课不提前引入grasp matrix/优化器。

应用：抓取任务的phase gate与可复查日志、per-finger负载分配、识别“接触建立但物体保持失败”。真实项目还要覆盖扰动方向、物体形状/姿态、重力、持续时间和重复trials；这些不是本课已验证的结果。

## Interview capsule

**30秒：** 我把六关节手部servo、具名物体接触力和点Jacobian整合到两/三指夹持。close需要持续接触gate，hold冻结目标后施加物体负载；以COM漂移、接触保持和无掌面支撑判断有限时间保持，低摩擦对照说明有法向力/最终静止仍可能失败。

**2分钟：** 说明freejoint导致nq≠nv及named地址；接触frame按行存储、按geom侧决定力的符号；区分法向标量和world合力。对同一world点用两body Jacobian计算相对切向速度，以加载前COM为漂移起点。介绍0.807 s接触gate、2 s负载、4 mm有限窗口判据，给出0.3 N两指4.971 mm与三指3.314 mm对照。最后用低摩擦滑到palm的案例解释为何接触与净力平衡不能证明抓取稳定或force closure。

## Must remember

自由物体不能在hold阶段被偷偷焊接或覆盖qpos；contact gate是进入条件，验收是另一件事；normal≠竖直支撑力；slip看接触点相对速度，drift看物体位移；所有力矩先统一参考点；有限时间通过不等于长期/全方向稳定。

## My Verification — Run / Modify / Explain

根README的学习框保持未勾选，助手执行不代替本人学习。

**Run：** 执行默认命令，查看图、events与四case的`hold_accepted`，区分“程序PASS”与“case通过”。

**Modify：** 先预测把负载从0.1 N增至0.3 N后的gate时间、物体漂移、每指支撑分量和physics步数，再执行：

```bash
python examples/19_teleoperation_dexterous/multi_contact_grasp.py --load .3
```

比较两指/三指是否越过4 mm，以及低摩擦末帧为何仍可能有手指法向力。只改变负载，不调整gate或验收阈值。

**Explain：**

1. 为什么object freejoint增加7维qpos却只增加6维qvel？joint状态地址与actuator id为什么仍分别解析？
2. contact gate通过后，为什么仍要单独验证整个加载区间的保持？missing_finger为什么不施加测试负载？
3. 三指normal总和大于0.1 N，而world合力约0.1 N，二者分别表示什么？力矩为什么必须统一到物体COM？
4. object COM速度与contact相对切向速度有何不同？净力约0为什么仍可能持续漂移？
5. 低摩擦case最终静止、仍有每指法向力，为什么保持失败？0.3 N三指通过又为何不能证明任意方向force closure？
