# S18.11 — Disturbance / Grasp Stability Integration：全过程验证与全部 trials

Engineering Complete（2026-10-10）；Learning Run / Modify / Explain待本人验证，以[根README](../README.md)为准。
[代码](../examples/19_teleoperation_dexterous/grasp_stability.py)复用[三指夹持](21_9_multi_contact_grasp.md)，本课约0.5～2小时。

## Problem → Why → Intuition

S18.9说明指定负载下能否保持，S18.10a/b区分摩擦允许集合、closure和有限力能力。现在回到自由物体动力学：已经夹住以后，短时外力或力矩会不会引起位姿超限、接触丢失、滑落？如何报告失败而不只挑出成功的trial？

直觉：短时扰动不仅需要瞬时抵消力，还会积累线动量/角动量。接触和servo随后可能恢复物体，但中途发生的过大滑移或倾覆不能被末帧静止掩盖。同样，电机没饱和也不能证明接触保持。

本课是固定测试矩阵内的有限时间工程验收，**失败trial不等于数学上的系统不稳定**；通过也不证明长期稳定、三维force closure或真实设备安全。

## Core concepts / 模型与 trial 协议

原hand.xml与S18.9代码不改；复用其`build_model`、`object_contacts`与hand_fixture的named mapping。固定palm、三指六hinge、30 g自由圆柱，nq/nv/nu=13/12/6。position servo kp0.25、kv0.02、gear1、输出cap0.08 N·m；gravity=0，物体COM从2 s起受到向下0.1 N负载。它仍是object-only重量替代负载，不是全系统重力下搬运。

继承S18.9的condim3和默认pyramidal接触模型，没有改变参数来让所有trial通过。接触可以通过力臂产生物体力矩，但没有独立的接触自旋/滚动力矩。

三条件×八方向=**每次命令24个计划trial**：

| 条件 | 目的 |
|---|---|
| three_finger，μ0.7 | 正常摩擦，已建立夹持后的扰动 |
| low_friction，μ0.03 | 接触gate通过但负载后滑落的前置失败 |
| missing_finger，μ0.7 | f2始终目标0，但仍要求三指gate；夹持未建立 |

| pulse | scale1的world峰值 / COM施加 |
|---|---|
| baseline | 无脉冲，保留0.1 N基准负载 |
| Fx_plus / Fx_minus | ±0.2 N，world X |
| Fz_down | -0.2 N，world Z，叠加到基准负载 |
| Tz_plus / Tz_minus | ±0.002 N·m，world Z力矩 |
| Tx_plus / Tx_minus | ±0.002 N·m，world X倾覆力矩 |

没有扫描全部方向、Fy或Ty；这是指定矩阵，不是六维任意wrench证明。圆柱几何绕Z轴对称，但仍记录材料body的姿态/自旋，不能因外形看起来一样就忽略角速度。

协议：

1. CLOSE按S18.9 ramp；每指物体normal>0.15 N连续50个1 ms采样后HOLD，冻结当帧目标；2 s仍未建立则FAULT。
2. 2 s保存物体位置/姿态锚点，对已HOLD的物体加0.1 N负载。2～2.499 s的失败均记录。
3. 2.5 s仅当HOLD且此前没有失败才将trial标为ready；否则抑制脉冲，但保留失败trial与完整统计分母。
4. ready trial在2.5≤t<2.7 s施加半正弦脉冲；一旦armed，按预定波形完整执行，不因中途失败提前停止脉冲。
5. 所有trial都运行至4 s，共4000 physics步/4001记录，不因失败提前退出；末0.25 s另检查恢复速度。

baseline也检查ready，但其`pulse_delivered=false`；不能把“无脉冲基准”与“前置失败导致脉冲被抑制”混为一谈。

## Mathematics：frame / shape / unit

### COM wrench 脉冲与冲量

`xfrc_applied[body]`为world `[Fx,Fy,Fz,Tx,Ty,Tz]`，shape `(6,)`，前三项N，后三项N·m，作用点为body COM。

\[
w_p(t)=s\hat w\sin\!\left(\frac{\pi(t-2.5)}{0.2}\right),\quad 2.5\le t<2.7,
\]

其余时间为0；s为pulse-scale。重置整个xfrc数组再写当帧wrench，避免上帧脉冲残留。
连续时间每个分量的冲量为`(2*0.2/π)*s*w_hat`；力冲量单位N·s，力矩冲量单位N·m·s。1 ms离散和是近似积分，不能把N与N·m的六维向量直接求普通范数称为“扰动力大小”。

物体COM线动量满足`m Δv=∫(F_contact+F_applied)dt`；角动量受COM总力矩改变。施加在COM的平移力没有额外偏心力矩；接触力矩仍需统一参考点：

\[
T_{contact}=\sum_i(p_i-p_c)\times f_i.
\]

若未来把力作用点改到COM之外，必须额外考虑力臂，不能只复制本课xfrc赋值。

### 位姿、相对滑动与输出饱和

相对2 s锚点：

\[
d(t)=\|p(t)-p(2)\|_2,\qquad
\theta(t)=2\arccos\operatorname{clip}(|q(t)^Tq(2)|,0,1).
\]

位置在world、单位m；q为world body单位quaternion、wxyz；θ为相对姿态角，rad。绝对内积使q和-q表示同一姿态。不能把quaternion四个分量差当角度。

相对切向速度沿用S18.9：在同一接触点分别求物体/手指world点Jacobian，差乘qvel，再投影到接触切向平面。只对finger-object且normal>0.05 N统计最大值；它不是COM速度。

本课gear1 hinge servo的未裁剪请求为：

\[
\tau_{req}=K_p(q_d-q)-K_v\dot q,\qquad
\tau_{actual}=\operatorname{clip}(\tau_{req},-.08,.08)\ \mathrm{N\cdot m}.
\]

饱和计数为每帧`|tau_req|>0.08`的关节数；总计单位为joint-samples，区别于“发生过饱和的trial数”。限幅限制驱动输出，不直接限制接触力、物体位姿或能量。

### 有限窗口验收

所有失败条件首次出现的时间存入字典，之后不删除。接受需ready且无下列失败：

| 检查 | 本课阈值 / 区间 |
|---|---|
| 每指接触保持 | 每指物体normal>0.05 N，2～4 s全采样 |
| COM漂移 | <4 mm，2～4 s |
| 相对姿态角 | <5°，2～4 s |
| 有效finger contact切向滑动 | <10 mm/s，2～4 s |
| 非手指支撑 | normal<1e−8 N，2～4 s，保留palm碰撞 |
| 向下掉落事件 | z比2 s锚点低20 mm；与任意方向drift失败分开记录 |
| 恢复速度 | 3.75～4 s COM速度<2 mm/s、world角速度<0.05 rad/s |
| 建立夹持 | 2 s之前gate通过；失败仍算计划trial |

这些是预先设置的课堂验收阈值，不是物理稳定性定理。微滑允许存在；没有要求pose或速度严格为0。`tail_not_recovered`记录3.75 s作为窗口起点，不冒充实际首次速度越界时刻。

## Math-to-code / APIs

| 调用 | 输入 → 返回 / 原地更新 | 本课用途 |
|---|---|---|
| `build_model(profile)` | 条件名 → model与μ | S18.9原fixture，各trial独立模型/状态 |
| `MjData(model)` | model → 状态与计算缓冲 | 不继承上一个trial的动量或接触 |
| named joint/actuator地址 | name → state地址/ctrl id | freejoint与六servo分别解析 |
| `mj_forward` / `mj_step` | 原地刷新当前解 / 原地积分1 ms | CSV先记录当帧状态与力，再推进时间 |
| `object_contacts` | model/data/body → force、normal、COM moment、slip、支撑与具名pair | 只统计真实object contact，不用自接触代替 |
| `mj_objectVelocity(..., mjOBJ_BODY, body, out, 0)` | body id、world方向标志 → **原地写**`out (6,)` | **前三项角速度rad/s、后三项线速度m/s**；顺序不同于xfrc |
| `mj_jac(...,jp,jr,COM,body)` | world COM → 写`jp/jr (3,nv)` | `jp qvel`与`jr qvel`独立核验线/角速度顺序与frame |
| `jp.T F + jr.T T` | world COM wrench → generalized force `(nv,)` | 不把world力矩直接当freejoint旋转dof分量 |
| `data.actuator_force[aa]` | named actuator ids → 六servo实际力矩 | 对照请求与clip，记录饱和与峰值 |

`mj_objectVelocity`的rot:lin输出定义见[官方API](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#mj-objectvelocity)。本模型body origin与COM重合；不用`data.cvel`直接猜body world速度。

## Minimal experiment / Run

依赖requirements已有mujoco/numpy/matplotlib、本地hand_fixture与multi_contact_grasp，无新安装。仓库根目录、WSL2 Ubuntu24.04、conda mujoco：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/19_teleoperation_dexterous/grasp_stability.py
```

产物只在ignored `tmp/s18_11_scale1/`：24份4001×73 CSV（12位有效数字）、逐trial接触快照JSON、results.json、24×9 summary CSV、正常摩擦轨迹图和全部trial结果图。快照为2.499/2.6/2.8/4 s；不生成全帧接触JSONL。代码PASS表示协议/数值检查正确，不能解释为24个抓取trial都成功。

## Expected / Actual / Explanation

2026-10-10 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；conda mujoco Python3.12.14，MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API及helper实际路径核验，无安装。
默认与`--pulse-scale 4`均exit0，各24trial×4000步，合计48trial/192000physics步。CSV紧凑序列化单独检查，浮点舍入近零角度时用合理容差；没有用舍入后的值驱动仿真。

| 脉冲scale | 计划/完成 | ready | 实际有脉冲 | 全部通过 | 正常摩擦条件通过 |
|---|---:|---:|---:|---:|---:|
| 1 | 24/24 | 8 | 7 | 8/24 | 8/8 |
| 4 | 24/24 | 8 | 7 | 4/24 | 4/8 |

两个设置下，低摩擦8trial和missing8trial均不ready，不施脉冲且不从总分母剔除。低摩擦gate0.807 s通过后仍在基准负载下滑落，最大漂移24.795 mm、出现palm支撑；missing在2 s超时，之后仍有动力学运动。baseline ready但没有pulse，其存在说明基准负载本身的影响。

正常摩擦的全过程最大指标：

| pulse | scale1 漂移mm / 角度° | scale4 漂移mm / 角度° | scale4结果 |
|---|---:|---:|---|
| baseline | 1.104 / 0 | 1.104 / 0 | 通过 |
| Fx_plus | 1.128 / 1.099 | 15.340 / 32.406 | slip/contact/pose/tail失败 |
| Fx_minus | 1.128 / 1.120 | 8.875 / 24.666 | slip/contact/pose/tail失败 |
| Fz_down | 1.243 / 0 | 1.846 / 0 | 通过 |
| Tz_plus / minus | 1.104 / 0.198 | 1.104 / 0.794 | 均通过 |
| Tx_plus / minus | 1.107 / 0.633 | 1.430 / 5.342 | slip/contact/rotation失败 |

scale4/Fx_plus在2.538 s先超过滑动阈值，2.543 s接触丢失，之后姿态/漂移超限；max slip232.547 mm/s。
scale4/Tx的最大漂移仍小于4 mm，但角度超过5°，并有接触丢失与约44.141 mm/s滑动。末帧角度已回落也不删除全过程失败；不能只看位置或末帧判断稳定性。

48trial的饱和joint-samples均0；最大实际力矩分别0.046850/0.069297 N·m，都低于0.08。这验证了请求/实际输出关系与界，但**没有在本课动态trial触发或验证饱和后的响应**；强扰动仍失败，失败不能简单归因于电机限幅。

每帧速度API与点Jacobian一致；接触力/COM力矩投影对freejoint各约束分量的最大数值误差6.67e−16（平移分量N、旋转分量N·m分别解释），Newton线力平衡最大3.77e−8 N，以1e−6 N检查。
独立48份CSV检查pulse形状/窗口/抑制与4倍缩放、gate/冻结target、全部分母、位姿/失败时间锁存/tail/acceptance、servo law/cap、接触快照方向/COM moment/slip与离散线动量冲量。baseline当前轨迹与**2026-10-10已保存S18.9基准**位置/关节轨迹一致，未另重跑旧课。
CLI非法scale/输入guards/3.11语法解析、四PNG目视、links/git diff --check通过；执行环境实际Python3.12，不冒充3.11运行。没有GUI或真实物理设备测试。

## Failure cases / Robotics context

- 只统计成功建立夹持的trial，会把8/24报告成8/8，隐藏输入/摩擦条件的前置失败；两种分母可都报告，但必须注明。
- 未就绪时仍施扰动，会把掌面上已滑落物体的响应误当夹持抗扰；本课ready gate在脉冲前检查。
- 只看最终速度/接触：低摩擦在palm上静止，strong Tx也可能重新接触并回落；锁存首次违规保留失败证据。
- 位移小就宣布成功：倾覆、滑动、接触丢失有独立指标；pure torque是专门的反例。
- 只看驱动限幅：外部wrench、接触配置、摩擦和惯性仍能破坏保持，本次无饱和而有失败。

应用：抓取测试矩阵与回归验证、区分建立失败与保持失败、记录每指接触和刚体运动，把符号/约束正确推进到可观察的动态行为。这里的确定性矩阵通过数不是随机样本的成功概率估计；没有材料/姿态/传感误差分布或长期扰动覆盖。

## Interview capsule

**30秒：** 我在三指自由圆柱上做接触gate和有限时长wrench脉冲测试，全部计划trial计入分母。验收看全过程位置、姿态、接触、滑动、非手指支撑及恢复速度，失败时间锁存。默认8/24通过，4倍脉冲4/24；前置失败不发脉冲，正常摩擦中强横向力/倾覆失败但电机没有饱和。

**2分钟：** 解释xfrc的world COM力/力矩顺序和mj_objectVelocity相反的rot:lin顺序，用Jacobian核验。介绍半正弦脉冲、2 s基准锚点、2.5 s ready、4 s固定结束；区分线动量与角动量。给出normal8/8→4/8与full8/24→4/24，并用strong Tx位置小但角度/接触超限说明多指标必要性。最后说明有限窗口验收与正式稳定性/closure/真实设备证据不同。

## Must remember

wrench有作用点/frame/unit；xfrc是F:torque，objectVelocity是angular:linear；参考冻结不是物体冻结；pulse前检查ready；失败不丢弃、不因末帧恢复删除；全部分母与条件分母分开；未饱和不等于保持；有限trial通过不是一般稳定性证明。

## My Verification — Run / Modify / Explain

根README学习框保持未勾选。S18.11是Stage18最后一个工程任务，完成本人验证后再回顾整个阶段；不自动实现新阶段。

**Run：** 执行默认命令，查看24trial结果矩阵、ready/delivered/accepted区别和正常摩擦四条指标图，确认低摩擦/缺指失败仍在分母。

**Modify：** 先预测4倍脉冲对gate时间、通过数、位姿/接触/恢复速度、饱和计数与physics步数的影响，再运行：

```bash
python examples/19_teleoperation_dexterous/grasp_stability.py --pulse-scale 4
```

重点比较Fx与Tx；不要调整验收阈值或从summary删除未ready的trial。

**Explain：**

1. `xfrc_applied`与`mj_objectVelocity`的六分量顺序、frame和单位分别是什么？力矩为何统一到COM？
2. 为什么必须在施脉冲前检查ready？24个计划trial与8个ready trial的统计分别说明什么？
3. 为什么strong Tx位置漂移小仍失败？末帧恢复为什么不能删除已记录的失败？
4. 为什么所有电机都未饱和，强扰动仍可能破坏保持？冻结target为什么不等于冻结物体？
5. 为什么pulse-scale4不改变gate时间/physics步数，却改变通过数？这些trial为何不能证明任意三维wrench或长期动态稳定？
