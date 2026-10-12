# S22.5 — Dexterous/contact task：先证明专家可行

[根README状态](../README.md#stage-22--contact-aware-robot-learning) · [代码](../examples/23_contact_aware_learning/dexterous_feasibility.py) · [原专家与scorer](../examples/19_teleoperation_dexterous/grasp_stability.py)。预计阅读、运行、修改和解释需0.5～2小时。

## Problem → Why → Intuition

S22.4在球指端／平面上比较了策略；本课换成三指保持**自由圆柱物体**。
学习之前先回答：已有专家在明确的接触、载荷和扰动下能否完成任务？若专家本身做不到，更多模仿训练不保证解决问题。

直觉：三个手指持续接触不等于稳定抓持。法向压力产生摩擦能力，物体仍可能沿指面滑动、转动，甚至掉落。
本课复用S18.11，不重造手模型或控制器；新增学习任务合同、全部attempt保存和独立评分审计。
这是小三指有限窗口hold，不是任意物体的in-hand manipulation或dexterous重定位。

## Core Concepts

固定掌、三指各两hinge：6个关节、6个position servo；物体有freejoint。
模型`nq=13`、`nv=12`、`nu=6`：物体位置3+单位四元数4，而速度为线速度3+角速度3。
物体质量30g；全局gravity=0，2s后仅对已进入HOLD的物体施加world向下0.1N，不能称为完整地球重力实验。
无weld、物体servo或支撑弹簧；自由物体由实际`mj_step`积分。

专家有内部状态：CLOSE、HOLD、FAULT，连续接触计数和冻结参考。CLOSE以1s ramp弯指；
三指各自法向力>0.15N连续50个1ms采样后冻结目标进入HOLD；2s仍未达到则超时。
这是contact-gated scripted expert，不是持续调节法向力的恢复控制器。

2s记录姿态anchor；2.5s检查ready；仅ready时施加2.5～2.7s半正弦扰动。
三条件×八任务：正常μ=.7、低摩擦μ=.03、缺第三指；baseline、Fx±、Fz向下、Tz±、Tx±。
未ready不施加pulse，仍保留在总分母中。baseline零pulse可ready但`pulse_delivered=False`。

## Mathematics：意义、shape、unit、frame

- 每关节servo：`τ=clip(.25(q_ref−q)−.02 qdot, −.08, .08)`，q/rad，qdot/rad·s⁻¹，τ/N·m。目标范围每指prox[0,.9]、dist[0,1.2]rad；目标限幅不保证实际关节绝不越界。
- COM外载荷`w_W=[Fx,Fy,Fz,Tx,Ty,Tz]`，shape(6,)，前三项N、后三项N·m。pulse=`scale*w0*sin(π(t−2.5)/.2)`；默认F幅值.2N、T幅值.002N·m。
- world平衡：`m*a_COM=F_contact+F_applied`。力矩关于同一COM；`Q_contact=Jp.T@F+Jr.T@T`，J各(3,nv)，不能直接把world torque当freejoint角自由度力矩。
- 漂移`||p−p_anchor||`[m]；旋转角`2 acos(clip(|quat·quat_anchor|,0,1))`[rad]；绝对值使q与−q等价。
- slip：`||(contact.frame[1:] @ (J_object−J_finger) @ qvel)||`[m/s]，是接触处相对切向速度，区别于物体world线速度。

2s后每1ms检查并锁存：任一指normal≤.05N、漂移≥4mm、角度≥5°、slip≥10mm/s、非手指支撑≥1e−8N、向下相对anchor跌落≥20mm。
末250ms还须所有采样线速度<2mm/s、角速度<.05rad/s。
`accepted=ready and 无历史失败 and 末段速度恢复`；末帧恢复不擦除过去失败。
评分是事后判据，不会自动制动servo。drop是特定向下阈值，未drop仍可能失败。

## Math-to-Code / APIs

入口调用原`run_trial`；原源码保持MuJoCo调用可见：

| API | 输入与输出／原地更新 | 用途 |
| --- | --- | --- |
| `MjModel.from_xml_string(xml)` | XML→编译后的模型 | 自由物体、接触与servo参数 |
| `MjData(model)` | 模型→状态及计算buffer | 新trial隔离 |
| `mj_forward(model,data)` | 更新contact、forces、qacc等；无返回状态、时间不推进 | 当前state下求力 |
| `mj_step(model,data)` | 原地推进qpos/qvel/time，1ms | 实际动力学，非qpos复制 |
| `mj_contactForce(model,data,index,out6)` | 写入接触坐标系[F,T] | `frame.T`转换world，按geom次序决定物体受力符号 |
| `mj_jac(model,data,Jp,Jr,point_W,body_id)` | 写入(3,nv)world Jacobian | 速度及广义力核对 |
| `mj_objectVelocity(...,out6,0)` | 写入world[angular,linear] | 注意与xfrc的[force,torque]顺序不同 |

`mapping`按名称分别查qpos/dof/actuator id；XML刻意乱序，`ctrl[:6]`不是标准关节顺序。
`audit`从physics行重新计算失败布尔值、ready、accepted及pulse，区别于直接信任metric字典。

### 保存合同：用于研究可行性，尚非训练数据协议

每trial NPZ含`observation(4001,28)`、`action(4000,6)`、`time_s(4001,)`、完整`physics(4001,73)`。
obs顺序见results.json：q6、qdot6、物体world xyz3、quaternion(wxyz)4、world omega3、world v3、每指normal3。
action是**absolute position reference[rad]**，不是S22.3的actual-q锚定delta[m]，也不是torque。
`t[k]`行action在接下来1ms积分中执行；terminal行只有obs，满足N/N+1。
原专家1kHz决策，不能抽出每20行便称为50Hz部署证明；后续改变控制频率须重新验证。

这些obs包含仿真姿态、速度和contact truth proxy，未证明真实sensor可得。
没有未来pulse、action target、failure label作为输入，但**没有完整专家内部状态**，不能保证给定obs唯一确定专家动作。
HOLD参考依赖历史；未来学习须显式处理历史／状态，不能声称当前单帧合同是完整Markov状态。
另一个时序限制：原专家先写当前reference，再`mj_forward`求接触力，随后记录同一行。
q/qdot/pose仍是积分前状态，但normal是**当前action条件下的求解力**，不是前一已执行动作下的部署传感器读数。
这些审计obs不能直接当严格因果的BC输入；S22.6应先观测上一held ctrl下的力、再输出新动作并重采集。
N/N+1只证明转移shape，不证明所有观测通道都已满足部署时序。
完整physics供审计，不能整行当policy输入。尚未做split、normalizer、模型训练或独立test。
重复pulse的前缀高度相关；将来必须按reset/source group隔离，失败action不能自动标作成功专家标签。

## Minimal Experiment / Run

先读源码`main → run_trial → object_contacts → audit`，再运行：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/23_contact_aware_learning/dexterous_feasibility.py --output tmp/s22_5/my_run
```

output必须不存在，产物仅ignored tmp。`results.json`保留24个trial、schema、source SHA和分组分母；PNG只绘正常摩擦八个任务。
PASS指合同／物理／评分审计通过，不表示24次任务全成功。

## Expected / Actual / Explanation

预期：正常摩擦专家能保持基线；低摩擦或缺指可能未ready；强pulse可能超出固定HOLD能力。
2026-10-12 / DESKTOP-781D67A，本次两命令均exit0：

| 条件 | 默认ready / accepted | 四倍ready / accepted |
| --- | --- | --- |
| 正常三指 | 8/8、8/8 | 8/8、4/8 |
| 低摩擦 | 0/8、0/8 | 0/8、0/8 |
| 缺指 | 0/8、0/8 | 0/8、0/8 |
| 全部计划 | 8/24、8/24 | 8/24、4/24 |

四倍Fx±失败包含slip、contact loss、rotation、drift与tail；Tx±发生slip、contact loss、rotation，
末段角度回到5°以内仍不能洗掉历史失败。最大Newton残差3.761e−8N，原断言<1e−6N通过。
两组48份NPZ重新核对scorer、pulse、action/obs切片；所有2.5s前轨迹逐值一致，baseline与未ready整轨迹逐值一致。
六条代表（每组正常Fx+、低摩擦baseline、缺指baseline）用保存ctrl与world外载荷从初始状态重新`mj_step`，
4001点关节q/qdot和物体姿态逐值一致，max error=0；没有中间状态复制，不宣称全部接触通道独立回放。
两PNG已目视检查；nan CLI exit2；Python3.11语法检查通过，实际Python3.12.14。

环境：WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco环境所属Python，MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2，
metadata/真实imports/module paths/API核验，无安装。验证目录`tmp/s22_5/verified`、`strong_verified`；
本地独立核验命令`python tmp/s22_5/check_artifacts.py`（临时检查脚本不提交）。无GUI运行。

## Failure Cases / Robotics Context

观察三指都有力却发生slip的情形；摩擦、接触几何、载荷方向共同影响稳定性。
理想平面force closure证书不保证这个动态三维系统全过程稳定。
本实验用途是筛查示范专家与scorer可用性，为接触学习建立可信基线；专家可行不等于BC能学会，
单一固定reset的24个方向/profile不是24个独立随机seed，也不是domain generalization评价。
低摩擦profile是失败诊断，不完成S22.7 held-out friction/mass实验。
未验证GUI、真人、真实传感器、硬件、长期保持或主动重抓恢复。

## Interview Capsule

30秒：我先验证接触专家，再谈模仿学习。三指free object通过具名servo闭合，持续法向力gate进入HOLD；有限时间内检查漂移、旋转、相对滑动与drop，全部attempt保留，历史失败锁存。当前只有仿真proxy和1kHz专家，不宣称真实灵巧操作。

2分钟：解释为什么nq与nv不同、position reference与torque不同、world wrench与速度六维顺序不同；
说明50ms接触gate、2s anchor、2.5s ready与pulse、末250ms恢复；展示N/N+1与仿真真值的来源；
比较默认和强扰动，指出未ready不得从分母删除。最后说明后续必须分来源组、处理专家记忆及频率变化，闭环重新检验学习策略。

## Must Remember / Modify / Explain

Modify：先预测`--pulse-scale 4`影响的是ready还是扰动后稳定性，再运行新目录：

```bash
python examples/23_contact_aware_learning/dexterous_feasibility.py --pulse-scale 4 --output tmp/s22_5/my_strong
```

比较两组正常摩擦accepted数、各失败first time、slip峰值；检查baseline与所有2.5s前轨迹不变。

Explain（请用自己的话回答）：

1. 为什么模型nq=13而nv=12？6维action是什么单位、如何映射到ctrl，为什么不能复用C轨道delta动作解释？
2. 三指接触gate、pulse ready与accepted分别检查什么？未ready／零pulse baseline如何计入分母？
3. slip速度相对谁、在哪个frame计算？为何物体world速度小也不能断言无滑动？
4. 为什么末帧恢复、未drop或force closure不能证明本任务成功？本课专家到底有没有主动恢复反馈？
5. NPZ为何N+1观测／N动作？哪些通道是仿真proxy？为什么当前单帧obs和简单50Hz抽样还不能直接当完整部署合同？

My Verification：根README是状态唯一来源；学习Run/Modify/Explain仅在本人报告后勾选。下一小任务S22.6等待明确请求。

2026-10-12 本人确认实验与预测均完成，随后补齐第1–2项Explain；README Run/Modify/Explain全部勾选，Learning Mastered。
解释覆盖nq/nv、本课6维absolute position reference[rad]及具名actuator映射、servo与实际动力学；
准确区分三指>0.15N持续50ms的CLOSE→HOLD gate、2.5s时HOLD且此前无失败的ready、
以及全程无锁存失败并通过末250ms恢复的accepted。ready不预判扰动后成功，accepted不是文件验收。
时序精度：接触力在当前reference写入后mj_forward求解、积分前记录，属action条件反馈，非必然已积分后的反馈。
本次仅文档状态更新，未重跑Python/MuJoCo/GUI；runtime沿用2026-10-12工程验证。
物理范围补充：本课全局gravity=0；重力项在该模型为零，2s后对HOLD物体施加的world向下0.1N是显式外载荷，不是全系统重力。
