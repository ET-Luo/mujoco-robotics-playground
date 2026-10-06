# S14.7a — UR5e Obstacle Planning and Tracking

[代码](../examples/15_motion_planning/ur5e_obstacle_planning.py) · [运行入口](../examples/15_motion_planning/README.md) ·
[唯一状态](../README.md#stage-14--motion-planning)。前置：[RRT-Connect](17_4_rrt_connect.md)、
[shortcut](17_5_path_smoothing.md)、[时间化](17_6_time_parameterization.md)。

## Problem → Why

把二维教学规划器接到真正六关节模型。IK得到合法终点并不能保证home→goal直连不撞；
即使参考路径通过碰撞检查，动力学跟踪仍可能偏离参考。
本课连接 IK endpoint→joint RRT-Connect→shortcut→cubic timing→position-actuator execution，
分别报告reference geometry与actual tracking。

## Intuition → Core Concepts

q现在是六个rad关节角。C-space中的直线一般不是工具在world空间中的直线。
规划器试姿态时只改查询副本，执行器只写ctrl并mj_step；两种“移动”有不同含义。
模型为官方Menagerie UR5e裸机械臂，无夹爪/物体/地面；固定球障碍用于隔离避障问题。
本课不推进S14.7b抓取、持物或phase策略。

障碍world center=[−.3344,.3975,.3638] m，radius=.05 m。
这是手工固定的教学场景，中心选在无障碍home→目标直连的中途工具附近；
运行期间没有根据候选路径移动障碍，也不是感知估计输入。
目标attachment_site world位置=[−.45,.20,.25] m、world rotation=diag(1,−1,−1)，
local工具+z朝world−z。目标直接定义于world，不能当作UR5e base frame坐标。

## Mathematics：shape、unit、frame

q∈R⁶ shape `(6,)` rad，qdot rad/s，qddot rad/s²；joint bounds取编译模型。
nearest距离 `||q_a−q_b||₂`（rad），steer η=.3 rad，粗边查询h=.04 rad。
不做角度wrap：这里使用有界关节坐标，线性插值必须遵守编译limits。

采样盒是home/goal各轴的min/max加减.7rad，再与joint limits相交，shape `(6,2)`。
这是**局部搜索域**，预算失败可能只表示这个域/随机输入未找到路径，不证明整个C-space无解。
所有endpoint仍用完整joint limit检查，不把局部采样盒当成机器人真实limits。

```text
configuration valid = compiled joint bounds
                      AND no generated penetrating/touching contacts
                      AND min(robot_collision_geom, obstacle) distance ≥ .015 m
edge valid = every sampled configuration valid
```

shortcut仍随机选择现有顶点、严格缩短才替换，但每条候选用h=.025rad；
最终全部输出边用h=.01rad复查，独立验证进一步用h=.005rad。
配时复用S14.6：每段v_max=.2 rad/s（各轴），a_max=.8 rad/s²（各轴），T向上到dt=.002s。
同段共享cubic时间律，waypoint速度0、C1一般非C2；保持几何线段，但不保证jerk。

执行与参考有不同门限：actual不得有生成contact或joint-limit越界，实际障碍距离须≥.005m；
max |q_command−q_actual|≤.05rad，final工具position error≤.003m / rotation error≤.02rad。
这些是本课明确教学policy，不是硬件安全规范或通用碰撞保证。

## Math-to-Code / APIs

- `build_scene`：`MjSpec.from_file(path)`加载官方MJCF，`worldbody.add_geom`添加固定球，
  `compile()`返回MjModel。本课无需新依赖或其他机器人框架。
- `mj_resetDataKeyframe(model,data,key_id)`返回None，原地初始化home状态，query与execution分别初始化。
  execution只初始化一次；IK和规划data不会复制回execution。
- `solve_pregrasp_ik`复用P0 bounded DLS，输入world target pose/关节地址/初始q，
  返回q/更新次数/残差，并原地改私有IK data。80update失败不代表目标全局不可达。
- `joint_addresses`显式读取qpos/DOF地址；不假设名字顺序等于actuator顺序。
- `geometry_queries`每次新建MjData，copy snapshot qpos、设置六个候选值、`mj_forward`。
  后者返回None，原地刷新FK/contact/dynamics，不推进时间。
- `mj_geomDistance(model,data,g1,g2,.1,None)`返回signed surface distance m，
  分离为正、穿透为负，结果上截断于distmax=.1m；None表示不请求world最近点对。
  这是直接指定geom pair的距离查询，不依靠普通contact过滤来发现近邻。
  结果为数值几何查询，不是任意模型的严格连续证明。
  参见[官方API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-geomdistance)
  和[官方碰撞距离语义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-distance)。
- `plan_connect`新增可选bounds/configuration_checker/edge_checker，默认二维行为保留，
  六维任务显式提供callbacks；算法主循环/parent/connect未换成黑盒。
- `time_path`从固定2维改为任意D，limit shape也必须为 `(D,)`；原二维输出逐元素回归通过。
- `shorten`是本课直接的六维vertex shortcut循环，保留长度比较、edge check与切片，未使用二维特例scorer。

**编译碰撞模型审计**：30 geoms，其中robot active collision geoms为8 capsules + 1 cylinder，
覆盖shoulder/upper-arm/forearm/wrist1/2/3；visual geoms不当作collision geoms。
模型nexclude=0，但标准同体/父子/contype过滤仍适用；base没有active collision geom。
self检查只针对模型会生成的contacts，不声称补齐全部真实机器人外形。
环境只有这个球，没有地面/其他障碍；裸臂实验不证明夹爪或payload安全。
独立球–capsule解析距离核对通过；cylinder与self检查仍依赖MuJoCo模型/API。

执行写六个position actuator的`ctrl`（rad），不是直接赋qpos。
`qfrc_applied[arm_dof]=qfrc_bias[arm_dof]`用模型bias作理想外力前馈（N·m），
包含重力/Coriolis/centrifugal项；这绕过真实硬件扭矩能力，不等于actuator限幅验证。
`mj_step`返回None，原地积分并推进time；随后`mj_forward`刷新积分后contacts/几何。
actual距离/contact在每个2ms积分后检查，不是continuous collision detection。
actual速度另作统计；没有宣称实际加速度/jerk满足参考limits。

## Minimal Experiment → Expected / Actual Result

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/15_motion_planning/ur5e_obstacle_planning.py
python examples/15_motion_planning/ur5e_obstacle_planning.py --velocity-scale 0.5
```

需已声明NumPy / official MuJoCo / Matplotlib / mujoco-menagerie；本课不导入OpenCV或RL。
2026-10-06，DESKTOP-781D67A：Python3.12.14 / NumPy2.5.3 / MuJoCo3.14.0 /
Matplotlib3.11.2 / Menagerie2026.9.1；无新依赖/安装。3.11兼容目标未执行，未运行GUI。

IK20更新，position residual=.000936651mm，rotation residual=1.850602e−7rad。
直接边首次拒绝index19、obstacle distance12.742418mm低于15mm参考policy；
直连midpoint另作查询，距离约−50.000382mm并生成接触，确认该连接确实进入障碍；
因此没有生成穿透contact的早期样本也不能忽略15mm参考margin。
seed7：RRT-Connect14nodes/25扩展尝试，path9→3顶点，5个shortcuts接受。
规划/shortcut/fine recheck通过；所有timed reference配置也通过，最小reference距离54.487516mm。

| case | trajectory / sim time | steps | max tracking | min actual distance | final position / rotation error |
| --- | --- | --- | --- | --- | --- |
| default v=.2rad/s | 8.488 / 10.488s | 5244 | .039322360rad | 54.483984mm | .024871mm / 3.728729e−5rad |
| velocity×.5 | 16.974 / 18.974s | 9487 | .019763381rad | 54.486458mm | .006224mm / 9.933493e−6rad |

两命令exit0，execution_passed，无import error/observed generated contacts。
actual peak velocity=.198597/.099815rad/s（当次观察）；1s initial + 1s final hold计入sim time。
首次工程尝试v=.4rad/s，max tracking=.077036911rad，最终pose虽小却未过.05rad门限；
把默认参考限速降到.2后通过，未放宽tracking门限。这个反例说明最终位置准确不代表沿途跟踪准确。
`--max-nodes 2`预期exit1/node_budget，保存partial planner结果，trajectory/execution为null，未执行。
CLI nan velocity scale exit2。planning失败不证明无解，不能拿未执行当安全执行。

产物ignored `tmp/s14_7a_ur5e_*/results.json`、`trajectory.npz`、`tracking.png`。
NPZ：参考time/q/qdot/qddot/path；trace每行 `[time, command6, actual_q6, actual_qvel6, capped_distance]`，
shape `(steps,20)`。参考time从0开始，actual含初始1s hold；不能把两者无偏移对齐。
图显示六轴tracking与实际障碍距离，默认图已目视检查；不是机器人camera图。
独立核对两套保存reference/trace、endpoint FK、clock、全部参考、h=.005更细边、
球–capsule解析距离、失败null、原二维planner及cubic输出回归通过。
默认同seed重跑确定性指标相同，wall time不作为性能benchmark；没有多seed鲁棒性结论。

## Explanation → Failure Cases

- IK收敛只解决末端约束，不检查中间连接；collision-free endpoints不是collision-free edge。
- 局部采样域可能漏解；time/node/attempt budget失败不是全局无解证明。
- shortcut长边需要更细复查；任何固定rad步长都不是连续robot几何安全证明。
- 模型过滤/缺失collision shapes影响self coverage；不能用本课裸臂通过证明gripper/held物安全。
- 相同cubic几何保证仅针对参考；actual tracking误差可能侵入障碍，必须逐步检查actual。
- 仅看最终pose：首版反例最终pose误差很小，但max tracking超标；两个指标不能互相替代。
- 更慢会减小当前servo lag，但不保证所有操作更稳，尤其持物接触任务，后续另评估。
- 理想bias外力不受硬件扭矩限制；这里没有真实驱动器能力/实际加速度上限验证。

## Robotics Context → Interview Capsule

这是motion planning与control接口的第一份真实六关节整合。Planner输出path，
post-processing输出参考，servo产生actual state，各层负责的成功判据不同。
S14.7b才进一步加入夹爪、持物变换和分阶段接触许可。

**30秒**：我用官方UR5e和私有IK生成终点，在有界六维joint box里手写RRT-Connect，
每个配置查self contacts与障碍距离；shortcut后细查边，按cubic双约束配时。
执行只写ctrl/mj_step并逐步检查实际碰撞、间距和tracking。规划通过不等于执行通过。

**2分钟**：世界目标pose经DLS给出q_goal，home直连不满足障碍margin。
RRT双树用Euclidean rad距离和bounded steer，callback提供六维几何谓词。
shortcut保持端点，最终更细的edge检查与每个时间参考检查独立于实际执行。
每段共享零端速cubic并满足参考v/a上限；执行数据只初始化一次，位置伺服加模型bias前馈。
沿途检测actual状态和最大tracking，最后评估pose误差。首版速度过快tracking失败，
降低参考速度后保持原门限通过。碰撞模型覆盖、离散分辨率与理想外力都限制结论。

## Must Remember → Run / Modify / Explain

必须记住：world target/frame；六维rad metric；compiled碰撞覆盖；规划query≠执行；沿途误差≠最终误差；参考限值≠硬件能力。

**Run**：默认命令，核对direct拒绝、planner success、reference检查与execution_passed分别报告。

**Modify**：先预测`--velocity-scale 0.5`对path、时间、tracking与实际clearance影响，再运行对照。
path不变是本次fixed seed观察；降低速度对tracking的改善需由数据验证，不能当作通用保证。

**Explain（五问）**：
1. IK endpoint、reference geometry与actual tracking三项分别证明什么？
2. 为什么target用world frame，而规划q是rad？为何局部采样box失败不证明无解？
3. 为什么shortcut后还要更细查边并检查timed references？有限步长仍有什么限制？
4. 为什么执行不能写candidate qpos？ctrl、mj_step、bias前馈各起什么作用？
5. 为什么首版最终pose准确仍失败？本课为何不证明gripper/held object或硬件执行安全？

**My Verification**：本人于2026-10-06明确确认实验与预测完成，五项Explain覆盖IK解与reference/actual分层、world pose与关节rad、局部IK及有限预算边界、完整采样检查、ctrl/mj_step/bias前馈、沿途与最终误差及未验证硬件/持物能力，Learning Mastered；状态只在根README维护。

精度补充：IK成功只说明找到一个满足容差的配置，不证明碰撞可达；本课规划还限制home/goal附近局部采样盒，盒外也可能存在解。参考几何和actual均为当前模型/采样点上的政策验证，不是连续安全证明；确定性网格更细不构成单调漏检概率保证。bias使用qfrc_applied理想广义外力接口，不受真实actuator扭矩能力约束。
