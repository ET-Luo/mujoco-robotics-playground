# S13.2 — Grasp Pose Generation / Endpoint IK Screening

约 1～2h。[代码](../examples/14_vision_manipulation/grasp_candidates.py) ·
[前课接口](16_vision_based_manipulation.md) · [状态唯一来源](../README.md#stage-13--vision-based-manipulation)。

## Problem → Why

感知输出 T_WO 是 object frame 的位姿，不是机器人应该到达的 gripper pose。
给定直立长方体的 full dimensions 和估计朝向，怎样生成不同开合方向的 top-down 候选，
退后得到 pre-grasp，并复用 IK 筛掉当前求解器无法满足的末端目标？
本课主要概念是 **object-relative grasp offset → world target → endpoint feasibility**。
不重新编写 IK，不执行抓取或完整 home→approach trajectory。

## Intuition → Core Concepts

先在物体自己的 frame 中描述“夹爪怎样放置”：+z_G 朝物体下方，x_G 为开合方向，
指腹中心对准物体中心。把这套相对关系乘到估计 object pose，就得到 world gripper pose。
再沿 −z_G 退后固定距离得到 pre-grasp。开口够宽后，才值得花时间求 IK。

限制为直立 yaw-only box，O 原点是几何中心，z_O=world +z。
倾斜物体明确拒绝，不将其旋转自动投影到水平面；这不是通用任意表面抓取算法。
物体尺寸来自已知 CAD/外部 geometry，不能从 T_WO 本身推断尺寸。

| 量 | shape / unit / meaning |
| --- | --- |
| T_WO_estimate | `(4,4)`；object→world，R 无量纲、t m；来自 S13.1 接口 |
| size_xyz_m | `(3,)`；full dx/dy/dz，m；不是 MJCF half sizes |
| alpha | 0/90/180/270 degree；gripper 相对 object 的 yaw 候选 |
| T_OG | `(4,4)`；gripper→object；G 为 attachment_site/gripper root |
| T_WG / T_WP | `(4,4)`；grasp / pre-grasp 的 G→world，方向相同 |
| width / opening / clearance | scalar m；闭合方向 box 宽度 / transit gap / 总余量 |
| finger slide s | scalar m，每根手指的 slide qpos；不是 aperture 本身 |
| q_pre / q_grasp | `(6,)`；UR5e arm joint rad，顺序见 ARM_JOINTS |
| endpoint residual | position m / orientation rad；不混为一个物理量 |

两对 180° 翻转对称的候选夹持几何相同，但交换手指身份与腕部朝向，可能有不同关节解或求解结果。
不是四个独立抓取成功试验，也没有候选 ranking/最优抓取选择。

## Mathematics：Object Offset → World Grasp → Pre-Grasp

夹爪 +z_G 朝下；基础旋转 diag(1,−1,−1) 的 determinant=+1，是 proper rotation。
定义指腹中心在 G 中的 p_GF=[0,0,0.035] m：

```text
R_OG(alpha) = Rz(alpha) diag(1,-1,-1)
t_OG = [0,0,0.035] m
T_WG = T_WO_estimate T_OG
p_WF = R_WG p_GF + t_WG = t_WO_estimate
```

因为 R_OG 将 +z_G 转为 −z_O，gripper root 比 object center 高 35 mm，
指腹中心沿 +z_G 向下 35 mm，刚好回到物体中心。
此 offset 是模型的 **root→pad** 几何约定，不是随意把 object center 当 TCP，也不是箱子高度的一半。

```text
approach_axis_W = R_WG[:,2]
R_WP = R_WG
t_WP = t_WG - d approach_axis_W
R_WG.T (t_WG-t_WP) = [0,0,d]       # pre→grasp 在 G 中沿 +z_G
```

对本 yaw-only 场景 axis_W=[0,0,−1]，所以 d=0.10 m 时 pre-grasp 位于 grasp 上方 100 mm。
默认估计 object center z=0.03 m，grasp root z=0.065 m、pre root z=0.165 m。
必须使用相对方向公式，不能一般性地把“退后”写成某个 world 轴加常数。

## Mathematics：Dimensions → Aperture

[教学夹爪 XML](../examples/12_pick_place/gripper.xml) 两指中心 x_G=±(0.015+s)，
每指 x half thickness=0.005 m，s∈[0,0.04] m，故内侧 gap：

```text
a(s) = 2*(0.015+s-0.005) = 0.020+2s m
min gap=0.020 m, max gap=0.100 m
width(alpha)=dx for 0/180; dy for 90/270
opening = width + 0.008 m        # total gap margin；每侧 4 mm
s_open = (opening-0.020)/2
s_nominal_contact = (width-0.020)/2
```

保留条件 width≥20 mm 且 opening≤100 mm，因此当前余量下最大可接受 box width 是 92 mm。
width<20 mm 时即使能张开也不能合到两侧，明确拒绝，不夹紧到负 slide。
最后的 contact slide 只作为几何说明，不发送 actuator ctrl，也不代表有接触力/摩擦稳定性。
本课 dz 为已知尺寸的一部分，但 center-clamp offset 由 pad geometry 决定，
没有做 height、finger length、palm/floor clearance 或任意形状的完整碰撞/适配检查。

## Math-to-Code / APIs

`generate_candidates(T_WO_estimate,size_xyz_m,d,clearance)` 返回四个 dict，
包含 T_OG/T_WG/T_WP、width/opening/slide、width status。
只读估计与尺寸，不读取 simulation known_object/truth/cache，不修改输入。
检查 finite/SE(3)、positive full sizes、d>0、clearance≥0 与 z_O=+z_W。

`screen_ik(model,candidate)`：width 拒绝则返回 width_rejected、q_pre/q_grasp=null。
其余每个候选创建独立 MjData、reset home、按候选 opening 设置 finger **qpos**，
然后复用 [solve_pregrasp_ik](../examples/12_pick_place/pregrasp_motion.py)：

```text
home q → IK(T_WP) → q_pre
q_pre  → IK(T_WG) → q_grasp
```

这是两个目标的几何求解顺序，不是机器人走过这段轨迹。
DLS 直接原地改变私有 data.qpos 并 mj_forward；其它候选不会继承上个候选的关节解。
pre 失败或 grasp 失败分别记录 reason；grasp 失败时已求出的 q_pre 可以保留，
但只有 status=ik_passed 的候选可算通过完整端点筛选。

复用 solver 的限制：最多 80 updates、每次 max joint step=0.05 rad、damping=0.01；
position tolerance<1e-4 m、orientation tolerance<1e-3 rad，并拒绝越 joint limit 的更新。
它把 m/rad 直接堆叠用于 DLS，是既有教学 scaling，不是加权物理最优指标。
π 附近 orientation error 会拒绝，solver 失败可能是 seed/budget/local solver 的限制，
**不能宣称不存在任何 IK 解**。

`mujoco.MjData(model)` 创建 model 匹配的动态 state/cache。
`mujoco.mj_resetDataKeyframe(model,data,key_id)` 原地重置为 home keyframe；
随后写入 finger qpos，`mujoco.mj_forward` 原地刷新 FK/cache，返回 None、time 不推进。
`joint_addresses` 返回 arm qpos indices 和 DOF indices；两者不能假定永远相同。
`mujoco.mj_jacSite` 输入 model/data/site_id，填充 `(3,nv)` jacp/jacr，返回 None；
提取六 arm DOF 列得到 `(6,6)`，平移块 m/rad、旋转块 rad/rad，方向为 world axes。
可核对 [MuJoCo 官方 Jacobian 文档](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite)。
`site_pose` 返回 world position/rotation 的 copy；端点 FK 再核验返回 q 的 residual/limits。

Model 复用 build_model（UR5e+教学夹爪）；旧 builder 带 known_object 固定 fixture，
本课不读它的 pose、不改变它去匹配估计、不把 contacts 当筛选依据。
图中的 box 是输入尺寸/估计生成的几何投影，不是 MuJoCo fixture 的观察结果。
模型仅提供 robot/gripper kinematics；不是视觉场景或 grasp simulation。

## Minimal Experiment / Run → Modify

从仓库根目录运行：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/14_vision_manipulation/grasp_candidates.py
python examples/14_vision_manipulation/grasp_candidates.py --size-x-m 0.12
# 可选：整个 candidate set 无可用解。
python examples/14_vision_manipulation/grasp_candidates.py --size-x-m 0.12 --size-y-m 0.12
```

Synthetic producer 给 T_CO，经过 S13.1 map 得 T_WO：center=[−0.45,0.20,0.03] m、yaw=20°。
不是 PnP 或 truth-read planning；不依赖 ignored 前课产物，无需另跑 perception_pose.py。
Default full dimensions=[0.04,0.03,0.06] m、clearance=0.008 m、d=0.10 m。
可选 CLI `--object-yaw-deg`、`--pregrasp-distance-m`；改变 orientation/height 可能改变局部 IK 结果。

输出 ignored `tmp/s13_2_grasp_x*_y*_yaw*_d*/`：summary.json、candidates.csv、candidates.npz、candidates.png。
JSON 记录每个 candidate 的几何与筛选阶段、q/residual/update 数；NPZ 仅保存实际求出的 q。
PNG 为 box XY footprint 与 candidate x_G opening axes，线代表 gap、圆点标记端点而非实体 fingers。
为避免 180° 对称线重合，显示位置有毫米偏移；真实候选的中心相同，不按图示偏移生成抓取。

**Run**：查看四组 T_OG/T_WG/T_WP、opening 与 IK 状态，核对 pad center/root offset 与 local approach。
**Modify**：先预测 size-x 40→120 mm 对四个候选的 width/opening/IK 调用数影响，再运行第二条命令。
保持 pose/yaw/d/size-y/z 不变，解释为什么 90/270° 候选目标与 q 解仍相同。
不要把较少的候选误读为更低“抓取成功率”。

## Expected / Actual Result → Explanation

2026-10-05，机器 Zero，WSL2 kernel 6.18.33.2、Ubuntu 24.04.5。
从本机 conda info --base 定位 hook、激活 mujoco，同执行 shell 核验
`/home/lucas/miniconda3/envs/mujoco/bin/python`；Python 3.12.14。
required distributions/imports/module paths：NumPy 2.5.3、MuJoCo 3.13.0、Menagerie 2026.9.2、
Matplotlib 3.11.2；P0 helper imports 正常。无新增依赖，未核验无关 cv2；3.11 兼容目标未执行。

上述三命令退出 0，无 import error；actual compiled pad/attachment/slide conventions 检查通过。

| dimensions dx/dy | alpha | width / opening mm | width result | endpoint IK |
| --- | --- | --- | --- | --- |
| 40/30 mm | 0/180° | 40 / 48 | pass | both pass |
| 40/30 mm | 90/270° | 30 / 38 | pass | both pass |
| 120/30 mm | 0/180° | 120 / 128 | reject | not called |
| 120/30 mm | 90/270° | 30 / 38 | pass | both pass |
| 120/120 mm | all four | 120 / 128 | reject | not called |

Counts width-pass / IK-pass：4/4、2/2、0/0。
Far target=[2,2,0.03] m 单独用合法 40 mm width，得到 `ik_failed: pre update budget`，
q_pre/q_grasp=null；这验证的是局部 solver 的拒绝记录，不是全局 reachability solver。

独立从保存 q 重新 FK：passing endpoints 最大 position error=6.212543e-5 m；
rotation matrix Frobenius error max=2.943951e-6（无量纲），脚本另外检查 orientation angle<1e-3 rad。
内置 proper/top-down rotation、pad=center、local pre→grasp=[0,0,d]、模型参数、limits/residual/time=0 检查通过。
独立核对三套 JSON/NPZ/4 CSV rows、width拒绝不造 q、direct chain 与 SO(3)、
world yaw/translation 同变关系、generator purity、invalid dimensions/tilt/min closing aperture 通过。
CLI size-x nan/−1/0.5 m 均退出 2。默认 PNG 已目视检查。
未重跑前课/P0 的独立脚本，无 GUI、image estimation、path interpolation、collision/force/retention/dynamics。
只有 endpoints 可行，未知 home→pre 或 pre→grasp 之间是否可连续、无碰撞地执行。

## Failure Cases

| 现象 | 含义 / 检查 |
| --- | --- |
| 用 object pose 直接作 target | 忽略 root/pad offset 与 gripper orientation |
| T_OG T_WO 相乘 | frame 链错序；T_WG=T_WO T_OG |
| 一根 slide 当 aperture | a=0.020+2s，两个手指各移动 s |
| 输入 MJCF half-size 当 full size | opening 判断小一倍；显式声明 full尺寸 |
| tilt 被偷偷 flatten | 更改实际 grasp geometry；本课明确拒绝 |
| grasp/pre 方向错 | pre 应沿 −z_G retreat，前进沿 +z_G |
| 只验证 pre IK | grasp 仍可能求解失败；必须区分两阶段 |
| IK failed=物理绝不可达 | seed/budget/limits/π branch 可能导致 local failure |
| endpoints pass=可执行 | 未检查中间路径、碰撞、joint tracking、接触/摩擦 |
| width-pass=尺寸全面适配 | 未覆盖 height/finger span/floor/palm/shape/uncertainty |
| candidate 后继承前一状态 | 结果受顺序污染；独立 MjData/home，内部 grasp 才 warm-start |

## Robotics Context

这一步把 perception object frame 变成 robot tool frame，适用于已知规则形状的候选提案。
真实系统还需尺寸/pose uncertainty、collision/contact/force、approach path 和执行反馈。
下一小任务 S13.3a 才接视觉估计驱动的完整 home→pre→approach 动力学。
本课没有替代 planner 的全路径检查，也没有完成 pick-and-place。

## Interview Capsule

**30 秒**：对 upright box 构造四个 Rz(alpha) top-down T_OG，root 比 pad center 高35 mm，
乘估计 T_WO 得 T_WG，再沿−z_G退后求 pre-grasp。按 width+clearance 与夹爪 gap 筛选，
每个 candidate 从独立 home 求 pre IK，再从 q_pre 求 grasp IK。端点 pass 不等于 path/grasp success。

**2 分钟**：解释 full尺寸、各 frame 与指腹 offset，推导 pad=center、pre局部方向与 opening=0.020+2s。
说明0/180对dx、90/270对dy，40→120 mm 为什么只筛掉两组。
介绍 reuse DLS 输入输出与 data mutation、arm DOF Jacobian、limits、80步/π branch 限制。
独立 FK 核验不读取 truth，失败记录分 width/pre/grasp；最后解释 local IK、碰撞、执行和接触稳定性的证据不同。

## Must Remember / My Verification

- T_WO 是物体，T_WO T_OG 才是 gripper target。
- G=attachment_site，pad offset=35 mm；不能换 TCP 却沿用 offset。
- Full size / aperture / 单根 slide 是不同量。
- Pre retreat沿−z_G；IK两个端点要分别验证，solver原地改变私有data。
- 格式、width、IK、路径、tracking、grasp success 分开判断。

状态仅在根 README 维护。本人于 2026-10-05 确认实验与预测完成，Run/Modify 已确认。
Explain 第 1、2、4、5 项核心判断正确；第 3 项仅说明两指各有 slide，
尚待补 width/opening/slide 定量关系与尺寸改变后的候选筛选，暂不标记 Mastered。

反馈核对：物体 pose 需结合 T_OG、35 mm tool offset、pre 沿 approach 反向退后、
IK 依赖初值以及端点成功不证明全路径安全的解释正确。
精度补充：T_OG 将 G 坐标映射到 O，表示 G 在 O 中的位姿；
每个新候选重置 home 以控制初值并避免顺序污染，同一候选 grasp 则从 q_pre warm-start。
本模型 a=0.020+2s，所需 a=width+0.008 m；两指各移动 s，并非每根移动整个 opening。
本次仅同步反馈，runtime 沿用 2026-10-05 Zero 工程验证，未重跑实验/仿真/GUI。

**Explain**：

1. T_OG 的方向是什么？为什么 T_WG=T_WO T_OG，而不是把 object pose 直接交给 IK？
2. 为什么 root 需要高35 mm？怎样验证指腹中心与物体中心一致？
3. width、opening、每根 slide 的关系是什么？size-x 改为120 mm为什么保留90/270°？
4. pre-grasp 沿哪个方向退后？为什么 grasp IK 从 q_pre 开始，每个新候选却重置 home？
5. 两端点 IK 成功能证明什么？失败又为什么不能直接证明全局不可达？

Engineering 后 STOP，交回 Run/Modify/Explain；不自动实现 S13.3a。
