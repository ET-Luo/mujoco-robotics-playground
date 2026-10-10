# S18.10a — Friction cone intuition：一个接触能承受多大切向力？

Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain），状态以[根README](../README.md)为准。
[代码](../examples/19_teleoperation_dexterous/friction_cone.py)；本课约0.5～2小时，无新增依赖。

## Problem → Why → Intuition

S18.9发现：有手指法向力，物体仍可能沿接触面滑动。现在先隔离一个接触，理解它能提供的切向力范围，再谈多接触的组合。直觉是“压得更紧，通常能承受更大的切向力；材料更防滑，同样压紧力能承受更多切向力”。这两种改变对摩擦锥的几何影响并不相同。

本课画的是**力空间**：坐标单位N，轴为接触法向力和两个切向力，不是世界空间的物体轨迹，也不是指尖允许运动的区域。

## Core concepts / 最小模型

使用100 g、半径20 mm球形接触头和world XY平面；接触头安装在三轴平移滑台上，**不能旋转**。三个slide的轴分别为world X/Y/Z，nq=nv=3、nu=0；滑台的转动约束会承受接触力矩，因此它不是自由球的滚动实验。

gravity=0，持续在body COM施加world向下`N_load`，没有xy弹簧、关节阻尼或位置控制。0～1 s先建立法向平衡；1 s起以0.5 N/s增加world对角线方向的切向力，最多扫到1 N / 3 s。发现切向速度首次超过**0.1 mm/s**就记录并结束该case；这是人为报告阈值，不是数学上的首次非零滑动。

| 参数 | 设置 / 目的 |
|---|---|
| 摩擦系数μ | 0.2 / 0.6，无量纲 |
| 法向外加载荷 | 默认0.5 / 1 N，Modify整体乘2 |
| `condim=3` | 法向 + 两个滑动摩擦分量；无自旋/滚动摩擦力矩 |
| `cone="elliptic"` | 各向同性时直接匹配圆形切向截面的摩擦锥 |
| `impratio=10` | 摩擦维相对法向维更硬，减少微滑；不改变μ |
| 接触参数 | `solref=.005 1`、`solimp=.99 .99 .001`，柔性接触 |
| 求解/积分 | Newton、iterations100、tolerance1e−12；默认Euler、dt1 ms |

两geom的滑动μ设为相同值；每次读取并核验实际`contact.friction[:2]`，避免把单个geom属性直接当最终接触属性。本课没有MuJoCo之外的摩擦求解器，也不改变既有hand.xml或抓取实验。

## Mathematics：shape / unit / frame

接触坐标系的受力向量：

\[
f_C=[f_n,f_{t1},f_{t2}]^T\in\mathbb R^3,\quad f_n\ge0,\quad
\|f_t\|_2=\sqrt{f_{t1}^2+f_{t2}^2}\le\mu f_n.
\]

所有力分量单位N；μ无量纲。`f_n`不是压强Pa。普通无黏附接触可以推开物体，不能提供任意拉力，所以要求`f_n≥0`。

固定`f_n`时，切向允许集合是半径`μ f_n`的圆盘；把所有`f_n≥0`的圆盘叠起来成为圆锥。锥的半角（相对法向轴）为：

\[
\alpha=\arctan\mu.
\]

μ=0.2/0.6的半角分别约11.310°/30.964°。增大μ会扩大锥的开口；固定μ增大法向力只是在同一锥上选择更大的截面。

摩擦不是每时每刻都等于`μ f_n`。理想静止平衡中，只需提供足够抵消外加切向载荷的摩擦力，其大小可以在0与上限之间。若所需力超过允许集合，接触力仍必须满足锥约束，不能因为外加载荷大就“合法地产生锥外摩擦力”。

本课外加力为：

\[
F_{ext}=T(t)\frac{[1,1,0]^T}{\sqrt2}+[0,0,-N_{load}]^T,
\quad T(t)=\operatorname{clip}(0.5(t-1),0,1)\ \mathrm N.
\]

`[1,1,0]/√2`是单位方向；若漏掉归一化，真实切向载荷将是写入系数的√2倍。法向平衡后`f_n≈N_load`，理想静态切向能力为`μN_load`。负载斜率0.5 N/s给出预测越界时间`1 + μN_load / 0.5`秒（若能力>1 N，本次扫描不会越界）。

**动态时不能直接替换`f_n=N_load`：** `m a_z=F_contact,z−N_load`。扫描/接触瞬态会产生小的法向加速度。真正检查接触力是否合法要用测得的`f_n`，而不是只用外加载荷。

利用率定义为`u=||ft||/(μ fn)`，仅在`fn>0`时有意义；代码用0表示没有有效分母的样本，并另存active count。`u≥0.99`只是“接近锥边界”的记录条件，速度阈值是另一个条件。

接触frame的三条world坐标轴按行存储，法向是row0。`frame @ v_W`得到接触坐标速度；因为平面静止、滑台不转动，接触点相对速度就是滑台world平移速度。切向速度为：

\[
v_{slip}=\|frame[1:3,:]v_W\|_2,\quad
m a_W=F_{contact,W}+F_{ext,W}.
\]

速度单位m/s，接触frame为`(3,3)`。这里world slide的qvel三分量就是world线速度；这个特例不能推广为“任意机器人qvel前三项都是world速度”。

## Math-to-code / API

| API或表达式 | 输入 → 输出 / 原地更新 | 用途 |
|---|---|---|
| `MjModel.from_xml_string` | 最小MJCF文本 → model | 独立单接触滑台，模型概念直接可见 |
| `MjData(model)` | model → 新状态与缓冲 | 每组μ/N独立初始化 |
| named `joint`、`jnt_qposadr/jnt_dofadr` | slide名称 → state地址 | xyz状态不用geom/body索引冒充 |
| `xfrc_applied[body,:3]` | world COM力 `(3,)`，N | 设置法向与切向外载；不是直接写contact力 |
| `mj_forward(model,data)` | 原地刷新派生量/接触解，无时间积分 | 把当前状态和当前力对齐后写CSV |
| `mj_contactForce(...,index,raw)` | contact index → 原地写`raw (6,)` | 前三项是`[fn,ft1,ft2]`，后三项本课为0 |
| `frame.T @ raw[:3]` | contact力 → world力 | 根据contact.geom[0/1]决定受力对象符号 |
| `mj_step(model,data)` | 原地积分、推进1 ms | 让外力和接触力决定实际速度，不覆盖qpos |

MuJoCo支持elliptic和pyramidal锥；本课显式选择前者，S18.9的默认pyramidal近似不能不加区分地当圆形截面。见[官方Contact计算模型](https://mujoco.readthedocs.io/en/latest/computation/#contact)。
官方说明regularized soft contact即使所需力在锥内也可能持续微滑，增大impratio可减轻微滑但不保证精确粘住，见[官方Preventing slip](https://mujoco.readthedocs.io/en/latest/modeling.html#preventing-slip)。本课不启用NoSlip后处理，不通过调参隐藏这一现象。

## Minimal experiment / Run

只需requirements已声明的mujoco/numpy/matplotlib，无local helper、新安装、ROS或RL。从仓库根目录：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/19_teleoperation_dexterous/friction_cone.py
```

输出到ignored `tmp/s18_10a_normal1/`：四份33列CSV、results.json、load_scans.png、force_cones.png。每帧在mj_forward之后记录，再mj_step；CSV行数为实际physics步数+1。首次报告滑动就停止，因此各case时长/步数不同；更新频率仍相同。

## Expected / Actual / Explanation

2026-10-10，DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；conda mujoco Python3.12.14，MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2 metadata/import/API核验。无依赖安装；实际执行不是Python3.11，单独通过3.11语法解析。
默认命令和`--normal-scale 2`均exit0，共8组扫描；0.5～1 s实际fn与法向外载一致至1e−8 N，稳态之后到停止均保持唯一active contact。

| Scale | μ | 法向外载N | 理想能力N | 首次速度>0.1 mm/s | 对应切向外载N | physics步数 |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 0.2 | 0.5 | 0.1 | 1.207 s | 0.1035 | 1207 |
| 1 | 0.2 | 1 | 0.2 | 1.407 s | 0.2035 | 1407 |
| 1 | 0.6 | 0.5 | 0.3 | 1.608 s | 0.3040 | 1608 |
| 1 | 0.6 | 1 | 0.6 | 2.208 s | 0.6040 | 2208 |
| 2 | 0.2 | 1 | 0.2 | 1.407 s | 0.2035 | 1407 |
| 2 | 0.2 | 2 | 0.4 | 1.807 s | 0.4035 | 1807 |
| 2 | 0.6 | 1 | 0.6 | 2.208 s | 0.6040 | 2208 |
| 2 | 0.6 | 2 | 1.2 | 扫到3 s未触发 | 扫描上限1.0 | 3000 |

首次报告时间比理想越界预测晚7～8 ms，受速度阈值、惯性、接触正则化和1 ms采样影响；它不是摩擦系数的直接测量，也不是精确静摩擦切换时刻。各组实际接触力利用率均不超过1（浮点误差量级1e−16）。

最强组μ0.6/N2的末利用率约0.833332，速度仍为**0.024969 mm/s**，小于报告阈值但非零。默认四组在外载小于理想能力一半时也测得最大约1.207/2.469/3.707/7.457 μm/s的微滑。因此代码中`first_slip_time=null`应读作“此次未超过指定阈值”，不能读作“严格零滑动”。

每帧world接触力与`qfrc_constraint`的残差0，Newton平衡残差最大2.99e−13 N。独立CSV检查重建frame/受力方向、切向范数/能力/u、相对速度、扫描协议与首次停止、法向平衡和动量变化；离散冲量残差最大8.93e−14 N·s。两命令重复的同μ/N trace完全一致。非法参数CLI/输入guards/3.11语法、四PNG目视和本地链接通过；没有GUI或真实材料测试。

## Failure cases / Robotics context

- 写`ft=μfn`作为所有时刻的摩擦力：把最大能力误当实际响应，零切向负载也会被错误加速。
- 逐分量检查`|ft1|≤μfn`且`|ft2|≤μfn`：这定义正方形；两个分量均0.8μfn时范数约1.131μfn，已在圆盘之外。
- 用外加载荷检查锥，不读实际fn：动态时法向力会变化；静态预测与逐帧合法性检查应分开。
- 把锥内解释为仿真严格粘住，或把null事件解释为零位移：本课已测得锥内微滑。
- 把single-contact范围当force closure：锥只描述一个接触可施的局部力，不证明多个接触能抵抗哪些物体力矩，也不证明抓取长期稳定。

机器人应用：估计夹持所需法向力、判断某接触的切向需求是否过大、分析脚底/物体表面的滑移；后续还需考虑接触几何与力臂、多接触分配和真实材料变化。球形平移滑台不会模拟自由滚动或完整手部动作。

## Interview capsule

**30秒：** 摩擦锥是单接触允许的力集合：法向非负，切向范数不超过μ乘法向力。增大μ扩大开口，增大法向力扩大同一锥上的截面。我用单接触滑台扫描外载，验证实际接触力始终在锥内，并区分力到边界与速度超过报告阈值。

**2分钟：** 解释接触坐标`[fn,ft1,ft2]`、各向同性圆盘与半角atanμ，说明摩擦力不是恒等于μfn。给出μ0.2/N0.5预测0.1 N、首次报告0.1035 N的对照，说明采样/速度阈值/惯性造成时间差。强调实际fn可不同于外加载荷，需转换受力对象/world frame检查Newton方程。最后用μ0.6/N2的u约0.833、非零微滑说明柔性接触的局限，并划清单接触锥与多接触force closure的边界。

## Must remember

力锥不是位置锥；μ无量纲、fn是N；normal为contact axis0；法向力非负；切向能力看二范数；最大摩擦≠实际摩擦；锥内≠数值零滑动；增大法向力不改变锥开口；单接触合法力不等于整体抓取稳定。

## My Verification — Run / Modify / Explain

2026-10-10 本人明确确认实验与预测完成，并准确回答全部五项Explain；根README记录Run/Modify/Explain完成，Learning Mastered。

**Run：** 执行默认命令，查看三条扫描曲线与3D力锥，比较理想能力、near_boundary和first_slip三个数值，确认CSV实际步数。

**Modify：** 先预测将法向载荷整体翻倍后，各组切向能力、锥开口、首次报告时间和步数如何改变，再运行：

```bash
python examples/19_teleoperation_dexterous/friction_cone.py --normal-scale 2
```

重点解释最强组为什么扫描结束而不是触发速度阈值，并检查其末速度是否严格为零。只改变法向载荷，不调整μ、阈值或接触参数。

**Explain：**

1. 摩擦锥为什么满足fn≥0和`||ft||≤μfn`？为何不能分别检查两个切向分量上限？
2. 增大μ与增大法向力分别改变锥的什么？为什么实际摩擦力不总等于μfn？
3. 外加切向载荷超过预测能力，为什么实际contact力仍在锥内？动态fn为何可能不同于法向外载？
4. `u<1`、`first_slip_time=null`分别能说明什么，为什么都不能证明严格零滑动？
5. 法向载荷翻倍为何改变首次报告时间和仿真步数，却不改变dt？单接触实验为什么不能证明多指force closure？

解释精度：动态法向力一般可能受夹紧控制影响，但本课滑台没有夹紧控制器；本课由外加载荷、接触与运动状态共同决定。此次仅同步七份文档，链接/状态/git diff --check通过；未重跑仿真或GUI，运行证据沿用本课2026-10-10工程验证。
