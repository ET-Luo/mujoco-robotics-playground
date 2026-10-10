# S18.10b — Force closure intuition：从局部接触力到物体 wrench

Engineering Complete（2026-10-10）；Learning Run / Modify / Explain待本人验证，以[根README](../README.md)为准。
[代码](../examples/19_teleoperation_dexterous/force_closure.py)，约0.5～2小时。依赖已有NumPy/Matplotlib，无新安装。

## Problem → Why → Intuition

S18.10a的摩擦锥回答“一个接触允许什么力”。抓取还要回答“这些接触的力和力臂，能否共同产生抵消外部扰动的物体wrench”。这需要同时考虑方向、接触不能拉，以及力矩。

直觉：两个接触都从左边向右推，即使各自能提供斜向摩擦力，也不能共同产生向左的合力。对向接触可以用内部夹紧抵消彼此的法向力，把切向力用于支撑或形成力偶。但无摩擦的对向夹紧无法抵抗垂直于夹紧方向的扰动。

本课只分析**固定几何、刚体、平面点接触静力学**。平面wrench为`[Fx,Fy,tau_z]`，这是三维的力/力矩空间，不是空间六维wrench。没有MuJoCo动力学积分、servo控制、滑动速度、真实设备或GUI；不会把本课结论转移为S18.9三维手部已证明closure。

## Core concepts / 三组布局

所有坐标在world XY中，物体参考点O位于原点；正tau_z表示从+Z看逆时针。
每个接触只有施加在**物体上**的力，没有独立接触力矩（hard point contact）。

| Case | 接触点（m） | 指向物体内部的world法向 | μ |
|---|---|---|---|
| opposed_friction | (-.03,0)、(.03,0) | +X、-X | 默认0.5 / Modify0.1 |
| opposed_frictionless | 同上 | 同上 | 恒为0 |
| same_side | (-.03,-.02)、(-.03,.02) | 均+X | 默认0.5 / Modify0.1 |

每接触的有限负载测试另加`fn≤1 N`。**Closure方向性质**不包含这个力上限：它问允许任意放大接触力时能否覆盖所有平面wrench方向。有限负载可行性则问在实际指定上限下能否抵消某个具体wrench，两个结果分别记录。

## Mathematics：shape / unit / frame

### 1. Grasp matrix：力 → 合力与力矩

把两接触的world力堆叠为`f=[f1x,f1y,f2x,f2y]`，shape `(4,)`，单位N。
对参考点O，令`r_i=p_i-O`，单位m：

\[
G_i=\begin{bmatrix}1&0\\0&1\\-r_{iy}&r_{ix}\end{bmatrix},\qquad
G=[G_1\ G_2]\in\mathbb R^{3\times4},\qquad
w_c=Gf=\begin{bmatrix}\sum f_x\\\sum f_y\\\sum(r_xf_y-r_yf_x)\end{bmatrix}.
\]

结果单位依次N、N、N·m。力矩必须相对同一个参考点；G本身没有把摩擦/单边接触约束加入。
本课三布局的G都rank3，这只能说明**允许任意有正负的world力**时能覆盖三轴wrench，不保证这些力物理允许。

### 2. 平面摩擦边：非负系数 → 合法接触力

每个单位法向`n_i=(nx,ny)`配切向`t_i=(-ny,nx)`。平面锥是两条边之间的楔形，**两边表示在这个二维接触模型中是精确的，不是空间圆锥的多面体近似**：

\[
e_i^+=n_i+\mu t_i,\quad e_i^-=n_i-\mu t_i,\quad
f_i=a_i^+e_i^+ + a_i^-e_i^-,\qquad a_i^+,a_i^-\ge0.
\]

边向量无量纲；a的单位N。于是`fn_i=a_i^++a_i^-≥0`，`ft_i=μ(a_i^+−a_i^-)`，自动满足`|ft_i|≤μfn_i`。μ=0时两条边重合，只剩法向。

令`C=blockdiag([e1+ e1-],[e2+ e2-])`，shape `(4,4)`，并定义：

\[
W=GC\in\mathbb R^{3\times4},\qquad w_c=Wa,\qquad a\ge0.
\]

W的列是单位法向幅度对应的边wrench，顶两行系数无量纲、第三行含m；a乘它得到N/N/N·m。**rank(W)**考虑了边方向，仍只测试有符号线性组合，不等于非负组合能覆盖所有方向。

### 3. Positive span 与平面closure证据

本课四条边满足平面force closure的条件是：

\[
\operatorname{rank}(W)=3,\qquad W\lambda=0,\quad \lambda_j>0\ \forall j.
\]

λ可归一化为`sum(lambda)=1`，此时是无量纲权重；再乘一个以N为单位的幅度可解释为内部夹紧。对向有摩擦的λ为`[.25,.25,.25,.25]`，四个正权重产生零物体wrench。

为什么两个条件一起有效：rank3保证任意目标wrench能找到一个有正负系数的解a；把足够大的正内部夹紧`c lambda`加上去，`W(a+c lambda)=Wa`不变，而所有系数变成非负。这样获得任意平面目标wrench的合法组合，但系数可能非常大，会超过有限压力上限。

这是[Modern Robotics官方Force Closure](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/)给出的正生成与严格正零空间判据；本课只在具体平面点接触模型内核验。

代码解增广系统`[W_scaled; ones] lambda=[0,0,0,1]`，检查残差、严格正权重和rank。本课恰有4边；在rank3且closure成立时增广系统给出唯一归一化权重。不能把这里的一次lstsq求解当任意多接触问题的通用正系数搜索器。

两个失败对照：

- 无摩擦对向：λ仍严格正、内部夹紧为零wrench，但W rank1，不能覆盖Fy或tau。
- 同侧：W rank3，但每列Fx=+1；非负组合永远Fx≥0。分离方向`d=[1,0,0]`满足`d^T W>0`，证明无法产生负Fx，不存在严格正零空间权重。

### 4. 力上限下的负载平衡

对指定外部wrench`w_ext`求：

\[
Wa=-w_{ext},\qquad a\ge0,\quad a_1^++a_1^-\le1\mathrm N,\quad a_2^++a_2^-\le1\mathrm N.
\]

注意负号：接触合wrench要抵消外部wrench。两接触的法向力可分别选择，1 N是上限，不是固定预压值。

本课不使用第三方优化器。4个系数、6个不等式形成紧致多面体：选择W的r条独立等式，枚举`4−r`条激活不等式；若组成满秩4×4系统，就求解并检查**全部**等式与不等式。非空紧致可行集合一定有顶点，因此穷尽顶点可判定这里的可行/不可行。代码返回一个总法向载荷最小的可行顶点，不是动态抓取控制器。

记录无约束lstsq作为错误对照：它可能准确平衡wrench，却使用负边系数（等于接触拉力）或超过1 N。仅凭残差小不能宣布可行；某个无约束解不合法，也不能直接宣布所有约束解不存在。

### 5. 单位与参考点

数值rank、残差与绘图使用`[Fx,Fy,tau_z/L]`，固定`L=.03 m`使三行都对应N。这个可逆缩放不改变rank或可行性；不把N与N·m直接相加来定义“距离”或“抓取质量”。图中第三轴是归一化力矩，不是Fz。

参考点从O移动到O'=O+d后：

\[
\tau_{O'}=\tau_O-d_xF_y+d_yF_x.
\]

G/W和外部wrench都要同时换参考点。若只改接触力矩、不改外部力矩，就改变了平衡问题；两者一致变换时，可行性与closure不变。

## Math-to-code / APIs

| 函数或API | 输入 → 返回值 | 用途 |
|---|---|---|
| `wrench_maps(points,normals,mu,origin)` | `(2,2)`m、`(2,2)`单位向量、μ、`(2,)`m → G/C/W | 显式拼接力臂与摩擦边，无黑盒grasp模型 |
| `np.linalg.matrix_rank` | 矩阵 → 整数rank | 检查独立wrench维数 |
| `np.linalg.lstsq(A,b)` | 增广系统 → 解、残差信息、rank、奇异值tuple | 代码取第0项后自行核验约束，不把最小二乘当可行性证明 |
| `np.linalg.solve(A,b)` | 满秩4×4与rhs → `(4,)`系数 | 解一组候选激活约束 |
| `bounded_balance(W,external)` | `(3,4)`W、`(3,)`N/N/N·m → `(4,)`N或None | 全部顶点检查后返回合法系数或不可行 |
| `feasible_vertices` / plot | 截断两接触三角形的9组顶点组合 → wrench集合凸包 | 可视化有限负载范围；颜色检查的是`-w_ext` |

没有调用MuJoCo；这是静态代数实验，physics step/time均不适用。下一任务才回到动态扰动整合。

## Minimal experiment / Run

从仓库根目录、WSL2 Ubuntu24.04、conda mujoco数值环境执行：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/19_teleoperation_dexterous/force_closure.py
```

产物只在ignored `tmp/s18_10b_mu0.5/`：results.json含矩阵/证书/逐负载力分配和无约束对照，load_tests.csv为24×7，contact_layouts.png与bounded_wrenches.png。case_id按三布局顺序，load_id按JSON测试顺序；可行集合为蓝色，所需接触wrench点`-w_ext`可行时绿色、不可行时红色。

八个外部负载：Fx±0.4 N、Fy±0.4 N、tau±0.012 N·m、mixed `[-.4,-.1,-.003]`、overload `[2,0,0]`。前六个单轴测试不是closure证明；证明来自本模型的rank+严格正零空间证书。

## Expected / Actual / Explanation

2026-10-10 DESKTOP-781D67A，WSL2 Ubuntu24.04.5；conda mujoco Python3.12.14，NumPy2.5.3/Matplotlib3.11.2 metadata/import/实际模块路径与linalg API核验。无安装；MuJoCo/ROS/RL不是本课执行依赖。
默认与`--mu .1`两命令均exit0，三布局×八负载×两μ设置共48个记录。

| 布局 | rank(G) | rank(W) | 严格正零空间 | 平面closure | μ0.5有限测试 | μ0.1有限测试 |
|---|---:|---:|---|---|---:|---:|
| 对向有摩擦 | 3 | 3 | 有，四权重均0.25 | 是 | 7/8 | 2/8 |
| 对向无摩擦 | 3 | 1 | 有，但缺维度 | 否 | 2/8 | 2/8 |
| 同侧 | 3 | 3 | 无，存在分离方向 | 否 | 2/8 | 1/8 |

对向μ0.5只有2 N的overload失败：净Fx的上限仅1 N（一个接触最多推1 N，另一接触只能抵消或为0）。因此即使有closure，也不能抵抗任意**大小**的负载。
对向μ0.1仍有closure，但在cap1 N下，最大纯Fy为`2μ=0.2 N`，最大纯tau为`2 L μ=0.006 N·m`，本次0.4 N与0.012 N·m测试都超限；只剩±Fx测试可行。
同侧μ0.5可抵抗负Fx及mixed；μ0.1仅负Fx可行。它不是完全无法承载，只是缺少某些方向。对向无摩擦仅±Fx可行。

独立验证：直接world力/叉积重建W各列与合法结果，所有fn非负/≤1 N且`|ft|≤μfn`；用手推闭式不等式对照1248个负载（含固定48项与确定种子随机项），可行/不可行全部吻合，归一化`[Fx,Fy,tau/L]`的最大平衡残差2.32e−16 N。
把参考点移到`[.017,-.013]` m并同时变换外部wrench后，1248项可行性保持。CLI非法μ、geometry/wrench输入guards与3.11语法解析通过；实际执行为3.12。四PNG目视、本地链接/git diff --check通过，无物体动力学/GUI验证。

## Failure cases / Robotics context

1. 把G或W满秩当closure：允许负系数的线性span不同于只能推的正span，同侧案例直接反驳。
2. 只检查零wrench内部夹紧：无摩擦案例有正权重，但不能产生切向力或力矩；还需要覆盖完整平面维度。
3. 对每接触分别合法就宣布整体合法：必须组合成对同一参考点的物体wrench，且方向/大小能抵消外载。
4. 最小二乘残差为0就宣布抓取可行：负法向/负边系数与超过力上限都不能被残差检查替代。
5. 有closure或通过八个静态负载，就宣布真实三维抓取稳定：没有验证六维wrench、接触保持、执行器映射、柔性/延迟、滑移或长期抗扰；本课证书只适用于这个理想平面模型。

应用：理解抓取接触布局的方向缺陷、区分几何closure与执行器力能力、检查物体力矩与接触力分配。真实手的关节力矩上限还要通过Jacobian/传动映射为可实现的接触力，不能直接把本课1 N上限当servo torque cap。

## Interview capsule

**30秒：** 我用G把平面接触力映射为物体合力/力矩，用C把非负摩擦边系数映射为合法接触力，得到W=GC。平面closure需要rank3和严格正零空间权重；同侧接触满秩仍缺负Fx，无摩擦对向有内部夹紧仍缺Fy/力矩。有限法向力上限另行限制可抵抗的负载大小。

**2分钟：** 给出`tau=rx fy−ry fx`及共同参考点，解释C的两边和非负系数。说明rank3允许有符号解，正零空间可加内部夹紧把它变为非负，但会增加法向载荷。列举μ0.5→0.1对向closure不变、有限测试7/8→2/8，证明方向覆盖与载荷能力不同。介绍显式顶点枚举以及单位/参考点变换检查，最后划清理想平面closure与真实三维动态稳定的边界。

## Must remember

G只组合world力；C加摩擦/单边约束；W=GC；rank不是正span；内部夹紧为零物体wrench但消耗接触载荷；抵抗外载要`Wa=-w_ext`；力矩统一参考点；closure不是无限执行器力，也不是稳定性证明。

## My Verification — Run / Modify / Explain

根README学习框保持未勾选，助手运行不代替本人学习。

**Run：** 运行默认命令，查看三布局证书、图与逐负载的可行力分配；找出same_side/push_x_plus的无约束解为何不能执行。

**Modify：** 先预测μ从0.5降至0.1后三布局rank、closure和有限负载结果如何变化，再运行：

```bash
python examples/19_teleoperation_dexterous/force_closure.py --mu .1
```

解释对向有摩擦为何仍closure却更难支撑Fy与tau，不调整normal cap或测试负载。

**Explain：**

1. G、C、W各映射什么？为何物体平面wrench是3维，接触力是4维？
2. 满秩为什么不等于force closure？同侧接触缺少哪个接触合力方向？
3. 严格正零空间权重表示什么内部力？为什么无摩擦案例仍不能closure？
4. μ降低后对向布局仍closure，为什么有限负载通过数下降？为何无约束lstsq残差小仍不够？
5. 更换力矩参考点时需同时改变哪些量？本课平面静态证书为何不能证明真实三维动态抓取稳定？
