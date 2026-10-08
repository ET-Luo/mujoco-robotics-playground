# S17.7 — Hybrid position-force：同一 surface frame 中分轴控制

约0.5～2h；前置：[S17.6法向力反馈](20_6_normal_force.md)。
[代码](../examples/18_compliant_control/hybrid_control.py) / [示例README](../examples/18_compliant_control/README.md)。
Engineering / Learning状态只维护在[根README](../README.md#stage-17--compliant--contact-rich-control)。

## Problem → Why → Intuition

工具既要沿表面到达一个位置，又要压住表面保持指定法向力，怎样组合两个任务？
沿表面的位置目标不会直接决定压紧力；法向位置若被刚性锁住，又会与调力要求冲突。
本课给切向位置任务与法向力任务各一个互不重叠的方向，先在同一surface frame中合成运动参考，
再由已有inner motor loop跟踪。切向位置与法向力分别验收，不能只看最终位置或总force向量。

## Core Concepts / Scope

复用normal_force的force_velocity与参数，及contact_transition模型/measure，依赖链到cartesian_spring。
仅requirements已声明mujoco/numpy/matplotlib，Agg headless；无新依赖。
同XY slide，M=diag(2,1)kg、球半径.02m、平面y=0朝+Y、site在球心、固定orientation。
condim1无摩擦；零gravity/passive damping/external，dt1ms、joint cap20N。
inner K400N/m、D[56.568542,40]N·s/m；normal target2N，outer gain.003m/(N·s)，
力反馈参考速度cap20mm/s、normal reference5…60mm；approach速度30mm/s仍是独立策略。
contact gate active pair且+normal反力>.05N；触碰后启动力反馈，丢失/3s搜索超时则锁存actual reference，
停止任务reference更新，但继续inner控制/mj_step。正常两case未触发失联退出；S17.6失联证据沿用2026-10-08。

本课只在5…7s做一次30mm切向点到点移动，然后保持到12s；Modify只把距离翻倍60mm。
没有连续往返扫描、曲面跟随、UR5e移植或S17.8a完整integration验收。
normal_only对照组仍有法向力反馈，但切向reference固定0；共同的任务goal仍要求切向移动。
两case不改变normal target、initial或其他参数。

## Mathematics：frame / shape / unit

定义surface原点为world原点，R_WS的列为surface轴在world中的方向：
t=+X、n=+Y、b=+Z，R_WS=I（3×3）。这是右手frame；本fixture轴恰好与world重合，
仍显式使用x_S=R_WSᵀx_W、v_S=R_WSᵀJp qvel、F_S=R_WSᵀF_W。
同原点才可直接转位置；一般不同原点需x_S=R_WSᵀ(x_W−p_WS)。
平面geom的local Z是其world几何normal；代码先forward，从geom_xmat的第3列核验R_WS第2列。
不将plane local Z误认为surface n的第3项，也不把MuJoCo contact frame当本课surface frame。

| 量 | shape / 单位 | 含义 |
| --- | --- | --- |
| x_S/v_S、reference | (3,) m / m/s | surface切向/法向/binormal坐标 |
| S_p=diag(1,0,0) | (3,3)，无量纲 | 选择切向位置任务 |
| S_f=diag(0,1,0) | (3,3)，无量纲 | 选择法向力反馈生成的运动任务 |
| f_n | scalar N | 环境→工具反力的surface normal分量 |
| Jp | (3,2) | world线速度Jacobian，slide列m/m |
| τ_req | (2,) N | slide generalized force；非hinge torque |

S_p²=S_p、S_f²=S_f、S_p S_f=0，各selector对称，表示正交投影。
S_p+S_f=diag(1,1,0)，覆盖本二维可动子空间；不是完整3D的I，binormal结构不可动。
不能未经frame转换就把world分量套入surface selector；一般world projector为R_WS S R_WSᵀ。
代码只支持本fixture指定方向，错误frame/normal或重叠selector明确拒绝，不宣称支持任意斜平面。

切向参考：u=clip((t−5)/2,0,1)，distance=L：

\[
x_{p,t}=L(10u^3-15u^4+6u^5),\quad
v_{p,t}=\frac L2(30u^2-60u^3+30u^4).
\]

两端速度/加速度0，位置连续且单调；L30/60mm时reference峰速度28.125/56.25mm/s。
切向不受normal reference的20mm/s界约束。

法向反馈复用S17.6：e_f=2−f_n，u_n=clip(−.003e_f,±.02)m/s，
y_ref由h u_n积分并投影到[.005,.06]m，v_ref为投影后的位移增量/h。
令切向输入x_p=[x_t,0,0]、法向输入x_f=[0,y_ref,0]（均(3,)）：

\[
x_{ref,S}=S_p x_p+S_f x_f,\quad
v_{ref,S}=S_p v_p+S_f v_f.
\]

两输入各只有自己的一项非零。
normal_only将x_p/v_p置零，所以控制切向0而不是跟随任务goal。
法向位置reference来自力误差反馈；它不是另一个独立固定法向位置任务。
**本实现是reference层的hybrid position-force，复用位置跟踪内层；不是直接把N与m相加，
也不是直接叠加两个torque控制器或完整6D经典hybrid wrench controller。**

inner：F_S=K_inner(x_ref,S−x_S)+D_inner(v_ref,S−v_S)，F_W=R_WS F_S，τ=JpᵀF_W。
K/D仅在两个可动轴上使用，Z力0；motor ctrl=τ/gear，actual=clip(τ,±20N)。
每步验证τᵀqvel=F_Sᵀv_S，确保力和速度变换符合功率关系。
本fixture无摩擦、constant inertia与独立slide，所以理论上切向运动不改变normal动力学；
一般机器人配置相关惯量、摩擦与contact耦合下，selector不保证物理动力学解耦。

## Math-to-Code / APIs

```python
sensed_s = R_WS.T @ sensed_world
reference_s = S_P @ tangent_position + S_F @ force_generated_position
velocity_s = S_P @ tangent_velocity + S_F @ force_generated_velocity
x_s = R_WS.T @ data.site_xpos[site]
v_s = R_WS.T @ (jp @ data.qvel)
force_w = R_WS @ force_s
requested = jp.T @ force_w
```

MjModel.from_xml_string返回编译模型，MjData返回actual可变state/buffer。
mj_forward更新FK/contact/动力学不推进time；mj_jacSite原地写world(3,nv)Jp/Jr；
mj_fullM展开(nv,nv)物理惯量；mj_step更新真实qpos/qvel/time，执行不重设实际状态。
mj_contactForce按当前contact index将contact-frame[force;moment]写入(6,)buffer，
转换world并按probe在geom[1]/geom[0]取正/反号，详见[S17.4](20_4_contact_transition.md)。
反力先转surface再取normal；selector只操作同frame同单位的运动参考。
本步先previous ctrl fresh forward取得input_normal用于力反馈；current ctrl再forward得到current_normal，
用于实际积分与验收；pre-step数据包含两种测量，切换时不将其误称同一个读数。

## Minimal Experiment / Expected → Actual

仓库根目录：

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/18_compliant_control/hybrid_control.py
python examples/18_compliant_control/hybrid_control.py --distance .06
```

每命令hybrid与normal_only各12001×23 CSV，0…12s pre-step；results.json、hybrid.png。
Expected：hybrid同时完成切向位移与normal force；normal_only保持法向力但切向任务失败。
Modify距离翻倍、时间不变：切向reference速度/加速度翻倍，本理想装置normal轨迹应不变。

切向合格：5…7s运动窗口最大goal误差<1mm，11…12s尾窗口<.1mm。
法向合格：运动与尾窗口每个采样active contact且反力>.05N、|current_normal−2|<.05N。
task_passed=两项均合格；normal_only的任务失败是预期对照，不把engineering PASS当各case任务PASS。
normal velocity本课未作为新task验收条件；normal dynamics持续核对。
峰值反力覆盖approach/contact初次触碰，不只统计稳态；窗口判据只在采样点且至12s为止。

Actual（2026-10-08，DESKTOP-781D67A，默认/Modify两命令exit0，无import error）：

| case / distance | 运动窗口切向最大误差mm | 尾窗口切向误差mm | 运动法向最大误差N | task |
| --- | --- | --- | --- | --- |
| hybrid / 30mm | .204596 | <1e−9 | .035781 | 通过 |
| hybrid / 60mm | .409191 | <1e−9 | .035781 | 通过 |
| normal_only / 30mm | 约30 | 30 | .035781 | 切向失败 |
| normal_only / 60mm | 约60 | 60 | .035781 | 切向失败 |

所有case运动/尾窗口contact100%；normal尾平均1.999964N、最大误差.000057872N。
两距离hybrid实际切向峰速度28.6304/57.2607mm/s，峰X motor力.090772/.181543N；
Y motor峰1.999980N，无motor饱和。reference峰速度与actual不同，不把reference当硬速度上限。
contact触碰1.334s；current reaction峰4.663290N，input测量峰6.358974N。
正常两case未退出，因此不声称本轮重新验证了动态contact loss；退出策略来自S17.6。

selector对称/幂等/无重叠与几何normal核对通过；静态错误normal的90°frame、重叠selector、
左手frame均ValueError拒绝；欠力负向/过力正向的符号probe通过。
错误配置未运行发散动力学。独立混杂输入测试验证selector去除无关轴，
非恒等旋转的向量功率变换核验通过；不是倾斜表面的动态验证。
每步J/Jr/M、power、actual motor cap、contact Jᵀ、动力学/零bias/passive/external与actual半隐式递推通过。
独立CSV quintic reference、outer/inner控制与projection、动力学budget，
恢复初始/峰值/运动中点/末尾contact求解核对通过；shape/clock/finite通过。
hybrid与normal_only，以及30/60mm之间的normal reference/actual/control/contact各字段≤1e−10相同；
本fixture的解耦通过，不等于一般hybrid控制保证解耦。
CLI distance0/nan各exit2；默认PNG目视检查；本地Markdown文件链接/git diff --check通过。
产物仅ignored tmp/s17_7_d0.03/d0.06与临时日志/tmp；无新依赖/requirements改动。
WSL2 Ubuntu24.04.5/kernel6.6.87.2，Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2，
当前conda hook激活mujoco，同执行shell核验interpreter、metadata、真实module路径及MuJoCo API。
代码保持3.11兼容语法，未在3.11执行；无GUI/UR5e/ROS/硬件验证。

## Explanation / Failure Cases / Robotics Context

法向位置reference是力反馈内部状态，切向reference来自任务trajectory；selector防止独立位置任务
把法向reference覆盖掉。内层仍计算两轴位置/速度误差，这与外层任务分轴不矛盾。
如果直接增加第二个法向固定位置任务，两者可能互相覆盖或叠加；不能同时任意满足墙面位置与反力。

frame错时，原本法向的反馈可能沿切向移动；符号错时，欠力向外退会加大误差。
本课guard使用几何normal与指定selectors核验，避免靠“数值看起来像X/Y”猜方向。
只测切向误差会漏接触丢失；只测反力会把normal_only判为完成整个任务。
移动时仍须测force/contact窗口，静态最终反力不能代替运动期间表现。
selector是任务投影，不是动力学解耦器；真实机械臂还存在惯量、重力、摩擦、姿态与饱和耦合。

本次无摩擦所以不研究切向摩擦力、滑移或stick-slip；平面无限，无边缘越界。
仅单次已知平面点到点移动，不证明连续扫描、未知曲面、通用稳定性或安全停止。
应用为恒力擦拭/打磨/装配贴面；下一课S17.8a再完成fixture扫描integration，不能将其提前标为完成。

## Interview Capsule

30秒：在surface frame选切向position与法向force，用互不重叠的projectors合成运动reference。
法向reference由实测力误差积分得到，切向由平滑trajectory提供，再交给motor inner loop。
分别评价切向误差、法向力误差与接触保持；normal_only反力合格但移动任务失败。

2分钟：说明surface轴/R的列、world↔surface向量变换与同原点位置假设；
写S_p/S_f幂等和互斥、reference合成及Jᵀmotor映射。解释法向参考不是独立固定位置任务，
指出这里是reference层hybrid而非直接混合力与位置单位。
展示30/60mm同2s：切向误差/速度/力需求翻倍、normal轨迹不变；说明这是指定无摩擦固定惯量装置，
不能把selector等同物理解耦，也不能只看一个任务的指标。

## Must Remember / My Verification — Run / Modify / Explain

先统一frame，再分轴；position/force任务不重叠；分别验收位置、力与接触。
这里S_p+S_f覆盖二维active subspace，binormal不可动；一般不自动等于3D单位矩阵。
Engineering Complete；2026-10-08 本人确认实验与预测完成并正确回答五项Explain，
Run/Modify/Explain全部完成，Learning Mastered；状态只见根README。
解释精度补充：相同时长/归一化轨迹下，距离翻倍使reference速度与加速度翻倍；
本无摩擦、固定惯量、未饱和线性fixture的actual切向误差本次也翻倍，
一般机器人不能据此保证误差严格缩放或法向动力学不变。

Run：默认命令后查看tmp/s17_7_d0.03/hybrid.png与JSON，解释normal_only为何task_passed=False。
Modify：先预测只将距离30→60mm、时长仍2s时，切向reference速度/加速度、actual误差/motor力与normal反力怎样变化，
再运行--distance .06，对照预测与实际；不改变目标力或增益。
Explain：

1. surface frame的t/n/b分别是什么？R_WS的列表示什么，为什么力反馈与selector必须在同一frame？
2. S_p与S_f怎样防止任务重叠？本二维装置为什么S_p+S_f不是完整3D的I？
3. normal力反馈仍生成位置reference，为何不等于同时给normal两个独立任务？
4. 为什么normal_only能保持2N却未完成任务？移动期间应该分别检查哪些指标？
5. 距离翻倍、时长固定时哪些量应翻倍？本fixture法向轨迹不变能否推广到一般机器人？

下一小任务S17.8a Surface Following Integration / fixture等待明确请求，不自动实现。
