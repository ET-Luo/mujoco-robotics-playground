# S18.7 — Fingertip FK / Jacobian

0.5～2h；前置：[三指模型](21_6_hand_fixture.md)、[既有Jacobian课](06_jacobian.md)。
[代码](../examples/19_teleoperation_dexterous/fingertip_jacobian.py) / [模型](../examples/19_teleoperation_dexterous/hand.xml)。
Engineering/Learning状态唯一来源：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

同一手的六个关节怎样影响某一指尖？不能直接把旧机器人的site名字、关节列和坐标系搬过来。
本课复用FK/中心差分方法，在新模型上核对world位置、姿态、Jacobian与速度。
直觉：一指尖沿两关节转动的瞬时切线运动，其他两指的关节不在它的body祖先链上，因此相关列为零。
这不是“所有手指运动永不耦合”：几何Jacobian的祖先依赖与动力学/接触耦合是两件事。

## Core Concepts / scope

同一hand.xml，nq=nv=6，三个tip site分别为f0_tip/f1_tip/f2_tip；固定palm。
复用hand_fixture的MODEL_PATH/mapping，不运行其main或动力学实验；helper链只导入mujoco/numpy/matplotlib。
requirements已有三包，无新依赖、Menagerie或ROS。三个配置open、bent、asymmetric，每配置三个site共9记录。
离线MjData设qpos检查几何，全部只mj_forward，没有mj_step、controller、IK、抓取或GUI。
中心差分在open边界可能写入负角度，这是验证数学FK的离线probe，不是可执行的限位内运动。

## Mathematics：frame / shape / unit

对finger i，theta=2πi/3；outward r=[cosθ,sinθ,0]，inward u=−r，z=[0,0,1]，world hinge轴a=z×u。
根位置b=.06r+[0,0,.046]m；q1/q2是该指prox/dist角度rad，L1=.04m、L2=.03m。

```text
p_W = b + u(L1 sin q1 + L2 sin(q1+q2))
        + z(L1 cos q1 + L2 cos(q1+q2))                   # (3,) m
J1 = u(L1 cos q1 + L2 cos(q1+q2))
     − z(L1 sin q1 + L2 sin(q1+q2))
J2 = u L2 cos(q1+q2) − z L2 sin(q1+q2)                  # (3,) m/rad
R_W_site = Rz(theta+π) Ry(q1+q2)                         # (3,3), columns site axes in world
```

MuJoCo Jp_W为(3,6)，**行world XYZ、列velocity DOF**。只有该指的prox/dist列为[J1,J2]，其他四列为0。
Jr_W同为(3,6)，所属两列均为a，无量纲（rad/rad）；其余列0。
`v_W=Jp_W@qvel`为(3,)m/s，`omega_W=Jr_W@qvel`为(3,)rad/s。
qpos address用于扰动姿态，jnt_dofadr用于Jacobian列；actuator id不用于选列。此手四种索引概念仍分开解析。

中心差分：`Jp[:,j]≈(p(q+εe_j)−p(q−εe_j))/(2ε)`，εrad。
角速度差分先求`A=(R_plus−R_minus)/(2ε) @ R_current.T`，取反对称部分的vee为world角速度列。
乘法反过来`R_current.T @ dR`表达site-frame角速度，不能仍标world。
本课site姿态随两角之和转动；平移和角速度两种Jacobian都核验，但不新增姿态控制。

局部预测：`Δp≈Jp Δq`，扰动该指[δ,−.4δ]rad，非线性余项通常O(δ²)。
中心差分截断误差O(ε²)，但ε过小使浮点相减舍入误差放大，不是越小越好。
open q2=0时两位置列平行，local (3,2) Jp rank1；bent/asymmetric为rank2。
它说明瞬时位置方向的限制，本课不求逆或实施IK；两DOF也不等于任意三维位置控制。

## Math-to-Code / APIs

`MjModel.from_xml_path`返回编译hand模型；`MjData(model)`返回可变state/buffers。
`mapping`具名解析joint/qpos/dof/actuator；tip也通过`model.site(name).id`独立解析。
`mj_forward(model,data)`原地刷新site_xpos(每site3,)与site_xmat(每site9,reshape3×3)，不推进time。
`mj_jacSite(model,data,jp,jr,site_id)`返回None，原地写world Jp/Jr(3,nv)；必须用当前FK状态。
`analytic(finger,q_pair)`返回解析world p/Jp局部两列/Jr局部两列/rotation；参数针对当前hand几何，不是通用机器人库。
`difference(...)`为probe分配独立MjData，逐joint qpos±ε，再用对应dof列装配；不会改变基准data。
速度使用非零canonical qvel=[.3,−.1,−.2,.4,.1,.2]rad/s，按dof地址赋值。
另外用q+h*qvel的微小离线位置变化核对Jp@qvel；h=1e−5s只是几何差分间隔，不是physics推进。
全部基准qpos/qvel检查保持不变，time=0；从未以ctrl修改关节或运行mj_step。

## Minimal Experiment / Expected → Actual

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/fingertip_jacobian.py
python examples/19_teleoperation_dexterous/fingertip_jacobian.py --delta .01
```

Modify仅把局部预测扰动δ=.001→.01rad，模型、基准配置与中心差分ε sweep不变。
预期线性预测误差约×100；finite_difference.csv不随δ改变。
每命令输出JSON9records、中心差分36×4CSV、预测9×8CSV、PNG，只ignored tmp/s18_7_delta*。

2026-10-10 DESKTOP-781D67A：WSL2 kernel6.6.87.2，同shell conda mujoco所属Python3.12.14；
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import路径与实际API通过，无安装/requirements改变。
两命令exit0；解析FK最大误差2.77556e−17m，解析Jp/Jr/rotation、全DOF差分、非零速度与非所属列0均通过。
默认/Modify最大局部预测误差2.54e−8/2.53999e−6m，即.0254/2.54µm，比值99.99975。

| ε rad | 最大Jp误差m/rad | 最大Jr差分误差 |
| --- | --- | --- |
| 1e−2 | 1.16666e−6 | 1.66666e−5 |
| 1e−4 | 1.16642e−10 | 1.66774e−9 |
| 1e−6 | 1.51668e−11 | 1.14743e−10 |
| 1e−8 | 9.31101e−10 | 8.70863e−9 |

本次ε1e−6更准，1e−8出现舍入误差回升；不称所有机器的最佳ε固定为1e−6。
错误frame对照把site-frame矩阵当world，最大Jp元素差.120594m/rad；错误索引以f0 actuator ids[1,4]代替dof[0,1]也明确失败。
这是误标矩阵/错选列对照，未执行错误控制器或真实机器人。
独立校验读取JSON/CSV，用compiled world hinge xaxis/xanchor的`screw column=a×(p−anchor)`交叉验证所有J列，核对rank、输出shapes与误差比；通过。
CLI δ0/nan/−1各exit2；3.11语法、两PNG目视、本地links/status/git diff --check通过。未使用3.11解释器执行。

## Explanation / Failure Cases

- state写入后忘mj_forward：site姿态缓存仍旧，差分和Jacobian会不匹配。
- 用actuator id选J列：数组维度相同不说明索引含义相同；乱序servo特意暴露问题。
- 把Jp认为site frame：返回行表达world方向，换到site需RᵀJp并相应旋转速度。
- 只验证一列/一指：漏掉第二hinge、其他指frame旋转和零列结构；本课覆盖所有六列、三个指、三配置。
- 将几何零列当动力学解耦：碰撞/惯性可以影响actual运动；J只描述当前configuration的瞬时几何映射。
- 离线FK写qpos合法，不表示该pose可无碰撞/无动力学约束执行；本课不进行路径或安全验证。

## Robotics Context

每指独立tip Jacobian是后续contact force与Jᵀ力矩映射的基础，但本课只确认几何接口。
要将多指结果组装成对象操作，还需object/contact/grasp frame与接触模型；不自动进入下一课。

## Interview Capsule

**30秒：**FK把该指两hinge角度变成world tip位置，mj_jacSite写(3,6) world位置/角速度Jacobian。
所属两dof列非零，其他指四列为零；解析两连杆和全列中心差分验证frame与索引。
JpΔq是局部线性预测，扰动十倍误差约百倍，不是任意大运动的精确位移。

**2分钟：**解释body/site/hinge轴、qpos与dof地址；给出world位置与局部两列公式、v=Jqvel单位；
比较解析API/中心差分及ε舍入权衡；用frame/actuator误用反例说明验证必要性，最后区分离线FK和实际动态执行。

## Must Remember

- J的列对应velocity DOF，不是actuator id。
- Jp/Jr返回world分量；位置是米，角速度是rad/s。
- 更新q后刷新FK；difference probes不改变基准状态。
- 几何依赖、动力学耦合和碰撞可行性分别讨论。

## My Verification — Run / Modify / Explain

状态只在根README；本人Run/Modify/Explain未确认前均保持空。
Run：默认执行，查看JSON own_dof_columns/J矩阵与PNG的ε误差曲线。
Modify：仅`--delta .01`，先预测linear error比例与FD曲线是否改变，再核对。
Explain：
1. 每指Jp为何是(3,6)，为什么只有两列非零？这些列应该由哪个地址选取？
2. Jp/Jr各是什么frame、单位？怎样从qvel得到tip速度与角速度？
3. 改qpos后为何必须mj_forward？本课为什么time始终0？
4. ε太大/太小分别有什么误差？δ扩大10倍为何预测误差约100倍？
5. 其他指J列为0为何不等于动力学解耦？离线FK核验不能证明哪些执行能力？

2026-10-10 本人确认实验、预测均完成，并提交五项Explain；根README Run/Modify/Explain全部完成，Learning Mastered。
解释精度：一般移动基座、物体/指间接触可使不同指动态耦合；本模型palm固定world，未接触时不同finger没有移动基座传递的惯性耦合。同指两个hinge仍共享连杆惯量；Jacobian零列只证明几何祖先依赖。
本次仅文档同步，未重跑数值/GUI；runtime沿用2026-10-10工程验证。
