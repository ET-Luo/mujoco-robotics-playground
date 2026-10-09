# S18.4 — Force feedback concept：反力的frame、sign与scale

约0.5～2h；前置：[S18.3](21_3_teleoperation_impedance.md)。
[代码](../examples/19_teleoperation_dexterous/force_feedback.py) / [示例入口](../examples/19_teleoperation_dexterous/README.md)。
Engineering / Learning唯一状态：[根README](../README.md#stage-18--teleoperation--dexterous-foundations)。

## Problem → Why → Intuition

机器人碰墙时的+world Y反力，应该在手柄坐标中显示成哪个方向？
先说清楚“谁作用于谁”，再转换坐标，最后决定反馈增益和限幅。
直觉：master向−X推使机器人向−world Y接近墙；墙把机器人向+Y推开，虚拟设备应向+master X推手，抵抗接近。
如果多加一次负号，会把阻力显示成助推力。

## Core Concepts / scope

复用S18.3 `run(.02, contact=True, local_feedback=True)`获得10s真实MuJoCo接触轨迹。
机器人1kHz本地阻抗、50Hz synthetic master、reference ZOH/vd0均保持；不是新控制器。
本课在仿真结束后将测力离线映射成master-frame虚拟力；没有真实haptic device、master动力学、人手运动反馈或力施加。
`robot.csv`原样保存；`feedback.csv`包含raw/capped/wrong-sign/delayed信号及假想功率；两PNG为时间曲线和坐标箭头。
依赖requirements已有mujoco/numpy/matplotlib；helper链force_feedback→teleoperation_impedance→incremental_reference/cartesian_spring/contact_transition，无新依赖。

输入明确为**environment-on-robot**，不是motor力或robot-on-environment；输出明确为**virtual device-on-hand**。
另一个合理定义是hand-on-device，它与device-on-hand反号。因此不能脱离作用对象说“反馈一定取负号”。
R_WM列为master坐标轴在world的方向，motion scale与feedback gain是不同参数。

## Mathematics：意义 / shape / unit / frame

```text
F_env_W                         # (3,) N，environment作用于robot，world
R_WM = [[0,-1,0],[1,0,0],[0,0,1]] # (3,3)，无量纲，M -> W
F_raw_M = g · R_WM.T · F_env_W   # (3,) N，virtual device作用于hand，master
β = min(1, F_cap / ||F_raw_M||₂) # scalar，zero时取1
F_virtual_M = β · F_raw_M
F_wrong_M = −F_virtual_M         # 本课作用对象定义下的错误对照
F_delayed_M[n] = F_virtual_M[n−80] # dt1ms，80ms离线延迟；起始历史为0
```

本例world力[F_x,F_y,F_z]对应master力[F_y,−F_x,F_z]。
墙只给+world Y，所以正确输出为+master X。先用R_WMᵀ完成W→M，再乘正gain，**不额外反号**。
gain默认.5、Modify2；无量纲。F_cap=1.5N是显示力norm界，与机器人motor20N cap不同。
一个β保持方向；逐轴clip在对角线可能让norm超过cap，也会改变方向。
平移力的换坐标只需旋转；完整wrench换参考点还需力矩/lever arm，本课不引入。

接近时scripted master速度v_M=[−.015,0,0]m/s；保持时0；退回时[+.015,0,0]。
假想功率`P_virtual=F_virtual_M·v_M`，scalar W。接近时正确力P<0、反号P>0：前者抵抗运动，后者助推。
退回时正确力也可能P>0；力的方向应来自接触物理，不能把每次正功率都判为符号错误。
这是用虚拟力与预设速度计算的**假想功率**，不是实测haptic能量或人手功率。

理想无约束、无延迟瞬时速度映射v_R=sR_WM v_M的对偶关系：

\[
F_{dual,M}=sR_{WM}^{\mathsf T}F_{env,W},\qquad
F_{dual,M}^{\mathsf T}v_M=F_{env,W}^{\mathsf T}v_R.
\]

用本课device-on-hand与environment-on-robot的定义，两者对应的功率数值相等；若定义相反端口作用力，端口功率符号也需改变。
任意feedback gain g不必等于motion scale s；本课s=1、g=.5/2，再加限幅与delay，不能声称理想功率相等。
即使理想对偶恒等式成立，也不是双边系统passivity/stability证明；actual机器人速度还受tracking/contact影响。

## Math-to-Code / APIs

`map_force(environment_force_W, rotation_WM, gain, cap)`是pure function：输入(3,)N、(3,3)、positive scalar gain和capN；
返回raw(3,)N、capped(3,)N、saturated bool，不修改输入。非有限/非法shape/frame/gain/cap拒绝。
`rotation.T @ force`将world分量写成master分量；`np.linalg.norm`给Euclidean长度，统一缩放β。
`run(gain)`先调用S18.3实际physics实验，再读current-command重解反力column14，构成[0,F_y,0]_W。
previous-command求解值只用于原课峰值诊断；不是另一时刻传感器采样，不能求和，也不把两次solve最大值当本课测力序列。

接触源复用Stage17 `measure`：`mj_contactForce(model,data,index,raw)`无新值返回，原地写(6,)contact-frame力N与力矩N·m。
contact.frame的**行**为world中的contact轴；`frame.T @ raw[:3]`转成world，再按geom顺序确定environment-on-probe sign。
`MjModel.from_xml_string`返回编译模型，`MjData(model)`返回可变实际状态；`mj_forward`原地刷新FK/动力学/contact，不推进time；
`mj_jacSite`写world Jp/Jr(3,nv)以算site速度；`mj_fullM`写物理惯量(nv,nv)用于检查；`mj_step`原地推进实际qpos/qvel/time1ms。
这些API在S18.3源与学习包详细解释，本课不改其调用或ctrl。虚拟力从未写入ctrl、xfrc_applied或qfrc_applied。

## Minimal Experiment / Expected → Actual

```bash
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/19_teleoperation_dexterous/force_feedback.py
python examples/19_teleoperation_dexterous/force_feedback.py --feedback-gain 2
```

2026-10-09 DESKTOP-781D67A：WSL2 Ubuntu24.04.5/kernel6.6.87.2，conda mujoco所属Python3.12.14；
MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2 metadata/import/API/helper路径核验，无安装/requirements改动。
两命令exit0；各robot10000×20 CSV、feedback10000×23 CSV、JSON/两PNG，只ignored tmp/s18_4_gain0.5与gain2。

| 量 | gain .5 | gain2 |
| --- | --- | --- |
| 当前contact solve输入峰N | 3.154064 | 3.154064 |
| 4–5s平均反力N | 1.998002 | 1.998002 |
| raw虚拟力峰N | 1.577032 | 6.308128 |
| capped虚拟力峰N | 1.5 | 1.5 |
| 4–5s平均master X虚拟力N | .999001 | 1.5 |
| 饱和samples | 1 | 2438 |
| 接近contact期间平均假想功率W | −.009988 | −.022468 |
| 反号对照平均假想功率W | +.009988 | +.022468 |
| 当前力归零后delayed仍非零samples | 80 | 80 |

浮点cap误差采用1e−12N容差。本课current solve峰3.154N与S18.3两次solve诊断峰3.158N不同，原因是取样语义不同，不是gain改变机器人。
两次运行**robot.csv全部列逐样本完全相同**。因此gain改变显示、不改变机器人反力或tracking。
四张PNG已目视；箭头蓝色为N，橙色只表示无量纲接近方向，不能比较二者长度为速度/力比例。

验证包含known-axis/斜向force norm与方向/zero/cap边界/输入不变/invalid guards；
独立CSV按[F_y,−F_x,F_z]公式核对frame/gain/cap/反号/80sample延迟/假想功率，核对接触动力学与Euler递推，
gain两运行robot全trace一致；理想dual power等式及独立display gain不匹配通过。
CLI gain0/nan/−1各exit2，3.11语法与本地links/git diff --check通过；未在Python3.11执行，未运行GUI。
独立校验初次用累计simulation time<3选接近样本，浮点使3s边界误入；改用记录的master速度<0选择运动区间后通过。
这是校验区间修正，未改动physics或反馈算法，不将初次失败计为PASS。

## Explanation / Failure Cases

1. 错用R_WM而不是R_WMᵀ：W→M方向错误，本例+Y会被映成−X；合法rotation也可能用错语义。
2. 测量robot-on-environment却当作environment-on-robot：输入先反号，显示会助推接近。
3. 正确信号又多加负号：把Newton第三定律的两侧作用力和“同一作用力换坐标”混淆。
4. 增益变大后显示平台不按比例变大：cap饱和把大小信息压平，不能从1.5N显示推断真实反力就是1.5N。
5. delayed显示仍非零：本例是固定80ms移位造成，不代表当前仍接触；未模拟网络抖动/丢包/真实设备。
6. 图上符号正确、norm有界，不证明双边闭环稳定。加入人、master动力学、执行器、采样和通信delay后需要额外分析。

## Robotics Context

这类映射用于remote操作的力提示、接触可视化、触觉控制前的信号检查。
真实haptic控制还需要设备calibration、力/速度/带宽限制与稳定性设计；本课仅建立作用对象、坐标和量纲概念。
图形反馈能传递接触信息，但人不会从屏幕曲线直接感受到物理力。

## Interview Capsule

**30秒：**输入environment-on-robot world力，用R_WMᵀ转为master，再乘显示gain并做norm限幅。
本例+world Y→+master X，抵抗−X接近；错误负号反而助推。
虚拟力只绘图，不施加到设备；任意gain/限幅/delay不能推出功率守恒或双边稳定。

**2分钟：**先定义作用对象，再解释R的列和变换方向；用known axis说明sign，
给gain与motion scale的区别、cap的饱和信息损失、delay造成旧接触显示；
用F·v区分接近阻力和退回正功，最后区分理想dual恒等式、假想功率与真实passivity证明。

## Must Remember

- 先说谁作用于谁，再说frame和sign。
- W→M用R_WMᵀ；力换坐标不自动增加负号。
- feedback gain不等于motion scale；norm cap不恢复信息，也不证明稳定。
- 虚拟力图、假想功率和真实haptics是不同证据。

## My Verification — Run / Modify / Explain

Learning仅由本人报告，状态只在根README；助手运行不代替本人Run。
Run：默认执行，打开feedback.png与force_frames.png，检查+Y→+X、接近时功率符号、延迟释放尾巴。
Modify：`--feedback-gain 2`，先预测raw峰、steady显示、饱和sample与robot trace是否改变，再对照CSV/JSON。
Explain：
1. 输入与输出分别是谁作用于谁？为什么本课正确反馈不额外加负号？
2. 为什么用R_WMᵀ？+world Y与+world X各对应master哪一轴、哪个符号？
3. gain从.5到2，为何稳态显示不是原来的4倍？是否改变机器人实际接触力？
4. 为什么接近时正确假想功率<0，但退回时可以>0？为什么这不是实测haptic功率？
5. 80ms延迟与norm限幅分别改变什么？为何它们和符号验证都不能证明双边passivity/stability？

2026-10-09 本人明确确认实验、预测均完成，并提交五项Explain；根README Run/Modify/Explain全部完成，Learning Mastered。
解释精度：真实弹性接触退回时可释放储能，正瞬时功率不等于主动产生能量；本课仅由虚拟力和预设master速度计算假想功率，未模拟master设备储能或验证向人手的真实能量传递。
本次只同步文档，未重跑数值实验或GUI；runtime沿用2026-10-09工程验证。
