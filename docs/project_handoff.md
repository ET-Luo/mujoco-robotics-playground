# 项目经验与进度交接

最后整理：2026-10-10（Stage16/17、S18.1–S18.7 Mastered；S18.8 Engineering Complete，Learning待本人验证）。
新会话先读 [AGENTS.md](../AGENTS.md) → 本文 → [README](../README.md) → 当次示例。
README 是 Stage 10 起 Engineering / Learning 的唯一状态来源。

## 当前边界与下一步

本人明确确认P0/P1完成，授权P2规划，已分别明确请求S16.1–S16.7实现。
已读取规则、README、交接、roadmap、P0最终trials/release与P1视觉/碰撞/ROS integration代码；
复用UR5e FK/J/IK/DLS、trajectory、gripper、camera/PnP/RGB-D/ICP/hand-eye、RRT、ROS2/TF2/URDF。
[P2审查与规划](p2_plan.md)记录去重与边界；Stage16–18逐Task0.5～2h，安排多处Integration。
S16.1 Code/Experiment/Docs完成；本人于2026-10-08明确确认实验与预测完成，
五项Explain准确覆盖ctrl/gear/限幅、负载力矩平衡、open loop/被动阻尼与验证边界，
Learning Run/Modify/Explain全部完成，Mastered；
具体命令/Modify/五问见[学习包](19_force_dynamics.md#my-verification--run--modify--explain)。
本次仅同步README、学习包、示例README、roadmap、P2 plan与本文；本地链接与git diff --check通过。
未重跑Python、仿真或GUI；runtime沿用2026-10-07工程验证。
**Stage16学习项全部完成；S17.1 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.2 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.3 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.4 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.5 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.6 Engineering Complete + Learning Mastered（本人确认Run/Modify并提交五项Explain；坐标/gate精度见学习包）；S17.7 Engineering Complete + Learning Mastered（本人确认Run/Modify并完成五项Explain）；S17.8a Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S17.8b Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S17.9 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；Stage17学习项全部完成；S18.1 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.2 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.3 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.4 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.5 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.6 Engineering Complete + Learning Mastered（2026-10-09 本人确认实验、预测并完成五项Explain）；S18.7 Engineering Complete + Learning Mastered（2026-10-10 本人确认实验、预测并完成五项Explain）；S18.8 Engineering Complete；Learning待本人Run/Modify/Explain；下一小任务S18.9等待明确请求。**

## S18.8 现场工程验证（2026-10-10，DESKTOP-781D67A）

本人明确继续请求；pwd正确，保留起始七份modified学习状态文档。WSL2 Ubuntu24.04.5/kernel6.6.87.2。
当前hook激活mujoco，同shell所属Python3.12.14；MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API/helper路径通过。
无安装/requirements改变；新增[contact_sensing.py](../examples/19_teleoperation_dexterous/contact_sensing.py)、[学习包](21_8_contact_sensing.md)。
hand.xml原文件不改，副本增加distal small/wide touch sites/sensors；fixed_probe为单固定sphere测量fixture，不是自由物体抓取。
`python examples/19_teleoperation_dexterous/contact_sensing.py`与`--tip-offset .03`各exit0，3case×4001×40CSV及JSONL/JSON/PNG。
no_contact全零；self_contact三pair/4980records，每指normal峰2.818674N，默认small/wide检测，移位small全0/漏4974force-qualified finger-samples。
fixed_probe仅f0_dist_geom/probe_geom、1729records、normal峰1.395928N，small两offset均0/漏1721sample，wide逐sample等normal。
两offset所有q/qvel/contact counts/normal/world F/wide/binary列逐sample相同；修改sensor位置不改变physics。
1.5s self每指normal2.244829N、netnorm1.944079N，全指world force相加0；probe netforce含切向量，touch是法向scalar。
每contact world sign/frame及point Jacobian投影对contact efc rows残差≤2.776e−17Nm，避免混入joint limits。
独立JSONL frame行组合/owner-sign重建/CSV/threshold/反作用力/probe归属/两offset对照六case通过。
CLI nan/−1/.01 exit2、builder guards、3.11语法/两PNG目视/本地links/status/git diff --check通过；产物只ignored tmp/s18_8_*与/tmp日志。
官方XML touch语义已核对：同body/site区域或normal-ray纳入的法向力和，非vector/压力；sensor_adr读值。
3.11未实际执行；无GUI/真实sensor误差/自由object/grasp或安全证明；inactive candidate/condim6防御分支未专门注入。
README Code/Experiment/Docs完成，Learning三项未确认；下一步默认Run、Modify tip_offset.03、学习包五问；不自动推进S18.9。

## S18.7 现场工程验证（2026-10-10，DESKTOP-781D67A）

本人明确继续请求；pwd正确/git status空。WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前hook激活mujoco，同shell所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API与hand_fixture路径通过；无安装/requirements改变。
新增[fingertip_jacobian.py](../examples/19_teleoperation_dexterous/fingertip_jacobian.py)、[学习包](21_7_fingertip_jacobian.md)，不修改hand模型/helper。
`python examples/19_teleoperation_dexterous/fingertip_jacobian.py`及`--delta .01`各exit0，9records/36×4FD CSV/9×8prediction CSV/PNG。
三个配置×三site：解析world p/R/Jp/Jr与MuJoCo一致，FKmax2.776e−17m；所属两dof列非零，其余四列零。
全DOF中心差分epsilon1e−2/4/6/8，Jp max errors1.167e−6/1.166e−10/1.517e−11/9.311e−10m/rad；极小epsilon舍入回升。
非零qvel的world v/omega及独立q+hqvel位移核验通过；probe独立MjData、基准qpos/qvel不变、time0，无mj_step。
delta.001/.01最大线性预测误差.0254/2.54µm，ratio99.99975；wrong frame误差.120594m/rad、actuator id误选列明确失败。
独立JSON/CSV与compiled world轴/anchor的cross公式检查全部site/列与open rank1/bent rank2通过。
CLI delta0/nan/−1 exit2；3.11语法/两PNG目视/本地links/status/git diff --check通过；产物只ignored tmp/s18_7_*。
3.11未实际执行；纯离线FK/J，无GUI/实际tracking/碰撞可行性/IK或硬件验证；open差分可超限，仅数学probe。
本人随后确认实验、预测并完成五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充一般移动基座耦合与本模型固定palm/no-contact指间惯性独立的区别；同指惯性与指间接触耦合仍可存在。
本次仅同步七份文档，links/状态/git diff --check通过；未重跑数值/GUI，runtime沿用2026-10-10工程验证。
下一小任务S18.8等待明确请求，不自动实施。

## S18.6 现场工程验证（2026-10-09，DESKTOP-781D67A）

本人明确继续请求；pwd正确，保留起始八份modified及S18.4/5四份untracked工作。
WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前hook激活mujoco，同shell所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API通过；无安装/requirements改变，无local helper。
新增[hand.xml](../examples/19_teleoperation_dexterous/hand.xml)、[hand_fixture.py](../examples/19_teleoperation_dexterous/hand_fixture.py)、[学习包](21_6_hand_fixture.md)。
固定palm、三radial指各2hinge/40+30mm capsule；nq/nv/nu6；gear1 position kp.25/kv.02/torque cap.08Nm，actuator ids[1,4,5,2,3,0]。
`python examples/19_teleoperation_dexterous/hand_fixture.py`及`--close-scale 2`最终各exit0、4case×4001×48CSV。
默认cycle无contact/close error4.901e−7rad；三single case其余两指全程不动，selected prox2s .249999877rad。
scale2 raw1.1/1.3→clip.9/1.2rad，actual2s .765464/1.200241rad；active contact1660sample/max3，三对different finger distal geom。
clipped7548joint sample/max depth.254095mm/max soft range violation.001939rad；无actuator饱和；尾.5s open error.000164/.000327rad，两组重新张开。
首版mount overlap三对−4mm初始contact导致隔离失败；显式排除palm/prox mounting pair修复，不关指间collision。
默认joint limit下scale2越界.0165633rad超预设.005rad容差；调整专用solreflimit.005/solimplimit.99后通过。
第一次误用geom solref属性导致XML schema拒绝；核对官方XML文档改joint属性名后通过；上述失败不计PASS，最终重跑两完整实验。
static正确/naive ctrl0→f2_dist/实际cap探针、固定palm、完整动力学残差≤6.73e−16Nm、独立compiled mapping/servo/CSV验证八case通过。
CLI scale0/nan/−1 exit2、3.11语法、四PNG目视、本地links/git diff --check通过；产物只ignored tmp/s18_6_*与/tmp日志。
3.11未实际执行；几何图仅中心线非GUI；hand-only无object/touch sensor/Jacobian/抓取/硬件安全验证。
本人随后确认实验、预测并提交五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充本模型kv速度反馈、±.08Nm actuator cap，以及passive damping与actuator输出的区别。
本次仅同步七份文档，links/状态/git diff --check通过；未重跑数值/GUI，runtime沿用2026-10-09工程验证。
下一小任务S18.7等待明确请求，不自动实施。

## S18.5 现场工程验证（2026-10-09，DESKTOP-781D67A）

本人明确继续请求；pwd正确，保留起始六份modified文档和S18.4两份untracked文件。
WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前hook激活mujoco，同shell所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API/helper路径通过；无安装/requirements改动。
新增[teleoperation_integration.py](../examples/19_teleoperation_dexterous/teleoperation_integration.py)、[学习包](21_5_teleoperation_integration.md)。
S18.2 map_sample仅增加可选lower/upper，旧默认/.5回归exit0；复用S18.1/3/4与Stage17 fixture/contact。
`python examples/19_teleoperation_dexterous/teleoperation_integration.py`及`--dropout-duration 1.2`各exit0、三case13000×32CSV。
50Hz master/1kHz local与physics；approach/contact/slide/clutch/recenter/retreat，simulation lease.1s；无新包即时保持旧ref，stale后首包rebase。
nominal末X25mm/box裁剪24sample；gap.6/1.2正确末X23.8/17.8mm，accepted619/589、stale519/1119sample。
最后issued6.38s，6.481s过期，7/7.6s恢复首帧ref delta0；6.6expired/6.62duplicate包均拒绝且不刷新lease。
错误对照恢复+.4mm，末X24.2/18.2mm；限速/末tracking通过但恢复no-motion契约False，未持续补旧运动。
所有case contact4–8s连续、平均1.998002N/峰3.158396N、无motor饱和；末1s tracking<1µm/speed<.001mm/s并分离。
stale期间actual X仍变约.629/.630mm，hold不是冻结qpos或卸力。
独立CSV controller/cap/动力学/Euler/ref/lease/rebase/virtual/contact/recovery检查六case通过；CLI0/nan/−1 exit2、packet guards、3.11语法/两PNG/links/git diff --check通过。
产物仅ignored tmp/s18_5_*与/tmp日志；3.11未实际执行，无GUI/keyboard/ROS/网络/设备/超力保护/硬件安全或双边稳定性验证。
本人随后确认实验、预测并提交五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充：错误恢复只应用限幅后.4mm额外位移，其余丢弃；rebase重建master差分锚点，保留ref而不重设actual state。
本次仅同步七份状态文档，links/状态/git diff --check通过；未重跑数值/GUI，runtime沿用2026-10-09工程验证。
下一小任务S18.6等待明确请求，不自动实施。

## S18.4 现场工程验证（2026-10-09，DESKTOP-781D67A）

本人明确继续请求；pwd正确，保留起始七份modified状态文档。WSL2 Ubuntu24.04.5/kernel6.6.87.2。
当前hook激活mujoco，同shell所属Python3.12.14；MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/import/API/helper路径通过。
无安装/requirements改动；新增[force_feedback.py](../examples/19_teleoperation_dexterous/force_feedback.py)、[学习包](21_4_force_feedback.md)。
复用S18.3 contact/local/.02run；current-command solve environment-on-robot world force→gain R_WMᵀ→norm cap1.5N虚拟device-on-hand显示。
`python examples/19_teleoperation_dexterous/force_feedback.py`与`--feedback-gain 2`各exit0；robot10000×20/feedback10000×23CSV、JSON/两PNG。
反力current solve峰3.154064N（S18.3双solve诊断峰3.158396N）；hold1.998002N；raw峰1.577032/6.308128N。
显示峰均1.5N、hold .999001/1.5N、饱和1/2438sample；两gain机器人全CSV逐sample相同。
正确+MX力抵抗−MX接近、反号助推，接近假想功率−.009988/−.022468W、反号正；80ms延迟离开后残留80sample。
known axes/oblique norm方向/zero/cap/input不变/invalid guards、独立CSV frame/cap/delay/power/physics、ideal duality通过。
独立校验初次用累计time<3误含3s浮点边界静止帧；改用记录master velocity<0选择接近区间后通过，算法/physics未改。
CLI gain0/nan/−1 exit2；3.11语法、四PNG目视、本地links/git diff --check通过。产物只ignored tmp/s18_4_*及/tmp日志。
Python3.11未实际执行；离线显示不写任何device/robot力，无GUI、人手/master动力学、网络、双边passivity/stability验证。
本人随后明确确认实验、预测并提交五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充：弹性储能释放可解释真实退回正功，但本课假想功率不证明设备储能或向人手真实传能。
本次仅同步七份状态文档；links/状态/git diff --check通过，未重跑数值实验或GUI，runtime沿用2026-10-09工程验证。
下一小任务S18.5等待明确请求，不自动实施。

## S18.3 现场工程验证（2026-10-09，DESKTOP-781D67A）

本人明确继续请求；pwd正确，保留起始六份modified文档与S18.2两份untracked文件。
WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前hook激活mujoco，同shell所属Python3.12.14核验。
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata、实际import路径/API及local helper路径通过；无安装/requirements改动。
新增[teleoperation_impedance.py](../examples/19_teleoperation_dexterous/teleoperation_impedance.py)与[学习包](21_3_teleoperation_impedance.md)。
复用Stage17 XY slide/motor/墙面contact及S18.1增量mapper；本fixture anchor[0,.06,0]m/box Y .015–.09m。
`python examples/19_teleoperation_dexterous/teleoperation_impedance.py`及`--master-period .1`各exit0、三case×10000×20CSV。
master20/100ms，position ZOH/vd0；local每1ms算反馈，physics两case频率均1kHz；同进程整数调度，不是实时/多线程/ROS。
local master更新499/99、反馈10000；contact反力峰3.158396/3.712520N、接触delay .097/.120s、hold1.998002N；final tracking通过，无饱和。
相对理想连续目标早期lag1.6425/2.2425mm；相对held目标约1.5mm，区分采样与动力学滞后。
packet-feedback对照计算500/100次：50Hz恢复；10Hz末error136.728mm/speed2.089m/s失败、峰445.824N、饱和4900joint samples。
独立CSV检查调度/三个case同ref/触墙前state/反馈或held力/cap/动力学/Euler递推/静态反力/退回无contact通过。
无接触未饱和线性谱半径h1/20/100ms=.982635/.746053/4.259467，解释10Hz反馈失败但非全局稳定性证明。
CLI周期0/nan/.03均exit2、schedule guards、3.11语法与两PNG目视、本地links/git diff --check通过；产物仅ignored tmp/s18_3_*及/tmp日志。
Python3.11未实际执行；无GUI/UR5e/设备/网络延迟/lease/硬件安全验证；motor cap不约束碰撞反力。
本人随后确认实验、预测并提交五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充2N弹簧静态平衡、contact诊断非显式力反馈、vd0设计选择与499→99初始化计数。
本次仅同步七份文档，本地links/状态/git diff --check通过；未重跑数值/GUI，runtime沿用2026-10-09工程验证。
下一小任务S18.4等待明确请求，不自动实施。

## S18.2 现场工程验证（2026-10-09，DESKTOP-781D67A）

本人明确继续请求；起始pwd正确/git status空。WSL2 Ubuntu24.04.5/kernel6.6.87.2。
当前conda hook激活mujoco，同shell核验所属Python3.12.14；NumPy2.5.3/Matplotlib3.11.2 metadata/import路径/API通过。
无安装/requirements变更；新增scaling_clutch.py与[学习包](21_2_scaling_clutch.md)，复用S18.1 helper。
`python examples/19_teleoperation_dexterous/scaling_clutch.py`及`--fine-scale .5`各exit0、450×19CSV。
scale只作用新增量；clutch inactive消费样本/reengage rebase；设备坐标reset标记rebase。
inactive与事件保持；首恢复delta worldY .04/.10mm，sample norm峰20mm/s（1e−12容差），box裁剪10/25sample。
解析九phase端点/反向第一帧释放/invalid scale与inactive nan拒绝通过；错误原始增量仅解析公式对照。
产物仅ignored tmp/s18_2_scale*；PNG目视、3.11语法、本地links/git diff --check通过。
Python3.11未实际执行；无actual robot/controller/GUI/device/连续导数/安全验证。
本人随后明确确认实验、预测并提交五项Explain；README Run/Modify/Explain全部完成，Learning Mastered。
学习包补充：增量scale切换无需额外rebase；遗漏rebase的错误增量限幅后部分应用，其余丢弃，不追赶完整错误目标。
本次仅同步六份文档；本地链接、状态一致性与git diff --check通过，未重跑Python实验或GUI。
Runtime沿用2026-10-09工程验证；下一小任务S18.3等待明确请求，不自动实施。

## S18.1 现场工程验证（2026-10-09，Zero）

本人明确请求；起始pwd正确/git status空，WSL2 Ubuntu24.04.5/kernel6.18.33.2。
当前conda hook激活mujoco，同执行shell核验环境与所属Python3.12.14；NumPy2.5.3/Matplotlib3.11.2 metadata/实际imports/module paths通过。
无安装或requirements变更；仅两包、无local helper，不执行MuJoCo/ROS，未在Python3.11运行（语法解析通过）。
新增[incremental_reference.py](../examples/19_teleoperation_dexterous/incremental_reference.py)、[学习包](21_1_incremental_teleoperation.md)、新示例README。
相邻master位移→R_WM world+90°Z转轴→norm限幅→world box投影；纯reference，无actual robot/qpos/control。
固定mapper scale1，dt20ms，v_sample norm cap20mm/s，world anchor[-.45,.20,.30]m，half widths[.04,.04,.02]m。
`python examples/19_teleoperation_dexterous/incremental_reference.py`与`--master-amplitude 2`各exit0、550×25CSV。
requested峰80/160mm/s，applied reference峰20mm/s（浮点容差1e−12）；speed-limited100/100、workspace-limited50/100sample。
首workspace裁剪3.02/2.02s；4.02s反向第一帧world Y−.2/−.4mm，立即释放，不保存拒绝增量欠账。
末bounded参考[-.47,.22,.31]/[-.49,.20,.32]m；unbounded末Y.36/.52m，预期超速/越界。
常量master offset不变/对角线norm与方向/斜向边界投影及nonfinite/dt/rotation/outside initial拒绝、输入不变静态checks通过。
`python tmp/s18_1_validation/check.py`独立轴公式/norm/box投影/累加/解析milestones、绝对锚定裁剪粘界对照及3.11语法通过。
CLI amplitude0/nan各exit2，两PNG目视、本地Markdown links/git diff --check通过；产物仅ignored tmp/s18_1_*及/tmp日志。
只证明离散样本reference约束；无连续导数/加速度界、实际可达/避碰/速度/contact/GUI/device/安全验证。
本人随后明确确认实验与预测完成，并准确回答五项Explain；README Learning三项完成，Mastered。
学习包记录frame/锚点、norm与workspace投影、拒绝增量丢弃及离散reference验证边界。
本次仅同步七份状态文档；本地Markdown文件链接与git diff --check通过，未重跑Python示例或图形生成。
Runtime沿用2026-10-09工程验证；S18.2可调scale/clutch/recenter与S18.3实际控制未实施，下一小任务S18.2等待明确请求。

## S17.9 现场工程验证（2026-10-09，Zero）

本人明确继续请求；保留起始七份S17.8b staged文档，后续会话内Git已有eb8e9f8 worker/interfaces提交，未撤回其工作。
pwd正确，WSL2 Ubuntu24.04.5/kernel6.18.33.2；当前hook激活mujoco/所属Python3.12.14与ROS空conda/systemPython3.12.3/Jazzy同shell分别核验。
MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2/Menagerie2026.9.2 metadata/真实imports/module paths通过；
ROS apt rclpy7.1.12/std-srvs5.3.8/action-msgs2.0.4/ament-cmake2.5.6/rosidl1.6.1，cmake/make/g++/apt NumPy已有，dpkg --audit空。
无安装/requirements改动。新增[学习包](20_9_worker_ros_monitoring.md)、control_worker/worker_ros/run_worker_ros与s17_interfaces0.0.1/build。
接口build/type support/实际imports通过，生成物只ignored tmp/s17_9_interfaces；ROS字段大小写和固定数组JSON准备失败已修复，未计PASS。
同S17.8b UR5e控制/模型，独立owner每1ms ctrl+mj_step；名义10ms wall batch非实时，不接收tick RPC；启动idle也推进physics。
reference单槽与最多8条生命周期queue；同机monotonic lease、generation/sequence/固定描述校验，Action begin不重设actual q/time；single goal perworker。
六launcher default、--command-delay .2/.7、--stop-after6、--cancel-at6及--cancel-at6 --pause-check均验证预期exit0。
default/.2 SUCCEEDED，terminal snapshot elapsed12.040/12.050s，扫描切向max.352200mm/force max.048350N，全contact/无饱和。
.7 ABORTED command_timeout/elapsed.500s/accepted0；stop6 ABORTED/6.560s；cancel两组CANCELED/6.080s，均ACK/applied。
所有终态后.25wall秒快照sim增加.24… .25s；成功仍约1.999954N力hold，stop/cancel检查时力0N。
pause qpos/qvel/time逐值相同，resume继续physics，不恢复旧任务；state11…249条/feedback11…241条真实ROS收发。
独立audit_worker.py无ROS/无tick physics、partial socket隔离、重复序号/future/stale/generation/descriptor拒绝、active pause拒绝、cancel/旧包不可重启/one-shot拒绝通过。
numerical.py六48列CSV完整q/v递推和每case24…146actual快照FK/J/bias+motor/contact/qacc/单步replay通过；max动力学残差≤1.688e−14Nm。
CLI delay nan/period0/cancel−2/stop20各exit2，无server exit3；两源码3.11语法、bash -n、分析PNG目视/本地links/git diff --check通过。
产物只ignored tmp/s17_9*及/tmp日志，正常清理无worker/bridge遗留；未在Python3.11运行。
未专门注入成功/取消竞态、worker crash/IPC应用超时或高负载；无GUI/硬件/跨机/长期稳定性或硬实时保证。
本人随后明确确认实验与预测完成并提交五项Explain，核心解释准确；README Learning三项完成，Mastered。
学习包补充owner主线程、成功后2N hold与cancel/timeout锁存actual pose并退出力外环、arrival单槽与序号应用校验边界。
Stage17学习项全部完成；本次仅同步七份状态文档，本地Markdown links与git diff --check通过。
未重跑Python、ROS通信、仿真或GUI，runtime沿用2026-10-09工程验证；不自动启动Stage18。

## S17.8b 现场工程验证（2026-10-09，Zero）

本人明确请求；保留起始七份S17.8a未提交状态文档，pwd正确；WSL2 Ubuntu24.04.5/kernel6.18.33.2。
当前conda hook激活mujoco，同shell核验环境/所属Python3.12.14；MuJoCo3.13.0/NumPy2.5.3/
Matplotlib3.11.2/Menagerie2026.9.2 metadata、实际imports/module paths/API通过，无安装/requirements改动，未在3.11运行。
新增[ur5e_surface_following.py](../examples/18_compliant_control/ur5e_surface_following.py)与[学习包](20_8b_surface_following_ur5e.md)。
裸UR5e+50g/r20mm sphere probe，attachment local+Z偏置40mm；只sphere-plane碰撞开启，无arm/self避碰保证。
surface t/n/b=world+X/+Z/−Y，origin[-.45,.20,.22]m；private DLS IK20updates到球心[-.45,.20,.25]m，
ep.164044μm/er4.605e−8rad/time0；不验证home→initial动态运动。执行仅motor+actual mj_step。
Cartesian K[1000,400,1000]/D[80,40,80]、orientation K60/D12，经新site Jp/Jr转joint torque，加full simulated bias，不取消contact。
approach10mm/s、force gain.006/参考速度cap20mm/s、dt1ms/implicitfast，关节caps150前三/28后三Nm、gear[1,2,1,1,1,1]。
`python examples/18_compliant_control/ur5e_surface_following.py`与`--leg-duration 1`最终各nominal/loss两case、exit0。
T2 nominal taskTrue，T1预期taskFalse：切向max error.352200/1.462038mm，normal max error.048350/.281013N，
contact100%、binormal.116618/.501542mm、orientation.656481/2.336228mrad、无饱和。
actual site norm速度峰28.9359/63.2352mm/s、joint最大分量峰.068096/.152785rad/s，nominal motor峰≤18.095460Nm；
末mean力1.999749/1.999989N，初始touch1.005s/input峰6.400016N/current峰5.235051N。
loss均6s合成plane下移.1m→contact_lost，锁存actual位置/姿态/零ref速度，无scan_complete；physics至12s，末力0。
初版30mm/s输入峰约19.6N退出；大initial gap在5s load_not_ready；原gain.003 T2 force error.069206N不合格。
最终减gap到10mm并慢接近、gain调.006，未放宽验收/退出门限；T1完整记录后的离线task失败不同于online critical guard退出。
每步motor/gear/contact Jᵀ/power/完整动力学通过，max residual≤3.161e−12Nm。
`python tmp/s17_8b_validation/check.py`四CSV control/ref/limit/failed锁存与每case246actual快照FK/J/bias/input/current测力/qacc、
contact-point applyFT及actual单步replay通过；jacobian_checks.py全q/v递推、数值Jp/Jr/40mm偏置/masks/mass/radius/ranges/静态motor饱和通过。
CLI duration0/nan各exit2，两PNG目视/本地Markdown links/git diff --check通过；产物只ignored tmp/s17_8b_*及/tmp日志。
无GUI/ROS/摩擦/未知曲面/真实force sensor/硬件或通用稳定性验证，未动态注入超速/关节越界/饱和恢复/搜索超时。
本人随后明确确认实验与预测完成，并准确回答五项Explain；README Learning三项完成，Mastered。
学习包补充gear=2的±75属于actuator force限制、ctrl限幅关闭，以及刚体偏置改变Jp而不改变Jr。
本次仅同步七份状态文档；本地Markdown links与git diff --check通过。
未重跑Python示例、仿真或GUI，runtime沿用2026-10-09工程验证；S17.9等待明确请求。

## S17.8a 现场工程验证（2026-10-09，Zero）

本人明确请求；起始pwd正确/git status空，WSL2 Ubuntu24.04.5/kernel6.18.33.2。
conda info --base定位hook并激活mujoco，同执行shell核验环境/所属python；
Python3.12.14/MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2 metadata、实际imports/module paths/API通过。
无安装/requirements变更，未在Python3.11运行。新增[surface_following.py](../examples/18_compliant_control/surface_following.py)
与[学习包](20_8a_surface_following_fixture.md)，复用S17.7/normal_force/helper三包链。
同无摩擦XY装置，approach→load→两段quintic30mm往返→hold；失联/超力/requested超cap锁存actual reference。
`python examples/18_compliant_control/surface_following.py`与`--leg-duration 1`最终各四case/exit0。
正常扫描5…9/5…7s，切向max error.204596/.740687mm，normal max error.035781N/contact100%，
末1s平均力1.999964N，实际切向峰速度28.6304/59.3115mm/s，无饱和。
6s合成移除plane与2mm plane step分别触发contact_lost/force_budget_exceeded；
超力input/current峰22.500547/20.563787N，末尾反力.801186N，不称退出即清零。
6s切向cap降.01N，T2在6.048s、T1在6s触发motor_saturation，均无scan_complete。
初版误要求注入与饱和退出同刻而assert失败，修正时序假设后通过，正常门限不变。
失败后锁存reference/速度零，physics继续至12s；T2饱和5953sample、actual末位置97.63mm，
T1饱和1655sample，证明reference停止不保证actual就地停止。
每步J/M/power/motor/contact/动力学与完整q/v半隐式递推通过；
`python tmp/s17_8a_validation/check.py`独立八CSV control/cap/dynamics/递推/quintic/normal反馈、
失败reason/reference与六类landmark actual状态恢复后的direct contact核验通过；两正常normal列≤1e−10相同。
CLI duration0/nan各exit2，默认PNG目视/本地Markdown links/git diff --check通过。
产物仅ignored tmp/s17_8a_*和/tmp日志。未注入search timeout/load-not-ready；
无GUI/UR5e/ROS/摩擦/未知曲面/硬件或通用稳定性验证。
本人随后明确确认实验与预测完成，并准确回答五项Explain；README Learning三项完成，Mastered。
学习包补充P力误差到reference速度的admittance式外环与完整虚拟动力学的区别；
phase4退出2N反馈并锁存actual位置，不把局部控制称为已验证的硬件安全停止。
本次仅同步七份状态文档；本地Markdown文件链接与git diff --check通过。
未重跑Python示例、仿真或GUI，runtime沿用2026-10-09工程验证；下一小任务S17.8b等待明确请求。

## S17.7 现场工程验证（2026-10-08，DESKTOP-781D67A）

本人明确请求；pwd正确，保留起始七份S17.6未提交状态文档；读取规则/交接/示例README/source/requirements/roadmap。
WSL2 Ubuntu24.04.5/kernel6.6.87.2，当前conda hook激活mujoco，同shell核验环境/所属python。
Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/真实imports/module paths/API通过，
无安装/requirements改动，未在Python3.11执行。新增[hybrid_control.py](../examples/18_compliant_control/hybrid_control.py)
与[学习包](20_7_hybrid_position_force.md)；复用normal_force/模型/measure/helper三包链。
同无摩擦XY装置，surface t/n/b=world X/Y/Z，显式R变换与Sp diag100/Sf diag010；normal target2N。
reference层hybrid：tangent quintic reference+normal force-generated reference互斥投影后交inner motor，非直接wrench叠加。
`python examples/18_compliant_control/hybrid_control.py`与`--distance .06`各2case exit0。
5…7s单次30/60mm位移，不实施连续扫描；0…12s/12001×23CSV。
hybrid运动切向max error.204596/.409191mm，normal max error.035781N，contact100%；tail位置差<1e−9mm。
normal tail均值1.999964N/max error.000057872N；actual tangent peak28.6304/57.2607mm/s，
Xmotor峰.090772/.181543N，Ymotor1.999980N，无饱和；current反力峰4.663290N/input6.358974N。
normal_only保持力但位置goal误差30/60mm，task False；hybrid两任务合格。
frame正交/normal几何对应/selector对称幂等互斥、wrong normal/overlap/left-hand静态拒绝、反馈sign probe通过。
每步J/Jr/M/power/motor cap/contact Jᵀ/动力学/零bias与passive/external及actual半隐式递推通过。
独立CSV quintic/outer+inner控制/projection与动力学、恢复初始/峰值/中点/末尾contact核验通过。
两mode及两distance的normal相关字段≤1e−10相同；static selector无关轴排除/非恒等旋转功率核验通过。
CLI distance0/nan各exit2；默认PNG目视与本地Markdown links/git diff --check通过。
产物仅ignored tmp/s17_7_d0.03/d0.06、临时日志/tmp；无GUI/UR5e/ROS/动态斜平面/硬件或通用解耦保证。
正常case未触发contact-loss；失联策略runtime沿用2026-10-08 S17.6，未称本轮重测通过。
本人随后确认实验与预测完成并正确回答五项Explain；根README Learning三项完成，Mastered。
学习包补充reference严格缩放与本fixture实际误差缩放的条件，不把selector当通用动力学解耦。
本次保留起始全部modified/untracked工作，仅同步七份文档；本地文件链接/git diff --check通过。
未重跑Python示例/仿真/GUI，runtime沿用2026-10-08工程验证；S17.8a等待明确请求。

## S17.6 现场工程验证（2026-10-08，DESKTOP-781D67A）

本人明确请求；pwd正确，保留起始七份S17.5未提交文档；读取规则/交接/示例source与README/requirements/roadmap。
WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前conda hook激活mujoco，同shell核验环境/所属python。
Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/真实import路径通过；
无安装/requirements改动，未在3.11执行。新增[normal_force.py](../examples/18_compliant_control/normal_force.py)
与[学习包](20_6_normal_force.md)，复用S17.4模型/measure/helper三包依赖。
active pair且反力>.05N gate；P力误差→reference速度（gain.003m/(Ns)、cap20mm/s）→5…60mm投影
→原K400/D[56.568542,40]motor；reference积分存在，无额外PI I-state；dt1ms/joint cap20N。
`python examples/18_compliant_control/normal_force.py`与`--target-force 4`最终各4case exit0。
closed7…8s平均反力1.997421/3.994886N、max error.004202/.008332N；接触100%、速度<2.3e−8m/s。
current peak4.663290/4.899475N、input检测peak6.358974N；reference末14.999035/9.998001mm。
fixed两命令动力学/控制/测力相同，始终1.998002N，4N目标误差2.001998N，未称闭环通过。
loss3s几何撤墙并当步contact_lost，1666loop采样后禁用/reference锁存/local hold继续mj_step；
absent无可达墙、force loop0、3s search_timeout；末contact force0，均tracking qualified False。
初版6s/4N尾窗口error.070953N未通过，contact正常；最终统一8s并将reference下界由10改5mm，
保留.05N门限，使4N平衡reference位于范围内部；不把初次失败算PASS。
每步J/Jr/M/actual cap/contact Jᵀ映射/动力学/零bias/passive/external、clock/shape/finite与q/v递推通过。
独立CSV outer/inner控制与projection、gate、force balance/depth、恢复四种状态contact核对通过；
静态反馈sign/超大误差speed cap、CLI target0/nan exit2通过，4N PNG已目视检查。
本地Markdown链接/git diff --check通过；产物仅ignored tmp/s17_6_f2/f4、日志/tmp。
动态motor/reference未饱和；无GUI/UR5e/ROS/硬件安全停止、noise/delay或通用稳定性验证。
本人随后确认实验与预测完成并提交五项Explain，核心逻辑正确；README Learning三项完成，Mastered。
学习包记录精度补充：本人假设X法向的符号逻辑应映射到实际world Y，本fixture零gravity，
gate为active pair+反力门限，timeout/bounds为独立保护；未把通用PI抗饱和建议称为已验证。
本次起始git status空，仅同步七份文档；本地文件链接/git diff --check通过。
未重跑Python示例/仿真/GUI，runtime沿用2026-10-08工程验证；S17.7等待明确请求。

## S17.5 现场工程验证（2026-10-08，DESKTOP-781D67A）

本人明确请求；pwd正确，保留起始全部modified/untracked S17.3/S17.4工作。
读取规则/交接/示例README/source/requirements/roadmap；WSL2 Ubuntu24.04.5/kernel6.6.87.2。
当前conda hook激活mujoco，同执行shell核验环境/所属python；Python3.12.14/MuJoCo3.14.0/
NumPy2.5.3/Matplotlib3.11.2 metadata/真实imports/module paths通过；无安装/requirements改动，未在3.11运行。
新增[admittance.py](../examples/18_compliant_control/admittance.py)与[学习包](20_5_admittance.md)。
合成world +Y测力仅驱动软件Mv/Dv/Kv；未施加物理外力；同XY装置真实motor inner loop跟踪生成reference。
`python examples/18_compliant_control/admittance.py`与`--virtual-mass 2`各6case/exit0，无import error。
Mv1/2kg，Dv4Ns/m，Kv20或0N/m；dt1ms，参考bounds40mm/50mm/s，innerK400/真实mass不变。
正负.4N脉冲逐点镜像；Mv1/2正reference峰24.1494/27.0143mm，峰速度51.449/41.721mm/s。
有K的.2N恒定bias末reference10mm；无K/无限制漂移487.55/475.05mm，末速度50mm/s。
有界bias到40mm停住；4N强脉冲触发速度/位置projection，reference≤40mm/50mm/s，撤力后释放趋回。
strong bounded实际速度峰56.794/56.737mm/s，tracking error峰.922834/.922793mm；界只保证reference。
各组motor无饱和，峰≤2.010896N；实际qpos/qvel始终由mj_step推进，未重设。
每步J/Jr、motor cap/qacc/零bias/passive/contact/external通过；time/shape/finite与实际半隐式递推通过。
四无界case独立连续阶跃/脉冲解max error≤.050mm；独立CSV虚拟递推/inner control/
动力学/能量/limit flags通过；±边界outward拒绝/inward释放静态probe通过。
CLI mass0/nan各exit2；默认PNG已目视检查；本地Markdown链接/git diff --check通过。
产物只ignored tmp/s17_5_m1/m2，临时日志/tmp；无GUI/实际测力或外力/contact/UR5e/ROS/硬件闭环验证。
本人随后确认实验与预测完成并正确回答五项Explain；根README Learning三项完成，Mastered。
学习包补充ω_n/ζ的相对缩放、投影后速度以实际reference增量回写及一般状态的净力方向。
本次起始git status空，仅同步七份文档；本地文件链接与git diff --check通过。
未重跑Python示例/仿真/GUI，runtime沿用2026-10-08工程验证；S17.6等待明确请求。

## S17.4 现场工程验证（2026-10-08，DESKTOP-781D67A）

本人明确请求；pwd正确，保留起始七份S17.3未提交状态文档。读取规则/交接/示例README与source/requirements/roadmap。
WSL2 Ubuntu24.04.5/kernel6.6.87.2；当前conda hook激活mujoco，同执行shell核验环境与所属python。
Python3.12.14/MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata/真实import/module paths/API通过；
无安装/requirements变更，未在Python3.11运行。新增[contact_transition.py](../examples/18_compliant_control/contact_transition.py)
与[学习包](20_4_contact_transition.md)，复用S17.1 XML副本/常量，加+Y平面/球condim1 pair。
K400/D[56.568542,40]、dt1ms、joint cap20N；solref.01/1、solimp.95/.95/.001；
approach→active pair+反力>.2N触发touch→.2s reference过渡→固定reference .015m hold。
`python examples/18_compliant_control/contact_transition.py`与`--slow-speed .06`各exit0。
slow.03/slow.06/fast.3触碰1.334/.667/.142s，积分反力峰5.189744/10.173776/58.295955N，
检测解峰6.358974/12.512237/69.989116N；冲击评分取两者最大，10N自选教学预算仅slow.03通过。
三组触碰后采样contact100%/无gap、末1s速度<1.3e−15m/s/反力1.998002N，hold均合格；
最大penetration.102131/.189185/1.137022mm、motor未饱和；fast实际触碰速度.330842m/s。
每步J/姿态/M、motor cap、contact符号与Jᵀ映射、动力学/零bias与passive/external通过；
半隐式q/v递推/clock/shape/finite通过；独立CSV控制/动力学/depth/phase/peak、
恢复初始/触碰/峰值/末尾状态直接contactForce核验通过；fast两命令CSV逐元素相同。
CLI speed0/nan各exit2；默认PNG已目视检查，本地Markdown文件链接/git diff --check通过。
产物仅ignored tmp/s17_4_v0.03与v0.06，临时日志/tmp。
触碰时yd连续但vd跳变；事件用上次ctrl的fresh forward反力，积分记录用当前ctrl重求解。
失败对照离线评分并继续观察hold，无超力急停/接触丢失注入与恢复/GUI/UR5e/硬件验证。
本人随后确认实验与预测完成，并正确回答五项Explain；根README Learning三项完成，Mastered。
学习包补充touch检测/transition顺序、稳态K偏差与检测解/积分解峰值区别。
本次保留起始全部修改，仅同步七份文档；本地文件链接/git diff --check通过。
未重跑Python示例/仿真/GUI，runtime沿用2026-10-08工程验证；S17.5等待明确请求。

## S17.3 现场工程验证（2026-10-08，DESKTOP-781D67A）

本人明确请求；起始pwd正确/git status空。读取规则、交接、示例README/source、requirements与roadmap。
当前机器WSL2 kernel6.6.87.2/Ubuntu24.04.5；conda info --base定位hook并激活mujoco，
同执行shell核验环境/所属python；Python3.12.14、MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2
metadata/真实imports/module paths/API通过，版本区别于Zero历史记录；无安装/requirements改动，未在3.11执行。
新增[impedance_sweep.py](../examples/18_compliant_control/impedance_sweep.py)与[学习包](20_3_impedance_sweep.md)。
复用S17.1固定XY fixture，6组K100/400×ζ.5/1/2，dt1ms/cap20N；另只改cap2N或dt80ms。
`python examples/18_compliant_control/impedance_sweep.py`和`--cap 5`各exit0。
六组settling1.381/.976/1.988/.723/.547/1.164s，无饱和；K提高对应peak actual4→16N。
K400/ζ1 cap2/5N settling.640/.574s，饱和joint-samples246/95；动态限幅核验通过。
粗dt80ms未settle，线性未饱和谱半径X/Y2.134/4.275，actual peak20N、requested77.509888N，
末.5s最大位置误差65.4503mm；限幅不能保证恢复，不声称已证明极限环或无限发散。
每步J/actual cap/qacc/零bias与passive/contact通过；独立clipped半隐式递推逐点核验q/v/force/time。
两命令未改七组CSV逐元素相同；cap0/nan CLI各exit2；默认PNG目视通过。
产物仅ignored tmp/s17_3_cap2与cap5，临时日志/tmp；本地Markdown链接/git diff --check通过。
无GUI/contact/UR5e/ROS/硬件验证；本人随后确认实验与预测完成并正确回答五项Explain，
README Learning Run/Modify/Explain全部完成，Mastered。学习包补充settling持续到记录末尾、
谱半径的未饱和局部适用范围及自由空间不证明接触稳定。
本次起始git status空，仅同步七份文档；本地文件链接与git diff --check通过。
未重跑Python示例/仿真/GUI，runtime沿用2026-10-08工程验证。下一小任务S17.4等待明确请求。

## S17.2 现场工程验证（2026-10-08，Zero）

保留起始全部modified/untracked S17.1工作，pwd正确；沿用本会话核验的WSL2 Ubuntu24.04.5。
同shell重新激活mujoco/核验所属python；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、
Matplotlib3.11.2 metadata/真实import/module paths通过，无安装/requirements改动，未在3.11执行。
新增[cartesian_impedance.py](../examples/18_compliant_control/cartesian_impedance.py)与
[学习包](20_2_cartesian_impedance.md)，复用cartesian_spring常量/XML及依赖链。
同XY slide，固定orientation、M=diag(2,1)kg，K100N/m，gravity/contact/passive damping零；
显式world v=Jp qvel，F=K(xd−x)−Dv→Jᵀ→gear motor，每1ms更新，cap±20N。
`python examples/18_compliant_control/cartesian_impedance.py`与`--damping-scale .5`各exit0。
默认D[28.284271,20]、ζ1，settling.976s/无越过目标；半Dζ.5，settling1.381s/越过目标约16.2%。
settling为|位置|<1mm且|速度|<2mm/s并持续到4s末；不作无限未来保证。
impedance峰值速度.110550/.164106m/s、actual joint force峰4N、无饱和；spring未settle。
连续解最大误差spring .154911mm、默认 .093510mm、半D .122545mm；动力学残差0N。
每步J/姿态/惯量/功率/gear/cap/零bias与passive/非正阻尼功率通过。
独立CSV recurrence/control/clock/shape/能量预算与settling重算、baseline不变核验通过；
错误阻尼符号静态功率检出，CLI nan/inf/0/−1各exit2。
产物仅ignored tmp/s17_2_*；无contact/GUI/UR5e/硬件/动态饱和恢复或通用稳定性验证。
默认PNG已目视核验；本地Markdown文件链接与git diff --check通过。
本人随后确认实验与预测完成，并正确回答五项Explain；README三项Learning完成，Mastered。
学习包补充e=x−xd与e=xd−x两种符号约定及任务空间有效质量的适用范围。
本次起始git status空，仅同步七份文档；本地文件链接与git diff --check通过。
未重跑Python、仿真或GUI；runtime沿用2026-10-08工程验证。下一小任务S17.3等待明确请求。

## S17.1 现场工程验证（2026-10-08，Zero）

起始git status空，pwd正确；WSL2 Ubuntu24.04.5/kernel6.18.33.2。
当前conda info --base发现hook，base→mujoco，同shell核验环境与所属python；
Python3.12.14/MuJoCo3.13.0/NumPy2.5.3/Matplotlib3.11.2 metadata、真实import/module paths/API通过。
无新依赖、安装或requirements改动；不需要Menagerie/local helper；未在Python3.11执行。
新增[cartesian_spring.py](../examples/18_compliant_control/cartesian_spring.py)、
[学习包](20_1_cartesian_spring.md)与示例README；根README为状态唯一来源。
二维slide、结构固定orientation、M=diag(2,1)kg，gravity/contact/passive damping零。
纯F=k(xd−x)→Jᵀ广义力→gear=[2,1]motor，joint cap±20N，无速度反馈/IK。
`python examples/18_compliant_control/cartesian_spring.py`及`--stiffness 200`各exit0，
默认/翻倍初始F[−4,+3]/[−8,+6]N，理论周期X/Y .888577/.628319与.628319/.444288s；
连续解最大位置差.152162/.218812mm，能量相对偏差.398242/.575907%，未settle。
势能梯度误差≤2.19e−11N；J/功率/固定姿态/惯量/gear/force cap通过；
独立cap probe请求[40,−40]→actual[20,−20]N，动态无饱和；zero始终保持initial offset。
CSV/JSON/PNG仅ignored tmp/s17_1_*；wrong-sign只做静态方向检出，未运行发散动力学。
独立CSV shape/clock/control与解析质量半隐式recurrence逐点核对通过；
CLI stiffness nan/inf/0各exit2；默认PNG已目视检查，本地Markdown文件链接与git diff --check通过。
无GUI/UR5e/contact/硬件或通用稳定性验证。
本人随后明确确认实验与预测完成，五项Explain正确覆盖恢复方向/势能、广义力单位、
等效质量与周期/耗能、gear及指令/驱动/接触力区别；README三项Learning完成，Mastered。
本次保留全部起始修改，仅同步七份状态文档；文件链接与git diff --check通过。
未重跑Python、仿真或GUI；runtime沿用2026-10-08工程验证。下一小任务S17.2，等待明确请求。

## S16.7 现场工程验证（2026-10-08，Zero）

保留起始全部未提交文档修改；WSL2 Ubuntu24.04.5/kernel6.18.33.2；当前conda hook
激活mujoco，同shell核验pwd/环境/所属python；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、
Matplotlib3.11.2、Menagerie2026.9.2 metadata/import/module paths/APIs通过，无安装/requirements变更。
新增[force_mapping_integration.py](../examples/17_force_dynamics/force_mapping_integration.py)与
[学习包](19_7_force_mapping_integration.md)，复用S16.3 build/gravity/参数，依赖链已读。
UR5e偏置点已知world force/couple，外扰applyFT→qfrc_applied，motor只ctrl；
contact/limits关闭，oracle load feedforward，未接UR5e contact或实际测力。
`python examples/17_force_dynamics/force_mapping_integration.py`与`--cap-scale .1`各exit0：
默认gravity_pd/correct/wrong sign loaded最大误差.037185621/3.434e−18/.074422293rad，
撤load后均恢复，tail误差≤8.73e−5rad/speed≤.001398rad/s，无饱和。
correct loaded shoulder-lift g−15.857087、load+4.450989、actual−20.308076Nm。
低cap correct饱和8262joint-samples，tail误差3.048211rad/speed2.994673rad/s，未恢复。
每步mapping误差≤2.67e−15Nm、power差≤1.25e−14W、动态残差≤6.40e−14Nm。
CSV/JSON/PNG仅ignored tmp/s16_7_*，默认PNG已目视核验；未用Python3.11执行。
无GUI/contact/硬件/未知外力/力闭环或通用稳定性验证；低cap PASS是失败检出。
本人随后确认实验与预测完成，README Run/Modify已勾选；Explain第1/3/4/5项准确，
本人随后补齐第2项：载荷World-fixed，P tool-fixed随姿态移动，力臂/J随q改变；
Explain完成，README三项Learning全部完成，Mastered。Stage16学习项全部完成。
本次保留起始全部工作，仅同步根/示例README、学习包、阶段笔记、roadmap、P2 plan与本文；
链接/状态/git diff --check通过，未重跑仿真或GUI，runtime沿用2026-10-08工程验证。
本地链接、CSV维度/clock/载荷包络、独立控制律重算/gear/clamp核验通过；
CLI cap nan/inf/0各exit2，git diff --check通过。

## S16.6 现场工程验证（2026-10-08，Zero）

起始git status空；WSL2 Ubuntu24.04.5/kernel6.18.33.2；当前conda hook激活mujoco，
同shell核验pwd/环境/所属python；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、
Matplotlib3.11.2、Menagerie2026.9.2 metadata/import/module paths/API通过，无安装/requirements变更。
新增[jacobian_transpose.py](../examples/17_force_dynamics/jacobian_transpose.py)与
[学习包](19_6_jacobian_transpose.md)，UR5e home/attachment_site，复用J写法而不导入local helper。
`python examples/17_force_dynamics/jacobian_transpose.py`与`--offset .2`各exit0：
纯力/力偶/组合wrench/偏置点力四case，applyFT/累加/同点换轴/直接P点J核对atol1e−12；
独立六列FK虚功误差≤1.78e−9Nm，虚拟motion两侧功率一致。
默认offset tau[−1.910008,4.650990,3.375990,.240000,.48,−.46]Nm；
偏置翻倍只让offset−site_force项翻倍。漏力臂矩/混轴错误被检出。
time0、actual qvel/外力数组0；无mj_step、motor执行/限幅、contact/GUI/硬件验证。
CSV/JSON/PNG仅ignored tmp/s16_6_*；默认PNG已目视检查；未用Python3.11执行。
本人随后确认实验与预测完成，五项Explain准确，README三项Learning完成，Mastered。
涵盖虚功/功率推导、同点同轴单位、偏置附加项、motor抵抗符号、applyFT累加与验证边界。
补充数学列向量与NumPy一维array形状区别；详见学习包。此次保留起始未提交工作，
只同步根/示例README、学习包、阶段笔记、roadmap、P2 plan与本文；
本地链接/状态/git diff --check通过，未重跑数值实验或GUI，runtime沿用2026-10-08工程验证。
本地链接、CSV维度/独立逐列力矩重算、offset附加项翻倍与三site case不变核验通过；
CLI offset nan/inf/0各exit2，git diff --check通过。

## S16.5 现场工程验证（2026-10-08，Zero）

保留起始全部未提交文档修改；WSL2 Ubuntu24.04.5/kernel6.18.33.2，当前conda hook激活
mujoco，同shell核验pwd/环境/所属python；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、
Matplotlib3.11.2 metadata/import/module paths/API通过，无安装/requirements变更。
新增[contact_wrench.py](../examples/17_force_dynamics/contact_wrench.py)与
[学习包](19_5_contact_wrench.md)，box-plane/斜墙球形指端，无local helper。
`python examples/17_force_dynamics/contact_wrench.py`与`--mass 2`最终均exit0；
box4contact/condim3，Fz9.810000000002/19.620000000005N，末速度≤1.74e−15m/s；
每步Newton最大残差≤5.99e−12N，初始无contact/撞击/尾段支撑均覆盖。
finger1contact/condim6/raw六分量非零，world F[1.882050808,.740192379,−.1]N，
COM M[−.001232051,−.001866025,−.001]Nm，末load预算误差≤4.81e−13Nm；
末线速度分量.002197249m/s、角速度分量.060501533rad/s，不误称静止。
初版静止验收失败后区分force balance与soft摩擦持续运动，只有box验收静止；
斜墙避免轴对齐误用frame转置漏检；final frame/sign±/common-point reaction/
漏COM力臂矩检查通过。CSV/JSON/PNG仅ignored tmp/s16_5_*；默认PNG已目视检查。
没有GUI、真实触觉/硬件、UR5e/force control、通用摩擦稳定性验证；未用Python3.11执行。
本人随后确认实验与预测完成，五项Explain准确说明API/轴/符号、力臂矩、
动态支撑、独立质量对照及平衡不等于静止；README三项Learning完成，Mastered。
术语补充旧geom1/geom2对应当前geom[0]/geom[1]。此次保留起始未提交工作，
只同步根/示例README、学习包、阶段笔记、roadmap、P2 plan与本文；
本地链接/状态/git diff --check通过，未重跑仿真或GUI，runtime沿用2026-10-08工程验证。
本地链接、CSV维度/clock/独立逐轴wrench重算、mass尾段比例与finger完全相同通过；
CLI mass nan/inf/0各exit2，git diff --check通过。

## S16.4 现场工程验证（2026-10-08，Zero）

保留起始全部未提交文档修改；WSL2 Ubuntu24.04.5/kernel6.18.33.2，当前conda hook
激活mujoco，同shell核验pwd/环境/所属python。Python3.12.14、NumPy2.5.3、
Matplotlib3.11.2 metadata/import/module paths/APIs通过，无安装/requirements改变。
新增[wrench_frames.py](../examples/17_force_dynamics/wrench_frames.py)与
[学习包](19_4_wrench_frames.md)，纯NumPy合成载荷，无local helper/MuJoCo runtime。
`python examples/17_force_dynamics/wrench_frames.py`与`--lever 0.4`均exit0：
默认M_O_W[.1,2.2,.3]、M_Q_W[1.1,1.7,.3]、M_Q_T[1.7,−1.1,.3]Nm；
修改P_x后分别[.1,4.2,.3]/[1.1,3.7,.3]/[3.7,−1.1,.3]Nm。
独立解析分量、换点/换轴双路径、roundtrip、norm、纯力偶、沿F换点、reaction、
同刚体功率一致性atol1e−12通过。漏换点/反位移符号分别误差1.118034/2.236068Nm被检出。
产物仅ignored tmp/s16_4_*，JSON/41×4CSV/PNG；默认图已目视核验。
没有动力学、接触测力、UR5e、GUI/硬件验证；3.11兼容目标未用3.11运行。
本人随后确认实验与预测完成，五项Explain准确覆盖契约/换点/旋转/力偶与reaction/功率，
README Run/Modify/Explain全部完成，Learning Mastered。此次保留起始未提交工作，
仅同步根/示例README、学习包、阶段笔记、roadmap、P2 plan与本文；
链接/状态/git diff --check通过，未重跑数值实验或GUI，runtime沿用2026-10-08工程验证。
本地Markdown链接、CSV独立分量公式与CLI lever nan/inf/−.2拒绝(exit2)通过；git diff --check通过。

## S16.3 现场工程验证（2026-10-08，Zero）

保留起始全部未提交修改。WSL2 Ubuntu24.04.5/kernel6.18.33.2，同shell核验pwd/
conda mujoco/所属python；Python3.12.14，MuJoCo3.13.0、NumPy2.5.3、Matplotlib3.11.2、
Menagerie2026.9.2 metadata/import/module paths通过，无安装/requirements变更。
新增[gravity_compensation.py](../examples/17_force_dynamics/gravity_compensation.py)与
[学习包](19_3_gravity_compensation.md)；私有UR5e servo改motor，name/address映射，
shoulder-lift gear2、模型joint cap150/28Nm；关闭contact/joint limits，无body_gravcomp。
`python examples/17_force_dynamics/gravity_compensation.py`、`--no-pulse`、`--cap-scale 0.1`
各exit0；独立probe g(q)、势能梯度误差9.349e−9Nm，每步mapping/clamp/force balance通过，
最大平衡残差≤1.43e−13Nm。默认5Nm/.5–.7s pulse，hold peak .023475859rad、
末.5s误差4.668e−8rad/speed1.880e−7rad/s；gravity-only最终偏移3.191614rad。
no-pulse gravity-only/hold保持home；低cap hold饱和6584joint-samples、final偏移1.799907rad，
未恢复且末窗口仍运动；PASS是饱和失败检出，不是hold成功。产物仅ignored tmp/s16_3_*。
无GUI、contact、硬件、payload或通用稳定性验证；代码3.11兼容目标未用3.11执行。
README Engineering完成；本人随后确认实验与预测完成，Run/Modify已勾选，
本人随后补正第2项，Explain完成，Learning Mastered：ctrl不限幅，
低cap shoulder-lift scalar force下限−7.5，
gear2→actual joint torque−15N·m，而非假设ctrlrange导致−2N·m。
第5项补充动态完整force budget，详见学习包。本次仅同步根README、学习包、示例README、阶段笔记、roadmap、P2 plan与本文；
本地链接/状态/git diff --check通过，未重跑仿真/GUI，runtime沿用2026-10-08工程验证。
本地链接、全部CSV shape/clock/mapping/clamp/200step pulse核验通过；CLI cap nan/0/inf各exit2。
默认PNG已目视检查，git diff --check通过。

## S16.2 现场工程验证（2026-10-08，Zero）

起始git status空；WSL2 Ubuntu24.04.5/kernel6.18.33.2。当前conda info --base发现hook，
激活mujoco并同shell核验pwd/环境/所属python；Python3.12.14，MuJoCo3.13.0、
NumPy2.5.3、Matplotlib3.11.2 metadata/真实module paths/API通过，无安装或新依赖。
新增[manipulator_dynamics.py](../examples/17_force_dynamics/manipulator_dynamics.py)与
[学习包](19_2_manipulator_dynamics.md)，world+y悬摆、无接触/约束，COM惯量单变量。
`python examples/17_force_dynamics/manipulator_dynamics.py`、`--inertia-scale 2`、
`--gravity 0`均exit0；解析M/bias/passive/a、每步力矩残差、static balance与Euler首步通过。
M .11/.13kg·m²，释放a −12.826812365/−10.853456616rad/s²，最大残差≤3.34e−16N·m；
gravity0释放保持静止。每组2001×11 CSV/JSON/PNG仅ignored tmp/s16_2_*。
无GUI、UR5e、Coriolis耦合、contact/非零constraint或驱动饱和验证；静态probe非恢复稳定证明。
本机mj_fullM真实签名(model,data,dst)，代码按现场API写；未在Python3.11执行。
Run/Modify/五问见学习包；本人随后明确确认实验与预测完成，五项Explain准确，
README Learning三项全部完成，Mastered。解释补充：d单位N·m·s/rad，
constraint=0仅验证无约束分支。此次保留全部起始未提交修改，只同步根README、
学习包、阶段/示例README、roadmap、P2 plan与本文；本地链接/状态与git diff --check通过。
未重跑仿真、GUI或运行时依赖核验；runtime沿用2026-10-08 Zero工程验证。

## P2 S16.1 现场工程验证（2026-10-07，DESKTOP-781D67A）

保留起始modified project_handoff（上一任务环境修复记录），未改P0/P1源码或历史学习状态。
本机WSL2 Ubuntu24.04.5 / kernel6.6.87.2；conda hook由当前conda info --base发现。
同执行shell核验pwd、mujoco、`/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python3.12.14、MuJoCo3.14.0、NumPy2.5.3、Matplotlib3.11.2 metadata/真实module paths/API通过；
只需这三包，无local helper、新安装或requirements改动；3.11兼容目标未用3.11运行。

新增[actuator_semantics.py](../examples/17_force_dynamics/actuator_semantics.py)：
同1hinge/惯量/passive damping，无gravity/contact，position servo gear1 vs motor feedback gear2
与motor open loop0；common joint torque cap±1.5Nm，external generalized torque±.8Nm在1–2s。
`python examples/17_force_dynamics/actuator_semantics.py`、`--kp 40`、`--external-torque -0.8`
均exit0/no import error，默认/Kp40/反向loaded offset .0400000455/.0199999990/−.0400000455rad；
servo/motor max state difference≤2.55e−15，tail error≤3.32e−11rad；
open loop final q8.280090/−7.680090rad且末窗口仍有速度，不误称4s已静止。
同状态probe未饱和−.2Nm与requested3Nm→actual1.5Nm，gear乘scalar output核验；
动态pulse无饱和，限幅证据仅probe。compiled trnid/gain/bias/gear与time0核验通过。
独立读取3组CSV：4001×8、dt/pulse1000step、force mapping/限幅/finite核验；
open loop以I=.04/passive damping=.1的独立Euler recurrence逐点对照通过。
Kp对照offset ratio2.00000237、open loop CSV完全不变；CLI nan Kp/inf load各exit2。
默认PNG已目视检查；产物只ignored tmp/s16_1_*，无GUI/UR5e/contact稳定性验证。

README新增P2状态/roadmap，learning_roadmap添加导航；新增P2 plan与Stage16/17/18 skeleton。
docs/19含完整Sprint package，future stages未填Actual或PASS；本地Markdown链接与git diff --check通过。
P1 S15.6b ROS/IPC tick预算推动physics，P2 S17.9计划独立worker loop、ROS低频reference/monitor；
后续cancel/timeout计划本地hold继续physics，与pause区分，本轮未修改worker。

## P0/P1 历史学习边界

本人已明确完成 S12.1–S12.4 Run/Modify/Explain，并明确授权 S12.5。
S12.5 PnP 的 Code + Experiment + Docs 已完成；本人随后明确确认实验与验证均完成，
五问 Explain 覆盖核心概念，Run/Modify/Explain 全部确认，Learning Mastered。
仅合成对应点与已知 K/d → T_CO；没有 detector、base transform 或 manipulation。
S12.6 工程完成后，本人于 2026-10-05 明确确认实验与预测均完成，并正确回答五项 Explain；
Run/Modify/Explain 全部确认，Learning Mastered。
S12.7a 工程完成后，本人于 2026-10-05 明确确认实验与预测均完成，并回答五项 Explain；
Run/Modify/Explain 全部确认，Learning Mastered。S12.7b 随后工程完成，本人明确确认实验与预测并回答五项 Explain，Learning Mastered。
S12.8a 本人已确认实验与预测，并补充正确的两种安装 X/Y 定义与固定关系；
Run/Modify/Explain 全部确认，Learning Mastered。S12.8b 随后工程完成，本人确认实验、预测并补充完整 Explain，Learning Mastered。
S13.1/S13.2 已 Engineering Complete + Learning Mastered。
S13.3a 已按 2026-10-06 明确请求实现并完成工程验证；本人已确认本课 Run/Modify/Explain。S13.3b已按后续明确请求实现，本人已确认Run/Modify/Explain并补正开口计算，Learning Mastered。
S13.4已实现，本人已确认Run/Modify/Explain，Learning Mastered。
S13.5已实现，本人已确认Run/Modify/Explain，Learning Mastered；Stage13学习项均完成；S14.1已实现且本人确认Run/Modify/Explain，Learning Mastered，见末节。

## 多电脑依赖规则（2026-10-05）

用户说明会在不同笔记本推进，历史已安装包不必在当前机器存在。
已更新 [AGENTS Dependency Rules](../AGENTS.md#dependency-rules) 与
[跨电脑流程](development_workflow.md#switching-laptops)：requirements 为共享声明，
任务执行前核验当前环境/版本/真实 imports，自动选择性补齐已声明的必要依赖；
先预览 resolver，保留工作中的核心版本，修复后核验 imports/pip check/本课最小实验。
窄任务不默认安装 Torch/RL 全集；可选核验缺包不迫使安装，也不能替代必需检查。
机器标识/日期必须随环境观察记录，conda 路径从当前机器发现；范围不是 exact lock。
本次仅修改规则/流程/README/交接；文档链接、规则一致性和 git diff --check 通过。
未安装包、未重跑 Python 学习实验、未核验当前 cv2 或 GUI；历史缺 cv2 记录仅属当次环境。
该规则更新时学习进度未变；S13.1 随后已明确授权实现，见下。

## Stage 14–15 换机检查（2026-10-07，DESKTOP-781D67A）

用户请求检查并补齐本机依赖；起始 pwd 正确、git status 空。
本机 WSL2 kernel 6.6.87.2 / Ubuntu 24.04.5；起始 base，按当前 conda info --base
发现并激活已有 mujoco，同执行 shell 核验环境名与
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python 3.12.14；metadata/真实 imports
与 module paths：MuJoCo 3.14.0、NumPy 2.5.3、Matplotlib 3.11.2、Menagerie 2026.9.1、
opencv-python-headless 4.14.0.94，均来自该环境；OpenCV 满足 >=4.10,<5。
MuJoCo MjModel/MjData/mj_forward/mj_step/mj_jacSite 与 cv2.solvePnP 可用；
`python -m pip check` 通过，无数值依赖需要安装，未升级核心包。
`python examples/15_motion_planning/collision_aware_pick_place.py --trials 1`
退出 0，placement_passed，1/1，final error 0.0007582125208920518 m；
产物只在 ignored tmp。未重跑所有 Stage14 示例、GUI 或学习验证。

本机 `/opt/ros` 不存在，无 ros-jazzy 包/ROS apt 源，cmake/make/g++ 缺失；
Ubuntu universe 已启用，C.UTF-8 locale 可用，dpkg --audit 空。
`sudo -n true` 返回需要密码，系统安装尚未执行；这不是 pip 缺包。
从官方 ros-infrastructure/ros-apt-source GitHub latest API 核验 release 1.3.0，
下载 noble deb 并通过 dpkg-deb -I 核对 ros2-apt-source 1.3.0~noble/all。
安装包和可执行命令脚本仅保存 ignored tmp：
`bash tmp/ros2_install_stage15.sh`（bash -n 通过）。脚本退出 conda，配置官方 apt 源，
apt update 后预览并安装示例 README 声明的 ROS Jazzy/runtime/build 依赖，
再验证系统 Python imports、构建 0.0.2 接口/type support、运行 adapters/manipulation 默认链。
apt resolver 计划必须在配置源/update 后才能生成，目前未验证；脚本后续检查均未执行。
下一步用户在自己的终端输入 sudo 密码运行脚本，再由助手检查 apt 版本、真实 imports、
接口构建与消息/集成运行结果。未改 requirements 或学习状态，不自动推进新课程。

### 安装后核验（2026-10-07，DESKTOP-781D67A）

用户确认在自己的终端输入 sudo 密码并完成 `bash tmp/ros2_install_stage15.sh`。
助手复核空 CONDA_DEFAULT_ENV/CONDA_PREFIX、`/usr/bin/python3` 3.12.3、ROS_DISTRO=jazzy。
README 声明的 13 个 ROS 模块真实 imports/module paths 均来自 `/opt/ros/jazzy`；
PlanPose/PlanManipulation/ExecuteManipulation 三接口 type support 通过，installed package.xml=0.0.2。
apt 版本记录在 ignored `tmp/ros2_stage15_apt_versions.txt`：ros2-apt-source 1.3.0~noble、
ros-base 0.11.0、rclpy 7.1.12、control-msgs 5.10.0、tf2-ros-py/tf2-py 0.36.23、
robot-state-publisher 3.3.4、ament-cmake 2.5.6、rosidl-default-generators/runtime 1.6.1；
cmake 3.28.3-1build7、make 4.3-4.1build2、g++ 4:13.2.0-7ubuntu1，dpkg --audit 空。

本轮核查用户脚本的实际运行产物，未重复运行已成功的完整链：
`tmp/s15_6a/run_qjw27L` 的 adapters 默认 SUCCEEDED/code0、360 feedback、sim7.2s、
final position error 0.116171mm；`tmp/s15_6b/run_YWvffe` 的 manipulation 默认
GoalStatus4/SUCCEEDED、placement_passed、1112 feedback、16phase、落点误差0.714212mm，
支撑/双指分离通过；live TF position error1.665e-16m、rotation matrix error8.882e-16。
两组 final_state before==after、runningFalse；无相关 Python/RSP 子进程残留。
两组 perception.log 在结束清理时出现 rclpy ExternalShutdownException traceback；
client/server/worker 无错误、任务结果成功，该退出日志问题未修复，不描述为完全无异常。
当前 Stage14–15 所需依赖已补齐；未重新验证全部失败/取消分支或 GUI，未改学习状态。
仅更新本文，git diff --check 通过。下一步可在本机复跑现有课程；不自动新增课程。

## S12.5 实现与现场验证（2026-10-04）

新增 [pnp_pose.py](../examples/13_perception_geometry/pnp_pose.py) 与
[完整教学包](15_5_pnp_pose.md)。12 个有身份的非共面 3D landmark，米制 object geometry；
SOLVEPNP_ITERATIVE 返回 object→optical camera R/t，无 truth initial guess。
复用 camera_calibration 的 projection/RMS helper，不重新执行标定；没有新增依赖。

保存编辑前既有未提交工作。核验项目目录、WSL2 kernel 6.6.87.2、Ubuntu 24.04.5；
激活 mujoco 后在执行同 shell 核验环境名与 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
现场 Python 3.12.14 / NumPy 2.5.3 / cv2 4.14.0；3.11 兼容目标未执行。

```bash
python examples/13_perception_geometry/pnp_pose.py
python examples/13_perception_geometry/pnp_pose.py --noise-px 0.8
python examples/13_perception_geometry/pnp_pose.py --noise-px 0
```

三命令均退出 0，无 import error。default / Modify 的 translation error=
0.000389763 / 0.001472155 m，rotation error=0.378563 / 1.555400 degree，
noisy reprojection RMS=0.258050 / 1.029309 pixel。零噪声 t<1e-8 m、rotation<1e-4 degree、RMS<1e-6 pixel。
默认错误 focal+5% 的 RMS=0.264791 pixel，但 translation error=0.038792973 m；
低 pixel residual 不等于准确 metric pose。单次 solve+checks 40.425/10.511/13.444 ms，非稳定性能 benchmark。

独立核对三组 NPZ/CSV shapes、saved reprojection 与 pose error、inverse 与 positive depth；
固定 geometry、noise ×4 通过。too-few/nonfinite/coplanar guards 拒绝，脚本内 collinear 拒绝；
CLI nan/negative noise 退出 2。默认 PNG 已目视检查，为对应点/残差图，不是 RGB camera photograph。
产物只在 ignored tmp/s12_5_pnp_noise*/。Markdown links 与 git diff --check 通过。
未重跑前课/P0，未运行 GUI、真实视觉、机器人执行；PASS 仅表示本课工程检查通过。

## S12.4 历史验证（2026-10-04）

[Calibration package](15_4_camera_calibration.md)：24 train / 8 known-pose held-out boards，
输入 corners/metric geometry，估计 K/d 与各 board→camera pose，k3 固定 0。
默认/0.6 px/零噪声 training RMS=0.201785/0.807089/0.000010 pixel；
known-pose clean held-out RMS=1.514152/6.207453/0.000043 pixel。
尺度 ×2 使 tvec ×2 而 K/d/RMS 基本不变；不能据小 training RMS 认定 K 准确。
原不可靠 held-out threshold 已去除，实际结果/失败分析保留该课。
本课新增 opencv-python-headless>=4.10,<5，在 mujoco 安装 4.14.0.94；不改 NumPy。
本人已确认实验、对比与 Explain，Learning Mastered；本轮不重跑标定。

## 已完成 Stage 12 前课（历史证据，2026-10-04）

- S12.1：[frame package](15_robot_perception_geometry.md)，base/world axes 核对，camera 移动后
  p_CO 变化但恢复 p_BO 不变；本人三项确认，Mastered。
- S12.2：[projection package](15_2_pinhole_projection.md)，focal scale 对比、depth gate、
  ray ambiguity；本人实验、预测与 Explain 确认，Mastered。
- S12.3：[CPU RGB/depth package](15_3_rgb_depth_acquisition.md)，EGL llvmpipe、
  frame axes、surface axial depth 与 shapes 检查；本人实验/对比/Explain 确认，Mastered。
  OSMesa probe 因缺库失败，EGL 软件替代成功；详见该课，不重新安装 OSMesa。

## P0 最终状态与审查

Stage 0–9 历史 checkbox 不改。Stage 10 工程均完成；S10.1/2 原 README Learning 空框
按本次「P0 已完成」声明同步，并注明是用户整体确认，不是助手运行追认。
S10.3–10.8b、S11.1–11.7 原先已记录本人 Run/Modify/Explain 全部 Mastered。

[本轮静态审查 / 复用清单](p1_plan.md)覆盖最终 pose_planning、pregrasp_motion、
transfer_and_descend、release_and_retreat、robustness_trials。重要边界：

- 现有 IK target 为 world-frame；真实 UR5e base 与 world 同原点但 xy 轴相反。
- S11.3 home→pre-grasp 是几何 reference 检查；S11.4 approach 是固定物体实验。
- 最终 run_to_support 直接设置 q_grasp 后闭爪；trials 没有从 home 连续执行完整动态 approach。
- sampled object xy 直接输入 planner，未验证 perception/calibration noise。
- 接触检查与 collision exclusions 有阶段适用范围；不是通用 self/obstacle/held-object planner。
- bounded DLS、cubic、model builder、actuator mapping、contact criteria、trial accounting 可复用；
  IK 原地改变 data，π 附近 orientation error 拒绝，不能隐藏这些接口限制。

既有验证（2026-10-04，本轮未重跑）：S11.7 seed=20261004/7 各 20/20；
xy±0.005 m、friction [1.8,2.2]、mass [0.045,0.055] kg；
successful final position error mean=0.002299761/0.002302795 m，
max=0.002344323/0.002346447 m。只支持这些小范围 known-pose trials。
既有 S11.6a 5 s transfer 曾因累积 relative slip 失败；更慢不保证 retention 更好。
详细 P0 实验与概念保留在[6D Pose](11_ur5e_6d_pose.md)、[6D IK](12_ur5e_6d_ik.md)、
[Trajectory](13_trajectory.md)、[Pick & Place](14_pick_place.md)及[示例 README](../examples/12_pick_place/README.md)。

## P1 路线

CPU-first / Ryzen 7 7840HS / 无 NVIDIA GPU / WSL2 Ubuntu 24.04。
Stage 12 geometry→projection→render/calibration/PnP→RGB-D→NumPy ICP→hand-eye；
Stage 13 vision estimate→grasp→完整操作→noise/trials；Stage 14 collision/C-space→RRT→
RRT-Connect→smoothing→timing→UR5e/held-object；Stage 15 才 ROS2 node/topic/service/action/
TF2/URDF/pipeline。每个 Task 0.5～2h，多步骤系统拆成小任务。
无 Isaac、YOLO、SAM、大视觉模型训练、MoveIt。新依赖仅在需要的 Task 说明并安装到 mujoco。
本轮新增[规划](p1_plan.md)、[Stage 12](15_robot_perception_geometry.md)完整首课、
[Stage 13](16_vision_based_manipulation.md)、[Stage 14](17_motion_planning.md)、
[Stage 15](18_ros2_integration.md)骨架；OpenCV 在 S12.4 新增，ROS2 未安装。

## S12.1 历史验证与已知限制（2026-10-04）

编辑前 pwd 正确、git status --short 为空；无既有未提交修改。
现场核验：WSL2 kernel 6.6.87.2-microsoft-standard-WSL2，Ubuntu 24.04.5。
起始 base；自动激活既有 mujoco，并在执行同一 shell 重新核验：

```text
pwd: /home/lucas/projects/mujoco-robotics-playground
CONDA_DEFAULT_ENV: mujoco
python: /home/lucas/miniconda3/envs/mujoco/bin/python
Python 3.12.14 / NumPy 2.5.3 / MuJoCo 3.14.0
```

代码兼容目标 3.11，本轮未在 3.11 执行；未修改环境版本或 requirements。
先加载官方 UR5e，mj_forward 后 base world rotation 实测 diag(-1,-1,1)，position=0。
以下 S12.1 命令均退出 0，无 import error：

```bash
python examples/13_perception_geometry/camera_frames.py
python examples/13_perception_geometry/camera_frames.py --camera-x -0.25
```

camera x 从 −0.35→−0.25 m，p_CO 从 [-0.10,-0.10,0.77]→[-0.20,-0.10,0.77] m；
p_BO 恒为 [0.45,-0.20,0.03] m，恢复 p_WO 恒为 [-0.45,0.20,0.03] m。
完整 rotation/translation chain、非原点 object test point、w=0 direction、rigid inverse 通过；
reflection 拒绝、wrong order 和 mm/m 混用失败演示通过。time=0。
这是 synthetic optical observation，无 GUI、renderer、标定、PnP、动力学或抓取验证。
新增/修改 Markdown 相对文件链接与 git diff --check 已检查；详见首课笔记。

## 本人 handoff

本人已完成 [S12.5 Learning Package](15_5_pnp_pose.md) 实验、验证与 Explain；
README 三项已按本人明确报告更新。精确补充：尺度来自已知 metric 3D geometry，
PnP 求该尺度下的 R/t；相机标定估 K/d，而 PnP 固定它们。
本次仅更新 README、学习包、示例说明、roadmap、P1 plan 与 handoff；
检查相对文档链接、状态一致性及 git diff --check，未重跑实验/仿真/GUI。
上述 runtime 证据沿用 2026-10-04 工程验证，不是本次新运行。
S12.6 已完成本人 Run/Modify/Explain；S12.7a 已由本人确认三项学习验证，S12.7b 已完成本人 Run/Modify/Explain；S12.8a 已按后续明确请求实现，见下。

## 环境与历史 GUI 经验

执行前必须重新核验，不以以上版本当作未来保证。当前不是 mujoco 时可自动用
`source /home/lucas/miniconda3/etc/profile.d/conda.sh`、`conda activate mujoco`，随后同 shell
检查环境名与 interpreter。禁止 base/system Python 运行、安装或 sudo pip。

用户于 2026-09-26 已确认 GUI 显示恢复；不继续把 WARN:COPY MODE 视作当前阻塞。
历史 viewer native 退出崩溃与 WSLg 共享内存问题需与 headless 验证分开；本轮未复测 GUI。
passive viewer 的 Python 循环没有暂停回调，空格不会暂停它。
历史具体错误与诊断见[MuJoCo notes](mujoco_notes.md)、[基础课](01_mujoco_basics.md)、
[UR5e 示例](../examples/02_ur5e_basics/README.md)。

## S12.6 现场工程验证（2026-10-05）

[学习包](15_6_rgbd_back_projection.md)、[代码](../examples/13_perception_geometry/rgbd_back_projection.py)。
起始 pwd 正确、git status --short 为空；WSL2 6.6.87.2 / Ubuntu 24.04.5。
base 自动切到 mujoco，同执行 shell 核验环境及 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14 / MuJoCo 3.14.0 / NumPy 2.5.3；3.11 兼容目标未执行。无新增依赖。

```bash
python examples/13_perception_geometry/rgbd_back_projection.py
python examples/13_perception_geometry/rgbd_back_projection.py --camera-height 1.0
```

两命令退出 0、无 import error，CPU EGL llvmpipe。各 76800 有效点；max pixel error
5.68e-14/2.84e-14 px，floor z≈0、top z≈0.06 m；实际 base 为 world xy 反向。
错误 normalized ray × Z 使离轴 floor patch mean z=0.126922/0.158652 m。
内置独立手算、invalid/empty depth、surface truth、base/world 链与 RGB mask/order 检查通过。
默认 PNG 目视检查；显示网格稀疏采样，NPZ 保存完整点云，产物仅 ignored tmp/s12_6_rgbd_z*/。
已知外参、ideal aligned RGB-D；无 GUI、真实相机、pose/ICP、动态操作验证。
Engineering 完成；本人已确认实验、预测与五项 Explain，Learning Mastered。
解释记录见学习包：axial Z/range、数组索引、frame 链、固定场景与 roundtrip 边界。
本次仅同步 README、学习包、示例说明、roadmap、P1 plan 与 handoff；
检查相对文档链接、状态一致性及 git diff --check，未重跑 Python、仿真或 GUI。
上述 runtime 数字沿用 2026-10-05 S12.6 工程验证；本轮未重跑 S12.6。

## S12.7a 现场工程验证（2026-10-05）

[学习包](15_7a_rigid_alignment.md)、[代码](../examples/13_perception_geometry/rigid_alignment.py)。
保留起始六份未提交文档更新；pwd 正确，WSL2 6.6.87.2 / Ubuntu 24.04.5。
从 base 自动激活 mujoco，同执行 shell 核验环境与
`/home/lucas/miniconda3/envs/mujoco/bin/python`；Python 3.12.14 / NumPy 2.5.3。
3.11 为兼容目标未执行；无新增依赖，不导入 MuJoCo 或启动渲染。

```bash
python examples/13_perception_geometry/rigid_alignment.py
python examples/13_perception_geometry/rigid_alignment.py --noise-m 0.004
python examples/13_perception_geometry/rigid_alignment.py --noise-m 0
```

三条命令退出 0、无 import error。default/Modify t error=0.000699880/0.002794669 m，
R error=0.552240/2.260010 degree，noisy RMS=0.001501730/0.006001646 m；
zero-noise RMS=1.23e-16 m。镜像 raw det=-1、RMS≈0；corrected det=+1、RMS=0.045317910 m，
SSE penalty=4*s3 通过；wrong correspondence RMS=0.049574911 m。
内置 truth/90° planar triangle、inverse、SO(3)、五类 input guards 通过。
独立 NPZ/CSV/summary 核对、reverse fit、common unit scaling、source origin shift 通过；
CLI noise nan/negative/over-limit 退出 2。默认 PNG 已目视检查。产物仅 ignored tmp/s12_7a_rigid_noise*/。
文档相对链接/状态与 git diff --check 检查通过。未重跑 P0/前课，无 GUI、真实点云、ICP loop 或机器人执行。
Engineering Complete；本人明确确认实验、预测与五项 Explain，Learning Mastered。
解释记录见学习包：中心化、Vt、reflection、非共线几何、residual 与 pose error、ICP 对应更新。
补充最小奇异方向修正损失与一般 source→target frame 的含义。
本次仅更新六份文档，检查相对链接、状态一致性及 git diff --check，未重跑实验/仿真/GUI。
上述 runtime 证据沿用 2026-10-05 工程验证；
S12.7b 本次已获明确授权并实现，见下。

## S12.7b 现场工程验证（2026-10-05）

[学习包](15_7b_icp_loop.md)、[代码](../examples/13_perception_geometry/icp_loop.py)。
起始 pwd 正确、git status --short 空；当前 WSL2 kernel **6.18.33.2** / Ubuntu 24.04.5，
不是历史 6.6 kernel。从 base 激活 mujoco，同执行 shell 核验环境名与
`/home/lucas/miniconda3/envs/mujoco/bin/python`；Python 3.12.14 / NumPy 2.5.3。
3.11 为兼容目标未执行，无新增依赖，不导入 MuJoCo 或渲染。

```bash
python examples/13_perception_geometry/icp_loop.py
python examples/13_perception_geometry/icp_loop.py --gate-m 0.008
python examples/13_perception_geometry/icp_loop.py --gate-m 0.000001
```

三命令退出 0，无 import error。20 mm：near full t error=0.045091 mm、R=0.047650°；
far full 小更新停止但 R error=141.863330°；partial t error=1.664512 mm、R=1.475980°。
8 mm：partial t error=0.093034 mm、R=0.115058°，保留 60/120；far full 仅 18/120，
RMS=4.900273 mm 仍 R error=139.446286°。1 μm 全部零对应/refusal，RMS=null。
内置 exact recovery/非原点组合、NN tie、SO(3)、固定 pair RMS 不增、上限、退化、input checks。
独立三组 NPZ/CSV/summary 的 brute-force distances/counts/RMS/t error 核对通过；
反转 rows、target frame origin shift、input preservation、非法 gate/reflection 初值检查通过。
默认 PNG 已目视检查。产物仅 ignored tmp/s12_7b_icp_gate*/。
六份相关文档的本地 Markdown 文件链接、README 工程/学习状态与 git diff --check 通过。
未重跑前课/P0，无 GUI、真实 RGB-D 注册、多 seed benchmark、机器人执行。
本人随后明确确认实验与预测均完成，并回答五项 Explain，Run/Modify/Explain 全部确认，Learning Mastered。
解释与 frame/RMS 精度补充见学习包。此次仅同步六份文档，保留全部既有未提交代码/文档；
本地 Markdown 文件链接、状态一致性与 git diff --check 通过，未重跑 Python 实验/仿真/GUI。
上面 runtime 证据沿用 2026-10-05 工程验证。S12.8a 已按后续明确请求实现，见下。

## S12.8a 现场工程验证（2026-10-05）

[学习包](15_8a_hand_eye_geometry.md)、[代码](../examples/13_perception_geometry/hand_eye_geometry.py)。
保留起始五份已修改文档与两份 untracked S12.7b 文件。pwd 正确；
WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5。从 base 自动激活 mujoco，
同执行 shell 核验环境名与 `/home/lucas/miniconda3/envs/mujoco/bin/python`；
Python 3.12.14 / NumPy 2.5.3，3.11 兼容目标未执行。无新增依赖。

```bash
python examples/13_perception_geometry/hand_eye_geometry.py
python examples/13_perception_geometry/hand_eye_geometry.py --axis-spread-deg 1
python examples/13_perception_geometry/hand_eye_geometry.py --axis-spread-deg 0
```

三命令退出 0，无 import error。每种安装各三组/5 poses/10 pairs，truth residual <1e-12；
multi_axis 35/1° K rank=8、L rank=3，但 rotation s8/L s3 从 1.002729042→0.034080102；
0° 为 rank 6/2。同轴 rank 6/2、纯平移 0/0，构造 X'=ZX 均保持零 residual。
纯平移 K=0 不代表完整方程无旋转信息；t_A=R_X t_B 仍约束旋转，t_X 始终不可观测。
内置绝对链、relative AX=XB、K vec_F 与平移分块式、反例、wrong-B、input guards 通过；
独立三套 NPZ/JSON/CSV（每套 60 pair rows）、generic inverse、反转成对采样、input preservation、
X' 对应另一恒定绝对 Y 检查通过；CLI nan/−1/61° 退出 2。默认 PNG 已目视检查。
产物仅 ignored tmp/s12_8a_geometry_spread*/。未运行前课/P0、GUI、PnP、机器人执行或 X solver。
安装定义/闭环核对官方 OpenCV 文档；K/L 与反例为本课直接推导，不用 hand-eye API。
六份相关文档的本地 Markdown 文件链接、工程/未勾选学习状态与 git diff --check 通过。
本人随后确认实验与预测均完成，并补充正确的两种安装 X/Y 方向与固定关系；
Run/Modify/Explain 全部确认，Learning Mastered。相对运动反向推导、退化与条件性记录见学习包。
S12.8b 已按后续明确请求实现，见下。
本次仅同步六份文档；本地链接、状态一致性与 git diff --check 通过，未重跑实验/仿真/GUI。
runtime 证据沿用 2026-10-05 工程验证；学习状态依据本人报告与回答，不由助手运行追认。
STOP，不自动实现 S12.8b calibration/noise/held-out。

## S12.8b 现场工程验证（2026-10-05）

[学习包](15_8b_hand_eye_calibration.md)、[代码](../examples/13_perception_geometry/hand_eye_calibration.py)。
保留起始五份 modified 文档与四份 untracked S12.7b/8a 文件；pwd 正确。
WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5；从 base 激活 mujoco，同执行 shell 核验
`CONDA_DEFAULT_ENV=mujoco` 与 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14 / NumPy 2.5.3；3.11 兼容目标未执行，无新增依赖。

```bash
python examples/13_perception_geometry/hand_eye_calibration.py
python examples/13_perception_geometry/hand_eye_calibration.py --noise-scale 4
python examples/13_perception_geometry/hand_eye_calibration.py --noise-scale 0
```

三命令退出 0，无 import error。两种安装×broad/near-axis，12 train / 6 held absolute poses。
默认 eye-in-hand broad/near X t error=0.864328/3.873388 mm，R=0.091122/1.191635°；
train pair rotation RMS=0.215155/0.208456°，held t RMS=0.878049/5.348452 mm。
默认 eye-to-hand broad/near X t=1.094404/4.114071 mm，R=0.108762/0.239907°。
scale=4 本次误差约 ×4；zero-noise t/held <1e-12 m，angle residual 有约 2.4e-6° 浮点下限。
L condition broad/near=1.342061/22.177587。只单 seed，不是稳健性 benchmark。
内置 clean recovery、SO(3)、同轴/纯平移拒绝；独立三套 NPZ/JSON/96 CSV rows、
held loop/general inverse、units×2、input preservation、clean reverse、另一 rotation-log-vector
Procrustes + translation 求解检查通过；非法 pose、CLI nan/−1/5 拒绝（CLI 退出 2）。
默认 PNG 已目视检查；产物仅 ignored tmp/s12_8b_calibration_noise*/。
最初 zero-noise 独立检查零绝对容差在 1e-16 m 失败，改 atol=1e-12 后通过；非算法更改。
**当前 cv2 无法导入（ModuleNotFoundError）**，与 2026-10-04 历史记录不同。
未安装依赖/未完成 OpenCV cross-check，改用独立 NumPy 验证；本课脚本无需 cv2。
不得据历史证据称当前 PnP/camera calibration 环境可用。未重跑前课/P0、GUI、真实图像或机器人。
六份相关文档的本地 Markdown 文件链接、工程/未勾选学习与 Stage 13 未开始状态、git diff --check 通过。
本人随后确认实验与预测完成，并正确补充齐次 ± 符号、平移分块式与
clean-held 仍含训练 Y_mean 偏差的说明；Run/Modify/Explain 全部确认，Learning Mastered。
学习验证记录见学习包；S13.1 随后已明确授权实现，见下。
本次同步六份文档，本地链接、状态一致性与 git diff --check 通过；未重跑实验/仿真/GUI。
runtime 沿用 2026-10-05 工程验证；学习状态按本人反馈记录。
STOP，不自动实现 S13.1。

## S13.1 现场工程验证（2026-10-05，机器 Zero）

[学习包](16_vision_based_manipulation.md)、[代码](../examples/14_vision_manipulation/perception_pose.py)。
起始 pwd 正确、git status --short 空。当前 Zero：WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5；
通过本机 conda info --base 找 shell hook，从 base 激活 mujoco，同执行 shell 核验
环境名与 `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python 3.12.14；required distribution versions + 实际 imports/module paths 核验：
NumPy 2.5.3、MuJoCo **3.13.0**、Menagerie 2026.9.2、Matplotlib 3.11.2。
这不是历史 MuJoCo 3.14 环境，3.11 兼容目标未执行；无新依赖/安装，不核验无关 cv2。

```bash
python examples/14_vision_manipulation/perception_pose.py
python examples/14_vision_manipulation/perception_pose.py --rms-limit-px 0.3
```

两命令退出 0，无 import error。actual R_WB=diag(−1,−1,1)、p=0；16 synthetic packet cases。
默认 accepted/rejected=2/14，nominal p_W=[−.448,.201,.027] m，t error=0.003741657 m/R=0.8°；
biased low-RMS 仍通过，t error=0.032155870 m。0.3 px gate 全部拒绝，不提供 usable pose。
base-as-world 错误示例 error=0.983470386 m。模型 cache only、time=0、无动力学。
内置 clean/non-origin chain、policy/SE(3)/frame/unit/time/refusal、norm preservation 通过；
独立 NPZ/JSON/CSV 16 rows、quoted reason、refusal/null/no output、direct T_WC T_CO、
non-origin/rotated alternative base、input/outputs memory isolation、quality/extrinsic guards 通过；
CLI nan/−1/3 退出 2。默认 PNG 已目视检查；ignored tmp/s13_1_pose_rms*/ 存产物。
Synthetic producer/scorer 用 truth，consumer 无 truth 参数；精确已知静态 T_BC，不重做 hand-eye。
无实际 image estimator/PnP、IK/可达/碰撞/抓取/GUI，未重跑前课/P0。
六份相关文档本地 Markdown 文件链接、S13.1 工程/未勾选学习与 S13.2 未开始状态、git diff --check 通过。
本人随后确认实验与预测完成，并回答五项 Explain，Run/Modify/Explain 全部确认，Learning Mastered。
解释记录与矩阵链、完整 pose、米制平移检查的精度补充见学习包；S13.2 已按后续明确请求实现，见下。
本次仅同步六份文档，保留既有未提交代码/文档；本地链接、状态一致性与 git diff --check 通过。
未重跑实验/仿真/GUI，runtime 证据沿用 2026-10-05 Zero 工程验证。
STOP，不自动实现 S13.2。

## S13.2 现场工程验证（2026-10-05，机器 Zero）

[学习包](16_2_grasp_pose_generation.md)、[代码](../examples/14_vision_manipulation/grasp_candidates.py)。
起始 pwd 正确、git status --short 空；Zero WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5。
本机 conda info --base 定位 hook、base→mujoco，同执行 shell 核验环境/interpreter：
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python 3.12.14；required distribution
versions/imports/module paths NumPy 2.5.3、MuJoCo 3.13.0、Menagerie 2026.9.2、Matplotlib 3.11.2。
P0 helper import 通过；无新增依赖，3.11 兼容目标未执行，不核验无关 cv2。

```bash
python examples/14_vision_manipulation/grasp_candidates.py
python examples/14_vision_manipulation/grasp_candidates.py --size-x-m 0.12
python examples/14_vision_manipulation/grasp_candidates.py --size-x-m 0.12 --size-y-m 0.12
```

三命令退出 0，无 import error。Yaw=20°、center=[−.45,.20,.03] m，尺寸40/30/60 mm。
四朝向开口48/38/48/38 mm，width/两端点 local IK pass=4/4；dx120→2/2，dx/dy120→0/0。
Far [2,2,.03] m（独立合法40mm width）记录 pre IK update budget，q null。
复用model/DLS，candidate独立data/home；不读取known_object pose，不检查 contacts/path。
Actual compiled root/site、pad offset35mm、slide range检查；proper top-down、pad=center、
pre局部方向、limits/residual/time=0 内置通过。独立三套JSON/NPZ/4 CSV rows、
保存q FK、refusal无伪造q、directchain、yaw/translation equivariance、generator purity、
invalid dims/tilt/min-gap 检查通过，CLI size-x nan/−1/0.5退出2。
Passing endpoints最大position residual=6.212543e-5 m，rotation matrix Frobenius max=2.943951e-6。
默认 PNG 已目视检查，图是输入box/opening axes投影（显示偏移不改变targets）；ignored tmp/s13_2_grasp*/。
无真实vision/PnP、continuous path/collision/grasp/dynamics/GUI，未重跑前课/P0独立脚本。
七份相关文档本地文件链接、S13.2工程/未勾选学习与S13.3a未开始状态、git diff --check通过。
本人随后确认实验与预测完成，并补充正确的开口定量关系：width30mm 对应总开口38mm、每根slide9mm；
dx120mm 时90°/270°夹持dy30mm，仍通过开口筛选。Run/Modify/Explain 全部确认，Learning Mastered。
下一小任务为 S13.3a，等待本人明确请求；反馈与精度说明见学习包。
本次仅同步六份文档，保留既有未提交工作；链接、状态一致性与 git diff --check 通过。
未重跑实验/仿真/GUI，runtime 沿用2026-10-05 Zero工程验证。
STOP，不自动实现S13.3a。

## S13.3a 现场工程验证（2026-10-06，机器 Zero）

[学习包](16_3a_vision_to_motion.md)、[代码](../examples/14_vision_manipulation/vision_to_motion.py)。
起始 pwd 正确、git status --short 空；WSL2 kernel 6.18.33.2 / Ubuntu 24.04.5。
本机 conda info --base 定位 hook，base→mujoco，同执行 shell 核验环境与
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python 3.12.14；required versions / imports /
module paths：NumPy 2.5.3、MuJoCo 3.13.0、Menagerie 2026.9.2、Matplotlib 3.11.2。
本机缺已声明的 OpenCV headless；dry-run 仅安装该包，不改核心包，随后执行
`python -m pip install 'opencv-python-headless>=4.10,<5'`，安装4.14.0.94；cv2 path/API 与
`python -m pip check` 通过。3.11 兼容目标未执行；requirements 未更改。

```bash
python examples/14_vision_manipulation/vision_to_motion.py
python examples/14_vision_manipulation/vision_to_motion.py --noise-px 0.8
python examples/14_vision_manipulation/vision_to_motion.py --noise-px 0
python examples/14_vision_manipulation/vision_to_motion.py --rms-limit-px 0.1
```

四命令退出0，无 import error。前三组motion_passed：5250 steps / 10.5 s、无 observed contacts；
default/0.8/0 px PnP t error=3.977845/5.257767/2.457555 mm；
final tracking=0.072987/0.073148/0.072836 mm，true-target=4.047543/5.329357/2.525082 mm。
严格 RMS gate 在 perception 拒绝，time=0，无新 motion result。
最初无 bias compensation 的 final error=8.600956 mm/0.021219 rad，未通过门限；
增加显式 qfrc_bias arm feedforward 后通过，未放宽门限；理想外力接口不代表硬件扭矩能力。
整图 color-ID detector 无 truth/ROI 输入；PnP 完整pose 经显式 tilt gate+upright prior，保留估计xyz/yaw。
noise=0 仍含 raster quantization；marker rig 是抽象合成点图，不是 opaque-box RGB camera image。
独立PNG centroid/CSV/NPZ/shape/time/frame chain/pre-final residual/reference concatenation核对通过；
missing ID/tilt/far IK 拒绝通过。默认两PNG目视检查，产物只在ignored tmp/s13_3a_motion*/。
未重跑前课/P0；无GUI、真实视觉、闭爪/lift/place验证，sampled collision不是通用规划保证。
CLI noise nan/−1 与 RMS 3 拒绝（exit 2）；本地 Markdown 文件链接、状态一致性、
新文件 whitespace 与 git diff --check 通过。
本人随后明确确认实验与预测完成，并回答五项Explain；Run/Modify/Explain全部确认，Learning Mastered。
解释与精度补充见学习包：本课非共面rig，零扰动已知主因是像素取整，不归因于planar geometry sensitivity。
本次仅同步七份文档，保留全部既有未提交代码/文档；本地文件链接、状态一致性与git diff --check通过。
未重跑Python学习实验/仿真/GUI；runtime证据沿用2026-10-06 Zero工程验证。
下一小任务S13.3b等待本人明确请求。
STOP，不自动实现S13.3b。

## S13.3b 现场工程验证（2026-10-06，机器 Zero）

[学习包](16_3b_vision_pick_place.md)、[代码](../examples/14_vision_manipulation/vision_pick_place.py)。
保留起始六份modified文档与S13.3a两份untracked文件；pwd正确。WSL2 kernel6.18.33.2/Ubuntu24.04.5。
本机conda info --base定位hook，激活mujoco，同执行shell核验环境/interpreter；
Python3.12.14、NumPy2.5.3、MuJoCo3.13.0、Menagerie2026.9.2、Matplotlib3.11.2、
OpenCV distribution4.14.0.94；required versions/import paths/API检查通过。无安装/新依赖，3.11未执行。

```bash
python examples/14_vision_manipulation/vision_pick_place.py
python examples/14_vision_manipulation/vision_pick_place.py --close-target 0.014
python examples/14_vision_manipulation/vision_pick_place.py --close-target 0.002
```

默认/.002 exit0、placement_passed，11114/11116步、time22.228/22.232s；lift52.728417/52.757120mm，
final position error.714212/.738792mm、rotation.008264/.013018°（相对estimated-yaw command）。
.014在close预期失败(exit1)，5750步/time11.5s，不执行lift；保存partial trace，不伪造final结果。
默认phase relative变化lift/transfer/descent=4.958479/10.667653/4.656103mm；累计23.846433mm。
逐阶段15mm规则与P0一致，不把它当累计保证。初版累计gate失败、固定朝向与更强夹紧未消除位移；
初版下降到底还触发finger-ground接触。最终持续support事件停止descent，未放行finger-ground。
no object-truth target correction / held-offset feedback / execution qpos-reset / weld。
独立CSV/NPZ/PNG centroid、2ms clock/phase sequence、phase与累计metrics、support/release窗口、
nominal目标重算、最终ground force=.4905N、私有IK隔离、CLI nan/−1/.05 exit2检查通过。
默认pipeline PNG目视检查，所有产物仅ignored tmp/s13_3b_pick_place*/；没有重跑前课/P0独立脚本。
文档链接/状态一致性/git diff --check通过；无GUI、真实camera、hardware或robustness benchmark。
Engineering Complete；本人随后确认实验与预测完成，Run/Modify已勾选。
本人随后正确补正20mm基准间隙、5/14mm slide对应30/48mm opening、接触/lift证据与close失败停止判据；Explain完成，Learning Mastered。
第2项乘法正确，补充T_OG为G→O映射；phase timer可重置，但data.time不能重置。
累计relative change为相对close_end最大偏离，并非路径长度；详见学习包。
本次同步七份文档；本地链接/状态一致性/git diff --check通过，未重跑实验/仿真/GUI。
runtime证据沿用2026-10-06 Zero工程验证；下一小任务为S13.4，等待本人明确请求。
STOP，不自动实现S13.4。

## S13.4 现场工程验证（2026-10-06，机器 Zero）

[学习包](16_4_perception_noise.md)、[代码](../examples/14_vision_manipulation/perception_noise.py)。
保留起始七份modified文档；pwd正确。WSL2 kernel6.18.33.2/Ubuntu24.04.5；本机conda info --base定位hook，
激活mujoco，同执行shell核验环境与`/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python3.12.14；required distributions/import paths/API核验：NumPy2.5.3、MuJoCo3.13.0、Menagerie2026.9.2、
Matplotlib3.11.2、OpenCV distribution4.14.0.94。无安装/新依赖；3.11兼容目标未执行。

```bash
python examples/14_vision_manipulation/perception_noise.py
python examples/14_vision_manipulation/perception_noise.py --scale 0.5
```

两命令exit0，无import error；固定图像/scene/control的7case分别评价，不是统计trials。
默认±4mm world x映射正确；base-origin yaw1°新增shift8.583909mm、raw error9.900mm、final5.000mm。
40mm camera y pose shift在close失败(time11.5s/5750steps，无final结果)。
6° camera-origin外参tilt新增shift80.860065mm/tilt6.422181°，prior拒绝(time0/empty trace)。
scale.5：20mm pose y case通过(final6.227mm)，tilt3.429361°通过prior但在close失败。
baseline final.714212mm，与S13.3b一致；所有原reported fit RMS=.353150px，post-fit pose diagnostic单独重算。
默认pose translation/yaw/large-y diagnostic=2.951576/2.458744/30.352346px；外参fault camera RMS不变。
独立14case CSV/NPZ、PNG/pixels、clock/frame chain/projection/final scoring、sin/cos yaw与lever-arm几何、
zero-amplitude/输入输出隔离/非法scale CLI exit2检查通过。默认comparison PNG目视检查；
图轴限已修复以完整显示reject rows，并用verified JSON重新绘图，未为绘图重跑动力学。
全部产物仅ignored tmp/s13_4_noise_scale*/；无GUI、真实视觉、重标定、随机robustness或硬件验证。
本地Markdown链接/状态一致性/git diff --check通过；现有执行器与P0代码未改。
本人随后明确确认实验与预测完成，并回答五项Explain；Run/Modify/Explain全部确认，Learning Mastered。
frame/pivot、lever-arm、quality与非线性说明见学习包；r是到旋转轴的垂直距离，
外参case改变的是使用错误外参的估计，并非真实camera/object运动。
本次仅同步七份文档；本地链接/状态一致性/git diff --check通过，未重跑实验/仿真/GUI。
runtime证据沿用2026-10-06 Zero工程验证；下一小任务S13.5等待本人明确请求。
STOP，不自动实现S13.5。

## S13.5 现场工程验证（2026-10-06，机器 Zero）

[学习包](16_5_repeated_trials.md)、[代码](../examples/14_vision_manipulation/repeated_trials.py)。
保留起始七份modified文档；pwd正确，WSL2 kernel6.18.33.2/Ubuntu24.04.5。
本机conda info --base定位hook，激活mujoco，同执行shell核验环境与
`/home/lucas/miniconda3/envs/mujoco/bin/python`；Python3.12.14。
required版本/import paths：NumPy2.5.3、MuJoCo3.13.0、Menagerie2026.9.2、Matplotlib3.11.2、
OpenCV distribution4.14.0.94；实际API通过脚本执行核验。无安装/新依赖，3.11兼容目标未执行。

```bash
python examples/14_vision_manipulation/repeated_trials.py
python examples/14_vision_manipulation/repeated_trials.py --seed 7
python examples/14_vision_manipulation/repeated_trials.py --noise-px 0.8
```

三组各8trial全部评价完毕，exit0，无import error；默认6/8，seed7/noise.8各2/8。
成功条件final error mean/max：.735565/.819936、.695762/.704416、.694290/.699436mm。
默认failure grasp_hold×2；seed7 detector×5+grasp_hold×1；noise.8 mapping×2+planning×2+detector×2。
默认Wilson95%=[.409275,.928521]；这不是普遍75%成功率或硬件鲁棒性结论。
默认wall all/success/failure means=4.360/5.028/2.357s，仅本机当次含compile/image/perception/planning/execution观察。
默认同seed重跑，物理status/phases/stage metrics/sim time逐项相同，wall time不同。
make_image加可选seed，旧默认图像逐字节核对通过；manifest/prefix与paired noise场景一致。
独立24trial图像重建/紧凑NPZ索引时间/坐标链/pose与final error/统计分母/failure counts通过；
Wilson与score二次方程roots一致，零成功/零失败组null、非法CLI exit2通过。
失败图像area10/11px证实marker覆盖；默认trial4/7估计z27.935/27.749mm，实际FK证实finger bottom负z，
支持ground-contact日志，不自动抬高target/放行接触/挑除失败。Reference collision仍使用simulator oracle几何。
默认dashboard PNG目视检查；三组紧凑产物共约1.6MB，仅ignored tmp/s13_5_trials*/。
本地链接/状态一致性/git diff --check通过；无GUI、真实视觉/硬件、物理随机化或稳定performance benchmark。
本人随后确认实验与预测完成，并回答五项Explain；Run/Modify/Explain全部确认，Learning Mastered。
end-to-end分母、成功条件偏差、paired随机输入与wall time、小N范围与证据分类说明见学习包。
本次仅同步七份文档；本地链接/状态一致性/git diff --check通过，未重跑实验/仿真/GUI。
runtime证据沿用2026-10-06 Zero工程验证；Stage13学习项均完成，下一小任务S14.1等待明确请求。
STOP，不自动实现Stage14。

## S14.1 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_motion_planning.md)、[示例](../examples/15_motion_planning/README.md)。
起始pwd正确、git status --short空；WSL2 kernel6.6.87.2 / Ubuntu24.04.5。
本机conda info --base定位hook，base→mujoco，同执行shell核验环境名与
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0；
required distribution versions、真实import/module paths与API通过。无新增/安装依赖，3.11未执行。

```bash
python examples/15_motion_planning/collision_checking.py
python examples/15_motion_planning/collision_checking.py --allowed-depth-m 0.001
```

两命令exit0，无import error；各13配置，默认6/13接受，1mm门限1/13接受。
Cartesian+ yaw教学模型，球形collision geometry；不使用UR5e或前课运行产物。
独立MjData、具体pair/phase/depth政策，固定T_GO更新held freejoint并再次forward。
self/robot-environment/object-environment拒绝；grasp/place许可；深穿透拒绝；
positive-distance contact记录不视为触碰。held-obstacle物体穿透20mm、pads仍分离。
初版palm尺寸使place触地，缩小教学palm后place浅接触通过；未放行robot-ground。
两组26行JSON、球–球/球–平面解析距离、旋转持物链、非法shape/nan/phase/held guards、
held→nonheld重复查询隔离通过；live qpos/qvel/time/xpos不变，query time=0。
CLI nan退出2。产物仅ignored tmp/s14_1_collision_depth*/。
本地Markdown链接、状态一致性与git diff --check通过。
无GUI、动力学、抓持稳定、UR5e、edge/连续路径或硬件验证；模型过滤限制详见学习包。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；解释记录见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；保留既有未提交代码/文档。
本地文档链接、状态一致性与git diff --check通过；未重跑Python/仿真/GUI，
runtime证据沿用2026-10-06本机工程验证。下一小任务S14.2等待明确请求。

## S14.2 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_2_configuration_space.md)、[代码](../examples/15_motion_planning/edge_checking.py)。
保留起始四份modified文档与untracked S14示例目录；pwd正确。
WSL2 kernel6.6.87.2 / Ubuntu24.04.5；本机conda info --base定位hook，激活mujoco，
同执行shell核验环境名/interpreter `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2；required metadata/真实imports/module paths检查通过，
实际API由脚本核验。无新增/安装依赖，3.11未执行。

```bash
python examples/15_motion_planning/edge_checking.py
python examples/15_motion_planning/edge_checking.py --step-m 0.02
```

两命令exit0，无import error；各7条边。二维XY slide sphere fixture，C-obstacle半径45mm。
直连1.6m，端点均合法；默认h=.4m，5点sampled_valid=True但exact_valid=False（漏检）；
h=.02m，81点拒绝，first_invalid index45 / x=.1m，解析最小clearance=-45mm。
手动绕行3段长度合计2.1m，两组均通过；不是自动搜索结果。
check_edge仅用MuJoCo配置查询，独立解析disk scorer不参与采样判定。
零长度clear/blocked、joint limits、输入/step/budget拒绝、隔离live状态内置/独立检查通过。
两组14条JSON边的端点/间距/首次非法索引、反向样本、编译joint range、dense scorer核对通过。
最初dense核对固定1e-5m容差过严失败：该网格最大半间距8e-5m；按距离1-Lipschitz的
半步长误差界重查通过，不改算法。CLI step nan退出2。
细步长C-space PNG目视检查，产物仅ignored tmp/s14_2_edges_step*/。
本地Markdown文件链接、状态一致性和git diff --check通过；未重跑S14.1/前课/P0。
无GUI、UR5e、动力学、RRT或连续通用保证。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；术语精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；文档状态一致性、
本地链接与git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.3手写RRT，等待明确请求。

## S14.3 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_3_rrt.md)、[代码](../examples/15_motion_planning/rrt.py)。
起始pwd正确、git status --short空；无既有未提交工作。
沿用本会话已确认WSL2 Ubuntu24.04.5；本机conda info --base定位hook，激活mujoco，
同执行shell重查环境名/interpreter `/home/lucas/miniconda3/envs/mujoco/bin/python`。
Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、Matplotlib3.11.2，metadata/真实imports/module paths核验；
实际MuJoCo API通过脚本执行核验。无新依赖/安装，3.11兼容目标未执行。

```bash
python examples/15_motion_planning/rrt.py
python examples/15_motion_planning/rrt.py --seed 19
python examples/15_motion_planning/rrt.py --max-nodes 2
```

default/seed19 exit0，success；nodes49/36、attempts48/46、rejected edges2/13。
edge calls50/48、configuration queries490/489；length2.181318/1.861461m，
path exact clearance160.602309/36.103977mm，sampled复查+圆盘解析评分通过。
当次search wall约.230/.208s，只含查询/搜索，非稳定benchmark。
max_nodes2 exit1，node_budget、2nodes/1attempt，path/length/indices/recheck均null，保留部分树。
独立同seed树/parent/path/计数复现，两组全部树边sampled/exact、长度、parent回溯、
steer截断、start/goal非法/同点/直连、node/attempt/time预算、goal_bias1停滞、输入guards通过。
CLI edge step nan exit2；time budget1e-12 exit1；coarse edge step.4 planner success但
后验exact拒绝（−45mm），CLI exit1，未用scorer纠正搜索。
默认树图目视检查；产物仅ignored tmp/s14_3_rrt_*/。
复用S14.2碰撞接口，single-tree RRT/goal bias，末段连接也检查；无RRT-Connect/平滑/时间化。
本地Markdown链接、状态一致性/git diff --check通过；未重跑前课/P0独立实验。
无GUI、UR5e、动力学、硬件或成功率benchmark；time预算是合作式而非硬deadline。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接和git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.4手写RRT-Connect，等待明确请求。

## S14.4 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_4_rrt_connect.md)、[代码](../examples/15_motion_planning/rrt_connect.py)。
保留起始五份modified文档，pwd正确；沿用本会话已确认WSL2 Ubuntu24.04.5。
本机conda info --base定位hook，激活mujoco并同shell核验环境名与
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、
Matplotlib3.11.2，metadata/真实imports/module paths与执行API核验通过；无安装/新依赖，3.11未执行。

```bash
python examples/15_motion_planning/rrt_connect.py
python examples/15_motion_planning/rrt_connect.py --max-attempts 20
python examples/15_motion_planning/rrt_connect.py --max-nodes 12
```

三命令均exit0，无import error，评价所有预算失败；不等于每次搜索成功。
每组seeds7…14×两planner；统一均匀采样（单树goal_bias=0），η=.15m/h=.02m，
same seed不表示same sample sequence。默认RRT/Connect均8/8成功，mean扩展108.625/14.125，
mean config queries872.375/206.875；20attempt RRT0/8、Connect8/8；12nodes各0/8。
48次保存运行的全部树边sampled/exact、η/parent、会合相同、反转/去重与长度通过；
同seed复现、反向root、同点/直连、非法端点/参数、node/attempt/time budget检查通过。
CONNECT每个EXTEND都计attempt并检查预算，总node包含两个root。
默认图目视检查；JSON/PNG仅ignored tmp/s14_4_connect_*/；原S14.3代码未改。
CLI nan exit2，coarse h=.4两planner误成功但解析评分拒绝exit1；极小time预算正常记录失败exit0。
local Markdown links/状态一致性/git diff --check通过；未重跑前课独立脚本或P0。
无GUI、UR5e、平滑/时间化/动态执行或普遍成功率/性能benchmark；只支持本课固定场景观察。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；TRAPPED和采样REACHED的精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接和git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.5 collision-checked path shortcut，等待明确请求。

## S14.5 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_5_path_smoothing.md)、[代码](../examples/15_motion_planning/path_smoothing.py)。
保留起始五份modified文档，pwd正确；沿用本会话WSL2 Ubuntu24.04.5验证。
本机conda info --base定位hook，激活mujoco，同shell核验环境名/interpreter
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、
Matplotlib3.11.2；metadata/实际imports/module paths与执行API核验通过，无新依赖/安装，3.11未执行。

```bash
python examples/15_motion_planning/path_smoothing.py
python examples/15_motion_planning/path_smoothing.py --attempts 0
python examples/15_motion_planning/path_smoothing.py --step-m 0.4
```

默认/0预算exit0，无import error。固定planner seed7/h=.005m新生成输入13顶点/1.774460133m。
独立shortcut seed17，默认100尝试6接受，输出3顶点/1.612792500m，clearance28.191341mm。
0预算保留13顶点与输入长度，clearance6.011070mm；仍执行输入/最终所有边复查。
coarse step.4反例7尝试1接受，输出2顶点/1.6m，sampled复查通过但解析−45mm，exit1。
独立history逐步回放、单调长度、接受新边、起终点/输入保留、同seed重现、final全部边/
密集样本、单点/两点/重复点、输入碰撞/越界/shape/step/seed/budget拒绝通过；CLI负attempts exit2。
默认图目视检查；产物仅ignored tmp/s14_5_shortcut_*/；前课代码未改。
local Markdown links/状态一致性/git diff --check通过；未重跑前课独立CLI/P0。
无GUI、UR5e、时间化/动力学；本课只简化折线，不保证曲率/速度连续、最优或更大clearance。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；实现与连续性精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接和git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.6 cubic时间化与速度/加速度限制，等待明确请求。

## S14.6 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_6_time_parameterization.md)、[代码](../examples/15_motion_planning/time_parameterization.py)。
保留起始五份modified文档，pwd正确；重查WSL2 kernel6.6.87.2 / Ubuntu24.04.5。
本机conda info --base定位hook，激活mujoco，同shell核验环境名/interpreter
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、
Matplotlib3.11.2、Menagerie2026.9.1；required metadata/真实imports/module paths与执行API通过。
Menagerie因复用旧cubic module顶层import必需，未加载UR5e资产；无新增/安装依赖，3.11未执行。

```bash
python examples/15_motion_planning/time_parameterization.py
python examples/15_motion_planning/time_parameterization.py --velocity-scale 0.5
python examples/15_motion_planning/time_parameterization.py --acceleration-scale 0.1
```

三命令exit0，无import error；同一3point path，length1.612792500m/clearance28.191341mm。
复用S10.8b comparison_trajectories cubic，T≥max_j(1.5|delta|/v,sqrt(6|delta|/a))，向上到dt=.01s。
default durations5.77/2.24、total8.01s/802samples；v×.5→11.54/4.47、16.01s/1602；
a×.1→10.75/6.68、17.43s/1744。解析速度/加速度限值均通过。
默认knot5.77s、双侧velocity0、acceleration jump[.741490,.092506]m/s²，C1一般非C2。
唯一时间拼接，NPZ knot保存左acceleration，JSON另存左右jump；普通waypoint不加dwell。
独立三套NPZ/JSON配时ceil/连续解析peak/poly replay、clock/dt、shape/端点/零速度、
几何path/长度不变、全部timed样本几何、零段hold、acceleration主导/input/budget guards通过。
CLI nan scale exit2；默认图目视检查。产物仅ignored tmp/s14_6_timing_*/，前课代码未改。
local Markdown links/状态一致性/git diff --check通过；未重跑前课独立CLI/P0。
无GUI、mj_step、扭矩/接触/UR5e动态执行；参考限值不代表硬件能力，未限制jerk。
Engineering Complete；本人随后明确确认实验与预测完成，并正确回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；归一化导数与约束主导切换精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接和git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.7a UR5e obstacle planning，等待明确请求。

## S14.7a 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_7a_ur5e_obstacle_planning.md)、[代码](../examples/15_motion_planning/ur5e_obstacle_planning.py)。
保留起始六份modified文档与两份untracked S14.6文件，pwd正确；沿用本会话WSL2 Ubuntu24.04.5验证。
本机conda info --base定位hook，激活mujoco，同shell核验环境名/interpreter
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、
Matplotlib3.11.2、Menagerie2026.9.1；required metadata/import/module paths/API核验通过。
无安装/新依赖，3.11兼容目标未执行，不导入无关OpenCV/RL。

```bash
python examples/15_motion_planning/ur5e_obstacle_planning.py
python examples/15_motion_planning/ur5e_obstacle_planning.py --velocity-scale 0.5
python examples/15_motion_planning/ur5e_obstacle_planning.py --max-nodes 2
```

默认/Modify exit0，execution_passed。裸官方UR5e，world固定sphere [−.3344,.3975,.3638]m，
active robot collision geoms8capsules+1cylinder，nexclude0；无base active collision/floor/gripper/held物。
固定world target私有DLS20updates，ep9.366507e−7m/er1.850602e−7rad。
直连margin拒绝，midpoint距离−50.000382mm确认碰撞。RRT-Connect14nodes/25attempts，9→3path/5shortcut接受。
callback/bounds使已有Connect接入六维；time_path改任意D，二维planner/cubic输出逐元素回归通过。
参考h=.04、shortcut.025、fine.01rad；独立.005rad与全部timed配置通过。
reference最小距离54.487516mm，explicit mj_geomDistance capped .1m，sphere–capsule解析核对通过。
默认reference8.488s，execution10.488s/5244steps，max tracking.039322360rad，min distance54.483984mm；
final ep.024871mm/er3.728729e−5rad。v×.5 sim18.974s/9487steps，max tracking.019763381rad，
min distance54.486458mm，final ep.006224mm/er9.933493e−6rad。无observed generated contacts。
首版v=.4rad/s max tracking.077036911rad超.05门限，最终pose虽好仍失败；降默认v到.2通过，未放宽policy。
max_nodes2 exit1/node_budget，保存partial JSON，trajectory/execution null，无执行；CLI nan scale exit2。
默认同seed重跑确定性结果一致。两套NPZ/trace/finer edges/reference clock/command metrics、
final actual FK、primitive capsule距离、失败null独立核对通过；默认tracking图目视检查。
产物仅ignored tmp/s14_7a_ur5e_*/。local Markdown links/状态一致性/git diff --check通过。
execution一次初始化，仅ctrl/mj_step；qfrc_bias理想外力feedforward，不证明硬件扭矩能力。
采样/模型过滤与局部sampling box限制详见学习包；无continuous碰撞保证、实际加速度限值、GUI或硬件验证。
Engineering Complete；本人随后明确确认实验与预测完成，并回答五项Explain，
Run/Modify/Explain全部确认，Learning Mastered；局部采样域与政策验证精度补充见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接和git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
下一小任务S14.7b collision-aware pick-and-place，等待明确请求。

## S14.7b 现场工程验证（2026-10-06，机器 DESKTOP-781D67A）

[学习包](17_7b_collision_aware_pick_place.md)、[代码](../examples/15_motion_planning/collision_aware_pick_place.py)。
起始pwd正确、git status --short空；本会话沿用已确认WSL2 Ubuntu24.04.5。
本机conda info --base定位hook，激活mujoco，同shell核验环境名/interpreter
`/home/lucas/miniconda3/envs/mujoco/bin/python`。Python3.12.14、NumPy2.5.3、MuJoCo3.14.0、
Matplotlib3.11.2、Menagerie2026.9.1、OpenCV distribution4.14.0.94/import4.14.0；metadata/import paths/API核验。
无新依赖/安装，3.11未执行；cv2来自复用执行模块imports，本课不运行图像/PnP。

```bash
python examples/15_motion_planning/collision_aware_pick_place.py
python examples/15_motion_planning/collision_aware_pick_place.py --trials 1 --close-target 0.014
python examples/15_motion_planning/collision_aware_pick_place.py --trials 1 --max-nodes 2
```

评价CLI均exit0，无import error；需看trial status，不将exit0当全成功。
默认已知pose/固定scene，planner seeds7/8/9：3/3 placement_passed，sim21.888/22.122/22.478s，
final error.758213/.929616/.807125mm，min actual distance8.967720/15.624921/22.355081mm。
transfer phase relative change11.512518/12.713926/13.794382mm均<15mm；seed7累计最大约24.61mm，非累计15mm保证。
14mm slide对照在close失败，不执行lift；nodes2对照在transfer node_budget失败，已执行close/lift但不执行transfer。
名义held T_GO=inverse(T_OG)，两次forward/full freejoint pose更新；actual free object无weld/姿态重置/held truth目标反馈。
transit/transfer双树+checked shortcuts+cubic；其余Cartesian动作逐点查参考；名义held tilt≤.15rad/transfer z≥.085m。
phase具体pair+6mmdepth，reference obstacle margin8mm/actual2mm；保持finger-ground与transfer-ground拒绝。
初版step close穿透约6.11mm失败；保持门限，加1s cubic ctrl ramp后通过。
较弱夹紧/raw RRT曾落地，加入高度/倾斜政策后仍有slip失败；checked shortcut缩短运输后保留phase15mm门限通过。
旧诊断仅ignored tmp/s14_7b_diagnostics/，当前失败CLI为14mm或node2，不把旧策略说成仍可直接运行。
默认seed7 transfer48attempts/6shortcuts，reference3.198s，lift52.910115mm；support持续/释放分离/retreat检查通过。
独立三trial全部actual_qpos几何回放、clock/phase/force/final pose、名义held链/参考抽检、
phase/cumulative slip重算、failure stopping/partial trace通过。旧S13.3b默认CLI回归placement_passed，ep.714212mm。
build_model新增可选obstacle，run_pipeline新增可选planner/observer/ramp；默认None/0不改变旧行为。
默认pipeline PNG目视检查；产物仅ignored tmp/s14_7b_pick_*/；CLI nan exit2。
local Markdown links/状态一致性/git diff --check通过；未运行GUI/硬件/ROS2或真实视觉，非物理随机化/普遍成功率结论。
Engineering Complete；本人随后明确确认实验与预测完成，并回答全部Explain，
Run/Modify/Explain全部确认，Learning Mastered；累计relative change的精确定义见学习包。
本次仅同步根README、学习包、示例README、roadmap与handoff；状态一致性、
本地链接与git diff --check通过。未重跑Python/仿真/GUI，runtime沿用2026-10-06本机工程验证。
Stage14学习项均完成；下一小任务S15.1 ROS2环境与node/topic，等待明确请求。

## S15.1 环境核验与教学准备（2026-10-07，机器 Zero）

新增 [node_topic.py](../examples/16_ros2_integration/node_topic.py)、
[示例 README](../examples/16_ros2_integration/README.md)，扩充[学习包](18_ros2_integration.md)。
起始 pwd 正确、git status --short 空；WSL2 kernel6.18.33.2 / Ubuntu24.04.5。
conda info --base 定位 hook，激活现有 mujoco，同 shell 核验环境名与
/home/lucas/miniconda3/envs/mujoco/bin/python；Python3.12.14。
requirements 已读；本课需 apt ROS2 rclpy/std_msgs，不导入前课 helper，无 pip 安装或 requirements 改动。
/opt/ros 不存在、ros2 不在 PATH、未发现 ros-jazzy 包；mujoco 内 find_spec 两包为 None。
sudo -n true 失败：需要密码；未安装系统包或修改 apt 源。
官方 Jazzy 源文档确认 Ubuntu24.04 支持和预编译扩展的解释器匹配要求；
官方 docs 页面访问被 Anubis 拒绝，改读 ros2/ros2_documentation jazzy 原始文档。
本人随后明确授权 ROS2 使用系统 Python，已将专用例外写入 AGENTS.md。

代码：同 topic Int32 / reliable+volatile depth10；publisher 等发现后发送有限序列，
subscriber 验证完整0…N−1；monotonic timeout，发布计数不视作接收 ACK。
通过 conda mujoco Python ast.parse 语法检查、本地 Markdown 文件链接与 git diff --check。
未执行示例、未核验 ROS2 真实 imports/apt versions/API/DDS/GUI；Code/Docs准备完成，
Experiment未完成，非 Engineering Complete；本人 Run/Modify/Explain 均未确认。
下一小步：本人终端完成需 sudo 密码的 Jazzy ros-base 安装，
再核验解释器/真实 imports/版本与双进程默认及0.1s Modify收发；不推进 S15.2。

后续授权核验（同日/Zero）：退出 conda，同 shell 环境名为空、which python3=/usr/bin/python3，
系统 Python3.12.3，rclpy/std_msgs find_spec=None。sudo -n true 仍需要密码。
UTF-8 locale / Ubuntu universe、updates、backports 已存在。
官方 ros-apt-source release API 返回1.3.0；下载 Noble deb 到
/tmp/s15_1_ros2-apt-source_1.3.0.noble_all.deb，dpkg-deb metadata核验。
安装命令已补进学习包，未运行 sudo dpkg/apt、未安装 ROS2；通信验证仍待完成。
保留起始所有未提交教学包修改；本轮规则/文档 links 与 git diff --check 通过。

## S15.1 现场工程验证（2026-10-07，机器 Zero）

本人报告 apt 安装完成；助手退出 conda并source /opt/ros/jazzy/setup.bash，
同shell核验pwd正确、环境名为空、which python3=/usr/bin/python3、ROS_DISTRO=jazzy。
Python3.12.3；真实rclpy/std_msgs module paths均在/opt/ros/jazzy/lib/python3.12/site-packages/，
Int32、Node、QoSProfile imports/API通过；apt rclpy7.1.12-1noble.20260912.162354、
std-msgs5.3.8-1noble.20260911.094424、ros-base0.11.0-1noble.20261006.035407。
dpkg --audit空；无助手追加安装，不运行pip check替代apt检查。

ROS_DOMAIN_ID=51 / ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST。
/usr/bin/python3 examples/16_ros2_integration/node_topic.py subscriber + publisher：
默认period.5与publisher --period .1均完整收到0…9/sequence_passed=True，两进程exit0。
启动至subscriber退出5.491785/1.428506s，含发现开销；日志9interval跨度约4.494/.892s。
ros2 node list --no-daemon、topic list -t --no-daemon、topic info /s15_1/sequence --verbose
均exit0；两node、Int32、1pub/1sub、reliable/volatile；depth输出UNKNOWN不作CLI确认。
无对端各 --timeout 1均exit1，sub空序列/pub0条；nan period/zero timeout/zero count均exit2。
日志与summary仅ignored tmp/s15_1_node_topic/；未运行echo、GUI、MuJoCo或跨机/硬件实验。
文档链接与git diff --check通过；保留起始所有未提交修改。
本人随后明确确认实验与预测完成，并回答全部五问Explain；README三项Learning已勾选，
S15.1 Learning Mastered。解释精度补充：volatile不向晚加入者重放旧消息，不表示无运行时缓存；
reliable不是应用处理ACK，apt解释器兼容要求不等于所有ROS2安装必须用系统Python。
本次仅同步README、学习包、示例README、roadmap、handoff；链接/状态与git diff --check通过。
未重跑Python示例、ROS2通信或GUI；runtime沿用2026-10-07本机工程验证。
下一小任务S15.2 Service，等待本人明确请求；不自动实施。

## S15.2 现场工程验证（2026-10-07，机器 Zero）

按本人请求新增[pose_service.py](../examples/16_ros2_integration/pose_service.py)、
[学习包](18_2_ros2_service.md)。保留起始五份modified文档；pwd正确。
本机WSL2 kernel6.18.33.2/Ubuntu24.04.5；退出conda、source Jazzy，同执行shell核验
空环境名、/usr/bin/python3、ROS_DISTRO=jazzy；Python3.12.3。
rclpy/std_srvs真实imports/Trigger/API/module paths核验，来自/opt/ros/jazzy；
apt rclpy7.1.12-1noble.20260912.162354、std-srvs5.3.8-1noble.20260911.100759；dpkg --audit空。
requirements已读，不导入local helper/MuJoCo，无安装、新pip依赖或colcon需要。

两进程命令：/usr/bin/python3 examples/16_ros2_integration/pose_service.py server + client；
DOMAIN52/LOCALHOST。默认position[.3,0,.1]m client exit0/successTrue；server
--position .3 0 .01、--unavailable、--position nan 0 .1均client exit1/明确不同业务reason。
无server client --wait-timeout .5 exit3；server --delay 1 + client --response-timeout .1
exit4，随后server仍response_ready，证明client停止等待不是remote cancel。
CLI delay nan/response timeout0/wait timeout-1 exit2；闭区间边界/.600001m越界核对通过。
ros2 service type/call成功；普通daemon list exit0但空，不算发现通过；
service list -t --no-daemon --spin-time 2显式发现Trigger通过。缓存/发现时序仅假设，未定位。
所有日志仅ignored tmp/s15_2_service/；transport_error防御分支未注入。
Trigger空request只检查缓存XYZ范围，非完整pose或真实planner；无IK、碰撞、动作/GUI/硬件验证。
本地Markdown链接、状态与git diff --check通过。README Code/Experiment/Docs完成；
本人随后确认实验与预测完成，并回答全部五问Explain；README Learning三项完成，Mastered。
解释精度补充：本地请求提交不证明远端执行；真实观测pose需要时间戳/过期检查。
本次仅同步根README、学习包、示例README、roadmap、handoff；links/状态/git diff --check通过。
未重跑ROS2、Python示例或GUI；runtime沿用2026-10-07本机工程验证。
下一小任务S15.3 Action，等待本人明确请求，不自动实施。

## S15.3 准备与环境核验（2026-10-07，机器 Zero）

本人明确请求；起始pwd正确、git status空。读取交接、示例README/source、requirements与roadmap。
退出conda/source Jazzy，同shell空环境、/usr/bin/python3、jazzy；Python3.12.3，
WSL2 kernel6.18.33.2/Ubuntu24.04.5；rclpy/action_msgs/trajectory_msgs已存在，control_msgs缺失。
apt-get -s install ros-jazzy-control-msgs：只新增5.10.0-1noble.20260911.141005，无升级/删除；
sudo -n true需要密码，已请本人执行sudo apt install ros-jazzy-control-msgs。
新增[trajectory_action.py](../examples/16_ros2_integration/trajectory_action.py)、
[学习包](18_3_ros2_action.md)，示例README声明apt依赖；requirements不改。
标准FollowJointTrajectory两点/单软件joint、feedback/result/cancel；reentrant+两executor线程，
单active goal/busy lock，取消观察后停止更新，非MuJoCo/实际tracking/物理停止。
包含非法goal、abort注入、取消、超时后主动cancel与未知状态语义。
conda mujoco内ast.parse通过；尚未真实import control_msgs/action收发，Experiment未完成。
待本人安装后核验包/真实imports，默认/取消/abort/reject/busy/timeout/CLI检查，
再更新工程结果与Run/Modify/Explain；不推进S15.4。

## S15.3 现场工程验证（2026-10-07，机器 Zero）

本人安装control_msgs后，退出conda/source Jazzy，同执行shell核验pwd、空环境名、
/usr/bin/python3 3.12.3、ROS_DISTRO=jazzy；四包真实imports/module paths来自/opt/ros/jazzy。
apt control-msgs5.10.0-1noble.20260911.141005、rclpy7.1.12-1noble.20260912.162354、
trajectory-msgs5.3.8-1noble.20260911.111503、action-msgs2.0.4-1noble.20260911.052334；dpkg --audit空。
保留起始所有未提交修改；无助手安装、requirements改动或MuJoCo执行。
DOMAIN53/LOCALHOST；/usr/bin/python3 examples/16_ros2_integration/trajectory_action.py server + client。
默认SUCCEEDED/code0/95feedback/q=.5rad/exit0；client --cancel-after .5→CANCELED/
25feedback/heldq=.128135/exit0，cancel确认且terminal之后无reference更新，额外.1s观察。
server --fail-after .5→ABORTED/code−4/24feedback/heldq=.121639/exit1。
client --target2→REJECTED/exit2/无feedback或reference更新。
client --result-timeout .1→主动cancel确认+CANCELED/heldq=.026044/exit4；不将超时报成功。
首goal duration3s时并发第二goal busy拒绝exit2，首goal仍成功；无server exit3；
非法nan duration/timeout0/cancel-after−2 exit2；独立名称/nonfinite/unsupported字段guards通过。
CLI action list/info exit0但空/0server；重启domain53 daemon复核仍失败，原因未定位。
直接rclpy graph2s发现正确action/type/server，通过；CLI不能报告PASS。本轮启动daemon已停止。
日志/summary仅ignored tmp/s15_3_action/；未注入goal响应超时、cancel拒绝/竞态/未知终态或异常分支。
无GUI、实际tracking、MuJoCo或硬件停止验证；本人随后确认实验与预测完成，
并回答全部五问Explain，Run/Modify/Explain全部完成，Learning Mastered。
学习包补充feedback延迟、goal拒绝非GoalStatus终态、取消可早于实际运动、
本课并发与非阻塞单线程替代设计的边界。CLI daemon限制保留。
本次仅同步README、学习包、示例README、roadmap、handoff；本地链接/状态/git diff --check通过。
未重跑ROS2/Python示例/GUI；runtime沿用2026-10-07本机工程验证。
下一小任务S15.4 TF2，等待本人明确请求，不自动实施。

## S15.4 现场工程验证（2026-10-07，机器 Zero）

本人明确请求；保留起始五份modified文档，pwd正确。读取规则/交接/示例与S12.1 source/requirements/roadmap。
新增[tf2_frames.py](../examples/16_ros2_integration/tf2_frames.py)、[学习包](18_4_ros2_tf2.md)。
退出conda/source Jazzy，同执行shell空环境、/usr/bin/python3 3.12.3、ROS_DISTRO=jazzy；
现场rclpy/tf2_ros/tf2_py/geometry_msgs/tf2_msgs imports/module paths/API通过，来自/opt/ros/jazzy。
apt rclpy7.1.12-1noble.20260912.162354、tf2-ros-py0.36.23-1noble.20260915.174232、
tf2-py0.36.23-1noble.20260915.173947、geometry-msgs5.3.8-1noble.20260911.102920、
tf2-msgs0.36.23-1noble.20260915.172312；dpkg --audit空。无新安装/pip依赖/MuJoCo helper导入。

DOMAIN54/LOCALHOST；/usr/bin/python3 examples/16_ros2_integration/tf2_frames.py broadcaster + probe。
默认与两端 --camera-x -.25均exit0；WO原点[-.45,.2,.03]m、非原点p_B[.43,-.21,.06]m一致。
late listener静态获取/任意time静态边、正逆链、history t10/x0+t12/x2→t11/x1、t9/t13拒绝/unknown frame通过。
--stamp-lag1→latest成功但age1.144746s stale拒绝exit1；
--stop-after2 + early probe --observe-for3→latest旧缓存age1.365788s拒绝exit1。
--stop-after1 + 停发后新listener→transform_unavailable/exit3；无发布者timeout.5同exit3。
初版将late listener误预期为缓存stale，实际缺失；澄清动态无历史重放，新增observe-for并分别验证。
CLI nan lag/max-age0/timeout-1/observe-1 exit2；独立nonfinite/非单位quat/future_stamp拒绝通过。
默认/移动动态age.111481/.159058s，非benchmark。日志仅ignored tmp/s15_4_tf2/。
本例树/工具运动为合成，不是当前UR5e实时TF；未做GUI、MuJoCo、旋转插值、全网重复parent检测或硬件验证。
本人随后确认实验与预测完成，并回答五问Explain；README Run/Modify/Explain全部完成，Mastered。
学习包补充latest common time、静态publisher存活/late listener缓存、相机移动对照重启外参的边界。
本次仅同步README、学习包、示例README、roadmap、handoff；links/状态/git diff --check通过。
未重跑ROS2、Python示例或GUI；runtime沿用2026-10-07本机工程验证。
下一小任务S15.5 URDF/joint states，等待本人明确请求，不自动实施。

## S15.5 现场工程验证（2026-10-07，机器 Zero）

本人明确请求；保留起始五份modified文档，pwd正确；读取交接/示例source/requirements/roadmap。
新增[学习包](18_5_urdf_joint_states.md)、[URDF](../examples/16_ros2_integration/ur5e_kinematics.urdf)、
[conda核验器](../examples/16_ros2_integration/urdf_reference.py)、
[ROS核验器](../examples/16_ros2_integration/joint_states_tf.py)。
同执行shell分别核验conda mujoco/所属python，及ROS侧空conda/system python/jazzy；不混import。
WSL2 kernel6.18.33.2/Ubuntu24.04.5。conda Python3.12.14/MuJoCo3.13.0/NumPy2.5.3/Menagerie2026.9.2
metadata与真实imports/module paths核验，非历史MuJoCo3.14版本。系统Python3.12.3，
rclpy/sensor_msgs/tf2_ros/ament_index_python真实imports来自/opt/ros/jazzy。
apt rclpy7.1.12-1noble.20260912.162354、sensor-msgs5.3.8-1noble.20260911.140426、
tf2-ros-py0.36.23-1noble.20260915.174232、RSP3.3.4-1noble.20260915.184559、
ament-index-python1.8.4-1noble.20260519.010916；dpkg --audit空。RSP已安装，apt预览无新增/升级；无安装/requirements改动。

python examples/16_ros2_integration/urdf_reference.py；再--delta .4 --output tmp/s15_5_urdf/reference_delta04.json。
各zero/home/pan_modified×9frame，URDF/MuJoCo max_matrix_error8.882e−16，mj_forward/time0；
compiled joint轴/range/pivot对照通过，按name→id→qposadr映射。
DOMAIN55/LOCALHOST；/usr/bin/python3 examples/16_ros2_integration/joint_states_tf.py：
默认、--reverse-order、.4快照各3配置全frame RSP TF对照exit0，max1.110e−15。
--wrong-order zero仍通过，home拒绝exit1，max矩阵元素误差2（不是2m位置误差）。
独立zero/home快照不变、pan_modified tool改变通过；delta nan exit2。
初版ros2 run wrapper实验超过25s未通过；改ament index直接管理RSP可执行进程后全部通过，
未确定超时唯一原因，不把初版计PASS。参考JSON/log/summary只ignored tmp/s15_5_urdf/。
教学URDF无mesh/collision/inertia/控制，velocity/effort限值占位；tool对应attachment_site。
仅离线三配置FK与JointState/TF，不是实时adapter或UR厂商官方描述；无GUI/动力学/硬件验证。
本人随后明确确认实验与预测完成；README Run/Modify已勾选。
本人随后补齐world/base的Rzπ、wrist/tool的xyz[0,.1,0]/Rx−π/2及attachment_site映射；
Explain完成，README三项Learning全部完成，Mastered。
学习包补充RSP不自动识别合法名称错配、矩阵组合与列向量求值次序区别。
本次仅同步README、学习包、示例README、roadmap、handoff；links/状态/git diff --check通过。
未重跑Python/ROS2/GUI，runtime沿用2026-10-07工程验证；下一小任务S15.6a ROS2 adapters，等待明确请求，不自动实施。

## S15.6a 准备与独立验证（2026-10-07，机器 Zero）

本人明确请求；pwd正确、起始git status空。读取规则/交接/示例README/source/roadmap/requirements。
新增[教学包](18_6a_ros2_adapters.md)、adapter_worker/ros_adapters、typed PlanPose接口package、build与launcher。
本课固定synthetic perception map_estimate→world pose→短DLS IK/cubic→裸UR5e实际mj_step；
ROS/system与conda worker用Unix socket，未混import。无gripper/真实视觉/RRT或碰撞保证/完整抓放。
conda同shell核验mujoco环境/所属python；Python3.12.14、MuJoCo3.13.0、NumPy2.5.3、
Menagerie2026.9.2、Matplotlib3.11.2 metadata/真实imports/module paths通过。
python examples/16_ros2_integration/adapter_worker.py --standalone exit0：reference6.58s/sim7.2s，
max tracking.037672993rad、final ep.116171mm/er7.419434e−6rad；理想bias外力，不证明硬件扭矩。
独立Unix IPC observe/plan/begin/10tick/stop通过，停后sim time/qpos不变；tick与旧plan replay拒绝、
非法workspace拒绝；Code ast.parse与shell bash -n通过。日志仅ignored tmp/s15_6a/ipc_checks/。
取消语义仅simulation暂停，保留速度，非物理停机；单issued plan匹配/消费是教学约定。

ROS同shell核验空conda、/usr/bin/python3 3.12.3、jazzy；rclpy/geometry_msgs/trajectory_msgs/
control_msgs/action_msgs真实imports来自/opt/ros/jazzy，版本见教学包；WSL2 kernel6.18.33.2。
cmake/make/ament/rosidl存在，自定义接口build失败No CMAKE_CXX_COMPILER：缺g++，非pip问题。
apt-get -s install g++仅新增g++/g++13及libstdc++dev共5项，无升级/删除；sudo需密码，
已请求本人执行sudo apt install g++，未获安装确认前不进行依赖后的构建/ROS集成。
requirements未改，无助手安装。接口生成物仅ignored tmp/s15_6a_interfaces/，非提交源。
links/状态与git diff --check通过；README Code/Docs准备、Experiment未完成，Learning均空。
下一步本人安装g++后build，核验generated PlanPose版本/import/API，再验证default、bad-frame/stale拒绝
及cancel停止推进；不启动S15.6b。

## S15.6a 完整链现场验证（2026-10-07，机器 Zero）

本人完成g++安装，助手source Jazzy、空conda/system Python3.12.3同shell核验后build_interfaces exit0。
g++ apt4:13.2.0-7ubuntu1、ament-cmake2.5.6-2noble.20260225.222913、
rosidl-default-generators1.6.1-1noble.20260911.052510，dpkg --audit空。
s15_interfaces0.0.1生成模块来自ignored install，PlanPose Request/Response/type support通过。
首次source install根local_setup错误导致import失败；单包CMake只有share/s15_interfaces/local_setup，
修正launcher与学习命令后通过，未计失败为PASS。

bash examples/16_ros2_integration/run_adapters.sh：typed plan330点/6.58sim秒，
实际360tick/feedback、SUCCEEDED/code0/exit0；sim7.2s，max tracking.037672993rad，
final ep.116171mm/er7.419434e−6rad，与standalone一致。
--bad-frame/--stale各exit1，service显式不同reason，无execution goal且worker simtime0。
--cancel-after .2 exit0，ACK+CANCELED，35feedback/sim.7s；暂停保留非零qvel，非物理停机。
新增launcher末尾两次state观察，间隔.1wall秒；四case均runningFalse，time/qpos不变。
独立actual-q MuJoCo FK/pose error/360×.02clock核对通过；failed new plan使旧plan不可begin通过。
子进程清理检查无仍运行worker/ROS adapter；日志/summary/final_state只ignored tmp/s15_6a/。
当前generated接口与五种ROS模块真实imports重核验；标准包版本见学习包，无追加安装或requirements改动。
保留起始全部未提交工作；源码syntax、shell bash -n、links/状态/git diff --check通过。
无GUI、真实视觉、gripper/抓放、RRT/碰撞保证、硬件制动/时钟同步验证；不把取消后恢复运行标为已测。
本人随后确认实验与预测完成并回答五问Explain；README Run/Modify/Explain全部完成，Mastered。
学习包注明实际typed字段/单位/clock、标准action无plan_id/generation协议、取消暂停并保留qvel。
本次仅同步README、学习包、示例README、roadmap、handoff；links/状态/git diff --check通过。
未重跑Python/ROS2/GUI；runtime沿用2026-10-07本机工程验证。
下一小任务S15.6b Final ROS2 manipulation pipeline，等待本人明确请求，不自动实施。

## S15.6b 完整 manipulation 现场验证（2026-10-07，机器 Zero）

本人明确请求，保留起始五份未提交状态文档。新增[学习包](18_6b_ros2_manipulation.md)、
manipulation_worker/manipulation_ros/run_manipulation与typed service/action；s15_interfaces升级0.0.2保留旧接口。
既有S13.3b run_pipeline只增默认None的pre-step execution_guard；完整接触抓放算法复用。
两环境各同shell核验：conda mujoco/所属Python3.12.14与ROS空conda/systemPython3.12.3/Jazzy。
MuJoCo3.13.0/NumPy2.5.3/Menagerie2026.9.2/Matplotlib3.11.2/OpenCV4.14.0.94 metadata、
真实imports/module paths通过；ROS apt/module paths/新旧接口type support通过，dpkg --audit空；无安装。

build_interfaces.sh exit0；run_manipulation.sh默认exit0/SUCCEEDED，16phase、11114step、
1112feedback、sim22.228s；落点误差.714212mm、相对估计目标朝向误差.008263959°，支撑/分离通过。
live TF位置2.776e−16m/旋转矩阵8.882e−16；actual state恢复mj_forward与trace独立核对通过。
--bad-frame/--stale/--bad-pose/--nonfinite均exit1，明确service reason，无goal/steps0。
--cancel-phase transfer exit0/CANCELED，sim13.52s，无后续下降/释放/退让；保留非零qvel。
--execution-timeout .3 exit4，主动cancel后CANCELED，最终复测sim.26s（调度相关）。
S15_CLOSE_TARGET=.014默认launcher exit1/ABORTED，close双指接触/force不足，sim11.5s，无lift。
停止后两次state间隔.15wall秒相同、runningFalse。产物/详细日志只ignored tmp/s15_6b/；
独立validation_summary.json记录trace、FK、落点、取消/失败phase检查。默认pipeline.png已查看。
旧vision_pick_place.py默认与run_adapters.sh默认回归exit0；无子进程遗留。

仅固定S13.3b合成视觉/CPU自由物体场景；未接S14.7b障碍RRT，无GUI、真实视觉/硬件安全验证。
取消是暂停mj_step而非物理制动；任务失败不自动继续/恢复；cancel-completion竞态有代码保护但未专门注入。
本人随后确认实验与预测完成并提交五问Explain，补正service仅准备初始grasp/approach、后续IK在Action中计算的范围；README Run/Modify/Explain全部完成，Learning Mastered。Stage15学习项全部完成；下一步等待本人选择复盘或新任务，不自动启动新阶段。
最终诊断字段/图像保存检查补充后，.3wall秒timeout smoke再次exit4；close_target/destination诊断、
landmarks.png与停止后state稳定核验通过。Python syntax、bash -n、本地Markdown链接、git diff --check通过。
本次仅同步根README、学习包、示例README、roadmap与handoff；未重跑Python/ROS2/GUI，
runtime沿用2026-10-07工程验证。解释补充：放置需支撑/分离/退让/settling，Action失败终态ABORTED。
本地Markdown链接、CSV shape/clock/独立重算force balance与git diff --check通过；默认PNG已目视检查。
