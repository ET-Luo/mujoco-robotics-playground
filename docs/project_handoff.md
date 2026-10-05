# 项目经验与进度交接

最后整理：2026-10-05（Stage 12 与 S13.1 Engineering Complete + Learning Mastered）。
新会话先读 [AGENTS.md](../AGENTS.md) → 本文 → [README](../README.md) → 当次示例。
README 是 Stage 10 起 Engineering / Learning 的唯一状态来源。

## 当前边界与下一步

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
S13.1 已实现，本人确认实验与预测并回答五项 Explain，Learning Mastered；下一可选任务 S13.2 等待明确请求。

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
解释记录与矩阵链、完整 pose、米制平移检查的精度补充见学习包；下一可选小任务 S13.2，等待明确请求。
本次仅同步六份文档，保留既有未提交代码/文档；本地链接、状态一致性与 git diff --check 通过。
未重跑实验/仿真/GUI，runtime 证据沿用 2026-10-05 Zero 工程验证。
STOP，不自动实现 S13.2。
