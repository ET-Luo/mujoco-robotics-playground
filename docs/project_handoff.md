# 项目经验与进度交接

最后整理：2026-09-24。供新会话的 Codex 和学习者快速恢复上下文。
先读根目录 [AGENTS.md](../AGENTS.md)，再读本文；执行前重新检查实际环境。

## 当前目标与边界

2026-09-24 排查更新：用户已明确允许本会话主动激活 `mujoco` 环境。
激活后确认 Python 为 `/home/lucas/miniconda3/envs/mujoco/bin/python`，版本
3.12.14，MuJoCo 3.13.0，Menagerie 2026.9.2；没有安装或升级依赖。
`python examples/02_ur5e_basics/main.py --headless` 完成 1,000 步，退出码 0，
模块缺失未复现。`timeout --kill-after=3s 30s python -u
examples/02_ur5e_basics/main.py --viewer --steps 2500` 完成 5.000 秒仿真，
退出码 0，此次未复现历史 GUI 退出崩溃；桌面画面及交互仍待用户确认。
另已启动 `--viewer --steps 150000` 供用户观察，其完成结果尚未验证。
用户随后确认只看到 `WARN:COPY MODE`，没有仿真画面，因此 GUI 显示仍未通过。
现场 WSLg 版本为 1.0.73.2；`/mnt/wslg/weston.log` 启动日志记录
`rdp_allocate_shared_memory: Failed to open "/mnt/shared_memory/{...}" with error: Input/output error`，
随后 `use_gfxredir = 0`、`enable_copy_warning_title = 1`。MuJoCo 窗口已登记到 WSLg，
长时仿真进程检查时仍存活且未输出新错误。证据指向 WSLg 共享内存/窗口传输问题，
但尚未验证恢复方法。建议保存工作后在 Windows PowerShell 执行 `wsl --shutdown`，
重开 WSL 后复测；若仍失败，再按微软文档更新 WSL。未代为关闭 WSL，以免中断其他工作。
下一小步：重启 WSLg 后确认 UR5e 画面可见；此前退出码 0 仅代表进程测试通过。

这是面向 Robotics Software Engineering、Embodied AI、Robot Learning 和机械臂操作的
个人学习与面试准备仓库。采用官方 MuJoCo API，CPU 优先，一次学习一个概念。
用户最近关注：如何打开 UR5e GUI，并在窗口中操作机械臂。

已完成基础搭建和 UR5e 加载/检查演示。没有实现自定义控制器、PD、IK、Cartesian
control、轨迹规划、RL 或任务环境。后续阶段仅有文档占位；不要自行推进高级算法。
代码完成不代表用户已经掌握对应知识。

## 进度与入口

| 内容 | 状态 | 入口 |
| --- | --- | --- |
| 仓库结构、环境规则、检查脚本、最小依赖 | 已完成 | [根 README](../README.md)、[环境检查](../scripts/check_env.sh) |
| 单铰链基础仿真 | 无界面验证通过 | [示例 01](../examples/01_basic_simulation/README.md) |
| 官方 UR5e 加载、关节/执行器检查、单目标微调 | 无界面验证通过 | [UR5e 示例](../examples/02_ur5e_basics/README.md)、[代码](../examples/02_ur5e_basics/main.py) |
| GUI 正常操作与干净退出 | 尚未完整验证；有退出错误 | 下文“GUI 已知问题” |
| Joint control 及后续阶段 | 未实现 | [学习路线](learning_roadmap.md) |

学习顺序：`01_basic_simulation` → `02_ur5e_basics` → `02_joint_control`。
两个 `02_` 目录是有意保留的，不要仅为统一编号重命名。

## 环境与启动

此前实测环境（不是新会话的自动保证）：

- Ubuntu 24.04 / WSL2，Miniconda 环境名 `mujoco`，VS Code 连接 WSL。
- Python 路径：`/home/lucas/miniconda3/envs/mujoco/bin/python`。
- Python 3.12.14、MuJoCo 3.14.0、`mujoco-menagerie` 2026.9.1。
- 代码兼容目标为 Python 3.11，但尚未在 3.11 运行验证；不要擅自替换现有环境。
- 依赖仅有 `mujoco`、`numpy`、`matplotlib`、`mujoco-menagerie`，版本未锁定。

用户在新终端的启动流程：

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
```

只有确认环境为 `mujoco` 且 Python 属于该环境后，才执行：

```bash
python --version
bash scripts/check_env.sh
python examples/01_basic_simulation/main.py
python examples/02_ur5e_basics/main.py --headless
```

Codex 检查到环境未激活时必须停止执行并报告。不要在 base/system Python 安装或运行。
此前已安装 Menagerie；新会话先检查，不要每次重复安装或升级依赖。

## 已验证的模型经验

- 基础示例：一个被动铰链，没有执行器；`nq=nv=1`，1,000 步推进到 2.000 秒。
- UR5e 使用 `mujoco_menagerie.load("universal_robots_ur5e")` 返回 `MjModel`，
  然后创建 `mujoco.MjData(model)`。默认加载包含地面的 scene。
- 包名是 `mujoco-menagerie`，Python 导入名是 `mujoco_menagerie`。
- 首次加载联网下载该模型，默认缓存于 `~/.cache/mujoco_menagerie`；
  不需要复制整个 Menagerie 仓库或把网格资产提交到本项目。
- 实测 UR5e：`nq=6`、`nv=6`、`nu=6`，6 个关节、6 个执行器、8 个 body（含 world）。
- 关节名称为 `shoulder_pan_joint`、`shoulder_lift_joint`、`elbow_joint`、
  `wrist_1_joint`、`wrist_2_joint`、`wrist_3_joint`；对应执行器名称去掉 `_joint`。
- `qpos` 是实际广义位置，`qvel` 是实际广义速度；`ctrl` 的含义取决于执行器配置，
  不能对任意模型都理解为目标角度或力矩。
- 此 UR5e 用 `general` 执行器实现位置伺服。代码检查 transmission、gain/bias、
  dynamics、gear 和范围后才把命令作为目标角度，不能仅凭名称判断。
- 从 `home` keyframe 同时初始化姿态和控制值，避免只设置 `qpos` 而让目标仍为零。
- 用 `actuator_trnid` 查执行器对应关节，再通过 `jnt_qposadr` 找位置地址；
  不要假设 actuator ID 就是 qpos 索引。
- 只将 `shoulder_pan` 目标由 -1.5708 改为 -1.5208 rad，并检查关节/控制范围。
  1,000 步后时间为 2.000 秒、状态有限，实际角度变化约 0.049998 rad。
  其余控制值未改变，但其他关节仍可能因重力和动力学耦合而运动。

以上来自此前真实运行；详细版本、模型 revision 和输出摘要见 UR5e 示例 README。

## GUI 操作经验与限制

在确认环境后运行以下命令可延长窗口操作时间：

```bash
python examples/02_ur5e_basics/main.py --viewer --steps 150000
```

当前 timestep 下约为五分钟仿真时间，实际墙钟时间可能更长。默认 1,000 步约两秒
就结束；自动关窗不一定是模型加载失败。此长时交互命令已向用户建议，但尚未实测。

| 操作 | 方法 |
| --- | --- |
| 旋转 / 平移 / 缩放视角 | 左键拖动 / 右键拖动 / 滚轮 |
| 显示或隐藏左 / 右面板 | `Tab` / `Shift+Tab` |
| 查看快捷键 | `F1` |
| 调整执行器目标 | 展开右侧 `Control`，小幅修改一个目标 |

建议先把 `shoulder_pan` 从约 -1.52 调到 -1.47 rad，观察对应关节。
这是操作建议，不是已验证的用户交互结果。GUI 控制修改由 `viewer.sync()` 同步；
当前循环不会每步覆盖目标。手动操作后不再满足“只改变一个命令”的自动演示条件。

当前采用 `launch_passive`，仿真由 Python 的 `mj_step` 循环推进。
**脚本没有暂停回调，按空格不会暂停它**；不要把 managed viewer 的暂停行为套用过来。
拖动刚体施加扰动也不等于实现末端目标控制或 IK。

操作依据：[官方 viewer 文档](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)、
[官方快捷键](https://mujoco.readthedocs.io/en/stable/programming/samples.html#shortcuts)。

## GUI 已知问题（保持与 headless 结果分开）

1. 基础示例：仿真到达 2.000 秒后出现 `GLXBadDrawable` / `X_GLXSwapBuffers`，
   随后测试超时，退出码 124。
2. UR5e 示例：完成全部仿真和最终状态打印，但退出时发生以下错误，退出码 134：

```text
Fatal glibc error: pthread_mutex_lock.c:450 (__pthread_mutex_lock_full): assertion failed: e != ESRCH || !robust
timeout: the monitored command dumped core
```

此前诊断发现 `DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`、WSLg 1.0.66，
没有 `glxinfo`。第一类错误疑似 GLX/WSLg 或关闭路径问题；第二类表现在 native
GUI/线程退出路径。**确切原因都未确认，也未修复**，不能声称 GUI 已验证正常。
没有安装额外图形依赖或设置渲染后端 workaround。

若用户要求诊断 GUI，先保留准确报错、退出码、是否打印最终状态，分别复测 headless
和 GUI。不要因关闭时报错就回退已通过验证的模型加载，也不要仅凭异常名称断言根因。
历史测试命令和完整记录见 [MuJoCo notes](mujoco_notes.md) 与 UR5e 示例 README。

## 下次如何继续

1. 读 `AGENTS.md` → 本文 → 与当前请求有关的示例 README/代码。
2. 检查 `git status --short`；此前 UR5e 工作仍有未提交修改，不要清理或覆盖。
   此记录不保证新会话时的 Git 状态，必须现场核对。
3. 按用户当次目标推进；若只是继续学习，建议先理解一个执行器目标与实际关节角的关系。
4. 用户若要求改善交互，可在这个示例中增量增加暂停/持续运行功能；这些目前尚未实现。
5. 完成工作后更新本文的状态和实测结果；同步相关 README，避免把建议写成已完成事项。

本次交接整理仅修改文档；没有重跑 Python/仿真，也没有新增运行验证结论。
