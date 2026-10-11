# S19.2 — Expert Demonstration Generation / Trajectory Dataset

## Problem → Why → Intuition

问题：怎样将一个可解释controller的运行变成一份有来源、可复现、可审查的trajectory dataset？
单个episode验证接口；多个episode才覆盖不同start/goal。示范质量不只看文件能否读取，
还要检查expert在实际MuJoCo闭环中是否达到任务标准。
本课复用S19.1的expert和Stage17 XY装置；新的主概念是**有边界与来源的episode集合**。
只完成采集，不划分train/val/test，不拟合normalizer，不训练BC。

## Core Concepts

Dataset是D={τ_i}，每个τ_i仍保持episode边界。一个NPZ是一条episode，manifest是集合索引。
expert生成动作，低层controller执行；实际状态只由mj_step积分，不拿goal/reference直接改qpos。

| 内容 | 合同 / shape / unit / frame |
| --- | --- |
| start | 两轴独立Uniform[-.03,.03]m；初速度[0,0]m/s；world XY |
| goal | X Uniform[-.02,.02]m，Y Uniform[-.01,.01]m；episode内固定 |
| observation | (T+1,6)：[x,y,vx,vy,gx,gy]，m / m/s / m |
| action | (T,2)：world XY绝对reference，m |
| time_s | (T+1,)，policy tick间隔.02s；底层physics/feedback .001s |
| terminated / truncated | (T,) bool；terminated全False，最后truncated=True |
| episode index | id、seed、file、SHA256、初态、goal、结果与采集source |
| task criterion | 末10个观测全部error<1mm且speed<2mm/s；success独立于truncation |

默认24条episode，seed1900…1923，每条100次action/2s。短horizon对照T20/.4s。
`bc_candidate`只是成功标记，不是已完成训练筛选；全部attempt的NPZ和record都保留。
不合格expert轨迹不能自动当正确BC标签；失败数据可供诊断，未来恢复/relabel另行明确。
source为classical_controller，不是人类teleoperation；没有外部dataset。

## Mathematics → Math-to-Code

τ_i=(o_0,a_0,o_1,…,a_(T−1),o_T)，D含24个独立reset episode。
a_t=x_t+clip(g−x_t,−.005,+.005)m；低层F=400(a_t−x)−Dv，
D=2√(M·400)N·s/m，M=[2,1]kg，实际slide广义力cap±20N。
这些式子复用S19.1，不重新设计expert。

`generate_demonstrations.generate`按episode seed迭代；`run_episode(seed,goal_x,horizon)`生成真实闭环轨迹。
X goal使用`SeedSequence([seed,192])`独立stream；原reset stream仍采start与goal Y。
因此改变horizon不会改变初态/goal，增加episodes也保持前面的episode不变；seed不同不是task success保证。
`np.savez_compressed`按字段保存数组；`np.load(...,allow_pickle=False)`重载并逐字段校验。
manifest记录model XML与三个源码SHA256、Python/package版本、机器名、分布、controller、horizon、criterion。
文件SHA验证字节完整性；数组一致性才用于检验同配置再生成，不要求ZIP容器字节必然一致。
版本/hash帮助追溯，并不能替代跨机器数值验证或环境锁文件。

MuJoCo API沿用S19.1：`MjModel.from_xml_string(XML)`返回编译模型；`MjData(model)`返回独立状态；
`mj_forward`原地计算、不推进时间；`mj_step`原地推进1ms。每条episode随后用fresh MjData重放动作。
重放是数据一致性检查，expert原采集每50Hz使用最新状态才是闭环。
本课只给run_episode增加可选horizon（默认100，旧S19.1行为不变），未修改旧Stage17模型。

## Minimal Experiment

依赖requirements已有mujoco/numpy/matplotlib；episode_contract→cartesian_spring helper也导入Matplotlib。
不需要Torch、Menagerie、OpenCV、ROS或新包。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/generate_demonstrations.py
```

命令打印新目录`tmp/s19_2/run_<UTC timestamp>/`，含24个NPZ、manifest.json、dataset.png。
每次默认创建新目录；`--output`可指定目录，但已存在目录拒绝覆盖。按打印路径打开PNG与manifest。
图左：实际world XY路径，圆点start、叉号goal，蓝色success/红色未达标；图右是全attempt结果分母。
报告generation_wall_s包含physics、fresh replay、保存/重载，排除绘图；不是纯模拟速度benchmark。
若运行中断，可能留下无完整manifest的目录，不能将其当成完整dataset；选新目录重新采集。

## Expected / Actual Result → Explanation

预期默认expert在2s内恢复；短horizon可能格式检查通过但任务不合格。
2026-10-11 DESKTOP-781D67A / WSL2 Ubuntu24.04.5 / kernel6.6.87.2；mujoco所属Python3.12.14，
MuJoCo3.14.0/NumPy2.5.3/Matplotlib3.11.2 metadata、import/API/helper路径核验，无安装。
现场命令：

```bash
python examples/20_learning_from_demonstrations/generate_demonstrations.py --output tmp/s19_2/default
python examples/20_learning_from_demonstrations/generate_demonstrations.py --horizon 20 --output tmp/s19_2/short
python examples/20_learning_from_demonstrations/generate_demonstrations.py --output tmp/s19_2/repeat
python examples/20_learning_from_demonstrations/episode_contract.py
```

默认24/24 success，2400 transitions，48000采集physics步及48000 replay步，generation wall约4.931s。
短horizon0/24 success，480 transitions，9600采集步及9600 replay步，wall约1.043s。
两组全部24条均保存；短组不是工程错误，也不能称专家任务成功。
重复默认24/24、wall约4.773s，同seed保存数组逐字段完全一致，短组obs/action与默认前缀一致。
72个NPZ独立SHA/shape/time/终态/success重算通过；独立Euler递推逐physics步重算与saved状态一致（atol1e−12）。
默认tail全episode最大error9.046e−6m、speed1.119e−4m/s；短组4.186e−2m、5.578e−2m/s。
默认输出约226KB、短组155KB（含manifest与PNG），只有ignored tmp保存生成物。
非法episodes0/seed−1/horizon9与nan、已有output目录均exit2；Python3.11语法解析通过，未实际在3.11运行。
旧S19.1默认回归exit0/3of3；两张dataset.png已目视查看。
改变horizon也改变验收时间窗：T100末10帧为1.82…2s，T20为.22….4s。
因此短组是采集时长反例，不能拿它与默认当作同horizon策略benchmark，也不合并进默认训练集合。
默认24/24只说明此分布、固定24seed与简化fixture上的expert表现，不证明coverage充分或机器人泛化。
没有训练loss、learned policy rollout、物体抓取或GUI验证。

## Failure Cases

- 把多条trajectory直接拼成一条会造出reset时刻的假transition；未来window不可跨episode。
- 只保留成功记录后才算success率会抹掉失败；指标必须使用全部attempt分母。
- 可读/shape正确不保证label可用；短episode保存一致却可能没有成功行为。
- 文件名/目录混用导致旧NPZ与新manifest不一致；本脚本拒绝已有目录。
- 不记录controller/model/horizon就无法解释同seed结果变化；修改expert仍称原版本会污染比较。
- 多数据不自动带来generalization；相邻帧高度相关，后续S19.3必须按episode划分。
- Offline loss在expert分布上测得，未来policy误差改变访问状态；本课没有验证这种shift的恢复能力。

## Robotics Context / Interview Capsule

应用：classical expert蒸馏、批量示范采集、teleoperation recorder的数据格式基础。
30秒：示范dataset是带episode边界的时序轨迹集合；记录分布/版本/seed，实际闭环验收并保留失败分母。
2分钟：先解释o/a单位与50Hz/1kHz执行，再说明独立reset采样、NPZ+manifest、source与hash；
讲短horizon证明数据正确与task success分离，最后强调后续episode split和learned rollout。
Must Remember：先验证expert再训练；失败保留但不自动模仿；数据来源和评价协议是算法比较的一部分。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。Learning三项待本人完成。
Run：执行默认命令，核对24条episode、2400 transitions、success分母与一条NPZ的shape；查看dataset.png。
Modify：先预测仅将horizon100→20会改变哪些字段、哪些同seed数据保持一致，再运行：

```bash
python examples/20_learning_from_demonstrations/generate_demonstrations.py --horizon 20
```

对比两份manifest和图，说明为何0/24与脚本工程PASS可以同时成立。
Explain（五问）：
1. 24条各100个action为何是2400个transition，却有2424个observation？能否跨episode配对？
2. 为什么从不同初态/goal采集比重复同一条轨迹更有价值？不同seed是否保证覆盖充分？
3. manifest为何同时记录seed、分布、model/controller来源与horizon？hash能否保证跨机器完全复现？
4. 短horizon数据全部保存、0/24成功；为什么不删除失败分母，也不能自动把这些轨迹用于成功行为BC？
5. expert闭环成功与saved-action replay一致分别证明什么？为何都不等于未来learned policy闭环成功？

下一小任务S19.3：episode-level split / train-only normalization。等待明确请求，本轮停止。
