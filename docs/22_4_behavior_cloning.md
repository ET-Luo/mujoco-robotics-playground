# S19.4 — Tiny CPU Behavior Cloning

## Problem → Why → Intuition

问题：能否用小神经网络从expert示范学习当前observation→action？
本课把已完成的数据合同、episode split、train-only统计接入PyTorch监督训练。
直觉：给网络专家访问过的状态，让它预测专家动作；部署时输入却由网络自己的动作改变。
因此离线loss下降只是预测能力证据，仍须真正MuJoCo闭环执行。本课包含validation rollout smoke，
完整独立test评价/扰动/shift实验分别留到S19.5/S19.6，不提前实施。

## Core Concepts

| 内容 | meaning / shape / unit / frame |
| --- | --- |
| raw observation | [x,y,vx,vy,gx,gy] (6,)，world m/m·s⁻¹/m |
| network input | train统计标准化后的float32 (B,6)，非恒定通道无量纲 |
| network output / label | normalized absolute XY reference (B,2)，不是ctrl、delta或实际qpos |
| physical action | output*action_scale+action_mean，world XY m |
| architecture | 6→32→32→2，两层Tanh；1346个parameter，CPU/1thread |
| train / validation | 1600 / 400 rows；train更新weights，validation选checkpoint |
| test | 文件未加载、不计算loss、不用于训练/选模型或本课rollout |
| constant baseline | train动作均值，忽略observation；使用相同action单位与scorer |

split seed193、训练seed194是两个不同随机源。本课保持S19.3 split和normalizer不变；
训练只shuffle train rows，不跨episode生成sequence。epochs改变训练预算，不改变dataset/task/split。
默认学习率.001/Adam、batch64、200epochs；不做large vision encoder、不需要GPU。
没有reward、critic、PPO/SAC更新；BC从expert动作标签学习。

## Mathematics → Math-to-Code

z_o=(o−μ_o)/s_o，z_a=(a−μ_a)/s_a，μ/s只来自train。
fθ(z)=W₃ tanh(W₂ tanh(W₁z+b₁)+b₂)+b₃。
L_BC=(1/(B·2)) Σ_i Σ_j (fθ(z_o)_ij−z_a_ij)²。
对prediction的梯度是2(prediction−target)/(B·2)，网络通过chain rule反传；Adam根据梯度更新θ。
原单位action RMSE=√mean[((prediction−target)*s_a)²]，单位m，均匀平均所有rows/两轴。
归一化MSE对各通道按1/s_a²加权，不能与m²直接混用。
部署：a=fθ((o−μ_o)/s_o)*s_a+μ_a。constant baseline为train raw action mean。
恒定通道采用保存的有效scale，不能拿raw std0直接反变换。

`load_prepared`核验train/validation/normalizer hash、split无交集、row IDs、normalization及train统计。
`make_policy`是直接nn.Sequential，没有高层学习框架。
每batch：optimizer.zero_grad→policy(x)→MSELoss→loss.backward→optimizer.step。
`zero_grad(set_to_none=True)`清空之前梯度；`backward`原地累积parameter.grad，不直接更新weights；`step`更新weights。
`nn.Linear(in,out)`构造可学习仿射层，输入(B,in)输出(B,out)；Tanh逐元素非线性。
`torch.tensor(...,dtype=float32)`复制训练数据到CPU tensor；NumPy的统计仍保留float64。
`policy.train()/eval()`设置模块模式；当前无dropout/batchnorm仍明确区分用途。
`torch.inference_mode()`禁止评价计算构建autograd图；validation没有optimizer更新。
每epoch结束重新计算full train/validation loss，最低validation checkpoint用deepcopy保留；epoch0也可入选。
保存`policy.pt`的state_dict/结构/seed/normalizer/split hash；重新`torch.load(...,weights_only=True,map_location='cpu')`
载入并验证prediction完全一致，不执行任意pickle对象。

## Minimal Experiment

执行依赖requirements已有torch==2.14.0+cpu、numpy、mujoco、matplotlib。
helper为episode_contract→cartesian_spring，同后三包；不需要SB3/Gymnasium/Menagerie/ROS。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193
```

若本机没有该prepared目录，运行S19.3，将其打印的split-seed193输出目录代入--prepared。
不要传原始S19.2数据，也不自动挑最新目录。每命令打印新的ignored tmp/s19_4/run_<UTC timestamp>/。
输出policy.pt、results.json、loss.csv/loss.png及4 validation reset×3policy的trajectory NPZ。
未传入新episode/goal示范；smoke从validation timestep0取实际初态与goal，expert/constant/BC各fresh MjData。
相同2s/100actions、50Hz policy、1kHz反馈、20N motor cap、末10观测error<1mm且speed<2mm/s。
不额外裁剪BC reference，也不直接赋qpos；每个动作只经已有env.advance→实际mj_step推进。
`MjModel.from_xml_string`返回编译模型；`MjData`返回独立状态；`mj_forward`原地更新不推进时间，
`mj_step`原地积分1ms，无新状态返回值。非finite动作明确拒绝，不能静默算成功。

## Expected / Actual Result

2026-10-11 DESKTOP-781D67A / WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco所属Python3.12.14，
NumPy2.5.3/MuJoCo3.14.0/Matplotlib3.11.2/Torch2.14.0+cpu metadata/import/API/helper路径核验。
Torch CUDA build=None、实际tensor device CPU；无安装或requirements改动。
现场命令：

```bash
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193 --output tmp/s19_4/default
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193 --epochs 20 --output tmp/s19_4/short
```

| 指标 | 默认200epochs | Modify20epochs |
| --- | ---: | ---: |
| 最低validation checkpoint epoch | 79 | 20 |
| selected train normalized MSE | .00183338 | .00640601 |
| selected validation normalized MSE | .00759851 | .01326897 |
| validation BC action RMSE | .734121mm | 1.062852mm |
| validation constant action RMSE | 8.001629mm | 8.001629mm |
| training wall（不含输入加载/保存/rollout） | 1.730s | .167s |
| expert / constant / BC闭环success | 4/4 / 0/4 / 0/4 | 4/4 / 0/4 / 0/4 |

两个命令exit0表示工程流程检查通过，BC任务success明确为0/4，不能记为机器人任务PASS。
默认BC四条tail max error为3.514/9.286/5.503/10.664mm；speed1.299/.926/2.501/.453mm/s。
short为24.307/10.619/8.391/11.927mm；speed1.911/7.321/26.882/.079mm/s。
默认1个validation-low-loss checkpoint，不是多training-seed benchmark；未宣称鲁棒/泛化性能。

## Explanation / Failure Analysis

loss图显示train继续下降但validation在79附近最低，故使用79而非最后200；不是early stopping，仍完成固定预算。
val RMSE .734mm是**在专家访问的状态上**对action的平均误差，不是末端闭环位置误差。
网络动作决定下一状态，下一输入可能脱离示范分布；小动作偏差、目标附近固定点偏差及速度反馈输入的差异
均可能影响rollout。实际已确认末尾位置超1mm，尚未通过干预/状态覆盖实验确定各原因贡献，后续S19.5/6分析。
不能将0/4简单解释为MuJoCo或训练程序坏了，也不能靠放宽criterion变成成功。
constant的低性能说明忽略观测不足；BC胜过constant的离线loss也不是操作成功证据。

验证：saved checkpoint prediction逐元素一致；独立重算validation MSE与CSV argmin；
24条保存rollout独立逐physics步Euler重算、逐tick因果动作/反归一化与scorer检查通过。
MSE数值/梯度示例pred[1,2]/target0→loss2.5、grad[1,2]通过。
非法epochs0/width3/seed−1/已有output均exit2，prepared hash损坏拒绝；Python3.11语法解析通过（未实际3.11执行）。
两loss.png已目视查看，links/anchors/git diff --check通过。无GUI、物体抓取、test评价或扰动恢复实验。

## Failure Cases / Robotics Context

- test选epoch、把val参与梯度或重新拟合normalizer会污染评价。
- 忘记zero_grad会累积旧梯度；忘记deepcopy可能把“最佳weights”变成最后weights。
- normalized动作直接当m或ctrl会破坏物理合同；CPU/safe checkpoint也仍需配套normalizer与模型结构。
- 低平均MSE可能掩盖关键瞬间误差、temporal dependence和多个合法动作模式；小MLP不解决multimodality。
- 4 reset与单train seed只作smoke；当前分布成功/失败都不能推出OOD能力。
应用：classical expert蒸馏、teleoperation行为克隆、后续ACT/Diffusion比较的最小baseline。
30秒 Interview：BC用监督loss拟合专家动作，模型由validation选；部署反归一化输出并真实闭环评价，loss低不等于成功。
2分钟 Interview：解释6→32→32→2模型、MSE/gradient/Adam数据流、train-only normalization与checkpoint；
用.734mm offline/0of4 rollout说明专家分布与policy诱导分布差别，最后说明test尚未使用与后续验证边界。
Must Remember：grad更新只用train；validation选择；test独立；action单位与closed-loop成功不能被loss替代。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。Learning三项待本人完成。
Run：执行默认命令，读loss.png/CSV的selected epoch与results.json，区分offline RMSE及三policy success分母。
Modify：先预测仅减少epochs200→20（split/seed/width/lr不变）的影响，再运行：

```bash
python examples/20_learning_from_demonstrations/behavior_cloning.py --prepared tmp/s19_3/verified193 --epochs 20
```

Explain（五问）：
1. BC输入/标签/部署输出分别是什么？为何不是reward优化或直接预测qpos？
2. zero_grad、backward、step各做什么？MSE在哪些维度取平均？
3. 为什么用validation选择checkpoint、deepcopy weights，却不加载test来选择？
4. 为什么必须保存action scale/mean并反归一化？这与motor ctrl如何衔接？
5. BC validation动作RMSE较小却闭环0/4说明什么？还缺哪些实验才能解释shift或泛化？

下一小任务S19.5 closed-loop evaluation / offline vs rollout，等待明确请求；本轮停止。
