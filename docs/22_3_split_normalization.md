# S19.3 — Episode-Level Split / Train-Only Normalization

## Problem → Why → Intuition

问题：怎样准备可公平评价的BC数据，避免模型提前看到评价集信息？
机器人相邻帧有时间依赖，随机按row划分会把同一试验分散到训练和评价集合。
先分episode，再拟合train统计，再用同一统计变换所有集合。像准备考试：可以统一单位，不能用考题来决定训练处理规则。
本课只做预处理；没有网络、loss或learned-policy rollout。

## Core Concepts

| 内容 | 含义 / shape / unit |
| --- | --- |
| train | 用于拟合normalizer与未来policy参数；默认16episode/1600rows |
| validation | 未来模型选择/超参数调整；4episode/400rows |
| test | 最后冻结模型后的评价；4episode/400rows，不能用来调参 |
| BC输入 / 标签 | observations[:-1] (T,6) / actions (T,2)，最后observation不加入动作配对 |
| 原始量 | world XY position/reference/goal m，velocity m/s |
| normalized量 | 每channel z-score；非恒定通道无量纲；不裁剪 |
| row keys | episode_id / timestep；拼接后仍可恢复episode边界 |

使用S19.2 manifest/NPZ，读取前核验hash、schema、shape、finite、clock与终态。
重复id/seed/file拒绝，防止同源episode伪装成独立split。当前单个dataset每seed对应独立reset；
未来recovery或同seed多份采集要按共同source group分组，不能仅换名字来隔离。
默认success-only是本课明确的baseline选择；记录excluded IDs与全部attempt/success数量。
这不是逐动作质量判断：时间不足失败仍可能有正确动作。`--selection all`仅允许显式预处理全部轨迹，
不证明全部轨迹适合BC。少于3条eligible episode拒绝，因为三个集合必须非空。

## Mathematics → Math-to-Code

对train输入和action分别逐channel计算：
μ_j=(1/N)Σx_ij；σ_j=√[(1/N)Σ(x_ij−μ_j)²]，即population std / ddof=0。
s_j=σ_j；若σ_j≤1e−12 max(1,|μ_j|)，标记constant并令s_j=1。
z_ij=(x_ij−μ_j)/s_j；x_ij=z_ij s_j+μ_j。
μ和s保留该channel的原单位；恒定通道fallback约定scale为原单位中的1，不代表它的真实std为1。
相同常量减去mean变成0；held-out该通道变化仍保留，不强行清零。当前真实6个obs/2个action通道无constant，
另用合成[0,1,2,3]与恒定.25两列验证零方差处理。

`split_ids`对sorted IDs用seeded permutation；val/test各max(1,N//6)，train取剩余。
默认N24→16/4/4。先固定IDs，再`stack_pairs`，最后`fit_normalizer(train)`。
`normalize`与`denormalize`是显式NumPy公式；没有隐式sklearn拟合或MuJoCo helper。
每条episode等长时row统计也等价等episode权重；未来可变长度会更偏重长episode，必须注明采样/加权规则。

## Minimal Experiment

执行依赖仅requirements已有NumPy；不导入MuJoCo/Torch/Matplotlib/local helper，不安装新包。

```bash
cd ~/projects/mujoco-robotics-playground
conda activate mujoco
pwd
echo $CONDA_DEFAULT_ENV
which python
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default
```

如果本机没有这个ignored dataset，先运行S19.2采集命令，并将它打印的新目录传给`--dataset`；
不要依赖别的电脑的tmp，也不自动选择“最新”目录以免误拿short数据。
每次打印新目录tmp/s19_3/run_<UTC timestamp>/，内含train/validation/test.npz、normalizer.npz、split.json。
各split NPZ含raw/normalized observations与actions、episode_id/timestep；normalizer保存mean/std/scale/constant。
split.json保存IDs、selection、输入manifest与输出文件hash、预处理源码hash、版本、统计和诊断。
原始dataset保持不变；已存在output拒绝覆盖。后续policy必须保存并复用同一split和normalizer，不能每轮重新分割。

## Expected / Actual Result → Explanation

2026-10-11 DESKTOP-781D67A，WSL2 Ubuntu24.04.5/kernel6.6.87.2；mujoco环境所属Python3.12.14，
NumPy2.5.3 metadata/import路径与load/std API核验；无安装。
实际命令（最终代码）：

```bash
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default --output tmp/s19_3/verified193
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default --split-seed 194 --output tmp/s19_3/verified194
```

两命令exit0、16/4/4与1600/400/400；默认selection排除0条。输入沿用2026-10-11 S19.2实际MuJoCo数据，
本轮没有重跑采集/physics。只变split seed会改变成员与统计，不改变源轨迹；同seed不受输入ID次序影响。
train normalized mean约0/std约1；val/test不同，这是正常结果，不应分别refit以强行变成0/1。
反归一化各组最大error≤6.94e−18（按各原始单位）。
错误row split的train/val、train/test、val/test均共享24条源episode；正确episode split交集0。
合成held-out X+100mm诊断：train-only μ/scale不变；错误all-data拟合的X mean变+33.333mm。
这是信息泄漏敏感性演示，不是物理扰动rollout，不根据test统计来选择split seed。
常量probe mask=[False,True]、scale=[1.118034,1]，finite/roundtrip通过。

保存后独立重算train μ/σ、全部raw row与源episode/timestep映射、z-score/roundtrip、artifact hashes通过。
duplicate source与文件损坏拒绝；非法seed/缺dataset/已存在output/short的success-only空集合均CLI exit2。
short显式`--selection all` exit0，24条失败轨迹仍保留并标source_success_count0，仅用于诊断。
3.11语法解析通过，但没有在Python3.11实际执行。无GUI、训练或新机器人执行证据。

## Failure Cases

- 按row split的held-out误差不能证明对独立episode的泛化；按episode分也只是当前分布held-out，非OOD保证。
- 用全数据拟合normalizer泄漏；val/test分别拟合又会让训练与部署输入坐标不同。
- 使用terminal observation却无对应action会错位；去掉边界key会使后续sequence跨reset。
- σ=0直接除导致NaN；机械裁剪z-score可能丢失域外信号，且破坏可逆性。
- 忘记将policy输出按**action**统计反归一化，会把无量纲值当米命令。
- 不合格episode不等于每帧action错误；selection必须明确，评价success分母始终保留全attempt。
- test预处理检查可用于完整性审查，但不能用其分布/表现来挑seed、模型或normalization规则。

## Robotics Context / Interview Capsule

应用：示范学习pipeline、BC/ACT/Diffusion统一数据协议、恢复数据分组。
30秒：机器人示范先按episode拆分，统计只从train拟合；同一normalizer用于评价与部署，并保存动作反变换。
2分钟：解释时序相关、三个集合用途、单位不同的逐channel μ/σ、constant处理与row key；
用row leakage和held-out shift反例说明信息边界，最后指出预处理通过仍需learned closed-loop rollout。
Must Remember：split先于fit；test不能调参；normalize与denormalize必须共享train统计。

## My Verification — Run / Modify / Explain

状态唯一来源：[README](../README.md#stage-19--learning-from-demonstrations)。Learning三项留待本人完成。
Run：执行默认命令，核对split.json的16/4/4、无交集、train-only统计与train均值/标准差。
Modify：先预测仅将split seed193→194的影响，再执行：

```bash
python examples/20_learning_from_demonstrations/prepare_dataset.py --dataset tmp/s19_2/default --split-seed 194
```

比较成员、normalizer与val统计；源NPZ不变，分母不变；seed194仅学习对照，不替换冻结基准来美化评价。
Explain（五问）：
1. 为什么先episode split再拼接BC rows？随机row split泄漏了什么？
2. train/validation/test各自允许用于什么？test统计能否用于挑split seed？
3. μ/σ为什么只能用train？val/test normalized均值不为0是否错误？
4. 恒定通道如何处理？为什么必须保存action normalizer并反归一化输出？
5. 本课检查通过为何不能证明learned policy闭环成功或OOD泛化？

下一小任务S19.4 CPU tiny BC，等待明确请求；本轮停止。
