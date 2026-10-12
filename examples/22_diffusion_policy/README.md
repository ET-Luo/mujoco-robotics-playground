# Stage 21 — Diffusion Policy

当前实现[S21.1双路径示范/回归均值反例/BC与ACT重训](../../docs/24_1_multimodal_paths.md)。
[根README](../../README.md#stage-21--diffusion-policy)是Engineering / Learning状态唯一来源。
S21.2已实现forward noising，S21.3已训练单步条件denoiser；S21.4已实现H10反向采样，S21.5已实现生成策略闭环，S21.6及以后仍规划。

依赖requirements已有NumPy/CPU Torch/MuJoCo/Matplotlib；复用Stage20 TinyACT、BC、window、latent helpers和旧XY fixture，无新安装。
使用WSL2 Ubuntu24.04，在mujoco环境核验后运行：

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/multimodal_paths.py
```

无需旧tmp输入，现场生成16train/4validation示范；同reset上下两路保持同组，另保留2个test reset，不生成/评估test。
新任务`xy_two_route_disk_v1`，原XY动力学、world absolute reference、50Hz/1kHz；圆盘为几何禁入区，无接触反力。
真实1kHz运动segment检查并锁存碰撞，最终到达不能洗掉失败；老A轨道checkpoint不可直接用于新任务。
train-only normalizer；BC100epoch/seed211与tiny ACT60epoch/seed212重训，validation offline选模，不用rollout选模。
新ignored tmp/s21_1/run_*含dataset/protocol/normalizer/train/validation NPZ、示范与评估轨迹、checkpoint/loss、results/trials.csv/paths.png。

Modify：将下方目录换成第一命令打印的实际输出路径：

```bash
python examples/22_diffusion_policy/multimodal_paths.py \
  --models-from tmp/s21_1/verified --mean-upper-weight .8
```

只改变解析mean control的模式概率；训练数据仍平衡、checkpoint不重训。不是给BC/ACT添加route标签。
2026-10-12 expert上下各2/2、uniform mean碰撞2/2；BC/ACT E1/E5成功0/2、1/2、2/2，p=.8 mean安全2/2且净空5.599mm。
两组204000Euler步/独立几何/scorer/actual预测/paired split/normalizer与frozen Modify通过；无test/GUI/接触反力或Diffusion验证。
实测结果与五问见学习包；Engineering Complete，Learning Run/Modify/Explain全部完成，Mastered（2026-10-12 本人确认实验、预测与五项Explain）。

## S21.2 — Forward noising

[学习包](../../docs/24_2_noising_basics.md)；仅依赖已有NumPy/Matplotlib，无local helper。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/noising_basics.py --output tmp/s21_2/verified
python examples/22_diffusion_policy/noising_basics.py --beta-end .02 --output tmp/s21_2/beta002
```

输出arrays.npz/results.json/noising.png。2026-10-12 Zero两组exit0；oracle/偏差解析式与50000独立Markov样本边缘矩通过。Engineering Complete；2026-10-12 本人确认Run/Modify完成，五项Explain通过，Learning Mastered；无MuJoCo/GUI/模型训练。

## S21.3 — Conditional denoiser

[学习包](../../docs/24_3_conditional_denoiser.md)；依赖已有NumPy/CPU Torch/MuJoCo/Matplotlib，local helpers为dual_path_task→episode_contract→cartesian_spring以及noising_basics，无新依赖。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/conditional_denoiser.py
python examples/22_diffusion_policy/conditional_denoiser.py --conditioning none
```

两次独立新输出目录；现场生成同paired train/validation示范，不读test，不重训BC/ACT。
H1/4930参数/K100/100epoch/CPU1thread，固定validation noise选模、新noise audit；checkpoint保存日程与normalizer。
2026-10-12 Zero state/none audit epsilon MSE .039117/.307293，恢复reference RMSE5.109/17.436mm；两组exit0与独立指标检查通过。
输出protocol/data/normalizer/checkpoint/loss/audit/results/PNG仅ignored tmp/s21_3；Engineering Complete + Learning Mastered（2026-10-12 本人确认Run/Modify与五项Explain）。
这不是反向采样或生成策略执行；单seed/两validation reset不证明安全/模式覆盖或优于ACT。

## S21.4 — Sequence diffusion / sampling

[学习包](../../docs/24_4_sequence_diffusion.md)；复用conditional_denoiser.collect与sequence_dataset.make_windows，依赖已有NumPy/CPU Torch/MuJoCo/Matplotlib。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/sequence_diffusion.py
# checkpoint路径换成上一命令打印的实际目录
python examples/22_diffusion_policy/sequence_diffusion.py \
  --checkpoint tmp/s21_4/final/chunk.pt --sample-seed 215
```

H10/23956参数、masked loss/input/output、DDPM100相邻步骤/k1无噪声、反归一化。
2026-10-12 Zero两组exit0；audit MSE .089860，seed214/215 sample对配对expert RMSE3.685/3.713mm。
窗口/100步独立重算/解析posterior/mask/reload与冻结Modify通过，样本仍抖动；无闭环/碰撞/模式覆盖证明。
产物仅ignored tmp/s21_4，Engineering Complete、Learning Run/Modify/Explain全部完成、Mastered（2026-10-12 本人确认）；下一步S21.5等待请求。

## S21.5 — Closed-loop Diffusion Policy

[学习包](../../docs/24_5_diffusion_rollout.md)；依赖已有NumPy/CPU Torch/MuJoCo/Matplotlib与S21.4/task helpers，无新安装。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/22_diffusion_policy/diffusion_rollout.py
# 换成Run打印的实际training/chunk.pt路径，冻结模型
python examples/22_diffusion_policy/diffusion_rollout.py \
  --checkpoint tmp/s21_5/default/training/chunk.pt --execute 1
```

默认自动准备相同S21.4包；可显式--checkpoint跳过准备。H10/K100、E5默认与E1 Modify、actual反馈和预算mask。
2026-10-12 Zero两组exit0，Diffusion均0/2、有发散；expert上下各2/2。
全actual Euler/geometry/scorer、360次actual重采样与执行索引、checkpoint/frozen Modify通过；不通过重训/裁剪隐藏失败。
产物仅ignored tmp/s21_5；Engineering Complete + Learning Mastered（2026-10-12 本人确认实验、预测与五项Explain）。无test/GUI/硬件实时或可靠策略证明。
