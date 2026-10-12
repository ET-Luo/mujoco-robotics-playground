# Stage 21 — Diffusion Policy

当前实现[S21.1双路径示范/回归均值反例/BC与ACT重训](../../docs/24_1_multimodal_paths.md)。
[根README](../../README.md#stage-21--diffusion-policy)是Engineering / Learning状态唯一来源。
S21.2已实现forward noising；S21.3及以后仍规划，尚无训练denoiser/反向采样。

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

输出arrays.npz/results.json/noising.png。2026-10-12 Zero两组exit0；oracle/偏差解析式与50000独立Markov样本边缘矩通过。Engineering Complete，Learning待本人Run/Modify/Explain；无MuJoCo/GUI/模型训练。
