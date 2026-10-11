# Stage 20 — ACT / Temporal Policy

当前仅实现[S20.1 sequence dataset / boundaries / mask](../../docs/23_1_sequence_dataset.md)。
状态唯一来源：[根README](../../README.md#stage-20--act--temporal-policy)。
依赖requirements已有NumPy；无Torch/MuJoCo/local helper导入，无新依赖。

```bash
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
python examples/21_act_temporal_policy/sequence_dataset.py --prepared tmp/s19_3/verified193
python examples/21_act_temporal_policy/sequence_dataset.py --prepared tmp/s19_3/verified193 --chunk 20
```

prepared为S19.3目录；缺失时按[Stage19](../20_learning_from_demonstrations/README.md) S19.2→3重建并替换路径。
默认L4/H10，每个动作时刻一个anchor；history左padding/action右padding，normalized zero，valid=True表示真实token。
仅train/validation；冻结split与normalizer，不读取test数组、不训练、不推进物理。
输出新ignored tmp/s20_1/run_*：train/validation NPZ与results.json；--output必须新目录。
2026-10-11本机两组exit0：train1600/validation400窗口；H10 train有效15280/padding720，H20有效28960/padding3040。
短episode跨界与masked loss反例通过；Run/Modify/Explain见学习包。S20.2及后续尚未实现。
