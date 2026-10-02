# UR5e 6D Pose

## Concept

S10.1 从已经完成的 UR5e site 世界位姿读取继续，只比较 position 与 orientation，
不发送控制命令。S10.2 再单独学习 orientation error；S10.3 扩展到 full Jacobian。

## Prediction

开始 S10.1 实验前由本人回答：

1. `site_xpos` 的 shape、单位和参考坐标系是什么？
2. `site_xmat` reshape 后的 shape 是什么？
3. 旋转矩阵的三列各自表示什么？
4. `site_xpos` 与 `site_xmat` 分别相对哪个坐标系描述 site？
5. 如果末端只改变朝向、不发生平移，site 原点的 position 是否一定改变？为什么？

## My TODO

- [ ] 回答 S10.1 五个问题，并给出每项的 shape / unit / frame。
- [ ] 运行前预测两个待比较姿态中 position 与三根 site 轴如何变化。

## Experiment

待预测完成后添加最小读取实验。复用
[`site_pose.py`](../examples/04_forward_kinematics/site_pose.py) 的 UR5e 加载、home reset、
`mj_forward` 和 `attachment_site` 查找方式，不在本步引入 IK、控制或 `mj_step`。

## Result


## Failure / uncertainty


## What I learned


## Interview questions


