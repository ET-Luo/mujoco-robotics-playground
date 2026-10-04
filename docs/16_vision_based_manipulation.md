# Stage 13 — Vision-Based Manipulation

规划骨架，未实现、未实验。任务/状态见[根 README](../README.md#stage-13--vision-based-manipulation)。
前置：Stage 12 geometry 与 PnP；ICP/hand-eye 各自独立，不要求首个抓取同时用到全部方法。

Problem / Why：用图像估计替代 planner 的 known pose 输入。
Intuition / Core Concepts：truth 只作评价；estimate、extrinsic、grasp offset 各自明确 frame。
Mathematics：T_BO=T_BC T_CO；T_BG=T_BO T_OG；T_WG=T_WB T_BG。
Math-to-Code：接入现有 W-frame IK、cubic reference、夹爪与接触判据，保留 mutation 语义。
Minimal Experiment / Expected Result：CPU 合成图像、已知尺寸目标，先 pose-only 核验，再从 home 执行完整流程。
Actual Result / Explanation：待当前 Task 授权后记录，不预填成功。
Failure Cases：遮挡、对称歧义、PnP 失败、时间错位、噪声、IK/执行失败、truth 泄漏。
Robotics Applications：固定相机 tabletop manipulation。
Interview Capsule：后续按 Task 整理 30 秒/2 分钟材料。
Must Remember：perception 成功、IK 成功、grasp 成功分开统计。
My Verification：待本人 Run / Modify / Explain；不预填本人结论。
