# Stage 14 — Motion Planning

规划骨架，未实现、未实验。任务/状态见[根 README](../README.md#stage-14--motion-planning)。
前置：Stage 13 pose/grasp 接口；使用既有 IK，不重教 IK。

Problem / Why：合法起终点之间的 cubic 路径可能撞障碍物。
Intuition / Core Concepts：在 C-space 找可行 path，随后赋时间；path 与 trajectory 分开。
Mathematics：q∈R⁶（rad），joint bounds、distance metric、edge interpolation 与 collision predicate。
Math-to-Code：独立 MjData 作几何查询，手写 CPU RRT/RRT-Connect；held object 随工具位姿更新。
Minimal Experiment / Expected Result：先二维障碍与 narrow passage，再 UR5e obstacle scene；fixed seed + budget。
Actual Result / Explanation：待具体 Task 实测，记录成功率、规划耗时、path length、collision resolution。
Failure Cases：漏检边、失败预算、周期关节距离、错误接触白名单、持物遗漏、平滑后碰撞、时间化限速。
Robotics Applications：障碍环境接近、带物转移；允许 finger/object 抓取接触不等于允许 arm/object 碰撞。
Interview Capsule：后续按 Task 整理 30 秒/2 分钟材料。
Must Remember：sample-free 不等于 continuous collision-free；budget failure 不证明无解。
My Verification：待本人 Run / Modify / Explain。
