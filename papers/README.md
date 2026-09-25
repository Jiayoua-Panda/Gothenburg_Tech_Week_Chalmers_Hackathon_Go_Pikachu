# 楼梯感知与双足机器人爬楼论文

这里整理了与 StairLab 概念直接相关、并报告仿真或真机结果的研究。项目当前的网页是交互式几何展示；下列论文结果属于各论文作者报告的实验，不是本项目已经复现的结果。

## 已下载的论文

| 论文 | 方法与已报告结果 | 与项目的关系 |
| --- | --- | --- |
| [FastStair: Learning to Run Up Stairs with Humanoid Robots](faststair-learning-to-run-up-stairs.pdf) — Yan Liu 等，*IEEE Robotics and Automation Letters*, 2026, 11(7):8841–8847. [arXiv 版本](https://arxiv.org/abs/2601.10365) · [DOI](https://doi.org/10.1109/LRA.2026.3701575) | 用落脚点规划器引导强化学习，再训练速度专家并融合。作者报告 LimX Oli 在楼梯上以最高 1.65 m/s 稳定上行，并在 12 秒内跑完每级高 17 cm 的 33 级螺旋楼梯。 | 很好的“已经做到多快、多长楼梯”的结果参照，也展示规划器如何给 RL 提供可行落脚先验。它侧重楼梯规划与控制，不是本文所说的 3D 楼梯感知方案。 |
| [Learning Vision-Based Bipedal Locomotion for Challenging Terrain](learning-vision-based-bipedal-locomotion.pdf) — Helei Duan 等，*ICRA 2024*, pp. 56–62. [论文页](https://arxiv.org/abs/2309.14594) · [DOI](https://doi.org/10.1109/ICRA57147.2024.10611621) | 先在仿真中用局部高度图训练运动策略，再用深度图像历史与本体状态训练高度图预测器。作者在 Cassie 上报告了跨越楼梯、随机高台和约 0.5 m 台阶的 sim-to-real 实验，最高速度约 1 m/s；无需显式位姿估计或真机微调。 | 与“深度相机 → 局部 3D/高度图 → RL 控制器”的项目设想最接近，可作为感知输入与 sim-to-real 训练流程的起点。 |

两份 PDF 均为 arXiv 版本，直接从 arXiv 官方论文页下载；两个页面都标注为 [Creative Commons Attribution 4.0 (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)。保留作者、标题、出处和原文链接，方便归属与核查。

## 相关成果（官方页面链接）

以下论文也报告了楼梯或复杂地形上的仿真/真机结果。我没有找到明确允许仓库重新分发这些论文 PDF 的许可条款，因此这里保留正式论文或预印本链接。

- [Learning Humanoid Locomotion with Perceptive Internal Model (PIM)](https://arxiv.org/abs/2411.14386) — Junfeng Long 等，*ICRA 2025*, pp. 9997–10003. 以机器人周围持续更新的 elevation map 作为感知输入；作者报告了多种人形机器人、室内外地形及楼梯实验，并称策略可在 RTX 4090 上约 3 小时训练。 [DOI](https://doi.org/10.1109/ICRA55743.2025.11128333)
- [Explicit Stair Geometry Conditioning for Robust Humanoid Locomotion](https://arxiv.org/abs/2605.09944) — 2026 arXiv 预印本。将踏步高度、深度和相对朝向等显式几何量输入 PPO；作者报告了对未见楼梯高度的仿真泛化，以及 Unitree G1 连续爬上 33 级室外楼梯的实验。适合对照“显式几何参数”与“高度图端到端编码”。
- [Blind Bipedal Stair Traversal via Sim-to-Real Reinforcement Learning](https://roboticsproceedings.org/rss17/p061.html) — Jonah Siekmann 等，*Robotics: Science and Systems (RSS) 2021*. 在 Cassie 上仅用本体感知，通过在仿真训练中随机化楼梯类地形，实现真实楼梯和台阶地形的 sim-to-real 行走；这是验证视觉感知是否带来增益时很有价值的无视觉基线。
- [Vision-Aided Reinforcement Learning for Humanoid Stair Climbing](https://doi.org/10.1109/ICOECAI67333.2025.11335927) — 2025 年论文，结合视觉与本体感知训练楼梯策略；论文介绍页报告了从仿真迁移到 Unitree G1 真机的楼梯实验。可与 PIM 和显式几何输入方案一起比较。

## 对 StairLab 实验设计的启发

可以先复现平地和本体感知楼梯基线，再比较：

1. 本体感知输入；
2. 深度图训练的局部高度图预测器；
3. 显式踏步高度/深度/宽度参数；
4. elevation map 或高度图输入策略。

在训练范围内和未见过的楼梯几何上分别统计成功率、完成时间、跌倒次数、脚步落点误差和感知噪声下的稳定性。这样网页可以展示真实实验数据与轨迹回放，并清楚区分论文结果和项目自己的测量。
