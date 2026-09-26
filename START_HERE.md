# 👉 先看这里：工厂路线实验（2026-09-26 凌晨）

> English one-page summary of the whole team's approach and results (incl. these experiments): [analysis/final_approach.md](analysis/final_approach.md)

> 作者：Zhichao（zhou-zhichao），和 Claude 一起做的。所有结果都是 MuJoCo 仿真，用的是已有的 G1-DWAQ 爬楼梯策略（第三方权重，不是我们训练的），**除了最后的微调，没有训练任何新模型**。
> 代码在 `scripts/`，结果、视频、详细说明在 [`artifacts/factory-course/`](artifacts/factory-course/)。

## 一句话

之前我们已经能让 G1 在一段直楼梯上爬上去、再切换到平地策略走走廊。我接着往"真实工厂"推进了一步：**设计了带拐角、回头楼梯、下楼梯的复杂路线，假设工厂里有固定摄像头告诉机器人它在哪，让机器人从起点走到终点**；然后测了"摄像头要多准才够"和"能不能搬东西"。

## 思路（为什么这么做）

1. SKF 的题目要一条完整的流程：训练 / 测试 / 为什么成功 / 为什么失败 / 商业指标。单段直楼梯太简单，看不出真实工厂的问题。
2. 工厂有 CAD 平面图，楼梯尺寸是已知的。所以**机器人不一定要"看见"楼梯**，只要知道自己在地图上哪里就能规划路线。这就是"外部摄像头定位 + 已知地图 + 盲走的爬楼梯策略"这个**混合方案**。题目原话里也说混合方法往往比单一方法更现实。
3. 这样就能回答三个对 SKF 有用的问题：
   - 这个方案能走多复杂的路线？
   - 摄像头系统要多准才够？这个答案可以直接变成采购标准。
   - 机器人能不能边走边搬东西？如果不行，是哪里不行、需要什么训练？

## 做了什么、结果是什么

| # | 实验 | 核心结果 | 说明文档 |
|---|---|---|---|
| 1 | **复杂工厂路线**：拐角、180° 回头楼梯、1.8 m 夹层、12 级下楼梯，全程 19.9 m、5 个弯 | 完美定位下，完整路线 25 个起点中 **22 个走到终点（88%）**；失败的 3 次全在下楼梯 | [factory-course/README.md](artifacts/factory-course/README.md) |
| 2 | **摄像头误差**：给位置加噪声、偏差、低刷新率、延迟，共 1025 次仿真 | 位置误差 ≤10 cm、偏差 ≤20 cm、≥2 Hz、延迟 ≤300 ms 时，和完美摄像头没有明显差别；**加上机器人自己的里程计融合后，1 Hz、1 秒延迟也能走完** | [localization-sweep/README.md](artifacts/factory-course/localization-sweep/README.md) |
| 3 | **搬箱子（不重新训练）**：手端 / 贴前胸 / 背在后背，0–15 kg | 手端箱子时策略几乎走不动（手臂被锁住，它没练过）；**贴在前胸、手臂自由时，上楼梯能带约 8 kg**；背在后背时 2 kg 就爬不上楼梯 | [payload-sweep/README.md](artifacts/factory-course/payload-sweep/README.md) |
| 4 | **微调（正在跑）**：在现有模型上加"胸前负重 0–12 kg + 重心前移"训练；另一版再多练下楼梯 | 在 Minerva 集群的 L40S 上训练中，预计上午出结果，然后用同样的场景复测 | 本文件底部 |

**只看视频的话**：先看 [L4 完整路线成功](artifacts/factory-course/runs/l4_factory_route_v30_rec_y+0cm_yaw+0_ideal.mp4)，再看摄像头误差的两组对比（[1 Hz 纯摄像头失败](artifacts/factory-course/localization-sweep/videos/l4_factory_route_v30_rec_align_y-5cm_yaw-5_s0cm_y0deg_1hz_0ms_b0cm_seed6.mp4) vs [加里程计融合成功](artifacts/factory-course/localization-sweep/videos/l4_factory_route_v30_rec_align_y-5cm_yaw-5_s0cm_y0deg_1hz_0ms_b0cm_fused_seed6.mp4)）和 [payload-sweep/videos/](artifacts/factory-course/payload-sweep/videos/)。

## 过程中发现的几件事（适合放进演讲的"为什么"）

- **这个策略右转比左转弱约 4 倍**：原地转弯下令 0.6 rad/s 时，左转实际约 0.25 rad/s，右转只有约 0.06 rad/s。改成满速 1.0 rad/s 转弯后，完整路线从约 120 s 缩短到约 80 s。
- **太小的指令会被忽略**：前进低于约 0.2 m/s、转向低于约 0.4 rad/s 时只会原地踏步。
- **最大的弱点是下楼梯**：下到第 3–5 级时会突然扭身（最多约 80°），然后走出楼梯。就算位置完全准确也会发生，这是"盲走"策略本身的限制，**训练精力应该花在这里**。
- **瓶颈不在摄像头，在策略**：普通水平的摄像头（5 cm / 10 Hz / 200 ms）就够了，再提升摄像头也解决不了下楼梯的问题。
- **手臂锁死比重量更致命**：同样的箱子，端在手里（手臂锁住）2 kg 就不行；绑在胸前、手臂自由，8 kg 还能上楼梯。因为这个策略训练时练过躯干 ±5 kg 的重量，但手臂一直是自由的。
- **重心位置很关键**：背包式（重心往后）爬不上楼梯；胸前（重心往前）反而有利于上楼。
- 为了不让策略被锁住的手臂搞乱，我们给策略看"虚拟的自由手臂"（实际手臂是锁着的）。这个技巧让手端箱子能走了，相当于把上半身和下半身的控制分开，思路和 FALCON 论文里的双智能体类似。

## 演讲可以讲的故事线（建议）

1. **问题**：工厂里的人形机器人需要爬楼梯、拐弯、搬东西。训练成本高，怎么用最少的训练做到？
2. **方案**：混合方案 = 已知工厂地图 + 固定摄像头定位（+ 机器人自身里程计）+ 现成的爬楼梯策略 + 我们写的路线跟随和监督逻辑（卡住后退重试、上楼梯前先对正）。
3. **证据**：完整路线 88% 成功；给出摄像头规格；给出负重上限；每一项都有视频。
4. **为什么失败**：下楼梯扭身、手臂锁死、重心位置。这些都指向**需要什么样的训练**。
5. **下一步训练流程**：用胸前负重 + 重心随机化 + 更多下楼梯地形去微调，然后用同一套仿真测试验证。这正是 SKF 要的"训练 → 测试 → 成功标准"的流程，而且测试套件已经做好了。

## 怎么复现 / 继续做

- 环境：Chalmers Minerva 集群 `/data/users/zhichaoz/skf/`。
  - `env/`：MuJoCo 仿真环境。
  - `isaac/`：Isaac Sim 5.1 + Isaac Lab 2.3 训练环境。
  - `repo/`：本仓库在集群上的副本。
  - `run_batch.sh`：用 Slurm 批量跑仿真。
- 本地也能跑单次仿真，需要先 clone [G1DWAQ_Lab](https://github.com/liuyufei-nubot/G1DWAQ_Lab)，然后：
  ```bash
  python scripts/run_factory_course.py --course l4_factory_route --recovery --align --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
  ```
- 每次运行都会记录代码和模型的哈希值，结果可以逐字节复现（脚本里已固定 PyTorch 单线程）。

## 微调状态（持续更新）

- 在 Minerva 上用 Isaac Lab 从 `model_9999.pt` 继续训练 2000 轮，两个版本在 neptune 节点上各占一块 L40S 同时跑：
  - `ft_payload`：躯干 +0–12 kg，重心向前 0–12 cm。
  - `ft_payload_descent`：同上，加上两倍比例的下楼梯地形。
- 训练完后导出到 MuJoCo，用同样的 L1 / L2 / L4 和胸前负重测试对比。结果会更新在这里。
