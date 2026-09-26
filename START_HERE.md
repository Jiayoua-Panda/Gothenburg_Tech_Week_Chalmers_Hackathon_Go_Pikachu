# 👉 先看这里：工厂路线实验（2026-09-26）

> English one-page summary of the whole team's approach and results (incl. these experiments): [analysis/final_approach.md](analysis/final_approach.md)

> 作者：Zhichao（zhou-zhichao）；前期实验与 Claude 协作，微调复测由 Codex 接续完成。以下成功次数均来自 MuJoCo 仿真；微调训练在 Isaac Lab 完成。前三项使用第三方 G1-DWAQ 权重，没有重新训练；第四项从该权重继续训练（奖励函数未改，配置见 [`training/g1-dwaq-finetune/`](training/g1-dwaq-finetune/)）。
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
| 4 | **微调并复测**：负重、额外下楼梯等设置，共 6 次训练 | L4 胸前 8 kg：负重版两个种子为 **14/25、20/25**（原模型 6/25）；但无负重仅 **0/25、3/25**（原模型 22/25），不能直接当通用策略 | [本文件底部](#微调结果2026-09-26)、[逐次结果](artifacts/factory-course/finetune-results.csv) |

**只看视频的话**：先看 [L4 完整路线成功](artifacts/factory-course/runs/l4_factory_route_v30_rec_y+0cm_yaw+0_ideal.mp4)，再看摄像头误差的两组对比（[1 Hz 纯摄像头失败](artifacts/factory-course/localization-sweep/videos/l4_factory_route_v30_rec_align_y-5cm_yaw-5_s0cm_y0deg_1hz_0ms_b0cm_seed6.mp4) vs [加里程计融合成功](artifacts/factory-course/localization-sweep/videos/l4_factory_route_v30_rec_align_y-5cm_yaw-5_s0cm_y0deg_1hz_0ms_b0cm_fused_seed6.mp4)）和 [payload-sweep/videos/](artifacts/factory-course/payload-sweep/videos/)。

## 过程中发现的几件事（适合放进演讲的"为什么"）

- **这个策略右转比左转弱约 4 倍**：原地转弯下令 0.6 rad/s 时，左转实际约 0.25 rad/s，右转只有约 0.06 rad/s。改成满速 1.0 rad/s 转弯后，完整路线从约 120 s 缩短到约 80 s。
- **太小的指令会被忽略**：前进低于约 0.2 m/s、转向低于约 0.4 rad/s 时只会原地踏步。
- **最大的弱点是下楼梯**：下到第 3–5 级时会突然扭身（最多约 80°），然后走出楼梯。就算位置完全准确也会发生，这是"盲走"策略本身的限制，**训练精力应该花在这里**。
- **瓶颈不在摄像头，在策略**：普通水平的摄像头（5 cm / 10 Hz / 200 ms）就够了，再提升摄像头也解决不了下楼梯的问题。
- **手臂锁死比重量更致命**：同样的箱子，端在手里（手臂锁住）2 kg 就不行；绑在胸前、手臂自由，8 kg 还能上楼梯。因为这个策略训练时练过躯干 ±5 kg 的重量，但手臂一直是自由的。
- **重心位置很关键**：背包式（重心往后）爬不上楼梯；胸前（重心往前）反而有利于上楼。
- **微调有明显取舍**：重载完整路线可以改善，但同一模型在无负重或轻载下常卡在上楼梯；即使训练奖励变好，也不能代替完整路线的复测。
- 为了不让策略被锁住的手臂搞乱，我们给策略看"虚拟的自由手臂"（实际手臂是锁着的）。这个技巧让手端箱子能走了，相当于把上半身和下半身的控制分开，思路和 FALCON 论文里的双智能体类似。

## 演讲可以讲的故事线（建议）

1. **问题**：工厂里的人形机器人需要爬楼梯、拐弯、搬东西。训练成本高，怎么用最少的训练做到？
2. **方案**：混合方案 = 已知工厂地图 + 固定摄像头定位（+ 机器人自身里程计）+ 现成的爬楼梯策略 + 我们写的路线跟随和监督逻辑（卡住后退重试、上楼梯前先对正）。
3. **证据**：完整路线 88% 成功；给出摄像头规格；给出负重上限；每一项都有视频。
4. **为什么失败**：下楼梯扭身、手臂锁死、重心位置。这些都指向**需要什么样的训练**。
5. **训练 → 测试 → 失败分析**：已做负重、重心和下楼梯地形微调，再用原测试套件复测。重载有改善，轻载和无负重严重退化；下一轮应把各负重档位的完整路线成功次数都列入验收标准。

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

## 微调结果（2026-09-26）

在 Minerva 上从第三方 `model_9999.pt` 继续训练，每次新增 2000 轮，最终检查点均为 `model_11998.pt`。`ft_payload` 给躯干增加 0–12 kg 并把重心向前随机移动 0–12 cm；`ft_payload_descent` 再提高下楼梯地形比例。两者各跑种子 42 和 7。另外，`ft_descent` 只增加下楼梯地形，`ft_payload_heavy` 把负重扩到 0–20 kg、前移扩到 0–15 cm，各跑种子 42。六次训练均写出了最终检查点；训练进程曾卡在 Isaac Sim 的退出阶段，在确认检查点和复测结果后已取消作业、释放 GPU。

同一套路线、控制器和完美定位，均开启卡住后退重试与上楼梯前对正。L4 使用相同的 25 个起点（横向 −10 到 +10 cm、朝向 −5° 到 +5° 的 5×5 网格）；表中为走到终点的次数，**不是实物成功率，也不是独立随机试验的统计估计**。箱子绑在胸前，手臂仍由策略控制。

| 策略（训练种子） | 无负重 | 胸前 2 kg | 胸前 5 kg | 胸前 8 kg | 胸前 12 kg |
|---|---:|---:|---:|---:|---:|
| 原模型 | 22/25 | 15/25 | 10/25 | 6/25 | 1/25 |
| `ft_payload`（42） | 0/25 | 7/25 | 13/25 | 14/25 | 15/25 |
| `ft_payload`（7） | 3/25 | 8/25 | 15/25 | **20/25** | **18/25** |
| `ft_payload_descent`（42） | 0/25 | 0/25 | 0/25 | 3/25 | 11/25 |
| `ft_payload_descent`（7） | 0/25 | 0/25 | 1/25 | 11/25 | 18/25 |
| `ft_descent`（42） | 0/25 | 22/25 | 21/25 | 13/25 | 0/25 |
| `ft_payload_heavy`（42） | 0/25 | 0/25 | 0/25 | 0/25 | 0/25 |

**结论**：负重微调确实提高了部分重载完整路线的表现，但明显牺牲了无负重和轻载表现。`ft_payload` 两个种子在 8 kg 下相差 6/25；加入更多下楼梯地形也没有稳定修复问题。原模型的直接下楼梯测试为 7/9，`ft_payload` 两个种子为 9/9，而两个 `ft_payload_descent` 种子仍为 7/9。L2 上楼梯胸前 8 kg 时，原模型 8/9，`ft_payload` 两个种子为 4/9、7/9；因此不能只挑 L4 重载的最好数字宣布模型已可用。

[1253 次逐次结果 CSV](artifacts/factory-course/finetune-results.csv) 包含原模型与六个微调检查点：每个版本共 179 次配对复测（L1 胸前 12 kg：9 次；L2 胸前 2/5/8/12 kg：各 9 次；直接下楼梯无负重：9 次；L4 无负重与胸前 2/5/8/12 kg：各 25 次），记录结果、失败位置、模型 SHA-256 和代码哈希。检查点保存在 Minerva 的 `/data/users/zhichaoz/skf/isaac/G1DWAQ_Lab/TienKung-Lab/logs/g1_dwaq/`；用 `scripts/run_factory_course.py --checkpoint /path/to/model_11998.pt` 可复测。下一轮训练需要同时保住无负重、轻载、重载和下楼梯表现，再谈通用工厂路线策略。
