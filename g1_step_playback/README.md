# G1 大步 / 小步回放（方案①）

用 Unitree 官方预训练的 G1-29dof 行走策略，在 MuJoCo 里回放不同速度指令，展示步长变化。**不需要训练，不需要 Isaac Sim，Windows + CPU 即可运行。**

## 运行

```bash
py -m pip install mujoco onnxruntime pyyaml opencv-python "imageio[ffmpeg]" matplotlib
py play_g1.py                    # 0.3 / 0.6 / 0.9 m/s 三段，录视频 + 统计
py play_g1.py --speeds 0.2 1.0   # 自定义速度（策略训练范围 -0.5 ~ 1.0 m/s）
py play_g1.py --viewer           # 交互窗口：↑/↓ 速度，←/→ 转向，空格停，R 复位
py play_g1.py --no-video         # 只算指标（几秒钟）
```

## 输出（`outputs/`）

| 文件 | 内容 |
|---|---|
| `g1_vx_*.mp4` | 每个速度一段侧视视频，地面标出落脚点（左蓝右红） |
| `comparison_side_by_side.mp4` | 三段并排对比，适合放进 PPT |
| `step_length_vs_speed.png` | 步长 vs 速度曲线 |
| `summary.csv` | 每段的实际速度、平均步长、步长标准差、步频、是否摔倒 |
| `step_events.csv` | 每一步的落地时间、左右脚、步长、落点坐标 |

## 结果（本机实测）

| 速度指令 | 实际速度 | 平均步长 | 步频 |
|---|---|---|---|
| 0.30 m/s | 0.30 m/s | 0.084 ± 0.010 m | 3.6 步/s |
| 0.60 m/s | 0.56 m/s | 0.185 ± 0.029 m | 3.0 步/s |
| 0.90 m/s | 0.75 m/s | 0.251 ± 0.023 m | 3.0 步/s |

三段都没有摔倒。步长 × 步频 ≈ 实际速度，数据自洽。

## 实现要点

- 观测、动作完全按 `policy/deploy.yaml` 复现，与 unitree_rl_lab 的 C++ 真机部署代码逻辑一致：
  - 6 项观测（角速度×0.2、重力投影、速度指令、关节角偏差、关节速度×0.05、上一步动作），每项保留 5 帧历史，按"每项旧→新"拼接，共 480 维；
  - 动作 = 输出 × 0.25 + 默认关节角，按 `joint_ids_map` 从策略顺序映射到 SDK/MuJoCo 顺序；
  - PD 力矩 200 Hz（dt = 5 ms），策略 50 Hz，和 Isaac Lab 训练设置一致。
- 步长定义：一只脚落地时，它与另一只（支撑）脚沿机身前进方向的距离；腾空 > 0.1 s 才计为一步。
- 前 4 s（2 s 站立 + 1 s 加速 + 1 s 过渡）不计入统计。

## 局限（写进报告）

1. **步长和速度耦合**：策略没有步长或步频指令，想要"同速度下大步 / 小步"需要方案②（加步态周期指令后微调）。
2. **高速饱和**：指令 0.9 m/s 时实际只有 0.75 m/s，步长上限约 0.25 m。
3. **低速步频偏高**：0.3 m/s 时步频升到 3.6 步/s，说明低速时策略同时缩短了步长和步时。
4. 这是 sim2sim（策略在 Isaac Lab 训练，这里在 MuJoCo 回放），没有在真机上验证。

## 来源

- `policy/`：[unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab)（Apache-2.0）`deploy/robots/g1_29dof/config/policy/velocity/v0`
- `models/g1/`：[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)（见 `models/LICENSE_unitree_mujoco`）
