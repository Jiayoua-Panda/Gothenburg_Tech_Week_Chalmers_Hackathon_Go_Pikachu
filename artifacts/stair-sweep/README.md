# G1 楼梯几何对照实验（2026-09-26）

本目录记录固定预训练策略在 MuJoCo 中的 **42 次确定性仿真测试**。14 组几何条件，每组从相对中心线 `−10 / 0 / +10 cm` 三个横向起点运行一次。完整逐次结果见 [`matrix.csv`](matrix.csv) 和 [`matrix.json`](matrix.json)；每次的同名 JSON、`.csv.gz` 分别保存条件/终止原因和无损压缩的 `t,x,y,z` 轨迹。`scenes/` 保存每组几何的 MuJoCo XML。代表性 MP4 见下文。

## 策略与控制

- 机器人和策略：第三方 [G1DWAQ_Lab](https://github.com/liuyufei-nubot/G1DWAQ_Lab) 的 `model_9999.pt`，源代码提交 `bebb0ea`，checkpoint SHA-256 见每次 JSON。该权重**不是本项目训练的**。
- 固定前进速度命令 `0.3 m/s`；策略以 50 Hz 更新，MuJoCo 物理以 200 Hz 更新。除专门标明 `no_center` 的消融录像外，使用完全相同的中心线控制器，根据 **MuJoCo 真实位置和朝向**调整横移与转向命令。策略没有读相机、深度图或人工标注的台阶几何。
- 墙体只在 MP4 中显示为半透明，以免挡住机器人；碰撞与动力学不变。
- 从 15 cm 高、31 cm 踏面深、160 cm 可通行宽的既有场景改为统一参数生成。每组均为 10 级上行台阶、两级踏面长度的顶部平台、上层走廊。`160 cm` 是旧实验基准，超出了网页滑块的 `70–140 cm` 区间；其余值都在网页可调范围内。

## 结果判定

到达上层走廊：机身根节点 `x > 顶部平台末端 + 3.13 m`，且 `z > 台阶总高 + 0.4 m`。根节点 `z < 0.32 m` 记为跌倒；位于台阶或平台区间时，横向偏离超过 `可通行半宽 + 0.2 m` 记为越界。**60 秒仿真时间**内未到达则记为 `time_limit`，并不自动等于跌倒。阈值与完整运行条件可见 [`run_stair_sweep.py`](../../scripts/run_stair_sweep.py)。

表格中的 `到顶/3` 仅指三个固定起点中的到顶次数，**不是未知环境下的成功率估计**。三个参数按一次只改一个的方式相对基准变化，其他条件固定。

| 台阶高 | 踏面深 | 可通行宽 | 到顶/3 | 其他结果 |
| ---: | ---: | ---: | ---: | --- |
| 8 cm | 31 cm | 160 cm | 3/3 | — |
| 10 cm | 31 cm | 160 cm | 3/3 | — |
| 12 cm | 31 cm | 160 cm | 2/3 | 1 次限时未到顶 |
| **15 cm** | **31 cm** | **160 cm** | **3/3** | 基准 |
| 18 cm | 31 cm | 160 cm | 2/3 | 1 次越界 |
| 20 cm | 31 cm | 160 cm | 0/3 | 3 次限时未到顶 |
| 15 cm | 20 cm | 160 cm | 1/3 | 2 次限时未到顶 |
| 15 cm | 25 cm | 160 cm | 0/3 | 1 次跌倒、1 次越界、1 次限时未到顶 |
| 15 cm | 35 cm | 160 cm | 3/3 | — |
| 15 cm | 40 cm | 160 cm | 2/3 | 1 次限时未到顶 |
| 15 cm | 31 cm | 70 cm | 3/3 | — |
| 15 cm | 31 cm | 90 cm | 3/3 | — |
| 15 cm | 31 cm | 110 cm | 3/3 | — |
| 15 cm | 31 cm | 140 cm | 3/3 | — |

总计 42 次：31 次到顶、1 次跌倒、2 次越界、8 次限时未到顶。结果**不是单调的高度或宽度阈值**：比如 12 cm 和 18 cm 的一个横向起点失败，而 15 cm 基准三个起点都到顶。各组只有三个起点，不能据此推断稳健性；较窄台阶在本场景成功，也不能推断真机在窄楼梯上安全。

## 录像索引

录像画面**没有文字叠加**。英文文件名依次标出结果、台阶高、踏面深、可通行宽、起点横向偏移和纠偏方式；`sim-truth-centering` 指仿真真值纠偏，`no-centering` 指不纠偏。`highlights.json` 也列出了选出的片段及终止原因。

| 类型 | 场景与起点 | 文件 |
| --- | --- | --- |
| 成功 | 15 / 31 / 160 cm，中心起点，基准 | [`success_rise15cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4`](success_rise15cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4) |
| 成功 | 18 / 31 / 160 cm，中心起点 | [`success_rise18cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4`](success_rise18cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4) |
| 成功 | 15 / 31 / 70 cm，中心起点 | [`success_rise15cm_tread31cm_width70cm_offset+0cm_sim-truth-centering.mp4`](success_rise15cm_tread31cm_width70cm_offset+0cm_sim-truth-centering.mp4) |
| 成功 | 15 / 40 / 160 cm，中心起点 | [`success_rise15cm_tread40cm_width160cm_offset+0cm_sim-truth-centering.mp4`](success_rise15cm_tread40cm_width160cm_offset+0cm_sim-truth-centering.mp4) |
| 限时未到顶 | 20 / 31 / 160 cm，中心起点 | [`time-limit_rise20cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4`](time-limit_rise20cm_tread31cm_width160cm_offset+0cm_sim-truth-centering.mp4) |
| 跌倒 | 15 / 25 / 160 cm，起点偏左 10 cm | [`fall_rise15cm_tread25cm_width160cm_offset-10cm_sim-truth-centering.mp4`](fall_rise15cm_tread25cm_width160cm_offset-10cm_sim-truth-centering.mp4) |
| 越界 | 18 / 31 / 160 cm，起点偏右 10 cm | [`stair-exit_rise18cm_tread31cm_width160cm_offset+10cm_sim-truth-centering.mp4`](stair-exit_rise18cm_tread31cm_width160cm_offset+10cm_sim-truth-centering.mp4) |
| 限时未到顶 | 15 / 20 / 160 cm，起点偏左 10 cm | [`time-limit_rise15cm_tread20cm_width160cm_offset-10cm_sim-truth-centering.mp4`](time-limit_rise15cm_tread20cm_width160cm_offset-10cm_sim-truth-centering.mp4) |
| 纠偏消融 | 15 / 31 / 160 cm，中心起点，**不使用中心线纠偏** | [`stair-exit_rise15cm_tread31cm_width160cm_offset+0cm_no-centering.mp4`](stair-exit_rise15cm_tread31cm_width160cm_offset+0cm_no-centering.mp4) |

## 复现

克隆上述第三方仓库，确保包含 `logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt`、机器人 MJCF 和 mesh。安装 `mujoco numpy torch Pillow pynput`，系统安装 `ffmpeg`，然后在本项目根目录运行：

```sh
python scripts/run_stair_matrix.py
python scripts/render_stair_sweep_highlights.py
```

单独录制某组：

```sh
python scripts/run_stair_sweep.py --rise-cm 15 --tread-cm 31 --width-cm 110 --start-y-cm 0 --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
```

`run_stair_matrix.py` 只生成数据，不渲染视频。录像脚本从当前配置重新跑同条件，并核对终止原因。已有的旧场景录像在 [`../stair-replays/`](../stair-replays/)；它们与本次参数化场景有细小几何差异，不混入本次 42 次统计。

轨迹可用 `gzip -dc artifacts/stair-sweep/r15_t31_w160_y0.csv.gz` 解压查看，也可由 Python 的 `gzip.open`、`pandas.read_csv` 直接读取。
