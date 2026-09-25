# G1 楼梯 MuJoCo 回放

这些视频是 [G1DWAQ_Lab](https://github.com/liuyufei-nubot/G1DWAQ_Lab) 已发布权重的本地仿真回放，不是本项目训练出的模型，也不是机器人视觉自主导航。它与本仓库 `g1_step_playback/` 使用的 Unitree 官方 V0 行走策略不同，场景也不同，不能直接把两组结果当作同条件对比。每段都有同名 CSV 轨迹和 JSON 运行条件。

| 视频 | 台阶 | 前进命令 | 中心线纠偏 | 结果 |
| --- | --- | --- | --- | --- |
| `success_15cm_center.mp4` | 15 cm 高，31 cm 深 | 0.3 m/s | 用 MuJoCo 的真实位置和朝向 | 31.78 s 到达上层走廊，x = 8.00 m |
| `failure_15cm_no_center_same_speed.mp4` | 15 cm 高，31 cm 深 | 0.3 m/s | 无 | 8.38 s 越出 1.6 m 宽楼梯边缘；与成功组同速对照 |
| `failure_15cm_no_center.mp4` | 15 cm 高，31 cm 深 | 0.4 m/s | 无 | 11.42 s 跌倒；速度与成功组不同，不能作为单独的纠偏对照 |
| `failure_20cm_center.mp4` | 20 cm 高，40 cm 深 | 0.3 m/s | 用 MuJoCo 的真实位置和朝向 | 10.64 s 跌倒 |

策略本身没有读取相机图像。视频中侧墙仅为拍摄设成半透明；碰撞和动力学未改。四个条件各是一条确定性回放，不能据此推断统计成功率。场景文件在 `scenes/`，录制脚本在 `scripts/render_stair_replays.py`。

复现时先克隆 G1DWAQ_Lab（其中包含 `model_9999.pt` 和机器人模型），安装 Python 依赖 `mujoco numpy torch Pillow pynput`，然后从本仓库根目录执行：

```sh
python scripts/render_stair_replays.py success_15cm_center --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
```

脚本会在临时目录组装本仓库的场景与第三方机器人模型，使用 `weights_only=True` 加载权重，并调用系统 `ffmpeg` 编码 MP4。
