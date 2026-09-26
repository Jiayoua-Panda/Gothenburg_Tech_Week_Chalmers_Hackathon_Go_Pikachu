"""④ 结合版：DarinSec 的技能切换 + 安全层  ×  Zhichao 的摄像头定位 + 路线监督。

和 ③ switch 的区别只在"机器人怎么知道自己在哪、怎么纠偏"：
  - 纠偏（横向偏移、朝向）用摄像头定位的估计值，不再用仿真器真值。
    摄像头模型照搬 scripts/run_factory_course.py 的 Localizer：σ 5 cm、10 Hz、200 ms 延迟、
    5 cm 固定偏差，并融合机器人自身里程计（START_HERE.md 里"普通摄像头就够了"的那一档）。
  - 上楼梯前对正：地图里下一段是楼梯、离第一级 < 0.35 m 且朝向偏 > 6° 时，停下原地转正到 < 3°。
  - 卡住后退重试：4 s 内前进 < 5 cm 就后退 1 s 再走（安全层让它停下时不算卡住）。
技能选择（高度扫描选 V0 / DWAQ）和安全层（ISO 1.67 m + 位置保持）与 ③ 完全相同。
通过 / 摔倒的判定仍按真值，只有控制用估计值。

不改 run_course.py：把 "combined" 注册进它的控制器表，直接复用巡回 / 分段模式。

用法：
    py combined.py --mode tour --course demo --safety iso      # 巡回视频 → outputs/combined/demo/
    py combined.py --mode segments --course demo --trials 10   # 每段 10 次随机起点（摄像头噪声也随机）
"""

from __future__ import annotations

import argparse
import math
from collections import deque

import numpy as np

import course as C
import run_course as R

CAMERA = dict(sigma_xy=0.05, sigma_yaw=math.radians(2.0), rate_hz=10.0, latency_s=0.2, bias_xy=0.05, fusion=True)
ALIGN_ZONE, ALIGN_ENTER, ALIGN_EXIT, ALIGN_MAX_S = 0.35, math.radians(6), math.radians(3), 3.0
STALL_WINDOW_S, STALL_MIN_GAIN, BACKOFF_S, BACKOFF_VX = 4.0, 0.05, 1.0, -0.2
TURN_RATE = 1.0  # 对正时用满速转向（DWAQ 小转向指令会被忽略，见 START_HERE.md）
STAIR_KINDS = {"stairs", "angled_stairs"}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Localizer:
    """摄像头定位（照搬 scripts/run_factory_course.py 的 Localizer，作者 Zhichao）。

    每次定位 = latency 秒前的真值 + 白噪声 + 固定偏差，按 rate_hz 送达；fusion 时在最新定位上
    叠加机器人自身里程计（速度有比例误差、陀螺有零偏），同时补偿低刷新率和延迟。
    """

    def __init__(self, dt, sigma_xy=0.0, sigma_yaw=0.0, rate_hz=0.0, latency_s=0.0, bias_xy=0.0, fusion=False,
                 seed=0, odom_scale_error=0.1, gyro_bias=0.01):
        self.dt = dt
        self.sigma_xy, self.sigma_yaw = sigma_xy, sigma_yaw
        self.period = 1 / rate_hz if rate_hz > 0 else 0.0
        self.latency, self.fusion = latency_s, fusion
        self.rng = np.random.default_rng(seed)
        angle = self.rng.uniform(-math.pi, math.pi)
        self.bias = np.array([bias_xy * math.cos(angle), bias_xy * math.sin(angle), 0.0])
        self.odom_scale = 1 + odom_scale_error * self.rng.choice([-1.0, 1.0])
        self.gyro_bias = gyro_bias * self.rng.choice([-1.0, 1.0])
        self.history, self.odometry = deque(), deque()
        self.estimate, self.next_update = None, 0.0

    def update(self, t, true_pose, world_velocity, yaw_rate):
        self.history.append((t, true_pose.copy()))
        while len(self.history) > 1 and self.history[1][0] <= t - self.latency:
            self.history.popleft()
        step = np.array([*(world_velocity[:2] * self.odom_scale * self.dt), (yaw_rate + self.gyro_bias) * self.dt])
        self.odometry.append((t, step))
        while self.odometry and self.odometry[0][0] < t - self.latency - self.period - 1.0:
            self.odometry.popleft()
        if self.estimate is None or t + 1e-9 >= self.next_update:
            fix_time, delayed = self.history[0]
            noise = np.array([self.rng.normal(0, self.sigma_xy), self.rng.normal(0, self.sigma_xy),
                              self.rng.normal(0, self.sigma_yaw)])
            self.estimate = delayed + noise + self.bias
            if self.fusion:
                for when, delta in self.odometry:
                    if when > fix_time:
                        self.estimate = self.estimate + delta
            self.next_update = t + self.period
        elif self.fusion:
            self.estimate = self.estimate + step
        return self.estimate.copy()


class CombinedSim(R.SwitchSim):
    """SwitchSim + 摄像头定位；记下实际执行的指令，好让"卡住"判断跳过安全层的主动停车。"""

    episode = 0  # 每次复位换一个摄像头随机种子（噪声、偏差方向、里程计误差都不同）

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        super().reset(x, y, yaw)
        CombinedSim.episode += 1
        self.localizer = Localizer(self.policy.step_dt, seed=CombinedSim.episode, **CAMERA)
        self.pose_est = np.array([x, y, yaw])
        self.last_cmd = np.zeros(3)

    def step(self, cmd):
        self.last_cmd = np.array(cmd, dtype=float)
        super().step(cmd)
        true_pose = np.array([self.data.qpos[0], self.data.qpos[1], self.yaw()])
        self.pose_est = self.localizer.update(self.data.time, true_pose, self.data.qvel[0:3], float(self.data.qvel[5]))


def next_stair_start(course, x):
    """地图（路线定义）里 x 前方最近的楼梯起点。"""
    starts = [x0 for _, x0, _, spec in course.segments if spec["kind"] in STAIR_KINDS and x0 > x - 0.05]
    return min(starts) if starts else math.inf


def controller_combined(sim, t, vx=0.5, settle=2.0, ramp=1.0):
    # 技能选择和速度与 ③ 完全一样（高度扫描是机载传感器，不依赖定位）
    cmd = np.array(R.controller_switch(sim, t, vx=vx, settle=settle, ramp=ramp), dtype=float)
    if t < settle:
        return cmd
    st = sim.ctrl_state
    st.setdefault("aligned", set())
    st.setdefault("watch", deque())
    st.setdefault("backoff_until", -1.0)
    st.setdefault("align_since", None)
    x_est, y_est, yaw_est = sim.pose_est

    # 纠偏：和 steer() 同样的增益，但输入是摄像头估计值
    cmd[1] = -1.0 * y_est
    cmd[2] = -1.5 * wrap(yaw_est)

    # 上楼梯前对正（地图知道楼梯在哪）
    x_stair = next_stair_start(sim.course, x_est)
    if 0.0 < x_stair - x_est < ALIGN_ZONE and x_stair not in st["aligned"]:
        err = wrap(yaw_est)
        if st["align_since"] is None and abs(err) > ALIGN_ENTER:
            st["align_since"] = t
        if st["align_since"] is not None:
            if abs(err) < ALIGN_EXIT or t - st["align_since"] > ALIGN_MAX_S:
                st["aligned"].add(x_stair)
                st["align_since"] = None
            else:
                st["watch"].clear()
                return sim.policy.clip_command([0.0, float(np.clip(-0.5 * y_est, -0.25, 0.25)), -math.copysign(TURN_RATE, err)])
        # 朝向还正就继续走，但整个接近区里一直检查（和 RouteFollower 一样），不是只在进入时看一次

    # 卡住后退重试：只在真的在往前走（安全层没让停）时计时
    if t < st["backoff_until"]:
        return sim.policy.clip_command([BACKOFF_VX, cmd[1], cmd[2]])
    if sim.last_cmd[0] > 0.1:
        st["watch"].append((t, x_est))
        while st["watch"] and st["watch"][0][0] < t - STALL_WINDOW_S:
            st["watch"].popleft()
        if t - st["watch"][0][0] >= STALL_WINDOW_S - 0.05 and x_est - st["watch"][0][1] < STALL_MIN_GAIN:
            st["backoff_until"] = t + BACKOFF_S
            st["watch"].clear()
            st["retries"] = st.get("retries", 0) + 1
            return sim.policy.clip_command([BACKOFF_VX, cmd[1], cmd[2]])
    else:
        st["watch"].clear()
    return sim.policy.clip_command(cmd)


def register():
    R.CONTROLLERS["combined"] = controller_combined
    R.CONTROLLER_LABELS["combined"] = "4 Combined: switch + camera localization + supervisor"
    make_sim = R.make_sim

    def make_sim_combined(controller, scene_xml, course):
        return CombinedSim(scene_xml, course) if controller == "combined" else make_sim(controller, scene_xml, course)

    R.make_sim = make_sim_combined


def main():
    register()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["tour", "segments", "full"], default="tour")
    p.add_argument("--course", choices=list(C.NAMED_COURSES), default="demo")
    p.add_argument("--safety", choices=["off", "stop", "hold", "iso"], default="iso")
    p.add_argument("--speed", type=float, default=0.5)
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--time-limit", type=float, default=90.0)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--show-scan", action="store_true")
    p.add_argument("--stuck-time", type=float, default=8.0)
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    args = p.parse_args()
    args.controller = "combined"
    {"tour": R.mode_tour, "segments": R.mode_segments, "full": R.mode_full}[args.mode](args)


if __name__ == "__main__":
    main()
