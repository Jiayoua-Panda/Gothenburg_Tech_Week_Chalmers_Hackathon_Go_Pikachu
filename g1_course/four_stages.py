"""四种状态，同一路线；安全层 ① 没有，②③④ 都是 ISO 1.67 m + 位置保持：

                  定位（保持在通道中间）              地形信息
    ① V0          外部摄像头 + 机器人里程计融合        无：全程 Unitree 平地策略
    ② Robot       只靠机器人自身里程计（航位推算）     机载高度扫描（DarinSec）：看到台阶 / 坡 → DWAQ，平地 → V0
    ③ External    工厂固定摄像头 + 机器人里程计融合    地图（Zhichao，不用机载扫描）：地图上的台阶 / 坡区段 → DWAQ；上楼梯前对正、卡住后退重试
    ④ Both        外部摄像头 + 机器人里程计融合        扫描 + 地图：任一方说有地形就用 DWAQ，两者都说平才回 V0；监督同 ③

摄像头：combined.CAMERA（σ 5 cm、10 Hz、200 ms 延迟、5 cm 固定偏差）。
里程计：腿部里程计比例误差 5 %、陀螺零偏 0.001 rad/s（≈ 3.4°/min，中等 MEMS、开机静止校准后），误差不会被校正。

路线 demo4 = demo 路线 + 两个故障注入段：
    U  通道上临时放了一块 10 cm 托盘，地图里没有（现场改了、地图没更新）→ 只靠地图会绊倒
    R  10 cm 台阶前铺着反光金属板，高度扫描在这里看不到东西（当成平地）→ 只靠眼睛会绊倒

用法：
    py four_stages.py --mode tour --stage all          # 四段巡回视频（依次）
    py four_stages.py --mode segments --stage all      # 每段 10 次随机起点
    py four_stages.py --mode compare                   # 四宫格视频 + 热力图
"""

from __future__ import annotations

import argparse
import math
from collections import deque

import numpy as np

import combined
import course as C
import hybrid
import run_course as R

STAGES = ["s1_v0", "s2_eyes", "s3_map", "s4_both"]
LABELS = {
    "s1_v0": "1 V0 (no terrain info)",
    "s2_eyes": "2 Robot only: height scan + own odometry",
    "s3_map": "3 External: map + fixed cameras (+ odometry)",
    "s4_both": "4 Robot + External",
}
LOCALIZATION = {"s1_v0": "fused", "s2_eyes": "odometry", "s3_map": "fused", "s4_both": "fused"}
# ① 是"什么都没加"的基线：没有安全层（行人横穿时会撞上）；②③④ 都用 ISO 1.67 m + 位置保持
SAFETY = {"s1_v0": "off", "s2_eyes": "iso", "s3_map": "iso", "s4_both": "iso"}
ODOM_SCALE_ERROR, GYRO_BIAS = 0.05, 0.001


class Odometry:
    """只靠机器人自身：腿部里程计给机体速度（有比例误差），陀螺积分给朝向（有零偏），从已知起点航位推算。"""

    def __init__(self, dt, start_pose, seed):
        rng = np.random.default_rng(seed)
        self.dt = dt
        self.scale = 1 + ODOM_SCALE_ERROR * rng.choice([-1.0, 1.0])
        self.bias = GYRO_BIAS * rng.choice([-1.0, 1.0])
        self.estimate = np.array(start_pose, dtype=float)

    def update(self, t, true_pose, world_velocity, yaw_rate):
        c, s = math.cos(true_pose[2]), math.sin(true_pose[2])
        v_body = np.array([c * world_velocity[0] + s * world_velocity[1], -s * world_velocity[0] + c * world_velocity[1]])
        self.estimate[2] += (yaw_rate + self.bias) * self.dt
        ce, se = math.cos(self.estimate[2]), math.sin(self.estimate[2])
        self.estimate[:2] += self.scale * self.dt * np.array([ce * v_body[0] - se * v_body[1], se * v_body[0] + ce * v_body[1]])
        return self.estimate.copy()


class StageSim(combined.CombinedSim):
    """CombinedSim，但定位方式按阶段选：融合 / 只有摄像头 / 只有里程计。"""

    def __init__(self, scene_xml, course, localization):
        self.localization = localization
        super().__init__(scene_xml, course)

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        super().reset(x, y, yaw)
        seed, dt = combined.CombinedSim.episode, self.policy.step_dt
        if self.localization == "odometry":
            self.localizer = Odometry(dt, (x, y, yaw), seed)
        elif self.localization == "camera":
            self.localizer = combined.Localizer(dt, seed=seed, **{**combined.CAMERA, "fusion": False})
        # "fused"：CombinedSim.reset 已经建好（摄像头 + 里程计融合）
TERRAIN_KINDS = {"ramp", "block", "stairs", "angled_stairs", "cross_slope", "rough"}
MAP_LEAD, MAP_TAIL = 0.9, 0.6  # 地图上的地形区段前 0.9 m 开始用 DWAQ（与扫描看得到的距离一致），过后 0.6 m 才放
LEVEL_TICKS = 25  # 连续 0.5 s 判定为平地才切回 V0（与 controller_switch 一致）

# demo 路线 + 两个故障注入段（插在 C3 之后、F1 之前，以及人横穿之后、楼梯之前）
DEMO4 = []
for spec in C.DEMO_COURSE:
    DEMO4.append(spec)
    if spec.get("name") == "C3 step 15cm":
        DEMO4 += [dict(kind="flat", length=1.0),
                  dict(kind="block", height=0.10, length=0.6, name="U pallet (unmapped)", unmapped=True)]
    if spec.get("name") == "S person crossing":
        DEMO4 += [dict(kind="flat", length=1.5),
                  dict(kind="block", height=0.10, length=1.0, name="R step, glare (blind)", scan_blind=True)]

"""
故障注入：扫描盲区 + 场景装饰（托盘木色、反光板银色，只是画面，不参与碰撞和扫描）。
"""

_terrain_features = hybrid.terrain_features
BLIND_BEFORE, BLIND_AFTER = 1.3, 0.3  # 反光板铺在台阶前 1.3 m 到台阶后 0.3 m


def blind_zones(course):
    return [(x0 - BLIND_BEFORE, x1 + BLIND_AFTER) for _, x0, x1, s in course.segments if s.get("scan_blind")]


def terrain_features_with_glare(sim, rng):
    x = sim.data.qpos[0]
    if any(a <= x <= b for a, b in blind_zones(sim.course)):
        rng.normal()  # 保持随机数序列一致
        return dict(up=0.0, down=0.0, edge=math.inf, pitch=0.0, roll=0.0)
    return _terrain_features(sim, rng)


_write_scene = C.write_scene

# 行人外观：工厂工人（安全帽、反光背心、工装裤）。只换画面，mocap 位置与碰撞距离判断不变。
# 人沿 y 方向横穿，所以肩膀沿 x 轴排开、脸朝 ±y。
_NC = 'contype="0" conaffinity="0" group="0"'
WORKER_GEOMS = "".join(f'<geom {g} {_NC}/>' for g in [
    'name="human_body" type="capsule" fromto="0 0 0.95 0 0 1.35" size="0.19" rgba="0.98 0.50 0.10 1"',  # 背心
    'type="cylinder" pos="0 0 1.12" size="0.196 0.022" rgba="0.85 0.88 0.9 1"',  # 反光条
    'type="capsule" fromto="0.10 0 0.10 0.10 0 0.88" size="0.075" rgba="0.16 0.22 0.38 1"',  # 腿
    'type="capsule" fromto="-0.10 0 0.10 -0.10 0 0.88" size="0.075" rgba="0.16 0.22 0.38 1"',
    'type="box" pos="0.10 0 0.045" size="0.065 0.13 0.045" rgba="0.1 0.1 0.1 1"',  # 鞋
    'type="box" pos="-0.10 0 0.045" size="0.065 0.13 0.045" rgba="0.1 0.1 0.1 1"',
    'type="capsule" fromto="0.25 0 1.36 0.29 0 0.88" size="0.055" rgba="0.98 0.50 0.10 1"',  # 手臂
    'type="capsule" fromto="-0.25 0 1.36 -0.29 0 0.88" size="0.055" rgba="0.98 0.50 0.10 1"',
    'type="sphere" pos="0.29 0 0.82" size="0.05" rgba="0.93 0.74 0.58 1"',  # 手
    'type="sphere" pos="-0.29 0 0.82" size="0.05" rgba="0.93 0.74 0.58 1"',
    'type="capsule" fromto="0 0 1.40 0 0 1.50" size="0.05" rgba="0.93 0.74 0.58 1"',  # 脖子
    'name="human_head" type="sphere" pos="0 0 1.60" size="0.11" rgba="0.93 0.74 0.58 1"',
    'type="sphere" pos="0 0 1.66" size="0.118" rgba="1.0 0.85 0.1 1"',  # 安全帽
    'type="cylinder" pos="0 0 1.64" size="0.145 0.008" rgba="1.0 0.85 0.1 1"',  # 帽檐
])


def write_scene_decorated(course, name, human=False):
    import re

    path = _write_scene(course, name, human=human)
    if human:
        xml = path.read_text(encoding="utf-8")
        xml = re.sub(r'(<body name="human"[^>]*>).*?(</body>)', lambda m: m.group(1) + WORKER_GEOMS + m.group(2),
                     xml, count=1, flags=re.S)
        path.write_text(xml, encoding="utf-8")
    deco = []
    for _, x0, x1, s in course.segments:
        if s.get("unmapped"):  # 托盘：木色外壳，略大于台阶方块
            top = course.ground((x0 + x1) / 2) + 0.004
            base = course.ground(x0 - 0.05)
            deco.append(f'<geom type="box" pos="{(x0 + x1) / 2:.4f} 0 {(top + base) / 2:.4f}" '
                        f'size="{(x1 - x0) / 2 + 0.006:.4f} {C.TRACK_HALF_WIDTH + 0.006:.4f} {(top - base) / 2:.4f}" '
                        f'rgba="0.66 0.47 0.24 1" contype="0" conaffinity="0" group="0"/>')
        if s.get("scan_blind"):  # 反光金属板：台阶前后的地面和台阶顶面
            a, b = x0 - BLIND_BEFORE, x1 + BLIND_AFTER
            for p0, p1 in ((a, x0), (x0, x1), (x1, b)):
                z = course.ground((p0 + p1) / 2) + 0.003
                deco.append(f'<geom type="box" pos="{(p0 + p1) / 2:.4f} 0 {z:.4f}" '
                            f'size="{(p1 - p0) / 2:.4f} {C.TRACK_HALF_WIDTH:.4f} 0.002" '
                            f'rgba="0.86 0.89 0.93 1" contype="0" conaffinity="0" group="0"/>')
    if deco:
        xml = path.read_text(encoding="utf-8")
        xml = xml.replace("</worldbody>", "    " + "\n    ".join(deco) + "\n  </worldbody>", 1)
        path.write_text(xml, encoding="utf-8")
    return path


"""
四个控制器。共同部分：settle、速度、摄像头定位纠偏；区别只在技能选择（和 ③④ 的监督逻辑）。
"""


def map_says_terrain(sim, x_est):
    for _, x0, x1, s in sim.course.segments:
        if s["kind"] in TERRAIN_KINDS and not s.get("unmapped") and x0 - MAP_LEAD <= x_est <= x1 + MAP_TAIL:
            return True
    return False


def scan_says(sim, st):
    """返回 (看到地形, 看到平地)，阈值与 controller_switch 相同。"""
    f = hybrid.terrain_features(sim, st["rng"])
    step_h = max(f["up"], -f["down"])
    tilted = abs(f["roll"]) > R.ROLL_SWITCH
    terrain = (step_h > 0.04 and f["edge"] < 0.9) or tilted
    level = step_h < 0.025 and f["edge"] == math.inf and abs(f["roll"]) < 0.5 * R.ROLL_SWITCH
    return terrain, level


def make_controller(use_scan, use_map, supervise):
    def controller(sim, t, vx=0.5, settle=2.0, ramp=1.0):
        st = sim.ctrl_state
        if "rng" not in st:
            st.update(rng=np.random.default_rng(0), level_ticks=0)
        if t < settle:
            return np.zeros(3)
        x_est, y_est, yaw_est = sim.pose_est

        if use_scan or use_map:
            s_terrain, s_level = scan_says(sim, st) if use_scan else (False, True)
            m_terrain = map_says_terrain(sim, x_est) if use_map else False
            if sim.skill == "flat" and (s_terrain or m_terrain):
                sim.request("stairs")
            if sim.skill == "stairs":
                level = s_level and not m_terrain
                st["level_ticks"] = st["level_ticks"] + 1 if level else 0
                if st["level_ticks"] >= LEVEL_TICKS:
                    sim.request("flat")

        speed = R.DWAQ_SPEED if (sim.skill == "stairs" or sim.next_skill) else vx
        v = speed * min(1.0, (t - settle) / ramp)
        cmd = np.array([v, -1.0 * y_est, -1.5 * combined.wrap(yaw_est)])
        if supervise:
            cmd = supervisor(sim, t, st, cmd)
        return sim.policy.clip_command(cmd)

    return controller


def supervisor(sim, t, st, cmd):
    """Zhichao 的路线监督：地图上下一段是楼梯时先对正；卡住后退重试（安全层停车不算卡住）。"""
    st.setdefault("aligned", set())
    st.setdefault("watch", deque())
    st.setdefault("backoff_until", -1.0)
    st.setdefault("align_since", None)
    x_est, y_est, yaw_est = sim.pose_est
    x_stair = combined.next_stair_start(sim.course, x_est)
    if 0.0 < x_stair - x_est < combined.ALIGN_ZONE and x_stair not in st["aligned"]:
        err = combined.wrap(yaw_est)
        if st["align_since"] is None and abs(err) > combined.ALIGN_ENTER:
            st["align_since"] = t
        if st["align_since"] is not None:
            if abs(err) < combined.ALIGN_EXIT or t - st["align_since"] > combined.ALIGN_MAX_S:
                st["aligned"].add(x_stair)
                st["align_since"] = None
            else:
                st["watch"].clear()
                return np.array([0.0, float(np.clip(-0.5 * y_est, -0.25, 0.25)), -math.copysign(combined.TURN_RATE, err)])
        # 朝向还正就继续走，但整个接近区里一直检查（和 RouteFollower 一样），不是只在进入时看一次
    if t < st["backoff_until"]:
        return np.array([combined.BACKOFF_VX, cmd[1], cmd[2]])
    if sim.last_cmd[0] > 0.1:
        st["watch"].append((t, x_est))
        while st["watch"] and st["watch"][0][0] < t - combined.STALL_WINDOW_S:
            st["watch"].popleft()
        if t - st["watch"][0][0] >= combined.STALL_WINDOW_S - 0.05 and x_est - st["watch"][0][1] < combined.STALL_MIN_GAIN:
            st["backoff_until"] = t + combined.BACKOFF_S
            st["watch"].clear()
            return np.array([combined.BACKOFF_VX, cmd[1], cmd[2]])
    else:
        st["watch"].clear()
    return cmd


CONTROLLERS = {
    "s1_v0": make_controller(use_scan=False, use_map=False, supervise=False),
    "s2_eyes": make_controller(use_scan=True, use_map=False, supervise=False),
    "s3_map": make_controller(use_scan=False, use_map=True, supervise=True),
    "s4_both": make_controller(use_scan=True, use_map=True, supervise=True),
}


def register():
    C.NAMED_COURSES["demo4"] = (DEMO4, C.BASE_3D)
    R.COURSE_TITLES["demo4"] = "G1 demo4: demo course + unmapped pallet (U) + scan-blind step under glare (R)"
    hybrid.terrain_features = terrain_features_with_glare
    C.write_scene = write_scene_decorated
    R.CONTROLLERS.update(CONTROLLERS)
    R.CONTROLLER_LABELS.update(LABELS)
    make_sim = R.make_sim
    R.make_sim = lambda c, xml, course: StageSim(xml, course, LOCALIZATION[c]) if c in CONTROLLERS else make_sim(c, xml, course)


def compare(course):
    import make_compare4 as M

    M.STAGES = [
        ("s1_v0", "1  V0  |  no terrain information  |  no safety layer", "1 V0", (200, 70, 60)),
        ("s2_eyes", "2  Robot only: height scan + own odometry", "2 Robot", (230, 150, 30)),
        ("s3_map", "3  External: map + fixed cameras (+ robot odometry)", "3 External", (140, 90, 200)),
        ("s4_both", "4  Robot + External combined", "4 Both", (50, 150, 90)),
    ]
    M.pass_rate_heatmap(course, person_safety=SAFETY)
    M.compare_video(course)


def main():
    register()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["tour", "segments", "profile", "compare"], default="tour")
    p.add_argument("--stage", choices=[*STAGES, "all"], default="all")
    p.add_argument("--course", default="demo4")
    p.add_argument("--safety", choices=["off", "stop", "hold", "iso"], default=None, help="默认按阶段：① off，②③④ iso")
    p.add_argument("--speed", type=float, default=0.5)
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--time-limit", type=float, default=40.0)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--show-scan", action="store_true")
    p.add_argument("--stuck-time", type=float, default=8.0)
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    args = p.parse_args()
    if args.mode == "compare":
        compare(args.course)
        return
    safety_arg = args.safety
    for stage in STAGES if args.stage == "all" else [args.stage]:
        args.controller = stage
        args.safety = safety_arg or SAFETY[stage]
        print(f"===== {LABELS[stage]}")
        {"tour": R.mode_tour, "segments": R.mode_segments, "profile": R.mode_profile}[args.mode](args)


if __name__ == "__main__":
    main()
