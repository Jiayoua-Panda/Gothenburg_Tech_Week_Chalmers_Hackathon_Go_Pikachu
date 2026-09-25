"""在测试路线上评测 G1 策略（V0 = Unitree 预训练策略，恒定速度指令）。

用法：
    py run_course.py --mode profile            # 只画路线剖面图
    py run_course.py --mode full               # 全程跑一次，录视频（第一次失败就停）
    py run_course.py --mode tour               # 巡回：失败后从下一段重新出发，一段视频看完所有路段
    py run_course.py --mode segments --trials 10   # 每段单独测 N 次，统计通过率
    py run_course.py --mode viewer             # 交互窗口看全程
    py run_course.py --mode scan --course 3d   # 感知视图：机器人前方高度图（PNG）
    py run_course.py --mode safety --trials 50 # 安全演示：人横穿通道，off / stop / hold / iso 四种安全配置对比
    加 --course 3d：侧倾坡 / 不平地面 / 斜向与窄楼梯（结果在 outputs/<controller>/3d/）
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
import time

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "g1_step_playback"))

import course as C  # noqa: E402
from perception import SCAN_CMAP, SCAN_FWD, SCAN_LAT, SCAN_RANGE, add_scan, height_scan  # noqa: E402
from play_g1 import FOOT_BODIES, G1Policy, G1Sim, StepTracker, add_footprints  # noqa: E402

POLICY_DIR = ROOT.parent / "g1_step_playback" / "policy"
OUT_ROOT = ROOT / "outputs"


class CourseSim(G1Sim):
    """G1Sim + 自定义场景、出生点、按脚下地面高度判摔倒、任意地形接触算落地。

    residual：叠加在策略目标关节角上的偏置（SDK 顺序），混合控制器用它抬腿；默认全 0，行为与 G1Sim 一致。
    """

    def __init__(self, policy, scene_xml, course: C.Course):
        self.course = course
        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.model.opt.timestep = 0.005
        self.data = mujoco.MjData(self.model)
        self.policy = policy
        self.decimation = round(policy.step_dt / self.model.opt.timestep)
        jnt = self.model.actuator_trnid[:, 0]
        self.qadr = self.model.jnt_qposadr[jnt]
        self.vadr = self.model.jnt_dofadr[jnt]
        self.tau_lim = self.model.actuator_ctrlrange[:, 1]
        self.pelvis = self.model.body("pelvis").id
        self.floor = self.model.geom("floor").id
        self.foot_ids = [self.model.body(n).id for n in FOOT_BODIES]
        self.spawn = (0.0, 0.0, 0.0)
        self.reset()

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.spawn = (x, y, yaw)
        self.residual = np.zeros(len(self.qadr))
        self.ctrl_state = {}  # 控制器自己的每回合状态（混合控制器的摆动相位等）
        super().reset()
        self.data.qpos[0], self.data.qpos[1] = x, y
        self.data.qpos[2] = 0.78 + self.course.ground(x, y)
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.model, self.data)

    def step(self, cmd):
        """同 G1Sim.step，只是 PD 目标 = 策略目标 + residual。"""
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.vadr]
        ang_vel_b, gravity_b = self.base_state()
        self.target = self.policy.act(q, dq, ang_vel_b, gravity_b, cmd) + self.residual
        for _ in range(self.decimation):
            q = self.data.qpos[self.qadr]
            dq = self.data.qvel[self.vadr]
            tau = self.policy.kp * (self.target - q) - self.policy.kd * dq
            self.data.ctrl[:] = np.clip(tau, -self.tau_lim, self.tau_lim)
            mujoco.mj_step(self.model, self.data)

    def feet_in_contact(self):
        contact = [False, False]
        for c in self.data.contact[: self.data.ncon]:
            for g_foot, g_other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if self.model.geom_bodyid[g_other] == 0:  # 世界体：地面或路线上的方块
                    body = self.model.geom_bodyid[g_foot]
                    if body in self.foot_ids:
                        contact[self.foot_ids.index(body)] = True
        return contact

    def fallen(self):
        _, g = self.base_state()
        x, y = self.data.qpos[0], self.data.qpos[1]
        rel_h = self.data.qpos[2] - self.course.ground(x, y)
        return rel_h < 0.45 or g[2] > -0.6

    def yaw(self):
        h = self.heading()
        return math.atan2(h[1], h[0])


"""
速度指令（"控制器"）。V0：恒定前进速度 + 操作员式纠偏（朝向、横向偏移）。
"""


def steer(sim):
    vy = -1.0 * sim.data.qpos[1]
    wz = -1.5 * sim.yaw()
    return vy, wz


def controller_v0(sim, t, vx=0.5, settle=2.0, ramp=1.0):
    if t < settle:
        return np.zeros(3)
    v = vx * min(1.0, (t - settle) / ramp)
    vy, wz = steer(sim)
    return sim.policy.clip_command([v, vy, wz])


def controller_hybrid(sim, t, **kw):
    """V0.5：感知 + V0 + 经典层（hybrid.py，参数来自 outputs/hybrid/best_params.json）。"""
    import hybrid

    return hybrid.controller(sim, t, **kw)


CONTROLLERS = {"v0": controller_v0, "hybrid": controller_hybrid}
CONTROLLER_LABELS = {"v0": "V0 pretrained", "hybrid": "V0.5 hybrid (scan + V0 + tuned layer)"}


class VideoSink:
    """边渲染边写 mp4，不把几千帧留在内存里（8 GB 内存的笔记本也能录长视频）。"""

    def __init__(self, path, fps):
        import imageio.v2 as imageio

        self.path = path
        self.writer = imageio.get_writer(path, fps=fps, codec="libx264", quality=8, macro_block_size=1)
        self.last = None

    def append(self, img):
        self.writer.append_data(img)
        self.last = img

    def hold(self, n):
        for _ in range(n if self.last is not None else 0):
            self.writer.append_data(self.last)

    def close(self):
        self.writer.close()


"""
一次试验。
"""


def run_trial(sim, controller, goal_x, time_limit, video_path=None, width=960, height=540, vx=0.5, show_scan=False,
              label="V0 pretrained"):
    tracker = StepTracker(sim)
    renderer = mujoco.Renderer(sim.model, height, width) if video_path else None
    sink = VideoSink(video_path, fps=round(1 / sim.policy.step_dt)) if video_path else None
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 3.2, 90.0, -10.0

    dt = sim.policy.step_dt
    status, t = "timeout", 0.0
    x_max, y_dev = -np.inf, 0.0
    for k in range(int(time_limit / dt)):
        t = k * dt
        cmd = controller(sim, t, vx=vx)
        sim.step(cmd)
        tracker.update(t, dt, record=True)
        x, y = sim.data.qpos[0], sim.data.qpos[1]
        x_max, y_dev = max(x_max, x), max(y_dev, abs(y))
        if abs(y) > C.TRACK_HALF_WIDTH:  # 走出 1.2 m 宽的通道：工业现场里等于撞上旁边的东西
            status = "off_track"
            break
        if sim.fallen():
            status = "fell"
            break
        if x >= goal_x:
            status = "pass"
            break
        if renderer is not None:
            renderer.update_scene(sim.data, camera=cam)
            add_footprints(renderer.scene, tracker.footprints)
            if show_scan:
                add_scan(renderer.scene, sim)
            img = renderer.render().copy()
            seg = sim.course.segment_at(x) or "-"
            lines = [f"{label}   segment: {seg}", f"x = {x:5.2f} m   cmd vx = {cmd[0]:.2f} m/s   t = {t:4.1f} s"]
            import cv2

            for j, s in enumerate(lines):
                cv2.putText(img, s, (14, 32 + 30 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            sink.append(img)
    if renderer is not None:
        renderer.close()
        sink.hold(int(1.0 / dt))  # 摔倒 / 结束后多留 1 s 画面
        sink.close()
    return dict(status=status, t=t, x=float(sim.data.qpos[0]), x_max=float(x_max), y_dev=float(y_dev),
                events=tracker.events)


def stride_metrics(events, course: C.Course, seg_name):
    """步幅区：区内平均步长与目标间距之差。"""
    seg = next(s for s in course.segments if s[0] == seg_name)
    _, x0, x1, spec = seg
    lengths = [e[2] for e in events if x0 <= e[3] <= x1]
    if not lengths:
        return float("nan"), float("nan")
    mean_len = float(np.mean(lengths))
    return mean_len, abs(mean_len - spec["spacing"])


"""
巡回模式：摔倒 / 卡住后记失败，从下一段起点前重新出发，一段视频看完所有路段。
"""

PASS_MARGIN = 0.8  # 路段之间的平地都 ≥ 1.0 m，0.8 m 仍在下一段起点之前

# 注意：MuJoCo 渲染出来是 RGB，不是 OpenCV 习惯的 BGR
STATUS_COLOR = {"PASS": (90, 220, 110), "FELL": (240, 70, 60), "STUCK": (255, 170, 40), "OFF": (200, 90, 240), "...": (200, 200, 200)}


def draw_overlay(img, label, seg, x, cmd, t, board, banner=None):
    import cv2

    font = cv2.FONT_HERSHEY_SIMPLEX
    for j, s in enumerate((f"{label}   segment: {seg}", f"x = {x:5.2f} m   cmd vx = {cmd[0]:.2f} m/s   t = {t:5.1f} s")):
        cv2.putText(img, s, (14, 32 + 30 * j), font, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    # 右侧计分板（半透明底）
    x0, y0, lh = img.shape[1] - 250, 14, 22
    overlay = img.copy()
    cv2.rectangle(overlay, (x0 - 10, y0 - 4), (img.shape[1] - 8, y0 + lh * len(board) + 8), (20, 20, 20), -1)
    img[:] = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)
    for i, (name, status) in enumerate(board):
        y = y0 + lh * (i + 1)
        cv2.putText(img, name, (x0, y), font, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(img, status, (x0 + 170, y), font, 0.5, STATUS_COLOR[status], 2, cv2.LINE_AA)
    if banner:
        text, color = banner
        (tw, th), _ = cv2.getTextSize(text, font, 1.1, 3)
        cx, cy = (img.shape[1] - tw) // 2, int(img.shape[0] * 0.88)  # 放在下方，不挡计分板
        cv2.rectangle(img, (cx - 16, cy - th - 14), (cx + tw + 16, cy + 14), (0, 0, 0), -1)
        cv2.putText(img, text, (cx, cy), font, 1.1, color, 3, cv2.LINE_AA)


def mode_tour(args):
    out = out_dir(args)
    course = C.build_named(args.course)
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, args.course), course)
    controller = CONTROLLERS[args.controller]
    segs = [s for s in course.segments if s[0] != "finish"]
    status = {s[0]: "..." for s in segs}
    label = CONTROLLER_LABELS[args.controller]

    renderer = None if args.no_video else mujoco.Renderer(sim.model, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 3.4, 90.0, -10.0

    dt = sim.policy.step_dt
    sink = None if renderer is None else VideoSink(out / "course_tour.mp4", fps=round(1 / dt))
    idx, t_total = 0, 0.0
    stuck_window = int(args.stuck_time / dt)
    sim.reset()
    t_local, hist_x = 0.0, []
    tracker = StepTracker(sim)
    cmd = np.zeros(3)

    def render(banner=None):
        if renderer is None:
            return
        renderer.update_scene(sim.data, camera=cam)
        add_footprints(renderer.scene, tracker.footprints)
        if args.show_scan:
            add_scan(renderer.scene, sim)
        img = renderer.render().copy()
        x = sim.data.qpos[0]
        board = [(s[0], status[s[0]]) for s in segs]
        draw_overlay(img, label, course.segment_at(x) or "-", x, cmd, t_total, board, banner)
        sink.append(img)

    while idx < len(segs):
        name, x0, x1, _ = segs[idx]
        cmd = controller(sim, t_local, vx=args.speed, settle=1.0)
        sim.step(cmd)
        tracker.update(t_local, dt, record=False)
        t_local += dt
        t_total += dt
        x = sim.data.qpos[0]
        hist_x.append(x)

        # 越过当前段终点后还要稳住走 PASS_MARGIN 米才算通过（冲下坡后在坡底摔倒算这一段失败）
        while idx < len(segs) and x > segs[idx][2] + PASS_MARGIN:
            status[segs[idx][0]] = "PASS"
            print(f"  {segs[idx][0]:16s} PASS   t={t_total:5.1f}s")
            idx += 1
        if idx >= len(segs):
            break

        off = abs(sim.data.qpos[1]) > C.TRACK_HALF_WIDTH
        fell = sim.fallen()
        stuck = t_local > 3.0 and len(hist_x) > stuck_window and x - hist_x[-stuck_window] < 0.3
        if off or fell or stuck:
            result = "OFF" if off else ("FELL" if fell else "STUCK")
            status[name] = result
            print(f"  {name:16s} {result:5s}  t={t_total:5.1f}s  x={x:.2f}")
            banner = (f"{result} on {name}  ->  next segment", STATUS_COLOR[result])
            for _ in range(int(1.5 / dt)):
                render(banner)
            idx += 1
            if idx >= len(segs):
                break
            sim.reset(x=segs[idx][1] - 0.8)
            t_local, hist_x = 0.0, []
            tracker = StepTracker(sim)
            continue
        render()

    for _ in range(int(2.0 / dt)):
        render(("course finished", (255, 255, 255)))
    if renderer is not None:
        renderer.close()

    n_pass = sum(v == "PASS" for v in status.values())
    print(f"巡回：通过 {n_pass}/{len(segs)} 段，总用时 {t_total:.1f} s")
    with open(out / "tour_results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["segment", "status"])
        w.writerows(status.items())
    if sink is not None:
        sink.close()
        print(f"视频：{sink.path}")


"""
感知视图：机体前方的局部高度图（V1 策略的输入长这样；V0 是盲走，看不到它）。
"""

def mode_scan(args):
    """把机器人放在几个路段前，渲染画面 + 它“看到”的高度图，输出一张 PNG 给报告用。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = out_dir(args)
    course = C.build_named(args.course)
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, args.course), course)
    segs = [s for s in course.segments if s[0] not in ("finish", "A warm-up")]
    picks = [s for s in segs if s[0].split(" ")[0] in ("F2", "G2", "H2", "I", "C2", "D2", "E1")][:4] or segs[:4]
    renderer = mujoco.Renderer(sim.model, 360, 480)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 2.8, 35.0, -30.0  # 从左后方看，楼梯在机器人前方
    fig, axes = plt.subplots(2, len(picks), figsize=(4.2 * len(picks), 6.6))
    axes = np.array(axes).reshape(2, -1)
    for k, (name, x0, _, _) in enumerate(picks):
        sim.reset(x=x0 - 0.35)
        mujoco.mj_forward(sim.model, sim.data)
        renderer.update_scene(sim.data, camera=cam)
        add_scan(renderer.scene, sim)
        axes[0, k].imshow(renderer.render())
        axes[0, k].set_title(name, fontsize=11)
        axes[0, k].axis("off")
        _, rel = height_scan(sim)
        im = axes[1, k].imshow(rel.T, origin="lower", cmap=SCAN_CMAP, vmin=-SCAN_RANGE, vmax=SCAN_RANGE,
                               extent=(SCAN_FWD[0] - 0.05, SCAN_FWD[-1] + 0.05, SCAN_LAT[0] - 0.05, SCAN_LAT[-1] + 0.05))
        axes[1, k].plot(0, 0, "k^", ms=8)
        axes[1, k].set_xlabel("forward [m]")
        if k == 0:
            axes[1, k].set_ylabel("left [m]")
    renderer.close()
    fig.colorbar(im, ax=axes[1, :].tolist(), label="height relative to robot [m]", fraction=0.02, pad=0.01)
    fig.suptitle(f"Perception view: {len(SCAN_FWD)}×{len(SCAN_LAT)} height scan in front of the robot "
                 "(input a V1 policy would use; V0 is blind)", fontsize=12)
    fig.savefig(out / "perception_view.png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"感知视图：{out / 'perception_view.png'}")


"""
安全演示：速度与间距监控（speed & separation monitoring, ISO/TS 15066 的思路）。
距离 > SSM_SLOW 全速；SSM_SLOW ~ SSM_STOP 线性减速；< SSM_STOP 指令归零站立，人离开 SSM_HOLD 秒后再走。
距离取机器人骨盆与人体中心的水平距离，带 SSM_LATENCY 的测量延迟（模拟外部传感器 + 通信）。

四种安全配置，按测试中发现的问题逐步改进：
    off   不监控
    stop  监控（停止距离 0.8 m）+ 停止时速度指令 = 0
          实测问题：V0 收到 0 指令仍会以约 0.08 m/s 往前蹭，不是真正停住
    hold  同上 + 停止时位置保持：外环 P 控制器把机器人拉回停止点（经典控制包住学习策略 = 混合方案）
          实测问题：人快速横穿时 0.8 m 不够（没算人的速度、延迟和刹车距离）
    iso   hold + 按 ISO/TS 15066 速度与间距监控公式算停止距离 S_p
"""

SAFETY_COURSE = [dict(kind="flat", length=8.0, name="S shared walkway"), dict(kind="flat", length=2.0, name="finish")]
SSM_SLOW, SSM_STOP, SSM_HOLD, SSM_LATENCY = 2.0, 0.8, 1.0, 0.1
CONTACT_DIST = 0.45  # 人体半径 0.22 + 机器人躯干约 0.2：小于这个水平距离视为碰撞
MOVING = 0.05  # 接触时机器人朝人方向的 0.5 s 平均速度 > 0.05 m/s 才算“机器人撞人”；否则是人走向已停下的机器人
HOLD_GAIN, HOLD_VMAX = 3.0, 0.3  # 位置保持外环：vx = -k·(沿朝向离停止点的距离)，限幅
# 调参实测（演示场景）：k=1 → 前冲 0.10 m、1.6 s 才停稳；k=3 → 0.046 m、0.6 s；k=5 只再好一点

# ISO/TS 15066 保护间距：S_p = v_h·(T_r + T_s) + v_r·T_r + S_s + C
ISO_V_HUMAN = 1.6  # 人的步行速度（ISO 13855 取值）m/s
ISO_T_STOP = 0.6  # 停止时间：hold 模式下触发 STOP 后走完 90% 前冲所需时间（实测 0.62 s）
ISO_S_BRAKE = 0.05  # 刹车距离：hold 模式实测前冲 0.046 m
ISO_S_P = ISO_V_HUMAN * (SSM_LATENCY + ISO_T_STOP) + 0.5 * SSM_LATENCY + ISO_S_BRAKE + CONTACT_DIST
SAFETY_CONFIGS = {
    "off": dict(monitor=False, stop=SSM_STOP, slow=SSM_SLOW, hold=False),
    "stop": dict(monitor=True, stop=SSM_STOP, slow=SSM_SLOW, hold=False),
    "hold": dict(monitor=True, stop=SSM_STOP, slow=SSM_SLOW, hold=True),
    "iso": dict(monitor=True, stop=ISO_S_P, slow=ISO_S_P + 1.2, hold=True),
}
SAFETY_MODES = tuple(SAFETY_CONFIGS)


class Person:
    """人横穿通道：从 y=+y0 走到路线中间，停留 dwell 秒，再走到 y=-y0。"""

    def __init__(self, x, t_start, speed=0.8, dwell=4.0, y0=3.0):
        self.x, self.t_start, self.speed, self.dwell, self.y0 = x, t_start, speed, dwell, y0

    def pos(self, t):
        t_in = self.y0 / self.speed
        tau = t - self.t_start
        if tau < 0:
            y = self.y0
        elif tau < t_in:
            y = self.y0 - self.speed * tau
        elif tau < t_in + self.dwell:
            y = 0.0
        else:
            y = max(-self.y0, -self.speed * (tau - t_in - self.dwell))
        return np.array([self.x, y])


class SafetyMonitor:
    def __init__(self, dt, enabled=True, stop=SSM_STOP, slow=SSM_SLOW):
        self.dt, self.enabled, self.stop, self.slow = dt, enabled, stop, slow
        self.buf = []
        self.delay = max(1, round(SSM_LATENCY / dt))
        self.hold = 0.0

    def update(self, dist):
        """输入真实距离，返回 (速度系数 0~1, 区域, 延迟后的测量距离)。"""
        self.buf.append(dist)
        d = self.buf[-self.delay - 1] if len(self.buf) > self.delay else self.buf[0]
        if not self.enabled:
            return 1.0, "OFF", d
        self.hold = SSM_HOLD if d < self.stop else max(0.0, self.hold - self.dt)
        if self.hold > 0:
            return 0.0, "STOP", d
        f = float(np.clip((d - self.stop) / (self.slow - self.stop), 0, 1))
        return f, ("SLOW" if f < 1 else "CLEAR"), d


ZONE_COLOR = {"CLEAR": (90, 220, 110), "SLOW": (255, 190, 40), "STOP": (240, 70, 60), "OFF": (200, 200, 200)}


def add_zones(scene, sim, stop=SSM_STOP, slow=SSM_SLOW):
    """地面上画出减速圈 / 停止圈。"""
    x, y = sim.data.qpos[0], sim.data.qpos[1]
    z = sim.course.ground(x, y) + 0.003
    for r, rgba in ((slow, (1.0, 0.75, 0.15, 0.18)), (stop, (0.95, 0.25, 0.2, 0.28))):
        if scene.ngeom >= scene.maxgeom:
            break
        # 圆柱的 size = (半径, 半高, -)
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([r, 0.002, 0.0]),
                            np.array([x, y, z]), np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
        scene.ngeom += 1


def run_safety_trial(sim, person, args, mode="hold", video_path=None):
    import cv2

    hid = sim.model.body("human").mocapid[0]
    dt = sim.policy.step_dt
    cfg = SAFETY_CONFIGS[mode]
    mon = SafetyMonitor(dt, enabled=cfg["monitor"], stop=cfg["stop"], slow=cfg["slow"])
    anchor = None  # 位置保持的停止点
    hist = []  # 机器人水平位置历史，用来算 0.5 s 平均速度
    win = int(0.5 / dt)
    goal_x = sim.course.end_x - 1.0
    renderer = mujoco.Renderer(sim.model, args.height, args.width) if video_path else None
    sink = VideoSink(video_path, fps=round(1 / dt)) if video_path else None
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 5.0, 125.0, -22.0
    sim.reset()
    log = []
    status, t, min_d, t_stop = "timeout", 0.0, np.inf, 0.0
    contact_speed = 0.0  # 进入接触距离时机器人朝人方向的最大 0.5 s 平均速度
    contact_zone, robot_close_1s = "", 0.0  # 第一次接触时的区域；接触前 1 s 内机器人自己朝人走了多远
    creep = 0.0  # 处于 STOP 状态时离停止点的最大前移
    for k in range(int(args.time_limit / dt)):
        t = k * dt
        pp = person.pos(t)
        sim.data.mocap_pos[hid] = [pp[0], pp[1], sim.course.ground(pp[0], pp[1])]
        rxy = sim.data.qpos[:2].copy()
        hist.append(rxy)
        mon_zone_prev = log[-1][4] if log else "CLEAR"  # 上一步安全层的状态（本步指令由它决定）
        dist = float(np.linalg.norm(rxy - pp))
        min_d = min(min_d, dist)
        if dist < CONTACT_DIST and len(hist) > win:
            v_avg = (hist[-1] - hist[-1 - win]) / (win * dt)
            contact_speed = max(contact_speed, float(v_avg @ (pp - rxy) / max(dist, 1e-6)))
            if not contact_zone:
                contact_zone = mon_zone_prev
                back = min(len(hist) - 1, int(1.0 / dt))
                robot_close_1s = float((hist[-1] - hist[-1 - back]) @ (pp - rxy) / max(dist, 1e-6))
        factor, zone, d_meas = mon.update(dist)
        cmd = controller_v0(sim, t, vx=args.speed, settle=1.0)
        cmd[0] *= factor
        if zone == "STOP":
            if anchor is None:
                anchor = rxy
            along = float((rxy - anchor) @ sim.heading()[:2])
            creep = max(creep, along)
            if cfg["hold"]:
                cmd[0] = float(np.clip(-HOLD_GAIN * along, -HOLD_VMAX, HOLD_VMAX))
        else:
            anchor = None
        sim.step(cmd)
        t_stop += dt if zone == "STOP" else 0.0
        log.append((round(t, 3), float(sim.data.qpos[0]), dist, d_meas, zone, float(cmd[0])))
        if sim.fallen():
            status = "fell"
            break
        if sim.data.qpos[0] >= goal_x:
            status = "pass"
            break
        if renderer is not None:
            renderer.update_scene(sim.data, camera=cam)
            if cfg["monitor"]:
                add_zones(renderer.scene, sim, cfg["stop"], cfg["slow"])
            img = renderer.render().copy()
            font = cv2.FONT_HERSHEY_SIMPLEX
            title = {"off": "V0 without safety monitor", "stop": "V0 + monitor, stop at 0.8 m (zero velocity command)",
                     "hold": "V0 + monitor, stop at 0.8 m + position hold",
                     "iso": f"V0 + monitor, ISO/TS 15066 distance {ISO_S_P:.2f} m + position hold"}[mode]
            lines = [title, f"distance to person = {dist:4.2f} m   cmd vx = {cmd[0]:.2f} m/s   t = {t:4.1f} s"]
            for j, line in enumerate(lines):
                cv2.putText(img, line, (14, 32 + 30 * j), font, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
            label = {"CLEAR": "CLEAR - full speed", "SLOW": "PERSON NEAR - slowing down",
                     "STOP": "PROTECTIVE STOP", "OFF": "no monitoring"}[zone]
            if dist < CONTACT_DIST:
                label, color = "COLLISION", ZONE_COLOR["STOP"]
            else:
                color = ZONE_COLOR[zone]
            (tw, th), _ = cv2.getTextSize(label, font, 1.0, 3)
            cx, cy = (img.shape[1] - tw) // 2, int(img.shape[0] * 0.9)
            cv2.rectangle(img, (cx - 14, cy - th - 12), (cx + tw + 14, cy + 12), (0, 0, 0), -1)
            cv2.putText(img, label, (cx, cy), font, 1.0, color, 3, cv2.LINE_AA)
            sink.append(img)
    if renderer is not None:
        renderer.close()
        sink.hold(int(1.0 / dt))
        sink.close()
    return dict(status=status, t=t, min_dist=min_d, stop_time=t_stop, log=log, creep=creep,
                contact=bool(min_d < CONTACT_DIST), robot_hit=bool(contact_speed > MOVING), contact_speed=contact_speed,
                contact_zone=contact_zone, robot_close_1s=robot_close_1s)


def mode_safety(args):
    """演示 + 统计：同一批随机场景，带 / 不带安全监控各跑一遍。"""
    out = OUT_ROOT / args.controller / "safety"
    out.mkdir(parents=True, exist_ok=True)
    course = C.build(SAFETY_COURSE, start_z=0.05)  # 5 cm 高的通道，视频里看得出路线
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, "safety", human=True), course)

    # V0 以 0.5 m/s 指令实际约 0.49 m/s，出发约 2.3 s 后开始前进：到达 x 的时刻 ≈ 2.3 + x / 0.49
    def arrival(x):
        return 2.3 + x / 0.49

    # 演示：人在机器人进入减速圈前走到通道中间，停 7 s（比如在搬东西），然后离开
    demo = Person(x=5.0, t_start=arrival(3.0) - 3.0 / 0.8 - 0.5, speed=0.8, dwell=7.0)
    for mode in SAFETY_MODES:
        r = run_safety_trial(sim, demo, args, mode=mode, video_path=None if args.no_video else out / f"safety_demo_{mode}.mp4")
        print(f"演示 {mode:4s}：{r['status']}  最近距离 {r['min_dist']:.2f} m  停止 {r['stop_time']:.1f} s  STOP 中前移 {r['creep']:.2f} m  "
              f"用时 {r['t']:.1f} s  接触时朝人速度 {r['contact_speed']:.2f} m/s")
        with open(out / f"safety_demo_{mode}_log.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "robot_x", "dist_true", "dist_measured", "zone", "cmd_vx"])
            w.writerows(r["log"])

    print(f"ISO/TS 15066 保护间距 S_p = {ISO_S_P:.2f} m")
    rng = np.random.default_rng(args.seed)
    rows = []
    for i in range(args.trials):
        # 人到达通道中间的时刻围绕“机器人经过那里”的时刻随机：人要么已经站着，要么正好从前面横穿
        x_p, speed = rng.uniform(4.0, 7.0), rng.uniform(0.5, 1.5)
        t_center = arrival(x_p) + rng.uniform(-4.0, 1.0)
        person = Person(x=x_p, t_start=t_center - 3.0 / speed, speed=speed, dwell=rng.uniform(0.0, 4.0))
        for mode in SAFETY_MODES:
            r = run_safety_trial(sim, person, args, mode=mode)
            rows.append(dict(trial=i, mode=mode, person_x=round(person.x, 2), person_t=round(person.t_start, 2),
                             person_speed=round(person.speed, 2), person_dwell=round(person.dwell, 2),
                             status=r["status"], min_dist=round(r["min_dist"], 3), contact=r["contact"],
                             robot_moving_contact=r["robot_hit"], contact_speed=round(r["contact_speed"], 3),
                             contact_zone=r["contact_zone"], robot_close_1s=round(r["robot_close_1s"], 3),
                             creep_in_stop=round(r["creep"], 3), stop_time=round(r["stop_time"], 2), time=round(r["t"], 2)))
    with open(out / "safety_results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    summary = []
    for mode in SAFETY_MODES:
        rs = [r for r in rows if r["mode"] == mode]
        row = dict(mode=mode, trials=len(rs),
                   robot_moving_contact=sum(r["robot_moving_contact"] for r in rs),
                   person_into_stopped_robot=sum(r["contact"] and not r["robot_moving_contact"] for r in rs),
                   contact_while_driving=sum(r["contact"] and r["contact_zone"] != "STOP" for r in rs),
                   contact_during_stop=sum(r["contact"] and r["contact_zone"] == "STOP" for r in rs),
                   max_robot_close_1s_during_stop=max([r["robot_close_1s"] for r in rs if r["contact"] and r["contact_zone"] == "STOP"], default=0.0),
                   fell=sum(r["status"] == "fell" for r in rs), reached_goal=sum(r["status"] == "pass" for r in rs),
                   median_min_dist=float(np.median([r["min_dist"] for r in rs])),
                   p5_min_dist=float(np.percentile([r["min_dist"] for r in rs], 5)),
                   mean_creep_in_stop=float(np.mean([r["creep_in_stop"] for r in rs])),
                   mean_time=float(np.mean([r["time"] for r in rs])))
        row.update(stop_dist=SAFETY_CONFIGS[mode]["stop"], slow_dist=SAFETY_CONFIGS[mode]["slow"],
                   position_hold=SAFETY_CONFIGS[mode]["hold"])
        summary.append(row)
        print(f"{mode:4s}：{row['trials']} 次  机器人运动中撞人 {row['robot_moving_contact']}  人走向已停机器人 {row['person_into_stopped_robot']}  "
              f"行驶中接触 {row['contact_while_driving']}  保护停止中接触 {row['contact_during_stop']}（接触前 1 s 机器人最多前移 {row['max_robot_close_1s_during_stop']:.2f} m）  "
              f"摔倒 {row['fell']}  到达终点 {row['reached_goal']}  最近距离中位数 {row['median_min_dist']:.2f} m（5% 分位 {row['p5_min_dist']:.2f}）  "
              f"STOP 中平均前移 {row['mean_creep_in_stop']:.2f} m  平均用时 {row['mean_time']:.1f} s")
    with open(out / "safety_summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print(f"结果：{out}")


"""
模式。
"""

COURSE_TITLES = {
    "full": None,
    "3d": "G1 3D test course (F cross slope · G rough ground · H angled stairs · I narrow stairs 0.6 m)",
}


def course_specs(args):
    return C.COURSE_3D if args.course == "3d" else C.FULL_COURSE


def out_dir(args):
    out = OUT_ROOT / args.controller
    if args.course != "full":
        out = out / args.course
    out.mkdir(parents=True, exist_ok=True)
    return out


def mode_profile(args):
    out = out_dir(args)
    course = C.build_named(args.course)
    C.plot_profile(course, out / "course_profile.png", COURSE_TITLES[args.course])
    print(f"路线全长 {course.end_x:.1f} m，剖面图：{out / 'course_profile.png'}")


def mode_full(args):
    out = out_dir(args)
    course = C.build_named(args.course)
    C.plot_profile(course, out / "course_profile.png", COURSE_TITLES[args.course])
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, args.course), course)
    t0 = time.time()
    video = None if args.no_video else out / "full_course.mp4"
    r = run_trial(sim, CONTROLLERS[args.controller], course.end_x - 0.5, time_limit=args.time_limit,
                  video_path=video, vx=args.speed, show_scan=args.show_scan, label=CONTROLLER_LABELS[args.controller])
    seg = course.segment_at(r["x"]) or "(end)"
    print(f"全程：{r['status']}  到达 x = {r['x']:.2f} m（{seg}），用时 {r['t']:.1f} s  [{time.time() - t0:.0f}s]")
    if video:
        print(f"视频：{video}")
    with open(out / "full_course_steps.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "foot", "step_length", "x", "y", "segment"])
        for e in r["events"]:
            w.writerow([*e, course.segment_at(e[3])])


def mode_segments(args):
    out = out_dir(args)
    policy = G1Policy(POLICY_DIR)
    rng = np.random.default_rng(args.seed)
    rows = []
    for spec in course_specs(args):
        name = spec.get("name")
        if not name or name == "finish":
            continue
        course = C.standalone(spec)
        sim = CourseSim(policy, C.write_scene(course, f"seg_{args.course}"), course)
        _, x0, x1, _ = course.segments[0]
        results = []
        for i in range(args.trials):
            # 起点随机偏移：落脚相位相对台阶边缘每次不同，才能看出可重复性
            sim.reset(x=rng.uniform(-0.15, 0.15), y=rng.uniform(-0.05, 0.05), yaw=rng.uniform(-0.05, 0.05))
            r = run_trial(sim, CONTROLLERS[args.controller], x1 + PASS_MARGIN, time_limit=args.time_limit, vx=args.speed)
            if spec["kind"] == "targets":
                r["mean_len"], r["len_err"] = stride_metrics(r["events"], course, name)
            results.append(r)
        n_pass = sum(r["status"] == "pass" for r in results)
        row = dict(
            segment=name,
            trials=args.trials,
            pass_rate=n_pass / args.trials,
            fell=sum(r["status"] == "fell" for r in results),
            off_track=sum(r["status"] == "off_track" for r in results),
            timeout=sum(r["status"] == "timeout" for r in results),
            mean_time_pass=float(np.mean([r["t"] for r in results if r["status"] == "pass"])) if n_pass else float("nan"),
            fail_x_rel=float(np.mean([r["x"] - x0 for r in results if r["status"] != "pass"])) if n_pass < args.trials else float("nan"),
            fail_progress_rel=float(np.mean([r["x_max"] - x0 for r in results if r["status"] != "pass"])) if n_pass < args.trials else float("nan"),
            max_lateral_dev=float(np.mean([r["y_dev"] for r in results])),
            stride_target=spec.get("spacing", float("nan")),
            stride_mean=float(np.nanmean([r.get("mean_len", np.nan) for r in results])) if spec["kind"] == "targets" else float("nan"),
            stride_err=float(np.nanmean([r.get("len_err", np.nan) for r in results])) if spec["kind"] == "targets" else float("nan"),
        )
        rows.append(row)
        extra = f"  步长 {row['stride_mean']:.3f} m（目标 {row['stride_target']:.2f}，误差 {row['stride_err']:.3f}）" if spec["kind"] == "targets" else ""
        fail = f"  最远到达≈段内 {row['fail_progress_rel']:.2f} m" if n_pass < args.trials else ""
        print(f"{name:16s} 通过 {n_pass:2d}/{args.trials}  摔倒 {row['fell']}  出界 {row['off_track']}  超时 {row['timeout']}"
              f"  最大横向偏移 {row['max_lateral_dev']:.2f} m{fail}{extra}")

    with open(out / "segment_results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    plot_pass_rates(rows, out / "segment_pass_rate.png", args.controller)
    print(f"结果：{out / 'segment_results.csv'}")


def plot_pass_rates(rows, path, label):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [r["segment"] for r in rows]
    rates = [100 * r["pass_rate"] for r in rows]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    bars = ax.bar(names, rates, color=["#4c78a8" if v >= 80 else "#e45756" for v in rates])
    ax.bar_label(bars, fmt="%.0f%%", fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("pass rate [%]")
    ax.set_title(f"{label.upper()}: pass rate per course segment ({rows[0]['trials']} trials each)")
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def mode_viewer(args):
    import mujoco.viewer

    course = C.build_named(args.course)
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, args.course), course)
    controller = CONTROLLERS[args.controller]
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = sim.pelvis
        viewer.cam.distance = 3.2
        t = 0.0
        while viewer.is_running():
            t0 = time.time()
            sim.step(controller(sim, t, vx=args.speed))
            t += sim.policy.step_dt
            if sim.fallen() or sim.data.qpos[0] > course.end_x - 0.5:
                print(f"{'摔倒' if sim.fallen() else '到达终点'}：x = {sim.data.qpos[0]:.2f} m（{course.segment_at(sim.data.qpos[0])}），复位")
                sim.reset()
                t = 0.0
            viewer.sync()
            time.sleep(max(0.0, sim.policy.step_dt - (time.time() - t0)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["profile", "full", "tour", "segments", "viewer", "scan", "safety"], default="full")
    p.add_argument("--controller", choices=list(CONTROLLERS), default="v0")
    p.add_argument("--course", choices=["full", "3d"], default="full", help="full = 原始 2D 路线；3d = 高度随 x、y 变化的路线")
    p.add_argument("--speed", type=float, default=0.5, help="V0 恒定前进速度 m/s")
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--time-limit", type=float, default=90.0)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--show-scan", action="store_true", help="视频里画出机器人前方的高度扫描点")
    p.add_argument("--stuck-time", type=float, default=8.0, help="巡回模式：这么多秒内前进 < 0.3 m 判为卡住")
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    args = p.parse_args()
    modes = {"profile": mode_profile, "full": mode_full, "tour": mode_tour, "segments": mode_segments, "viewer": mode_viewer,
             "scan": mode_scan, "safety": mode_safety}
    modes[args.mode](args)


if __name__ == "__main__":
    main()
