"""在测试路线上评测 G1 策略（V0 = Unitree 预训练策略，恒定速度指令）。

用法：
    py run_course.py --mode profile            # 只画路线剖面图
    py run_course.py --mode full               # 全程跑一次，录视频（第一次失败就停）
    py run_course.py --mode tour               # 巡回：失败后从下一段重新出发，一段视频看完所有路段
    py run_course.py --mode segments --trials 10   # 每段单独测 N 次，统计通过率
    py run_course.py --mode viewer             # 交互窗口看全程
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
from play_g1 import FOOT_BODIES, G1Policy, G1Sim, StepTracker, add_footprints, save_video  # noqa: E402

POLICY_DIR = ROOT.parent / "g1_step_playback" / "policy"
OUT_ROOT = ROOT / "outputs"


class CourseSim(G1Sim):
    """G1Sim + 自定义场景、出生点、按脚下地面高度判摔倒、任意地形接触算落地。"""

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
        super().reset()
        self.data.qpos[0], self.data.qpos[1] = x, y
        self.data.qpos[2] = 0.78 + self.course.ground(x)
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.model, self.data)

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
        x = self.data.qpos[0]
        rel_h = self.data.qpos[2] - self.course.ground(x)
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


CONTROLLERS = {"v0": controller_v0}


"""
一次试验。
"""


def run_trial(sim, controller, goal_x, time_limit, record=False, width=960, height=540, vx=0.5):
    tracker = StepTracker(sim)
    renderer = mujoco.Renderer(sim.model, height, width) if record else None
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 3.2, 90.0, -10.0

    dt = sim.policy.step_dt
    frames, status, t = [], "timeout", 0.0
    for k in range(int(time_limit / dt)):
        t = k * dt
        cmd = controller(sim, t, vx=vx)
        sim.step(cmd)
        tracker.update(t, dt, record=True)
        x = sim.data.qpos[0]
        if sim.fallen():
            status = "fell"
            break
        if x >= goal_x:
            status = "pass"
            break
        if renderer is not None:
            renderer.update_scene(sim.data, camera=cam)
            add_footprints(renderer.scene, tracker.footprints)
            img = renderer.render().copy()
            seg = sim.course.segment_at(x) or "-"
            lines = [f"V0 pretrained   segment: {seg}", f"x = {x:5.2f} m   cmd vx = {cmd[0]:.2f} m/s   t = {t:4.1f} s"]
            import cv2

            for j, s in enumerate(lines):
                cv2.putText(img, s, (14, 32 + 30 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            frames.append(img)
    if renderer is not None:
        renderer.close()
        # 摔倒 / 结束后多留 1 s 画面
        frames += frames[-1:] * int(1.0 / dt)
    return dict(status=status, t=t, x=float(sim.data.qpos[0]), events=tracker.events, frames=frames)


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
STATUS_COLOR = {"PASS": (90, 220, 110), "FELL": (240, 70, 60), "STUCK": (255, 170, 40), "...": (200, 200, 200)}


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
    out = OUT_ROOT / args.controller
    out.mkdir(parents=True, exist_ok=True)
    course = C.build(C.FULL_COURSE)
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, "full"), course)
    controller = CONTROLLERS[args.controller]
    segs = [s for s in course.segments if s[0] != "finish"]
    status = {s[0]: "..." for s in segs}
    label = f"{args.controller.upper()} pretrained" if args.controller == "v0" else args.controller.upper()

    renderer = None if args.no_video else mujoco.Renderer(sim.model, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 3.4, 90.0, -10.0

    dt = sim.policy.step_dt
    frames, idx, t_total = [], 0, 0.0
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
        img = renderer.render().copy()
        x = sim.data.qpos[0]
        board = [(s[0], status[s[0]]) for s in segs]
        draw_overlay(img, label, course.segment_at(x) or "-", x, cmd, t_total, board, banner)
        frames.append(img)

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

        fell = sim.fallen()
        stuck = t_local > 3.0 and len(hist_x) > stuck_window and x - hist_x[-stuck_window] < 0.3
        if fell or stuck:
            result = "FELL" if fell else "STUCK"
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
    if frames:
        path = out / "course_tour.mp4"
        save_video(path, frames, fps=round(1 / dt))
        print(f"视频：{path}")


"""
模式。
"""


def mode_profile(args):
    out = OUT_ROOT / args.controller
    out.mkdir(parents=True, exist_ok=True)
    course = C.build(C.FULL_COURSE)
    C.plot_profile(course, out / "course_profile.png")
    print(f"路线全长 {course.end_x:.1f} m，剖面图：{out / 'course_profile.png'}")


def mode_full(args):
    out = OUT_ROOT / args.controller
    out.mkdir(parents=True, exist_ok=True)
    course = C.build(C.FULL_COURSE)
    C.plot_profile(course, out / "course_profile.png")
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, "full"), course)
    t0 = time.time()
    r = run_trial(sim, CONTROLLERS[args.controller], course.end_x - 0.5, time_limit=args.time_limit,
                  record=not args.no_video, vx=args.speed)
    seg = course.segment_at(r["x"]) or "(end)"
    print(f"全程：{r['status']}  到达 x = {r['x']:.2f} m（{seg}），用时 {r['t']:.1f} s  [{time.time() - t0:.0f}s]")
    if r["frames"]:
        save_video(out / "full_course.mp4", r["frames"], fps=round(1 / sim.policy.step_dt))
        print(f"视频：{out / 'full_course.mp4'}")
    with open(out / "full_course_steps.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "foot", "step_length", "x", "y", "segment"])
        for e in r["events"]:
            w.writerow([*e, course.segment_at(e[3])])


def mode_segments(args):
    out = OUT_ROOT / args.controller
    out.mkdir(parents=True, exist_ok=True)
    policy = G1Policy(POLICY_DIR)
    rng = np.random.default_rng(args.seed)
    rows = []
    for spec in C.FULL_COURSE:
        name = spec.get("name")
        if not name or name == "finish":
            continue
        course = C.standalone(spec)
        sim = CourseSim(policy, C.write_scene(course, "seg"), course)
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
            timeout=sum(r["status"] == "timeout" for r in results),
            mean_time_pass=float(np.mean([r["t"] for r in results if r["status"] == "pass"])) if n_pass else float("nan"),
            fail_x_rel=float(np.mean([r["x"] - x0 for r in results if r["status"] != "pass"])) if n_pass < args.trials else float("nan"),
            stride_target=spec.get("spacing", float("nan")),
            stride_mean=float(np.nanmean([r.get("mean_len", np.nan) for r in results])) if spec["kind"] == "targets" else float("nan"),
            stride_err=float(np.nanmean([r.get("len_err", np.nan) for r in results])) if spec["kind"] == "targets" else float("nan"),
        )
        rows.append(row)
        extra = f"  步长 {row['stride_mean']:.3f} m（目标 {row['stride_target']:.2f}，误差 {row['stride_err']:.3f}）" if spec["kind"] == "targets" else ""
        fail = f"  失败点≈段内 {row['fail_x_rel']:.2f} m" if n_pass < args.trials else ""
        print(f"{name:16s} 通过 {n_pass:2d}/{args.trials}  摔倒 {row['fell']}  超时 {row['timeout']}{fail}{extra}")

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

    course = C.build(C.FULL_COURSE)
    sim = CourseSim(G1Policy(POLICY_DIR), C.write_scene(course, "full"), course)
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
    p.add_argument("--mode", choices=["profile", "full", "tour", "segments", "viewer"], default="full")
    p.add_argument("--controller", choices=list(CONTROLLERS), default="v0")
    p.add_argument("--speed", type=float, default=0.5, help="V0 恒定前进速度 m/s")
    p.add_argument("--trials", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--time-limit", type=float, default=90.0)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--stuck-time", type=float, default=8.0, help="巡回模式：这么多秒内前进 < 0.3 m 判为卡住")
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    args = p.parse_args()
    modes = {"profile": mode_profile, "full": mode_full, "tour": mode_tour, "segments": mode_segments, "viewer": mode_viewer}
    modes[args.mode](args)


if __name__ == "__main__":
    main()
