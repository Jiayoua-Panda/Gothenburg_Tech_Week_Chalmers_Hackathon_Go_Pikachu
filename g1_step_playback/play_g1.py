"""G1 预训练行走策略的 MuJoCo 回放（方案①：不训练，只回放）。

用 unitree_rl_lab 自带的 G1-29dof 速度跟踪策略 (policy/policy.onnx)，
在 MuJoCo 里给不同的前进速度指令。策略的步频基本固定（实测约 3 步/s），
所以步长 ≈ 速度 / 步频：低速 = 小步，高速 = 大步。

用法：
    py play_g1.py                      # 默认 0.3/0.6/0.9 m/s 三段，录视频 + 统计步长
    py play_g1.py --speeds 0.2 1.0     # 自定义速度
    py play_g1.py --viewer             # 交互窗口，方向键调速度
"""

import argparse
import csv
import pathlib
import time
from collections import deque

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = pathlib.Path(__file__).resolve().parent
SCENE_XML = ROOT / "models" / "g1" / "scene_29dof.xml"
POLICY_DIR = ROOT / "policy"
OUT_DIR = ROOT / "outputs"

SIM_DT = 0.005  # 与 Isaac Lab 训练一致：sim.dt = 0.005, decimation = 4 → 策略 50 Hz
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")


class G1Policy:
    """按 deploy.yaml 复现 Isaac Lab 的观测拼接和动作处理（与 unitree 的 C++ 部署代码一致）。"""

    def __init__(self, policy_dir):
        cfg = yaml.safe_load(open(policy_dir / "deploy.yaml", encoding="utf-8"))
        self.session = ort.InferenceSession(str(policy_dir / "policy.onnx"))
        self.step_dt = cfg["step_dt"]
        # joint_ids_map[i] = 策略第 i 个关节在 SDK/MuJoCo 顺序中的下标
        self.ids_map = np.array(cfg["joint_ids_map"], dtype=int)
        self.kp = np.array(cfg["stiffness"])  # SDK 顺序
        self.kd = np.array(cfg["damping"])  # SDK 顺序
        self.default_pos = np.array(cfg["default_joint_pos"])  # 策略顺序
        act = cfg["actions"]["JointPositionAction"]
        self.action_scale = np.array(act["scale"])
        self.action_offset = np.array(act["offset"])
        self.obs_cfg = cfg["observations"]  # 顺序即拼接顺序
        self.cmd_ranges = cfg["commands"]["base_velocity"]["ranges"]
        self.history = None
        self.last_action = np.zeros(len(self.ids_map))

    def clip_command(self, cmd):
        r = self.cmd_ranges
        return np.array([
            np.clip(cmd[0], *r["lin_vel_x"]),
            np.clip(cmd[1], *r["lin_vel_y"]),
            np.clip(cmd[2], *r["ang_vel_z"]),
        ])

    def reset(self):
        self.history = None
        self.last_action[:] = 0.0

    def _terms(self, q_pol, dq_pol, ang_vel_b, gravity_b, cmd):
        return {
            "base_ang_vel": ang_vel_b,
            "projected_gravity": gravity_b,
            "velocity_commands": cmd,
            "joint_pos_rel": q_pol - self.default_pos,
            "joint_vel_rel": dq_pol,
            "last_action": self.last_action,
        }

    def act(self, q_sdk, dq_sdk, ang_vel_b, gravity_b, cmd):
        """输入 SDK 顺序的关节状态，返回 SDK 顺序的目标关节角。"""
        q_pol, dq_pol = q_sdk[self.ids_map], dq_sdk[self.ids_map]
        terms = self._terms(q_pol, dq_pol, ang_vel_b, gravity_b, cmd)
        if self.history is None:  # 复位时用第一帧填满历史（Isaac Lab 同样行为）
            self.history = {k: deque(maxlen=c["history_length"]) for k, c in self.obs_cfg.items()}
        for name, c in self.obs_cfg.items():
            v = np.asarray(terms[name], dtype=np.float64) * np.array(c["scale"])
            buf = self.history[name]
            if not buf:
                buf.extend([v] * c["history_length"])
            else:
                buf.append(v)
        # 每一项按"旧→新"展开，再按项拼接：5 × (3+3+3+29+29+29) = 480
        obs = np.concatenate([np.concatenate(self.history[n]) for n in self.obs_cfg])
        action = self.session.run(None, {"obs": obs[None].astype(np.float32)})[0][0]
        self.last_action = action.astype(np.float64)
        target_pol = action * self.action_scale + self.action_offset
        target_sdk = np.zeros_like(target_pol)
        target_sdk[self.ids_map] = target_pol
        return target_sdk


class G1Sim:
    def __init__(self, policy):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
        self.model.opt.timestep = SIM_DT
        self.data = mujoco.MjData(self.model)
        self.policy = policy
        self.decimation = round(policy.step_dt / SIM_DT)

        # 执行器顺序 = SDK 顺序；记录每个执行器对应关节的 qpos/qvel 地址
        jnt = self.model.actuator_trnid[:, 0]
        self.qadr = self.model.jnt_qposadr[jnt]
        self.vadr = self.model.jnt_dofadr[jnt]
        self.tau_lim = self.model.actuator_ctrlrange[:, 1]

        self.pelvis = self.model.body("pelvis").id
        self.floor = self.model.geom("floor").id
        self.foot_ids = [self.model.body(n).id for n in FOOT_BODIES]
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        default_sdk = np.zeros(len(self.qadr))
        default_sdk[self.policy.ids_map] = self.policy.default_pos
        self.data.qpos[2] = 0.78
        self.data.qpos[self.qadr] = default_sdk
        mujoco.mj_forward(self.model, self.data)
        self.policy.reset()
        self.target = default_sdk

    def base_state(self):
        R = self.data.xmat[self.pelvis].reshape(3, 3)  # body → world
        gravity_b = R.T @ np.array([0.0, 0.0, -1.0])
        ang_vel_b = self.data.qvel[3:6].copy()  # 自由关节角速度在机体系下，对应 IMU 陀螺仪
        return ang_vel_b, gravity_b

    def heading(self):
        fwd = self.data.xmat[self.pelvis].reshape(3, 3)[:, 0]
        fwd = np.array([fwd[0], fwd[1], 0.0])
        return fwd / (np.linalg.norm(fwd) + 1e-9)

    def feet_in_contact(self):
        contact = [False, False]
        for c in self.data.contact[: self.data.ncon]:
            for g_foot, g_other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if g_other == self.floor:
                    body = self.model.geom_bodyid[g_foot]
                    if body in self.foot_ids:
                        contact[self.foot_ids.index(body)] = True
        return contact

    def fallen(self):
        _, g = self.base_state()
        return self.data.qpos[2] < 0.45 or g[2] > -0.6

    def step(self, cmd):
        """一个策略步 (20 ms)：策略推理一次，PD 力矩跑 decimation 个物理步。"""
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.vadr]
        ang_vel_b, gravity_b = self.base_state()
        self.target = self.policy.act(q, dq, ang_vel_b, gravity_b, cmd)
        for _ in range(self.decimation):
            q = self.data.qpos[self.qadr]
            dq = self.data.qvel[self.vadr]
            tau = self.policy.kp * (self.target - q) - self.policy.kd * dq
            self.data.ctrl[:] = np.clip(tau, -self.tau_lim, self.tau_lim)
            mujoco.mj_step(self.model, self.data)


class StepTracker:
    """脚落地时记录：步长 = 落地脚与另一只（支撑）脚沿前进方向的距离。"""

    def __init__(self, sim):
        self.sim = sim
        self.prev = [True, True]
        self.air_time = [0.0, 0.0]
        self.events = []  # (t, foot, step_length, x, y)
        self.footprints = []  # (x, y, foot)

    def update(self, t, dt, record):
        contact = self.sim.feet_in_contact()
        for i in range(2):
            if not contact[i]:
                self.air_time[i] += dt
            # 腾空至少 0.1 s 后再落地才算一步，过滤接触抖动
            if contact[i] and not self.prev[i] and self.air_time[i] > 0.1:
                p = self.sim.data.xpos[self.sim.foot_ids[i]]
                other = self.sim.data.xpos[self.sim.foot_ids[1 - i]]
                length = float(np.dot(p - other, self.sim.heading()))
                if record:
                    self.events.append((t, FOOT_BODIES[i].split("_")[0], length, p[0], p[1]))
                self.footprints.append((p[0], p[1], i))
            if contact[i]:
                self.air_time[i] = 0.0
        self.prev = contact


def add_footprints(scene, footprints):
    colors = (np.array([0.2, 0.5, 1.0, 1.0]), np.array([1.0, 0.3, 0.2, 1.0]))  # 左蓝 右红
    for x, y, foot in footprints[-40:]:
        if scene.ngeom >= scene.maxgeom:
            break
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER,
            np.array([0.03, 0.03, 0.002]), np.array([x, y, 0.002]),
            np.eye(3).flatten(), colors[foot].astype(np.float32),
        )
        scene.ngeom += 1


def command_at(t, vx, settle=2.0, ramp=1.0):
    """先站立 settle 秒，再在 ramp 秒内线性加速到 vx。"""
    if t < settle:
        return 0.0
    return vx * min(1.0, (t - settle) / ramp)


def run_episode(sim, vx, duration, width, height, record_video, measure_after=4.0):
    import cv2

    sim.reset()
    tracker = StepTracker(sim)
    renderer = mujoco.Renderer(sim.model, height, width) if record_video else None
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.pelvis
    cam.distance, cam.azimuth, cam.elevation = 3.0, 90.0, -12.0  # 侧视，最能看出步长

    frames, dt = [], sim.policy.step_dt
    x0, t_meas0, fell = None, None, False
    n_steps = int(duration / dt)
    for k in range(n_steps):
        t = k * dt
        cmd = sim.policy.clip_command([command_at(t, vx), 0.0, 0.0])
        sim.step(cmd)
        measuring = t >= measure_after
        if measuring and x0 is None:
            x0, t_meas0 = sim.data.qpos[:2].copy(), t
        tracker.update(t, dt, record=measuring)
        if sim.fallen():
            fell = True
            break
        if renderer is not None:
            renderer.update_scene(sim.data, camera=cam)
            add_footprints(renderer.scene, tracker.footprints)
            img = renderer.render().copy()
            lengths = [e[2] for e in tracker.events]
            text = f"cmd vx = {cmd[0]:.2f} m/s   t = {t:4.1f} s"
            text2 = f"mean step length = {np.mean(lengths):.3f} m" if lengths else "mean step length = -"
            for j, s in enumerate((text, text2)):
                cv2.putText(img, s, (12, 28 + 26 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            frames.append(img)

    if renderer is not None:
        renderer.close()
    t_end = (k + 1) * dt
    dist = float(np.dot(sim.data.qpos[:2] - x0, sim.heading()[:2])) if x0 is not None else float("nan")
    lengths = np.array([e[2] for e in tracker.events])
    summary = {
        "cmd_vx": vx,
        "actual_vx": dist / (t_end - t_meas0) if x0 is not None and t_end > t_meas0 else float("nan"),
        "mean_step_length": float(lengths.mean()) if len(lengths) else float("nan"),
        "std_step_length": float(lengths.std()) if len(lengths) else float("nan"),
        "cadence_steps_per_s": len(lengths) / (t_end - t_meas0) if x0 is not None else float("nan"),
        "n_steps": int(len(lengths)),
        "fell": fell,
    }
    return summary, tracker.events, frames


def save_video(path, frames, fps):
    import imageio.v2 as imageio

    with imageio.get_writer(path, fps=fps, codec="libx264", quality=8, macro_block_size=1) as w:
        for f in frames:
            w.append_data(f)


def side_by_side(frame_lists):
    n = min(len(f) for f in frame_lists)
    return [np.concatenate([f[i] for f in frame_lists], axis=1) for i in range(n)]


def plot_summary(rows, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    v = np.array([r["cmd_vx"] for r in rows])
    L = np.array([r["mean_step_length"] for r in rows])
    s = np.array([r["std_step_length"] for r in rows])
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.errorbar(v, L, yerr=s, fmt="o-", capsize=4, label="measured (MuJoCo)")
    va = np.array([r["actual_vx"] for r in rows])
    f = np.nanmean([r["cadence_steps_per_s"] for r in rows])
    vv = np.linspace(0, max(v.max(), 0.1), 50)
    ax.plot(vv, vv / f, "--", color="gray", label=f"v / cadence  (cadence ≈ {f:.1f} steps/s)")
    ax.plot(v, va / f, "x", color="gray", label="actual speed / cadence")
    ax.set_xlabel("commanded forward speed [m/s]")
    ax.set_ylabel("step length [m]")
    ax.set_title("G1 pretrained policy: step length vs speed")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run_batch(args):
    OUT_DIR.mkdir(exist_ok=True)
    policy = G1Policy(POLICY_DIR)
    sim = G1Sim(policy)
    rows, all_frames = [], []
    with open(OUT_DIR / "step_events.csv", "w", newline="", encoding="utf-8") as fe:
        ev_writer = csv.writer(fe)
        ev_writer.writerow(["cmd_vx", "t", "foot", "step_length", "x", "y"])
        for vx in args.speeds:
            t0 = time.time()
            summary, events, frames = run_episode(
                sim, vx, args.duration, args.width, args.height, record_video=not args.no_video
            )
            rows.append(summary)
            for e in events:
                ev_writer.writerow([vx, *e])
            if frames:
                save_video(OUT_DIR / f"g1_vx_{vx:.2f}.mp4", frames, fps=round(1 / policy.step_dt))
                all_frames.append(frames)
            flag = "  [摔倒]" if summary["fell"] else ""
            print(
                f"vx={vx:.2f}  实际速度={summary['actual_vx']:.3f} m/s  "
                f"步长={summary['mean_step_length']:.3f}±{summary['std_step_length']:.3f} m  "
                f"步频={summary['cadence_steps_per_s']:.2f} 步/s  ({time.time() - t0:.0f}s){flag}"
            )

    with open(OUT_DIR / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    plot_summary(rows, OUT_DIR / "step_length_vs_speed.png")
    if len(all_frames) > 1:
        save_video(OUT_DIR / "comparison_side_by_side.mp4", side_by_side(all_frames), fps=round(1 / policy.step_dt))
    print(f"输出已保存到 {OUT_DIR}")


def run_viewer(args):
    import mujoco.viewer

    policy = G1Policy(POLICY_DIR)
    sim = G1Sim(policy)
    cmd = np.zeros(3)

    def on_key(key):
        # GLFW 键码：↑265 ↓264 ←263 →262，R=82 复位，空格=32 停下
        if key == 265:
            cmd[0] += 0.1
        elif key == 264:
            cmd[0] -= 0.1
        elif key == 263:
            cmd[2] += 0.1
        elif key == 262:
            cmd[2] -= 0.1
        elif key == 32:
            cmd[:] = 0.0
        elif key == 82:
            cmd[:] = 0.0
            sim.reset()
        cmd[:] = policy.clip_command(cmd)
        print(f"cmd: vx={cmd[0]:.1f} m/s  wz={cmd[2]:.1f} rad/s")

    print("↑/↓ 前进速度  ←/→ 转向  空格 停下  R 复位")
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = sim.pelvis
        viewer.cam.distance = 3.0
        while viewer.is_running():
            t0 = time.time()
            sim.step(cmd.copy())
            if sim.fallen():
                print("摔倒，自动复位")
                sim.reset()
            viewer.sync()
            time.sleep(max(0.0, policy.step_dt - (time.time() - t0)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speeds", type=float, nargs="+", default=[0.3, 0.6, 0.9], help="前进速度指令 m/s（策略范围 -0.5~1.0）")
    p.add_argument("--duration", type=float, default=12.0, help="每段时长 s（含 2 s 站立 + 1 s 加速）")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--no-video", action="store_true", help="只统计，不录视频")
    p.add_argument("--viewer", action="store_true", help="打开交互窗口")
    args = p.parse_args()
    run_viewer(args) if args.viewer else run_batch(args)


if __name__ == "__main__":
    main()
