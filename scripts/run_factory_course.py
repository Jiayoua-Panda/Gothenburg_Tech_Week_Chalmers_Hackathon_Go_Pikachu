"""Walk a G1 through a multi-segment factory course with a waypoint route follower.

The low-level controller is the third-party G1-DWAQ blind stair policy (it never
sees the scene). A route follower turns the robot pose into velocity commands:
it tracks each corridor or stair centreline, slows before corners and stairs,
and turns on the spot at corner landings. It never turns on a stair flight.

The pose comes from a `Localizer`. With its default settings it returns the
MuJoCo ground-truth pose every control tick, standing in for an ideal external
camera tracking system. The noise, update-rate and latency options model a
degraded camera system; the policy still receives no image.
"""

from __future__ import annotations

import argparse
from collections import deque
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import types
from unittest import mock

import mujoco
import numpy as np
import torch

from factory_course import COURSES, Course, plot_course, scene_xml
from render_stair_replays import load_runner


ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT_RELATIVE = Path("logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt")
DT = 0.02
CORNER_TOLERANCE = 0.15  # start turning within 15 cm of the corner point; turn mode re-centres
MIN_WALK_SPEED = 0.3     # m/s; the policy often steps in place at 0.2 m/s
STALL_WINDOW_S = 4.0     # stall recovery: less than STALL_MIN_GAIN progress in this window ...
STALL_MIN_GAIN = 0.05    # ... metres triggers a short back-off
BACKOFF_S = 1.0
ALIGN_ZONE = 0.35         # align-before-stairs: within this distance of a flight's first riser ...
ALIGN_ENTER_RAD = 0.10    # ... stop and turn if heading is off by more than ~6°,
ALIGN_EXIT_RAD = 0.05     # until it is within ~3°,
ALIGN_MAX_TICKS = 150     # or give up after 3 s
GOAL_RADIUS = 0.3        # m; true pelvis position must be this close to the goal point
TURN_RATE = 1.0          # rad/s, top of the policy's training range; at 0.6 its right turns nearly stall
TURN_DONE_RAD = 0.15     # ~9°: hand over to the walking controller


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class Localizer:
    """Pose source for the route follower: ideal by default, optionally a degraded camera system.

    Camera model: each fix is the true pose `latency` seconds ago plus white noise (σ per axis)
    and a constant calibration bias (fixed magnitude, random direction per run), delivered at
    `rate_hz`. Without fusion the follower holds the last fix until the next one arrives.

    With `fusion`, the robot integrates its own odometry (planar velocity with a per-run scale
    error, gyro yaw rate with a constant bias) on top of the latest fix, including the interval
    the fix spent in transit, so it compensates both low rate and latency.
    """

    def __init__(self, sigma_xy: float = 0.0, sigma_yaw: float = 0.0, rate_hz: float = 0.0,
                 latency_s: float = 0.0, bias_xy: float = 0.0, fusion: bool = False, seed: int = 0,
                 odom_scale_error: float = 0.1, gyro_bias: float = 0.01):
        self.sigma_xy, self.sigma_yaw = sigma_xy, sigma_yaw
        self.period = 1 / rate_hz if rate_hz > 0 else 0.0
        self.latency = latency_s
        self.fusion = fusion
        self.rng = np.random.default_rng(seed)
        angle = self.rng.uniform(-math.pi, math.pi)
        self.bias = np.array([bias_xy * math.cos(angle), bias_xy * math.sin(angle), 0.0])
        self.odom_scale = 1 + odom_scale_error * self.rng.choice([-1.0, 1.0])
        self.gyro_bias = gyro_bias * self.rng.choice([-1.0, 1.0])
        self.history: deque[tuple[float, np.ndarray]] = deque()
        self.odometry: deque[tuple[float, np.ndarray]] = deque()
        self.estimate: np.ndarray | None = None
        self.next_update = 0.0

    @property
    def ideal(self) -> bool:
        return (self.sigma_xy == 0 and self.sigma_yaw == 0 and self.period == 0 and self.latency == 0
                and not self.bias.any())

    def update(self, t: float, true_pose: np.ndarray, world_velocity: np.ndarray, yaw_rate: float) -> np.ndarray:
        if self.ideal:
            return true_pose.copy()
        self.history.append((t, true_pose.copy()))
        while len(self.history) > 1 and self.history[1][0] <= t - self.latency:
            self.history.popleft()
        step = np.array([*(world_velocity[:2] * self.odom_scale * DT), (yaw_rate + self.gyro_bias) * DT])
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


class RouteFollower:
    def __init__(self, course: Course, flat_speed: float, stair_speed: float, recovery: bool = False,
                 align: bool = False):
        self.segments = course.segments
        self.flat_speed, self.stair_speed = flat_speed, stair_speed
        self.recovery = recovery
        self.align = align
        self.aligning = False
        self.align_ticks = 0
        self.aligned_segments: set[int] = set()
        self.alignments = 0
        self.recoveries = 0
        self.backoff_until = -1.0
        self.watch: deque[tuple[float, float]] = deque()
        self.index = 0
        self.mode = "follow"
        self.done = False
        self.offsets = np.cumsum([0.0] + [s.length for s in self.segments])

    def frame(self, pose: np.ndarray, index: int | None = None) -> tuple[float, float, float]:
        segment = self.segments[self.index if index is None else index]
        heading = math.radians(segment.heading_deg)
        dx, dy = pose[0] - segment.start[0], pose[1] - segment.start[1]
        along = dx * math.cos(heading) + dy * math.sin(heading)
        across = -dx * math.sin(heading) + dy * math.cos(heading)
        return along, across, wrap(pose[2] - heading)

    def turn_follows(self) -> bool:
        return (self.index + 1 < len(self.segments)
                and self.segments[self.index + 1].heading_deg != self.segments[self.index].heading_deg)

    def command(self, pose: np.ndarray) -> np.ndarray:
        segment = self.segments[self.index]
        if self.mode == "turn":
            target = math.radians(self.segments[self.index + 1].heading_deg)
            error = wrap(target - pose[2])
            px, py = segment.end
            dx, dy = px - pose[0], py - pose[1]
            ex = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
            ey = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
            if abs(error) < TURN_DONE_RAD:
                self.index += 1
                self.mode = "follow"
                return self.command(pose)
            # In-place turns at small or moderate yaw-rate commands stall (right turns at 0.6 rad/s
            # reach only ~0.06 rad/s), so always command the full rate; the walking controller
            # removes the last few degrees while moving.
            return np.array([np.clip(0.6 * ex, -0.2, 0.2), np.clip(0.6 * ey, -0.2, 0.2),
                             math.copysign(TURN_RATE, error)])

        along, across, heading_error = self.frame(pose)
        remaining = segment.length - along
        arrived = remaining <= (CORNER_TOLERANCE if self.turn_follows() else 0.0)
        if arrived:
            if self.index == len(self.segments) - 1:
                self.done = True
                return np.zeros(3)
            if self.turn_follows():
                self.mode = "turn"
            else:
                self.index += 1
            return self.command(pose)

        stairs_next = self.index + 1 < len(self.segments) and self.segments[self.index + 1].kind != "flat"
        if (self.align and stairs_next and remaining < ALIGN_ZONE and self.index not in self.aligned_segments):
            if not self.aligning and abs(heading_error) > ALIGN_ENTER_RAD:
                self.aligning, self.align_ticks = True, 0
                self.alignments += 1
            if self.aligning:
                self.align_ticks += 1
                if abs(heading_error) < ALIGN_EXIT_RAD or self.align_ticks > ALIGN_MAX_TICKS:
                    self.aligning = False
                    self.aligned_segments.add(self.index)
                else:
                    return np.array([0.0, np.clip(-0.5 * across, -0.25, 0.25),
                                     -math.copysign(TURN_RATE, heading_error)])
        near_stairs = (segment.kind != "flat"
                       or (self.index + 1 < len(self.segments) and self.segments[self.index + 1].kind != "flat"
                           and remaining < 1.0)
                       or (self.index > 0 and self.segments[self.index - 1].kind != "flat" and along < 0.5))
        speed = self.stair_speed if near_stairs else self.flat_speed
        if self.turn_follows():
            speed = min(speed, max(MIN_WALK_SPEED, 0.8 * remaining))
        return np.array([speed, np.clip(-0.5 * across, -0.25, 0.25),
                         np.clip(-1.2 * heading_error - 0.8 * across, -0.6, 0.6)])

    def supervise(self, t: float, pose: np.ndarray, command: np.ndarray) -> np.ndarray:
        """Stall recovery: if walking makes no progress, step back briefly and retry."""
        if not self.recovery or self.mode != "follow" or self.done:
            self.watch.clear()
            return command
        if t < self.backoff_until:
            return np.array([-0.2, command[1], command[2]])
        progress = self.progress(pose)
        self.watch.append((t, progress))
        while self.watch and self.watch[0][0] < t - STALL_WINDOW_S:
            self.watch.popleft()
        if t - self.watch[0][0] >= STALL_WINDOW_S - 0.05 and progress - self.watch[0][1] < STALL_MIN_GAIN:
            self.recoveries += 1
            self.backoff_until = t + BACKOFF_S
            self.watch.clear()
            return np.array([-0.2, command[1], command[2]])
        return command

    def progress(self, pose: np.ndarray) -> float:
        along, _, _ = self.frame(pose)
        segment = self.segments[self.index]
        return float(self.offsets[self.index] + np.clip(along, 0.0, segment.length))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--course", choices=COURSES, required=True)
    parser.add_argument("--start-y-cm", type=float, default=0.0, help="lateral start offset (left positive)")
    parser.add_argument("--start-yaw-deg", type=float, default=0.0)
    parser.add_argument("--flat-speed", type=float, default=0.5)
    parser.add_argument("--stair-speed", type=float, default=0.3)
    parser.add_argument("--recovery", action="store_true", help="back off and retry when progress stalls")
    parser.add_argument("--align", action="store_true", help="stop and square up before entering a stair flight")
    parser.add_argument("--loc-sigma-cm", type=float, default=0.0, help="camera position noise (1σ)")
    parser.add_argument("--loc-yaw-sigma-deg", type=float, default=0.0, help="camera heading noise (1σ)")
    parser.add_argument("--loc-rate-hz", type=float, default=0.0, help="camera update rate; 0 = every tick")
    parser.add_argument("--loc-latency-ms", type=float, default=0.0)
    parser.add_argument("--loc-bias-cm", type=float, default=0.0, help="constant calibration offset (random direction)")
    parser.add_argument("--loc-fusion", action="store_true", help="fuse camera fixes with robot odometry")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-time-s", type=float, default=None)
    parser.add_argument("--stuck-s", type=float, default=25.0, help="stop if route progress stalls this long")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--plot", action="store_true", help="also write a top-down map with this run's path")
    parser.add_argument("--tag", default="", help="extra run-name suffix")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/factory-course/runs")
    parser.add_argument("--source-root", type=Path, default=Path("/tmp/skf-g1-dwaq/TienKung-Lab"))
    args = parser.parse_args()

    course = COURSES[args.course]()
    max_time = args.max_time_s or 3 * course.total_length / args.stair_speed + 30
    localizer = Localizer(args.loc_sigma_cm / 100, math.radians(args.loc_yaw_sigma_deg), args.loc_rate_hz,
                          args.loc_latency_ms / 1000, args.loc_bias_cm / 100, args.loc_fusion, args.seed)
    loc_label = "ideal" if localizer.ideal else (
        f"s{args.loc_sigma_cm:g}cm_y{args.loc_yaw_sigma_deg:g}deg_{args.loc_rate_hz:g}hz_"
        f"{args.loc_latency_ms:g}ms_b{args.loc_bias_cm:g}cm{'_fused' if args.loc_fusion else ''}_seed{args.seed}")
    control_label = (f"v{round(args.stair_speed * 100)}" + ("_rec" if args.recovery else "")
                     + ("_align" if args.align else ""))
    run_name = (f"{course.name}_{control_label}_y{args.start_y_cm:+g}cm_yaw{args.start_yaw_deg:+g}_"
                f"{loc_label}{args.tag}")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    source_root = args.source_root.expanduser().resolve()
    source_models = source_root / "legged_lab/assets/unitree/g1/mjcf"
    checkpoint = source_root / CHECKPOINT_RELATIVE
    # The runner imports pynput for its interactive keyboard, which needs an X display; it is unused here.
    sys.modules.setdefault("pynput", types.SimpleNamespace(keyboard=None))
    module = load_runner(source_root / "legged_lab/scripts/sim2sim_g1_dwaq.py")

    class SafeRunner(module.G1DwaqMujocoRunner):
        def load_policy(self, checkpoint_path: str) -> None:
            original_load = torch.load

            def weights_only_load(*load_args, **load_kwargs):
                load_kwargs["weights_only"] = True
                return original_load(*load_args, **load_kwargs)

            with mock.patch.object(torch, "load", weights_only_load):
                super().load_policy(checkpoint_path)

    with tempfile.TemporaryDirectory(prefix="skf-factory-course-") as temp_dir:
        temp = Path(temp_dir)
        (temp / "g1_29dof_rev_1_0_daf.xml").symlink_to(source_models / "g1_29dof_rev_1_0_daf.xml")
        (temp / "meshes").symlink_to(source_models / "meshes", target_is_directory=True)
        scene_path = temp / f"{course.name}.xml"
        scene_path.write_text(scene_xml(course))
        runner = SafeRunner(module.G1DwaqSim2SimCfg(), str(checkpoint), str(scene_path))

        yaw0 = math.radians(args.start_yaw_deg)
        runner.data.qpos[1] = args.start_y_cm / 100
        runner.data.qpos[2] += course.segments[0].z_start
        runner.data.qpos[3:7] = [math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)]
        mujoco.mj_forward(runner.model, runner.data)
        runner.command_vel[:] = [0.0, 0.0, 0.0]
        runner.obs_history[:] = runner.normalize_obs(runner.get_current_obs())

        follower = RouteFollower(course, args.flat_speed, args.stair_speed, args.recovery, args.align)
        fps, frame_width, frame_height = 20, 960, 540
        video_path = output_dir / f"{run_name}.mp4"
        ffmpeg = renderer = camera = None
        frames = 0
        last_frame = None
        if args.video:
            renderer = mujoco.Renderer(runner.model, height=frame_height, width=frame_width)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth, camera.elevation, camera.distance = 45, -35, 5.5
            ffmpeg = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
                 "-video_size", f"{frame_width}x{frame_height}", "-framerate", str(fps), "-i", "pipe:0",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "22", "-pix_fmt", "yuv420p",
                 "-movflags", "+faststart", str(video_path)],
                stdin=subprocess.PIPE,
            )

        def write_frame() -> None:
            nonlocal frames, last_frame
            if renderer is None:
                return
            x, y, z = runner.data.qpos[:3]
            camera.lookat[:] = [float(x), float(y), float(z) - 0.1]
            renderer.update_scene(runner.data, camera=camera)
            last_frame = renderer.render().tobytes()
            ffmpeg.stdin.write(last_frame)
            frames += 1

        def true_pose() -> np.ndarray:
            w, qx, qy, qz = runner.data.qpos[3:7]
            yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return np.array([runner.data.qpos[0], runner.data.qpos[1], yaw])

        trace = []
        outcome, detail = "time_limit", ""
        best_progress, best_progress_time = 0.0, 0.0
        settle_ticks = 25  # stand still for 0.5 s so the policy's history is consistent
        start_wall = time.monotonic()
        next_frame_time = 0.0
        try:
            for tick in range(round(max_time / DT)):
                t = float(runner.data.time)
                truth = true_pose()
                estimate = localizer.update(t, truth, runner.data.qvel[0:3], float(runner.data.qvel[5]))
                command = np.zeros(3) if tick < settle_ticks else follower.supervise(
                    t, estimate, follower.command(estimate))
                if follower.done:
                    # the follower stops on its *estimated* pose; judge arrival on the true pose
                    goal_error = math.hypot(truth[0] - course.goal[0], truth[1] - course.goal[1])
                    outcome = "goal" if goal_error <= GOAL_RADIUS else "wrong_stop"
                    detail = f"stopped {100 * goal_error:.0f} cm from the goal"
                    break
                runner.command_vel[:] = command
                current = runner.normalize_obs(runner.get_current_obs())
                runner.update_obs_history(current)
                with torch.inference_mode():
                    action = runner.policy.act_inference(
                        torch.from_numpy(current).unsqueeze(0),
                        torch.from_numpy(runner.get_flattened_obs_history()).unsqueeze(0),
                    )
                runner.action[:] = np.clip(action.squeeze(0).numpy(), -runner.cfg.sim.clip_actions,
                                           runner.cfg.sim.clip_actions)
                for _ in range(runner.cfg.sim.decimation):
                    runner.data.ctrl[:runner.num_actions] = runner.pd_control(runner.position_control())
                    mujoco.mj_step(runner.model, runner.data)
                runner.gait_phase_time += DT

                if runner.data.time + 1e-9 >= next_frame_time:
                    write_frame()
                    next_frame_time += 1 / fps

                truth = true_pose()
                z = float(runner.data.qpos[2])
                gravity_z = float(runner.get_gravity_orientation(runner.data.qpos[3:7])[2])
                segment = follower.segments[follower.index]
                along, across, _ = follower.frame(truth)
                surface = segment.surface_z(along) if follower.mode == "follow" else segment.z_end
                progress = follower.progress(truth) if follower.mode == "follow" else float(
                    follower.offsets[follower.index + 1])
                trace.append([round(float(runner.data.time), 3), *np.round(truth, 4).tolist(), round(z, 4),
                              *np.round(estimate, 4).tolist(), follower.index, follower.mode,
                              *np.round(command, 3).tolist(), round(progress, 3)])

                if not np.isfinite(runner.data.qpos).all() or gravity_z > -0.6 or z - surface < 0.4:
                    outcome, detail = "fall", f"segment {follower.index} ({segment.kind})"
                    break
                if follower.mode == "follow":
                    lateral = abs(across)
                else:
                    # turning on the square corner landing centred on the segment end
                    lateral = max(abs(truth[0] - segment.end[0]), abs(truth[1] - segment.end[1]))
                if lateral > course.width / 2:
                    outcome, detail = "path_exit", f"segment {follower.index} ({segment.kind}), {follower.mode}"
                    break
                if progress > best_progress + 0.1:
                    best_progress, best_progress_time = progress, t
                elif t - best_progress_time > args.stuck_s and tick > settle_ticks:
                    outcome, detail = "stuck", f"segment {follower.index} ({segment.kind}), {follower.mode}"
                    break
            write_frame()
            if ffmpeg is not None and last_frame is not None:
                for _ in range(fps):
                    ffmpeg.stdin.write(last_frame)
        finally:
            if renderer is not None:
                renderer.close()
            if ffmpeg is not None:
                ffmpeg.stdin.close()
                if ffmpeg.wait() != 0:
                    raise RuntimeError(f"ffmpeg failed for {run_name}")

    columns = ["t", "x", "y", "yaw", "z", "est_x", "est_y", "est_yaw", "segment", "mode",
               "cmd_vx", "cmd_vy", "cmd_wz", "progress_m"]
    with gzip.open(output_dir / f"{run_name}.csv.gz", "wt", encoding="utf-8") as handle:
        handle.write(",".join(columns) + "\n")
        for row in trace:
            handle.write(",".join(str(v) for v in row) + "\n")
    follow_rows = [row for row in trace if row[9] == "follow"]
    errors = []
    for row in follow_rows:
        _, across, _ = follower.frame(np.array(row[1:4]), index=row[8])
        errors.append(abs(across))
    final_progress = trace[-1][-1] if trace else 0.0
    colour = {"goal": "#22a352", "fall": "#d9412b", "path_exit": "#e08a1e"}.get(outcome, "#7a4fd1")
    if args.plot:
        plot_course(course, output_dir / f"{run_name}.png",
                    [(f"pelvis ({outcome})", [(row[1], row[2]) for row in trace], colour)])
    summary = {
        "run": run_name,
        "course": course.summary() | {"segments": None},
        "outcome": outcome,
        "detail": detail,
        "sim_time_s": round(float(runner.data.time), 2),
        "wall_time_s": round(time.monotonic() - start_wall, 1),
        "route_progress_m": round(final_progress, 3),
        "route_completion": round(final_progress / course.total_length, 4),
        "final_goal_distance_cm": round(100 * math.hypot(runner.data.qpos[0] - course.goal[0],
                                                         runner.data.qpos[1] - course.goal[1]), 1),
        "mean_abs_cross_track_cm": round(100 * float(np.mean(errors)), 2) if errors else None,
        "max_abs_cross_track_cm": round(100 * float(np.max(errors)), 2) if errors else None,
        "final_xyz_m": [round(float(v), 3) for v in runner.data.qpos[:3]],
        "start_offset_cm": args.start_y_cm,
        "start_yaw_deg": args.start_yaw_deg,
        "flat_speed_mps": args.flat_speed,
        "stair_speed_mps": args.stair_speed,
        "stall_recovery": args.recovery,
        "recoveries": follower.recoveries,
        "align_before_stairs": args.align,
        "alignments": follower.alignments,
        "localization": {
            "source": "MuJoCo ground truth" + ("" if localizer.ideal else " + simulated camera error"),
            "sigma_xy_cm": args.loc_sigma_cm, "sigma_yaw_deg": args.loc_yaw_sigma_deg,
            "rate_hz": args.loc_rate_hz or 1 / DT, "latency_ms": args.loc_latency_ms,
            "bias_cm": args.loc_bias_cm, "odometry_fusion": args.loc_fusion and not localizer.ideal,
            "seed": args.seed,
        },
        "camera_images_used": False,
        "policy": "G1DWAQ_Lab model_9999.pt (third-party, blind)",
        "checkpoint_sha256": sha256(checkpoint),
        "code_sha256": {name: sha256(Path(__file__).with_name(name))[:12]
                        for name in ("run_factory_course.py", "factory_course.py")},
        "video": video_path.name if args.video else None,
        "trace": f"{run_name}.csv.gz",
        "trace_columns": columns,
    }
    summary["course"].pop("segments")
    (output_dir / f"{run_name}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: summary[k] for k in ("run", "outcome", "detail", "sim_time_s", "wall_time_s",
                                             "route_completion", "max_abs_cross_track_cm")}), flush=True)


if __name__ == "__main__":
    main()
