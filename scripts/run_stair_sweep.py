"""Evaluate one parameterized corridor/stair/corridor scene with a fixed G1 policy.

The controller optionally uses MuJoCo ground-truth position and yaw for
centerline correction. No camera observation reaches the policy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import time
from unittest import mock
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

from render_stair_replays import load_runner


ROOT = Path(__file__).resolve().parents[1]
BASE_SCENE = ROOT / "artifacts/stair-replays/scenes/skf-corridor-stairs-corridor.xml"
N_STEPS = 10
STAIR_START_X = 1.155
UPPER_LENGTH = 7.0
GOAL_UPPER_DISTANCE = 3.13


def name_for(rise_cm: int, tread_cm: int, width_cm: int, start_y_cm: int) -> str:
    lateral = "y0" if start_y_cm == 0 else f"y{'p' if start_y_cm > 0 else 'm'}{abs(start_y_cm)}"
    return f"r{rise_cm:02d}_t{tread_cm:02d}_w{width_cm:03d}_{lateral}"


def generate_scene(rise: float, tread: float, width: float, scene_path: Path) -> dict:
    """Use the earlier 15/31 scene topology and change its three geometry values."""
    tree = ET.parse(BASE_SCENE)
    geoms = {geom.attrib["name"]: geom for geom in tree.findall(".//worldbody/geom")}
    top = N_STEPS * rise
    stair_end = STAIR_START_X + N_STEPS * tread
    landing_end = stair_end + 2 * tread
    upper_center = landing_end + UPPER_LENGTH / 2

    def set_box(name: str, size: tuple[float, float, float], pos: tuple[float, float, float]) -> None:
        geom = geoms[name]
        geom.set("size", " ".join(f"{v:.6f}" for v in size))
        geom.set("pos", " ".join(f"{v:.6f}" for v in pos))

    for index in range(1, N_STEPS + 1):
        height = index * rise
        x = STAIR_START_X + (index - 0.5) * tread
        set_box(f"stair_up_{index}", (tread / 2, width / 2, height / 2), (x, 0, height / 2))
    set_box("platform", (tread, width / 2, top / 2), (stair_end + tread, 0, top / 2))
    set_box("upper_corridor", (UPPER_LENGTH / 2, 0.8, top / 2), (upper_center, 0, top / 2))
    for side, y in (("left", 0.9), ("right", -0.9)):
        set_box(f"upper_wall_{side}", (UPPER_LENGTH / 2, 0.05, 0.6), (upper_center, y, top + 0.7))
    scene_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(scene_path, encoding="unicode")
    return {
        "stair_start_x_m": STAIR_START_X,
        "stair_end_x_m": stair_end,
        "landing_end_x_m": landing_end,
        "upper_corridor_start_x_m": landing_end,
        "goal_x_m": landing_end + GOAL_UPPER_DISTANCE,
        "top_height_m": top,
        "number_of_steps": N_STEPS,
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rise-cm", type=int, required=True)
    parser.add_argument("--tread-cm", type=int, required=True)
    parser.add_argument("--width-cm", type=int, required=True)
    parser.add_argument("--start-y-cm", type=int, default=0)
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--no-center", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--max-steps", type=int, default=3000)
    parser.add_argument("--source-root", type=Path, default=Path("/tmp/skf-g1-dwaq/TienKung-Lab"))
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/stair-sweep")
    args = parser.parse_args()
    if not (6 <= args.rise_cm <= 25 and 20 <= args.tread_cm <= 45 and 70 <= args.width_cm <= 160):
        parser.error("geometry outside supported sweep range")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    case_id = name_for(args.rise_cm, args.tread_cm, args.width_cm, args.start_y_cm)
    if args.no_center:
        case_id += "_no_center"
    rise, tread, width = args.rise_cm / 100, args.tread_cm / 100, args.width_cm / 100
    scene_path = output_dir / "scenes" / f"r{args.rise_cm:02d}_t{args.tread_cm:02d}_w{args.width_cm:03d}.xml"
    geometry = generate_scene(rise, tread, width, scene_path)

    source_root = args.source_root.expanduser().resolve()
    source_commit = subprocess.check_output(
        ["git", "-C", str(source_root), "rev-parse", "--short", "HEAD"], text=True
    ).strip()
    source_models = source_root / "legged_lab/assets/unitree/g1/mjcf"
    checkpoint = source_root / "logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt"
    module = load_runner(source_root / "legged_lab/scripts/sim2sim_g1_dwaq.py")

    class SafeRunner(module.G1DwaqMujocoRunner):
        def load_policy(self, checkpoint_path: str) -> None:
            original_load = torch.load

            def weights_only_load(*load_args, **load_kwargs):
                load_kwargs["weights_only"] = True
                return original_load(*load_args, **load_kwargs)

            with mock.patch.object(torch, "load", weights_only_load):
                super().load_policy(checkpoint_path)

    with tempfile.TemporaryDirectory(prefix="skf-stair-sweep-") as temp_dir:
        temp = Path(temp_dir)
        (temp / "g1_29dof_rev_1_0_daf.xml").symlink_to(source_models / "g1_29dof_rev_1_0_daf.xml")
        (temp / "meshes").symlink_to(source_models / "meshes", target_is_directory=True)
        model_path = temp / scene_path.name
        model_path.write_bytes(scene_path.read_bytes())
        runner = SafeRunner(module.G1DwaqSim2SimCfg(), str(checkpoint), str(model_path))
        runner.data.qpos[1] = args.start_y_cm / 100
        mujoco.mj_forward(runner.model, runner.data)
        runner.command_vel[:] = [args.speed, 0, 0]
        initial = runner.normalize_obs(runner.get_current_obs())
        runner.obs_history[:] = initial

        fps = 20
        frame_width, frame_height = 960, 540
        temporary_video_path = output_dir / f".{case_id}.video-in-progress.mp4"
        ffmpeg = None
        renderer = None
        camera = None
        frames = 0
        last_frame = None
        if args.video:
            runner.model.vis.global_.offwidth = frame_width
            runner.model.vis.global_.offheight = frame_height
            for geom_id in range(runner.model.ngeom):
                geom_name = mujoco.mj_id2name(runner.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                if "wall" in geom_name:
                    runner.model.geom_rgba[geom_id, 3] = 0.08  # visual only
            ffmpeg = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
                 "-video_size", f"{frame_width}x{frame_height}", "-framerate", str(fps), "-i", "pipe:0",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
                 "-movflags", "+faststart", str(temporary_video_path)],
                stdin=subprocess.PIPE,
            )
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = 90
            camera.elevation = -22
            camera.distance = 4.8
            renderer = mujoco.Renderer(runner.model, height=frame_height, width=frame_width)

        def write_frame() -> None:
            nonlocal frames, last_frame
            if renderer is None or ffmpeg is None or camera is None:
                return
            x, y, z = runner.data.qpos[:3]
            camera.lookat[:] = [float(x) + 1.0, 0.0, max(1.0, float(z) + 0.35)]
            renderer.update_scene(runner.data, camera=camera)
            last_frame = renderer.render().tobytes()
            assert ffmpeg.stdin is not None
            ffmpeg.stdin.write(last_frame)
            frames += 1

        trace = []
        outcome = "time_limit"
        next_frame_time = 0.0
        start_wall = time.monotonic()
        try:
            write_frame()
            next_frame_time = 1 / fps
            for _ in range(args.max_steps):
                if not args.no_center:
                    _, y0, _ = runner.data.qpos[:3]
                    w, qx, qy, qz = runner.data.qpos[3:7]
                    yaw = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                    runner.command_vel[:] = [
                        args.speed,
                        np.clip(-0.5 * y0, -0.25, 0.25),
                        np.clip(-1.2 * yaw - 0.8 * y0, -0.6, 0.6),
                    ]
                current = runner.normalize_obs(runner.get_current_obs())
                runner.update_obs_history(current)
                with torch.inference_mode():
                    action = runner.policy.act_inference(
                        torch.from_numpy(current).unsqueeze(0),
                        torch.from_numpy(runner.get_flattened_obs_history()).unsqueeze(0),
                    )
                runner.action[:] = np.clip(action.squeeze(0).numpy(), -runner.cfg.sim.clip_actions, runner.cfg.sim.clip_actions)
                for _ in range(runner.cfg.sim.decimation):
                    runner.data.ctrl[:runner.num_actions] = runner.pd_control(runner.position_control())
                    mujoco.mj_step(runner.model, runner.data)
                runner.gait_phase_time += runner.cfg.sim.dt * runner.cfg.sim.decimation
                x, y, z = map(float, runner.data.qpos[:3])
                trace.append((float(runner.data.time), x, y, z))
                if runner.data.time + 1e-9 >= next_frame_time:
                    write_frame()
                    next_frame_time += 1 / fps
                if z < 0.32 or not np.isfinite(runner.data.qpos).all():
                    outcome = "fall"
                    break
                if STAIR_START_X <= x <= geometry["landing_end_x_m"] and abs(y) > width / 2 + 0.2:
                    outcome = "stair_exit"
                    break
                if x > geometry["goal_x_m"] and z > geometry["top_height_m"] + 0.4:
                    outcome = "goal"
                    break
            write_frame()
            if ffmpeg is not None:
                assert ffmpeg.stdin is not None and last_frame is not None
                for _ in range(fps):
                    ffmpeg.stdin.write(last_frame)
        finally:
            if renderer is not None:
                renderer.close()
            if ffmpeg is not None:
                assert ffmpeg.stdin is not None
                ffmpeg.stdin.close()
                if ffmpeg.wait() != 0:
                    raise RuntimeError(f"ffmpeg failed for {case_id}")

        outcome_label = {"goal": "success", "fall": "fall", "stair_exit": "stair-exit",
                         "time_limit": "time-limit"}[outcome]
        control_label = "no-centering" if args.no_center else "sim-truth-centering"
        video_path = output_dir / (
            f"{outcome_label}_rise{args.rise_cm}cm_tread{args.tread_cm}cm_"
            f"width{args.width_cm}cm_offset{args.start_y_cm:+d}cm_{control_label}.mp4"
        )
        if args.video:
            temporary_video_path.replace(video_path)

        trace_path = output_dir / f"{case_id}.csv.gz"
        np.savetxt(trace_path, trace, delimiter=",", header="t,x,y,z", comments="")
        metadata = {
            "case": case_id,
            "rise_cm": args.rise_cm,
            "tread_cm": args.tread_cm,
            "usable_width_cm": args.width_cm,
            "initial_lateral_offset_cm": args.start_y_cm,
            "speed_command_mps": args.speed,
            "center_correction_uses_simulator_ground_truth": not args.no_center,
            "camera_perception_used": False,
            "policy": "G1DWAQ_Lab model_9999.pt",
            "source_commit": source_commit,
            "checkpoint_sha256": sha256(checkpoint),
            "scene": str(scene_path.relative_to(output_dir)),
            "scene_sha256": sha256(scene_path),
            "trajectory": trace_path.name,
            **geometry,
            "outcome": outcome,
            "sim_time_s": float(runner.data.time),
            "wall_time_s": time.monotonic() - start_wall,
            "final_xyz_m": list(map(float, runner.data.qpos[:3])),
            "max_x_m": float(max(point[1] for point in trace)),
            "max_z_m": float(max(point[3] for point in trace)),
            "video": video_path.name if args.video else None,
            "video_fps": fps if args.video else None,
            "video_frames": frames + fps if args.video else None,
        }
        (output_dir / f"{case_id}.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(metadata, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
