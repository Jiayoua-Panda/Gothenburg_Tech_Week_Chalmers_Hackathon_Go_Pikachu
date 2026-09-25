"""Record the four G1-DWAQ stair evaluations as honest MuJoCo replays.

This is an artifact script for the local experiment, not a trained vision policy.
It expects the third-party G1DWAQ_Lab checkout; the custom scenes are included
in this repository. The third-party checkpoint is loaded with weights_only=True.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from unittest import mock

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch


CASES = {
    "success_15cm_center": {
        "scene": "skf-corridor-stairs-corridor.xml",
        "speed": 0.3,
        "center": True,
        "max_steps": 2200,
        "label": "15 cm 台阶 · 0.3 m/s · 仿真真值纠偏 · 无相机感知",
    },
    "failure_15cm_no_center_same_speed": {
        "scene": "skf-corridor-stairs-corridor.xml",
        "speed": 0.3,
        "center": False,
        "max_steps": 800,
        "stop_on_exit": True,
        "label": "15 cm 台阶 · 0.3 m/s · 无中心线纠偏 · 无相机感知",
    },
    "failure_15cm_no_center": {
        "scene": "skf-corridor-stairs-corridor.xml",
        "speed": 0.4,
        "center": False,
        "max_steps": 1200,
        "label": "15 cm 台阶 · 0.4 m/s · 无中心线纠偏 · 无相机感知",
    },
    "failure_20cm_center": {
        "scene": "skf-corridor-stairs-20cm-40cm.xml",
        "speed": 0.3,
        "center": True,
        "max_steps": 1200,
        "label": "20 cm 台阶 · 0.3 m/s · 仿真真值纠偏 · 无相机感知",
    },
}


def load_runner(path: Path):
    spec = importlib.util.spec_from_file_location("g1_dwaq_safe_sim2sim", path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("case", choices=CASES)
    parser.add_argument("--source-root", type=Path, default=Path("/tmp/skf-g1-dwaq/TienKung-Lab"))
    parser.add_argument("--runner-script", type=Path, help="Override the source checkout's sim2sim_g1_dwaq.py")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "artifacts/stair-replays")
    args = parser.parse_args()

    case = CASES[args.case]
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    source_root = args.source_root.expanduser().resolve()
    source_models = source_root / "legged_lab/assets/unitree/g1/mjcf"
    custom_scene = Path(__file__).resolve().parents[1] / "artifacts/stair-replays/scenes" / case["scene"]
    scene_temp = tempfile.TemporaryDirectory(prefix="skf-g1-dwaq-scene-")
    scene_dir = Path(scene_temp.name)
    (scene_dir / "g1_29dof_rev_1_0_daf.xml").symlink_to(source_models / "g1_29dof_rev_1_0_daf.xml")
    (scene_dir / "meshes").symlink_to(source_models / "meshes", target_is_directory=True)
    model_path = scene_dir / case["scene"]
    shutil.copy2(custom_scene, model_path)
    checkpoint_path = source_root / "logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt"
    runner_script = args.runner_script or source_root / "legged_lab/scripts/sim2sim_g1_dwaq.py"
    module = load_runner(runner_script)

    class SafeRunner(module.G1DwaqMujocoRunner):
        def load_policy(self, checkpoint_path: str) -> None:
            original_load = torch.load

            def weights_only_load(*load_args, **load_kwargs):
                load_kwargs["weights_only"] = True
                return original_load(*load_args, **load_kwargs)

            with mock.patch.object(torch, "load", weights_only_load):
                super().load_policy(checkpoint_path)

    runner = SafeRunner(module.G1DwaqSim2SimCfg(), str(checkpoint_path), str(model_path))
    runner.command_vel[:] = [case["speed"], 0, 0]
    initial = runner.normalize_obs(runner.get_current_obs())
    runner.obs_history[:] = initial

    width, height, fps = 960, 540, 20
    runner.model.vis.global_.offwidth = width
    runner.model.vis.global_.offheight = height
    # Visual-only transparency keeps the side walls from obscuring the robot.
    # Collision geometry and physics are unchanged.
    for geom_id in range(runner.model.ngeom):
        name = mujoco.mj_id2name(runner.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if "wall" in name:
            runner.model.geom_rgba[geom_id, 3] = 0.08

    video_path = output_dir / f"{args.case}.mp4"
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{width}x{height}", "-framerate", str(fps), "-i", "pipe:0",
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(video_path),
        ],
        stdin=subprocess.PIPE,
    )
    font_path = "/System/Library/Fonts/Hiragino Sans GB.ttc"
    font = ImageFont.truetype(font_path, 25)
    small_font = ImageFont.truetype(font_path, 19)
    end_font = ImageFont.truetype(font_path, 35)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 120 if args.case == "failure_15cm_no_center_same_speed" else 90
    camera.elevation = -22
    camera.distance = 4.8
    renderer = mujoco.Renderer(runner.model, height=height, width=width)
    frames = 0
    last_frame = None
    trace = []

    def write_frame(status: str | None = None) -> None:
        nonlocal frames, last_frame
        x, y, z = runner.data.qpos[:3]
        camera.lookat[:] = [float(x) + 1.0, 0.0, max(1.0, float(z) + 0.35)]
        renderer.update_scene(runner.data, camera=camera)
        picture = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(picture, "RGBA")
        draw.rectangle((0, 0, width, 80), fill=(0, 0, 0, 185))
        draw.text((20, 7), case["label"], font=font, fill=(255, 255, 255, 255))
        draw.text((20, 46), f"MuJoCo 回放  |  t={runner.data.time:.1f}s  x={x:.2f}m  y={y:.2f}m", font=small_font, fill=(218, 230, 238, 255))
        if status:
            draw.rectangle((0, height - 76, width, height), fill=(0, 0, 0, 200))
            draw.text((20, height - 66), status, font=end_font, fill=(255, 225, 108, 255))
        last_frame = picture.tobytes()
        assert ffmpeg.stdin is not None
        ffmpeg.stdin.write(last_frame)
        frames += 1

    status = "达到上层走廊" if args.case.startswith("success") else "未完成路线"
    reason = "time_limit"
    next_frame_time = 0.0
    try:
        write_frame()
        next_frame_time = 1 / fps
        for step in range(case["max_steps"]):
            if case["center"]:
                _, y0, _ = runner.data.qpos[:3]
                w, qx, qy, qz = runner.data.qpos[3:7]
                yaw = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                runner.command_vel[:] = [
                    case["speed"],
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
            if step % 250 == 0:
                print(f"{args.case}: t={runner.data.time:.2f} x={x:.3f} y={y:.3f} z={z:.3f}", flush=True)
            if z < 0.32 or not np.isfinite(runner.data.qpos).all():
                status, reason = "跌倒，路线失败", "fall"
                break
            if case.get("stop_on_exit") and abs(y) > 1.0:
                status, reason = "越出楼梯边缘，路线失败", "stair_exit"
                break
            if x > 8.0 and z > 1.9:
                status, reason = "达到上层走廊", "goal"
                break
        write_frame(status)
        assert ffmpeg.stdin is not None and last_frame is not None
        for _ in range(fps):
            ffmpeg.stdin.write(last_frame)
    finally:
        renderer.close()
        if ffmpeg.stdin:
            ffmpeg.stdin.close()
        code = ffmpeg.wait()
        if code:
            raise RuntimeError(f"ffmpeg exited with code {code}")

    np.savetxt(output_dir / f"{args.case}.csv", trace, delimiter=",", header="t,x,y,z", comments="")
    metadata = {
        "case": args.case,
        "scene": case["scene"],
        "policy": "third-party G1DWAQ_Lab model_9999.pt",
        "speed_command_mps": case["speed"],
        "center_correction_uses_simulator_ground_truth": case["center"],
        "camera_perception_used": False,
        "visual_only_wall_alpha": 0.08,
        "outcome": reason,
        "sim_time_s": float(runner.data.time),
        "final_xyz_m": list(map(float, runner.data.qpos[:3])),
        "video_fps": fps,
        "video_frames": frames + fps,
    }
    (output_dir / f"{args.case}.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    scene_temp.cleanup()
    print(json.dumps(metadata, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
