"""Test Unitree's flat-walking V0 policy against the existing G1 stair scenes.

The flat control disables the scene's stair/landing collisions but keeps the
same robot model, infinite ground plane, initial pose, policy, and controller.
Corridor centering uses MuJoCo ground-truth position and heading.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from run_stair_sweep import GOAL_UPPER_DISTANCE, STAIR_START_X, generate_scene


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "g1_step_playback"))
from play_g1 import G1Policy  # noqa: E402

POLICY_DIR = ROOT / "g1_step_playback/policy"
V0_ROBOT_MODEL = ROOT / "g1_step_playback/models/g1/g1_29dof.xml"
DEFAULT_SOURCE_ROOT = Path("/tmp/skf-g1-dwaq/TienKung-Lab")
FPS = 20
DT = 0.02


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rise-cm", type=int, default=15)
    parser.add_argument("--tread-cm", type=int, default=31)
    parser.add_argument("--width-cm", type=int, default=160)
    parser.add_argument("--start-y-cm", type=int, default=0)
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--flat-control", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--max-time-s", type=float, default=60.0)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/v0-stair-baseline")
    args = parser.parse_args()
    if not (6 <= args.rise_cm <= 25 and 20 <= args.tread_cm <= 45 and 70 <= args.width_cm <= 160):
        parser.error("geometry outside the existing stair-sweep range")
    if not (-0.5 <= args.speed <= 1.0):
        parser.error("speed outside V0 command range")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    policy = G1Policy(POLICY_DIR)
    reference_model = mujoco.MjModel.from_xml_path(str(V0_ROBOT_MODEL))
    torque_limit = reference_model.actuator_ctrlrange[:29, 1].copy()
    assert abs(policy.step_dt - DT) < 1e-9
    top_height = 0.0 if args.flat_control else args.rise_cm / 100 * 10
    goal_x = STAIR_START_X + 12 * args.tread_cm / 100 + GOAL_UPPER_DISTANCE
    mode = "flat_control" if args.flat_control else f"rise{args.rise_cm}cm_tread{args.tread_cm}cm_width{args.width_cm}cm"
    start = f"center" if args.start_y_cm == 0 else f"offset{args.start_y_cm:+d}cm"
    name = f"v0_{mode}_cmd{round(args.speed * 100)}cmps_{start}"
    temporary_video = output_dir / f".{name}.video-in-progress.mp4"

    with tempfile.TemporaryDirectory(prefix="skf-v0-stair-probe-") as temporary:
        scene_dir = Path(temporary)
        source_models = args.source_root / "legged_lab/assets/unitree/g1/mjcf"
        (scene_dir / "g1_29dof_rev_1_0_daf.xml").symlink_to(source_models / "g1_29dof_rev_1_0_daf.xml")
        (scene_dir / "meshes").symlink_to(source_models / "meshes", target_is_directory=True)
        scene_path = scene_dir / "scene.xml"
        generate_scene(args.rise_cm / 100, args.tread_cm / 100, args.width_cm / 100, scene_path)
        if args.flat_control:
            tree = ET.parse(scene_path)
            for geom in tree.findall(".//worldbody/geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
                geom.set("rgba", "0 0 0 0")
            tree.write(scene_path, encoding="unicode")

        model = mujoco.MjModel.from_xml_path(str(scene_path))
        model.opt.timestep = 0.005
        data = mujoco.MjData(model)
        model_joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j))
                        for j in model.actuator_trnid[:29, 0]]
        reference_joints = [mujoco.mj_id2name(reference_model, mujoco.mjtObj.mjOBJ_JOINT, int(j))
                            for j in reference_model.actuator_trnid[:29, 0]]
        if model_joints != reference_joints:
            raise RuntimeError("V0 and stair-scene actuator joint order differ")
        initial_joints = np.zeros(29)
        initial_joints[policy.ids_map] = policy.default_pos
        data.qpos[:7] = [0, args.start_y_cm / 100, 0.78, 1, 0, 0, 0]
        data.qpos[7:36] = initial_joints
        mujoco.mj_forward(model, data)
        policy.reset()

        renderer = None
        encoder = None
        camera = None
        last_frame = None
        next_frame_time = 0.0
        frames = 0
        if args.video:
            frame_width, frame_height = 960, 540
            model.vis.global_.offwidth = frame_width
            model.vis.global_.offheight = frame_height
            for geom_id in range(model.ngeom):
                geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                if "wall" in geom_name:
                    model.geom_rgba[geom_id, 3] = 0.08
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = 90
            camera.elevation = -22
            camera.distance = 4.8
            renderer = mujoco.Renderer(model, height=frame_height, width=frame_width)
            encoder = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
                 "-video_size", f"{frame_width}x{frame_height}", "-framerate", str(FPS), "-i", "pipe:0",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
                 "-movflags", "+faststart", str(temporary_video)],
                stdin=subprocess.PIPE,
            )

        def write_frame() -> None:
            nonlocal last_frame, frames
            if renderer is None or encoder is None or camera is None:
                return
            x, _, z = data.qpos[:3]
            camera.lookat[:] = [float(x) + 1.0, 0.0, max(1.0, float(z) + 0.35)]
            renderer.update_scene(data, camera=camera)
            last_frame = renderer.render().tobytes()
            assert encoder.stdin is not None
            encoder.stdin.write(last_frame)
            frames += 1

        trace: list[tuple[float, float, float, float]] = []
        outcome = "time_limit"
        max_x = 0.0
        max_z = 0.0
        try:
            write_frame()
            next_frame_time = 1 / FPS
            for _ in range(round(args.max_time_s / DT)):
                x, y, _ = map(float, data.qpos[:3])
                rotation = data.xmat[model.body("pelvis").id].reshape(3, 3)
                gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
                yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                command = policy.clip_command([
                    args.speed,
                    np.clip(-0.5 * y, -0.3, 0.3),
                    np.clip(-1.2 * yaw - 0.8 * y, -0.2, 0.2),
                ])
                target = policy.act(data.qpos[7:36], data.qvel[6:35], data.qvel[3:6], gravity, command)
                for _ in range(round(DT / model.opt.timestep)):
                    q = data.qpos[7:36]
                    dq = data.qvel[6:35]
                    data.ctrl[:29] = np.clip(policy.kp * (target - q) - policy.kd * dq,
                                            -torque_limit, torque_limit)
                    mujoco.mj_step(model, data)
                x, y, z = map(float, data.qpos[:3])
                max_x = max(max_x, x)
                max_z = max(max_z, z)
                trace.append((float(data.time), x, y, z))
                if data.time + 1e-9 >= next_frame_time:
                    write_frame()
                    next_frame_time += 1 / FPS
                if z < 0.32 or not np.isfinite(data.qpos).all():
                    outcome = "fall"
                    break
                if not args.flat_control and STAIR_START_X <= x <= goal_x and abs(y) > args.width_cm / 200 + 0.2:
                    outcome = "stair_exit"
                    break
                if x > goal_x and z > top_height + 0.4:
                    outcome = "goal"
                    break
            write_frame()
            if encoder is not None:
                assert encoder.stdin is not None and last_frame is not None
                for _ in range(FPS):
                    encoder.stdin.write(last_frame)
        finally:
            if renderer is not None:
                renderer.close()
            if encoder is not None:
                assert encoder.stdin is not None
                encoder.stdin.close()
                if encoder.wait() != 0:
                    raise RuntimeError("video encode failed")

        video_path = output_dir / f"{outcome}_{name}_sim-truth-centering.mp4"
        if args.video:
            temporary_video.replace(video_path)
        trajectory_path = output_dir / f"{name}.csv.gz"
        with gzip.open(trajectory_path, "wt", encoding="utf-8") as target:
            np.savetxt(target, trace, delimiter=",", header="t,x,y,z", comments="")
        summary = {
            "policy": "Unitree G1 29-DoF velocity V0 policy.onnx",
            "policy_sha256": sha256(POLICY_DIR / "policy.onnx"),
            "robot_mjcf": "G1DWAQ_Lab g1_29dof_rev_1_0_daf.xml",
            "rise_cm": args.rise_cm,
            "tread_cm": args.tread_cm,
            "width_cm": args.width_cm,
            "flat_control": args.flat_control,
            "initial_lateral_offset_cm": args.start_y_cm,
            "command_speed_mps": args.speed,
            "center_correction_uses_simulator_ground_truth": True,
            "camera_perception_used": False,
            "fall_height_threshold_m": 0.32,
            "outcome": outcome,
            "sim_time_s": float(data.time),
            "max_x_m": max_x,
            "max_pelvis_z_m": max_z,
            "goal_x_m": goal_x,
            "final_xyz_m": list(map(float, data.qpos[:3])),
            "trajectory": trajectory_path.name,
            "video": video_path.name if args.video else None,
            "video_frames": frames + FPS if args.video else None,
        }
        (output_dir / f"{name}.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
