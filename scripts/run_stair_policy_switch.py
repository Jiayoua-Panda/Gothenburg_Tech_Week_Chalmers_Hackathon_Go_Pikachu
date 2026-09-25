"""Pilot a G1-DWAQ stair policy to Unitree V0 flat-walking policy handoff.

The handoff is driven by MuJoCo ground-truth x/y/heading. This is a simulation
experiment, not a visual-navigation or real-robot controller.
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
from unittest import mock

import mujoco
import numpy as np
import torch

from render_stair_replays import load_runner


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "g1_step_playback"))
from play_g1 import G1Policy  # noqa: E402

DEFAULT_SOURCE_ROOT = Path("/tmp/skf-g1-dwaq/TienKung-Lab")
CHECKPOINT_RELATIVE = Path("logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt")
SCENE = ROOT / "artifacts/stair-sweep/scenes/r15_t31_w160.xml"
FLAT_POLICY = ROOT / "g1_step_playback/policy"
FLAT_MODEL = ROOT / "g1_step_playback/models/g1/g1_29dof.xml"
UPPER_START_X = 4.875
TOP_HEIGHT = 1.5
SWITCH_X = UPPER_START_X + 0.8
GOAL_X = UPPER_START_X + 5.6
DT = 0.02


def load_dwaq_runner(scene_path: Path, start_y_cm: int, source_root: Path):
    module = load_runner(source_root / "legged_lab/scripts/sim2sim_g1_dwaq.py")

    class SafeRunner(module.G1DwaqMujocoRunner):
        def load_policy(self, checkpoint_path: str) -> None:
            original_load = torch.load

            def weights_only_load(*load_args, **load_kwargs):
                load_kwargs["weights_only"] = True
                return original_load(*load_args, **load_kwargs)

            with mock.patch.object(torch, "load", weights_only_load):
                super().load_policy(checkpoint_path)

    runner = SafeRunner(module.G1DwaqSim2SimCfg(), str(source_root / CHECKPOINT_RELATIVE), str(scene_path))
    runner.data.qpos[1] = start_y_cm / 100
    mujoco.mj_forward(runner.model, runner.data)
    runner.command_vel[:] = [0.3, 0, 0]
    initial = runner.normalize_obs(runner.get_current_obs())
    runner.obs_history[:] = initial
    return runner


def joint_names(model: mujoco.MjModel) -> list[str]:
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j))
            for j in model.actuator_trnid[:, 0]]


def base_state(runner) -> tuple[np.ndarray, np.ndarray, float]:
    body_id = runner.model.body("pelvis").id
    rotation = runner.data.xmat[body_id].reshape(3, 3)
    gravity_body = rotation.T @ np.array([0.0, 0.0, -1.0])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    angular_velocity_body = runner.data.qvel[3:6].copy()
    return angular_velocity_body, gravity_body, float(yaw)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("dwaq", "dwaq-fast", "hybrid"), required=True)
    parser.add_argument("--flat-speed", type=float, default=0.6)
    parser.add_argument("--blend-s", type=float, default=1.5)
    parser.add_argument("--start-y-cm", type=int, default=0)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--max-time-s", type=float, default=75)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/stair-policy-switch")
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    args = parser.parse_args()
    if not (0.3 <= args.flat_speed <= 1.0):
        parser.error("flat-speed must stay in the V0 policy's 0.3–1.0 m/s command range")

    flat_policy = G1Policy(FLAT_POLICY)
    flat_model = mujoco.MjModel.from_xml_path(str(FLAT_MODEL))
    torque_limit = flat_model.actuator_ctrlrange[:, 1].copy()
    with tempfile.TemporaryDirectory(prefix="skf-policy-switch-") as temporary:
        scene_dir = Path(temporary)
        source_models = args.source_root / "legged_lab/assets/unitree/g1/mjcf"
        (scene_dir / "g1_29dof_rev_1_0_daf.xml").symlink_to(source_models / "g1_29dof_rev_1_0_daf.xml")
        (scene_dir / "meshes").symlink_to(source_models / "meshes", target_is_directory=True)
        local_scene = scene_dir / SCENE.name
        local_scene.write_bytes(SCENE.read_bytes())
        runner = load_dwaq_runner(local_scene, args.start_y_cm, args.source_root)
        if joint_names(flat_model) != joint_names(runner.model):
            raise RuntimeError("G1 policies do not control joints in the same order")
        if abs(flat_policy.step_dt - DT) > 1e-9:
            raise RuntimeError("Policy control rates differ")

        phase = "stairs"
        stable_ticks = 0
        warmup_ticks = 0
        blend_started = None
        switch_started = None
        trace: list[dict] = []
        outcome = "time_limit"
        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        lateral_label = "center" if args.start_y_cm == 0 else (
            f"left{abs(args.start_y_cm)}cm" if args.start_y_cm < 0 else f"right{args.start_y_cm}cm"
        )
        commanded_speed = 0.3 if args.mode == "dwaq" else args.flat_speed
        run_name = f"{args.mode}_cmd{round(commanded_speed * 100)}cmps_{lateral_label}"
        video_path = output_dir / f"{run_name}_rise15cm_tread31cm_width160cm.mp4"
        ffmpeg = None
        renderer = None
        camera = None
        frame_count = 0
        next_frame_time = 0.0
        last_frame = None
        if args.video:
            width, height, fps = 960, 540, 20
            runner.model.vis.global_.offwidth = width
            runner.model.vis.global_.offheight = height
            for geom_id in range(runner.model.ngeom):
                geom_name = mujoco.mj_id2name(runner.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
                if "wall" in geom_name:
                    runner.model.geom_rgba[geom_id, 3] = 0.08
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = 90
            camera.elevation = -22
            camera.distance = 4.8
            renderer = mujoco.Renderer(runner.model, height=height, width=width)
            ffmpeg = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
                 "-video_size", f"{width}x{height}", "-framerate", str(fps), "-i", "pipe:0",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
                 "-movflags", "+faststart", str(video_path)],
                stdin=subprocess.PIPE,
            )

        def write_frame() -> None:
            nonlocal frame_count, last_frame
            if renderer is None or ffmpeg is None or camera is None:
                return
            x, _, z = runner.data.qpos[:3]
            camera.lookat[:] = [float(x) + 1.0, 0.0, max(1.0, float(z) + 0.35)]
            renderer.update_scene(runner.data, camera=camera)
            last_frame = renderer.render().tobytes()
            assert ffmpeg.stdin is not None
            ffmpeg.stdin.write(last_frame)
            frame_count += 1

        write_frame()
        next_frame_time = 1 / 20
        for _ in range(round(args.max_time_s / DT)):
            x, y, z = map(float, runner.data.qpos[:3])
            angular_velocity, gravity, yaw = base_state(runner)
            if x > SWITCH_X and z > TOP_HEIGHT + 0.45 and abs(y) < 0.35 and gravity[2] < -0.75:
                stable_ticks += 1
            else:
                stable_ticks = 0
            if phase == "stairs" and stable_ticks >= 20 and args.mode != "dwaq":
                phase = "warmup" if args.mode == "hybrid" else "fast"
                switch_started = float(runner.data.time)
                flat_policy.reset()

            dwaq_speed = args.flat_speed if phase == "fast" else 0.3
            runner.command_vel[:] = [
                dwaq_speed,
                np.clip(-0.5 * y, -0.25, 0.25),
                np.clip(-1.2 * yaw - 0.8 * y, -0.6, 0.6),
            ]
            dwaq_target = None
            if phase != "flat":
                observation = runner.normalize_obs(runner.get_current_obs())
                runner.update_obs_history(observation)
                with torch.inference_mode():
                    action = runner.policy.act_inference(
                        torch.from_numpy(observation).unsqueeze(0),
                        torch.from_numpy(runner.get_flattened_obs_history()).unsqueeze(0),
                    )
                runner.action[:] = np.clip(
                    action.squeeze(0).numpy(), -runner.cfg.sim.clip_actions, runner.cfg.sim.clip_actions
                )
                dwaq_target = runner.position_control()

            flat_target = None
            if args.mode == "hybrid" and phase in ("warmup", "blend", "flat"):
                if phase == "warmup":
                    command_speed = 0.3
                elif phase == "blend":
                    command_speed = 0.3 + (args.flat_speed - 0.3) * min(
                        1.0, (runner.data.time - blend_started) / args.blend_s
                    )
                else:
                    command_speed = args.flat_speed
                flat_command = flat_policy.clip_command([
                    command_speed,
                    np.clip(-0.5 * y, -0.3, 0.3),
                    np.clip(-1.2 * yaw - 0.8 * y, -0.2, 0.2),
                ])
                q = runner.data.qpos[7:36].copy()
                dq = runner.data.qvel[6:35].copy()
                flat_target = flat_policy.act(q, dq, angular_velocity, gravity, flat_command)
                if phase == "warmup":
                    warmup_ticks += 1
                    if warmup_ticks >= 10:
                        phase = "blend"
                        blend_started = float(runner.data.time)

            if phase == "blend":
                progress = np.clip((runner.data.time - blend_started) / args.blend_s, 0.0, 1.0)
                alpha = progress * progress * (3 - 2 * progress)
                if progress >= 1.0:
                    phase = "flat"

            for _ in range(runner.cfg.sim.decimation):
                q = runner.data.qpos[7:36]
                dq = runner.data.qvel[6:35]
                if phase == "flat":
                    tau = np.clip(flat_policy.kp * (flat_target - q) - flat_policy.kd * dq,
                                  -torque_limit, torque_limit)
                elif phase == "blend":
                    dwaq_tau = runner.pd_control(dwaq_target)
                    flat_tau = np.clip(flat_policy.kp * (flat_target - q) - flat_policy.kd * dq,
                                       -torque_limit, torque_limit)
                    tau = (1 - alpha) * dwaq_tau + alpha * flat_tau
                else:
                    tau = runner.pd_control(dwaq_target)
                runner.data.ctrl[:29] = tau
                mujoco.mj_step(runner.model, runner.data)
            runner.gait_phase_time += DT
            x, y, z = map(float, runner.data.qpos[:3])
            _, gravity, _ = base_state(runner)
            trace.append({"t": float(runner.data.time), "x": x, "y": y, "z": z,
                          "phase": phase, "speed_command": float(flat_command[0]) if phase == "flat" else float(runner.command_vel[0]),
                          "gravity_z": float(gravity[2])})
            if runner.data.time + 1e-9 >= next_frame_time:
                write_frame()
                next_frame_time += 1 / 20
            if (z < TOP_HEIGHT + 0.35 and x > UPPER_START_X + 0.25) or gravity[2] > -0.5:
                outcome = "fall"
                break
            if x > GOAL_X and z > TOP_HEIGHT + 0.4:
                outcome = "goal"
                break

        if args.video:
            write_frame()
            assert ffmpeg is not None and ffmpeg.stdin is not None and last_frame is not None
            for _ in range(20):
                ffmpeg.stdin.write(last_frame)
            renderer.close()
            ffmpeg.stdin.close()
            if ffmpeg.wait() != 0:
                raise RuntimeError("Video encode failed")

        smooth_segment = [point for point in trace if 7.0 <= point["x"] <= 10.4]
        smoothness = None
        if len(smooth_segment) > 10:
            xs = np.array([point["x"] for point in smooth_segment])
            ys = np.array([point["y"] for point in smooth_segment])
            zs = np.array([point["z"] for point in smooth_segment])
            ts = np.array([point["t"] for point in smooth_segment])
            vxs = np.diff(xs) / np.diff(ts)
            smoothness = {
                "segment_x_m": [7.0, 10.4],
                "mean_forward_speed_mps": float(vxs.mean()),
                "forward_speed_std_mps": float(vxs.std()),
                "pelvis_height_std_cm": float(zs.std() * 100),
                "max_abs_lateral_offset_cm": float(np.abs(ys).max() * 100),
            }
        summary = {
            "mode": args.mode,
            "flat_speed_command_mps": commanded_speed,
            "initial_lateral_offset_cm": args.start_y_cm,
            "scene": str(SCENE.relative_to(ROOT)),
            "outcome": outcome,
            "sim_time_s": float(runner.data.time),
            "switch_time_s": switch_started,
            "switch_x_m": next((p["x"] for p in trace if p["t"] >= switch_started), None) if switch_started else None,
            "final_xyz_m": list(map(float, runner.data.qpos[:3])),
            "max_x_m": max(point["x"] for point in trace),
            "goal_x_m": GOAL_X,
            "smoothness_segment": smoothness,
            "video": video_path.name if args.video else None,
            "video_frames": frame_count + 20 if args.video else None,
            "center_correction_uses_simulator_ground_truth": True,
            "camera_perception_used": False,
            "dwaq_checkpoint_sha256": sha256(args.source_root / CHECKPOINT_RELATIVE),
            "flat_policy_sha256": sha256(FLAT_POLICY / "policy.onnx") if args.mode == "hybrid" else None,
        }
        (output_dir / f"{run_name}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
        with gzip.open(output_dir / f"{run_name}-trace.json.gz", "wt", encoding="utf-8") as target:
            json.dump(trace, target, ensure_ascii=False)
        print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
