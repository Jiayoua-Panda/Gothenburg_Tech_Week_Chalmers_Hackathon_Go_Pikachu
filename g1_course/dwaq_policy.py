"""G1-DWAQ 楼梯策略推理（第三方权重），接口与 G1Policy 相同，可直接放进 CourseSim。

来源：G1DWAQ_Lab（https://github.com/liuyufei-nubot/G1DWAQ_Lab），commit bebb0ea，
TienKung-Lab/legged_lab/scripts/sim2sim_g1_dwaq.py，BSD-3-Clause。权重 model_9999.pt 不是本项目训练的。
这里只重写推理路径：VAE 编码器取均值（速度 3 + 潜变量 16）→ actor MLP；用 numpy 算，torch 只用来读 checkpoint。

观测（100 维，Isaac Lab 关节顺序）：角速度 3、投影重力 3、速度指令 3、关节位置偏差 29、关节速度 29、上一步动作 29、
步态相位 4（周期 0.8 s，左右相差半个周期）。编码器输入 5 帧历史（旧 → 新）。
权重：third_party/g1_dwaq/model_9999.pt（fetch_dwaq.sh 下载，SHA-256 5042017a…759c1，不进 git）。
"""

from __future__ import annotations

import pathlib

import numpy as np

CKPT = pathlib.Path(__file__).resolve().parent / "third_party" / "g1_dwaq" / "model_9999.pt"

MUJOCO_DOF_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
LAB_DOF_NAMES = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint", "left_hip_roll_joint",
    "right_hip_roll_joint", "waist_roll_joint", "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint", "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint",
]
# MuJoCo（= SDK）顺序
DEFAULT_POS = np.array([
    -0.2, 0.0, 0.0, 0.42, -0.23, 0.0,
    -0.2, 0.0, 0.0, 0.42, -0.23, 0.0,
    0.0, 0.0, 0.0,
    0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
    0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
])
KP = np.array([200, 150, 150, 200, 20, 20, 200, 150, 150, 200, 20, 20, 200, 200, 200,
               100, 100, 50, 50, 40, 40, 40, 100, 100, 50, 50, 40, 40, 40], dtype=float)
KD = np.array([5, 5, 5, 5, 2, 2, 5, 5, 5, 5, 2, 2, 5, 5, 5] + [2] * 14, dtype=float)
HISTORY = 5
ACTION_SCALE = 0.25
CLIP = 100.0
GAIT_PERIOD, GAIT_OFFSET = 0.8, 0.5


def _elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))


class DwaqPolicy:
    step_dt = 0.02  # sim dt 0.005 × decimation 4，与 V0 相同
    init_height = 0.793

    def __init__(self, path=CKPT):
        import torch

        if not pathlib.Path(path).exists():
            raise FileNotFoundError(f"{path} missing — run g1_course/fetch_dwaq.sh")
        sd = torch.load(path, map_location="cpu", weights_only=True)["model_state_dict"]

        def lin(name):
            return sd[f"{name}.weight"].numpy().astype(np.float64), sd[f"{name}.bias"].numpy().astype(np.float64)

        self.actor = [lin(f"actor.{i}") for i in (0, 2, 4, 6)]
        self.encoder = [lin("encoder.0"), lin("encoder.2")]
        self.mean_vel, self.mean_latent = lin("encode_mean_vel"), lin("encode_mean_latent")
        mj = {n: i for i, n in enumerate(MUJOCO_DOF_NAMES)}
        lab = {n: i for i, n in enumerate(LAB_DOF_NAMES)}
        self.m2l = np.array([mj[n] for n in LAB_DOF_NAMES])
        self.l2m = np.array([lab[n] for n in MUJOCO_DOF_NAMES])
        self.kp, self.kd = KP, KD
        self.cmd_ranges = {"lin_vel_x": [-0.5, 1.0], "lin_vel_y": [-0.3, 0.3], "ang_vel_z": [-0.6, 0.6]}
        self.reset()

    def default_sdk(self):
        return DEFAULT_POS.copy()

    def reset(self):
        self.action = np.zeros(29)
        self.history = None
        self.phase_time = 0.0

    def clip_command(self, cmd):
        r = self.cmd_ranges
        return np.array([np.clip(cmd[0], *r["lin_vel_x"]), np.clip(cmd[1], *r["lin_vel_y"]), np.clip(cmd[2], *r["ang_vel_z"])])

    def _gait_phase(self):
        left = (self.phase_time % GAIT_PERIOD) / GAIT_PERIOD
        right = (self.phase_time / GAIT_PERIOD + GAIT_OFFSET) % 1.0
        return np.array([np.sin(2 * np.pi * left), np.cos(2 * np.pi * left), np.sin(2 * np.pi * right), np.cos(2 * np.pi * right)])

    def act(self, q_sdk, dq_sdk, ang_vel_b, gravity_b, cmd):
        """输入 SDK 顺序关节状态，返回 SDK 顺序目标关节角（与 G1Policy.act 同一约定）。"""
        obs = np.concatenate([ang_vel_b, gravity_b, np.asarray(cmd, dtype=float),
                              (q_sdk - DEFAULT_POS)[self.m2l], dq_sdk[self.m2l], np.clip(self.action, -CLIP, CLIP),
                              self._gait_phase()])
        obs = np.clip(obs, -CLIP, CLIP)
        if self.history is None:
            self.history = np.tile(obs, (HISTORY, 1))
        else:
            self.history = np.vstack([self.history[1:], obs])
        h = self.history.ravel()
        for w, b in self.encoder:
            h = _elu(w @ h + b)
        code = np.concatenate([self.mean_vel[0] @ h + self.mean_vel[1], self.mean_latent[0] @ h + self.mean_latent[1]])
        a = np.concatenate([code, obs])
        for k, (w, b) in enumerate(self.actor):
            a = w @ a + b
            if k < len(self.actor) - 1:
                a = _elu(a)
        self.action = np.clip(a, -CLIP, CLIP)
        self.phase_time += self.step_dt
        return (self.action * ACTION_SCALE)[self.l2m] + DEFAULT_POS
