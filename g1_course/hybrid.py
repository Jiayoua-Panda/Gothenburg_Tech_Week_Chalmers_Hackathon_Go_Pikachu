"""V0.5 混合控制器：高度扫描（感知）+ V0 学习策略 + 可调的经典层。

经典层两部分，参数向量 θ 见 DEFAULT_PARAMS（optimize.py 在 CPU 上用 CMA-ES 调）：
    1. 指令调度（governor）：前方看到台阶就减速（或加速）；侧倾坡上把横向速度和朝向往高处修正。
    2. 摆动腿残差：前方有升高时，摆动腿的髋 / 膝 / 踝叠加一段半正弦偏置，把脚抬高。
        残差加在策略输出的目标关节角上（CourseSim.residual），策略本身不改。

第 2 阶段（诊断：V0 上台阶失败是“脚尖踢到立面”）加的参数，默认 0 = 与第 1 阶段完全一样：
    w_toe  触发距离从骨盆改为从摆动脚脚尖算（本体感觉：脚的位置来自正运动学）
    reach  摆动后半段额外屈髋，让脚整个落在踏面上
    push   另一只脚已在更高处时，支撑腿伸膝 + 蹬踝，把身体顶上去
    boost  看到上台阶时加一点速度（第 1 阶段只会减速）

感知用 perception.height_scan 的小网格（9 × 5 条射线），加 1 cm 高斯噪声，模拟真实深度相机 / 激光雷达。
"""

import json
import math
import os
import pathlib

import numpy as np

from perception import height_scan

CTRL_FWD = np.round(np.arange(0.0, 0.81, 0.1), 3)  # 前方 0 ~ 0.8 m
CTRL_LAT = np.array([-0.4, -0.2, 0.0, 0.2, 0.4])  # 左右 ±0.4 m
SCAN_NOISE = 0.01  # m
PARAMS_PATH = pathlib.Path(__file__).resolve().parent / "outputs" / "hybrid" / "best_params.json"

DEFAULT_PARAMS = dict(
    slow_gain=0.5,  # 看到台阶时的减速比例（按台阶高度 / 0.15 m 缩放）
    v_min=0.25,  # 减速下限 m/s
    k_y=1.5,  # 横向偏移 → vy
    k_slope=2.0,  # 侧倾坡度 dz/dlat → vy（往高处）
    k_yaw=1.5,  # 朝向误差 → wz
    yaw_slope=0.5,  # 侧倾坡上朝高处偏的目标朝向 rad / (dz/dlat)
    hip=0.35,  # 摆动腿残差幅值 rad（按台阶高度 / 0.1 m 缩放，最多 1.5 倍）
    knee=0.6,
    ankle=0.25,
    t_swing=0.32,  # 半正弦持续时间 s
    trigger=0.45,  # 台阶边缘在前方多少 m 内才加残差
    # 第 2 阶段
    w_toe=0.0,  # 0 = 从骨盆算触发距离，1 = 从摆动脚脚尖算
    reach=0.0,  # 摆动后半段额外屈髋 rad
    t_reach=0.35,  # reach 窗口长度 s（从离地起）
    push=0.0,  # 支撑腿伸膝 rad（踝同时蹬 0.5 倍）
    boost=0.0,  # 上台阶时加速 m/s（按台阶高度 / 0.15 m 缩放）
)
PARAM_NAMES = list(DEFAULT_PARAMS)
PARAM_BOUNDS = dict(
    slow_gain=(0.0, 0.9), v_min=(0.1, 0.5), k_y=(0.0, 4.0), k_slope=(0.0, 6.0), k_yaw=(0.0, 4.0),
    yaw_slope=(0.0, 2.0), hip=(0.0, 0.9), knee=(0.0, 1.4), ankle=(0.0, 0.6), t_swing=(0.15, 0.5),
    trigger=(0.15, 0.8), w_toe=(0.0, 1.0), reach=(0.0, 0.8), t_reach=(0.2, 0.6), push=(0.0, 0.6),
    boost=(0.0, 0.5),
)
TOE = 0.12  # 脚尖在踝关节（ankle_roll_link）前方约 0.12 m（脚底前侧接触球的位置）
LEG_JOINTS = ("hip_pitch", "knee", "ankle_pitch")


def leg_indices(model):
    """按执行器名找每条腿的髋 / 膝 / 踝俯仰（SDK 顺序下标），不写死编号。"""
    names = [model.actuator(i).name for i in range(model.nu)]
    return [[names.index(f"{side}_{j}") for j in LEG_JOINTS] for side in ("left", "right")]


def terrain_features(sim, rng):
    """从小网格高度扫描提取：前方升高 / 下降、最近边缘距离、坡度（平面拟合）。"""
    _, rel = height_scan(sim, CTRL_FWD, CTRL_LAT)
    rel = np.nan_to_num(rel, nan=-0.5) + rng.normal(0.0, SCAN_NOISE, rel.shape)  # 没打到 = 通道外，当成深坑
    center = rel[:, 1:4]  # |lat| ≤ 0.2 m
    band = center[1:7]  # 前方 0.1 ~ 0.6 m
    up, down = max(0.0, float(band.max())), min(0.0, float(band.min()))
    rows = np.where(np.abs(center).mean(axis=1) > 0.03)[0]
    edge = float(CTRL_FWD[rows[0]]) if len(rows) else math.inf
    F, L = np.meshgrid(CTRL_FWD[:5], CTRL_LAT, indexing="ij")  # 近处 0 ~ 0.4 m 拟合平面 z = a·f + b·l + c
    A = np.column_stack([F.ravel(), L.ravel(), np.ones(F.size)])
    (a, b, _), *_ = np.linalg.lstsq(A, rel[:5].ravel(), rcond=None)
    return dict(up=up, down=down, edge=edge, pitch=float(a), roll=float(b))


def make_controller(params=None):
    p = {**DEFAULT_PARAMS, **(params or {})}

    def controller(sim, t, vx=0.5, settle=2.0, ramp=1.0):
        st = sim.ctrl_state
        if "legs" not in st:
            st.update(legs=leg_indices(sim.model), air=[0.0, 0.0], amp=[0.0, 0.0], rng=np.random.default_rng(0))
        if t < settle:
            sim.residual[:] = 0.0
            return np.zeros(3)
        f = terrain_features(sim, st["rng"])
        st["features"] = f

        # 1. 指令调度
        v = vx * min(1.0, (t - settle) / ramp)
        h = max(f["up"], -f["down"])
        if f["edge"] < 1.0 and h > 0.03:
            v = max(min(v, p["v_min"]), v * (1.0 - p["slow_gain"] * min(1.0, h / 0.15)))
            v += p["boost"] * min(1.0, f["up"] / 0.15)
        y, yaw = sim.data.qpos[1], sim.yaw()
        vy = -p["k_y"] * y + p["k_slope"] * f["roll"]
        wz = -p["k_yaw"] * (yaw - p["yaw_slope"] * f["roll"])
        cmd = sim.policy.clip_command([v, vy, wz])

        # 2. 摆动腿残差：离地时锁定幅值，摆动中看到更高的台阶再加大
        contact = sim.feet_in_contact()
        res = np.zeros_like(sim.residual)
        heading = sim.heading()
        pelvis = sim.data.xpos[sim.pelvis]
        foot_pos = [sim.data.xpos[b] for b in sim.foot_ids]
        for leg, (hip, knee, ankle) in enumerate(st["legs"]):
            toe_fwd = float((foot_pos[leg] - pelvis) @ heading) + TOE
            dist = f["edge"] - p["w_toe"] * toe_fwd  # w_toe = 0 时就是第 1 阶段的骨盆距离
            want = 0.0 if f["up"] < 0.035 or dist > p["trigger"] else min(1.5, f["up"] / 0.1)  # 0.035 m ≈ 3.5σ 噪声
            if contact[leg]:
                st["air"][leg], st["amp"][leg] = 0.0, 0.0
                rise = float(foot_pos[1 - leg][2] - foot_pos[leg][2])  # 另一只脚高出多少（本体感觉）
                if p["push"] > 0.0 and rise > 0.03:
                    k = min(1.5, rise / 0.1)
                    res[knee] -= p["push"] * k  # 伸膝
                    res[ankle] += 0.5 * p["push"] * k  # 蹬踝（正 = 脚尖向下）
                continue
            if st["air"][leg] == 0.0 or want > st["amp"][leg]:
                st["amp"][leg] = max(st["amp"][leg], want)
            st["air"][leg] += sim.policy.step_dt
            s = st["air"][leg] / p["t_swing"]
            if st["amp"][leg] > 0.0 and s < 1.0:
                shape = st["amp"][leg] * math.sin(math.pi * s)
                res[hip] -= p["hip"] * shape  # 髋屈（负 = 大腿向前抬）
                res[knee] += p["knee"] * shape  # 屈膝
                res[ankle] -= p["ankle"] * shape  # 勾脚尖
            r = (st["air"][leg] / p["t_reach"] - 0.4) / 0.6  # reach：摆动窗口的后 60%
            if st["amp"][leg] > 0.0 and p["reach"] > 0.0 and 0.0 < r < 1.0:
                res[hip] -= p["reach"] * st["amp"][leg] * math.sin(math.pi * r)
        sim.residual[:] = res
        return cmd

    return controller


def load_params(path=None):
    """读参数文件（缺的新参数用默认值补齐）。路径：参数 > 环境变量 HYBRID_PARAMS > outputs/hybrid/best_params.json。"""
    path = pathlib.Path(path or os.environ.get("HYBRID_PARAMS") or PARAMS_PATH)
    if path.exists():
        return {**DEFAULT_PARAMS, **{k: v for k, v in json.loads(path.read_text()).items() if k in DEFAULT_PARAMS}}
    return dict(DEFAULT_PARAMS)


_default = None


def controller(sim, t, **kw):
    """run_course 的 CONTROLLERS["hybrid"]：用 best_params.json（没有就用默认值）。"""
    global _default
    if _default is None:
        _default = make_controller(load_params())
    return _default(sim, t, **kw)
