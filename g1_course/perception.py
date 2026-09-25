"""感知：机器人前方的局部高度图（射线向下打到地形上）。

V0 是盲走，看不到它；混合控制器（hybrid.py）和将来的 V1 策略把它当输入。
射线只看地形组（course.TERRAIN_GROUP），会穿过机器人自身。
"""

import mujoco
import numpy as np

import course as C

SCAN_FWD = np.round(np.arange(-0.2, 1.21, 0.1), 3)  # 沿朝向 -0.2 ~ 1.2 m
SCAN_LAT = np.round(np.arange(-0.5, 0.51, 0.1), 3)  # 左右 ±0.5 m
SCAN_MASK = np.array([g == C.TERRAIN_GROUP for g in range(6)], dtype=np.uint8)  # 只看地形组，射线穿过机器人自身
DOWN = np.array([0.0, 0.0, -1.0])


def height_scan(sim, fwd=SCAN_FWD, lat_grid=SCAN_LAT):
    """返回 (点[nf, nl, 3] 世界坐标, 相对高度[nf, nl])；相对高度以机器人正下方那一格为 0。"""
    h = sim.heading()
    left = np.array([-h[1], h[0], 0.0])
    base = sim.data.xpos[sim.pelvis]
    pts = np.full((len(fwd), len(lat_grid), 3), np.nan)
    gid = np.zeros(1, np.int32)
    for i, f in enumerate(fwd):
        for j, lat in enumerate(lat_grid):
            p = base + f * h + lat * left
            p[2] = base[2] + 0.5
            d = mujoco.mj_ray(sim.model, sim.data, p, DOWN, SCAN_MASK, 1, -1, gid)
            if d >= 0:
                pts[i, j] = p + d * DOWN
    i0, j0 = int(np.argmin(np.abs(fwd))), int(np.argmin(np.abs(lat_grid)))
    return pts, pts[..., 2] - pts[i0, j0, 2]


SCAN_CMAP, SCAN_RANGE = "RdYlBu_r", 0.2  # 低于脚下 = 蓝，平 = 浅黄，高于脚下 = 红（±0.2 m 饱和）；视频和 PNG 共用


def scan_color(rel):
    import matplotlib

    return np.array(matplotlib.colormaps[SCAN_CMAP](0.5 + 0.5 * float(np.clip(rel / SCAN_RANGE, -1, 1))), dtype=np.float32)


def add_scan(scene, sim):
    pts, rel = height_scan(sim)
    for p, r in zip(pts.reshape(-1, 3), rel.ravel()):
        if np.isnan(r) or scene.ngeom >= scene.maxgeom:
            continue
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.016, 0, 0]),
            p + np.array([0, 0, 0.012]), np.eye(3).flatten(), scan_color(r),
        )
        scene.ngeom += 1
