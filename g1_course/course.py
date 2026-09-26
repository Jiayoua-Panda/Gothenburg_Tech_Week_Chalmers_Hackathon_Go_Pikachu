"""测试路线定义：分段列表 → MuJoCo 场景 XML + 地面高度剖面。

改路线只需要改 FULL_COURSE。每段是一个 dict：
    flat      平地                 length
    ramp      坡道（angle>0 上坡，<0 下坡，单位度）  angle, length（水平长度）
    targets   步幅区（地面落脚目标） spacing, n
    block     单级台阶（上去再下来）  height, length
    stairs    楼梯（rise>0 上，<0 下） rise, tread, n

3D 路段（COURSE_3D，高度随 x 和 y 变化）：
    cross_slope    侧倾坡（绕行走线向一侧倾斜，入口/出口 0.6 m 内渐变）  angle, length
    rough          不平地面（随机起伏，高度场 hfield）              amp, length, seed
    angled_stairs  斜向楼梯（上 n 级 + 平台 + 下楼，整体绕 z 转 yaw 度） rise, tread, n, yaw, width, landing
"""

from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass, field

import numpy as np

MODEL_DIR = pathlib.Path(__file__).resolve().parent.parent / "g1_step_playback" / "models" / "g1"
TRACK_HALF_WIDTH = 0.6
FOOT_Y = 0.12  # G1 两脚踝的横向间距约 ±0.12 m
HF_RES = 0.02  # 高度场网格间距 m
TERRAIN_GROUP = 2  # 地面和路线几何体放在 geom group 2，高度扫描只看这一组（机器人在 0/1 组）

FULL_COURSE = [
    dict(kind="flat", length=2.0, name="A warm-up"),
    dict(kind="ramp", angle=5, length=2.0, name="B1 up 5deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=-5, length=2.0, name="B1 down 5deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=10, length=2.0, name="B2 up 10deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=-10, length=2.0, name="B2 down 10deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=15, length=1.5, name="B3 up 15deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=-15, length=1.5, name="B3 down 15deg"),
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.05, length=1.0, name="C1 step 5cm"),
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.10, length=1.0, name="C2 step 10cm"),
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.15, length=1.0, name="C3 step 15cm"),
    dict(kind="flat", length=1.0),
    dict(kind="stairs", rise=0.10, tread=0.30, n=4, name="D1 up 10/30"),
    dict(kind="flat", length=1.0),
    dict(kind="stairs", rise=0.15, tread=0.25, n=4, name="D2 up 15/25"),
    dict(kind="flat", length=1.5),
    dict(kind="stairs", rise=-0.15, tread=0.30, n=4, name="E1 down 15/30"),
    dict(kind="flat", length=1.0),
    dict(kind="stairs", rise=-0.10, tread=0.30, n=4, name="E2 down 10/30"),
    dict(kind="flat", length=2.0, name="finish"),
]

# 3D 路线整体架在 BASE_3D 高的平台上，侧倾坡低的一侧才不会埋进地板
BASE_3D = 0.25
COURSE_3D = [
    dict(kind="flat", length=2.0, name="A warm-up"),
    dict(kind="cross_slope", angle=5, length=2.5, name="F1 cross 5deg"),
    dict(kind="flat", length=1.0),
    dict(kind="cross_slope", angle=10, length=2.5, name="F2 cross 10deg"),
    dict(kind="flat", length=1.0),
    dict(kind="rough", amp=0.02, length=2.0, seed=1, name="G1 rough 2cm"),
    dict(kind="flat", length=1.0),
    dict(kind="rough", amp=0.04, length=2.0, seed=2, name="G2 rough 4cm"),
    dict(kind="flat", length=1.0),
    dict(kind="angled_stairs", rise=0.10, tread=0.30, n=3, yaw=15, width=2.0, landing=0.6, name="H1 stairs 15deg"),
    dict(kind="flat", length=1.5),
    dict(kind="angled_stairs", rise=0.10, tread=0.30, n=3, yaw=30, width=2.0, landing=0.6, name="H2 stairs 30deg"),
    dict(kind="flat", length=1.5),
    dict(kind="angled_stairs", rise=0.10, tread=0.30, n=3, yaw=0, width=0.6, landing=0.6, name="I narrow 0.6m"),
    dict(kind="flat", length=1.5),
    dict(kind="flat", length=2.0, name="finish"),
]
KINDS_3D = {"cross_slope", "rough", "angled_stairs"}

# 三种状态对比用的演示路线：坡度 5° → 10°、台阶 5 / 10 / 15 cm、楼梯 10 / 15 cm、侧坡、斜楼梯，
# 中间一段有人突然横穿通道（person=True，run_course.py --mode tour 会放一个人进来）。
# 每个有名字的路段后面都跟 ≥ 1 m 平地：通过判定要求越过终点后再走 0.8 m。
DEMO_COURSE = [
    dict(kind="flat", length=2.0, name="A warm-up"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=5, length=2.0, name="B1 up 5deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=10, length=2.0, name="B2 up 10deg"),
    dict(kind="flat", length=1.0),
    dict(kind="ramp", angle=-10, length=3.0, name="B3 down 10deg"),  # 3 m 下 0.53 m，回到起点高度
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.05, length=1.0, name="C1 step 5cm"),
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.10, length=1.0, name="C2 step 10cm"),
    dict(kind="flat", length=1.0),
    dict(kind="block", height=0.15, length=1.0, name="C3 step 15cm"),
    dict(kind="flat", length=1.0),
    dict(kind="cross_slope", angle=5, length=2.5, name="F1 cross 5deg"),
    dict(kind="flat", length=1.0),
    dict(kind="flat", length=4.0, name="S person crossing", person=True),
    dict(kind="flat", length=1.0),
    dict(kind="stairs", rise=0.10, tread=0.30, n=3, name="D1 up 3x10cm"),
    dict(kind="flat", length=1.0),
    dict(kind="stairs", rise=0.15, tread=0.25, n=2, name="D2 up 2x15cm"),
    dict(kind="flat", length=1.5),
    dict(kind="stairs", rise=-0.15, tread=0.30, n=4, name="E1 down 4x15cm"),
    dict(kind="flat", length=1.0),
    dict(kind="angled_stairs", rise=0.10, tread=0.30, n=3, yaw=15, width=2.0, landing=0.6, name="H1 stairs 15deg"),
    dict(kind="flat", length=1.5),
    dict(kind="flat", length=2.0, name="finish"),
]
NAMED_COURSES = {"full": (FULL_COURSE, 0.0), "3d": (COURSE_3D, BASE_3D), "demo": (DEMO_COURSE, BASE_3D)}


def _smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3 - 2 * u)


def _cross_slope_field(xs, ys, spec):
    """绕行走线 y=0 侧倾：z = y·tan(angle)·s(x)，入口/出口 0.6 m 内渐变。

    y=0 处高度始终不变，只测侧倾，不混入上下坡。返回 (dz ≥ 0, 基准高度偏移)。
    """
    L, ramp = spec["length"], 0.6
    s = np.minimum(_smoothstep(xs / ramp), _smoothstep((L - xs) / ramp))
    k = math.tan(math.radians(spec["angle"]))
    return np.outer(ys + TRACK_HALF_WIDTH, s) * k + TRACK_HALF_WIDTH * k * (1 - s)[None, :], -TRACK_HALF_WIDTH * k


def _rough_field(xs, ys, spec):
    """0 ~ amp 的随机起伏（约 0.15 m 波长），两端 0.3 m 内渐变到平地。"""
    rng = np.random.default_rng(spec.get("seed", 0))
    z = rng.uniform(0, 1, (len(ys), len(xs)))
    k = 7  # 7 × 2 cm ≈ 0.14 m 的盒式平滑，做两遍
    kernel = np.ones(k) / k
    for _ in range(2):
        z = np.apply_along_axis(lambda r: np.convolve(r, kernel, mode="same"), 1, z)
        z = np.apply_along_axis(lambda c: np.convolve(c, kernel, mode="same"), 0, z)
    z = (z - z.min()) / (z.max() - z.min())
    L = spec["length"]
    s = np.minimum(_smoothstep(xs / 0.3), _smoothstep((L - xs) / 0.3))
    return z * spec["amp"] * s[None, :], 0.0


@dataclass
class Course:
    boxes: list = field(default_factory=list)  # (x0, x1, z_top)
    ramps: list = field(default_factory=list)  # (x0, x1, z0, z1)
    targets: list = field(default_factory=list)  # (x, y, segment_name)
    segments: list = field(default_factory=list)  # (name, x0, x1, spec)
    oboxes: list = field(default_factory=list)  # 绕 z 旋转的方块 (cx, cy, hx, hy, yaw, z_top)，底面在 z=0
    hfields: list = field(default_factory=list)  # (x0, x1, z_base, dz[ny, nx])，dz ≥ 0，y 覆盖 ±TRACK_HALF_WIDTH
    start_z: float = 0.0
    end_x: float = 0.0

    def ground(self, x: float, y: float = 0.0) -> float:
        z = 0.0
        W = TRACK_HALF_WIDTH
        on_track = -W <= y <= W
        for x0, x1, top in self.boxes:
            if x0 <= x < x1 and on_track:
                z = max(z, top)
        for x0, x1, z0, z1 in self.ramps:
            if x0 <= x < x1 and on_track:
                z = max(z, z0 + (z1 - z0) * (x - x0) / (x1 - x0))
        for cx, cy, hx, hy, yaw, top in self.oboxes:
            c, s = math.cos(yaw), math.sin(yaw)
            lx, ly = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
            if abs(lx) <= hx and abs(ly) <= hy:
                z = max(z, top)
        for x0, x1, zb, dz in self.hfields:
            if x0 <= x < x1 and on_track:
                ny, nx = dz.shape
                u, v = (x - x0) / (x1 - x0) * (nx - 1), (y + W) / (2 * W) * (ny - 1)
                i, j = min(int(u), nx - 2), min(int(v), ny - 2)
                fu, fv = u - i, v - j
                h = (dz[j, i] * (1 - fu) * (1 - fv) + dz[j, i + 1] * fu * (1 - fv)
                     + dz[j + 1, i] * (1 - fu) * fv + dz[j + 1, i + 1] * fu * fv)
                z = max(z, zb + h)
        return z

    def segment_at(self, x: float) -> str:
        for name, x0, x1, _ in self.segments:
            if x0 <= x < x1:
                return name
        return ""


def build(spec_list, lead_in=1.5, start_z=0.0) -> Course:
    """按顺序铺设各段；x=0 是机器人出生点，前面留 lead_in 米平地。"""
    c = Course(start_z=start_z)
    x, z = -1.0, start_z

    def ground_box(x0, x1, top):
        if top > 1e-6:
            c.boxes.append((x0, x1, top))

    ground_box(x, lead_in, z)
    x = lead_in
    for s in spec_list:
        x0 = x
        k = s["kind"]
        if k == "flat":
            x1 = x0 + s["length"]
            ground_box(x0, x1, z)
        elif k == "ramp":
            x1 = x0 + s["length"]
            z1 = z + s["length"] * math.tan(math.radians(s["angle"]))
            ground_box(x0, x1, min(z, z1))  # 坡道下方的实心底座
            c.ramps.append((x0, x1, z, z1))
            z = z1
        elif k == "targets":
            for i in range(s["n"]):
                c.targets.append((x0 + i * s["spacing"], FOOT_Y if i % 2 == 0 else -FOOT_Y, s.get("name", "")))
            x1 = x0 + s["n"] * s["spacing"]
            ground_box(x0, x1, z)
        elif k == "block":
            x1 = x0 + s["length"]
            ground_box(x0, x1, z + s["height"])
        elif k == "stairs":
            for i in range(s["n"]):
                ground_box(x0 + i * s["tread"], x0 + (i + 1) * s["tread"], z + (i + 1) * s["rise"])
            x1 = x0 + s["n"] * s["tread"]
            z = z + s["n"] * s["rise"]
        elif k in ("cross_slope", "rough"):
            x1 = x0 + s["length"]
            nx = int(round(s["length"] / HF_RES)) + 1
            ny = int(round(2 * TRACK_HALF_WIDTH / HF_RES)) + 1
            xs, ys = np.linspace(0, s["length"], nx), np.linspace(-TRACK_HALF_WIDTH, TRACK_HALF_WIDTH, ny)
            dz, z_off = (_cross_slope_field if k == "cross_slope" else _rough_field)(xs, ys, s)
            c.hfields.append((x0, x1, z + z_off, dz))
        elif k == "angled_stairs":
            # 局部坐标 u 沿楼梯方向：上 n 级 → 平台 → 下 n-1 级回到原高度；整体中心在 y=0，绕 z 转 yaw
            t, r, n, L = s["tread"], s["rise"], s["n"], s["landing"]
            yaw, hy = math.radians(s["yaw"]), s["width"] / 2
            steps = [(i * t, (i + 1) * t, z + (i + 1) * r) for i in range(n)]
            steps.append((n * t, n * t + L, z + n * r))
            steps += [(n * t + L + j * t, n * t + L + (j + 1) * t, z + (n - 1 - j) * r) for j in range(n - 1)]
            u_len = steps[-1][1]
            extent_x = u_len * math.cos(yaw) + 2 * hy * math.sin(yaw)
            x1 = x0 + extent_x
            ground_box(x0, x1, z)
            cx = x0 + extent_x / 2
            for u0, u1, top in steps:
                um = (u0 + u1) / 2 - u_len / 2
                c.oboxes.append((cx + um * math.cos(yaw), um * math.sin(yaw), (u1 - u0) / 2, hy, yaw, top))
        else:
            raise ValueError(k)
        if s.get("name"):
            c.segments.append((s["name"], x0, x1, s))
        x = x1
    ground_box(x, x + lead_in, z)
    c.end_x = x + lead_in
    return c


def standalone(spec) -> Course:
    """单独测试一段：前后各 1.5 m 平地；下楼梯段从对应高度的平台出发。"""
    start_z = 0.0
    if spec["kind"] == "stairs" and spec["rise"] < 0:
        start_z = -spec["n"] * spec["rise"]
    elif spec["kind"] == "ramp" and spec["angle"] < 0:
        start_z = -spec["length"] * math.tan(math.radians(spec["angle"]))
    elif spec["kind"] in KINDS_3D:
        start_z = BASE_3D
    return build([spec], start_z=start_z)


def build_named(name: str) -> Course:
    """"full" = 原始 2D 路线；"3d" = 侧倾坡 / 不平地面 / 斜向与窄楼梯；"demo" = 三种状态对比用的混合路线。"""
    spec_list, start_z = NAMED_COURSES[name]
    return build(spec_list, start_z=start_z)


def write_scene(course: Course, name: str, human: bool = False) -> pathlib.Path:
    """生成场景 XML。必须放在 g1_29dof.xml 同目录，include 和 meshdir 才能找到。

    human=True 时加一个 mocap 人体（胶囊，无碰撞），由仿真代码移动，用于安全演示。
    """
    grp = f'group="{TERRAIN_GROUP}"'
    fric = 'friction="1 0.005 0.0001"'
    geoms = []
    for i, (x0, x1, top) in enumerate(course.boxes):
        cx, hx, hz = (x0 + x1) / 2, (x1 - x0) / 2, top / 2
        shade = 0.55 + 0.25 * (i % 2)
        geoms.append(
            f'<geom name="course_{i}" type="box" pos="{cx:.4f} 0 {hz:.4f}" size="{hx:.4f} {TRACK_HALF_WIDTH} {hz:.4f}" '
            f'rgba="{shade:.2f} {shade:.2f} {shade + 0.05:.2f} 1" {fric} {grp}/>'
        )
    for i, (cx, cy, hx, hy, yaw, top) in enumerate(course.oboxes):
        shade = 0.5 + 0.25 * (i % 2)
        geoms.append(
            f'<geom name="obox_{i}" type="box" pos="{cx:.4f} {cy:.4f} {top / 2:.4f}" size="{hx:.4f} {hy:.4f} {top / 2:.4f}" '
            f'quat="{math.cos(yaw / 2):.6f} 0 0 {math.sin(yaw / 2):.6f}" '
            f'rgba="{shade:.2f} {shade + 0.03:.2f} {shade + 0.1:.2f} 1" {fric} {grp}/>'
        )
    assets = []
    W = TRACK_HALF_WIDTH
    for i, (x0, x1, z0, z1) in enumerate(course.ramps):
        zb = min(z0, z1)
        verts = [(x0, -W, zb), (x0, W, zb), (x1, -W, zb), (x1, W, zb), (x0, -W, z0), (x0, W, z0), (x1, -W, z1), (x1, W, z1)]
        assets.append(f'<mesh name="ramp_{i}" vertex="{" ".join(f"{v:.4f}" for p in verts for v in p)}"/>')
        geoms.append(f'<geom name="ramp_{i}" type="mesh" mesh="ramp_{i}" rgba="0.62 0.66 0.58 1" {fric} {grp}/>')
    for i, (x0, x1, zb, dz) in enumerate(course.hfields):
        ny, nx = dz.shape
        amp = max(float(dz.max()), 1e-4)
        # elevation 的第一行对应 +y（实测），所以按 y 翻转；dz.min() = 0，归一化后仍是 dz / amp
        elev = " ".join(f"{v:.4f}" for v in (dz[::-1] / amp).ravel())
        base = max(zb, 0.02)
        assets.append(f'<hfield name="hf_{i}" nrow="{ny}" ncol="{nx}" size="{(x1 - x0) / 2:.4f} {W} {amp:.4f} {base:.4f}" elevation="{elev}"/>')
        geoms.append(
            f'<geom name="hfield_{i}" type="hfield" hfield="hf_{i}" pos="{(x0 + x1) / 2:.4f} 0 {zb:.4f}" '
            f'rgba="0.58 0.62 0.70 1" {fric} {grp}/>'
        )
    for i, (tx, ty, _) in enumerate(course.targets):
        geoms.append(
            f'<geom name="target_{i}" type="cylinder" pos="{tx:.4f} {ty:.4f} {course.ground(tx, ty) + 0.001:.4f}" '
            f'size="0.04 0.001" rgba="1 0.85 0.1 0.9" contype="0" conaffinity="0"/>'
        )
    bodies = []
    if human:
        bodies.append(
            '<body name="human" mocap="true" pos="0 5 0">'
            '<geom name="human_body" type="capsule" fromto="0 0 0.25 0 0 1.45" size="0.22" rgba="0.95 0.55 0.15 1" contype="0" conaffinity="0"/>'
            '<geom name="human_head" type="sphere" pos="0 0 1.62" size="0.12" rgba="0.95 0.75 0.55 1" contype="0" conaffinity="0"/>'
            '</body>'
        )
    xml = f"""<mujoco model="g1 course {name}">
  <include file="g1_29dof.xml"/>
  <statistic center="4 0 0.5" extent="6.0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="-130" elevation="-20" offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    {chr(10).join('    ' + m for m in assets)}
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" {grp}/>
    {chr(10).join('    ' + g for g in geoms)}
    {chr(10).join('    ' + b for b in bodies)}
  </worldbody>
</mujoco>
"""
    path = MODEL_DIR / f"course_{name}.xml"
    path.write_text(xml, encoding="utf-8")
    return path


def plot_profile(course: Course, path, title=None):
    """y=0 处的高度剖面；有 3D 路段时再加一张俯视高度图。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    is_3d = bool(course.oboxes or course.hfields)
    xs = [i * 0.01 for i in range(int(-100), int(course.end_x * 100))]
    zs = [course.ground(x) for x in xs]
    if is_3d:
        fig, (ax, ax2) = plt.subplots(2, 1, figsize=(14, 6.4), sharex=True, gridspec_kw=dict(height_ratios=[1, 1.1]))
    else:
        fig, ax = plt.subplots(figsize=(14, 3.2))
    ax.fill_between(xs, 0, zs, color="#8a93a6")
    ax.plot(xs, zs, color="#39414f", lw=1)
    for tx, _, _ in course.targets:
        ax.plot(tx, course.ground(tx) + 0.01, "v", color="#f2b705", ms=5)
    for name, x0, x1, _ in course.segments:
        if name == "finish":
            continue
        ax.axvspan(x0, x1, color="#4c78a8", alpha=0.08)
        ax.text((x0 + x1) / 2, max(zs) + 0.12, name.split(" ", 1)[0], ha="center", fontsize=9, weight="bold")
    ax.set_ylabel("height at y=0 [m]" if is_3d else "height [m]")
    ax.set_ylim(-0.02, max(zs) + 0.25)
    ax.set_xlim(-1, course.end_x)
    ax.set_title(title or "G1 test course (A flat · B slopes up/down · C single steps · D stairs up · E stairs down)")
    if is_3d:
        gx = np.arange(-1.0, course.end_x, 0.02)
        gy = np.arange(-1.1, 1.1, 0.02)
        H = np.array([[course.ground(x, y) for x in gx] for y in gy])
        im = ax2.imshow(H, origin="lower", extent=(gx[0], gx[-1], gy[0], gy[-1]), aspect="auto", cmap="viridis")
        ax2.axhline(0, color="white", lw=0.8, ls="--", alpha=0.7)
        ax2.axhspan(-TRACK_HALF_WIDTH, TRACK_HALF_WIDTH, fill=False, ec="white", lw=0.6, alpha=0.5)
        ax2.set_ylabel("y [m]")
        ax2.set_xlabel("x [m]  (top view; dashed = walking line)")
        fig.colorbar(im, ax=ax2, label="height [m]", pad=0.01, fraction=0.03)
    else:
        ax.set_xlabel("x [m]")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
