"""测试路线定义：分段列表 → MuJoCo 场景 XML + 地面高度剖面。

改路线只需要改 FULL_COURSE。每段是一个 dict：
    flat      平地                 length
    ramp      坡道（angle>0 上坡，<0 下坡，单位度）  angle, length（水平长度）
    targets   步幅区（地面落脚目标） spacing, n
    block     单级台阶（上去再下来）  height, length
    stairs    楼梯（rise>0 上，<0 下） rise, tread, n
"""

from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass, field

MODEL_DIR = pathlib.Path(__file__).resolve().parent.parent / "g1_step_playback" / "models" / "g1"
TRACK_HALF_WIDTH = 0.6
FOOT_Y = 0.12  # G1 两脚踝的横向间距约 ±0.12 m

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


@dataclass
class Course:
    boxes: list = field(default_factory=list)  # (x0, x1, z_top)
    ramps: list = field(default_factory=list)  # (x0, x1, z0, z1)
    targets: list = field(default_factory=list)  # (x, y, segment_name)
    segments: list = field(default_factory=list)  # (name, x0, x1, spec)
    start_z: float = 0.0
    end_x: float = 0.0

    def ground(self, x: float) -> float:
        z = 0.0
        for x0, x1, top in self.boxes:
            if x0 <= x < x1:
                z = max(z, top)
        for x0, x1, z0, z1 in self.ramps:
            if x0 <= x < x1:
                z = max(z, z0 + (z1 - z0) * (x - x0) / (x1 - x0))
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
    return build([spec], start_z=start_z)


def write_scene(course: Course, name: str) -> pathlib.Path:
    """生成场景 XML。必须放在 g1_29dof.xml 同目录，include 和 meshdir 才能找到。"""
    geoms = []
    for i, (x0, x1, top) in enumerate(course.boxes):
        cx, hx, hz = (x0 + x1) / 2, (x1 - x0) / 2, top / 2
        shade = 0.55 + 0.25 * (i % 2)
        geoms.append(
            f'<geom name="course_{i}" type="box" pos="{cx:.4f} 0 {hz:.4f}" size="{hx:.4f} {TRACK_HALF_WIDTH} {hz:.4f}" '
            f'rgba="{shade:.2f} {shade:.2f} {shade + 0.05:.2f} 1" friction="1 0.005 0.0001"/>'
        )
    meshes = []
    W = TRACK_HALF_WIDTH
    for i, (x0, x1, z0, z1) in enumerate(course.ramps):
        zb = min(z0, z1)
        verts = [(x0, -W, zb), (x0, W, zb), (x1, -W, zb), (x1, W, zb), (x0, -W, z0), (x0, W, z0), (x1, -W, z1), (x1, W, z1)]
        meshes.append(f'<mesh name="ramp_{i}" vertex="{" ".join(f"{v:.4f}" for p in verts for v in p)}"/>')
        geoms.append(f'<geom name="ramp_{i}" type="mesh" mesh="ramp_{i}" rgba="0.62 0.66 0.58 1" friction="1 0.005 0.0001"/>')
    for i, (tx, ty, _) in enumerate(course.targets):
        geoms.append(
            f'<geom name="target_{i}" type="cylinder" pos="{tx:.4f} {ty:.4f} {course.ground(tx) + 0.001:.4f}" '
            f'size="0.04 0.001" rgba="1 0.85 0.1 0.9" contype="0" conaffinity="0"/>'
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
    {chr(10).join('    ' + m for m in meshes)}
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    {chr(10).join('    ' + g for g in geoms)}
  </worldbody>
</mujoco>
"""
    path = MODEL_DIR / f"course_{name}.xml"
    path.write_text(xml, encoding="utf-8")
    return path


def plot_profile(course: Course, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [i * 0.01 for i in range(int(-100), int(course.end_x * 100))]
    zs = [course.ground(x) for x in xs]
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
    ax.set_xlabel("x [m]")
    ax.set_ylabel("height [m]")
    ax.set_ylim(-0.02, max(zs) + 0.25)
    ax.set_xlim(-1, course.end_x)
    ax.set_title("G1 test course (A flat · B slopes up/down · C single steps · D stairs up · E stairs down)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
