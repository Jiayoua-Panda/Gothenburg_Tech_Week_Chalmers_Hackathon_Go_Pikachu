"""Factory route courses built from straight corridors, stair flights and corner landings.

A course is described like a turtle walk along the walkway centreline:
`flat(length)`, `stairs(steps, rise, tread, up)` and `turn(degrees)`. The builder
produces both the MuJoCo geometry and the route (segment list) that the
path-following controller in `run_factory_course.py` tracks. Turns always happen
on a flat landing, never on a stair flight.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from pathlib import Path


@dataclass
class Segment:
    kind: str                       # "flat", "stairs_up", "stairs_down"
    start: tuple[float, float]
    heading_deg: float
    length: float
    width: float
    z_start: float
    z_end: float
    steps: int = 0
    rise: float = 0.0
    tread: float = 0.0

    @property
    def direction(self) -> tuple[float, float]:
        a = math.radians(self.heading_deg)
        return math.cos(a), math.sin(a)

    @property
    def end(self) -> tuple[float, float]:
        dx, dy = self.direction
        return self.start[0] + dx * self.length, self.start[1] + dy * self.length

    def surface_z(self, s: float) -> float:
        """Walking-surface height at arc length `s` along this segment."""
        if self.kind == "flat" or s < 0:
            return self.z_start
        if s >= self.length:
            return self.z_end
        index = min(self.steps, int(s // self.tread) + 1)
        if self.kind == "stairs_up":
            return self.z_start + index * self.rise
        return self.z_start - (index - 1) * self.rise


@dataclass
class Box:
    name: str
    center: tuple[float, float, float]
    half: tuple[float, float, float]
    material: str
    collide: bool = True


@dataclass
class Course:
    name: str
    description: str
    width: float
    segments: list[Segment] = field(default_factory=list)
    boxes: list[Box] = field(default_factory=list)

    @property
    def total_length(self) -> float:
        return sum(segment.length for segment in self.segments)

    @property
    def goal(self) -> tuple[float, float, float]:
        last = self.segments[-1]
        return (*last.end, last.z_end)

    def summary(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "width_m": self.width,
            "path_length_m": round(self.total_length, 3),
            "turns": sum(1 for a, b in zip(self.segments, self.segments[1:]) if a.heading_deg != b.heading_deg),
            "stair_flights": sum(1 for s in self.segments if s.kind != "flat"),
            "risers": sum(s.steps for s in self.segments),
            "max_height_m": round(max(max(s.z_start, s.z_end) for s in self.segments), 3),
            "goal_xyz_m": [round(v, 3) for v in self.goal],
            "segments": [asdict(s) for s in self.segments],
        }


class CourseBuilder:
    def __init__(self, name: str, description: str, width: float = 1.2):
        self.course = Course(name, description, width)
        self.x, self.y, self.heading, self.z = 0.0, 0.0, 0.0, 0.0
        self._count = 0
        self._last_flat = math.inf

    def _box(self, prefix: str, cx: float, cy: float, along: float, across: float,
             top: float, material: str) -> None:
        """Axis-aligned solid block from the ground to `top` (or a thin floor marking)."""
        self._count += 1
        horizontal = abs(math.cos(math.radians(self.heading))) > 0.5
        hx, hy = (along / 2, across / 2) if horizontal else (across / 2, along / 2)
        if top <= 1e-6:
            self.course.boxes.append(Box(f"{prefix}_{self._count}", (cx, cy, 0.002), (hx, hy, 0.002),
                                         "walkway", collide=False))
        else:
            self.course.boxes.append(Box(f"{prefix}_{self._count}", (cx, cy, top / 2), (hx, hy, top / 2), material))

    def flat(self, length: float) -> "CourseBuilder":
        dx, dy = math.cos(math.radians(self.heading)), math.sin(math.radians(self.heading))
        segment = Segment("flat", (self.x, self.y), self.heading, length, self.course.width, self.z, self.z)
        self._box("floor", self.x + dx * length / 2, self.y + dy * length / 2, length, self.course.width,
                  self.z, "landing")
        self.course.segments.append(segment)
        self.x, self.y = segment.end
        self._last_flat = length
        return self

    def stairs(self, steps: int, rise: float, tread: float, up: bool = True) -> "CourseBuilder":
        if self._last_flat < self.course.width / 2 - 1e-9:
            raise ValueError("a stair flight needs at least half a walkway width of flat landing before it")
        dx, dy = math.cos(math.radians(self.heading)), math.sin(math.radians(self.heading))
        z_end = self.z + steps * rise if up else self.z - steps * rise
        if z_end < -1e-9:
            raise ValueError("stairs would go below the ground floor")
        segment = Segment("stairs_up" if up else "stairs_down", (self.x, self.y), self.heading,
                          steps * tread, self.course.width, self.z, z_end, steps, rise, tread)
        for index in range(1, steps + 1):
            top = self.z + index * rise if up else self.z - (index - 1) * rise
            s = (index - 0.5) * tread
            self._box("step", self.x + dx * s, self.y + dy * s, tread, self.course.width, top, "stairs")
        self.course.segments.append(segment)
        self.x, self.y = segment.end
        self.z = z_end
        self._last_flat = 0.0
        return self

    def turn(self, degrees: float) -> "CourseBuilder":
        """Turn on the spot at the current centreline point; adds a square corner landing."""
        w = self.course.width
        if self._last_flat < w / 2 - 1e-9:
            raise ValueError("a corner needs at least half a walkway width of flat landing before it")
        self._box("corner", self.x, self.y, w, w, self.z, "landing")
        self.heading = (self.heading + degrees) % 360
        self._last_flat = 0.0
        return self

    def build(self) -> Course:
        last = self.course.segments[-1]
        dx, dy = last.direction
        self.course.boxes.append(Box("goal_pad", (last.end[0] - dx * 0.3, last.end[1] - dy * 0.3, last.z_end + 0.003),
                                     (0.3, 0.3, 0.003), "goal", collide=False))
        first = self.course.segments[0]
        fx, fy = first.direction
        z0 = first.z_start
        back = (0.3 * abs(fx) + self.course.width / 2 * abs(fy), 0.3 * abs(fy) + self.course.width / 2 * abs(fx))
        if z0 > 0:  # the robot starts on a raised floor: extend it behind the start so no foot is on the edge
            self.course.boxes.append(Box("start_apron", (-0.3 * fx, -0.3 * fy, z0 / 2), (*back, z0 / 2), "landing"))
        self.course.boxes.append(Box("start_pad", (0.0, 0.0, z0 + 0.003), (0.3, 0.3, 0.003), "start", collide=False))
        return self.course


RISE, TREAD, WIDTH = 0.15, 0.31, 1.2


def course_l1_corner() -> Course:
    return (CourseBuilder("l1_corner", "Ground-floor corridor with one 90° left corner", WIDTH)
            .flat(3.0).turn(90).flat(3.0).build())


def course_l2_corner_stairs() -> Course:
    return (CourseBuilder("l2_corner_stairs",
                          "Corner, straight 10-riser flight (15/31 cm), landing, right corner, mezzanine corridor",
                          WIDTH)
            .flat(2.0).turn(90).flat(1.5).stairs(10, RISE, TREAD).flat(1.5).turn(-90).flat(3.0).build())


def course_l3_switchback() -> Course:
    return (CourseBuilder("l3_switchback",
                          "Switchback stair: 6 risers, 180° U-turn landing, 6 risers, then corner and corridor",
                          WIDTH)
            .flat(2.0).stairs(6, RISE, TREAD).flat(0.6).turn(90).flat(WIDTH).turn(90).flat(0.6)
            .stairs(6, RISE, TREAD).flat(0.6).turn(-90).flat(3.0).build())


def course_l4_factory_route() -> Course:
    return (CourseBuilder("l4_factory_route",
                          "Full route: corridor, switchback up to mezzanine, two corners, straight flight down, "
                          "ground-floor corridor to goal", WIDTH)
            .flat(2.0).stairs(6, RISE, TREAD).flat(0.6).turn(90).flat(WIDTH).turn(90).flat(0.6)
            .stairs(6, RISE, TREAD).flat(0.6).turn(-90).flat(2.5).turn(-90).flat(1.0)
            .stairs(12, RISE, TREAD, up=False).flat(2.0).turn(90).flat(2.0).build())


def course_down_only() -> Course:
    """Diagnostic: start on a mezzanine and walk down one straight flight."""
    builder = CourseBuilder("diag_down", "Diagnostic: 1.5 m mezzanine, straight 10-riser descent", WIDTH)
    builder.z = 10 * RISE
    return builder.flat(2.0).stairs(10, RISE, TREAD, up=False).flat(2.0).build()


def course_turn_in_place() -> Course:
    return (CourseBuilder("diag_uturn", "Diagnostic: ground-floor 180° U-turn", WIDTH)
            .flat(2.0).turn(90).flat(WIDTH).turn(90).flat(2.0).build())


COURSES = {
    "l1_corner": course_l1_corner,
    "l2_corner_stairs": course_l2_corner_stairs,
    "l3_switchback": course_l3_switchback,
    "l4_factory_route": course_l4_factory_route,
    "diag_down": course_down_only,
    "diag_uturn": course_turn_in_place,
}


MATERIALS = {
    "stairs": "0.62 0.64 0.66 1",
    "landing": "0.45 0.50 0.56 1",
    "walkway": "0.30 0.34 0.40 1",
    "goal": "0.20 0.75 0.35 1",
    "start": "0.25 0.50 0.90 1",
}


def scene_xml(course: Course) -> str:
    xs = [b.center[0] for b in course.boxes] + [0.0]
    ys = [b.center[1] for b in course.boxes] + [0.0]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    extent = max(max(xs) - min(xs), max(ys) - min(ys), 4.0)
    materials = "\n".join(f'    <material name="{n}" rgba="{c}" />' for n, c in MATERIALS.items())
    geoms = []
    for box in course.boxes:
        contact = "" if box.collide else ' contype="0" conaffinity="0"'
        geoms.append(
            f'    <geom name="{box.name}" type="box" size="{box.half[0]:.4f} {box.half[1]:.4f} {box.half[2]:.4f}" '
            f'pos="{box.center[0]:.4f} {box.center[1]:.4f} {box.center[2]:.4f}" material="{box.material}"{contact} />'
        )
    return f"""<mujoco model="factory course {course.name}">
  <include file="g1_29dof_rev_1_0_daf.xml" />
  <statistic center="{cx:.2f} {cy:.2f} 0.8" extent="{extent:.1f}" />
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
    <rgba haze="0.15 0.25 0.35 1" />
    <global offwidth="1280" offheight="720" />
  </visual>
  <asset>
{materials}
  </asset>
  <worldbody>
    <light pos="{cx:.2f} {cy:.2f} 6" dir="0 0 -1" directional="true" />
{chr(10).join(geoms)}
  </worldbody>
</mujoco>
"""


def plot_course(course: Course, path: Path, traces: list[tuple[str, list[tuple[float, float]], str]] = ()) -> None:
    """Top-down map of the course, optionally with robot pelvis traces (label, xy points, colour)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
    top = max(max(s.z_start, s.z_end) for s in course.segments) or 1.0
    cmap = plt.get_cmap("Blues")
    for box in sorted(course.boxes, key=lambda b: b.center[2] + b.half[2]):
        if box.name.startswith(("goal", "start")):
            continue
        height = box.center[2] + box.half[2] if box.collide else 0.0
        ax.add_patch(Rectangle((box.center[0] - box.half[0], box.center[1] - box.half[1]),
                               2 * box.half[0], 2 * box.half[1],
                               facecolor=cmap(0.15 + 0.7 * height / top), edgecolor="0.4", linewidth=0.4))
    cx = [course.segments[0].start[0]] + [s.end[0] for s in course.segments]
    cy = [course.segments[0].start[1]] + [s.end[1] for s in course.segments]
    ax.plot(cx, cy, "--", color="0.35", linewidth=1, label="planned centreline")
    ax.plot(*course.segments[0].start, "o", color="#2f6fe0", markersize=8, label="start")
    ax.plot(*course.goal[:2], "*", color="#22a352", markersize=14, label="goal")
    for label, points, colour in traces:
        if points:
            ax.plot([p[0] for p in points], [p[1] for p in points], color=colour, linewidth=1.2, alpha=0.8,
                    label=label)
            ax.plot(*points[-1], "x", color=colour, markersize=6)
    for segment in course.segments:
        if segment.kind != "flat":
            mx = (segment.start[0] + segment.end[0]) / 2
            my = (segment.start[1] + segment.end[1]) / 2
            arrow = "↑" if segment.kind == "stairs_up" else "↓"
            ax.annotate(f"{arrow} {segment.steps}×{segment.rise * 100:.0f} cm", (mx, my), ha="center",
                        va="center", fontsize=7, color="0.1")
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.margins(0.08)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{course.name}: {course.total_length:.1f} m path, surface shaded by height (max {top:.2f} m)",
                 fontsize=9)
    ax.legend(loc="best", fontsize=7)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Write course scenes, maps and route summaries")
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "artifacts/factory-course/courses")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, factory in COURSES.items():
        course = factory()
        (args.output_dir / f"{name}.xml").write_text(scene_xml(course))
        (args.output_dir / f"{name}.json").write_text(json.dumps(course.summary(), indent=2) + "\n")
        plot_course(course, args.output_dir / f"{name}.png")
        print(name, json.dumps({k: v for k, v in course.summary().items() if k != "segments"}))
