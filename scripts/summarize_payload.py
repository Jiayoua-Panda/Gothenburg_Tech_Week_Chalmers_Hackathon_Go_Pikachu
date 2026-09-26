"""Summarize the box-carrying (payload) sweep: goal rate vs box mass per course and carry configuration."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
# validated categorical slots 1-4; slots 3-4 are below 3:1 contrast, so every series also has its own
# marker and the numbers are in results.md (which also lists the far hold and real-arm-observation runs)
SERIES = [
    (("near", "virtual", 0.3), "near hold, virtual arms, 0.3 m/s corridors", "#2a78d6", "o"),
    (("near", "virtual", 0.5), "near hold, virtual arms, 0.5 m/s corridors", "#eb6834", "s"),
    (("chest", "-", 0.5), "box strapped to chest, arms free, 0.5 m/s", "#1baf7a", "^"),
    (("back", "-", 0.5), "box on back, arms free, 0.5 m/s", "#eda100", "D"),
]
COURSES = [("l1_corner", "L1 corner (flat)"), ("l2_corner_stairs", "L2 corner + 10 risers up"),
           ("l4_factory_route", "L4 full route (24 risers, 5 turns)")]


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, default=ROOT / "artifacts/factory-course/payload-sweep")
    args = parser.parse_args()
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for path in sorted(args.runs.glob("*.json")):
        run = json.loads(path.read_text())
        mount = run.get("payload_mount")
        hold = mount if mount in ("chest", "back") else (run.get("carry_pose") or "arms free")
        key = (run["course"]["name"], hold, run.get("arm_observation") or "-",
               run["flat_speed_mps"], run.get("payload_kg"))
        groups[key].append(run)

    rows = []
    for (course, hold, obs, speed, mass), runs in sorted(groups.items(), key=lambda kv: tuple(
            (v if v is not None else -1) for v in kv[0])):
        outcomes = Counter(r["outcome"] for r in runs)
        goals = outcomes.get("goal", 0)
        low, high = wilson(goals, len(runs))
        times = [r["sim_time_s"] for r in runs if r["outcome"] == "goal"]
        rows.append({"course": course, "hold": hold, "arm_observation": obs, "corridor_speed_mps": speed,
                     "payload_kg": "" if mass is None else mass, "runs": len(runs), "goals": goals,
                     "ci95_low": round(low, 3), "ci95_high": round(high, 3),
                     "path_exit": outcomes.get("path_exit", 0), "stuck": outcomes.get("stuck", 0),
                     "fall": outcomes.get("fall", 0), "wrong_stop": outcomes.get("wrong_stop", 0),
                     "time_limit": outcomes.get("time_limit", 0),
                     "mean_goal_time_s": round(sum(times) / len(times), 1) if times else ""})
    with (args.runs / "summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = ["| Course | Hold | Arm observation | Corridor speed | Box | Goal | 95% interval | Path exit | Stuck | "
             "Fall | Wrong stop | Time limit | Mean time |",
             "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for r in rows:
        box = "none (arms free)" if r["payload_kg"] == "" else f"{r['payload_kg']:g} kg"
        lines.append(f"| {r['course']} | {r['hold']} | {r['arm_observation']} | {r['corridor_speed_mps']:g} m/s | {box} | "
                     f"{r['goals']}/{r['runs']} | {r['ci95_low']:.0%}–{r['ci95_high']:.0%} | {r['path_exit']} | "
                     f"{r['stuck']} | {r['fall']} | {r['wrong_stop']} | {r['time_limit']} | "
                     f"{r['mean_goal_time_s'] or '—'} {'s' if r['mean_goal_time_s'] else ''} |")
    (args.runs / "results.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    plot(rows, args.runs / "payload_tolerance.png")


def plot(rows: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2,
                         "ytick.color": INK_2, "text.color": INK})
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6), dpi=150, facecolor=SURFACE, sharey=True)
    for ax, (course, title) in zip(axes, COURSES):
        ax.set_facecolor(SURFACE)
        for (hold, obs, speed), label, colour, marker in SERIES:
            points = sorted((r for r in rows if r["course"] == course and r["hold"] == hold
                             and r["arm_observation"] == obs and r["corridor_speed_mps"] == speed
                             and r["payload_kg"] != ""), key=lambda r: r["payload_kg"])
            if not points:
                continue
            xs = [r["payload_kg"] for r in points]
            ys = [100 * r["goals"] / r["runs"] for r in points]
            ax.plot(xs, ys, color=colour, linewidth=2, marker=marker, markersize=6, markeredgecolor=SURFACE,
                    markeredgewidth=1.2, label=label)
        n = next((r["runs"] for r in rows if r["course"] == course and r["payload_kg"] != ""), 0)
        ax.set_title(f"{title} — {n} runs per point", loc="left", fontsize=9.5, fontweight="bold", color=INK)
        ax.set_xlabel("box mass (kg)")
        ax.set_xticks([0, 2, 5, 8, 12, 15])
        ax.set_ylim(-3, 103)
        ax.set_yticks([0, 25, 50, 75, 100], ["0%", "25%", "50%", "75%", "100%"])
        ax.grid(axis="y", color=GRID, linewidth=0.6)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("routes completed")
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, frameon=False, fontsize=8)
    fig.suptitle("Carrying a box with the unmodified G1-DWAQ stair policy: goal rate vs box mass "
                 "(held in locked arms, or strapped to chest/back with arms free)", x=0.01, ha="left", fontsize=10.5,
                 color=INK)
    fig.tight_layout(rect=(0, 0.1, 1, 0.95))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


if __name__ == "__main__":
    main()
