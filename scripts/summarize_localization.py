"""Summarize the camera-localization error sweep on the L4 factory route.

Each condition changes one property of the simulated camera system relative to the ideal
(ground-truth, 50 Hz) pose, or applies a combined camera preset. Reports goal rate with a
Wilson 95% interval, writes sweep.csv / results.md and a small-multiples chart.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CAMERA_ONLY, FUSED = "#2a78d6", "#eb6834"  # validated categorical slots 1-2 (light surface)
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
PRESETS = {
    (2.0, 1.0, 30.0, 100.0, 2.0): "good",
    (5.0, 3.0, 10.0, 200.0, 5.0): "typical",
    (10.0, 5.0, 5.0, 300.0, 10.0): "poor",
}
SWEEPS = [  # (family, index in key, axis label)
    ("position noise", 0, "position noise σ (cm)"),
    ("heading noise", 1, "heading noise σ (°)"),
    ("calibration bias", 4, "constant position bias (cm)"),
    ("update rate", 2, "camera update rate (Hz)"),
    ("latency", 3, "camera latency (ms)"),
]
IDEAL = (0.0, 0.0, 50.0, 0.0, 0.0)


def wilson(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return 0.0, 0.0
    p = successes / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def classify(key: tuple) -> tuple[str, float | str]:
    params = key[:5]
    if params == IDEAL:
        return "ideal", 0.0
    if params in PRESETS:
        return "preset", PRESETS[params]
    changed = [i for i, (value, ideal) in enumerate(zip(params, IDEAL)) if value != ideal]
    assert len(changed) == 1, key
    family = next(name for name, index, _ in SWEEPS if index == changed[0])
    return family, params[changed[0]]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, default=ROOT / "artifacts/factory-course/localization-sweep")
    args = parser.parse_args()

    groups: dict[tuple, list[dict]] = defaultdict(list)
    for path in sorted(args.runs.glob("*.json")):
        run = json.loads(path.read_text())
        loc = run["localization"]
        key = (loc["sigma_xy_cm"], loc["sigma_yaw_deg"], loc["rate_hz"], loc["latency_ms"], loc.get("bias_cm", 0.0),
               bool(loc.get("odometry_fusion")))
        groups[key].append(run)

    rows = []
    for key, runs in groups.items():
        family, level = classify(key)
        outcomes = Counter(r["outcome"] for r in runs)
        goals = outcomes.get("goal", 0)
        low, high = wilson(goals, len(runs))
        times = [r["sim_time_s"] for r in runs if r["outcome"] == "goal"]
        rows.append({
            "family": family, "level": level, "odometry_fusion": key[5],
            "sigma_xy_cm": key[0], "sigma_yaw_deg": key[1], "rate_hz": key[2], "latency_ms": key[3],
            "bias_cm": key[4], "runs": len(runs), "goals": goals, "goal_rate": goals / len(runs),
            "ci95_low": round(low, 3), "ci95_high": round(high, 3),
            "path_exit": outcomes.get("path_exit", 0), "fall": outcomes.get("fall", 0),
            "stuck": outcomes.get("stuck", 0), "wrong_stop": outcomes.get("wrong_stop", 0),
            "mean_goal_time_s": round(sum(times) / len(times), 1) if times else None,
            "mean_route_completion": round(sum(r["route_completion"] for r in runs) / len(runs), 3),
        })
    order = {"ideal": 0, **{name: i + 1 for i, (name, _, _) in enumerate(SWEEPS)}, "preset": 9}
    preset_order = {"good": 0, "typical": 1, "poor": 2}
    rows.sort(key=lambda r: (order[r["family"]], r["odometry_fusion"],
                             preset_order.get(r["level"], 0) if r["family"] == "preset" else r["level"]))
    with (args.runs / "sweep.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    lines = ["| Varied property | Value | Odometry fusion | Goal | 95% interval | Path exit | Fall | Stuck | "
             "Wrong stop | Mean time |",
             "| --- | ---: | :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for r in rows:
        level = r["level"] if isinstance(r["level"], str) else f"{r['level']:g}"
        lines.append(f"| {r['family']} | {level} | {'yes' if r['odometry_fusion'] else 'no'} | "
                     f"{r['goals']}/{r['runs']} | {r['ci95_low']:.0%}–{r['ci95_high']:.0%} | {r['path_exit']} | "
                     f"{r['fall']} | {r['stuck']} | {r['wrong_stop']} | "
                     f"{r['mean_goal_time_s'] if r['mean_goal_time_s'] is not None else '—'} s |")
    (args.runs / "results.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    plot(rows, args.runs / "localization_tolerance.png")


def plot(rows: list[dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2,
                         "ytick.color": INK_2, "text.color": INK, "axes.titlecolor": INK})
    ideal = next(r for r in rows if r["family"] == "ideal")
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), dpi=150, facecolor=SURFACE)
    for ax, (family, _, label) in zip(axes.flat, SWEEPS):
        ax.set_facecolor(SURFACE)
        for fused, colour, name in ((False, CAMERA_ONLY, "camera only"), (True, FUSED, "camera + odometry")):
            points = [r for r in rows if r["family"] == family and r["odometry_fusion"] == fused]
            if not points:
                continue
            base = dict(ideal, level=50.0 if family == "update rate" else 0.0)
            points = sorted([base] + points, key=lambda r: r["level"])
            xs = [r["level"] for r in points]
            ys = [100 * r["goal_rate"] for r in points]
            ax.fill_between(xs, [100 * r["ci95_low"] for r in points], [100 * r["ci95_high"] for r in points],
                            color=colour, alpha=0.12, linewidth=0)
            ax.plot(xs, ys, color=colour, linewidth=2, marker="o", markersize=5, markeredgecolor=SURFACE,
                    markeredgewidth=1.5, label=name)
        if family == "update rate":
            ax.set_xscale("log")
            ax.invert_xaxis()
            ax.set_xticks([50, 20, 10, 5, 2, 1], ["50", "20", "10", "5", "2", "1"])
        if family in ("update rate", "latency"):
            ax.legend(frameon=False, fontsize=8, loc="lower left")
        ax.set_title(family, loc="left", fontsize=10, fontweight="bold")
        ax.set_xlabel(label)
        ax.set_ylim(-3, 103)
        ax.set_yticks([0, 25, 50, 75, 100], ["0%", "25%", "50%", "75%", "100%"])
        ax.grid(axis="y", color=GRID, linewidth=0.6)
        ax.spines[["top", "right"]].set_visible(False)
    axes.flat[0].set_ylabel("routes completed (of 25)")
    axes.flat[3].set_ylabel("routes completed (of 25)")

    ax = axes.flat[5]
    ax.set_facecolor(SURFACE)
    presets = ["good", "typical", "poor"]
    width = 0.36
    for offset, fused, colour, name in ((-width / 2 - 0.01, False, CAMERA_ONLY, "camera only"),
                                        (width / 2 + 0.01, True, FUSED, "camera + odometry")):
        values = [next(r for r in rows if r["family"] == "preset" and r["level"] == p
                       and r["odometry_fusion"] == fused) for p in presets]
        xs = [i + offset for i in range(3)]
        ax.bar(xs, [100 * v["goal_rate"] for v in values], width=width, color=colour, label=name)
        for x, v in zip(xs, values):
            ax.text(x, 100 * v["goal_rate"] + 2, f"{v['goals']}", ha="center", fontsize=8, color=INK_2)
    ax.set_xticks(range(3), ["good\n2 cm, 1°\n30 Hz, 100 ms", "typical\n5 cm, 3°\n10 Hz, 200 ms",
                             "poor\n10 cm, 5°\n5 Hz, 300 ms"], fontsize=7.5)
    ax.set_title("combined camera presets", loc="left", fontsize=10, fontweight="bold")
    ax.set_ylim(0, 125)
    ax.set_yticks([0, 25, 50, 75, 100], ["0%", "25%", "50%", "75%", "100%"])
    ax.grid(axis="y", color=GRID, linewidth=0.6)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=8, loc="upper right")

    fig.suptitle("L4 factory route (19.9 m, 5 turns, 24 risers): goal rate vs camera localization error — "
                 "25 runs per point, shaded 95% interval", x=0.01, ha="left", fontsize=10.5, color=INK)
    fig.text(0.01, 0.005, f"Ideal camera (ground truth, 50 Hz): {ideal['goals']}/25. Presets also include a constant "
             "bias of 2 / 5 / 10 cm. Simulation only; G1-DWAQ blind stair policy with route follower, stall recovery "
             "and square-up before stairs.", fontsize=7.5, color=INK_2)
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


if __name__ == "__main__":
    main()
