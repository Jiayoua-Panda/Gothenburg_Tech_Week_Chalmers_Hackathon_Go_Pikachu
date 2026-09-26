"""Aggregate factory-course run JSONs into matrix.csv and a Markdown results table."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import gzip
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from factory_course import COURSES, plot_course  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
FIELDS = ["run", "course", "stair_speed_mps", "stall_recovery", "align_before_stairs", "start_offset_cm", "start_yaw_deg",
          "loc_sigma_cm", "loc_yaw_sigma_deg", "loc_rate_hz", "loc_latency_ms", "loc_bias_cm", "loc_fusion", "seed", "outcome", "detail",
          "sim_time_s", "route_completion", "recoveries", "mean_abs_cross_track_cm", "max_abs_cross_track_cm",
          "video"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, default=ROOT / "artifacts/factory-course/runs")
    parser.add_argument("--output", type=Path, default=None, help="directory for matrix.csv / results.md")
    args = parser.parse_args()
    output = args.output or args.runs
    rows = []
    for path in sorted(args.runs.glob("*.json")):
        run = json.loads(path.read_text())
        loc = run["localization"]
        rows.append({
            "run": run["run"], "course": run["course"]["name"], "stair_speed_mps": run["stair_speed_mps"],
            "stall_recovery": run.get("stall_recovery", False),
            "align_before_stairs": run.get("align_before_stairs", False), "start_offset_cm": run["start_offset_cm"],
            "start_yaw_deg": run["start_yaw_deg"], "loc_sigma_cm": loc["sigma_xy_cm"],
            "loc_yaw_sigma_deg": loc["sigma_yaw_deg"], "loc_rate_hz": loc["rate_hz"],
            "loc_latency_ms": loc["latency_ms"], "loc_bias_cm": loc.get("bias_cm", 0.0),
            "loc_fusion": loc.get("odometry_fusion", False), "seed": loc["seed"], "outcome": run["outcome"],
            "detail": run["detail"], "sim_time_s": run["sim_time_s"], "route_completion": run["route_completion"],
            "recoveries": run.get("recoveries", 0), "mean_abs_cross_track_cm": run["mean_abs_cross_track_cm"],
            "max_abs_cross_track_cm": run["max_abs_cross_track_cm"], "video": run["video"],
        })
    output.mkdir(parents=True, exist_ok=True)
    with (output / "matrix.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        key = (row["course"], row["stair_speed_mps"], row["stall_recovery"], row["align_before_stairs"],
               row["loc_sigma_cm"],
               row["loc_yaw_sigma_deg"], row["loc_rate_hz"], row["loc_latency_ms"], row["loc_bias_cm"],
               row["loc_fusion"])
        groups[key].append(row)
    lines = ["| Course | Stair cmd | Supervisor | Localization | Goal | Other outcomes | Mean goal time | "
             "Mean completion | Max cross-track |",
             "| --- | ---: | :---: | --- | ---: | --- | ---: | ---: | ---: |"]
    for key in sorted(groups):
        course, speed, recovery, align, sigma, yaw_sigma, rate, latency, bias, fusion = key
        runs = groups[key]
        outcomes = Counter(r["outcome"] for r in runs)
        goals = [r for r in runs if r["outcome"] == "goal"]
        others = ", ".join(f"{n} {o.replace('_', ' ')}" for o, n in sorted(outcomes.items()) if o != "goal") or "—"
        loc = "ideal (ground truth, 50 Hz)" if not (sigma or yaw_sigma or latency or bias or rate < 50) else (
            f"σ {sigma:g} cm / {yaw_sigma:g}°, {rate:g} Hz, {latency:g} ms, bias {bias:g} cm"
            + (", + odometry" if fusion else ""))
        mean_time = f"{sum(r['sim_time_s'] for r in goals) / len(goals):.1f} s" if goals else "—"
        completion = sum(r["route_completion"] for r in runs) / len(runs)
        cross = max((r["max_abs_cross_track_cm"] or 0) for r in runs)
        supervisor = " + ".join(n for n, on in (("recovery", recovery), ("align", align)) if on) or "none"
        lines.append(f"| {course} | {speed:g} m/s | {supervisor} | {loc} | "
                     f"{len(goals)}/{len(runs)} | {others} | {mean_time} | {completion:.0%} | {cross:.0f} cm |")
    (output / "results.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    colours = {"goal": "#22a352", "fall": "#d9412b", "path_exit": "#e08a1e", "stuck": "#7a4fd1"}
    for key, runs in groups.items():
        course, speed, recovery, align, sigma, yaw_sigma, rate, latency, bias, fusion = key
        if sigma or yaw_sigma or latency or bias or rate < 50:
            continue
        traces, seen = [], set()
        for run in sorted(runs, key=lambda r: r["outcome"] != "goal"):
            with gzip.open(args.runs / f"{run['run']}.csv.gz", "rt") as handle:
                next(handle)
                points = [(float(v[1]), float(v[2])) for v in (line.split(",") for line in handle)][::5]
            label = {"goal": "reached goal"}.get(run["outcome"], run["outcome"].replace("_", " ")) if run["outcome"] not in seen else None
            seen.add(run["outcome"])
            traces.append((label, points, colours.get(run["outcome"], "0.3")))
        name = f"overview_{course}_v{round(speed * 100)}{'_rec' if recovery else ''}{'_align' if align else ''}.png"
        plot_course(COURSES[course](), output / name, traces)


if __name__ == "__main__":
    main()
