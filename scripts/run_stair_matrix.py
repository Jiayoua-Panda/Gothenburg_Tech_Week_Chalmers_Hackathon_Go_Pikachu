"""Run a controlled one-factor-at-a-time staircase geometry sweep.

The three start offsets are deterministic conditions, not random repetitions.
All runs use the same pretrained policy, 0.3 m/s forward command, and the same
MuJoCo truth-state centerline controller.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/stair-sweep"
RUNNER = ROOT / "scripts/run_stair_sweep.py"


def geometries() -> list[tuple[int, int, int]]:
    baseline = (15, 31, 160)
    cases = [baseline]
    cases += [(rise, 31, 160) for rise in (8, 10, 12, 18, 20)]
    cases += [(15, tread, 160) for tread in (20, 25, 35, 40)]
    cases += [(15, 31, width) for width in (70, 90, 110, 140)]
    return cases


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for rise, tread, width in geometries():
        for lateral in (-10, 0, 10):
            command = [
                sys.executable, str(RUNNER), "--rise-cm", str(rise),
                "--tread-cm", str(tread), "--width-cm", str(width),
                "--start-y-cm", str(lateral), "--output-dir", str(OUTPUT),
            ]
            result = subprocess.run(command, text=True, capture_output=True)
            if result.returncode:
                raise RuntimeError(f"Experiment failed: {command}\n{result.stdout}\n{result.stderr}")
            metadata = json.loads(result.stdout.splitlines()[-1])
            rows.append(metadata)
            print(f"{metadata['case']}: {metadata['outcome']} at {metadata['sim_time_s']:.2f}s", flush=True)

    fields = ["case", "rise_cm", "tread_cm", "usable_width_cm", "initial_lateral_offset_cm",
              "outcome", "sim_time_s", "max_x_m", "max_z_m", "goal_x_m", "top_height_m",
              "final_xyz_m", "scene", "scene_sha256", "checkpoint_sha256", "trajectory"]
    with (OUTPUT / "matrix.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fields})
    (OUTPUT / "matrix.json").write_text(json.dumps({
        "design": "one factor at a time; 14 geometries; 3 fixed lateral offsets per geometry",
        "offsets_cm": [-10, 0, 10],
        "speed_command_mps": 0.3,
        "policy_trained_by_this_project": False,
        "camera_perception_used": False,
        "center_correction_uses_simulator_ground_truth": True,
        "runs": rows,
    }, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
