"""Render selected success/failure examples from the fixed stair sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/stair-sweep"
RUNNER = ROOT / "scripts/run_stair_sweep.py"

# rise, tread, usable width, initial lateral offset, expected outcome, no center correction
HIGHLIGHTS = [
    (15, 31, 160, 0, "goal", False),
    (18, 31, 160, 0, "goal", False),
    (15, 31, 70, 0, "goal", False),
    (15, 40, 160, 0, "goal", False),
    (20, 31, 160, 0, "time_limit", False),
    (15, 25, 160, -10, "fall", False),
    (18, 31, 160, 10, "stair_exit", False),
    (15, 20, 160, -10, "time_limit", False),
    (15, 31, 160, 0, "stair_exit", True),
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index-only", action="store_true", help="Refresh video links after clips are rendered")
    args = parser.parse_args()
    rendered = []
    for rise, tread, width, lateral, expected, no_center in HIGHLIGHTS:
        if not args.index_only:
            command = [
                sys.executable, str(RUNNER), "--rise-cm", str(rise),
                "--tread-cm", str(tread), "--width-cm", str(width),
                "--start-y-cm", str(lateral), "--output-dir", str(OUTPUT), "--video",
            ]
            if no_center:
                command.append("--no-center")
            result = subprocess.run(command, text=True, capture_output=True)
            if result.returncode:
                raise RuntimeError(f"Video render failed: {command}\n{result.stdout}\n{result.stderr}")
            metadata = json.loads(result.stdout.splitlines()[-1])
        else:
            from run_stair_sweep import name_for
            case_id = name_for(rise, tread, width, lateral) + ("_no_center" if no_center else "")
            metadata = json.loads((OUTPUT / f"{case_id}.json").read_text())
        if metadata["outcome"] != expected:
            raise RuntimeError(f"Unexpected outcome for {metadata['case']}: {metadata['outcome']} != {expected}")
        if not metadata["video"] or not (OUTPUT / metadata["video"]).is_file():
            raise RuntimeError(f"Missing rendered video for {metadata['case']}")
        rendered.append({"case": metadata["case"], "outcome": metadata["outcome"], "video": metadata["video"]})
        print(f"{metadata['case']}: {metadata['outcome']} -> {metadata['video']}", flush=True)
    (OUTPUT / "highlights.json").write_text(json.dumps(rendered, ensure_ascii=False, indent=2) + "\n")

    matrix_path = OUTPUT / "matrix.json"
    matrix = json.loads(matrix_path.read_text())
    video_by_case = {item["case"]: item["video"] for item in rendered}
    for row in matrix["runs"]:
        row["video"] = video_by_case.get(row["case"])
        row["trajectory"] = f"{row['case']}.csv.gz"
        if row["video"]:
            video_metadata = json.loads((OUTPUT / f"{row['case']}.json").read_text())
            row["video_fps"] = video_metadata["video_fps"]
            row["video_frames"] = video_metadata["video_frames"]
    matrix_path.write_text(json.dumps(matrix, ensure_ascii=False, indent=2) + "\n")

    csv_path = OUTPUT / "matrix.csv"
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    if "video" not in fields:
        fields.append("video")
    for row in rows:
        row["video"] = video_by_case.get(row["case"], "")
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
