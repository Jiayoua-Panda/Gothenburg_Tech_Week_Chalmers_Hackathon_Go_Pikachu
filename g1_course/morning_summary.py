"""把整夜的结果汇总成一页：outputs/MORNING_SUMMARY.md（有什么结果就写什么，可以反复运行）。

对比四个控制器在同一套测试路段上的通过率：
    V0        Unitree 预训练策略（盲走）
    V0.5      高度扫描 + V0 + CMA-ES 调好的经典层（optimize.py --evaluate，10 次新试验）
    DWAQ      第三方楼梯策略 G1-DWAQ（全程）
    switch    高度扫描选技能：平地 V0，台阶 DWAQ
"""

from __future__ import annotations

import csv
import datetime as dt
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "outputs"
GROUPS = [
    ("Flat and 5° slopes", ["A warm-up", "B1 up 5deg", "B1 down 5deg"]),
    ("Rough floor 2–4 cm (3D)", ["G1 rough 2cm", "G2 rough 4cm"]),
    ("Downhill 10–15°", ["B2 down 10deg", "B3 down 15deg"]),
    ("Uphill 10–15°", ["B2 up 10deg", "B3 up 15deg"]),
    ("Cross slope 5–10° (3D)", ["F1 cross 5deg", "F2 cross 10deg"]),
    ("Single steps 5–15 cm", ["C1 step 5cm", "C2 step 10cm", "C3 step 15cm"]),
    ("Stairs up 10–15 cm", ["D1 up 10/30", "D2 up 15/25"]),
    ("Stairs down 10–15 cm", ["E1 down 15/30", "E2 down 10/30"]),
    ("Angled / narrow stairs (3D)", ["H1 stairs 15deg", "H2 stairs 30deg", "I narrow 0.6m"]),
]


def read(path):
    if not path.exists():
        return []
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def segments(ctrl):
    """{segment: (passes, trials)}，2D 和 3D 合并（3D 的 A warm-up 并入平地）。"""
    res = {}
    for p in (OUT / ctrl / "segment_results.csv", OUT / ctrl / "3d" / "segment_results.csv"):
        for r in read(p):
            n, k = int(r["trials"]), round(float(r["pass_rate"]) * int(r["trials"]))
            a, b = res.get(r["segment"], (0, 0))
            res[r["segment"]] = (a + k, b + n)
    return res


def hybrid():
    """V0.5：用第 1 阶段评估（第 2 阶段没有更好，见 outputs/hybrid/stage2/evaluation.csv）；记下 train / test。"""
    for p in (OUT / "hybrid" / "evaluation.csv", OUT / "hybrid" / "stage2" / "evaluation.csv"):
        rows = [r for r in read(p) if r["controller"] == "V0.5 hybrid"]
        if rows:
            return {r["segment"]: (round(float(r["pass_rate"]) * int(r["trials"])), int(r["trials"]), r["split"]) for r in rows}, p
    return {}, None


def cell(v):
    if not v:
        return "–"
    k, n = v[0], v[1]
    return f"{100 * k / n:.0f}% ({k}/{n})" + (" *test*" if len(v) > 2 and v[2] == "test" else "")


def group_cell(data, names):
    ks = [data[n][0] for n in names if n in data]
    ns = [data[n][1] for n in names if n in data]
    return f"{100 * sum(ks) / sum(ns):.0f}% ({sum(ks)}/{sum(ns)})" if ns else "–"


def status(log, done):
    p = OUT / log
    if not p.exists():
        return "not started"
    text = p.read_text(encoding="utf-8", errors="replace")
    return "done" if done in text else "running…"


def main():
    v0, dwaq, sw = segments("v0"), segments("dwaq"), segments("switch")
    hy, hy_src = hybrid()
    all_names = [n for _, names in GROUPS for n in names]
    lines = [
        "# Morning summary — SKF humanoid stairs",
        "",
        f"_Generated {dt.datetime.now():%Y-%m-%d %H:%M} by `g1_course/morning_summary.py` (re-run any time)._",
        "",
        "| Job | Status |",
        "|---|---|",
        f"| V0.5 overnight tuning + evaluation (`outputs/overnight.log`) | {status('overnight.log', 'ALL DONE')} |",
        f"| Stairs evaluation, DWAQ + switch (`outputs/stairs_eval.log`) | {status('stairs_eval.log', 'STAIRS EVAL DONE')} |",
        f"| Switch tour video (`outputs/switch/course_tour.mp4`) | {status('stairs_video.log', 'TOUR VIDEO DONE')} |",
        "",
        "## Pass rate by terrain group",
        "",
        "| Terrain | V0 (Unitree, blind) | V0.5 hybrid (ours) | G1-DWAQ only | **Switch: scan picks V0 / DWAQ** |",
        "|---|---|---|---|---|",
    ]
    for g, names in GROUPS:
        lines.append(f"| {g} | {group_cell(v0, names)} | {group_cell(hy, names)} | {group_cell(dwaq, names)} | "
                     f"**{group_cell(sw, names)}** |")
    lines += ["", "## Per segment", "",
              "| Segment | V0 | V0.5 | DWAQ | Switch |", "|---|---|---|---|---|"]
    for n in all_names:
        lines.append(f"| {n} | {cell(v0.get(n))} | {cell(hy.get(n))} | {cell(dwaq.get(n))} | {cell(sw.get(n))} |")
    tour = read(OUT / "switch" / "tour_results.csv")
    if tour:
        passed = sum(r["status"] == "PASS" for r in tour)
        lines += ["", f"**Switch tour (one continuous run over the whole 2D course):** {passed}/{len(tour)} segments passed "
                  "→ video `outputs/switch/course_tour.mp4`."]
    lines += [
        "",
        "## How to read this",
        "",
        f"- V0.5 numbers come from `{hy_src.relative_to(ROOT) if hy_src else '—'}`; segments marked *test* were never "
        "used for tuning (held-out).",
        "- DWAQ = third-party G1-DWAQ weights (G1DWAQ_Lab, BSD-3), **not trained by us**; we re-implemented its "
        "inference and run it on our robot model and course. Switch = our height scan decides when to use it.",
        "- All controllers steer to the walkway centre with simulator position (on a real robot: localisation).",
        "- V0 / DWAQ / switch: 10 randomised starts per segment (`run_course.py --mode segments`).",
    ]
    (OUT / "MORNING_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT / 'MORNING_SUMMARY.md'}")


if __name__ == "__main__":
    main()
