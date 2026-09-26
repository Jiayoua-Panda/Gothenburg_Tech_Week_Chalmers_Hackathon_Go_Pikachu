"""四种状态同一路线对比：2×2 视频 + 分段通过率热力图。

    ① V0      | ② V0.5
    ③ switch  | ④ combined

先跑：py run_course.py --mode tour --course demo --controller {v0,hybrid,switch} --safety {off,stop,iso}
      py combined.py --mode tour --course demo --safety iso
      （热力图另需各自的 --mode segments）
再跑：py make_compare4.py
"""

from __future__ import annotations

import argparse
import csv

import cv2
import imageio.v2 as imageio
import numpy as np

from make_compare import OUT, W, H, label, tour_result

STAGES = [
    ("v0", "1  V0 pretrained  |  no safety layer", "1 V0", (200, 70, 60)),
    ("hybrid", "2  V0.5 tuned overnight  |  simple stop 0.8 m", "2 V0.5", (230, 150, 30)),
    ("switch", "3  Skill switch  |  ISO stop  |  perfect position", "3 Switch", (60, 170, 90)),
    ("combined", "4  Switch + camera localization + supervisor  |  ISO stop", "4 Combined", (50, 110, 210)),
]


def summary_strip(course, results):
    strip = np.full((60, 2 * W, 3), 245, np.uint8)
    x = 20
    for key, _, short, color in STAGES:
        n_pass, n, hit = results[key]
        text = f"{short}: {n_pass}/{n}" + ("  HIT" if hit else "")
        cv2.rectangle(strip, (x, 20), (x + 22, 42), color, -1)
        cv2.putText(strip, text, (x + 32, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (30, 30, 30), 2, cv2.LINE_AA)
        x += 470
    return strip


def compare_video(course):
    readers = [imageio.get_reader(OUT / k / course / "course_tour.mp4") for k, *_ in STAGES]
    fps = readers[0].get_meta_data()["fps"]
    strip = summary_strip(course, {k: tour_result(k, course) for k, *_ in STAGES})
    iters = [iter(r) for r in readers]
    last = [None] * len(STAGES)
    path = OUT / f"compare4_{course}.mp4"
    with imageio.get_writer(path, fps=fps, codec="libx264", quality=8, macro_block_size=1) as w:
        while True:
            alive = False
            for i, it in enumerate(iters):
                try:
                    last[i] = next(it)
                    alive = True
                except StopIteration:
                    pass
            if not alive:
                break
            cells = [label(cv2.resize(f, (W, H)).copy(), t, c) for f, (_, t, _, c) in zip(last, STAGES)]
            w.append_data(np.vstack([np.hstack(cells[:2]), np.hstack(cells[2:]), strip]))
    for r in readers:
        r.close()
    print(f"对比视频：{path}")
    small = compress(path)
    print(f"压缩版（2 倍速，1280 宽，放 PPT 用）：{small}")


def compress(path, speed=2.0, width=1280, crf=28):
    import subprocess

    import imageio_ffmpeg

    out = path.with_name(f"{path.stem}_{speed:g}x.mp4")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-i", str(path),
                    "-filter:v", f"setpts=PTS/{speed},scale={width}:-2", "-an", "-c:v", "libx264", "-crf", str(crf),
                    "-preset", "veryfast", "-pix_fmt", "yuv420p", str(out)], check=True)
    return out


def safety_pass_rate(mode):
    """安全演示（run_course.py --mode safety，50 个行人横穿场景）里"行驶中没撞到人"的比例 [%]。"""
    with open(OUT / "v0" / "safety" / "safety_summary.csv", encoding="utf-8") as f:
        row = next(r for r in csv.DictReader(f) if r["mode"] == mode)
    return 100 * (1 - int(row["contact_while_driving"]) / int(row["trials"]))


def pass_rate_heatmap(course, person_safety=None):
    """person_safety：{阶段: 安全模式}。给了就把行人段换成安全演示的结果（分段测试里那一段没有人）。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    table, segments = [], None
    for key, *_ in STAGES:
        with open(OUT / key / course / "segment_results.csv", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        segments = [r["segment"] for r in rows]
        vals = [100 * float(r["pass_rate"]) for r in rows]
        if person_safety:
            for j, s in enumerate(segments):
                if s.startswith("S "):
                    vals[j] = safety_pass_rate(person_safety[key])
        table.append(vals)
    table = np.array(table)
    fig, ax = plt.subplots(figsize=(13, 3.4))
    im = ax.imshow(table, cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
    for i in range(table.shape[0]):
        for j in range(table.shape[1]):
            ax.text(j, i, f"{table[i, j]:.0f}", ha="center", va="center", fontsize=9,
                    color="white" if table[i, j] < 25 or table[i, j] > 85 else "black")
    if person_safety:
        shown = [f"{s} *" if s.startswith("S ") else s for s in segments]
        note = "* person column: 50 person-crossing runs from the safety test, % with no contact while the robot is driving"
    else:
        # 分段测试里行人段只是一段平地（人只在巡回视频里出现），标出来免得误读成"避让行人"
        shown = [f"{s} (flat*)" if s.startswith("S ") else s for s in segments]
        note = "* no person in the segment test; person crossing is tested in the tour video"
    ax.set_xticks(range(len(segments)), shown, rotation=30, ha="right", fontsize=9)
    ax.set_yticks(range(len(STAGES)), [f"{s[2]}  ({t.mean():.0f}%)" for s, t in zip(STAGES, table)])
    ax.set_title("Pass rate per segment [%], same course, 10 random starts each (localization errors random too)\n"
                 + note, fontsize=10)
    fig.colorbar(im, ax=ax, fraction=0.02, pad=0.01)
    fig.tight_layout()
    path = OUT / f"compare4_{course}_pass_rate.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"热力图：{path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--course", default="demo")
    p.add_argument("--only", choices=["video", "heatmap"], default=None)
    args = p.parse_args()
    if args.only != "video":
        pass_rate_heatmap(args.course)
    if args.only != "heatmap":
        compare_video(args.course)


if __name__ == "__main__":
    main()
