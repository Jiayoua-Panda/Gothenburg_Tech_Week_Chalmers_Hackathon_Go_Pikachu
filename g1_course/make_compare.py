"""三种状态同一路线对比视频：2×2 画面（1920×1080）。

    ① V0      | ② V0.5
    ③ switch  | 路线剖面 + 结果

先跑：py run_course.py --mode tour --course demo --controller {v0,hybrid,switch}
再跑：py make_compare.py
"""

from __future__ import annotations

import argparse
import csv
import pathlib

import cv2
import imageio.v2 as imageio
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "outputs"
W, H = 960, 540

# (控制器, 画面底部标签, 右下面板里的短名, 颜色)；安全层与 run_course.py --safety 对应
STAGES = [
    ("v0", "1  V0 pretrained  |  no safety layer", "1 V0", (200, 70, 60)),
    ("hybrid", "2  V0.5 tuned overnight  |  simple stop 0.8 m", "2 V0.5", (230, 150, 30)),
    ("switch", "3  Skill switch  |  ISO stop 1.67 m + hold", "3 Switch", (60, 170, 90)),
]


def label(img, text, color):
    cv2.rectangle(img, (0, H - 44), (W, H), color, -1)
    cv2.putText(img, text, (14, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def info_panel(course, results):
    panel = np.full((H, W, 3), 245, np.uint8)
    prof = imageio.imread(OUT / "v0" / course / "course_profile.png")[..., :3]
    prof = prof[: prof.shape[0] // 2]  # 只要上半部分（高度剖面）
    s = W / prof.shape[1]
    prof = cv2.resize(prof, (W, int(prof.shape[0] * s)), interpolation=cv2.INTER_AREA)
    panel[: prof.shape[0]] = prof
    y = prof.shape[0] + 50
    cv2.putText(panel, "Same course, same starts, three stages", (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 30), 2, cv2.LINE_AA)
    for i, (key, _, short, color) in enumerate(STAGES):
        n_pass, n, hit = results[key]
        yy = y + 50 + 44 * i
        cv2.rectangle(panel, (20, yy - 26), (44, yy - 2), color, -1)
        cv2.putText(panel, short, (56, yy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (30, 30, 30), 2, cv2.LINE_AA)
        text = f"{n_pass}/{n} segments passed" + ("   person HIT" if hit else "")
        cv2.putText(panel, text, (230, yy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
    cv2.putText(panel, "Single run each (tour). Pass rates over 10 random starts: see segment tests.",
                (20, H - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 90), 1, cv2.LINE_AA)
    return panel


def tour_result(key, course):
    with open(OUT / key / course / "tour_results.csv", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return sum(r["status"] == "PASS" for r in rows), len(rows), any(r["status"] == "HIT" for r in rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--course", default="demo")
    args = p.parse_args()

    readers = [imageio.get_reader(OUT / k / args.course / "course_tour.mp4") for k, *_ in STAGES]
    fps = readers[0].get_meta_data()["fps"]
    panel = info_panel(args.course, {k: tour_result(k, args.course) for k, *_ in STAGES})
    iters = [iter(r) for r in readers]
    last = [None] * 3
    path = OUT / f"compare_{args.course}.mp4"
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
            frame = np.vstack([np.hstack([cells[0], cells[1]]), np.hstack([cells[2], panel])])
            w.append_data(frame)
    for r in readers:
        r.close()
    print(f"对比视频：{path}")


if __name__ == "__main__":
    main()
