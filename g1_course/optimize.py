"""在 CPU 上调 V0.5 混合控制器（hybrid.py）的参数：CMA-ES，训练集 / 测试集分开。

用法：
    py optimize.py --probe                    # 默认参数 vs V0，训练路段各 4 次（几分钟）
    py optimize.py --generations 60           # 整夜跑；每代写 outputs/hybrid/es_log.csv 和 best_params.json
    py optimize.py --evaluate --trials 10     # 用 best_params.json 跑训练集 + 测试集，和 V0 对比
    第 2 阶段（课程：5 cm → 10 cm，从第 1 阶段最优参数热启动，结果写到 stage2/）：
    py optimize.py --out outputs/hybrid/stage2 --init outputs/hybrid/best_params.json \
        --train "A warm-up,C1 step 5cm,C2 step 10cm,D1 up 10/30,H1 stairs 15deg" --weights "C2 step 10cm=2,D1 up 10/30=2"

适应度 = 各训练路段平均 (通过率 + 0.5 × 前进比例)；平地 A 一旦失败重罚（不许为了台阶牺牲平地）。
同一代所有候选用同一组随机起点（公平比较），不同代换一组（避免对某组起点过拟合）。
"""

from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import pathlib
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "g1_step_playback"))

import course as C  # noqa: E402
import hybrid as H  # noqa: E402

OUT = ROOT / "outputs" / "hybrid"
TRAIN = ["A warm-up", "C1 step 5cm", "C2 step 10cm", "D1 up 10/30", "H1 stairs 15deg", "F1 cross 5deg", "B2 up 10deg"]
TEST = ["C3 step 15cm", "D2 up 15/25", "E1 down 15/30", "E2 down 10/30", "B3 up 15deg", "B1 up 5deg",
        "F2 cross 10deg", "G1 rough 2cm", "G2 rough 4cm", "H2 stairs 30deg", "I narrow 0.6m"]
ALL_SEGMENTS = TRAIN + TEST
WEIGHTS: dict = {}  # 路段权重（--weights），默认 1
TIME_LIMIT = 35.0

_SIMS: dict = {}


def spec_by_name(name):
    for spec in C.FULL_COURSE + C.COURSE_3D:
        if spec.get("name") == name:
            return spec
    raise KeyError(name)


def _get_sim(name):
    """每个进程每个路段一个仿真（场景文件名带 pid，避免并行写同一个 XML）。"""
    if name not in _SIMS:
        import run_course as R
        from play_g1 import G1Policy

        course = C.standalone(spec_by_name(name))
        tag = f"es_{os.getpid()}_{len(_SIMS)}"
        sim = R.CourseSim(G1Policy(R.POLICY_DIR), C.write_scene(course, tag), course)
        _, x0, x1, _ = course.segments[0]
        _SIMS[name] = (sim, x0, x1)
    return _SIMS[name]


def run_one(job):
    """一次试验：job = (params 或 None 表示 V0, 路段名, 随机种子)。"""
    import run_course as R

    params, name, seed = job
    sim, x0, x1 = _get_sim(name)
    rng = np.random.default_rng(seed)
    sim.reset(x=rng.uniform(-0.15, 0.15), y=rng.uniform(-0.05, 0.05), yaw=rng.uniform(-0.05, 0.05))
    ctrl = R.controller_v0 if params is None else H.make_controller(params)
    r = R.run_trial(sim, ctrl, x1 + R.PASS_MARGIN, time_limit=TIME_LIMIT, vx=0.5)
    progress = float(np.clip((r["x_max"] - x0) / (x1 + R.PASS_MARGIN - x0), 0.0, 1.0))
    return name, r["status"], progress


def summarize(results):
    """[(name, status, progress)] → {name: (通过率, 平均前进比例, 摔倒数, 出界数)}"""
    out = {}
    for name in dict.fromkeys(n for n, _, _ in results):
        rs = [r for r in results if r[0] == name]
        out[name] = (np.mean([s == "pass" for _, s, _ in rs]), np.mean([p for _, _, p in rs]),
                     sum(s == "fell" for _, s, _ in rs), sum(s == "off_track" for _, s, _ in rs))
    return out


def fitness(summary):
    w = np.array([WEIGHTS.get(n, 1.0) for n in summary])
    score = float(np.average([rate + 0.5 * prog for rate, prog, _, _ in summary.values()], weights=w))
    flat_rate = summary.get("A warm-up", (1.0,))[0]
    return float(score - 2.0 * (1.0 - flat_rate))


def to_params(u):
    return {k: float(lo + np.clip(v, 0, 1) * (hi - lo)) for v, (k, (lo, hi)) in zip(u, H.PARAM_BOUNDS.items())}


def to_unit(params):
    return [(params[k] - lo) / (hi - lo) for k, (lo, hi) in H.PARAM_BOUNDS.items()]


def print_table(title, rows):
    print(f"\n{title}")
    print(f"{'segment':18s}" + "".join(f"{c:>22s}" for c in rows))
    names = list(next(iter(rows.values())))
    for n in names:
        cells = []
        for summ in rows.values():
            rate, prog, fell, off = summ[n]
            cells.append(f"{rate * 100:5.0f}% prog {prog:4.2f} f{fell} o{off}")
        print(f"{n:18s}" + "".join(f"{c:>22s}" for c in cells))


def mode_probe(pool, args):
    segs = TRAIN + ["F2 cross 10deg"]
    configs = {"V0": None, "hybrid default": dict(H.DEFAULT_PARAMS),
               "governor only": {**H.DEFAULT_PARAMS, "hip": 0.0, "knee": 0.0, "ankle": 0.0}}
    rows = {}
    for label, params in configs.items():
        t0 = time.time()
        jobs = [(params, n, 1000 + k) for n in segs for k in range(args.trials)]
        rows[label] = summarize(pool.map(run_one, jobs))
        print(f"{label}: {time.time() - t0:.0f} s")
    print_table(f"probe ({args.trials} trials each)", rows)


def mode_es(pool, args):
    import cma

    OUT.mkdir(parents=True, exist_ok=True)
    start = H.load_params(args.init) if (args.resume or args.init) else dict(H.DEFAULT_PARAMS)
    es = cma.CMAEvolutionStrategy(to_unit(start), args.sigma, {"bounds": [0, 1], "popsize": args.popsize, "seed": args.seed})
    (OUT / "config.json").write_text(json.dumps(dict(train=TRAIN, weights=WEIGHTS, init=args.init, start=start,
                                                     popsize=args.popsize, trials=args.trials, sigma=args.sigma), indent=2))
    log_path = OUT / "es_log.csv"
    new_log = not (args.resume and log_path.exists())
    best = (-np.inf, None, None)
    with open(log_path, "w" if new_log else "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_log:
            w.writerow(["gen", "best_gen", "mean_gen", "best_ever", "seconds", *H.PARAM_NAMES, *[f"{n} pass" for n in TRAIN]])
        for gen in range(args.generations):
            t0 = time.time()
            cands = es.ask()
            seeds = [10_000 * (gen + 1) + k for k in range(args.trials)]
            jobs = [(to_params(u), n, s) for u in cands for n in TRAIN for s in seeds]
            res = pool.map(run_one, jobs, chunksize=2)
            per = len(TRAIN) * len(seeds)
            summaries = [summarize(res[i * per:(i + 1) * per]) for i in range(len(cands))]
            fits = [fitness(s) for s in summaries]
            es.tell(cands, [-x for x in fits])
            i_best = int(np.argmax(fits))
            if fits[i_best] > best[0]:
                best = (fits[i_best], to_params(cands[i_best]), summaries[i_best])
                (OUT / "best_params.json").write_text(json.dumps({**best[1], "_fitness": best[0], "_generation": gen}, indent=2))
            p = to_params(cands[i_best])
            w.writerow([gen, round(fits[i_best], 4), round(float(np.mean(fits)), 4), round(best[0], 4), round(time.time() - t0, 1),
                        *[round(p[k], 4) for k in H.PARAM_NAMES], *[round(summaries[i_best][n][0], 2) for n in TRAIN]])
            f.flush()
            passes = "  ".join(f"{n.split(' ')[0]}:{summaries[i_best][n][0] * 100:.0f}%" for n in TRAIN)
            print(f"gen {gen:3d}  best {fits[i_best]:.3f}  mean {np.mean(fits):.3f}  best-ever {best[0]:.3f}  "
                  f"({time.time() - t0:.0f} s)  {passes}", flush=True)
    print(f"done: {OUT / 'best_params.json'}")


def mode_evaluate(pool, args):
    params = H.load_params(args.params or OUT / "best_params.json")
    test = [n for n in ALL_SEGMENTS if n not in TRAIN]
    rows_out = []
    for split, segs in (("train", TRAIN), ("test", test)):
        table = {}
        for label, prm in (("V0", None), ("V0.5 hybrid", params)):
            jobs = [(prm, n, 1000 + k) for n in segs for k in range(args.trials)]
            table[label] = summarize(pool.map(run_one, jobs))
            for n, (rate, prog, fell, off) in table[label].items():
                rows_out.append(dict(split=split, controller=label, segment=n, pass_rate=rate, progress=round(prog, 3),
                                     fell=fell, off_track=off, trials=args.trials))
        print_table(f"{split} set ({args.trials} trials each)", table)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "evaluation.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    plot_comparison(rows_out, OUT / "v0_vs_hybrid.png")
    print(f"\nresults: {OUT / 'evaluation.csv'}, {OUT / 'v0_vs_hybrid.png'}")


def plot_comparison(rows, path):
    """每个路段 V0 vs V0.5 通过率，左边训练集、右边测试集（没参与调参）。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n_train = sum(r["split"] == "train" and r["controller"] == "V0" for r in rows)
    n_test = sum(r["split"] == "test" and r["controller"] == "V0" for r in rows)
    fig, axes = plt.subplots(1, 2, figsize=(15, 4.2), gridspec_kw=dict(width_ratios=[n_train, n_test]), sharey=True)
    for ax, split in zip(axes, ("train", "test")):
        names = [r["segment"] for r in rows if r["split"] == split and r["controller"] == "V0"]
        x = np.arange(len(names))
        for k, (label, color) in enumerate((("V0", "#9aa5b1"), ("V0.5 hybrid", "#2f6ba8"))):
            vals = [100 * next(r["pass_rate"] for r in rows if r["split"] == split and r["controller"] == label and r["segment"] == n)
                    for n in names]
            bars = ax.bar(x + (k - 0.5) * 0.38, vals, 0.38, color=color, label=label)
            ax.bar_label(bars, fmt="%.0f", fontsize=7)
        ax.set_xticks(x, names, rotation=30, ha="right", fontsize=8)
        ax.set_title("train segments (used for tuning)" if split == "train" else "test segments (never seen in tuning)", fontsize=10)
        ax.set_ylim(0, 112)
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("pass rate [%]")
    axes[0].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--probe", action="store_true")
    p.add_argument("--evaluate", action="store_true")
    p.add_argument("--generations", type=int, default=60)
    p.add_argument("--popsize", type=int, default=12)
    p.add_argument("--sigma", type=float, default=0.25, help="CMA-ES 初始步长（参数已归一化到 0~1）")
    p.add_argument("--trials", type=int, default=3, help="每个候选每个路段的试验次数")
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--resume", action="store_true", help="从 --out 里的 best_params.json 继续")
    p.add_argument("--init", help="热启动用的参数文件（例如第 1 阶段的 best_params.json）")
    p.add_argument("--params", help="--evaluate 用的参数文件（默认 --out/best_params.json）")
    p.add_argument("--out", help="输出目录（默认 outputs/hybrid）")
    p.add_argument("--train", help="训练路段，逗号分隔（默认 TRAIN）")
    p.add_argument("--weights", help='路段权重，例如 "C2 step 10cm=2,D1 up 10/30=2"')
    args = p.parse_args()
    global OUT, TRAIN
    if args.out:
        OUT = (ROOT / args.out) if not pathlib.Path(args.out).is_absolute() else pathlib.Path(args.out)
    if args.train:
        TRAIN = [n.strip() for n in args.train.split(",")]
        for n in TRAIN:
            spec_by_name(n)
    if args.weights:
        for item in args.weights.split(","):
            name, w = item.rsplit("=", 1)
            WEIGHTS[name.strip()] = float(w)
    if args.resume and not args.init:
        args.init = str(OUT / "best_params.json")
    with mp.get_context("spawn").Pool(args.workers) as pool:
        if args.probe:
            mode_probe(pool, args)
        elif args.evaluate:
            mode_evaluate(pool, args)
        else:
            mode_es(pool, args)


if __name__ == "__main__":
    main()
