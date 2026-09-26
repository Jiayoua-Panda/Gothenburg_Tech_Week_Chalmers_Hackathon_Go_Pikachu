"""整夜自动流程（跨平台，可脱离终端 / 编辑器运行）：

    1. 第 1 阶段结果重选：es_log.csv 里前几名用新种子重测，选最稳的（optimize.py --reselect）
    2. 第 2 阶段 CMA-ES：从第 1 阶段热启动，重点 10 cm 台阶 / 楼梯，跑到 --stage2-until
    3. 第 2 阶段重选 + 两阶段各自评估（10 次新试验，训练集 vs 未见过的测试集，V0 对比）

用法：
    py overnight.py --detach                     # 后台运行，关掉终端 / VS Code 也不停
    py overnight.py --stage2-until 08:30         # 前台运行
日志：outputs/overnight.log（最后一行 “ALL DONE”）
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import pathlib
import shutil
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "outputs"
H1 = OUT / "hybrid"
H2 = H1 / "stage2"
LOG = OUT / "overnight.log"
TRAIN2 = "A warm-up,C1 step 5cm,C2 step 10cm,D1 up 10/30,H1 stairs 15deg,F1 cross 5deg"
W2 = "C2 step 10cm=2,D1 up 10/30=2,H1 stairs 15deg=1.5,F1 cross 5deg=0.5"


def log(msg):
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(f"{dt.datetime.now():%a %b %d %H:%M:%S %Y} {msg}\n")


def run(args, out=None, wait=True):
    cmd = [sys.executable, str(ROOT / "optimize.py"), *args]
    f = open(out or LOG, "a", encoding="utf-8")
    p = subprocess.Popen(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT)
    if wait:
        p.wait()
        f.close()
    return p


def until(hhmm):
    """今天或明天的 hh:mm（如果已经过了就算明天）。"""
    h, m = map(int, hhmm.split(":"))
    now = dt.datetime.now()
    t = now.replace(hour=h, minute=m, second=0, microsecond=0)
    return t if t > now else t + dt.timedelta(days=1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stage2-until", default="08:30")
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--detach", action="store_true", help="脱离当前终端，在后台运行")
    args = p.parse_args()

    if args.detach:
        flags = {}
        if os.name == "nt":
            flags["creationflags"] = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            flags["start_new_session"] = True  # 新会话：编辑器 / 终端关闭时不会被一起杀掉
        cmd = [sys.executable, __file__, "--stage2-until", args.stage2_until, "--workers", str(args.workers)]
        if sys.platform == "darwin" and shutil.which("caffeinate"):
            cmd = ["caffeinate", "-i", *cmd]  # 防止 Mac 睡眠（需插电）
        subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL, **flags)
        print(f"started in background; log: {LOG}")
        return

    w = ["--workers", str(args.workers)]
    log(f"overnight.py start (stage 2 until {args.stage2_until}, pid {os.getpid()})")

    # 1. 第 1 阶段重选
    run(["--reselect", "--top", "8", "--trials", "8", *w])
    src = H1 / "reselected_params.json"
    shutil.copy(src if src.exists() else H1 / "best_params.json", H1 / "stage1_best_params.json")
    log("stage 1 choice: " + (H1 / "stage1_best_params.json").read_text().replace("\n", " "))

    # 2. 第 2 阶段，到时间就停
    H2.mkdir(parents=True, exist_ok=True)
    proc = run(["--out", str(H2), "--init", str(H1 / "stage1_best_params.json"), "--train", TRAIN2, "--weights", W2,
                "--generations", "1000", "--popsize", "14", "--trials", "4", "--sigma", "0.25", *w],
               out=H2 / "es_run.log", wait=False)
    log(f"stage 2 started (pid {proc.pid})")
    deadline = until(args.stage2_until)
    while proc.poll() is None and dt.datetime.now() < deadline:
        time.sleep(30)
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            proc.kill()
    log("stage 2 stopped")
    run(["--reselect", "--top", "8", "--trials", "8", "--out", str(H2), "--train", TRAIN2, *w])

    # 3. 评估（新种子）
    log("evaluating stage 1")
    run(["--evaluate", "--trials", "10", "--params", str(H1 / "stage1_best_params.json"), *w])
    s2 = H2 / "reselected_params.json"
    s2 = s2 if s2.exists() else H2 / "best_params.json"
    log(f"evaluating stage 2 ({s2.name})")
    run(["--evaluate", "--trials", "10", "--out", str(H2), "--train", TRAIN2, "--params", str(s2), *w])
    log("ALL DONE")


if __name__ == "__main__":
    main()
