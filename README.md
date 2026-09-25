# SKF Humanoid Hackathon

GTW Chalmers Hackathon 2026 — SKF challenge: *Analyze and compare learning methods suitable for industrial humanoids*.

We take **Path 2 — analyse the pipeline** (Collect → Train → Simulate → Validate → Deploy), with locomotion
(required) backed by working simulation evidence and manipulation (desired) as a feasibility analysis.

## Analysis (start here)

- [analysis/pipeline.md](analysis/pipeline.md) — the Skill Factory pipeline, digital twin, sensors, safety layer, failure loop
- [analysis/task_locomotion.md](analysis/task_locomotion.md) — stairs and step length: V0 results, root cause, V1 recipe
- [analysis/task_manipulation.md](analysis/task_manipulation.md) — pick & place and wiping: method and assessment
- [analysis/method_comparison.md](analysis/method_comparison.md) — 7 learning methods × 8 industrial criteria
- [analysis/skf_use_case.md](analysis/skf_use_case.md) — the "cleanliness runner" use case with KPIs

## Simulation (Unitree G1 in MuJoCo, CPU only)

```sh
python3 -m venv .venv && .venv/bin/pip install mujoco onnxruntime pyyaml opencv-python "imageio[ffmpeg]" matplotlib
cd g1_course
../.venv/bin/python run_course.py --mode segments --trials 10              # 2D test matrix (slopes, steps, stairs)
../.venv/bin/python run_course.py --mode segments --course 3d --trials 10  # 3D: cross slope, rough floor, angled/narrow stairs
../.venv/bin/python run_course.py --mode tour --course 3d --show-scan      # video with the robot's height scan
../.venv/bin/python run_course.py --mode scan --course 3d                  # perception-view figure
../.venv/bin/python run_course.py --mode safety --trials 50                # person in the walkway: off / stop / hold / iso

# V0.5 hybrid: height scan + V0 policy + a classical layer tuned by CMA-ES on CPU (no GPU)
../.venv/bin/pip install cma
../.venv/bin/python optimize.py --probe                                    # quick check: V0 vs untuned hybrid
../.venv/bin/python optimize.py --generations 100 --popsize 14 --trials 4  # overnight tuning → outputs/hybrid/best_params.json
../.venv/bin/python optimize.py --evaluate --trials 10                     # train vs held-out segments, V0 vs V0.5
../.venv/bin/python run_course.py --mode segments --course 3d --controller hybrid
```

- [`g1_step_playback/`](g1_step_playback/) — Unitree's pretrained G1 walking policy replayed in MuJoCo; step length vs speed.
- [`g1_course/`](g1_course/) — parametric test course (2D and 3D), evaluation, perception scan, safety demo;
  `perception.py` (height scan), `hybrid.py` (V0.5 controller), `optimize.py` (CMA-ES tuning, train/test split).
  Results in [`g1_course/outputs/v0/`](g1_course/outputs/v0/) (`3d/`, `safety/`).

Evaluation results in `g1_course/outputs/v0/segment_results.csv` were regenerated on macOS (MuJoCo 3.x);
the videos in the same folder are from the earlier Windows run. Pass rates agree within trial noise.

## StairLab interactive concept

Open [index.html](index.html) in a browser to explore the saved StairLab interface. No build step is required. GitHub displays the HTML source; download or clone the repository to open the page locally.

- Adjust stair rise, tread depth, and usable width.
- Switch between spatial and perception views.
- Show or hide illustrative footholds.
- Use the responsive layout on desktop or mobile.

The saved starting view is perception mode, with a **20 cm rise, 40 cm tread, and 110 cm width**. Subsequent changes are remembered by the browser when local storage is available.

This is an interactive geometry and interface concept, not a physics simulation or a trained robot policy. Robot poses, sensor coverage, and footholds are illustrative; experimental metrics remain unmeasured.

Files:

- `index.html`: standalone browser export, including its display and state-storage runtime.
- `web/stairlab.fragment.html`: editable interface source, preserved from the conversation visualization with the selected starting values.

The standalone page is exported with the visualize skill's `scripts/render.py`:

```sh
python3 /path/to/visualize/scripts/render.py web/stairlab.fragment.html index.html --title "StairLab — Industrial Humanoid Learning" --force
```

## MuJoCo evaluations

- [Unitree V0 walking playback](g1_step_playback/README.md) and [test-course results](g1_course/outputs/v0/segment_results.csv): the official pretrained walking policy, evaluated without new training.
- [G1-DWAQ stair replays](artifacts/stair-replays/README.md): a separate third-party stair policy on custom corridor–stair–corridor scenes. Four videos include one successful route and three failures; the successful route used simulator ground-truth position and heading for steering, without camera perception.

These use different policies and test scenes, so their results are not a controlled policy comparison.

## Docs

- [SKF challenge brief](SKF_Analyze%20and%20compare%20learning%20methods%20suitable%20for%20industrial%20humanoids.pdf)
- [Participant schedule & practical info](Participant_Schedule_Practical_Information_Hackathon_2026%20%282%29.pdf)
- [Challenge to Pitch crash course](GTW%20Chalmers%20Hackathon%20-%20Challenge%20to%20Pitch%20Crash%20Course.pdf)
- [Research papers and reported results](papers/README.md)
