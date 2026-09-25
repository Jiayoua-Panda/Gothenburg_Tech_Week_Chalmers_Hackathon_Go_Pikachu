# Task 1 (required): Locomotion — vision-based stair climbing and step-length control

Evidence tags: **[measured]** = produced by code in this repository · **[reported]** = published results ·
**[proposed]** = our design, not yet built.

## 1. What we built and ran

| Component | File | What it does |
|---|---|---|
| Policy replay (sim2sim) | [`g1_step_playback/play_g1.py`](../g1_step_playback/play_g1.py) | Runs Unitree's pretrained G1-29dof velocity policy (trained in Isaac Lab) in MuJoCo, reproducing the exact observation/action pipeline of Unitree's C++ deployment code. |
| 2D test course | [`g1_course/course.py`](../g1_course/course.py) `FULL_COURSE` | Slopes 5–15°, single steps 5–15 cm, stairs up/down (10–15 cm rise). |
| 3D test course | `COURSE_3D` | Cross slopes 5°/10°, rough floor 2/4 cm, stairs approached at 15°/30°, 0.6 m narrow stairs. |
| Evaluation | [`g1_course/run_course.py`](../g1_course/run_course.py) | Per-segment pass rate over randomised starts (position ±15 cm, heading ±3°), falls, off-track events, lateral deviation, videos. |
| Perception view | `--mode scan` | 15 × 11 height scan (1.4 m × 1.0 m) in front of the robot, via ray casting — the input a V1 policy would get. |

Run it: `python run_course.py --mode segments --course 3d --trials 10` (CPU only; the full 2D + 3D matrix of 220 trials takes 81 s on an 8 GB laptop).

## 2. Results — V0 (Unitree pretrained, blind) **[measured]**

### Step length ("change of step length")

| Command | Actual speed | Mean step length | Cadence |
|---|---|---|---|
| 0.3 m/s | 0.30 m/s | 0.084 ± 0.010 m | 3.6 steps/s |
| 0.6 m/s | 0.56 m/s | 0.185 ± 0.029 m | 3.0 steps/s |
| 0.9 m/s | 0.75 m/s | 0.251 ± 0.023 m | 3.0 steps/s |

Source: [`g1_step_playback/outputs/summary.csv`](../g1_step_playback/outputs/summary.csv).
**Step length can be changed — but only through speed.** The policy has no step-length or cadence input,
so "short steps at normal speed" (e.g. for a narrow tread) is impossible without retraining. It also
saturates: 0.9 m/s commanded gives 0.75 m/s.

### Terrain (10 randomised trials per segment)

| Segment | Pass | Failure mode |
|---|---|---|
| A flat | 10/10 | — |
| B1 slope ±5° | 10/10 up, 10/10 down | — |
| B2 slope 10° | 0/10 up, 9/10 down | stalls / slides sideways uphill |
| B3 slope 15° | 0/10 up, 9/10 down | stalls, 7/10 leave the walkway |
| C1–C3 single step 5/10/15 cm | 0/10 each | falls at the step edge |
| D1–D2 stairs up | 0/10 each | falls within the first steps |
| E1–E2 stairs down | 0/10, 2/10 | steps down blindly, falls part-way |
| **F1/F2 cross slope 5°/10°** | 0/10 each | drifts downhill ~0.6 m, leaves the 1.2 m walkway |
| **G1/G2 rough floor 2/4 cm** | 10/10, 9/10 | one trip on 4 cm bumps |
| **H1/H2 stairs at 15°/30°** | 0/10 each | falls within the first steps |
| **I narrow stairs 0.6 m** | 0/10 | falls within the first steps |

Sources: [`outputs/v0/segment_results.csv`](../g1_course/outputs/v0/segment_results.csv),
[`outputs/v0/3d/segment_results.csv`](../g1_course/outputs/v0/3d/segment_results.csv); charts and videos in the same folders.

## 3. Why it does not work — root cause

1. **It is blind.** V0's observation is only body angular velocity, gravity direction, velocity command,
   joint positions/velocities and its last action (480 numbers, 5-frame history). There is no terrain input,
   so a 5 cm step and a flat floor look identical until the foot hits the edge.
2. **It was trained on flat ground** for velocity tracking. Robust to small bumps (G1/G2 pass) because
   training uses some randomisation, but never learned to lift the foot for a riser.
3. **No lateral terrain awareness.** On a cross slope it corrects its lean by stepping downhill, and nothing
   tells it that the walkway ends — a real industrial risk (walkways have edges, machines beside them).
4. **Step length is coupled to speed** — no command channel to adapt stride to tread depth.

These are exactly the gaps that the literature closes with perception + RL.

## 4. Why the proposed V1 should work — method **[proposed]**

**RL in simulation with a height-map observation, terrain curriculum, domain randomisation, teacher–student.**

| Ingredient | What it fixes | Precedent **[reported]** |
|---|---|---|
| Height map around the feet (from depth camera/lidar) | Sees risers, edges, slope direction | Duan et al., ICRA 2024 — Cassie on stairs and ~0.5 m steps, sim-to-real without real-world fine-tuning; Long et al. (PIM), ICRA 2025 — humanoid stairs using an elevation map, ~3 h training on one RTX 4090 |
| Terrain curriculum (flat → slopes → steps → stairs → angled/narrow) | Learns hard terrain progressively | Standard in legged_gym / Isaac Lab rough-terrain tasks |
| Explicit stair parameters as input (rise, tread, heading) | Faster learning, explainable input | "Explicit Stair Geometry Conditioning", 2026 preprint — Unitree G1 climbs 33 outdoor steps |
| Foothold planner prior | Chooses feasible footholds, faster stairs | FastStair, RA-L 2026 — LimX Oli up to 1.65 m/s on stairs |
| Step-length / gait-period command | Decouples stride from speed | Needed for the "change of step length" task — add a gait-period command and fine-tune |
| Domain randomisation + teacher–student | sim-to-real transfer | Duan et al.; Siekmann et al., RSS 2021 (blind Cassie stairs) as the no-vision baseline |

Concrete V1 recipe (Isaac Lab, `unitree_rl_lab` G1 velocity task as the starting point):
1. Add a height-scan observation (the same 15 × 11 grid we render in MuJoCo) to the actor for the teacher.
2. Rough-terrain curriculum with stair rise 5–18 cm, tread 25–40 cm, yaw ±30°, width 0.6–2 m, cross slope ±10°.
3. Add a gait-period command (e.g. 0.3–0.5 s) → step length becomes controllable independent of speed.
4. Distil to a student using noisy, delayed, partly occluded height maps (what the real lidar gives).
5. Export ONNX → re-run **this repository's** 2D + 3D matrix in MuJoCo (sim2sim) as the acceptance test.

Compute: a few GPU-hours per training run is the order reported (PIM: ~3 h on an RTX 4090). SKF offered
Google Cloud GPU access — that is the first next step. We had no GPU during the hackathon.

## 5. Assessment against SKF's checkpoints

| Checkpoint | Assessment |
|---|---|
| **Feasibility & implementation** | High. Pipeline pieces exist (Isaac Lab, Unitree RL lab, MuJoCo); our validation matrix and perception scan run today. Missing: GPU training run. |
| **Benefits** | Stairs/steps/slopes are the main reason wheeled AGVs cannot serve mezzanines, labs and older buildings. One trained policy covers all sites via scanning. |
| **Limitations** | Perception failures (reflective/transparent surfaces, dust), sim-to-real gap on foot contact, battery time, carrying loads changes dynamics (must be randomised in training). |
| **Data needs** | No human demonstrations. Needs: accurate robot model, site scan, randomisation ranges, and field failure logs for the loop. |
| **Safety** | A fall of a ~35 kg robot on stairs is the key hazard → stairs are restricted zones until the gate is passed; people-free stair use first; harness/gantry for first hardware trials; safety layer from [pipeline.md](pipeline.md). |
| **Industrial readiness** | Blind flat walking: deployable today (V0 = 100 % on flat). Perceptive stair climbing: demonstrated in research on real robots, **not yet** a certified industrial product; humanoid safety standard (ISO 25785-1) still in development. |
