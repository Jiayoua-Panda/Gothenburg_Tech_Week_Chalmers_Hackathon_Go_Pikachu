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
| Perception view | [`g1_course/perception.py`](../g1_course/perception.py), `--mode scan` | 15 × 11 height scan (1.4 m × 1.0 m) in front of the robot, via ray casting — the input a V1 policy would get. |
| V0.5 hybrid | [`hybrid.py`](../g1_course/hybrid.py), [`optimize.py`](../g1_course/optimize.py) | Height scan + V0 + a classical layer (speed/steering governor, swing-leg residual) tuned overnight by CMA-ES on a laptop CPU, with a train / held-out split. |
| Stair skill | [`dwaq_policy.py`](../g1_course/dwaq_policy.py) | Our numpy re-implementation of the third-party **G1-DWAQ** stair policy's inference (matches the reference to 1e-6). Weights: G1DWAQ_Lab, BSD-3 — **not trained by us** (`fetch_dwaq.sh`). |
| Skill switching | `SwitchSim`, `controller_switch` in [`run_course.py`](../g1_course/run_course.py) | **Our height scan picks the skill**: V0 on level ground, G1-DWAQ when a step, stair or cross slope is ahead; warm-up + 0.8 s torque blend at each handover. |

Run it: `python run_course.py --mode segments --course 3d --trials 10 --controller switch` (CPU only; the full V0 2D + 3D
matrix of 220 trials takes 81 s on an 8 GB laptop; DWAQ/switch runs take ~10 min because they walk at 0.3 m/s on stairs).

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

**Diagnosis [measured]:** on every step and stair the failure is a **toe stub** — the swing foot does not clear the
riser, the feet stay at the edge while the pelvis keeps moving, and the robot pitches over. Walking faster or
slower does not change it (0/4 at 0.3, 0.7 and 1.0 m/s on 5 and 10 cm steps).

1. **It was trained on flat ground** for velocity tracking. Robust to small bumps (G1/G2 pass) because training
   uses some randomisation, but it never learned to lift the foot for a riser. This is the main cause: a policy
   *trained on stairs* (G1-DWAQ, section 4b) climbs the same stairs although it is also blind.
2. **It is blind.** V0's observation is only body angular velocity, gravity direction, velocity command,
   joint positions/velocities and its last action (480 numbers, 5-frame history). A 5 cm step and a flat floor
   look identical until the foot hits the edge — so it cannot slow down, choose footholds or pick a different
   skill in time.
3. **No lateral terrain awareness.** On a cross slope it corrects its lean by stepping downhill, and nothing
   tells it that the walkway ends — a real industrial risk (walkways have edges, machines beside them).
4. **Step length is coupled to speed** — no command channel to adapt stride to tread depth.

## 4. What we tried overnight **[measured]**

### 4a. V0.5 — a tuned classical layer on the blind policy (does not generalise)

Perception-triggered swing-leg lift + speed/steering governor, 11–16 parameters tuned by CMA-ES on the laptop
(≈ 150 generations, ~8 h CPU). Candidates were re-tested on fresh seeds before selection, because the logged best
score from 4 trials was often luck (0.705 logged → 0.30 re-tested). Final check: 10 new trials per segment.

| Segment | V0 | V0.5 | Used for tuning? |
|---|---|---|---|
| C1 step 5 cm | 0 % | **60 %** | train |
| C2 step 10 cm | 0 % | 10 % | train |
| F1 cross slope 5° | 10 % | **100 %** | train |
| D1 stairs, H1 angled stairs | 0 % | 0 % | train |
| **B1 ramp 5° up** | 100 % | **0 %** | held-out |
| **G2 rough floor 4 cm** | 90 % | **10 %** | held-out |
| D2, C3, E1/E2, H2, I (stairs, 15 cm step) | 0–10 % | 0–10 % | held-out |

It helped where it was tuned and **broke terrain it had never seen** (the scan reads a ramp or bumps as a step
and triggers the leg lift). A hand-designed patch on a blind flat-ground policy overfits; this is why the
held-out split matters. Sources: [`outputs/hybrid/evaluation.csv`](../g1_course/outputs/hybrid/evaluation.csv),
[`es_log.csv`](../g1_course/outputs/hybrid/es_log.csv), stage 2 in `outputs/hybrid/stage2/` (not better).

### 4b. Right skill for the terrain — our scan switches V0 ⇄ a stair-trained policy (works)

G1-DWAQ (third-party, trained with a stairs curriculum; blind, proprioceptive history + VAE) runs unchanged on
our robot model. Our height scan decides when to hand over. 10 randomised trials per segment, same seeds as V0:

| Terrain group | V0 (flat-trained) | V0.5 | G1-DWAQ only | **Switch (scan picks V0 / DWAQ)** |
|---|---|---|---|---|
| Flat and 5° slopes | 100 % | 50 % | 100 % | **100 %** |
| Rough floor 2–4 cm | 95 % | 50 % | 100 % | **100 %** |
| Downhill 10–15° | 90 % | – | 100 % | **100 %** |
| Uphill 10–15° | 0 % | 0 % | 100 % | **100 %** |
| Cross slope 5–10° | 0 % | 50 % | 100 % | **90 %** |
| Single steps 5–15 cm | 0 % | 23 % | 100 % | **100 %** |
| Stairs up 10–15 cm | 0 % | 0 % | 95 % | **100 %** |
| Stairs down 10–15 cm | 10 % | 10 % | 95 % | **100 %** |
| Angled / narrow stairs | 0 % | 0 % | 93 % | **97 %** |
| **All 220 trials** | **36 %** | – | **98 %** | **99 %** |

Why switch rather than DWAQ everywhere: DWAQ is only validated at 0.3 m/s; V0 walks level ground at 0.5 m/s.
Mean time to pass the flat segment: V0 12.3 s, **switch 12.3 s**, DWAQ only 26.8 s — the switch keeps V0's speed
on the flat and DWAQ's capability on stairs. One continuous run over the whole 2D course: 12/14 segments, two
short stalls (15° ramp, 15 cm stairs) caught by the 8 s stall rule → `outputs/switch/course_tour.mp4`.

Test changed the design again: the first selector only looked for height steps along the walking line, so V0
walked cross slopes and drifted off (75 %). Adding the scan's lateral slope (plane fit, > ≈ 3°) to the selector
raised it to 90 % (first version kept in `outputs/switch/v1_selector/`).

**Caveats (say them in the pitch):** the DWAQ weights are third-party, not trained by us; walkway centring uses
simulator position (on a robot: localisation); stairs are MuJoCo boxes with exact geometry and 1 cm scan noise.
Sources: `outputs/{v0,dwaq,switch}/segment_results.csv` and `3d/`, summary in `outputs/MORNING_SUMMARY.md`.

## 5. Next: our own V1 — method **[proposed]**

Section 4b shows the target is reachable: a policy trained with a stairs curriculum climbs everything in our
matrix. V1 means training that skill ourselves (licensable, tunable, faster than 0.3 m/s) and adding the
height map so it can plan footholds and speed:

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

## 6. Assessment against SKF's checkpoints

| Checkpoint | Assessment |
|---|---|
| **Feasibility & implementation** | High. Demonstrated in our simulation: scan-selected stair skill passes 99 % of 220 trials incl. stairs up/down, angled and narrow stairs. Missing: training our own stair skill (GPU) and hardware tests. |
| **Benefits** | Stairs/steps/slopes are the main reason wheeled AGVs cannot serve mezzanines, labs and older buildings. One trained policy covers all sites via scanning. |
| **Limitations** | Perception failures (reflective/transparent surfaces, dust), sim-to-real gap on foot contact, battery time, carrying loads changes dynamics (must be randomised in training). |
| **Data needs** | No human demonstrations. Needs: accurate robot model, site scan, randomisation ranges, and field failure logs for the loop. |
| **Safety** | A fall of a ~35 kg robot on stairs is the key hazard → stairs are restricted zones until the gate is passed; people-free stair use first; harness/gantry for first hardware trials; safety layer from [pipeline.md](pipeline.md). |
| **Industrial readiness** | Flat walking: deployable today (V0 = 100 % on flat). Stair skills: shown in simulation (ours) and on real robots in research, **not yet** a certified industrial product; humanoid safety standard (ISO 25785-1) still in development. |
