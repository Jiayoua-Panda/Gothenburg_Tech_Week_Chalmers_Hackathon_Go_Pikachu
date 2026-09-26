# Humanoid Skill Factory — the learning pipeline

**Question from SKF:** *How can we make the training of humanoids in an industrial setting easier and more efficient?*

**Our answer:** treat every new skill as a product that moves through a fixed, repeatable pipeline, and
treat every failure (a fall, a near-miss, a dropped part) as training data rather than as an incident.
The pipeline is the same for locomotion and manipulation; what changes per task is the data source and
the training method (see [method_comparison.md](method_comparison.md)).

```
            ┌──────────────────────── failure loop ────────────────────────┐
            ▼                                                              │
 ┌─────────┐   ┌─────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐    │
 │ COLLECT │ → │  TRAIN  │ → │ SIMULATE │ → │ VALIDATE │ → │  DEPLOY  │ ───┘
 └─────────┘   └─────────┘   └──────────┘   └──────────┘   └──────────┘
  site scan     RL (loco)     digital twin   test matrix    safety layer
  demos         IL (manip)    domain rand.   pass gates     monitoring
  failure logs  fine-tune     sim2sim        KPIs           staged rollout
```

Legend for evidence in this folder: **[measured]** = produced by the code in this repository, with a file
reference; **[reported]** = published by others, cited; **[proposed]** = our design, not yet built.

---

## 1. Collect

| Input | Locomotion (stairs) | Manipulation (pick & place) |
|---|---|---|
| **Site geometry** | Scan of the real stairs, ramps, walkways (lidar or photogrammetry) → stair rise/tread/width, slopes, floor roughness | Scan of stations, tables, fixtures, tray positions |
| **Robot data** | Robot model (URDF/MJCF), joint limits, motor data from Unitree | Same, plus hand model (e.g. Unitree Dex3) |
| **Demonstrations** | Not needed — RL learns from reward | Teleoperated demos (VR/XR teleop of the G1), typically tens to hundreds per task **[reported]** |
| **Failure logs** | Falls, stumbles, off-track events from deployed robots | Failed grasps, drops, collisions |

**The digital-twin idea [proposed]:** a new site is scanned once (and re-scanned when the layout changes).
The scan becomes a simulation scene. Stair measurements feed straight into a parametric course generator —
exactly what [`g1_course/course.py`](../g1_course/course.py) does today with hand-entered numbers
(`rise`, `tread`, `n`, `yaw`, `width`). So "onboarding a new facility" becomes a scan, not a reprogramming job.

## 2. Train

- **Locomotion:** reinforcement learning in massively parallel simulation (Isaac Lab / legged_gym style),
  with a terrain curriculum (flat → slopes → steps → stairs → angled/narrow stairs), domain randomisation
  (friction, mass, motor strength, latency, sensor noise) and a **height-map observation** so the policy
  can *see* the terrain. Teacher–student distillation turns a privileged "teacher" (sees exact terrain)
  into a "student" that works from real sensor data. See [task_locomotion.md](task_locomotion.md).
- **Manipulation:** imitation learning from teleoperated demonstrations (ACT, Diffusion Policy), optionally
  initialised from a vision-language-action (VLA) foundation model and refined with RL in simulation.
  See [task_manipulation.md](task_manipulation.md).
- **Instruction layer:** a language/VLA model maps "go up the stairs to the wash room and place the rings
  in fixture 3" to a sequence of skills. It does **not** drive the joints directly.

## 3. Simulate

- Train and test in the **digital twin of the actual site**, randomised around it (± a few cm per stair,
  friction ranges, lighting), so the policy is not overfitted to one exact geometry.
- **sim2sim check:** a policy trained in one simulator (Isaac Lab) is replayed in another (MuJoCo) before any
  hardware. We already do this: Unitree's G1 policy was trained in Isaac Lab and we run it in MuJoCo
  **[measured]** ([`g1_step_playback/`](../g1_step_playback/)).

## 4. Validate — the gate before hardware

A skill only passes to deployment when it clears a **test matrix** with pass-rate gates. We built the
matrix tooling and ran V0 (Unitree's pretrained flat-ground policy) and our **scan-selected skill switch**
(V0 on the flat, the stair-trained G1-DWAQ policy on steps and stairs — see
[task_locomotion.md §4b](task_locomotion.md)) through it:

| Dimension | Segments | V0 **[measured]** | Switch **[measured]** |
|---|---|---|---|
| Flat, gentle slopes | A, B1 (±5°) | 100 % | 100 % |
| Steep slopes | B2/B3 (10°, 15°) up / down | 0 % up, 90 % down | 100 % / 100 % |
| Single steps | C1–C3 (5/10/15 cm) | 0 % | 100 % |
| Straight stairs | D1–D2 up, E1–E2 down | 0 % up, 10 % down | 100 % / 100 % |
| Cross slope (3D) | F1/F2 (5°, 10°) | 0 % — drifts off a 1.2 m walkway | 90 % |
| Rough floor (3D) | G1/G2 (2 cm, 4 cm) | 95 % | 100 % |
| Angled / narrow stairs (3D) | H1/H2 (15°, 30°), I (0.6 m) | 0 % | 97 % |
| **All 220 trials** | | **36 %** | **99 %** |
| Person in the walkway | safety scenarios | see [safety results](#safety-layer) | |

Sources: `g1_course/outputs/{v0,switch}/segment_results.csv` and `3d/` (10 randomised trials per segment);
one-page summary in [`g1_course/outputs/MORNING_SUMMARY.md`](../g1_course/outputs/MORNING_SUMMARY.md).

Example gate for stairs **[proposed]**: ≥ 99 % success over 1 000 simulated trials on the twin ± randomisation,
≥ 95 % on *unseen* stair geometries, zero off-track events, then a supervised hardware trial on a gantry/harness.

## 5. Deploy

Four layers run on the robot and in the building **[proposed]**:

1. **Instruction layer** — language/VLA model selects the task ("go to the wash room", "pick ring").
2. **Skill policies + skill selection** — several learned policies (50 Hz), each for the terrain it was trained
   on; perception picks which one runs. **Demonstrated [measured]:** our height scan hands over from V0 (flat) to
   a stair-trained policy before the first riser and back on the landing — 99 % of 220 trials, including stairs
   down, angled and narrow stairs; V0 alone: 36 %.
3. **Onboard perception** — the G1 carries a depth camera and a 3D lidar in its head (per Unitree's
   published spec — verify for the exact variant). These build a **local height map** around the feet in
   real time: the input a V1 stair policy uses. Our simulation renders exactly this scan
   ([`perception_view.png`](../g1_course/outputs/v0/3d/perception_view.png)).
4. **Safety layer (rule-based, not learned)** — see below. It is deliberately *not* AI: it must be
   explainable and certifiable, and it overrides the skill policies.

### Safety layer

- **Fixed sensors in the building** (ceiling lidar/cameras) track people — they see around corners the robot
  cannot. The robot slows when a person is within the slow distance and stops at the protective distance
  (*speed & separation monitoring*, ISO/TS 15066).
- **Contact detection** without extra hardware: compare expected vs measured joint torque; an unexpected
  residual means contact. Tactile skin only where needed.
- **Robot-to-robot**: coordinated by a fleet manager that knows every robot's position and plan — not by sensors.
- **A humanoid's emergency stop is a fall.** Cutting motor power drops the robot. "Safe stop" must therefore
  be an active behaviour (stand still, then crouch/sit), which is itself a learned-then-validated skill.

What our simulation showed when we tested this idea **[measured]** — a person steps into a 1.2 m walkway in
front of the walking robot; 50 randomised scenarios (position, timing, walking speed 0.5–1.5 m/s, dwell time),
each run with four safety configurations. The safety design changed **three times because of testing**:

| Configuration | Contacts while robot driving | Contacts during protective stop | Robot's own approach in last 1 s before contact | Median closest distance |
|---|---|---|---|---|
| No monitor | 44 / 50 | 0 | — | 0.17 m |
| Stop at 0.8 m (velocity command = 0) | 0 | 24 / 50 | up to 0.30 m | 0.50 m |
| + position hold (classical P-loop) | 0 | 20 / 50 | up to 0.30 m | 0.65 m |
| + ISO/TS 15066 distance, 1.67 m | 0 | **11 / 50** | **≤ 0.12 m** | **1.14 m** |

1. *Naive stop.* V0 does **not** stand still on a zero-velocity command: it kept creeping toward the person,
   0.27 m on average while "stopped". **A learned policy's "stop" is not a guaranteed stop.**
2. *Position hold* — a classical P-loop wrapped around the learned policy (gain tuned from 1 to 3 after
   measuring: overshoot 0.10 → 0.046 m, settled in 0.6 s instead of 1.6 s). Creep fell to 0.05 m. **Hybrid wins.**
   But contacts remained: a fixed 0.8 m ignores the person's speed, sensor latency and the robot's braking.
3. *ISO/TS 15066 protective separation distance* from our **measured** stopping behaviour:
   S_p = v_h·(T_r + T_s) + v_r·T_r + S_s + C = 1.6·(0.1 + 0.6) + 0.5·0.1 + 0.05 + 0.45 ≈ **1.67 m**.
   Contacts fell from 44 to 11 of 50 — and in **all 11 the robot was already in protective stop** (it moved
   ≤ 0.12 m in the final second); the scripted person, who never steps aside, walked into it.
   Separation monitoring alone cannot prevent that. ISO/TS 15066 therefore pairs it with power & force
   limiting — and it is why a validated "stand still / crouch" skill belongs in the pipeline.

Contact = horizontal centre distance < 0.45 m (person radius 0.22 m + robot torso ≈ 0.2 m). Sensor latency 0.1 s.
Source: [`safety_summary.csv`](../g1_course/outputs/v0/safety/safety_summary.csv),
[`safety_results.csv`](../g1_course/outputs/v0/safety/safety_results.csv), demo videos `safety_demo_{off,stop,hold,iso}.mp4`.

Relevant standards (check current editions before relying on them): ISO 10218-1/-2 (industrial robots),
ISO/TS 15066 (collaborative operation, speed & separation monitoring), ISO 13855 (human approach speed),
ISO 3691-4 (driverless industrial trucks), and **ISO 25785-1**, a standard for industrial mobile robots with
actively controlled stability (legged/humanoid) that is **in development**.

## 6. The failure loop — why this makes training *easier and more efficient*

```
robot falls / near-miss on site  →  log state, sensor data, map position
        →  recreate the situation in the site twin (automatic: we have the scan)
        →  add it to the training curriculum + the validation matrix
        →  retrain / fine-tune  →  must pass the full matrix again  →  staged rollout
```

- **Easier:** no hand-programming per site; a scan plus the generic pipeline replaces it.
- **More efficient:** failures found in the field are reproduced in minutes in simulation, not re-staged
  physically; the validation matrix is re-run automatically (our full 2D + 3D matrix, 220 trials, runs in
  81 s on an 8 GB laptop CPU **[measured]**).
- **Safer:** nothing reaches hardware without passing the same gates; the safety layer is independent of
  the learned skills.

## Where we are vs. the pipeline

| Stage | Status in this repository |
|---|---|
| Collect | Unitree robot model + pretrained policy; parametric course generator (stand-in for a site scan) |
| Train | CPU-only: overnight CMA-ES tuning of a classical layer (V0.5) — improved trained terrain, broke held-out terrain (documented). RL training of our own stair skill needs a GPU → V1 recipe in [task_locomotion.md](task_locomotion.md) |
| Simulate | MuJoCo sim2sim of two Isaac Lab policies (Unitree V0, third-party G1-DWAQ) on our robot model — working |
| Validate | 2D + 3D test matrix (220 trials), perception scan, safety scenarios — working, results above |
| Deploy | Scan-based skill selection V0 ⇄ stair skill (99 % of the matrix); safety layer (speed & separation + position hold) |
