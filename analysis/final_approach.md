# Final approach — the Humanoid Skill Factory

**SKF's question:** *How can we make the training of humanoids in an industrial setting easier and more efficient?*

**Our answer:** don't train one giant policy per task and site. **Compose** pretrained skills, let perception and
a known map decide which skill runs where, wrap everything in a rule-based safety layer, pass every skill through
the **same test gates**, and spend training effort **only on the failures the gates find**.

With this approach the team reached **99 % on a 220-run terrain matrix** and **88 % on a 19.9 m factory route with
stairs up and down — without training a new model.** The gates also told us exactly what to train next
(long descents, carried loads). We ran those fine-tunes, and the same gates caught a regression the training reward did not show.

All results are MuJoCo simulation of a Unitree G1 (29 DoF), reproducible from this repository.
Tags: **[measured]** = produced here · **[reported]** = published by others · **[proposed]** = design, not built.

---

## 1. The six parts

| # | Part | What it does | Who / where |
|---|---|---|---|
| 1 | **Skill library** | Pretrained skills per terrain: Unitree V0 for level ground (fast), the stair-trained G1-DWAQ policy for stairs and slopes (third-party, BSD-3, not trained by us). Later: our own fine-tuned skills. | `g1_step_playback/`, `g1_course/dwaq_policy.py` |
| 2 | **Know where you are** | Factory map / CAD + fixed cameras + on-board odometry → a route follower plans corners, landings and flights. | Zhou — `artifacts/factory-course/` |
| 3 | **See the terrain, pick the skill** | An on-board height scan in front of the feet hands over V0 ⇄ stair skill before a riser, step or cross slope, and back on level ground. | `g1_course/run_course.py` (`SwitchSim`, `controller_switch`) |
| 4 | **Rule-based safety layer** | Speed & separation monitoring with the ISO/TS 15066 protective distance computed from measured braking; classical position hold, because a learned "stop" is not a stop. | `run_course.py --mode safety` |
| 5 | **Test gates** | The same matrices accept every skill: 220-run terrain matrix (2D + 3D), factory routes, camera-error and payload sweeps. | `g1_course/outputs/`, `artifacts/factory-course/` |
| 6 | **Failure loop → targeted training** | Failures found by the gates define the next training run — nothing else. | fine-tuning on Chalmers GPU (Zhou), `training/g1-dwaq-finetune/` |

This is the team's first idea from day 1, now tested: map the facility with cameras, sense people and
terrain on the robot, and keep a safety layer that is independent of the learned parts.

## 2. Evidence **[measured]**

| Claim | Result | Source |
|---|---|---|
| A flat-trained policy fails stairs | Unitree V0: **36 %** of 220 runs; 0 % stairs up, 0 % single steps. Diagnosis: the toe stubs the riser; speed does not help. | `g1_course/outputs/v0/` |
| Perception picks the right skill | Scan-selected V0 ⇄ stair skill: **99 %** (217/220). Stairs up 100 %, stairs down 100 % (4-step flights), angled / narrow 97 %, cross slopes 90 %. Level ground at V0's speed: 12 s per segment vs 27 s with the stair skill alone. One continuous 3D run: all 8 segments passed (`outputs/switch/3d/course_tour.mp4`). | `g1_course/outputs/switch/`, `outputs/MORNING_SUMMARY.md` |
| Known map + cameras reach a real route | 19.9 m factory route (5 turns, switchback up to a 1.8 m mezzanine, 12 risers down): **88 %** (22/25 starts), all failures on the long descent. | `artifacts/factory-course/` |
| An ordinary camera system is enough | Comparable to a perfect camera within position σ ≤ 10 cm, bias ≤ 20 cm, ≥ 2 Hz, ≤ 300 ms latency; with odometry fusion ≥ 1 Hz and ≤ 1 s. Heading noise up to 15° barely matters. | `artifacts/factory-course/localization-sweep/` |
| Safety must not rely on the learned policy | Person in the walkway, 50 scenarios: contacts while driving **44 → 0**. V0 "stopped" still crept 0.27 m; a position hold cut it to 0.05 m; ISO distance 1.67 m from measured braking. | `g1_course/outputs/v0/safety/` |
| Carrying without retraining has limits | Chest carrier, arms free: ~8 kg up one flight (8/9). Hand-held (locked arms): ~2 kg, only with the "virtual free arms" trick. Backpack: fails from 2 kg. Full route with a load: 15/25 at 2 kg, 6/25 at 8 kg (chest). | `artifacts/factory-course/payload-sweep/` |
| Fine-tuning must pass the same gates | 6 Isaac Lab fine-tunes (reward unchanged; payload/CoM randomisation, more descent terrain), re-tested on the same 25 route starts: 8 kg chest 6 → 14–20/25, no load 22 → 0–3/25. Reward improved; the gate rejected the policy. | `artifacts/factory-course/finetune-results.csv`, `training/g1-dwaq-finetune/` |
| Step length only follows speed | 0.084 / 0.185 / 0.251 m at 0.3 / 0.6 / 0.9 m/s; no step-length command in V0. | `g1_step_playback/outputs/` |

## 3. What did not work — and where the training goes

| Finding | Why | Consequence |
|---|---|---|
| **V0.5**: a classical layer (leg lift + steering) on V0, tuned overnight by CMA-ES on a laptop | +60 pp on 5 cm steps and +90 pp on a 5° cross slope where it was tuned, but **broke terrain it had never seen** (5° ramp 100 → 0 %, 4 cm rough floor 90 → 10 %). | Patching a skill overfits; train the skill on the terrain. Always keep a held-out test set. |
| **Long descents** | The stair skill twists on steps 3–5 of 10–12-riser flights, even with perfect position. Short (4-step) flights pass 100 %. | Trained with 2× descent terrain (`ft_payload_descent`, `ft_descent`): did **not** fix it; the descent test stayed at 7/9 and the no-load route dropped to 0/25. Needs a perception-aware stair skill or another approach. |
| **Loads** | The stair skill was trained with ±5 kg torso mass and free arms; locked arms or a rear centre of mass break it. | Use a front carrier now. Payload + centre-of-mass fine-tune (`ft_payload`): full route at 8 kg 6/25 → 14–20/25, but with no load 22/25 → 0–3/25. Not deployable; the next run must hold every load level. |
| **Right turns in place ~4× weaker; small commands ignored** | Properties of the stair skill found while building routes. | The route follower compensates (full-rate turns, minimum speeds) — a supervisor, not retraining. |
| **Learned stop is not a stop** | V0 creeps at zero velocity command. | Safety stop stays classical and certifiable. |

## 4. Industrial KPIs and specifications from the tests

| KPI / spec | Value from our tests | Use |
|---|---|---|
| Camera system for localisation | σ ≤ 10 cm, bias ≤ 20 cm, ≥ 2 Hz, ≤ 300 ms (≥ 1 Hz / ≤ 1 s with odometry) — largest tested values not clearly worse; keep margin | Purchase spec |
| Protective separation distance | 1.67 m at 0.5 m/s walking (ISO/TS 15066 formula, measured stop 0.6 s / 0.05 m) | Safety design, zone layout |
| Payload without retraining | ~8 kg on a chest carrier (one flight), ~2 kg hand-held | Tray / carrier design |
| Terrain gate | ≥ 99 % over the matrix before hardware (switch today: 99 % of 220) | Acceptance test for each new skill |
| Test cost | 220 V0 runs in 81 s on a laptop CPU; skill-switch matrix ≈ 10 min | Fast iteration |

Onboarding a new site then means: floor plan / scan → install cameras to the spec → run the gates on the
site's routes → train only what fails. No per-site programming of skills.

## 5. Next steps (call to action for SKF)

1. **Next targeted fine-tune** that keeps no-load, light-load, heavy-load and descent results all at or above the original (the first six runs traded one for another);
   then train our own stair skill with the height map on SKF's offered cloud GPU.
2. **One real route**: scan SKF's machining-line → cleanliness-lab route, place cameras to the spec, run the gates.
3. **Pilot / thesis**: one station, one route, the KPIs above.

## 6. Honest limits
- All results are simulation (MuJoCo), rigid geometry, no sensor models beyond the noise we injected.
- The stair skill is third-party (G1DWAQ_Lab, BSD-3); our work is the composition, perception, safety, gates and analysis.
- Centring and routing use simulator pose or simulated camera error, not a real camera pipeline.
- Success rates come from 9–25 fixed starts or 10 random starts per case — estimates, not guarantees.

Detailed documents: [pipeline.md](pipeline.md) · [task_locomotion.md](task_locomotion.md) ·
[task_manipulation.md](task_manipulation.md) · [method_comparison.md](method_comparison.md) ·
[skf_use_case.md](skf_use_case.md) · team summary [START_HERE.md](../START_HERE.md) ·
factory course [artifacts/factory-course/](../artifacts/factory-course/README.md).
