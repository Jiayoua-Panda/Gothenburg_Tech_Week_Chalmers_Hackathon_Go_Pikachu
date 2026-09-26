# SKF use case: the "cleanliness runner"

A proposed learning pipeline for one SKF use case, including data collection, training, testing and
success measures. The scenario is **hypothetical** — built from the challenge brief (machining &
cleanliness), not from SKF process data. All numbers marked *assumption* should be replaced by SKF's own.

## 1. The job

Bearing rings are sampled from a machining line and taken to a **cleanliness inspection lab** (washing,
filtration, particle analysis). Today a person carries sample trays between the line and the lab — often
across a shared walkway and, in many older plants, up a flight of stairs to a mezzanine lab.

**Robot task:** walk from the line to the lab (walkway with people, a ramp or stairs), pick the rings from
the tray, place them in the washing/inspection fixture, wipe the inspection bench, return.

This single job needs **both** tasks from the brief: locomotion (stairs, shared walkway) and manipulation
(pick & place, wiping) — and it is a place where a wheeled AGV or a fixed robot cell does not fit well.

## 2. Pipeline for this use case

| Stage | What happens | Owner | Time *(assumption)* |
|---|---|---|---|
| **Collect** | Scan the route and lab (handheld lidar/photogrammetry, 1 day); measure stairs, ramps, walkway width; record 50–200 teleop demos per manipulation step (ring pick, fixture place, bench wipe); log ring sizes and fixture CAD | SKF process specialist + integrator | 1–2 weeks |
| **Train** | Locomotion: RL with height map on the site twin + randomisation (GPU, cloud). Manipulation: ACT/Diffusion Policy on the demos; VLA layer for task sequencing | Integrator / research partner (thesis) | 1–3 weeks incl. iterations |
| **Simulate** | Full route in the twin: 1 000+ episodes with randomised people, stair geometry ± 2 cm, lighting, ring sizes; sim2sim check in a second simulator | Automated | hours per run |
| **Validate** | Test matrix gates (below) in sim → supervised hardware tests: gantry on stairs, lab station with dummy rings | SKF safety + integrator | 2–4 weeks |
| **Deploy** | Staged: (1) off-shift, no people; (2) with people, reduced speed; (3) normal operation. Safety layer: building sensors for speed & separation, position hold, restricted stair use | SKF production + safety | 1–3 months |
| **Failure loop** | Every fall/near-miss/drop is logged, reproduced in the twin, added to training and to the test matrix | Automated + weekly review | continuous |

## 3. Success measures (KPIs)

### Technical gates (must pass before each rollout stage)

| KPI | Target *(assumption)* | Where we are **[measured]** |
|---|---|---|
| Stair success rate in sim (site twin ± randomisation) | ≥ 99 % over 1 000 trials | V0: 0 %. Scan-selected stair skill: 100 % up, 100 % down (20 trials each), 97 % angled/narrow (30) |
| Success on unseen geometries (other rises/angles) | ≥ 95 % | Switch over the full 220-trial matrix: 99 % (stair skill is third-party; our own V1 still to train) |
| Off-walkway events | 0 | V0: 10/10 on a 5° cross slope; switch: 1/20 on 5–10° cross slopes (plus 1 timeout), 1/10 on 0.6 m narrow stairs |
| Falls per 1 000 steps (flat + ramps) | < 0.1 | V0: no falls on flat or 5° ramps in our tests |
| Contacts with the robot driving (not in protective stop), sim scenarios | 0 | V0 + ISO/TS 15066 monitor + position hold: 0 / 50 (without monitor: 44 / 50); 11 / 50 people walked into the stopped robot ([`safety_summary.csv`](../g1_course/outputs/v0/safety/safety_summary.csv)) |
| Placement accuracy (ring in fixture) | ≥ 99 % within tolerance | not implemented |
| Test matrix run time | < 1 h per candidate policy | 220 locomotion trials in 81 s on a laptop CPU |
| Full route line → lab (with stairs up and down) | ≥ 99 % | Factory route 19.9 m, 24 risers: 88 % (22/25) with camera-based route following (Zhou) |
| Fixed-camera localisation spec | met before go-live | σ ≤ 10 cm, bias ≤ 20 cm, ≥ 2 Hz, ≤ 300 ms (or ≥ 1 Hz / ≤ 1 s with odometry) — measured in a 1025-run sweep |
| Sample tray payload up stairs | tray + rings ≤ tested limit | ~8 kg on a chest carrier up one flight, ~2 kg hand-held (no retraining) |

### Business KPIs

| KPI | Why it matters | How to measure |
|---|---|---|
| **Time to bring a new site online** | The core of "easier": scan + retrain instead of reprogramming | Days from scan to passing the gate. Target *(assumption)*: < 2 weeks per new site |
| **Time to add a new skill** | Efficiency of the pipeline | Demo hours + GPU hours + validation hours per skill |
| **Operator hours saved** | Direct value | Trips/shift × minutes/trip; e.g. 20 trips × 10 min = 3.3 h/shift *(assumption)* |
| **Sample turnaround time** | Faster cleanliness feedback → less scrap if a machine drifts | Minutes from sampling to lab result |
| **Incidents** | Safety is a hard constraint, not a trade-off | Contacts, falls, near-misses per 1 000 h |
| **Reuse across plants** | Scalability | Share of the pipeline (models, tests, safety layer) reused unchanged at the second site |

## 4. Why SKF should care

- **Easier:** the expensive part — the pipeline, test matrix, safety layer — is built once; each new site
  is a scan and a validation run.
- **More efficient:** field failures turn into training data automatically; the robot improves exactly
  where it is weak.
- **Low-risk path:** start with flat, people-free routes (V0-level walking is already reliable on flat
  ground and gentle ramps in our tests) and unlock stairs and shared walkways only when the gates pass.

## 5. Next steps (call to action)

1. **GPU run** (SKF's offered Google Cloud access): train V1 — height map + stair curriculum + step-length
   command — and re-run this repository's test matrix as the acceptance test.
2. **Site scan** of one real SKF route (line → cleanliness lab) to replace our parametric course.
3. **Pilot / thesis** with SKF: one station, one route, the KPIs above.
