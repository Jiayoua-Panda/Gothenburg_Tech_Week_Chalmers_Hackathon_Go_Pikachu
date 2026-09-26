# Morning summary — SKF humanoid stairs

_Generated 2026-09-26 08:46 by `g1_course/morning_summary.py` (re-run any time)._

| Job | Status |
|---|---|
| V0.5 overnight tuning + evaluation (`outputs/overnight.log`) | done |
| Stairs evaluation, DWAQ + switch (`outputs/stairs_eval.log`) | done |
| Switch tour video (`outputs/switch/course_tour.mp4`) | done |

## Pass rate by terrain group

| Terrain | V0 (Unitree, blind) | V0.5 hybrid (ours) | G1-DWAQ only | **Switch: scan picks V0 / DWAQ** |
|---|---|---|---|---|
| Flat and 5° slopes | 100% (40/40) | 50% (10/20) | 100% (40/40) | **100% (40/40)** |
| Rough floor 2–4 cm (3D) | 95% (19/20) | 50% (10/20) | 100% (20/20) | **100% (20/20)** |
| Downhill 10–15° | 90% (18/20) | – | 100% (20/20) | **100% (20/20)** |
| Uphill 10–15° | 0% (0/20) | 0% (0/20) | 100% (20/20) | **100% (20/20)** |
| Cross slope 5–10° (3D) | 0% (0/20) | 50% (10/20) | 100% (20/20) | **90% (18/20)** |
| Single steps 5–15 cm | 0% (0/30) | 23% (7/30) | 100% (30/30) | **100% (30/30)** |
| Stairs up 10–15 cm | 0% (0/20) | 0% (0/20) | 95% (19/20) | **100% (20/20)** |
| Stairs down 10–15 cm | 10% (2/20) | 10% (2/20) | 95% (19/20) | **100% (20/20)** |
| Angled / narrow stairs (3D) | 0% (0/30) | 0% (0/30) | 93% (28/30) | **97% (29/30)** |

## Per segment

| Segment | V0 | V0.5 | DWAQ | Switch |
|---|---|---|---|---|
| A warm-up | 100% (20/20) | 100% (10/10) | 100% (20/20) | 100% (20/20) |
| B1 up 5deg | 100% (10/10) | 0% (0/10) *test* | 100% (10/10) | 100% (10/10) |
| B1 down 5deg | 100% (10/10) | – | 100% (10/10) | 100% (10/10) |
| G1 rough 2cm | 100% (10/10) | 90% (9/10) *test* | 100% (10/10) | 100% (10/10) |
| G2 rough 4cm | 90% (9/10) | 10% (1/10) *test* | 100% (10/10) | 100% (10/10) |
| B2 down 10deg | 90% (9/10) | – | 100% (10/10) | 100% (10/10) |
| B3 down 15deg | 90% (9/10) | – | 100% (10/10) | 100% (10/10) |
| B2 up 10deg | 0% (0/10) | 0% (0/10) | 100% (10/10) | 100% (10/10) |
| B3 up 15deg | 0% (0/10) | 0% (0/10) *test* | 100% (10/10) | 100% (10/10) |
| F1 cross 5deg | 0% (0/10) | 100% (10/10) | 100% (10/10) | 90% (9/10) |
| F2 cross 10deg | 0% (0/10) | 0% (0/10) *test* | 100% (10/10) | 90% (9/10) |
| C1 step 5cm | 0% (0/10) | 60% (6/10) | 100% (10/10) | 100% (10/10) |
| C2 step 10cm | 0% (0/10) | 10% (1/10) | 100% (10/10) | 100% (10/10) |
| C3 step 15cm | 0% (0/10) | 0% (0/10) *test* | 100% (10/10) | 100% (10/10) |
| D1 up 10/30 | 0% (0/10) | 0% (0/10) | 100% (10/10) | 100% (10/10) |
| D2 up 15/25 | 0% (0/10) | 0% (0/10) *test* | 90% (9/10) | 100% (10/10) |
| E1 down 15/30 | 0% (0/10) | 10% (1/10) *test* | 90% (9/10) | 100% (10/10) |
| E2 down 10/30 | 20% (2/10) | 10% (1/10) *test* | 100% (10/10) | 100% (10/10) |
| H1 stairs 15deg | 0% (0/10) | 0% (0/10) | 100% (10/10) | 100% (10/10) |
| H2 stairs 30deg | 0% (0/10) | 0% (0/10) *test* | 100% (10/10) | 100% (10/10) |
| I narrow 0.6m | 0% (0/10) | 0% (0/10) *test* | 80% (8/10) | 90% (9/10) |

**Switch tour (one continuous run over the whole 2D course):** 12/14 segments passed → video `outputs/switch/course_tour.mp4`.

## How to read this

- V0.5 numbers come from `outputs/hybrid/evaluation.csv`; segments marked *test* were never used for tuning (held-out).
- DWAQ = third-party G1-DWAQ weights (G1DWAQ_Lab, BSD-3), **not trained by us**; we re-implemented its inference and run it on our robot model and course. Switch = our height scan decides when to use it.
- All controllers steer to the walkway centre with simulator position (on a real robot: localisation).
- V0 / DWAQ / switch: 10 randomised starts per segment (`run_course.py --mode segments`).
