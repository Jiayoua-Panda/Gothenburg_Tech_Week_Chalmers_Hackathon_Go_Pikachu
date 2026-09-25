# Comparison of learning methods for industrial humanoids

Scores are our **qualitative assessment** (1 = poor, 5 = strong for industrial use), based on the published
results cited in [task_locomotion.md](task_locomotion.md) and [task_manipulation.md](task_manipulation.md)
and on our own simulation tests. They are meant to structure a discussion, not to be precise.

## 1. The eight industrial criteria from the SKF brief

| Method | Data effort | Training time | Safety | Repeatability | Adaptability | Compute needs | Explainability | Deployment maturity |
|---|---|---|---|---|---|---|---|---|
| **Manual programming** (scripted trajectories, classical control) | 4 — engineering time, no data | 2 — weeks of expert tuning per task/site | 5 — fully verifiable | 5 | 1 — breaks when anything changes | 5 | 5 | 5 for arms; 2 for humanoid balance on terrain |
| **Teleoperation / learning from demonstration** | 2 — operator hours per task | 4 | 3 — human in the loop | 3 | 3 | 4 | 3 | 3 |
| **Imitation learning** (ACT, Diffusion Policy) | 3 — tens–hundreds of demos | 4 — hours | 3 | 3 — distribution shift outside demos | 3 | 3 | 2 | 3 |
| **Reinforcement learning in simulation** | 5 — no human data | 3 — GPU-hours per run + reward design | 2 alone / 4 with safety layer | 4 in-distribution | 4 via randomisation | 2 — GPU | 2 | 3 locomotion (Unitree ships RL walking policies) / 2 manipulation |
| **Sim-to-real transfer** (domain randomisation, teacher–student, sim2sim) | 4 | 3 | 4 — test before hardware | 4 | 4 | 2 | 3 | 3 |
| **Human feedback** (corrections, preference/RLHF, DAgger-style) | 3 — continuous but small | 4 — incremental | 4 | 4 | 4 | 3 | 3 | 2 |
| **VLA / foundation models** (OpenVLA, π0, GR00T N1) | 3 — needs task fine-tuning data | 2 — large pre-training (done by vendor) | 2 — hard to bound | 2 | 5 — language, new objects | 1 — large GPU at inference | 1 | 1–2 |

## 2. Which method for which kind of task

| | Locomotion | Manipulation | Complete industrial workflow |
|---|---|---|---|
| **Best primary method** | RL in simulation + sim-to-real | Teleop demos + imitation learning | Hybrid: VLA/LLM planner → learned skills → rule-based safety |
| **Why** | Physics is well simulated; no human can demonstrate balance torques; millions of trials are free in sim | Humans demonstrate hand tasks easily; contact-rich/deformable physics is poorly simulated | No single method covers language, planning, balance, dexterity and certifiable safety |
| **Where the others fit** | Manual/classical: safety layer, position hold, stand-up/crouch; Human feedback: fix field failures | RL: fine-tune grasps in sim; Human feedback: operator corrections | Human feedback: supervisors correct the plan; Manual: interlocks, zones |

## 3. Conclusion — hybrids are the realistic answer

1. **No single method meets all eight criteria.** The methods that adapt best (RL, VLA) are the least
   explainable and certifiable; the most certifiable (manual programming) does not adapt.
2. **Layer them by what each is good at:**
   - *Language/VLA layer* for flexibility: "what to do".
   - *Learned skill policies* (RL for walking, imitation learning for hands) for robustness: "how to move".
   - *Classical, rule-based layer* for guarantees: "never do this" — speed & separation monitoring,
     position hold, force limits, restricted zones.
3. **We saw this in our own test.** Unitree's learned walking policy walks flat ground perfectly but its
   "stop" is not a stop: at zero velocity command it kept creeping toward a person (0.27 m on average).
   A 10-line classical position-hold loop around it cut the creep to 0.05 m, and an ISO/TS 15066 separation
   distance computed from the measured stopping behaviour cut contacts in our scenarios from 44/50 to 11/50 —
   every remaining one with the robot already stopped. Learned + classical beat either alone.
   (Source: [`g1_course/outputs/v0/safety/`](../g1_course/outputs/v0/safety/).)
4. **What makes training easier and more efficient** is not the choice of one algorithm but the pipeline
   around them: site scan → digital twin → automatic test matrix → failure loop ([pipeline.md](pipeline.md)).
