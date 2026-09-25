# Task 2 (desired): Manipulation — pick & place, and wiping a surface

This part is a **theoretical feasibility analysis** — we did not implement manipulation. Evidence tags:
**[reported]** = published results · **[proposed]** = our design.

## 1. The tasks, framed for SKF

| Task | SKF version | What makes it hard |
|---|---|---|
| **Pick & place** | Take a bearing ring (e.g. 0.2–2 kg) from a tray at the machining line and place it in a washing or cleanliness-inspection fixture | Precise placement (mm), smooth/oily metal surfaces, rings of many sizes, must not damage surfaces |
| **Wipe the table / clean the whiteboard** | Wipe a workbench or inspection surface before cleanliness measurement | Contact force control (press enough, not too much), coverage of the whole area, deformable cloth |

Why the same pipeline fits ([pipeline.md](pipeline.md)): Collect → Train → Simulate → Validate → Deploy
is unchanged; what changes is **Collect** (human demonstrations instead of reward engineering) and
**Train** (imitation learning instead of pure RL).

## 2. Recommended method — hybrid **[proposed]**

```
 teleoperated demos ──► imitation learning (ACT / Diffusion Policy) ──► RL fine-tune in sim (optional)
        ▲                          ▲                                        │
        │                 VLA / foundation model prior                      ▼
   operator in VR           (language → which skill, where)          validate on twin → deploy
```

1. **Collect demonstrations by teleoperation.** An operator wearing a VR/XR headset drives the G1's arms
   and hands (Unitree publishes open-source XR teleoperation tooling for G1 — verify the current repo).
   Record camera images, joint states and hand commands.
2. **Train an imitation-learning policy.** ACT (Zhao et al., 2023, ALOHA) and Diffusion Policy (Chi et al.,
   2023) learned fine bimanual tasks from roughly tens of demonstrations per task **[reported]**.
   Plan for 50–200 demos per task variant, ~1–3 h of operator time.
3. **Use a VLA/foundation model for the "what and where".** Models such as OpenVLA, π0 or NVIDIA GR00T N1
   (a humanoid foundation model) map a language instruction and camera images to actions **[reported]**.
   Today their precision and reliability are below industrial requirements, so we use them as the
   *instruction and pre-training* layer, fine-tuned on SKF demos — not as the final controller.
4. **Refine in simulation where physics is well modelled.** Rigid pick & place: yes (grasp success reward,
   domain randomisation of ring size/friction/pose). Wiping with a cloth: only partly (deformables are
   poorly simulated) → rely more on demonstrations + force limits.
5. **Wrap with classical control and rules.** Force/torque limits, fixed approach poses, "placed correctly"
   check by vision or a fixture sensor. This is the same lesson as our locomotion safety test: *wrap the
   learned policy in a guaranteed classical layer* (we measured that V0's learned "stop" was not a real stop).

## 3. Locomotion vs manipulation — why the recipe differs

| | Locomotion (stairs) | Manipulation (pick & place / wipe) |
|---|---|---|
| Main learning method | RL in simulation | Imitation learning from demos (+ RL fine-tune) |
| Human data needed | None | Tens–hundreds of demos per task |
| Simulation fidelity | Good (rigid contacts, known geometry) | Good for rigid parts; weak for cloth, liquids, oily surfaces |
| Transfer to a new site | Scan → twin → retrain/validate (automatic) | New fixtures/objects → a few new demos |
| Failure consequence | Fall (robot damage, danger on stairs) | Dropped/damaged part, contamination, pinch risk |
| Maturity today **[reported]** | Perceptive stairs shown on real humanoids in research | Lab demos of fine manipulation; industrial reliability not yet shown at scale |

## 4. Assessment against SKF's checkpoints

| Checkpoint | Pick & place | Wipe table / whiteboard |
|---|---|---|
| **Feasibility & implementation** | High for a fixed station with known ring types; teleop + ACT is a well-trodden path | Medium: coverage is easy, consistent contact force is the hard part |
| **Benefits** | Flexible labour for low-volume, high-mix handling (many ring sizes) where a fixed robot cell is not worth it; same robot also walks between stations | Removes a repetitive manual cleaning step before cleanliness measurement |
| **Limitations** | Hand precision/strength, occlusion by the hand, oily surfaces; cycle time slower than a dedicated robot cell | Cloth simulation weak; wear of the cloth; verifying "clean" needs a separate measurement |
| **Data needs** | 50–200 teleop demos per variant, camera + joint logs; object CAD for simulation | Demos with force data; ideally a wrist force/torque sensor |
| **Safety** | Pinch points, dropped parts; power & force limiting (ISO/TS 15066) for any contact with people | Low-energy task; main risk is the arm reaching into a person's space |
| **Industrial readiness** | Pilot-ready at one station within months; not yet a certified product | Pilot-ready for non-critical surfaces |

## 5. How we would validate it (same gate idea as locomotion)

- Test matrix: ring size × pose on tray × lighting × fixture position (± 2 cm) × clutter.
- Gate: ≥ 99 % correct placements over 1 000 simulated episodes, ≥ 98 % over 200 real trials at the pilot
  station, zero placements outside tolerance counted as "silent errors".
- Failure loop: every failed grasp is logged with images → added as a new demo (operator corrects it via
  teleop) → retrain → re-validate. This is where "easier and more efficient" comes from over time:
  the robot gets data exactly where it is weak.
