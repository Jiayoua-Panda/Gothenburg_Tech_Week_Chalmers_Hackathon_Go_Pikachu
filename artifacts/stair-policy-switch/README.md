# Stair-to-flat policy handoff pilot

This is a MuJoCo simulation of a Unitree G1 walking through a corridor, up ten 15 cm risers (31 cm treads, 160 cm usable width), and along a 5.6 m upper corridor. The stair controller is the existing third-party [G1DWAQ_Lab](https://github.com/liuyufei-nubot/G1DWAQ_Lab) `model_9999.pt` checkpoint. The flat controller is the pretrained [Unitree G1 29-DoF velocity V0 policy](https://github.com/unitreerobotics/unitree_rl_lab/tree/main/deploy/robots/g1_29dof/config/policy/velocity/v0) stored in [`g1_step_playback/policy`](../../g1_step_playback/policy/). Neither checkpoint was trained in this project.

## Controller

- G1-DWAQ climbs with a 0.3 m/s forward command. Once the pelvis is at least 0.8 m beyond the final stair and has been upright, elevated, and within 35 cm of the centerline for 20 consecutive control ticks (0.4 simulated seconds), the handoff begins.
- The V0 policy is warmed up for 10 ticks (0.2 s) while G1-DWAQ still controls the robot. Joint order and 50 Hz control rate are checked before the run. The commanded forward speed ramps from 0.3 to 0.6 or 0.8 m/s while a smoothstep weight blends the two controllers' PD torques over 1.5 s. The flat controller then takes over.
- The comparison named `dwaq-fast` keeps the stair policy and raises its speed command at the same trigger. `dwaq` keeps that policy at 0.3 m/s throughout.

The switch trigger and corridor-centering commands use **MuJoCo ground-truth position and heading**. Neither policy uses a camera in this experiment. This establishes a policy handoff in simulation, not a perception-based or real-robot controller.

## Measured result

Each row below uses the same centered initial state. Arrival is when pelvis x exceeds 10.475 m while its height exceeds 1.9 m. Segment statistics use only x = 7.0–10.4 m, after the handoff has settled. Lower standard deviation and lateral offset indicate steadier motion on this particular scene; they are not a general stability guarantee.

| Controller after stairs | Command | Arrival | Upper-corridor mean x speed | Speed SD | Pelvis-height SD | Max lateral offset |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| G1-DWAQ throughout | 0.3 m/s | 43.36 s | 0.237 m/s | 0.109 m/s | 1.061 cm | 12.52 cm |
| G1-DWAQ, faster command | 0.6 m/s | 31.62 s | 0.581 m/s | 0.054 m/s | 0.469 cm | 9.11 cm |
| G1-DWAQ → Unitree V0 | 0.6 m/s | 32.78 s | 0.555 m/s | 0.024 m/s | 0.238 cm | 5.05 cm |
| G1-DWAQ, faster command | 0.8 m/s | 30.06 s | 0.713 m/s | 0.059 m/s | 0.384 cm | 9.20 cm |
| G1-DWAQ → Unitree V0 | 0.8 m/s | 30.90 s | 0.735 m/s | 0.032 m/s | 0.322 cm | 6.41 cm |

At each tested speed, all three fixed starts (−10, 0, +10 cm lateral offset) reached the upper-corridor goal for both faster-command and hybrid controllers: 12/12 runs. The 0.3 m/s baseline also reached the goal from all three starts. These are deterministic tests on one stair geometry; the result is **not** an estimated success rate across unseen scenes. For the centered 0.8 m/s hybrid run, the handoff began at 23.40 s, pelvis x = 5.777 m. It completed the route without the configured fall condition. The G1-DWAQ-only speed increase arrived 0.84 s earlier overall because it did not spend 1.5 s blending; its measured upper-corridor speed was slightly lower and more variable.

Full results are in [`comparison.csv`](comparison.csv), each run's JSON, and its compressed `-trace.json.gz`. The two MP4s have **no text burned into the picture**:

- [G1-DWAQ → V0 at 0.8 m/s](hybrid_cmd80cmps_center_rise15cm_tread31cm_width160cm.mp4)
- [G1-DWAQ only at 0.8 m/s](dwaq-fast_cmd80cmps_center_rise15cm_tread31cm_width160cm.mp4)

## Reproduce

Use the setup in the [stair geometry sweep](../stair-sweep/README.md) for the G1-DWAQ source, checkpoint, MuJoCo, PyTorch, and FFmpeg. Install `onnxruntime` and `pyyaml` for the flat policy. The source checkout used here was G1DWAQ_Lab commit `bebb0ea`; the checkpoint SHA-256 was `5042017a558b98ab24a3784d1f960383ee42015d79b67b24b74f6b6a117759c1`, and the V0 ONNX SHA-256 was `610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f`.

From the repository root, substitute your G1DWAQ_Lab `TienKung-Lab` checkout path:

```bash
python scripts/run_stair_policy_switch.py --mode hybrid --flat-speed 0.8 --start-y-cm 0 --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
python scripts/run_stair_policy_switch.py --mode dwaq-fast --flat-speed 0.8 --start-y-cm 0 --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
```

All velocities above are simulated pelvis motion, not hardware measurements. The scene and physics do not include actuator thermal limits, state-estimation error, sensor latency, or randomized friction/geometry; these should be evaluated before any deployment claim.
