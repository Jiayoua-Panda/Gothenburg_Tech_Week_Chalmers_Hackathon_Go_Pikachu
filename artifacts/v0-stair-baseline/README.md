# Unitree V0 flat-walking policy on the existing stair scenes

This is a small capability check, not a new training run. The pretrained Unitree G1 29-DoF velocity V0 ONNX policy is run from the lower corridor with its own initial joint pose and PD gains. The MuJoCo robot body and generated 10-step stair scene are the same as in the [G1-DWAQ geometry sweep](../stair-sweep/README.md). The command is 0.3 m/s; position and heading from **MuJoCo ground truth** provide the same form of corridor-centering feedback. V0's deployment yaw-command range is narrower, so the two policies' low-level controllers are not identical. No camera or height-map observation enters V0.

The flat control keeps the robot body, infinite ground plane, policy, command, and goal x position, but disables and hides all scene-added stair, platform, and wall geometry. It verifies that V0 can walk the route length under this robot model and controller before interpreting a stair failure.

| Scene | Initial lateral offset | Outcome | Simulation time | Maximum pelvis x |
| --- | ---: | --- | ---: | ---: |
| Flat control | 0 cm | Goal | 30.76 s | 8.008 m |
| 8 cm risers | 0 cm | Fall near first steps | 5.26 s | 1.709 m |
| 15 cm risers | −10 cm | Fall near first steps | 7.06 s | 1.711 m |
| 15 cm risers | 0 cm | Fall near first steps | 6.78 s | 1.697 m |
| 15 cm risers | +10 cm | Fall near first steps | 7.78 s | 1.710 m |
| 18 cm risers | 0 cm | Fall near first steps | 7.08 s | 1.583 m |
| 20 cm risers | 0 cm | Fall near first steps | 6.42 s | 1.504 m |

All stairs have 31 cm treads and 160 cm usable width. The first riser starts at x = 1.155 m. A fall means pelvis z < 0.32 m; the same threshold was used in the earlier G1-DWAQ sweep. The goal is pelvis x > 8.005 m with z > stair total height + 0.4 m, within 60 simulated seconds. The 15 cm G1-DWAQ comparison reached the goal in 3/3 fixed starts; its 18 cm condition reached the goal in 2/3, while its 20 cm condition reached the goal in 0/3. Those observations do not establish a general stair-height limit or real-robot performance.

These two videos have **no text in the picture**:

- [V0 walking the flat control to the goal](goal_v0_flat_control_cmd30cmps_center_sim-truth-centering.mp4)
- [V0 falling at the 15 cm stair](fall_v0_rise15cm_tread31cm_width160cm_cmd30cmps_center_sim-truth-centering.mp4)

Each run has a JSON summary and compressed `t,x,y,z` trace in this directory. The source policy is the [official Unitree V0 velocity configuration](https://github.com/unitreerobotics/unitree_rl_lab/tree/main/deploy/robots/g1_29dof/config/policy/velocity/v0) copied into [`g1_step_playback/policy`](../../g1_step_playback/policy/), ONNX SHA-256 `610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f`.

Reproduce after installing `mujoco numpy onnxruntime pyyaml ffmpeg` and the G1DWAQ_Lab robot MJCF and meshes described in the [stair sweep](../stair-sweep/README.md):

```bash
python scripts/probe_v0_stairs.py --rise-cm 15 --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
python scripts/probe_v0_stairs.py --rise-cm 15 --flat-control --video --source-root /path/to/G1DWAQ_Lab/TienKung-Lab
```

The V0 failure is a measured result for this checkpoint and this simulator setup. It does not prove that all flat-walking policies fail stairs, or identify whether training distribution, observation design, robot dynamics, or control limits are the dominant cause.
