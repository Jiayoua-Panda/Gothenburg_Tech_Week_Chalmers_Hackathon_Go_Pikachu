# G1-DWAQ fine-tuning: reward function, config and run parameters

> 中文摘要：微调**没有改奖励函数**，用的是 G1-DWAQ 原版奖励（见下表和 [`g1_dwaq_config.py`](g1_dwaq_config.py) 里的 `G1DwaqRewardCfg`，函数实现在 [`mdp_rewards.py`](mdp_rewards.py)）。我们只改了域随机化（躯干负重 + 重心前移）和地形比例（更多下楼梯），由环境变量 `SKF_FT` 选择。改动见 [`skf_finetune.diff`](skf_finetune.diff)。

These are the files behind the six fine-tunes reported in [`START_HERE.md`](../../START_HERE.md) and
[`artifacts/factory-course/finetune-results.csv`](../../artifacts/factory-course/finetune-results.csv) (on the `factory-course` branch).
Training ran in Isaac Sim 5.1 + Isaac Lab 2.3 on Chalmers Minerva (one L40S per run), starting from the third-party
checkpoint `2026-01-16_00-46-00/model_9999.pt` of [G1DWAQ_Lab](https://github.com/liuyufei-nubot/G1DWAQ_Lab)
(commit `bebb0ea`), 2000 additional PPO iterations each, 4096 environments.

## What changed, and what did not

- **Reward: unchanged.** All runs use the original `G1DwaqRewardCfg`. `legged_lab/mdp/rewards.py` is unmodified.
- **Domain randomisation and terrain: changed**, selected by the environment variable `SKF_FT`
  (the whole change is the 27-line block in [`skf_finetune.diff`](skf_finetune.diff)):

| `SKF_FT` | Torso added mass | Torso CoM shift (x / y / z) | Terrain |
|---|---|---|---|
| unset (original) | original setting | none | `ROUGH_TERRAINS_CFG` |
| `payload` | 0 … 12 kg | 0 … +12 cm / ±2 cm / −2 … +5 cm | unchanged |
| `payload_heavy` | 0 … 20 kg | 0 … +15 cm / ±2 cm / −2 … +5 cm | unchanged |
| `descent` | original setting | none | `stairs_down*` share ×2, all other sub-terrains ×0.75 |
| `payload_descent` | as `payload` | as `payload` | as `descent` |

The CoM shift models a load carried on the chest; the arms stay under the policy.

## Reward terms (original G1-DWAQ, used unchanged)

| Term | Function | Weight | Parameters |
|---|---|---:|---|
| `track_lin_vel_xy_exp` | `track_lin_vel_xy_yaw_frame_exp` | 2.0 | std 0.5 |
| `track_ang_vel_z_exp` | `track_ang_vel_z_world_exp` | 2.0 | std 0.5 |
| `lin_vel_z_l2` | `lin_vel_z_l2` | −1.0 | |
| `ang_vel_xy_l2` | `ang_vel_xy_l2` | −0.05 | |
| `energy` | `energy` | −1e-3 | |
| `dof_acc_l2` | `joint_acc_l2` | −2.5e-7 | |
| `action_rate_l2` | `action_rate_l2` | −0.01 | |
| `undesired_contacts` | `undesired_contacts` | −1.0 | all bodies except ankles, threshold 1 N |
| `fly` | `fly` | −1.0 | both ankle_roll off the ground, threshold 1 N |
| `body_orientation_l2` | `body_orientation_l2` | −2.0 | torso |
| `flat_orientation_l2` | `flat_orientation_l2` | −1.0 | |
| `termination_penalty` | `is_terminated` | −200.0 | |
| `feet_air_time` | `feet_air_time_positive_biped` | 0.15 | threshold 0.4 s |
| `feet_slide` | `feet_slide` | −0.25 | ankle_roll |
| `feet_force` | `body_force` | −3e-3 | threshold 500 N, max 400 |
| `feet_too_near` | `feet_too_near_humanoid` | −2.0 | threshold 0.2 m |
| `feet_stumble` | `feet_stumble` | −2.0 | ankle_roll |
| `dof_pos_limits` | `joint_pos_limits` | −2.0 | |
| `joint_deviation_hip` | `joint_deviation_l1_always` | −0.3 | hip yaw, hip roll |
| `joint_deviation_ankle` | `joint_deviation_l1_always` | −0.2 | ankles |
| `joint_deviation_arms` | `joint_deviation_l1_always` | −0.2 | waist, shoulders, elbows, wrists |
| `joint_deviation_legs` | `joint_deviation_l1_always` | −0.02 | hip pitch, knee |
| `alive` | `alive` | 0.15 | |
| `idle_penalty` | `idle_when_commanded` | −2.0 | command > 0.2 m/s but speed < 0.1 m/s |
| `gait_phase_contact` | `gait_phase_contact` | 0.2 | stance phase < 0.55 |
| `feet_swing_height` | `feet_swing_height` | −0.2 | target 0.08 m |

The exact values actually used by each run are also recorded in `runs/*/env.yaml` (section `rewards`).

## Runs

| Run directory on Minerva | `SKF_FT` | Seed | Final checkpoint |
|---|---|---:|---|
| `2026-09-26_04-26-34_ft_payload` | `payload` | 42 | `model_11998.pt` |
| `2026-09-26_04-48-24_ft_payload_s7` | `payload` | 7 | `model_11998.pt` |
| `2026-09-26_04-26-36_ft_payload_descent` | `payload_descent` | 42 | `model_11998.pt` |
| `2026-09-26_04-48-26_ft_payload_descent_s7` | `payload_descent` | 7 | `model_11998.pt` |
| `2026-09-26_04-26-36_ft_descent` | `descent` | 42 | `model_11998.pt` |
| `2026-09-26_04-26-35_ft_payload_heavy` | `payload_heavy` | 42 | `model_11998.pt` |

`runs/<run>/` holds each run's `env.yaml`, `agent.yaml` and `run_meta.json` as written by the trainer. PPO settings:
lr 1e-3 (adaptive, desired KL 0.01), γ 0.99, λ 0.95, clip 0.2, entropy 0.01, 5 epochs × 4 mini-batches, 24 steps per env.
Checkpoints (7.9 MB each) stay on Minerva under
`/data/users/zhichaoz/skf/isaac/G1DWAQ_Lab/TienKung-Lab/logs/g1_dwaq/`.

## Reproduce

1. Clone G1DWAQ_Lab at `bebb0ea` and apply [`skf_finetune.diff`](skf_finetune.diff) (or copy [`g1_dwaq_config.py`](g1_dwaq_config.py) to
   `TienKung-Lab/legged_lab/envs/g1/g1_dwaq_config.py`).
2. Put the base checkpoint at `TienKung-Lab/logs/g1_dwaq/2026-01-16_00-46-00/model_9999.pt`.
3. Submit one job per variant with [`train_finetune.sbatch`](train_finetune.sbatch) (edit the paths for your cluster):

```bash
SKF_FT=payload RUN_NAME=ft_payload sbatch --export=ALL train_finetune.sbatch
SKF_FT=payload RUN_NAME=ft_payload_s7 EXTRA_ARGS="--seed=7" sbatch --export=ALL train_finetune.sbatch
```

The seed-42 runs used the default seed; the recorded seed of every run is in its `run_meta.json`.
Evaluate a checkpoint on the MuJoCo factory course with `scripts/run_factory_course.py --checkpoint /path/to/model_11998.pt`.

## Result in one line

Chest 8 kg on the full route: 6/25 → 20/25 (`ft_payload`, seed 7), but no load: 22/25 → 3/25. The payload
randomisation traded light-load performance for heavy-load performance; see `START_HERE.md` on the `factory-course` branch.

## License

`g1_dwaq_config.py` and `mdp_rewards.py` are from G1DWAQ_Lab (BSD-3-Clause, see [`LICENSE-G1DWAQ_Lab`](LICENSE-G1DWAQ_Lab));
`g1_dwaq_config.py` contains our `SKF_FT` block.
