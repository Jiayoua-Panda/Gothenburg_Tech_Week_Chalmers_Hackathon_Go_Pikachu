#!/bin/bash
# =============================================================================
# G1 DWAQ fine-tune (Isaac Sim 5.1.0 + Isaac Lab 2.3.0) on Minerva -- ONE L40S GPU.
# NOT SUBMITTED. Before the first submission the user must read and accept the
# NVIDIA Omniverse / Isaac Sim EULA, then uncomment the OMNI_KIT_ACCEPT_EULA line.
#   https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html
# Submit:   sbatch /data/users/zhichaoz/skf/isaac/train_finetune.sbatch
# Override: MAX_ITERS=3000 NUM_ENVS=4096 sbatch --export=ALL train_finetune.sbatch
# =============================================================================
#SBATCH -J g1dwaq-ft
#SBATCH -p long
#SBATCH --gres=gpu:L40s:1
##SBATCH --nodelist=neptune          # optional: pin a node (neptune|uranus have L40S)
#SBATCH -c 16
#SBATCH --mem=64G
#SBATCH -t 1-00:00:00
#SBATCH -o /data/users/zhichaoz/skf/isaac/logs/g1dwaq-ft_%j.log

set -eo pipefail

export OMNI_KIT_ACCEPT_EULA=YES  # user accepted the NVIDIA Omniverse / Isaac Sim EULA on 2026-09-26
export SKF_FT=${SKF_FT:-payload}
echo "SKF_FT=$SKF_FT"

ROOT=/data/users/zhichaoz/skf/isaac
TK=$ROOT/G1DWAQ_Lab/TienKung-Lab            # separate copy; shared checkout is untouched
MAX_ITERS=${MAX_ITERS:-2000}                 # N = number of ADDITIONAL iterations (see notes)
NUM_ENVS=${NUM_ENVS:-4096}
LOAD_RUN=${LOAD_RUN:-2026-01-16_00-46-00}    # dir under $TK/logs/g1_dwaq/ (regex-matched)
CKPT=${CKPT:-model_9999.pt}                  # file inside LOAD_RUN (regex-matched)
RUN_NAME=${RUN_NAME:-ft_from_9999}

# conda env (python 3.11, isaacsim 5.1.0, isaaclab 2.3.0, torch 2.7.0+cu128)
source /data/users/zhichaoz/miniconda3/bin/activate $ROOT/env

# Slurm on Minerva sets CUDA_VISIBLE_DEVICES but does NOT hide the other GPUs from
# nvidia-smi/Vulkan (no device cgroup). Log what we got so GPU placement can be checked.
echo "host=$(hostname) CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES SLURM_JOB_GPUS=$SLURM_JOB_GPUS"
nvidia-smi --query-gpu=index,name,uuid,memory.used,memory.total --format=csv

if [[ "${OMNI_KIT_ACCEPT_EULA:-}" != "YES" ]]; then
  echo "ERROR: OMNI_KIT_ACCEPT_EULA not set. Isaac Sim would block on the interactive EULA prompt." >&2
  echo "       Accept the EULA first, then uncomment the export line in this script." >&2
  exit 1
fi

cd "$TK"   # train.py resolves logs/ relative to CWD -> $TK/logs/g1_dwaq/...
ls -la "logs/g1_dwaq/$LOAD_RUN/$CKPT"

# NOTE: --resume is argparse type=bool: ANY non-empty value (even "False") means True.
#       To train from scratch, simply omit --resume/--load_run/--checkpoint.
python legged_lab/scripts/train.py \
    --task=g1_dwaq \
    --headless \
    --num_envs=$NUM_ENVS \
    --max_iterations=$MAX_ITERS \
    --resume=True \
    --load_run=$LOAD_RUN \
    --checkpoint=$CKPT \
    --run_name=$RUN_NAME ${EXTRA_ARGS:-}

echo "finished: new run dir is the newest $TK/logs/g1_dwaq/*_$RUN_NAME"
