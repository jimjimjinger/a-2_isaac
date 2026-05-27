#!/bin/bash
# run_curriculum_v4.sh — v4 커리큘럼 학습
#
# Stage 1: flat only, goal=0.5m
# Stage 2: flat + rough, goal=1.0m
# Stage 3: rough + slope, goal=1.5m
# Stage 4: full terrain incl. crater, goal=2.0m
#
# 사용법:
#   처음 시작:           ./run_curriculum_v4.sh
#   체크포인트에서:       ./run_curriculum_v4.sh /path/to/model_1000.pt

set -e

ISAACLAB="/mnt/data/isaac_sim/IsaacLab/isaaclab.sh"
SCRIPT="$(cd "$(dirname "$0")" && pwd)/train_recovery_v4.py"
LOG_ROOT="/home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery_v4"
NUM_ENVS=128
CHUNK=500    # 500 iter씩 저장 (OOM 대비 잦은 체크포인트)

# stage별 총 iter
STAGE_ITERS=(1000 1500 2000 3000)
STAGE_NAMES=("stage1_flat" "stage2_rough" "stage3_slope" "stage4_crater")
STAGE_TERRAIN=(1 2 3 4)
STAGE_GOALS=(0.5 1.0 1.5 2.0)

CHECKPOINT="${1:-}"
LOG_DIR=""

echo "=============================="
echo "  Rover Recovery v4 커리큘럼"
echo "  stages: ${#STAGE_ITERS[@]}  / envs: $NUM_ENVS"
echo "=============================="

for i in "${!STAGE_ITERS[@]}"; do
    STAGE_ITER="${STAGE_ITERS[$i]}"
    STAGE_NAME="${STAGE_NAMES[$i]}"
    STAGE_TERRAIN_ID="${STAGE_TERRAIN[$i]}"
    STAGE_GOAL="${STAGE_GOALS[$i]}"
    STAGE_NUM=$((i + 1))

    echo ""
    echo ">>> Stage $STAGE_NUM: $STAGE_NAME  ($STAGE_ITER iter)"
    echo "    terrain stage: $STAGE_TERRAIN_ID / forward goal: $STAGE_GOAL m"
    echo "    체크포인트: ${CHECKPOINT:-없음}"

    COMMON="--num_envs $NUM_ENVS --headless --max_iterations $STAGE_ITER"
    [ -n "$LOG_DIR" ] && COMMON="$COMMON --resume_log_dir $LOG_DIR"

    export ROVER_RECOVERY_STAGE="$STAGE_TERRAIN_ID"
    export ROVER_FORWARD_GOAL_M="$STAGE_GOAL"

    if [ -n "$CHECKPOINT" ]; then
        $ISAACLAB -p "$SCRIPT" $COMMON --checkpoint "$CHECKPOINT"
    else
        $ISAACLAB -p "$SCRIPT" $COMMON
    fi

    EXIT_CODE=$?
    if [ $EXIT_CODE -ne 0 ]; then
        echo "[ERROR] Stage $STAGE_NUM 실패 (exit=$EXIT_CODE)"
        echo "  재시작: ./run_curriculum_v4.sh $CHECKPOINT"
        exit $EXIT_CODE
    fi

    LATEST_DIR=$(ls -td "$LOG_ROOT"/2* 2>/dev/null | head -1)
    [ -z "$LATEST_DIR" ] && { echo "[ERROR] 로그 디렉토리 없음: $LOG_ROOT"; exit 1; }

    CHECKPOINT=$(ls "$LATEST_DIR/model_"*.pt 2>/dev/null | sort -V | tail -1)
    [ -z "$CHECKPOINT" ] && { echo "[ERROR] 체크포인트 없음: $LATEST_DIR"; exit 1; }

    LOG_DIR="$LATEST_DIR"
    echo "    완료. 체크포인트: $CHECKPOINT"
done

echo ""
echo "=============================="
echo "  v4 전체 학습 완료!"
echo "  최종 체크포인트: $CHECKPOINT"
echo "  log: $LOG_DIR"
echo "  TensorBoard: tensorboard --logdir $LOG_ROOT"
echo "=============================="
