#!/bin/bash
# run_curriculum.sh — 500 iter씩 끊어서 학습, 자동 이어받기
#
# 사용법:
#   처음 시작:           ./run_curriculum.sh
#   특정 체크포인트에서: ./run_curriculum.sh /path/to/model_500.pt

set -e

ISAACLAB="/mnt/data/isaac_sim/IsaacLab/isaaclab.sh"
SCRIPT="$(cd "$(dirname "$0")" && pwd)/train_recovery.py"
LOG_ROOT="/home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery"
NUM_ENVS=128   # RTX 5060 8GB: 128 envs ~4.8GB 사용
CHUNK=500       # 한 번에 학습할 iter 수
TOTAL=3000      # 총 목표 iter 수

CHECKPOINT="${1:-}"   # 첫 번째 인자로 체크포인트 경로 지정 가능

echo "=============================="
echo "  Rover Recovery 분할 학습"
echo "  총 iter: $TOTAL  / chunk: $CHUNK"
echo "  envs: $NUM_ENVS"
echo "=============================="

for START in $(seq 0 $CHUNK $((TOTAL - 1))); do
    END=$((START + CHUNK))
    RUN=$((START / CHUNK + 1))
    echo ""
    echo ">>> Run $RUN: iter $START → $END"
    echo "    체크포인트: ${CHECKPOINT:-없음 (처음 시작)}"

    if [ -n "$CHECKPOINT" ]; then
        $ISAACLAB -p "$SCRIPT" \
            --num_envs $NUM_ENVS \
            --headless \
            --max_iterations $CHUNK \
            --eval_interval 999999 \
            --checkpoint "$CHECKPOINT"
    else
        $ISAACLAB -p "$SCRIPT" \
            --num_envs $NUM_ENVS \
            --headless \
            --max_iterations $CHUNK \
            --eval_interval 999999
    fi

    # 숫자 기준으로 가장 높은 번호의 체크포인트 선택 (ls -t는 model_0.pt를 잘못 선택할 수 있음)
    LATEST_DIR=$(ls -td "$LOG_ROOT"/2* 2>/dev/null | head -1)
    if [ -z "$LATEST_DIR" ]; then
        echo "[ERROR] 로그 디렉토리를 찾을 수 없음: $LOG_ROOT"
        exit 1
    fi

    CHECKPOINT=$(ls "$LATEST_DIR/model_"*.pt 2>/dev/null | sort -t_ -k2 -n | tail -1)
    if [ -z "$CHECKPOINT" ]; then
        echo "[ERROR] 체크포인트를 찾을 수 없음: $LATEST_DIR/model_*.pt"
        exit 1
    fi

    echo "    완료. 다음 체크포인트: $CHECKPOINT"
done

echo ""
echo "=============================="
echo "  전체 학습 완료! ($TOTAL iter)"
echo "  최종 체크포인트: $CHECKPOINT"
echo "=============================="
