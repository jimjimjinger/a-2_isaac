#!/bin/bash
# run_curriculum.sh — 1000 iter씩 끊어서 학습, LR/log 자동 이어받기
#
# 사용법:
#   처음 시작:           ./run_curriculum.sh
#   특정 체크포인트에서: ./run_curriculum.sh /path/to/model_1000.pt

set -e

ISAACLAB="/mnt/data/isaac_sim/IsaacLab/isaaclab.sh"
SCRIPT="$(cd "$(dirname "$0")" && pwd)/train_recovery.py"
LOG_ROOT="/home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery"
NUM_ENVS=128   # RTX 5060 8GB: 128 envs ~4.8GB
CHUNK=1000
TOTAL=3000

CHECKPOINT="${1:-}"
LOG_DIR=""     # 첫 chunk에서 생성된 dir을 이후 chunk가 이어받음

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
    echo "    log dir:   ${LOG_DIR:-새로 생성}"

    # 공통 인수
    COMMON_ARGS="--num_envs $NUM_ENVS --headless --max_iterations $CHUNK --eval_interval 999999"

    # chunk 2+ 는 log dir 이어받기 (TensorBoard 연속)
    if [ -n "$LOG_DIR" ]; then
        COMMON_ARGS="$COMMON_ARGS --resume_log_dir $LOG_DIR"
    fi

    if [ -n "$CHECKPOINT" ]; then
        $ISAACLAB -p "$SCRIPT" $COMMON_ARGS --checkpoint "$CHECKPOINT"
        EXIT_CODE=$?
    else
        $ISAACLAB -p "$SCRIPT" $COMMON_ARGS
        EXIT_CODE=$?
    fi

    if [ $EXIT_CODE -ne 0 ]; then
        echo "[ERROR] 학습 실패 (exit code: $EXIT_CODE). CUDA 오류 시 재부팅 후:"
        echo "  ./run_curriculum.sh $CHECKPOINT"
        exit $EXIT_CODE
    fi

    # 숫자 기준으로 가장 높은 번호의 체크포인트 선택
    LATEST_DIR=$(ls -td "$LOG_ROOT"/2* 2>/dev/null | head -1)
    if [ -z "$LATEST_DIR" ]; then
        echo "[ERROR] 로그 디렉토리를 찾을 수 없음: $LOG_ROOT"
        exit 1
    fi

    CHECKPOINT=$(ls "$LATEST_DIR/model_"*.pt 2>/dev/null | sort -V | tail -1)
    if [ -z "$CHECKPOINT" ]; then
        echo "[ERROR] 체크포인트를 찾을 수 없음: $LATEST_DIR/model_*.pt"
        exit 1
    fi

    # 첫 번째 run의 log dir을 이후 run이 이어받음
    LOG_DIR="$LATEST_DIR"

    echo "    완료. 다음 체크포인트: $CHECKPOINT"
    echo "    연속 log dir: $LOG_DIR"
done

echo ""
echo "=============================="
echo "  전체 학습 완료! ($TOTAL iter)"
echo "  최종 체크포인트: $CHECKPOINT"
echo "  log dir: $LOG_DIR"
echo "=============================="
