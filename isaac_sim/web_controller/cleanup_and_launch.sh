#!/usr/bin/env bash
# cleanup_and_launch.sh
# Isaac Sim 이전 세션 잔재를 정리하고 웹 서버를 시작한다.
# 컴퓨터 재시작 없이 깨끗한 상태로 재실행할 때 사용.
#
# 사용법:
#   bash cleanup_and_launch.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================================"
echo "  [1/4] 이전 Isaac Sim / ROS2 프로세스 정리"
echo "======================================================"

# Isaac Sim 관련 프로세스 종료
for pattern in "kit/python" "isaacsim" "omni.isaac" "fastrtps" "fast-rtps"; do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo "  종료: $pattern (PID: $pids)"
        kill $pids 2>/dev/null || true
    fi
done

# 웹 서버 이전 인스턴스 종료 (포트 8001)
pids=$(lsof -ti :8001 2>/dev/null || true)
if [ -n "$pids" ]; then
    echo "  포트 8001 점유 프로세스 종료 (PID: $pids)"
    kill $pids 2>/dev/null || true
fi

sleep 2

# 강제 종료 확인
for pattern in "kit/python" "isaacsim"; do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo "  강제 종료: $pattern"
        kill -9 $pids 2>/dev/null || true
    fi
done

echo ""
echo "======================================================"
echo "  [2/4] ROS2 데몬 재시작"
echo "======================================================"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ros2 daemon stop 2>/dev/null || true
    sleep 1
    ros2 daemon start 2>/dev/null || true
    echo "  ROS2 daemon 재시작 완료"
fi

echo ""
echo "======================================================"
echo "  [3/4] 공유 메모리 / DDS 임시 파일 정리"
echo "======================================================"

# Fast-RTPS / CycloneDDS 공유 메모리
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
rm -f /tmp/.ros/log/latest 2>/dev/null || true
echo "  정리 완료"

echo ""
echo "======================================================"
echo "  [4/4] 웹 서버 시작"
echo "======================================================"

WS_INSTALL="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")")/install/setup.bash"
if [ -f "$WS_INSTALL" ]; then
    source "$WS_INSTALL"
fi

PYTHON="$SCRIPT_DIR/venv/bin/python3"
[ -f "$PYTHON" ] || PYTHON="python3"

echo ""
echo "  브라우저: http://localhost:8001"
echo "  Isaac Sim은 별도 터미널에서 먼저 실행하세요."
echo "======================================================"
echo ""

cd "$SCRIPT_DIR"
"$PYTHON" main.py
