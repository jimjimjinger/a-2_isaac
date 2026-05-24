#!/usr/bin/env bash
# launch_web_controller.sh
# Isaac Sim 로버 웹 컨트롤러 실행 스크립트
#
# 사전 조건: Isaac Sim에서 vehicle_v3.usd가
# 실행 중이고 ROS2 Bridge가 활성화되어 있어야 합니다.
#
# 사용법:
#   bash launch_web_controller.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS2 환경 소싱
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[warn] /opt/ros/humble/setup.bash 없음"
fi

WS_INSTALL="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")")/install/setup.bash"
if [ -f "$WS_INSTALL" ]; then
    source "$WS_INSTALL"
fi

echo "======================================================"
echo "  Rover Web Controller"
echo "  http://localhost:8001"
echo "======================================================"
echo ""
echo "  Isaac Sim에서 vehicle_v3.usd를 먼저 실행하세요."
echo "  예상 토픽:"
echo "    /cmd_vel                    (INPUT — 로버 속도 명령)"
echo "    /camera/rover/image_raw     (OUTPUT — 카메라 RGB)"
echo "    /imu/data                   (OUTPUT — IMU)"
echo ""
echo "  브라우저: http://localhost:8001"
echo "  조종: W/A/S/D 또는 방향키, Space = 비상 정지"
echo "======================================================"
echo ""

cd "$SCRIPT_DIR"
python3 main.py
