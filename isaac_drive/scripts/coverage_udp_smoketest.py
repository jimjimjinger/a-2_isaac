"""coverage UDP 릴레이 체인 스모크 테스트 (Isaac Sim 없이).

CoverageNode + CoverageUdpRelay + MockUdpSim 를 한 프로세스에서 함께 돌려,
sim_ros_bridge 가 쓸 UDP 경로가 ROS2 와 제대로 맞물리는지 검증한다:

  CoverageNode ─ROS2─ CoverageUdpRelay ─UDP─ MockUdpSim
       ▲                                          │
       └────────── /rover/estimated_pose ◀────────┘

MockUdpSim 은 sim_ros_bridge 의 UDP 측을 흉내내는 unicycle mock(Isaac Sim 대역).
N초 후 reveal % 가 올랐으면 UDP↔ROS2 배선 정상.
※ Isaac Sim 물리 검증이 아니라 릴레이 배선 검증이다.

실행 (시스템 ROS2 source 후):
    python3 scripts/coverage_udp_smoketest.py
"""
import math
import os
import socket
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # isaac_drive 패키지
sys.path.insert(0, HERE)                    # scripts/ (coverage_udp_relay)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from isaac_drive.coverage_node import CoverageNode
from coverage_udp_relay import CMD_FMT, POSE_FMT, CoverageUdpRelay

DURATION_SEC = 20.0


class MockUdpSim(Node):
    """sim_ros_bridge 의 UDP 측 대역 — UDP 로 cmd 받아 unicycle 적분, pose 를
    UDP 로 relay 에 송신. Isaac Sim 없이 닫힌 루프를 닫는다."""

    def __init__(self, relay_host="127.0.0.1", pose_port=5005, cmd_port=5006,
                 rate_hz=60.0):
        super().__init__("mock_udp_sim")
        self.x = self.y = self.yaw = 0.0
        self.lin = self.ang = 0.0
        self.dt = 1.0 / rate_hz
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("0.0.0.0", cmd_port))
        self._rx.setblocking(False)
        self._pose_addr = (relay_host, pose_port)
        self.create_timer(self.dt, self._step)

    def _step(self) -> None:
        while True:
            try:
                data, _ = self._rx.recvfrom(64)
            except BlockingIOError:
                break
            if len(data) == struct.calcsize(CMD_FMT):
                self.lin, self.ang = struct.unpack(CMD_FMT, data)
        self.yaw += self.ang * self.dt
        self.x += self.lin * math.cos(self.yaw) * self.dt
        self.y += self.lin * math.sin(self.yaw) * self.dt
        self._tx.sendto(struct.pack(POSE_FMT, self.x, self.y, self.yaw),
                        self._pose_addr)


def main() -> None:
    rclpy.init()
    coverage = CoverageNode()
    relay = CoverageUdpRelay()
    mock = MockUdpSim()

    executor = SingleThreadedExecutor()
    for n in (coverage, relay, mock):
        executor.add_node(n)

    reveal0 = coverage.fog.overall_ratio()
    print(f"\n[udp-smoketest] 시작 — {DURATION_SEC:.0f}s 동안 UDP↔ROS2 닫힌 루프\n")
    end = time.time() + DURATION_SEC
    try:
        while time.time() < end and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        reveal1 = coverage.fog.overall_ratio()
        moved = math.hypot(mock.x, mock.y)
        print("\n" + "=" * 58)
        print(f"[udp-smoketest] coverage tick : {coverage._tick_index}")
        print(f"[udp-smoketest] relay pose 중계: {relay.pose_count}")
        print(f"[udp-smoketest] 로버 이동      : 원점에서 {moved:.1f}m")
        print(f"[udp-smoketest] reveal         : "
              f"{reveal0*100:.2f}% → {reveal1*100:.2f}%")
        ok = (coverage._tick_index > 0 and relay.pose_count > 0
              and reveal1 > reveal0 and moved > 0.5)
        print(f"[udp-smoketest] 결과           : "
              f"{'PASS — UDP 릴레이 배선 정상' if ok else 'FAIL'}")
        print("=" * 58)
        executor.shutdown()
        for n in (coverage, relay, mock):
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
