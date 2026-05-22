"""coverage_node ROS2 배선 스모크 테스트 (Isaac Sim 없이).

CoverageNode + MockPoseEcho 를 한 프로세스에서 함께 돌려, ROS2 토픽을 통한
닫힌 루프가 도는지 검증한다:

  CoverageNode  : /rover/estimated_pose 구독 → coverage → /cmd_vel 등 발행
  MockPoseEcho  : /cmd_vel 구독 → unicycle 적분 → /rover/estimated_pose 발행
                  (Isaac Sim + sim_bridge 없이 로버 물리를 흉내내는 mock)

DURATION_SEC 초 구동 후 reveal % 가 올라갔는지 확인.
※ Isaac Sim 물리 검증이 아니라 인터페이스 배선 검증이다.

실행:
    source /opt/ros/humble/setup.bash
    source ~/dev_ws/rover_ws/install/setup.bash
    python3 scripts/coverage_ros_smoketest.py
"""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # isaac_drive 패키지 import 가능하게

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from isaac_drive.coverage_node import CoverageNode

DURATION_SEC = 20.0


class MockPoseEcho(Node):
    """/cmd_vel 을 unicycle 로 적분해 /rover/estimated_pose 로 되돌린다.

    Isaac Sim + sim_bridge 없이 coverage_node 의 닫힌 루프를 닫는 mock.
    coverage_node 가 보낼 명령을 그대로 받아 로버 pose 를 만든다.
    """

    def __init__(self, x0=0.0, y0=0.0, yaw0=0.0, rate_hz=60.0):
        super().__init__("mock_pose_echo")
        self.x, self.y, self.yaw = float(x0), float(y0), float(yaw0)
        self.lin = 0.0
        self.ang = 0.0
        self.dt = 1.0 / rate_hz
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/rover/estimated_pose", 10)
        self.create_timer(self.dt, self._step)

    def _on_cmd(self, msg: Twist) -> None:
        self.lin = msg.linear.x
        self.ang = msg.angular.z

    def _step(self) -> None:
        self.yaw += self.ang * self.dt
        self.x += self.lin * math.cos(self.yaw) * self.dt
        self.y += self.lin * math.sin(self.yaw) * self.dt
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self._pub.publish(m)


def main() -> None:
    rclpy.init()
    coverage = CoverageNode()
    mock = MockPoseEcho()

    executor = SingleThreadedExecutor()
    executor.add_node(coverage)
    executor.add_node(mock)

    reveal_start = coverage.fog.overall_ratio()
    print(f"\n[smoketest] 시작 — {DURATION_SEC:.0f}s 동안 ROS2 닫힌 루프 구동\n")

    end = time.time() + DURATION_SEC
    try:
        while time.time() < end and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        reveal_end = coverage.fog.overall_ratio()
        moved = math.hypot(mock.x, mock.y)
        ratios = coverage.fog.all_sector_ratios()
        print("\n" + "=" * 56)
        print(f"[smoketest] tick 수   : {coverage._tick_index}")
        print(f"[smoketest] 로버 이동 : ({mock.x:+.2f}, {mock.y:+.2f}) "
              f"= 원점에서 {moved:.1f}m")
        print(f"[smoketest] reveal    : {reveal_start*100:.2f}% "
              f"→ {reveal_end*100:.2f}%")
        print("[smoketest] 구역별    : "
              + " ".join(f"S{i+1}={r*100:.0f}%" for i, r in enumerate(ratios)))
        ok = (coverage._tick_index > 0 and reveal_end > reveal_start
              and moved > 0.5)
        print(f"[smoketest] 결과      : "
              f"{'PASS — ROS2 배선 정상' if ok else 'FAIL'}")
        print("=" * 56)
        executor.shutdown()
        coverage.destroy_node()
        mock.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
