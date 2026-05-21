"""UDP ↔ ROS2 릴레이 — sim_ros_bridge(Isaac Sim)와 coverage_node 를 잇는다.

Isaac Sim 프로세스는 rclpy 를 못 쓴다(Python 3.11 vs ROS2 Humble 3.10 — C 확장
ABI 불일치). 그래서 sim_ros_bridge 는 UDP 로만 통신하고, 이 릴레이가 그 UDP 를
ROS2 토픽으로 변환한다. 이 노드는 일반 ROS2 프로세스(시스템 Python 3.10)라
rclpy 가 정상 동작한다.

  · UDP 로 받은 로버 pose  →  /rover/estimated_pose (PoseWithCovarianceStamped)
  · /cmd_vel (Twist) 구독  →  UDP 로 sim_ros_bridge 에 전달

실행 (시스템 ROS2 source 후):
    python3 scripts/coverage_udp_relay.py
"""
import math
import socket
import struct

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

POSE_FMT = "<fff"   # x, y, yaw  (sim → relay)
CMD_FMT = "<ff"     # lin, ang   (relay → sim)


class CoverageUdpRelay(Node):
    """sim_ros_bridge 의 UDP 와 coverage_node 의 ROS2 토픽을 양방향 중계."""

    def __init__(self) -> None:
        super().__init__("coverage_udp_relay")
        self.declare_parameter("pose_port", 5005)        # sim → relay
        self.declare_parameter("cmd_port", 5006)         # relay → sim
        self.declare_parameter("sim_host", "127.0.0.1")
        self.declare_parameter("publish_hz", 60.0)

        pose_port = int(self.get_parameter("pose_port").value)
        self._cmd_addr = (str(self.get_parameter("sim_host").value),
                          int(self.get_parameter("cmd_port").value))

        # UDP — pose 수신(non-blocking), cmd 송신
        self._pose_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pose_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._pose_rx.bind(("0.0.0.0", pose_port))
        self._pose_rx.setblocking(False)
        self._cmd_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._latest_pose: tuple[float, float, float] | None = None
        self._pose_count = 0

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/rover/estimated_pose", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(
            1.0 / max(float(self.get_parameter("publish_hz").value), 1.0),
            self._tick)
        self.get_logger().info(
            f"coverage_udp_relay ready — pose :{pose_port} ← sim, "
            f"cmd → {self._cmd_addr}")

    def _on_cmd(self, msg: Twist) -> None:
        """/cmd_vel → UDP → sim_ros_bridge."""
        self._cmd_tx.sendto(
            struct.pack(CMD_FMT, msg.linear.x, msg.angular.z), self._cmd_addr)

    def _tick(self) -> None:
        """UDP 로 받은 pose 중 최신만 /rover/estimated_pose 로 발행."""
        got = False
        while True:
            try:
                data, _ = self._pose_rx.recvfrom(64)
            except BlockingIOError:
                break
            if len(data) == struct.calcsize(POSE_FMT):
                self._latest_pose = struct.unpack(POSE_FMT, data)
                self._pose_count += 1
                got = True
        if got and self._latest_pose is not None:
            self._publish_pose(*self._latest_pose)

    def _publish_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)

    @property
    def pose_count(self) -> int:
        return self._pose_count


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageUdpRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
