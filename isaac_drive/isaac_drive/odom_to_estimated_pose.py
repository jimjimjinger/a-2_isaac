"""odom_to_estimated_pose — Isaac Sim 의 GT odom 을 I5 인터페이스로 변환하는 어댑터.

vehicle_v3 는 내장 그래프로 nav_msgs/Odometry 를 /ground_truth/odom 으로
발행한다 (isaac_ros_topics.md). coverage_node 와 I5 인터페이스 계약은
geometry_msgs/PoseWithCovarianceStamped (/rover/estimated_pose)를 쓴다.
이 노드가 그 사이를 메운다 — Odometry.pose 는 PoseWithCovariance 로
PoseWithCovarianceStamped.pose 와 동일 타입이라 그대로 옮긴다.

실제 시스템에서는 T5 localization 이 /rover/estimated_pose 를 직접
발행하므로, 이 노드는 그 자리의 시뮬레이션용 placeholder 다.

default odom_topic 은 vehicle_v3 표준 (/ground_truth/odom). 옛 vehicle_v1/
sim_ros2_bridge 가 /odom 으로 발행하던 시절의 호환은 parameter override
(`-p odom_topic:=/odom`) 로 가능.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToEstimatedPose(Node):
    """/odom (Odometry) → /rover/estimated_pose (PoseWithCovarianceStamped)."""

    def __init__(self) -> None:
        super().__init__("odom_to_estimated_pose")
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        self.declare_parameter("pose_topic", "/rover/estimated_pose")
        odom_topic = str(self.get_parameter("odom_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.get_logger().info(
            f"odom_to_estimated_pose ready — {odom_topic} → {pose_topic}")

    def _on_odom(self, msg: Odometry) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose   # Odometry.pose 와 PoseWithCovarianceStamped.pose 동일 타입
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomToEstimatedPose()
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
