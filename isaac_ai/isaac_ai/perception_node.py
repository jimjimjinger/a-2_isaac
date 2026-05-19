"""Mock perception node for the Mars rover ROS2 module skeleton."""

from __future__ import annotations

import rclpy
from isaac_interfaces.msg import PerceptionResult
from rclpy.node import Node


class PerceptionNode(Node):
    """Publishes perception results from camera/depth inputs.

    This first implementation is intentionally a mock source. The Isaac Sim camera,
    depth, and detector code can replace `_publish_mock_detection` without changing
    downstream mission/navigation contracts.
    """

    def __init__(self) -> None:
        super().__init__("perception_node")
        self.declare_parameter("mock_detection_enabled", True)
        self.declare_parameter("mock_period_sec", 8.0)
        self.declare_parameter("mock_object_id", "mineral_001")
        self.declare_parameter("mock_object_type", "mineral")
        self.declare_parameter("mock_x", 4.0)
        self.declare_parameter("mock_y", 2.0)
        self.declare_parameter("mock_z", 0.0)
        self.declare_parameter("mock_confidence", 0.9)

        self.publisher = self.create_publisher(PerceptionResult, "/perception_result", 10)
        period_sec = float(self.get_parameter("mock_period_sec").value)
        self.create_timer(max(period_sec, 0.1), self._publish_mock_detection)
        self.get_logger().info("perception_node ready")

    def _publish_mock_detection(self) -> None:
        if not bool(self.get_parameter("mock_detection_enabled").value):
            return

        msg = PerceptionResult()
        msg.mineral_detected = True
        msg.object_id = str(self.get_parameter("mock_object_id").value)
        msg.object_type = str(self.get_parameter("mock_object_type").value)
        msg.confidence = float(self.get_parameter("mock_confidence").value)
        msg.x = float(self.get_parameter("mock_x").value)
        msg.y = float(self.get_parameter("mock_y").value)
        msg.z = float(self.get_parameter("mock_z").value)
        msg.obstacle_detected = False
        msg.obstacle_distance = 0.0
        msg.terrain_traversable = True
        msg.terrain_slope = 0.0
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
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
