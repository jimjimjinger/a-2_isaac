"""Mock driving policy node for the Mars rover ROS2 module skeleton."""

from __future__ import annotations

import rclpy
from isaac_interfaces.msg import MissionState, PerceptionResult, SelectedDriveAction
from rclpy.node import Node


class DrivingPolicyNode(Node):
    """Selects the next driving action from perception and mission state.

    The mock policy publishes a simple target action when the mission is exploring.
    Later this node can load the PPO policy and translate observations into the same
    `SelectedDriveAction` message.
    """

    def __init__(self) -> None:
        super().__init__("driving_policy_node")
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("default_action", "policy_drive")

        self.last_perception: PerceptionResult | None = None
        self.last_mission_state = ""

        self.publisher = self.create_publisher(SelectedDriveAction, "/selected_drive_action", 10)
        self.create_subscription(PerceptionResult, "/perception_result", self._on_perception, 10)
        self.create_subscription(MissionState, "/mission_state", self._on_mission_state, 10)

        period_sec = float(self.get_parameter("publish_period_sec").value)
        self.create_timer(max(period_sec, 0.1), self._publish_action)
        self.get_logger().info("driving_policy_node ready")

    def _on_perception(self, msg: PerceptionResult) -> None:
        self.last_perception = msg

    def _on_mission_state(self, msg: MissionState) -> None:
        self.last_mission_state = msg.state

    def _publish_action(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            return
        if self.last_mission_state and self.last_mission_state != "exploring":
            return

        msg = SelectedDriveAction()
        msg.action = str(self.get_parameter("default_action").value)
        if self.last_perception is not None and self.last_perception.mineral_detected:
            msg.target_x = self.last_perception.x
            msg.target_y = self.last_perception.y
            msg.target_yaw = 0.0
            msg.confidence = self.last_perception.confidence
            msg.reason = f"target from {self.last_perception.object_id}"
        else:
            msg.target_x = 2.0
            msg.target_y = 0.0
            msg.target_yaw = 0.0
            msg.confidence = 0.5
            msg.reason = "default exploration action"
        msg.linear_velocity = 0.5
        msg.angular_velocity = 0.0
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DrivingPolicyNode()
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
