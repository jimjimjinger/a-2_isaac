"""Navigation manager placeholder node."""

from __future__ import annotations

import rclpy
from isaac_interfaces.msg import MissionState, PerceptionResult, SelectedDriveAction
from rclpy.node import Node


class NavigationManagerNode(Node):
    """Observes mission/perception/policy topics for navigation orchestration.

    The current drive action server lives in `mobile_base_executor_node`. This node
    is kept as the future place for obstacle avoidance, manual/autonomous mode
    switching, and path planning.
    """

    def __init__(self) -> None:
        super().__init__("navigation_manager_node")
        self.create_subscription(MissionState, "/mission_state", self._on_mission_state, 10)
        self.create_subscription(PerceptionResult, "/perception_result", self._on_perception, 10)
        self.create_subscription(SelectedDriveAction, "/selected_drive_action", self._on_drive_action, 10)
        self.get_logger().info("navigation_manager_node ready")

    def _on_mission_state(self, msg: MissionState) -> None:
        self.get_logger().debug(f"mission state: {msg.state}")

    def _on_perception(self, msg: PerceptionResult) -> None:
        self.get_logger().debug(f"perception mineral_detected={msg.mineral_detected}")

    def _on_drive_action(self, msg: SelectedDriveAction) -> None:
        self.get_logger().debug(f"selected drive action: {msg.action}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationManagerNode()
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
