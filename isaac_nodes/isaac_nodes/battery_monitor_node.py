"""Battery monitor publisher for the Mars rover module skeleton."""

from __future__ import annotations

import rclpy
from isaac_interfaces.msg import BatteryState
from rclpy.node import Node


class BatteryMonitorNode(Node):
    """Publishes rover battery state.

    The mock implementation drains slowly while not charging. Later this node can
    subscribe to Isaac Sim telemetry or a real BMS bridge.
    """

    def __init__(self) -> None:
        super().__init__("battery_monitor_node")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("initial_percentage", 100.0)
        self.declare_parameter("drain_per_tick", 0.0)
        self.declare_parameter("charging", False)
        self.declare_parameter("low_threshold", 25.0)
        self.declare_parameter("critical_threshold", 10.0)

        self.percentage = float(self.get_parameter("initial_percentage").value)
        self.publisher = self.create_publisher(BatteryState, "/battery_state", 10)
        publish_hz = max(float(self.get_parameter("publish_hz").value), 0.1)
        self.create_timer(1.0 / publish_hz, self._publish_battery)
        self.get_logger().info("battery_monitor_node ready")

    def _publish_battery(self) -> None:
        charging = bool(self.get_parameter("charging").value)
        drain = float(self.get_parameter("drain_per_tick").value)
        if not charging:
            self.percentage = max(0.0, self.percentage - max(drain, 0.0))

        low_threshold = float(self.get_parameter("low_threshold").value)
        critical_threshold = float(self.get_parameter("critical_threshold").value)

        msg = BatteryState()
        msg.percentage = float(self.percentage)
        msg.is_charging = charging
        msg.is_low = self.percentage <= low_threshold
        msg.is_critical = self.percentage <= critical_threshold
        msg.voltage = 24.0 * (self.percentage / 100.0)
        msg.current = -1.0 if not charging else 2.0
        msg.source = "mock_battery_monitor"
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BatteryMonitorNode()
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
