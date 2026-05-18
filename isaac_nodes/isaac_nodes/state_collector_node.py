import rclpy
from rclpy.node import Node


class StateCollectorNode(Node):
    def __init__(self):
        super().__init__("state_collector_node")
        self.get_logger().info("state_collector_node ready")


def main():
    rclpy.init()
    node = StateCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

