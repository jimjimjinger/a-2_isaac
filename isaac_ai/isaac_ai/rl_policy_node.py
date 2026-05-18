import rclpy
from rclpy.node import Node


class RlPolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy_node")
        self.get_logger().info("rl_policy_node ready")


def main():
    rclpy.init()
    node = RlPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

