import rclpy
from rclpy.node import Node


class RobotExecutorNode(Node):
    def __init__(self):
        super().__init__("robot_executor_node")
        self.get_logger().info("robot_executor_node ready")


def main():
    rclpy.init()
    node = RobotExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

