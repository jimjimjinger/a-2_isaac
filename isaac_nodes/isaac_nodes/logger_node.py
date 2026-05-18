import rclpy
from rclpy.node import Node


class LoggerNode(Node):
    def __init__(self):
        super().__init__("logger_node")
        self.get_logger().info("logger_node ready")


def main():
    rclpy.init()
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

