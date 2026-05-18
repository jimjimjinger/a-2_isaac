import rclpy
from rclpy.node import Node


class VisionAiNode(Node):
    def __init__(self):
        super().__init__("vision_ai_node")
        self.get_logger().info("vision_ai_node ready")


def main():
    rclpy.init()
    node = VisionAiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

