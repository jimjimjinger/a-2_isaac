import rclpy
from rclpy.node import Node


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")
        self.get_logger().info("task_manager_node ready")


def main():
    rclpy.init()
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

