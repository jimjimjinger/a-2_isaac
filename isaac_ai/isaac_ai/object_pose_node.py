import rclpy
from rclpy.node import Node


class ObjectPoseNode(Node):
    def __init__(self):
        super().__init__("object_pose_node")
        self.get_logger().info("object_pose_node ready")


def main():
    rclpy.init()
    node = ObjectPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

