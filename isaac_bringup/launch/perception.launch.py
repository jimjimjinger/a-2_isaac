"""Launch perception nodes (vision, depth, lidar)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_perception",
                executable="perception_node",
                name="perception_node",
                output="screen",
            ),
        ]
    )
