"""Launch manipulation nodes (M0609 arm executor)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_manipulation",
                executable="arm_executor_node",
                name="arm_executor_node",
                output="screen",
            ),
        ]
    )
