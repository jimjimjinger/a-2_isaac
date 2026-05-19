"""Launch rover navigation nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_navigation",
                executable="navigation_manager_node",
                name="navigation_manager_node",
                output="screen",
            ),
            Node(
                package="isaac_navigation",
                executable="mobile_base_executor_node",
                name="mobile_base_executor_node",
                output="screen",
            ),
        ]
    )
