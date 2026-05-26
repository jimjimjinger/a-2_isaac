"""Launch the driving node (coverage_node — 단일 주행 브레인)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_drive",
                executable="coverage_node",
                name="coverage_node",
                output="screen",
            ),
        ]
    )
