"""Launch supervisor nodes (battery monitor)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_supervisor",
                executable="battery_monitor_node",
                name="battery_monitor_node",
                output="screen",
            ),
        ]
    )
