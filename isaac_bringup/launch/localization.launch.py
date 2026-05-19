"""Launch localization node (TRN + EKF multi-sensor fusion)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_localization",
                executable="localization_node",
                name="localization_node",
                output="screen",
            ),
        ]
    )
