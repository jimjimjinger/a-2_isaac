"""Launch drive nodes (drive manager + mobile base executor)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_drive",
                executable="drive_manager_node",
                name="drive_manager_node",
                output="screen",
            ),
            Node(
                package="isaac_drive",
                executable="mobile_base_executor_node",
                name="mobile_base_executor_node",
                output="screen",
            ),
        ]
    )
