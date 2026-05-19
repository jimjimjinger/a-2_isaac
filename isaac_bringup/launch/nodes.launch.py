"""Launch mission, battery, and arm nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="false"),
            Node(
                package="isaac_nodes",
                executable="battery_monitor_node",
                name="battery_monitor_node",
                output="screen",
            ),
            Node(
                package="isaac_nodes",
                executable="arm_executor_node",
                name="arm_executor_node",
                output="screen",
            ),
            Node(
                package="isaac_nodes",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[{"auto_start": auto_start}],
            ),
        ]
    )
