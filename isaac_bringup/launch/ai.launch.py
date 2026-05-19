"""Launch AI perception and driving policy nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_ai",
                executable="perception_node",
                name="perception_node",
                output="screen",
            ),
            Node(
                package="isaac_ai",
                executable="driving_policy_node",
                name="driving_policy_node",
                output="screen",
            ),
        ]
    )
