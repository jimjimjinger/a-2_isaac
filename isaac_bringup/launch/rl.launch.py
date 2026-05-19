"""Launch RL driving policy node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_rl",
                executable="driving_policy_node",
                name="driving_policy_node",
                output="screen",
            ),
        ]
    )
