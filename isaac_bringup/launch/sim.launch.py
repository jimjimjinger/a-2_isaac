"""Launch Isaac Sim bridge nodes.

This launch file currently starts the ROS2 service bridge. The full Isaac Sim
application can be attached here once the simulation runner is migrated.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_sim",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
            ),
        ]
    )
