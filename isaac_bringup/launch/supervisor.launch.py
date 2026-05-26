"""Launch supervisor — mission_manager_node (시연용 핵심) + battery_monitor (보조).

mvp.launch.py 와 동일 params: approach_engage_dist_m=30, approach_lin_speed=1.2.
odom_topic default=/ground_truth/odom (GT cheat 모드).

실행:
  ros2 launch isaac_bringup supervisor.launch.py

T5 정공법 모드로 odom 바꿀 때:
  ros2 launch isaac_bringup supervisor.launch.py \\
    mission_manager_node:odom_topic:=/rover/estimated_odom
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_supervisor",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "approach_engage_dist_m": 30.0,
                        "approach_lin_speed": 1.2,
                    }
                ],
            ),
            Node(
                package="isaac_supervisor",
                executable="battery_monitor_node",
                name="battery_monitor_node",
                output="screen",
            ),
        ]
    )
