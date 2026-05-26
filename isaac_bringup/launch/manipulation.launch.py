"""Launch manipulation — arm_executor_node (M0609 DLS-IK + /grasp/command).

mvp.launch.py 와 동일 — ik_descend_dz=-0.40 (perception z bias 보정).
enable_ik=True default. odom_topic default=/ground_truth/odom (GT cheat).

실행:
  ros2 launch isaac_bringup manipulation.launch.py

T5 정공법 모드:
  ros2 launch isaac_bringup manipulation.launch.py \\
    arm_executor_node:odom_topic:=/rover/estimated_odom
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_manipulation",
                executable="arm_executor_node",
                name="arm_executor_node",
                output="screen",
                parameters=[{"ik_descend_dz": -0.40}],
            ),
        ]
    )
