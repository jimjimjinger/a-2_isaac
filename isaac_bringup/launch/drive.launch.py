"""Launch driving — coverage_node (BCD anchor sweep + A* obstacle 회피).

mvp.launch.py 와 동일 — cmd_vel→/coverage/cmd_vel_raw remap 으로 supervisor 가
EXPLORE/APPROACH phase 별 mux 처리. odom_to_estimated_pose 별도 띄워야
/rover/estimated_pose 받음 (mvp.launch.py 가 함께 처리).

실행 (supervisor 없이 단독 검증):
  ros2 run isaac_drive odom_to_estimated_pose &
  ros2 launch isaac_bringup drive.launch.py
  # rover 가 직접 /coverage/cmd_vel_raw 로 발행 — 다른 노드가 mux 안 함

권장: 시연용은 mvp.launch.py 사용 (supervisor + odom_adapter 포함).
"""

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
                remappings=[("/cmd_vel", "/coverage/cmd_vel_raw")],
            ),
        ]
    )
