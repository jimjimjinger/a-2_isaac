"""Launch perception — yolo_perception_node (nav + wrist YOLO + depth → mineral world XYZ).

mvp.launch.py 와 동일 default params.

실행:
  ros2 launch isaac_bringup perception.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_perception",
                executable="yolo_perception_node",
                name="yolo_perception_node",
                output="screen",
            ),
        ]
    )
