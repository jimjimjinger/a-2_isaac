"""Camera viewer launch — body + wrist cam annotated 토픽 두 개를 rqt_image_view
인스턴스로 동시 표시.

mvp.launch.py 의 yolo_perception_node 가 발행하는:
  - /perception/image_annotated         (nav/body cam + YOLO bbox)
  - /perception/wrist_image_annotated   (wrist cam + YOLO bbox)

시연 시 두 카메라 view 를 직접 보고 perception 결과 모니터.

실행:
  source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
  ros2 launch isaac_bringup rqt_views.launch.py

각 rqt_image_view 인스턴스가 별도 GUI 창으로 뜸. 첫 인자로 topic 자동 select
(rqt_image_view 의 standalone widget mode).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_body_view",
                arguments=["/perception/image_annotated"],
                output="screen",
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_wrist_view",
                arguments=["/perception/wrist_image_annotated"],
                output="screen",
            ),
        ]
    )
