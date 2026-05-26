"""다중 rover 카메라 viewer launch — body + wrist 토픽을 rover 별로
rqt_image_view 인스턴스 두 개씩 띄움.

기본 (source=raw) — Isaac Sim 직접 발행:
  - /<rover_ns>/camera/rover/image_raw       (nav/body cam, YOLO bbox 없음)
  - /<rover_ns>/camera/wrist/image_raw       (wrist cam)
  Pro: 안정적 (QoS 항상 매칭, Isaac Sim 살아있으면 무조건 떠 있음)

source=annotated — yolo_perception_node 발행:
  - /<rover_ns>/perception/image_annotated   (nav cam + YOLO bbox)
  - /<rover_ns>/perception/wrist_image_annotated
  Pro: YOLO 검출 결과 시각화

실행:
  ros2 launch isaac_bringup rqt_views_multi.launch.py
    → default = annotated, rover_1/rover_2 (창 4개)
  ros2 launch isaac_bringup rqt_views_multi.launch.py source:=raw
    → raw cam (annotated 가 안 보일 때 fallback)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rover_views(ns: str, source: str):
    nsp = "/" + ns
    if source == "raw":
        body_topic  = f"{nsp}/camera/rover/image_raw"
        wrist_topic = f"{nsp}/camera/wrist/image_raw"
    else:  # annotated
        body_topic  = f"{nsp}/perception/image_annotated"
        wrist_topic = f"{nsp}/perception/wrist_image_annotated"
    # Humble rqt_image_view 는 positional topic 만 받음 (--initial-topics 없음)
    return [
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name=f"rqt_body_view_{ns}",
            arguments=[body_topic],
            output="screen",
        ),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name=f"rqt_wrist_view_{ns}",
            arguments=[wrist_topic],
            output="screen",
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    rovers_str = LaunchConfiguration("rovers").perform(context).strip()
    source = LaunchConfiguration("source").perform(context).strip().lower()
    if source not in ("raw", "annotated"):
        source = "annotated"
    rovers = [r for r in rovers_str.split() if r]
    if not rovers:
        rovers = ["rover_1", "rover_2"]
    nodes = []
    for ns in rovers:
        nodes.extend(_rover_views(ns, source))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "rovers", default_value="rover_1 rover_2",
            description="공백 구분 namespace 리스트"),
        DeclareLaunchArgument(
            "source", default_value="annotated",
            description="'annotated' (YOLO bbox) 또는 'raw' (Isaac 직접 카메라)"),
        OpaqueFunction(function=_launch_setup),
    ])
