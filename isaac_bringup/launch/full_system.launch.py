"""Launch full system — mvp + localization (T5 정공법) + rqt_views.

mvp.launch.py 가 시연용 core 5 노드 (perception + coverage + supervisor + arm
+ odom_to_estimated_pose GT cheat) 제공. 여기에 T5 EKF stack + 카메라 view 를
include 옵션으로 추가.

⚠️ Isaac Sim 자체는 launch 못 띄움 — 별도 터미널에서:
  tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

실행 (full system, GT cheat 모드):
  ros2 launch isaac_bringup full_system.launch.py

T5 EKF 정공법으로 (mvp 의 odom_to_estimated_pose 와 충돌하니 둘 중 하나만):
  ros2 launch isaac_bringup full_system.launch.py use_localization:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package: str, launch_file: str,
             condition=None) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    use_localization = LaunchConfiguration("use_localization")
    use_rqt_views = LaunchConfiguration("use_rqt_views")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_localization",
                default_value="false",
                description="True=T5 EKF stack 사용 (mvp 의 GT cheat 어댑터와 충돌 — 한쪽만)",
            ),
            DeclareLaunchArgument(
                "use_rqt_views",
                default_value="false",
                description="True=body+wrist 카메라 rqt_image_view 2개 띄움",
            ),
            _include("isaac_bringup", "mvp.launch.py"),
            _include(
                "isaac_bringup", "localization.launch.py",
                condition=IfCondition(use_localization),
            ),
            _include(
                "isaac_bringup", "rqt_views.launch.py",
                condition=IfCondition(use_rqt_views),
            ),
        ]
    )
