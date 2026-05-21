"""Launch the full Mars rover resource collection ROS2 system."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package: str, launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        )
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            _include("isaac_bringup", "sim.launch.py"),
            _include("isaac_bringup", "localization.launch.py"),
            _include("isaac_bringup", "perception.launch.py"),
            _include("isaac_bringup", "drive.launch.py"),
            _include("isaac_bringup", "manipulation.launch.py"),
            _include("isaac_bringup", "supervisor.launch.py"),
        ]
    )
