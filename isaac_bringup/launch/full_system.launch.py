"""Launch the full Mars rover resource collection ROS2 system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package: str, launch_file: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        )
    )


def generate_launch_description() -> LaunchDescription:
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start", default_value="false"),
            _include("isaac_bringup", "sim.launch.py"),
            _include("isaac_bringup", "localization.launch.py"),
            _include("isaac_bringup", "perception.launch.py"),
            _include("isaac_bringup", "rl.launch.py"),
            _include("isaac_bringup", "drive.launch.py"),
            _include("isaac_bringup", "manipulation.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("isaac_bringup"), "launch", "supervisor.launch.py"])
                ),
                launch_arguments={"auto_start": auto_start}.items(),
            ),
        ]
    )
