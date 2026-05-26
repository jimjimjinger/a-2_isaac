"""Full integrated demo using the real localization stack, not GT odom.

This launch replaces the presentation-only GT adapter path in mvp.launch.py:
  /ground_truth/odom -> odom_to_estimated_pose -> /rover/estimated_pose

Instead, it starts localization.launch.py and wires every consumer to:
  /rover/estimated_odom   nav_msgs/Odometry
  /rover/estimated_pose   geometry_msgs/PoseWithCovarianceStamped
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_terrain_root() -> str:
    env = os.environ.get("ISAAC_TERRAIN_ROOT")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(
        os.path.join(here, "..", "..", "..", "isaac_sim", "assets", "generated_terrains")
    )
    if os.path.isdir(candidate):
        return candidate
    return os.path.expanduser(
        "~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains"
    )


def _build_nodes(context, *args, **kwargs):
    terrain_id = LaunchConfiguration("terrain_id").perform(context)
    terrain_root = LaunchConfiguration("terrain_root").perform(context)
    terrain_dir = os.path.join(terrain_root, terrain_id)
    bringup_share = get_package_share_directory("isaac_bringup")
    localization_launch = os.path.join(
        bringup_share, "launch", "localization.launch.py"
    )
    rviz_config = os.path.join(
        bringup_share, "rviz", "localization_map.rviz"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "terrain_id": terrain_id,
                "terrain_root": terrain_root,
            }.items(),
        ),
        Node(
            package="isaac_perception",
            executable="yolo_perception_node",
            name="yolo_perception_node",
            output="screen",
            parameters=[
                {
                    "odom_topic": "/rover/estimated_odom",
                }
            ],
        ),
        Node(
            package="isaac_drive",
            executable="coverage_node",
            name="coverage_node",
            output="screen",
            parameters=[
                {
                    "terrain_dir": terrain_dir,
                    "pose_topic": "/rover/estimated_pose",
                    "cmd_vel_topic": "/coverage/cmd_vel_raw",
                    "max_lin": 1.2,
                    "max_ang": 1.0,
                    "robot_radius": 0.9,
                    "enable_minimap": False,
                    "enable_minimap_topics": True,
                }
            ],
        ),
        Node(
            package="isaac_supervisor",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[
                {
                    "odom_topic": "/rover/estimated_odom",
                    "terrain_dir": terrain_dir,
                    "coverage_cmd_topic": "/coverage/cmd_vel_raw",
                    "cmd_vel_topic": "/cmd_vel",
                    "approach_engage_dist_m": 30.0,
                    "approach_lin_speed": 1.2,
                }
            ],
        ),
        Node(
            package="isaac_manipulation",
            executable="arm_executor_node",
            name="arm_executor_node",
            output="screen",
            parameters=[
                {
                    "odom_topic": "/rover/estimated_odom",
                    "ik_descend_dz": -0.40,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_localization_map",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "terrain_id",
                default_value="terrain_00023",
                description="Generated terrain directory name.",
            ),
            DeclareLaunchArgument(
                "terrain_root",
                default_value=_default_terrain_root(),
                description="Directory containing terrain_<id> assets.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
