"""Launch localization sensor processing and fusion nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    terrain_id = LaunchConfiguration("terrain_id")
    terrain_root = LaunchConfiguration("terrain_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "terrain_id",
                default_value="terrain_00001",
                description="Generated terrain directory name used by TRN.",
            ),
            DeclareLaunchArgument(
                "terrain_root",
                default_value=(
                    "/home/rokey/dev_ws/rover_ws/src/a2_isaac/"
                    "isaac_sim/assets/generated_terrains"
                ),
                description="Directory containing terrain_<id>/heightmap.npy and meta.json.",
            ),
            Node(
                package="isaac_localization",
                executable="joint_state_splitter_node",
                name="joint_state_splitter",
                output="screen",
            ),
            Node(
                package="isaac_localization",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
            ),
            Node(
                package="isaac_localization",
                executable="imu_integrator_node",
                name="imu_integrator_node",
                output="screen",
            ),
            Node(
                package="isaac_localization",
                executable="local_height_patch_node",
                name="local_height_patch_node",
                output="screen",
            ),
            Node(
                package="isaac_localization",
                executable="trn_node",
                name="trn_node",
                output="screen",
                parameters=[
                    {
                        "terrain_id": terrain_id,
                        "terrain_root": terrain_root,
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="ekf_fusion_node",
                name="ekf_fusion_node",
                output="screen",
            ),
            Node(
                package="isaac_localization",
                executable="localization_node",
                name="localization_node",
                output="screen",
                parameters=[
                    {
                        "terrain_id": terrain_id,
                        "terrain_root": terrain_root,
                    }
                ],
            ),
        ]
    )
