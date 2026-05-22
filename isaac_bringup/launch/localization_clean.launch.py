"""Launch localization through isolated topics to avoid stale ROS graph mixing."""

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
                name="joint_state_splitter_clean",
                output="screen",
                parameters=[
                    {
                        "wheel_topic": "/rover/wheel_states_clean",
                        "arm_topic": "/joint_states_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="wheel_odom_node",
                name="wheel_odom_clean",
                output="screen",
                parameters=[
                    {
                        "joint_state_topic": "/rover/wheel_states_clean",
                        "odom_topic": "/rover/wheel_odom_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="imu_integrator_node",
                name="imu_integrator_clean",
                output="screen",
                parameters=[
                    {
                        "imu_odom_topic": "/rover/imu_odom_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="local_height_patch_node",
                name="local_height_patch_clean",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/rover/wheel_odom_clean",
                        "patch_topic": "/rover/local_height_patch_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="trn_node",
                name="trn_clean",
                output="screen",
                parameters=[
                    {
                        "terrain_id": terrain_id,
                        "terrain_root": terrain_root,
                        "prior_odom_topic": "/rover/wheel_odom_clean",
                        "local_patch_topic": "/rover/local_height_patch_clean",
                        "trn_pose_topic": "/rover/trn_pose_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="ekf_fusion_node",
                name="ekf_fusion_clean",
                output="screen",
                parameters=[
                    {
                        "wheel_odom_topic": "/rover/wheel_odom_clean",
                        "imu_odom_topic": "/rover/imu_odom_clean",
                        "trn_pose_topic": "/rover/trn_pose_clean",
                        "estimated_odom_topic": "/rover/estimated_odom_clean",
                        "estimated_pose_topic": "/rover/estimated_pose_clean",
                    }
                ],
            ),
            Node(
                package="isaac_localization",
                executable="localization_node",
                name="localization_node_clean",
                output="screen",
                parameters=[
                    {
                        "terrain_id": terrain_id,
                        "terrain_root": terrain_root,
                        "estimated_odom_topic": "/rover/estimated_odom_clean",
                        "estimated_marker_topic": "/rover/estimated_marker_clean",
                    }
                ],
            ),
        ]
    )
