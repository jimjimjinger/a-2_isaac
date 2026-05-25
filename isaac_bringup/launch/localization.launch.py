"""Launch localization sensor processing and fusion nodes.

T5 정공법 stack: joint_state_splitter + wheel_odom + imu_integrator + sun_yaw
+ local_height_patch + trn + ekf_fusion + localization_node.
EKF 가 /rover/estimated_odom (Odometry) + /rover/estimated_pose (PoseWithCov)
모두 발행 → arm_executor (Odometry) 와 coverage (PoseWithCov) 둘 다 호환.

**EKF Initial Pose Prior** (real-Mars-rover EDL 패턴, 9b8ebeb): terrain meta.json
의 spawn_locations[0] 을 launch 시점에 읽어 ekf_fusion 의 initial_x/y/z/yaw
파라미터로 주입. mission control 이 위성사진 + DTE uplink 로 EDL 직후 초기
절대 위치 알려주는 것과 동일 (GT cheat 아님 — 시작 1회).
없으면 (0,0,0) 으로 시작 → coverage 의 map frame 과 mismatch → 어제 본 그림.

실행 (terrain_00004 기본):
    ros2 launch isaac_bringup localization.launch.py

다른 terrain:
    ros2 launch isaac_bringup localization.launch.py terrain_id:=terrain_00010
"""

import json
import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_terrain_root() -> str:
    """isaac_sim package source 의 generated_terrains 디렉토리.
    env var ISAAC_TERRAIN_ROOT 가 있으면 우선. 없으면 git checkout 위치 추정."""
    env = os.environ.get("ISAAC_TERRAIN_ROOT")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(
        here, "..", "..", "..", "isaac_sim", "assets", "generated_terrains"))
    if os.path.isdir(candidate):
        return candidate
    return os.path.expanduser(
        "~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains")


def _read_initial_pose(terrain_root: str, terrain_id: str) -> dict:
    """terrain meta.json 의 spawn_locations[0] → ekf prior dict.
    실패 시 (0,0,0) 반환 — EKF 가 default 동작."""
    meta_path = os.path.join(terrain_root, terrain_id, "meta.json")
    try:
        with open(meta_path) as f:
            data = json.load(f)
        spawns = data.get("spawn_locations", [])
        if spawns:
            s = spawns[0]
            return {
                "initial_x": float(s.get("x", 0.0)),
                "initial_y": float(s.get("y", 0.0)),
                "initial_z": float(s.get("z", 0.0)),
                "initial_yaw": float(s.get("yaw", 0.0)),
            }
    except Exception as e:
        print(f"[localization.launch] WARN: cannot read {meta_path}: {e}")
    return {"initial_x": 0.0, "initial_y": 0.0,
            "initial_z": 0.0, "initial_yaw": 0.0}


def _build_nodes(context, *args, **kwargs):
    """OpaqueFunction — launch arg resolve 후 prior 읽어 EKF Node 생성."""
    terrain_id_str = LaunchConfiguration("terrain_id").perform(context)
    terrain_root_str = LaunchConfiguration("terrain_root").perform(context)
    prior = _read_initial_pose(terrain_root_str, terrain_id_str)
    print(f"[localization.launch] EKF prior from "
          f"{terrain_root_str}/{terrain_id_str}/meta.json: "
          f"x={prior['initial_x']:.2f} y={prior['initial_y']:.2f} "
          f"z={prior['initial_z']:.2f} "
          f"yaw={math.degrees(prior['initial_yaw']):.1f}°")

    return [
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
            executable="sun_yaw_node",
            name="sun_yaw_node",
            output="screen",
            parameters=[
                {
                    "world_sun_yaw": -0.4363323129985824,
                    "max_publish_hz": 10.0,
                }
            ],
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
                    "terrain_id": terrain_id_str,
                    "terrain_root": terrain_root_str,
                }
            ],
        ),
        Node(
            package="isaac_localization",
            executable="ekf_fusion_node",
            name="ekf_fusion_node",
            output="screen",
            parameters=[
                {
                    "sun_yaw_topic": "/rover/sun_yaw",
                    "use_sun_yaw": True,
                    "default_sun_yaw_cov": 1.5,
                    "min_sun_yaw_cov": 0.25,
                    "max_sun_yaw_innovation": 1.2,
                    # EDL initial pose prior — terrain spawn 으로 시드.
                    **prior,
                }
            ],
        ),
        Node(
            package="isaac_localization",
            executable="localization_node",
            name="localization_node",
            output="screen",
            parameters=[
                {
                    "terrain_id": terrain_id_str,
                    "terrain_root": terrain_root_str,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "terrain_id",
                default_value="terrain_00004",
                description="Generated terrain directory name used by TRN.",
            ),
            DeclareLaunchArgument(
                "terrain_root",
                default_value=_default_terrain_root(),
                description="Directory containing terrain_<id>/heightmap.npy and meta.json.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
