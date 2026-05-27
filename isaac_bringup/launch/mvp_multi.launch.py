"""다중 rover 시연용 통합 런치 (Phase 3 of multi-rover MVP).

run_vehicle_v3.py --rovers rover_1 rover_2 ... 와 짝지어 사용. 각 rover 마다
별도 namespace 의 5 노드 묶음을 띄움. 토픽은 /<namespace>/... prefix.

기존 mvp.launch.py 는 변경 없이 보존 (단일 rover 시연용).

사용법 (T1 Isaac Sim 띄운 후):
  ros2 launch isaac_bringup mvp_multi.launch.py
    → default 로 rover_1, rover_2 두 대 노드 묶음 띄움
  ros2 launch isaac_bringup mvp_multi.launch.py rovers:='rover_1 rover_2 rover_3'
    → 임의 N대

각 rover 의 토픽 격리:
  /rover_1/ground_truth/odom, /rover_1/cmd_vel, /rover_1/arm/joint_command, ...
  /rover_2/ground_truth/odom, /rover_2/cmd_vel, /rover_2/arm/joint_command, ...

mineral claim 공유 토픽 (모든 rover 공통, namespace 없음):
  /mineral_claims  (Phase 4 에서 추가)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _rover_group(ns: str, terrain_dir: str, collection_goal: int,
                 arrive_r: float, fringe_r: float, reveal_radius: float):
    """단일 rover 의 5 노드 묶음 — 토픽 모두 /<ns>/... prefix.

    terrain_dir 는 T1 Isaac Sim 의 --terrain 인자와 동일한 terrain 폴더 경로.
    coverage/mission_manager 가 같은 obstacle_grid 를 사용해야 anchor 도달 가능.
    collection_goal 은 각 rover 가 N개 채집 시 RETURN_TO_BASE 전환.
    """
    nsp = "/" + ns
    return [
        # T5 cheat 어댑터 — /<ns>/ground_truth/odom → /<ns>/rover/estimated_pose
        Node(
            package="isaac_drive",
            executable="odom_to_estimated_pose",
            name="odom_to_estimated_pose",
            namespace=ns,
            output="screen",
            parameters=[{
                "odom_topic": f"{nsp}/ground_truth/odom",
                "pose_topic": f"{nsp}/rover/estimated_pose",
            }],
        ),
        # T2 perception — nav + wrist YOLO + depth backproject
        Node(
            package="isaac_perception",
            executable="yolo_perception_node",
            name="yolo_perception_node",
            namespace=ns,
            output="screen",
            parameters=[{
                "nav_rgb_topic":     f"{nsp}/camera/rover/image_raw",
                "nav_depth_topic":   f"{nsp}/camera/rover/depth",
                "nav_info_topic":    f"{nsp}/camera/rover/camera_info",
                "odom_topic":        f"{nsp}/ground_truth/odom",
                "nav_det_topic":     f"{nsp}/perception/detections",
                "nav_ann_topic":     f"{nsp}/perception/image_annotated",
                "wrist_rgb_topic":   f"{nsp}/camera/wrist/image_raw",
                "wrist_depth_topic": f"{nsp}/camera/wrist/depth",
                "wrist_info_topic":  f"{nsp}/camera/wrist/camera_info",
                "wrist_det_topic":   f"{nsp}/perception/wrist_detections",
                "wrist_ann_topic":   f"{nsp}/perception/wrist_image_annotated",
            }],
        ),
        # T3 coverage — BCD anchor sweep + A* obstacle 회피
        # /cmd_vel → /coverage/cmd_vel_raw remap: supervisor 가 mux.
        # minimap_publisher 와 mission_state 가 absolute "/mission/..." 로
        # 박혀 있어 두 rover 가 충돌 → remap 으로 namespace 강제 격리.
        # matplotlib viewer: 두 rover 모두 활성. minimap_rover_id 가 StateWriter
        # 에 전달되어 각자 unique npz (예: starcraft_map_state_rover_1.npz)
        # 와 별도 viewer subprocess 가 떠 두 matplotlib 창이 독립 동작.
        Node(
            package="isaac_drive",
            executable="coverage_node",
            name="coverage_node",
            namespace=ns,
            output="screen",
            parameters=[{
                "pose_topic":    f"{nsp}/rover/estimated_pose",
                "cmd_vel_topic": f"{nsp}/coverage/cmd_vel_raw",
                "terrain_dir": terrain_dir,
                # matplotlib Starcraft Map viewer 비활성 — Web HUD 의
                # COVERAGE MAP canvas 가 두 rover 추적 가능해진 시점부터
                # 더이상 필요 없음. enable_minimap_topics 만 True 로
                # /<ns>/mission/{minimap,path,markers} 발행 유지 → Web HUD
                # 가 그 토픽 받아 그림.
                "enable_minimap": False,
                "enable_minimap_topics": True,
                "minimap_rover_id": ns,
                "reveal_radius": reveal_radius,
            }],
            remappings=[
                # minimap_publisher 의 absolute 토픽 → namespace 강제
                ("/mission/minimap",  f"{nsp}/mission/minimap"),
                ("/mission/path",     f"{nsp}/mission/path"),
                ("/mission/markers",  f"{nsp}/mission/markers"),
                # coverage_node 의 MissionState publisher (absolute /mission_state)
                ("/mission_state",    f"{nsp}/mission_state"),
                # mission_manager 의 replan publish 를 같은 ns 의 coverage 가 받게
                ("/coverage/replan_request", f"{nsp}/coverage/replan_request"),
            ],
        ),
        # Supervisor — EXPLORE→APPROACH→PICK phase + cmd_vel mux + arm action
        Node(
            package="isaac_supervisor",
            executable="mission_manager_node",
            name="mission_manager_node",
            namespace=ns,
            output="screen",
            parameters=[{
                "approach_engage_dist_m": 30.0,
                "approach_lin_speed": 1.2,
                "coverage_cmd_topic": f"{nsp}/coverage/cmd_vel_raw",
                "cmd_vel_topic":      f"{nsp}/cmd_vel",
                "detections_topic":   f"{nsp}/perception/detections",
                "odom_topic":         f"{nsp}/ground_truth/odom",
                "phase_topic":        f"{nsp}/mission/phase",
                # 상대 토픽 — namespace=ns 가 자동 prefix → /<ns>/execute_arm_task
                "arm_action_name":    "execute_arm_task",
                "collection_goal":    collection_goal,
                # ── Multi-rover 협조 활성 (단일 시연은 default False) ──
                "enable_mineral_claim":  True,
                "enable_rover_avoid":    True,
                "claim_rover_id":        ns,
                "basecamp_arrive_radius_m": arrive_r,
                "basecamp_fringe_radius_m": fringe_r,
                "state_topic":        f"{nsp}/mission/state",
                "mode_topic":         f"{nsp}/mission/mode",
                "estop_topic":        f"{nsp}/mission/estop",
                "terrain_dir":        terrain_dir,
                # 협조 param (enable_mineral_claim/enable_rover_avoid 등) 은
                # main 의 mission_manager_node 에 declare 안 됨 — 단순 namespace
                # 복사 단계 (옵션 X) 라 미설정. 협조 단계 (옵션 Y) 진입 시
                # mission_manager 코드 + param declare 추가 후 활성.
            }],
            remappings=[
                # mission_manager_node 의 absolute publisher → namespace 강제
                ("/coverage/replan_request", f"{nsp}/coverage/replan_request"),
                ("/supervisor/path",         f"{nsp}/supervisor/path"),
                ("/supervisor/target",       f"{nsp}/supervisor/target"),
            ],
        ),
        # T2 manipulation — DLS-IK 상태머신 + /grasp/command FixedJoint snap
        Node(
            package="isaac_manipulation",
            executable="arm_executor_node",
            name="arm_executor_node",
            namespace=ns,
            output="screen",
            parameters=[{
                "ik_descend_dz": -0.40,
                "joint_command_topic":    f"{nsp}/arm/joint_command",
                "joint_state_topic":      f"{nsp}/joint_states_raw",
                "wrist_detections_topic": f"{nsp}/perception/wrist_detections",
                "odom_topic":             f"{nsp}/ground_truth/odom",
                "grasp_command_topic":    f"{nsp}/grasp/command",
            }],
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    rovers_str = LaunchConfiguration("rovers").perform(context).strip()
    rovers = [r for r in rovers_str.split() if r]
    if not rovers:
        rovers = ["rover_1", "rover_2"]
    terrain_id = LaunchConfiguration("terrain_id").perform(context)
    terrain_root = LaunchConfiguration("terrain_root").perform(context)
    terrain_dir = os.path.join(terrain_root, terrain_id)
    collection_goal = int(LaunchConfiguration("collection_goal").perform(context))
    arrive_r = float(LaunchConfiguration("basecamp_arrive_radius_m").perform(context))
    fringe_r = float(LaunchConfiguration("basecamp_fringe_radius_m").perform(context))
    enable_dashboard = LaunchConfiguration("enable_dashboard").perform(context)
    enable_web_video = LaunchConfiguration("enable_web_video").perform(context)
    web_video_port = LaunchConfiguration("web_video_port").perform(context)
    reveal_radius = float(LaunchConfiguration("reveal_radius").perform(context))
    print(f"[mvp_multi] terrain_dir={terrain_dir}  rovers={rovers}  "
          f"collection_goal={collection_goal} (per rover)")
    nodes = []
    for ns in rovers:
        nodes.extend(_rover_group(ns, terrain_dir, collection_goal,
                                  arrive_r, fringe_r, reveal_radius))

    # Web HUD — 단일 인스턴스가 모든 rover namespace 추적 (옵션 X 단계 2).
    # mission_web_node 의 rover_namespaces param 으로 ns 리스트 전달.
    if str(enable_dashboard).lower() in ("true", "1", "yes"):
        nodes.append(Node(
            package="isaac_supervisor",
            executable="mission_web_node",
            name="mission_web_node",
            output="screen",
            additional_env={
                "WEB_VIDEO_PORT": str(web_video_port),
            },
            parameters=[{
                "rover_namespaces": rovers,
                "terrain_preview_path": os.path.join(terrain_dir, "preview.png"),
            }],
        ))
    if str(enable_web_video).lower() in ("true", "1", "yes"):
        nodes.append(Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            parameters=[{
                "port": int(web_video_port),
                "address": "0.0.0.0",
            }],
        ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "rovers", default_value="rover_1 rover_2",
            description="공백 구분 namespace 리스트. 예: 'rover_1 rover_2 rover_3'"),
        DeclareLaunchArgument(
            "terrain_id", default_value="terrain_00023",
            description="T1 Isaac Sim 의 --terrain 인자와 동일해야 obstacle_grid 정합."),
        DeclareLaunchArgument(
            "terrain_root", default_value=_default_terrain_root(),
            description="terrain_<id> 디렉토리들이 모여있는 루트."),
        DeclareLaunchArgument(
            "collection_goal", default_value="5",
            description="각 rover 가 N개 채집 시 RETURN_TO_BASE. 빠른 시연 1."),
        DeclareLaunchArgument(
            "basecamp_arrive_radius_m", default_value="8.0",
            description="베이스캠프 중심에서 이 거리 안에 들어오면 MISSION_COMPLETE."),
        DeclareLaunchArgument(
            "basecamp_fringe_radius_m", default_value="7.5",
            description="RTB 시 A* target = basecamp 중심에서 이 거리 fringe."),
        DeclareLaunchArgument(
            "enable_dashboard", default_value="true",
            description="mission_web_node (멀티 rover 추적 Web HUD) 활성."),
        DeclareLaunchArgument(
            "enable_web_video", default_value="true",
            description="web_video_server MJPEG 스트리머 활성."),
        DeclareLaunchArgument(
            "web_video_port", default_value="8090",
            description="web_video_server HTTP 포트. Web HUD 이미지 src 가 이 포트."),
        DeclareLaunchArgument(
            "reveal_radius", default_value="5.0",
            description="coverage_node 의 fog reveal 반경 (m). 멀티 rover 시연 "
                        "가시성 위해 default 5.0. 더 키우려면 reveal_radius:=7.0."),
        OpaqueFunction(function=_launch_setup),
    ])
