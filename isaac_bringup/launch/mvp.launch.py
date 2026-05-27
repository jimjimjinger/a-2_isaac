"""MVP presentation launch — GT cheat 모드 + 검증된 grip/coverage/arm/supervisor/perception.

화성 rover 자율 mineral 수집 시연용 통합 런치. T5 localization 정공법은
trn / sun_yaw 정확도 미검증으로 졸업 작업 (별도). 시연용으론 vehicle_v3 의
GT_SCRIPT 가 발행하는 /ground_truth/odom 을 odom_to_estimated_pose 가
/rover/estimated_pose 로 forwarding (GT cheat). 매 frame cheat 라 정공법 아님.

졸업 경로: localization.launch.py 가 T5 EKF stack 발행 → odom_to_estimated_pose
빼고 mvp.launch.py 의 odom_topic 파라미터 swap (코드 변경 X).

2터미널 실행:
  T1 (Isaac Sim, source 없이):
    cd ~/dev_ws/rover_ws/src/a2_isaac
    temp/ros-isaac-python-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

  T2 (ROS2 노드 묶음):
    source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
    ros2 launch isaac_bringup mvp.launch.py

  (선택) MANUAL 원격조종:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r cmd_vel:=/teleop/cmd_vel
    Dashboard 의 'Switch to MANUAL' 버튼으로 mode 전환.

내장 노드 (8개):
  - odom_to_estimated_pose  (GT cheat: /ground_truth/odom → /rover/estimated_pose)
  - yolo_perception_node    (mineral world XYZ + 이미지 detection)
  - coverage_node           (robot_radius=1.0 보수적 회피)
  - mission_manager_node    (supervisor: EXPLORE/APPROACH/PICK/RTB/COMPLETE + AUTO/MANUAL mux)
  - arm_executor_node       (T2 IK + ik_descend_dz=-0.40 perception bias 보정)
  - battery_monitor_node    (mock drain → /battery_state, critical 시 RTB 트리거)
  - mission_web_node        (Flask+SocketIO, SC2 풍 HUD — http://localhost:8088)
  - web_video_server        (MJPEG: /camera/* + /perception/*_image_annotated)

UI 접속:
  브라우저 http://localhost:8088 (또는 같은 LAN 의 다른 PC 에서 host IP).
  카메라 영상은 web_video_server 가 :8080 에서 MJPEG 로 별도 서빙 — index.html
  의 <img src=...> 가 자동 임베드. enable_web_video:=false 로 끌 수 있음.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


def generate_launch_description() -> LaunchDescription:
    terrain_id_arg = DeclareLaunchArgument(
        "terrain_id", default_value="terrain_00023",
        description="coverage/mission_manager 가 사용할 terrain. T1 Isaac Sim "
                    "의 --terrain 인자와 같아야 obstacle_grid/baseanchor 정합. "
                    "default=terrain_00023 (epic obstacle 4종 = T5 localization "
                    "sun_yaw/TRN 특징점). 다른 terrain 으로 실험 시 명시 전달.")
    terrain_root_arg = DeclareLaunchArgument(
        "terrain_root", default_value=_default_terrain_root(),
        description="terrain_<id> 디렉토리들이 모여있는 루트.")
    collection_goal_arg = DeclareLaunchArgument(
        "collection_goal", default_value="5",
        description="광물 N개 채집 시 RETURN_TO_BASE 자동 전환.")
    battery_drain_arg = DeclareLaunchArgument(
        "battery_drain_per_tick", default_value="0.15",
        description="1Hz tick 당 배터리 감소 %. 0 으로 두면 배터리 종료 조건 비활성.")
    enable_dashboard_arg = DeclareLaunchArgument(
        "enable_dashboard", default_value="true",
        description="Web mission_dashboard (Flask+SocketIO) 자동 실행 여부.")
    enable_web_video_arg = DeclareLaunchArgument(
        "enable_web_video", default_value="true",
        description="web_video_server MJPEG 스트리머 자동 실행 여부 (apt 설치 필요).")
    web_video_port_arg = DeclareLaunchArgument(
        "web_video_port", default_value="8090",
        description=("web_video_server 가 노출할 HTTP 포트. 8080 은 시스템 nginx 가 "
                     "차지하는 일이 있어 충돌 회피용으로 8090 default. "
                     "mission_web_node 에 WEB_VIDEO_PORT 환경변수로 자동 전달."))
    reveal_radius_arg = DeclareLaunchArgument(
        "reveal_radius", default_value="5.0",
        description="coverage_node 의 fog reveal 반경 (m). 시연 가시성 위해 "
                    "default 5.0. 발표 직전 키우려면 reveal_radius:=7.0 식으로 override.")

    terrain_dir = PathJoinSubstitution([
        LaunchConfiguration("terrain_root"),
        LaunchConfiguration("terrain_id"),
    ])

    return LaunchDescription(
        [
            terrain_id_arg,
            terrain_root_arg,
            collection_goal_arg,
            battery_drain_arg,
            enable_dashboard_arg,
            enable_web_video_arg,
            web_video_port_arg,
            reveal_radius_arg,
            # T5 cheat 어댑터 — /ground_truth/odom → /rover/estimated_pose.
            Node(
                package="isaac_drive",
                executable="odom_to_estimated_pose",
                name="odom_to_estimated_pose",
                output="screen",
            ),
            # T2 perception — nav cam YOLO + depth backproject → mineral world XYZ.
            Node(
                package="isaac_perception",
                executable="yolo_perception_node",
                name="yolo_perception_node",
                output="screen",
            ),
            # T3 coverage — BCD anchor sweep + A* obstacle 회피.
            # /cmd_vel → /coverage/cmd_vel_raw remap: supervisor 가 EXPLORE 시
            # 그대로 forward / APPROACH 시 P-control 로 분리 (cmd_vel mux 역할).
            # remap 없으면 supervisor 우회 → mineral 지나침.
            # robot_radius default(0.7) 유지 — 1.0 시도는 실 디버깅 차이 없어 철회.
            Node(
                package="isaac_drive",
                executable="coverage_node",
                name="coverage_node",
                output="screen",
                remappings=[("/cmd_vel", "/coverage/cmd_vel_raw")],
                parameters=[{
                    "terrain_dir": terrain_dir,
                    # matplotlib viewer 비활성 — 시연용으론 Web HUD 의 minimap
                    # 위젯만 사용. viewer 는 디버깅용 (별도 `/tmp/starcraft_map_state.npz`
                    # 폴링, partial-write 시 깜박임 발생). topics 는 계속 발행해
                    # Web HUD 가 동작.
                    "enable_minimap": False,
                    "enable_minimap_topics": True,
                    "reveal_radius": LaunchConfiguration("reveal_radius"),
                }],
            ),
            # Supervisor — EXPLORE→APPROACH→PICK→RTB→COMPLETE 전환 + AUTO/MANUAL
            # mux + arm action client + battery_state subscribe + MissionState 발행.
            Node(
                package="isaac_supervisor",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "approach_engage_dist_m": 30.0,
                        "approach_lin_speed": 1.2,
                        "collection_goal": LaunchConfiguration("collection_goal"),
                        "terrain_dir": terrain_dir,
                    }
                ],
            ),
            # T2 manipulation — DLS-IK 상태머신 + /grasp/command FixedJoint snap.
            # ik_descend_dz=-0.40 = perception 의 z bias (+47cm 평균) 보정.
            # odom_topic default=/ground_truth/odom (GT cheat 모드).
            Node(
                package="isaac_manipulation",
                executable="arm_executor_node",
                name="arm_executor_node",
                output="screen",
                parameters=[{"ik_descend_dz": -0.40}],
            ),
            # Mock 배터리 드레인 publisher — mission_manager 가 subscribe.
            Node(
                package="isaac_supervisor",
                executable="battery_monitor_node",
                name="battery_monitor_node",
                output="screen",
                parameters=[{
                    "drain_per_tick": LaunchConfiguration("battery_drain_per_tick"),
                }],
            ),
            # Web dashboard — Flask + SocketIO, SC2 풍 HUD.
            # http://localhost:8088 접속. read-only 디자인 단계 — 버튼 비활성.
            # WEB_VIDEO_PORT 환경변수로 카메라 MJPEG 포트를 전달 (index.html
            # 의 <img src=...> 가 그 포트로 향함).
            Node(
                package="isaac_supervisor",
                executable="mission_web_node",
                name="mission_web_node",
                output="screen",
                additional_env={
                    "WEB_VIDEO_PORT": LaunchConfiguration("web_video_port"),
                },
                condition=IfCondition(LaunchConfiguration("enable_dashboard")),
            ),
            # MJPEG 스트리머 — 브라우저가 <img src="http://host:8080/stream?topic=..."> 로 임베드.
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                parameters=[{
                    "port": LaunchConfiguration("web_video_port"),
                    "address": "0.0.0.0",
                }],
                condition=IfCondition(LaunchConfiguration("enable_web_video")),
            ),
        ]
    )
