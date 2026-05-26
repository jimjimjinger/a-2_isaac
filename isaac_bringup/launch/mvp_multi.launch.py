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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rover_group(ns: str):
    """단일 rover 의 5 노드 묶음 — 토픽 모두 /<ns>/... prefix."""
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
        # /cmd_vel → /coverage/cmd_vel_raw remap: supervisor 가 mux
        # minimap viewer: rover_1 만 띄우고 rover_2+ 는 state file 만 쓰기 →
        # 단일 matplotlib 창이 모든 rover overlay
        Node(
            package="isaac_drive",
            executable="coverage_node",
            name="coverage_node",
            namespace=ns,
            output="screen",
            parameters=[{
                "pose_topic":    f"{nsp}/rover/estimated_pose",
                "cmd_vel_topic": f"{nsp}/coverage/cmd_vel_raw",
                "minimap_rover_id": ns,
                # 첫 rover (rover_1) 만 viewer subprocess 띄움
                "minimap_spawn_viewer": (ns == "rover_1"),
            }],
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
                "arm_action_name":    f"{nsp}/execute_arm_task",
                # Mineral claim 협조 (공유 토픽 /mineral_claims, namespace 무관)
                "enable_mineral_claim": True,
                "claim_rover_id":      ns,
                "claim_topic":         "/mineral_claims",
                "claim_skip_radius_m": 1.5,
                "claim_ttl_sec":       5.0,
                # 동적 충돌 회피 (공유 /rover_positions 로 상대 위치 받음)
                "enable_rover_avoid":  True,
                "rover_positions_topic": "/rover_positions",
                "rover_avoid_radius_m":  1.2,
                "rover_position_ttl_sec": 2.0,
                "rover_replan_trigger_m": 0.8,
            }],
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
    nodes = []
    for ns in rovers:
        nodes.extend(_rover_group(ns))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "rovers", default_value="rover_1 rover_2",
            description="공백 구분 namespace 리스트. 예: 'rover_1 rover_2 rover_3'"),
        OpaqueFunction(function=_launch_setup),
    ])
