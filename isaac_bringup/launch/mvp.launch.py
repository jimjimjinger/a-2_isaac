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

내장 노드 (5개):
  - odom_to_estimated_pose  (GT cheat: /ground_truth/odom → /rover/estimated_pose)
  - yolo_perception_node    (mineral world XYZ + 이미지 detection)
  - coverage_node           (robot_radius=1.0 보수적 회피)
  - mission_manager_node    (supervisor: EXPLORE/APPROACH/PICK phase mux)
  - arm_executor_node       (T2 IK + ik_descend_dz=-0.40 perception bias 보정)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
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
            ),
            # Supervisor — EXPLORE→APPROACH→PICK phase 전환 + cmd_vel mux + arm
            # action client. odom_topic default=/ground_truth/odom (GT cheat 모드).
            Node(
                package="isaac_supervisor",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "approach_engage_dist_m": 30.0,
                        "approach_lin_speed": 1.2,
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
        ]
    )
