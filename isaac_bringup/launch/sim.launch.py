"""Launch isaac_sim mock service bridge — sim_bridge_node.

⚠️ **이 launch 는 Isaac Sim 자체를 띄우지 않습니다**. sim_bridge_node 는
mission orchestration 테스트용 mock lifecycle services (CheckSystemReady /
ResetSimulation / SaveExplorationMap) 만 제공.

Isaac Sim 본체 실행은:
  cd ~/dev_ws/rover_ws/src/a2_isaac
  tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

실행 (mock service 만 필요한 단위 테스트용):
  ros2 launch isaac_bringup sim.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="isaac_sim",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
            ),
        ]
    )
