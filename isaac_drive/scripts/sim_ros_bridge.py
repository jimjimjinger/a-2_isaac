"""Isaac Sim ↔ ROS2 데이터 브리지 — coverage_node 를 Isaac Sim 물리 위에서 검증.

run_coverage_test.py 에서 coverage 루프(SectorPlanner/Navigator/Mission)를 떼고,
그 자리에 ROS2 pub/sub 를 붙인 것. Isaac Sim 안에서 로버를 spawn 하고 매 step:

  · 로버 GT pose  →  /rover/estimated_pose (geometry_msgs/PoseWithCovarianceStamped)
  · /cmd_vel (geometry_msgs/Twist) 구독  →  RoverController.drive() 로 로버 구동

coverage_node 는 별도 프로세스(ros2 run)로 띄운다. 그러면 닫힌 루프가 닫힌다:

  coverage_node ── /cmd_vel ─────────────▶ sim_ros_bridge ─▶ Isaac Sim 로버
  coverage_node ◀─ /rover/estimated_pose ─ sim_ros_bridge ◀─ 로버 pose

─────────────────────────────────────────────────────────────────────────────
실행 (터미널 2개, 둘 다 ROS2 source 필요):

  # 터미널 A — Isaac Sim 브리지
  cd .../isaac_drive/scripts
  <isaac-python> sim_ros_bridge.py                 # 기본 terrain_00001
  <isaac-python> sim_ros_bridge.py --terrain terrain_00002

  # 터미널 B — coverage 노드 (terrain 을 브리지와 반드시 일치시킬 것)
  ros2 run isaac_drive coverage_node               # 기본 terrain_00001
  ros2 run isaac_drive coverage_node --ros-args -p terrain_dir:=<terrain_00002 경로>

확인: Isaac Sim 창에서 로버가 sweep → `ros2 topic echo /mission_state` 로 진척률.

⚠️ 전제: isaac-python 환경에서 `import rclpy` 가 돼야 한다(ROS2 Humble + Isaac
   Sim 둘 다 Python 3.10). 안 되면 ROS2 를 source 한 셸에서 isaac-python 을
   실행하거나, rclpy 를 Isaac Sim Python 에 설치해야 한다.
⚠️ MARS_WORLD 는 alias(mars_exploration_world.usd)가 아니라 명시적 per-terrain
   USD(worlds/<terrain_id>.usd)를 쓴다 — planner 데이터와 씬을 같은 terrain 으로.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

# argparse 는 SimulationApp 보다 먼저 — SimulationApp 이 sys.argv 를 건드리기 전에.
_parser = argparse.ArgumentParser(description="Isaac Sim ↔ ROS2 coverage 검증 브리지")
_parser.add_argument("--terrain", default="terrain_00001",
                     help="terrain id (coverage_node 의 terrain_dir 와 일치시킬 것)")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_args, _ = _parser.parse_known_args()

# SimulationApp 은 다른 omniverse import 보다 먼저 와야 한다.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node

# isaac_drive/scripts/ 의 rover.py(RoverController) import 가능하게.
HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)            # .../isaac_drive
WS = os.path.dirname(PKG_ROOT)              # .../a2_isaac
sys.path.insert(0, HERE)

from rover import RoverController

TERRAIN_ID = _args.terrain
MARS_WORLD = f"{WS}/isaac_sim/worlds/{TERRAIN_ID}.usd"
TERRAIN_DIR = f"{WS}/isaac_sim/assets/generated_terrains/{TERRAIN_ID}"


class SimRosBridge(Node):
    """로버 pose 를 ROS2 로 발행하고, /cmd_vel 을 받아 로버를 구동한다."""

    def __init__(self, rover: RoverController):
        super().__init__("sim_ros_bridge")
        self.rover = rover
        self._lin = 0.0
        self._ang = 0.0
        self._cmd_count = 0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/rover/estimated_pose", 10)

    def _on_cmd(self, msg: Twist) -> None:
        self._lin = msg.linear.x
        self._ang = msg.angular.z
        self._cmd_count += 1

    def publish_pose(self) -> tuple[float, float, float]:
        """로버 GT pose 를 /rover/estimated_pose 로 발행."""
        x, y, yaw = self.rover.get_pose_2d()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)
        return x, y, yaw

    def drive_rover(self) -> None:
        """마지막으로 받은 /cmd_vel 을 로버에 적용."""
        self.rover.drive(self._lin, self._ang)

    @property
    def cmd_count(self) -> int:
        return self._cmd_count


def main() -> None:
    if not os.path.isfile(MARS_WORLD):
        print(f"[sim_ros_bridge] ✗ 월드 USD 없음: {MARS_WORLD}")
        simulation_app.close()
        sys.exit(1)

    # ── Isaac Sim World + per-terrain 씬 USD ──
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=MARS_WORLD, prim_path="/World/MarsScene")
    print(f"[sim_ros_bridge] 씬 로드: {MARS_WORLD}")

    # ── 로버 spawn (meta.json 의 검증된 spawn 위치 사용) ──
    spawn_xyz = (0.0, 0.0, 1.0)
    meta_path = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        spots = meta.get("spawn_locations") or []
        if spots:
            s = spots[0]
            spawn_xyz = (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)
    print(f"[sim_ros_bridge] 로버 spawn: {spawn_xyz}")

    rover = RoverController(my_world)
    rover.spawn(initial_position=spawn_xyz)
    for _ in range(10):
        simulation_app.update()
    my_world.reset()
    rover.initialize()
    my_world.play()

    # ── ROS2 브리지 노드 ──
    rclpy.init()
    bridge = SimRosBridge(rover)
    print("[sim_ros_bridge] ready — 다른 터미널에서 coverage_node 를 띄우세요:")
    print(f"[sim_ros_bridge]   ros2 run isaac_drive coverage_node"
          f"{'' if TERRAIN_ID == 'terrain_00001' else f'  (-p terrain_dir:={TERRAIN_DIR})'}")

    step = 0
    try:
        while simulation_app.is_running() and rclpy.ok():
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            x, y, yaw = bridge.publish_pose()
            rclpy.spin_once(bridge, timeout_sec=0.0)   # /cmd_vel 콜백 처리
            bridge.drive_rover()

            if step % 120 == 0:
                print(f"[sim_ros_bridge] step {step:6d}  "
                      f"pose=({x:+6.2f},{y:+6.2f},{math.degrees(yaw):+6.1f}°)  "
                      f"cmd=({bridge._lin:+.2f},{bridge._ang:+.2f})  "
                      f"cmd_rx={bridge.cmd_count}")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
