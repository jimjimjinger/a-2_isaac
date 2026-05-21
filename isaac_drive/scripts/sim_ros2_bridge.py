"""Isaac Sim ↔ ROS2 브리지 (정공법) — isaacsim.ros2.bridge 확장 사용.

UDP 우회(sim_ros_bridge.py + coverage_udp_relay.py) 대신, Isaac Sim 의
ROS2 Bridge 확장으로 ROS2 토픽을 C++/DDS 레벨에서 직접 발행·구독한다.
rclpy 를 안 거치므로 Isaac Sim Python(3.11) vs ROS2 Humble(3.10) 버전
불일치와 무관하다 — UDP 릴레이가 필요 없다.

  · 구독: /cmd_vel (geometry_msgs/Twist)  — ROS2SubscribeTwist OmniGraph 노드
          → 매 프레임 Python 이 읽어 RoverController.drive() (Ackermann 변환)
  · 발행: /odom    (nav_msgs/Odometry)    — ROS2PublishOdometry 에 로버 절대 월드 pose 기록

닫힌 루프:
  coverage_node ─/cmd_vel─▶ [이 브리지] ─▶ Isaac Sim 로버
  coverage_node ◀─/rover/estimated_pose─ odom_to_estimated_pose ◀─/odom─ [이 브리지]

실행 (터미널 2개 — UDP 릴레이 불필요):
    # A — Isaac Sim 브리지
    <isaac-python> sim_ros2_bridge.py --terrain terrain_00004
    # B — coverage + odom 어댑터 (시스템 ROS2)
    ros2 run isaac_drive odom_to_estimated_pose
    ros2 run isaac_drive coverage_node

⚠️ coverage_node 와 ROS_DOMAIN_ID 를 맞출 것 (이 프로세스 실행 셸의 env).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

# argparse 는 SimulationApp 보다 먼저.
_parser = argparse.ArgumentParser(description="Isaac Sim ↔ ROS2 bridge (정공법)")
_parser.add_argument("--terrain", default="terrain_00004",
                     help="terrain id (coverage_node 의 terrain_dir 와 일치시킬 것). "
                          "terrain_00001 은 v1 잔재라 씬 어긋남 — v2 terrain 사용")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_args, _ = _parser.parse_known_args()

# SimulationApp 은 다른 omniverse import 보다 먼저.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

# ── ROS2 Bridge 확장 활성화 (SimulationApp 직후) ──
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)            # .../isaac_drive
WS = os.path.dirname(PKG_ROOT)              # .../a2_isaac
sys.path.insert(0, HERE)

from rover import RoverController

TERRAIN_ID = _args.terrain
MARS_WORLD = f"{WS}/isaac_sim/worlds/{TERRAIN_ID}.usd"
TERRAIN_DIR = f"{WS}/isaac_sim/assets/generated_terrains/{TERRAIN_ID}"

GRAPH_PATH = "/ActionGraph"
CMD_VEL_TOPIC = "cmd_vel"   # → /cmd_vel
ODOM_TOPIC = "odom"         # → /odom


def _build_ros2_graph() -> None:
    """ROS2 Bridge OmniGraph 구축 — /cmd_vel 구독 + /odom 발행.

    구독: ROS2SubscribeTwist 노드 — Python 이 매 프레임 읽어 Ackermann 구동.
    발행: ROS2PublishOdometry 노드 — Python 이 매 프레임 로버의 **절대 월드
          pose** 를 inputs 에 써 넣는다. IsaacComputeOdometry 는 spawn 을
          원점(0,0)으로 하는 상대 odometry 라 쓰지 않는다 — coverage_node 가
          obstacle_grid 를 절대좌표로 인덱싱하므로 절대 pose 가 필수.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.SET_VALUES: [
                ("SubTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("PubOdom.inputs:topicName", ODOM_TOPIC),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PubOdom.inputs:timeStamp"),
            ],
        },
    )


def main() -> None:
    if not os.path.isfile(MARS_WORLD):
        print(f"[sim_ros2_bridge] ✗ 월드 USD 없음: {MARS_WORLD}")
        simulation_app.close()
        sys.exit(1)

    # ── Isaac Sim World + per-terrain 씬 ──
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=MARS_WORLD, prim_path="/World/MarsScene")
    print(f"[sim_ros2_bridge] 씬 로드: {MARS_WORLD}")

    # ── 로버 spawn (meta.json 의 검증된 spawn 위치) ──
    spawn_xyz = (0.0, 0.0, 1.0)
    meta_path = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            spots = json.load(f).get("spawn_locations") or []
        if spots:
            s = spots[0]
            spawn_xyz = (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)
    print(f"[sim_ros2_bridge] 로버 spawn: {spawn_xyz}")

    rover = RoverController(my_world)
    rover.spawn(initial_position=spawn_xyz)
    for _ in range(10):
        simulation_app.update()
    my_world.reset()
    rover.initialize()

    # ── ROS2 Bridge OmniGraph 구축 (로버 prim 존재 후) ──
    _build_ros2_graph()
    print(f"[sim_ros2_bridge] ROS2 그래프 구축 완료 — "
          f"구독 /{CMD_VEL_TOPIC}, 발행 /{ODOM_TOPIC}")

    my_world.play()
    print("[sim_ros2_bridge] ready — coverage_node + odom_to_estimated_pose 를 띄우세요")

    lin_attr = og.Controller.attribute(
        f"{GRAPH_PATH}/SubTwist.outputs:linearVelocity")
    ang_attr = og.Controller.attribute(
        f"{GRAPH_PATH}/SubTwist.outputs:angularVelocity")
    pos_attr = og.Controller.attribute(f"{GRAPH_PATH}/PubOdom.inputs:position")
    ori_attr = og.Controller.attribute(f"{GRAPH_PATH}/PubOdom.inputs:orientation")

    lin_x = ang_z = 0.0
    step = 0
    try:
        while simulation_app.is_running():
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            # 로버 절대 월드 pose → /odom 발행 노드에 기록 (상대 odometry 아님)
            x, y, yaw = rover.get_pose_2d()
            og.Controller.set(pos_attr, [float(x), float(y), 0.0])
            og.Controller.set(
                ori_attr,
                [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

            # /cmd_vel (그래프가 구독) → Ackermann 구동
            lin = og.Controller.get(lin_attr)
            ang = og.Controller.get(ang_attr)
            if lin is not None:
                lin_x = float(lin[0])
            if ang is not None:
                ang_z = float(ang[2])
            rover.drive(lin_x, ang_z)

            if step % 120 == 0:
                print(f"[sim_ros2_bridge] step {step:6d}  "
                      f"pose=({x:+6.2f},{y:+6.2f})  cmd=({lin_x:+.2f},{ang_z:+.2f})")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
