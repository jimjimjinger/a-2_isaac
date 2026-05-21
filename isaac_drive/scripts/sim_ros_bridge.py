"""Isaac Sim ↔ UDP 브리지 — coverage_node 를 Isaac Sim 물리 위에서 검증.

⚠️ 이 환경(Isaac Sim Python 3.11 vs ROS2 Humble rclpy 3.10)에서는 isaac-python
   안에서 `import rclpy` 가 원천적으로 불가능하다(C 확장 ABI 불일치, 2회 검증).
   그래서 Isaac Sim 프로세스는 ROS2 를 직접 안 쓰고, stdlib socket(UDP)로만
   pose/cmd 를 주고받는다. UDP↔ROS2 변환은 coverage_udp_relay.py(일반 ROS2
   프로세스, Python 3.10)가 담당한다.

  coverage_node ─ROS2─ coverage_udp_relay ─UDP─ sim_ros_bridge ─ Isaac Sim
       ▲                                                              │
       └──────────────── /rover/estimated_pose ◀─────────────────────┘

  · 매 step: 로버 GT pose (x,y,yaw) → UDP 로 relay 에 송신
  · relay 가 보낸 최신 cmd (lin,ang) → RoverController.drive() 로 로버 구동

─────────────────────────────────────────────────────────────────────────────
실행 (터미널 3개):

  # A — UDP↔ROS2 릴레이 (시스템 ROS2 source 한 셸)
  python3 scripts/coverage_udp_relay.py

  # B — coverage 노드 (시스템 ROS2)
  ros2 run isaac_drive coverage_node

  # C — Isaac Sim 브리지 (ROS2 source 하지 말 것 — isaac-python 환경 오염 방지)
  cd .../isaac_drive/scripts
  isaac-python sim_ros_bridge.py              # 기본 terrain_00001
  isaac-python sim_ros_bridge.py --terrain terrain_00002 --headless

확인: Isaac Sim 창에서 로버 sweep / `ros2 topic echo /mission_state` 진척률.
⚠️ MARS_WORLD 는 alias 가 아니라 명시적 worlds/<terrain_id>.usd 사용.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys

# argparse 는 SimulationApp 보다 먼저.
_parser = argparse.ArgumentParser(description="Isaac Sim ↔ UDP coverage 검증 브리지")
_parser.add_argument("--terrain", default="terrain_00001",
                     help="terrain id (coverage_node 의 terrain_dir 와 일치시킬 것)")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_parser.add_argument("--relay-host", default="127.0.0.1")
_parser.add_argument("--pose-port", type=int, default=5005, help="pose 송신 → relay")
_parser.add_argument("--cmd-port", type=int, default=5006, help="cmd 수신 ← relay")
_args, _ = _parser.parse_known_args()

# SimulationApp 은 다른 omniverse import 보다 먼저.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

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

_POSE_FMT = "<fff"   # x, y, yaw
_CMD_FMT = "<ff"     # lin, ang


def main() -> None:
    if not os.path.isfile(MARS_WORLD):
        print(f"[sim_ros_bridge] ✗ 월드 USD 없음: {MARS_WORLD}")
        simulation_app.close()
        sys.exit(1)

    # ── Isaac Sim World + per-terrain 씬 ──
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=MARS_WORLD, prim_path="/World/MarsScene")
    print(f"[sim_ros_bridge] 씬 로드: {MARS_WORLD}")

    # ── 로버 spawn (meta.json 의 검증된 spawn 위치) ──
    spawn_xyz = (0.0, 0.0, 1.0)
    meta_path = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            spots = json.load(f).get("spawn_locations") or []
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

    # ── UDP 소켓 ──
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        # pose 송신
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        # cmd 수신
    rx.bind(("0.0.0.0", _args.cmd_port))
    rx.setblocking(False)
    relay_pose_addr = (_args.relay_host, _args.pose_port)
    print(f"[sim_ros_bridge] UDP — pose→{relay_pose_addr}, cmd 수신 :{_args.cmd_port}")
    print("[sim_ros_bridge] ready — coverage_udp_relay + coverage_node 를 띄우세요")

    lin = ang = 0.0
    step = 0
    try:
        while simulation_app.is_running():
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            # 로버 pose → relay
            x, y, yaw = rover.get_pose_2d()
            tx.sendto(struct.pack(_POSE_FMT, x, y, yaw), relay_pose_addr)

            # relay 가 보낸 cmd 중 가장 최신만 사용 (큐 비우기)
            while True:
                try:
                    data, _ = rx.recvfrom(64)
                except BlockingIOError:
                    break
                if len(data) == struct.calcsize(_CMD_FMT):
                    lin, ang = struct.unpack(_CMD_FMT, data)
            rover.drive(lin, ang)

            if step % 120 == 0:
                print(f"[sim_ros_bridge] step {step:6d}  "
                      f"pose=({x:+6.2f},{y:+6.2f},{math.degrees(yaw):+6.1f}°)  "
                      f"cmd=({lin:+.2f},{ang:+.2f})")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        tx.close()
        rx.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
