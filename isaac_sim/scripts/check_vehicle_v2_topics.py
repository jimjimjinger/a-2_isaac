#!/usr/bin/env python3
"""vehicle_v3.usd 에서 발행되는 ROS2 토픽을 확인하는 스크립트.

Isaac Sim을 GUI 모드로 실행해서 vehicle_v3.usd를 로드한 후,
시뮬레이션을 play 하고 60초간 대기한다. 이 사이에 다른 터미널에서
ros2 topic list / ros2 topic hz <topic> 으로 토픽을 확인한다.

실행 순서:
  터미널 1 (Isaac Sim Python):
    source /opt/ros/humble/setup.bash
    source ~/dev_ws/rover_ws/install/setup.bash
    isaac-python src/a2_isaac/isaac_sim/scripts/check_vehicle_v2_topics.py

  터미널 2 (ROS2 확인):
    source /opt/ros/humble/setup.bash
    ros2 topic list
    ros2 topic hz /camera/rover/image_raw
    ros2 topic echo /imu/data --once
    ros2 topic info /cmd_vel
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

_SCRIPT_PATH = Path(__file__).resolve()
A2_ROOT = _SCRIPT_PATH.parents[2]
VEHICLE_USD = A2_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v3.usd"

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Usd
from isaacsim.core.api import World

print(f"\n{'='*60}")
print(f"[check] vehicle_v3.usd 로드 중...")
print(f"  경로: {VEHICLE_USD}")
print(f"{'='*60}\n")

if not VEHICLE_USD.exists():
    print(f"[ERROR] USD 파일 없음: {VEHICLE_USD}")
    simulation_app.close()
    sys.exit(1)

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# USD 로드
root_prim = stage.DefinePrim("/World", "Xform")
root_prim.GetReferences().AddReference(str(VEHICLE_USD))
for _ in range(20):
    simulation_app.update()

# prim 트리 스캔 — ActionGraph 노드 출력
print("[check] Stage prim 스캔 (Action Graph / ROS2 노드):")
ros2_nodes = []
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    prim_type = prim.GetTypeName()
    path_str = str(prim.GetPath())
    if "ActionGraph" in path_str or "OgnRos2" in prim_type or "Ros2" in prim_type:
        print(f"  {path_str}  [{prim_type}]")
        ros2_nodes.append(path_str)

if not ros2_nodes:
    print("  (ROS2 관련 prim 없음 — Action Graph가 없는 순수 Vehicle USD 일 수 있음)")

# 시뮬레이션 play
world.play()
print("\n[check] 시뮬레이션 시작됨. 30초간 대기 중...")
print("  → 다른 터미널에서 아래 명령으로 토픽 확인:")
print("      source /opt/ros/humble/setup.bash")
print("      ros2 topic list")
print("      ros2 topic hz /camera/rover/image_raw")
print("      ros2 topic echo /imu/data --once\n")

EXPECTED_TOPICS = [
    "/cmd_vel              geometry_msgs/msg/Twist          (INPUT — 로버 속도 명령)",
    "/camera/rover/image_raw  sensor_msgs/msg/Image         (OUTPUT — 로버 카메라 RGB)",
    "/camera/rover/depth       sensor_msgs/msg/Image         (OUTPUT — 로버 깊이 카메라)",
    "/camera/rover/camera_info sensor_msgs/msg/CameraInfo    (OUTPUT — 카메라 내부 파라미터)",
    "/imu/data                 sensor_msgs/msg/Imu           (OUTPUT — IMU 가속도/자이로)",
    "/joint_states_raw         sensor_msgs/msg/JointState    (OUTPUT — 전체 관절 상태)",
]
print("[check] 예상 토픽 목록 (rover_m0609_localization.usd 기준):")
for t in EXPECTED_TOPICS:
    print(f"  {t}")
print()

deadline = time.time() + 30.0
step = 0
while simulation_app.is_running() and time.time() < deadline:
    world.step(render=True)
    step += 1
    if step % 60 == 0:
        remaining = int(deadline - time.time())
        print(f"  [{remaining:2d}초 남음] 시뮬 중... (ros2 topic list 실행하세요)")

world.stop()
print("\n[check] 완료. 시뮬레이션 종료.")
simulation_app.close()
