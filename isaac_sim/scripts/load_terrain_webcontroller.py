#!/usr/bin/env python3
"""vehicle_v3.usd 를 메인 스테이지로 열고, terrain 을 sublayer 로 추가한다.

vehicle_v3.usd 를 reference 로 붙이면 OmniGraph Action Graph 노드의
prim 경로가 바뀌어 ROS2 토픽이 비활성화된다.
메인 스테이지로 열면 Action Graph 경로가 그대로 유지되어 ROS2 Bridge 가
정상 동작한다.

실행:
  source /opt/ros/humble/setup.bash
  source ~/dev_ws/rover_ws/install/setup.bash

  /mnt/data/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
    src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py

  # 특정 terrain 지정:
    ... load_terrain_webcontroller.py --terrain terrain_00001

  # terrain 없이 vehicle 만:
    ... load_terrain_webcontroller.py --no-terrain
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_SCRIPT_PATH = Path(__file__).resolve()
A2_ROOT      = _SCRIPT_PATH.parents[2]
VEHICLE_V3_USD = A2_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v3.usd"
GENERATED_DIR  = A2_ROOT / "isaac_sim" / "assets" / "generated_terrains"
WORLDS_DIR     = A2_ROOT / "isaac_sim" / "worlds"

# ── 인수 파싱 (SimulationApp 전) ──────────────────────────────────────────────
ap = argparse.ArgumentParser(description="vehicle_v3 + terrain 웹 조종 로더")
ap.add_argument("--terrain", default=None,
                help="terrain ID (예: terrain_00001). 기본: 가장 최근 생성본")
ap.add_argument("--usd", default=None,
                help="terrain_only.usd 직접 경로 (--terrain 보다 우선)")
ap.add_argument("--no-terrain", action="store_true",
                help="terrain 없이 vehicle_v3.usd 만 실행")
ap.add_argument("--gravity", type=float, default=3.72,
                help="중력 가속도 m/s² (기본: 화성 3.72)")
ARGS = ap.parse_args()

# ── SimulationApp ─────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
import omni.kit.app
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World

# ── ROS2 Bridge 확장 활성화 ───────────────────────────────────────────────────
_ext_mgr = omni.kit.app.get_app().get_extension_manager()
for _ext_name in ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"):
    try:
        if not _ext_mgr.is_extension_enabled(_ext_name):
            _ext_mgr.set_extension_enabled_immediate(_ext_name, True)
            print(f"[ros2] 확장 활성화: {_ext_name}")
        else:
            print(f"[ros2] 이미 활성화됨: {_ext_name}")
        break
    except Exception as e:
        print(f"[ros2] {_ext_name} 실패: {e}")

for _ in range(5):
    simulation_app.update()


# ── terrain USD 경로 결정 ─────────────────────────────────────────────────────
def _resolve_terrain_usd():
    if ARGS.no_terrain:
        return None
    if ARGS.usd:
        p = Path(ARGS.usd).expanduser().resolve()
        if not p.exists():
            print(f"[ERROR] USD 없음: {p}")
            sys.exit(1)
        return p
    if ARGS.terrain:
        p = GENERATED_DIR / ARGS.terrain / "terrain_only.usd"
        if not p.exists():
            print(f"[ERROR] terrain 없음: {p}")
            sys.exit(1)
        return p
    # 가장 최근 terrain_only.usd
    candidates = sorted(GENERATED_DIR.glob("*/terrain_only.usd"),
                        key=lambda x: x.stat().st_mtime, reverse=True)
    if candidates:
        return candidates[0]
    # 최신 master scene fallback
    mew = WORLDS_DIR / "mars_exploration_world.usd"
    if mew.exists():
        print("[warn] generated terrain 없음 — mars_exploration_world.usd 사용")
        return mew
    print("[warn] terrain USD 없음 — vehicle 만 로드")
    return None

terrain_usd = _resolve_terrain_usd()

# ── vehicle_v3.usd 를 메인 스테이지로 열기 ───────────────────────────────────
print(f"\n{'='*60}")
print(f"  Vehicle  : {VEHICLE_V3_USD}")
print(f"  Terrain  : {terrain_usd or '없음'}")
print(f"  Gravity  : {ARGS.gravity} m/s²")
print(f"{'='*60}\n")

if not VEHICLE_V3_USD.exists():
    print(f"[ERROR] vehicle_v3.usd 없음: {VEHICLE_V3_USD}")
    simulation_app.close()
    sys.exit(1)

print("[1/3] vehicle_v3.usd 를 메인 스테이지로 열기...")
ctx = omni.usd.get_context()
ctx.open_stage(str(VEHICLE_V3_USD))
for _ in range(20):
    simulation_app.update()
print("  → 완료")

# ── terrain 을 sublayer 로 추가 ───────────────────────────────────────────────
if terrain_usd is not None:
    print(f"[2/3] terrain sublayer 추가: {terrain_usd.name}")
    stage = ctx.get_stage()
    root_layer = stage.GetRootLayer()

    terrain_str = str(terrain_usd)
    if terrain_str not in root_layer.subLayerPaths:
        root_layer.subLayerPaths.append(terrain_str)

    for _ in range(15):
        simulation_app.update()

    # PhysicsScene (화성 중력) — vehicle_v3 에 없으면 추가
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(ARGS.gravity)
        print(f"  → PhysicsScene 추가 (중력 {ARGS.gravity} m/s²)")
    print("  → terrain 로드 완료")
else:
    print("[2/3] terrain 없이 vehicle 만 실행")

# ── ROS2 Action Graph 확인 ────────────────────────────────────────────────────
print("[3/3] ROS2 Action Graph 확인...")
stage = ctx.get_stage()
ag_paths = []
for prim in Usd.PrimRange(stage.GetPseudoRoot()):
    path = str(prim.GetPath())
    if "ActionGraph" in path and prim.GetTypeName() != "":
        ag_paths.append(path)
        if len(ag_paths) <= 6:
            print(f"  ✓ {path}  [{prim.GetTypeName()}]")
if len(ag_paths) > 6:
    print(f"  ... 외 {len(ag_paths)-6}개")
if not ag_paths:
    print("  ⚠ ActionGraph 없음 — vehicle_v3.usd ROS2 설정 확인 필요")

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
world.play()

print(f"\n{'='*60}")
print("  시뮬레이션 play 시작!")
print()
print("  발행 토픽 (Isaac Sim → ROS2):")
print("    /camera/rover/image_raw   (RGB 카메라 ~60 Hz)")
print("    /imu/data                 (IMU ~102 Hz)")
print("    /joint_states_raw         (관절 상태)")
print()
print("  구독 토픽 (ROS2 → Isaac Sim):")
print("    /cmd_vel                  (속도 명령)")
print()
print("  웹 서버 시작 (다른 터미널):")
print("    source /opt/ros/humble/setup.bash")
print("    cd src/a2_isaac/isaac_sim/web_controller && python3 main.py")
print()
print("  브라우저: http://localhost:8001")
print("  종료: Q 또는 Ctrl+C")
print(f"{'='*60}\n")

step = 0
try:
    while simulation_app.is_running():
        world.step(render=True)
        step += 1
        if step % 600 == 0:
            print(f"  [{step // 60}s] 실행 중 — http://localhost:8001")
except KeyboardInterrupt:
    print("\n[종료] Ctrl+C")

world.stop()
simulation_app.close()
