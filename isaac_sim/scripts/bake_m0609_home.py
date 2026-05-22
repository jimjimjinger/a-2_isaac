"""vehicle_v1.usd 의 m0609 HOME 자세를 link xform 에 굳힌다 (Isaac Sim 필요).

build_integrated_vehicle.py(순수 pxr)는 m0609 joint 의 drive target/state 만
HOME 으로 설정한다 — 이는 물리 시뮬 play 시점에 적용되므로, USD 를 정지
상태로 열면 로봇팔이 0도(+Z 직립)로 보인다.

이 스크립트는 vehicle_v1.usd 를 Isaac Sim 에 로드해 articulation 으로 HOME
joint 자세를 적용한 뒤, PhysX 가 FK 로 계산한 링크 자세를 USD 에 그대로
export 한다. → 정지 상태에서도 HOME 자세로 보인다.

build_integrated_vehicle.py 를 먼저 돌린 뒤 실행:
    <isaac-python> isaac_sim/scripts/bake_m0609_home.py

⚠️ Isaac Sim API 의존 스크립트라 첫 실행 시 로그로 검증하며 맞춰야 한다.
   (dof_names 순서, articulation root, USD write-back 동작 확인)
"""
import sys

# SimulationApp 환경에서 print 가 버퍼링돼 유실되지 않도록 line-buffered.
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import math
from pathlib import Path

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from pxr import Usd, UsdGeom, UsdPhysics

# ── 경로 / 상수 ─────────────────────────────────────────────────────
_ISAAC_SIM = Path(__file__).resolve().parents[1]
VEHICLE_V1 = _ISAAC_SIM / "assets" / "vehicle" / "vehicle_v1.usd"

# articulation root (진단: vehicle_v1.usd 의 ArticulationRoot prim)
ARTIC_ROOT = "/Root/Vehicle/m0609/base_link"
M0609_ROOT = "/Root/Vehicle/m0609"

# m0609 HOME 자세 — joint_3·joint_5 를 90도 (radian). '카메라 down'.
HOME_RAD = {
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": math.radians(90.0),
    "joint_4": 0.0,
    "joint_5": math.radians(90.0),
    "joint_6": 0.0,
}


def _print_m0609_link_xforms(stage, tag):
    """m0609 링크들의 로컬 translate 를 찍어 자세 변화 확인."""
    m = stage.GetPrimAtPath(M0609_ROOT)
    print(f"  [{tag}] m0609 링크 translate:")
    for c in m.GetChildren():
        if c.GetTypeName() != "Xform":
            continue
        for op in UsdGeom.Xformable(c).GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                t = op.Get()
                print(f"      {c.GetName()}: ({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})")
                break


def main():
    if not VEHICLE_V1.is_file():
        print(f"[bake] ✗ {VEHICLE_V1} 없음 — build_integrated_vehicle.py 먼저 실행")
        simulation_app.close()
        return

    # 1) vehicle_v1.usd 열기
    ctx = omni.usd.get_context()
    ctx.open_stage(str(VEHICLE_V1))
    for _ in range(20):
        simulation_app.update()
    stage = ctx.get_stage()
    print(f"[bake] 로드: {VEHICLE_V1.name}")
    _print_m0609_link_xforms(stage, "before")

    # 2) World + m0609 articulation
    world = World()
    # 중력 0 — joint 자세만 잡고 차량이 추락하지 않도록
    try:
        world.get_physics_context().set_gravity(0.0)
    except Exception as e:
        print(f"[bake] 중력 0 설정 실패(무시): {e}")

    art = SingleArticulation(prim_path=ARTIC_ROOT, name="vehicle_art")
    world.scene.add(art)
    world.reset()
    art.initialize()

    dof = list(art.dof_names)
    print(f"[bake] dof_names ({len(dof)}): {dof}")

    # 3) HOME joint 자세 적용 (m0609 6축만, 나머지는 현재값 유지)
    pos = np.array(art.get_joint_positions(), dtype=float)
    applied = []
    for i, name in enumerate(dof):
        if name in HOME_RAD:
            pos[i] = HOME_RAD[name]
            applied.append(name)
    art.set_joint_positions(pos)
    print(f"[bake] HOME 적용: {applied}")

    # 4) step — joint 자세를 USD 에 반영
    if not world.is_playing():
        world.play()
    for _ in range(30):
        world.step(render=False)

    # 5) 검증 출력 + export
    _print_m0609_link_xforms(stage, "after")
    stage.GetRootLayer().Export(str(VEHICLE_V1))
    print(f"[bake] ✓ {VEHICLE_V1.name} 저장 — m0609 HOME 자세 link xform 에 굳힘")

    simulation_app.close()


if __name__ == "__main__":
    main()
