"""Grip 단위 테스트 환경 — vehicle_v3 + 실 시나리오 mineral (blue, 30cm 광물).

terrain 통합과 동일한 USD/physics 패턴으로 mineral spawn:
- tier2_mineral/blue_mineral.usd reference (광물 형태 mesh, T2 YOLO 학습 데이터)
- RigidBody + Collision + convexHull + mass 0.3kg
- /World/Minerals scope 생성 (vehicle_v3 GRASP_SCRIPT 의 검색 경로 일치)

용도:
1. viewport 시각으로 vehicle + mineral 둘 다 보이는지 확인
2. 별도 터미널의 arm_executor + ros2 action 으로 grip 동작 검증
3. mineral world XYZ 주기 출력으로 잡힘/들림 PASS/FAIL 보조 판정

실행: <isaac-python> isaac_sim/scripts/test_grip_unit.py [--mineral-x 0.7] [--headless]
"""
import argparse
import os
import sys

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

_p = argparse.ArgumentParser(description="vehicle_v3 + blue_mineral grip 단위 테스트")
_p.add_argument("--mineral-x", type=float, default=0.7,
                help="mineral world X (rover 정면). default 0.7m")
_p.add_argument("--mineral-y", type=float, default=0.0)
_p.add_argument("--mineral-z", type=float, default=0.2,
                help="mineral spawn z. bbox bottom ≈ -0.17 이라 ground 닿게 ~0.2")
_p.add_argument("--headless", action="store_true")
_a, _ = _p.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": _a.headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM = os.path.dirname(HERE)
V3 = f"{ISAAC_SIM}/assets/vehicle/vehicle_v3.usd"
MINERAL_USD = f"{ISAAC_SIM}/assets/markers/tier2_mineral/blue_mineral.usd"
ROVER_PRIM = "/World/Rover"
MINERAL_PRIM = "/World/Minerals/blue_test"


def _set_translate(stage, prim_path: str, x: float, y: float, z: float) -> None:
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    op = None
    for o in xf.GetOrderedXformOps():
        if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op = o
            break
    if op is None:
        op = xf.AddTranslateOp()
    op.Set(Gf.Vec3d(x, y, z))


def main() -> None:
    for f in (V3, MINERAL_USD):
        if not os.path.isfile(f):
            print(f"[grip-unit] ✗ 파일 없음: {f}")
            app.close()
            sys.exit(1)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    GroundPlane(prim_path="/World/GroundPlane", z_position=0.0,
                color=np.array([0.4, 0.35, 0.3]))

    # DistantLight — viewport 어둠 방지 (terrain 통합에선 terrain 가 light 가짐)
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/SunLight"))
    sun.CreateIntensityAttr().Set(3000.0)
    sun.CreateAngleAttr().Set(0.53)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

    # vehicle_v3 spawn @ origin + z=0.2 (휠 ground 닿게)
    add_reference_to_stage(usd_path=V3, prim_path=ROVER_PRIM)
    _set_translate(stage, ROVER_PRIM, 0.0, 0.0, 0.2)

    # /World/Minerals scope — vehicle_v3 GRASP_SCRIPT 가 여기서 검색
    UsdGeom.Xform.Define(stage, "/World/Minerals")

    # Blue mineral spawn — terrain_generator 의 _define_translated_reference
    # 패턴 그대로 채택. mineral USD 의 default prim 이 자체 orient/scale
    # xformOps 를 갖고 있어 직접 reference + translate 하면 회전/크기 깨짐
    # → wrapper Xform + 자식 Reference Xform 으로 분리해 mineral 자체 transform 보존.
    wrapper = UsdGeom.Xform.Define(stage, MINERAL_PRIM)
    UsdGeom.XformCommonAPI(wrapper.GetPrim()).SetTranslate(
        Gf.Vec3d(_a.mineral_x, _a.mineral_y, _a.mineral_z))
    ref = UsdGeom.Xform.Define(stage, f"{MINERAL_PRIM}/Reference")
    ref.GetPrim().GetReferences().AddReference(MINERAL_USD)

    # Physics — terrain_generator 와 동일하게 wrapper prim 에 적용.
    m_prim = wrapper.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(m_prim)
    UsdPhysics.CollisionAPI.Apply(m_prim)
    UsdPhysics.MeshCollisionAPI.Apply(m_prim).CreateApproximationAttr().Set("convexHull")
    UsdPhysics.MassAPI.Apply(m_prim).CreateMassAttr().Set(0.3)

    print(f"[grip-unit] vehicle_v3 @ (0, 0, 0.2)")
    print(f"[grip-unit] blue_mineral @ "
          f"({_a.mineral_x}, {_a.mineral_y}, {_a.mineral_z})")
    print(f"[grip-unit] pickup distance from rover origin = "
          f"{(_a.mineral_x**2 + _a.mineral_y**2)**0.5:.2f} m")

    for _ in range(30):
        app.update()
    world.reset()
    world.play()
    print()
    print("[grip-unit] ready — viewport 에서 vehicle + blue mineral 시각 확인")
    print("[grip-unit] 별도 터미널 (grip 시도):")
    print("  source /opt/ros/humble/setup.bash && \\")
    print("    source ~/dev_ws/rover_ws/install/setup.bash")
    print("  ros2 run isaac_manipulation arm_executor_node &")
    print(f"  ros2 action send_goal /execute_arm_task "
          f"isaac_interfaces/action/ExecuteArmTask \\")
    print(f"      \"{{command: 'pick_mineral', target_id: 'test', "
          f"target_x: {_a.mineral_x}, target_y: {_a.mineral_y}, "
          f"target_z: {_a.mineral_z}}}\"")

    step = 0
    try:
        while app.is_running():
            world.step(render=True)
            if step % 300 == 0:
                m = stage.GetPrimAtPath(MINERAL_PRIM)
                if m.IsValid():
                    M = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(0)
                    t = M.ExtractTranslation()
                    print(f"[grip-unit] step {step:5d} mineral world=("
                          f"{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
