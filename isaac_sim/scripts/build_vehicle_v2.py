"""vehicle_v2 빌드 — vehicle_v1.usd 에 후방·하단 밸러스트 추가.

v1(주행게인 USD default·m0609 HOME 베이크 완료본)을 복사해 rover Body
후방(-X)·하단(-Z)에 counterweight rigid body 를 FixedJoint 로 부착한다.
두산 팔(차량 질량 44%)이 위에 얹히며 무게중심이 접지면 위 0.48m·전방
0.10m 로 쏠린 것을 보정 — top-heavy / 전방 전복 경향 완화.

밸러스트는 별도 rigid body 라 PhysX 가 관성(parallel-axis 포함)을 정확히
계산한다. 시각 박스로 보이고, 질량·위치는 아래 파라미터로 튜닝.

순수 pxr (Isaac Sim 불필요):
    python3 isaac_sim/scripts/build_vehicle_v2.py
"""
import shutil
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

_VEH = Path(__file__).resolve().parents[1] / "assets" / "vehicle"
V1 = str(_VEH / "vehicle_v1.usd")
V2 = str(_VEH / "vehicle_v2.usd")

ROVER   = "/Root/Vehicle/rover"
BODY    = f"{ROVER}/Body"
BALLAST = f"{ROVER}/Ballast"

# ── 밸러스트 파라미터 (rover Body 로컬프레임) ──
BALLAST_MASS  = 12.0                  # kg — 전방쏠림 보정량. 튜닝 대상
BALLAST_LOCAL = (-0.38, 0.0, -0.05)   # 후방(-X)·하단(-Z), 바스켓 영역
BALLAST_SIZE  = (0.24, 0.32, 0.14)    # 시각 박스 치수 (m). 물리는 질량으로


def _box_inertia(m, sx, sy, sz):
    """균질 직육면체의 대각 관성텐서."""
    return Gf.Vec3f(m / 12 * (sy * sy + sz * sz),
                    m / 12 * (sx * sx + sz * sz),
                    m / 12 * (sx * sx + sy * sy))


def main():
    shutil.copy(V1, V2)
    print(f"[v2] 복사: vehicle_v1.usd → vehicle_v2.usd")

    stage = Usd.Stage.Open(V2)
    cache = UsdGeom.XformCache()

    body = stage.GetPrimAtPath(BODY)
    rover = stage.GetPrimAtPath(ROVER)
    if not body.IsValid() or not rover.IsValid():
        raise SystemExit(f"✗ prim 없음: {BODY} 또는 {ROVER}")

    # 밸러스트 변환 — Body 좌표계에서 BALLAST_LOCAL 만큼 평행이동한 위치·자세
    m_off = Gf.Matrix4d(1.0)
    m_off.SetTranslate(Gf.Vec3d(*BALLAST_LOCAL))
    m_ballast_world = m_off * cache.GetLocalToWorldTransform(body)
    m_ballast_local = m_ballast_world * cache.GetLocalToWorldTransform(rover).GetInverse()

    # ── 밸러스트 rigid body ──
    bp = stage.DefinePrim(BALLAST, "Xform")
    xf = UsdGeom.Xformable(bp)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m_ballast_local)

    UsdPhysics.RigidBodyAPI.Apply(bp)
    mass_api = UsdPhysics.MassAPI.Apply(bp)
    mass_api.CreateMassAttr(BALLAST_MASS)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    mass_api.CreateDiagonalInertiaAttr(_box_inertia(BALLAST_MASS, *BALLAST_SIZE))

    # 시각 박스 (counterweight 가 씬에서 보이도록 — 어두운 색)
    geom = UsdGeom.Cube.Define(stage, f"{BALLAST}/Geom")
    geom.CreateSizeAttr(1.0)
    gxf = UsdGeom.Xformable(geom.GetPrim())
    gxf.ClearXformOpOrder()
    gxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*BALLAST_SIZE))
    geom.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.10, 0.12)])

    # ── FixedJoint: Body ↔ Ballast (밸러스트를 articulation 링크로 고정) ──
    fj = UsdPhysics.FixedJoint.Define(stage, f"{BODY}/BallastFixedJoint")
    fj.CreateBody0Rel().SetTargets([Sdf.Path(BODY)])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(BALLAST)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*BALLAST_LOCAL))
    fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    stage.GetRootLayer().Save()
    print(f"[v2] 밸러스트 {BALLAST_MASS}kg @ Body로컬 {BALLAST_LOCAL} 부착 완료")
    print(f"[v2] ✓ vehicle_v2.usd 저장")


if __name__ == "__main__":
    main()
