"""usd_physx_setup.py — vehicle_v3.usd PhysX 설정 수정 및 vehicle_v3_physx.usd 생성.

실행 (Isaac Sim 불필요 — pxr만 사용):
  python3 usd_physx_setup.py

수정 내용:
  1. Dynamic rigid body: maxLinearVelocity 확장, CCD 활성화
  2. Mesh collision: Triangle Mesh → convexHull / convexDecomposition 변환
     - 바퀴 (Drive): convexHull  (원형에 가까운 단순 형상)
     - 차체 (Body): convexDecomposition  (복잡한 프레임 형상)
     - Rocker/Boogie: convexDecomposition  (복잡한 링크 형상)
     - Steer: convexHull  (소형 브라켓)
     - M0609 arm links: convexHull  (URDF 변환 링크, 이미 단순 형상)
  3. contactOffset / restOffset 설정
  4. ArticulationRoot solver iteration 강화
  5. 질량 보정 (rover/Body 80 kg 등)
  6. COM -inf 교정

출력: vehicle_v3.usd 와 같은 디렉토리에 vehicle_v3_physx.usd 생성
"""
from __future__ import annotations

import math
import os

from pxr import Gf, Sdf, Usd, UsdPhysics

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR    = os.path.dirname(os.path.abspath(__file__))
_ASSET  = os.path.join(_DIR, "../../../isaac_sim/assets/vehicle")
SRC_USD = os.path.join(_ASSET, "vehicle_v3.usd")
DST_USD = os.path.join(_ASSET, "vehicle_v3_physx.usd")

# ── 기본 설정값 ───────────────────────────────────────────────────────────────
MAX_LIN_VEL    = 100.0    # m/s  — 복원 운동 중 속도 제한 해제
MAX_ANG_VEL    = 5729.58  # rad/s — 기본값 유지
CONTACT_OFFSET = 0.02     # m
REST_OFFSET    = 0.008    # m

# ── 질량 보정 맵 {prim_path_suffix: new_mass_kg} ─────────────────────────────
_MASS_MAP: dict[str, float] = {
    "/rover/Body":         80.0,
    "/rover/FL_Drive":      4.0,
    "/rover/FR_Drive":      4.0,
    "/rover/CL_Drive":      4.0,
    "/rover/CR_Drive":      4.0,
    "/rover/RL_Drive":      4.0,
    "/rover/RR_Drive":      4.0,
    "/rover/FL_Steer":      2.0,
    "/rover/FR_Steer":      2.0,
    "/rover/RL_Steer":      2.0,
    "/rover/RR_Steer":      2.0,
    "/rover/L_Rocker":      5.0,
    "/rover/R_Rocker":      5.0,
    "/rover/L_Boogie":      3.0,
    "/rover/R_Boogie":      3.0,
    "/rover/Differential":  1.0,
    "/rover/Ballast":       5.0,
}

# ── 충돌 근사 타입 매핑 ───────────────────────────────────────────────────────
# Dynamic rigid body 아래 Mesh에 적용할 physics:approximation 토큰
# 키: prim path에 이 문자열이 포함될 때 해당 근사 타입 사용
# 순서 중요: 더 구체적인 패턴이 앞에 와야 함
_COLLISION_APPROX: list[tuple[str, str]] = [
    # 바퀴 — 원형에 가까우므로 convexHull 충분
    ("/FL_Drive/",   "convexHull"),
    ("/FR_Drive/",   "convexHull"),
    ("/CL_Drive/",   "convexHull"),
    ("/CR_Drive/",   "convexHull"),
    ("/RL_Drive/",   "convexHull"),
    ("/RR_Drive/",   "convexHull"),
    # 스티어 브라켓 — 소형 단순 형상
    ("/FL_Steer/",   "convexHull"),
    ("/FR_Steer/",   "convexHull"),
    ("/RL_Steer/",   "convexHull"),
    ("/RR_Steer/",   "convexHull"),
    # 로커/부기 — 복잡한 링크 형상
    ("/L_Rocker/",   "convexDecomposition"),
    ("/R_Rocker/",   "convexDecomposition"),
    ("/L_Boogie/",   "convexDecomposition"),
    ("/R_Boogie/",   "convexDecomposition"),
    # 차체 본체 — 가장 복잡한 형상
    ("/rover/Body/", "convexDecomposition"),
    # M0609 arm links — URDF에서 이미 단순화된 convex mesh
    ("/m0609/",      "convexHull"),
    # onrobot gripper
    ("/onrobot_",    "convexHull"),
]

# convexDecomposition 추가 파라미터
_CDN_MAX_CONVEX_HULLS = 32   # 분해 개수 (복잡도/성능 트레이드오프)
_CDN_MIN_VERTEX      = 8


def _get_or_create_attr(
    prim: Usd.Prim,
    attr_name: str,
    type_name: Sdf.ValueTypeName,
):
    attr = prim.GetAttribute(attr_name)
    if not attr.IsValid():
        attr = prim.CreateAttribute(attr_name, type_name)
    return attr


def fix_rigid_body(prim: Usd.Prim) -> None:
    """RigidBodyAPI prim에 PhysX 속도 제한·CCD·sleep 설정 적용."""
    _get_or_create_attr(
        prim, "physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float
    ).Set(MAX_LIN_VEL)

    _get_or_create_attr(
        prim, "physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool
    ).Set(True)

    _get_or_create_attr(
        prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float
    ).Set(0.005)

    _get_or_create_attr(
        prim, "physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float
    ).Set(0.001)


def fix_contact_report(prim: Usd.Prim) -> None:
    """RigidBodyAPI prim에 contact reporter API를 명시적으로 부착."""
    from pxr import PhysxSchema

    cr_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
    cr_api.CreateThresholdAttr().Set(0.0)


def fix_collision_offsets(prim: Usd.Prim) -> None:
    """CollisionAPI prim에 contactOffset / restOffset 설정."""
    _get_or_create_attr(
        prim, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float
    ).Set(CONTACT_OFFSET)

    _get_or_create_attr(
        prim, "physxCollision:restOffset", Sdf.ValueTypeNames.Float
    ).Set(REST_OFFSET)


def fix_mesh_collision_approx(prim: Usd.Prim, approx: str) -> bool:
    """Mesh collision 근사 타입 설정.

    PhysicsMeshCollisionAPI의 physics:approximation 속성을 변경.
    triangle mesh(기본값 "none" / "meshSimplification")에서
    convexHull 또는 convexDecomposition으로 교체.

    반환값: True면 변경 발생.
    """
    attr = _get_or_create_attr(
        prim, "physics:approximation", Sdf.ValueTypeNames.Token
    )
    current = attr.Get()
    if current == approx:
        return False

    attr.Set(approx)

    # convexDecomposition 추가 파라미터 설정
    if approx == "convexDecomposition":
        _get_or_create_attr(
            prim,
            "physxConvexDecompositionCollision:maxConvexHulls",
            Sdf.ValueTypeNames.UInt,
        ).Set(_CDN_MAX_CONVEX_HULLS)
        _get_or_create_attr(
            prim,
            "physxConvexDecompositionCollision:minThickness",
            Sdf.ValueTypeNames.Float,
        ).Set(0.001)

    return True


def fix_articulation(prim: Usd.Prim) -> None:
    """ArticulationRootAPI solver iteration 강화 + self-collision 활성화."""
    _get_or_create_attr(
        prim,
        "physxArticulation:solverVelocityIterationCount",
        Sdf.ValueTypeNames.UInt,
    ).Set(4)

    # self-collision — rover body와 arm link 간 관통 방지
    sc = prim.GetAttribute("physxArticulation:enabledSelfCollisions")
    if sc.IsValid():
        sc.Set(True)
    else:
        prim.CreateAttribute(
            "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool
        ).Set(True)


def fix_mass(prim: Usd.Prim, new_mass: float) -> None:
    """physics:mass 갱신 + COM -inf 교정."""
    attr = prim.GetAttribute("physics:mass")
    if attr.IsValid():
        attr.Set(new_mass)

    com_attr = prim.GetAttribute("physics:centerOfMass")
    if com_attr.IsValid():
        com = com_attr.Get()
        if com is not None and any(math.isinf(v) for v in com):
            com_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            print(f"    [fix] COM -inf → (0,0,0): {prim.GetPath()}")


def _get_collision_approx(path: str) -> str | None:
    """경로에서 적합한 collision approx 타입을 반환. 매핑 없으면 None."""
    for keyword, approx in _COLLISION_APPROX:
        if keyword in path:
            return approx
    return None


def main() -> None:
    print(f"[usd_physx_setup] 입력: {SRC_USD}")

    stage = Usd.Stage.Open(SRC_USD)
    if not stage:
        raise FileNotFoundError(f"USD 파일 없음: {SRC_USD}")

    rigid_count  = 0
    coll_count   = 0
    approx_count = 0
    art_count    = 0
    mass_count   = 0

    for prim in stage.Traverse():
        schemas  = prim.GetAppliedSchemas()
        path     = prim.GetPath().pathString
        typename = prim.GetTypeName()

        # ── 1. RigidBody 수정 ──────────────────────────────────────────
        if "PhysicsRigidBodyAPI" in schemas:
            fix_rigid_body(prim)
            fix_contact_report(prim)
            rigid_count += 1

        # ── 2. Collision offset 수정 ───────────────────────────────────
        if "PhysicsCollisionAPI" in schemas:
            fix_collision_offsets(prim)
            coll_count += 1

        # ── 3. Mesh collision 근사 타입 변환 ───────────────────────────
        if (
            "PhysicsMeshCollisionAPI" in schemas
            and typename == "Mesh"
        ):
            approx = _get_collision_approx(path)
            if approx is not None:
                changed = fix_mesh_collision_approx(prim, approx)
                if changed:
                    approx_count += 1
                    print(f"  [fix] {approx:26s}: {path}")

        # ── 4. ArticulationRoot 수정 ───────────────────────────────────
        if "PhysicsArticulationRootAPI" in schemas:
            fix_articulation(prim)
            art_count += 1
            print(f"  [fix] ArticulationRoot 강화: {path}")

        # ── 5. 질량 보정 ───────────────────────────────────────────────
        if "PhysicsMassAPI" in schemas:
            # COM -inf 항상 교정
            com_attr = prim.GetAttribute("physics:centerOfMass")
            if com_attr.IsValid():
                com = com_attr.Get()
                if com is not None and any(math.isinf(v) for v in com):
                    com_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
                    print(f"  [fix] COM -inf → (0,0,0): {path}")

            for suffix, new_mass in _MASS_MAP.items():
                if path.endswith(suffix):
                    fix_mass(prim, new_mass)
                    print(f"  [fix] mass → {new_mass:6.1f} kg: {path}")
                    mass_count += 1
                    break

    stage.Export(DST_USD)

    print(f"\n[usd_physx_setup] 완료")
    print(f"  RigidBody     수정: {rigid_count:3d} 개")
    print(f"  Collision     수정: {coll_count:3d} 개")
    print(f"  Mesh approx   수정: {approx_count:3d} 개  (triangle→convex)")
    print(f"  Articulation  수정: {art_count:3d} 개")
    print(f"  Mass          보정: {mass_count:3d} 개")
    print(f"  출력: {DST_USD}")


if __name__ == "__main__":
    main()
