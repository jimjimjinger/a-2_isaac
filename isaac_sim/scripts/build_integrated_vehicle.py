"""통합 로버 차량 v1 빌드 — vehicle_origin_T2.usd 베이스 보정·바스켓 추가.

vehicle_origin_T2.usd (rover + M0609 + RG2-FT 결합본, T2 제작) 를 베이스로:
  · 후방 바스켓 (visual-only, T5 build_rover_m0609_scene.py 에서 이식)
  · Vehicle 원점 재정렬 — T2 원본은 차량이 spawn 좌표(5,0,0.5)에 박혀 있어
    Vehicle 원점이 차량에서 ~5m 떨어짐. 차량을 원점으로 가져온다.
  · MDL 머티리얼 경로 교정 — T2 원본은 rover MDL 을 /home/rokey/... 절대경로로
    참조해 타 환경에서 외형이 깨진다(머티리얼 미로드 → fallback). repo 상대
    경로로 재작성한다.
  · m0609 HOME 자세 — T2 원본은 6축이 전부 0도라 로봇팔이 +Z 로 곧게
    펴진다. HOME pose(0,0,90,0,90,0)로 설정해 팔을 차량 위에 접는다.
을 적용해 vehicle_v1.usd 를 만든다.

순수 pxr(USD) 로만 동작 — Isaac Sim/SimulationApp 불필요.

    python3 isaac_sim/scripts/build_integrated_vehicle.py

⚠️ D455 wrist 카메라는 미포함 — 자산(Nucleus) 확보 후 v1.1 에서 추가.
   휠 freeze·RoverAnchor 등 모드 의존 설정도 미포함(런타임 모드 레이어 담당).
"""
from pathlib import Path
import shutil
import sys

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

# ── 경로 (스크립트 = isaac_sim/scripts/build_integrated_vehicle.py) ──
_ISAAC_SIM = Path(__file__).resolve().parents[1]
_VEHICLE_DIR = _ISAAC_SIM / "assets" / "vehicle"
ORIGIN_USD = _VEHICLE_DIR / "vehicle_origin_T2.usd"
OUTPUT_USD = _VEHICLE_DIR / "vehicle_v1.usd"

# ── vehicle_origin_T2.usd 내부 prim 경로 ──
VEHICLE_ROOT = "/Root/Vehicle"
ROVER_ROOT   = "/Root/Vehicle/rover"
M0609_ROOT   = "/Root/Vehicle/m0609"
GRIPPER_ROOT = "/Root/Vehicle/onrobot_rg2ft"
ROVER_BODY   = "/Root/Vehicle/rover/Body"

# ── 후방 바스켓 부착 대상 + 치수 (T5 build_rover_m0609_scene.py 에서 이식) ──
# rover Body 기준 로컬 좌표. -X = 로봇팔 반대편 = 후방.
BASKET_LOCAL  = (-0.38, 0.0, 0.02)
BASKET_LENGTH = 0.22
BASKET_WIDTH  = 0.46
BASKET_HEIGHT = 0.18
BASKET_WALL   = 0.035
BASKET_BOTTOM = 0.035

# ── m0609 HOME 자세 (T2 pickplace_visual_rover.py 의 HOME_JOINT_POSITIONS_DEG) ──
# joint_3=90·joint_5=90 → 손목이 아래를 보는 '카메라 down' 자세.
# UsdPhysics angular drive/state 단위는 degree.
M0609_HOME_DEG = {
    "joint_1": 0.0, "joint_2": 0.0, "joint_3": 90.0,
    "joint_4": 0.0, "joint_5": 90.0, "joint_6": 0.0,
}


# ── USD 헬퍼 ────────────────────────────────────────────────────────
def _make_preview_material(stage, mat_path, rgb, roughness=0.55, metallic=0.35):
    """UsdPreviewSurface 머티리얼 생성."""
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _define_local_box(stage, path, translation, scale, material=None):
    """size=1 Cube 를 translation/scale 로 배치 (로컬 박스)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    return cube.GetPrim()


def _get_translate_op(prim):
    """prim 의 xformOp:translate op 반환 (없으면 None)."""
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            return op
    return None


# ── 빌드 단계 ───────────────────────────────────────────────────────
def attach_rear_basket(stage, rover_body):
    """로버 Body 뒤쪽에 visual 바스켓 부착 (충돌체 없음)."""
    body_prim = stage.GetPrimAtPath(rover_body)
    if not body_prim.IsValid():
        raise RuntimeError(f"rover Body prim 없음: {rover_body}")

    basket_path = f"{rover_body}/RearBasket"
    basket = stage.DefinePrim(basket_path, "Xform")
    xf = UsdGeom.Xformable(basket)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*BASKET_LOCAL))

    mat = _make_preview_material(stage, f"{basket_path}/Looks/BasketDarkMetal",
                                 rgb=(0.09, 0.075, 0.06), roughness=0.5, metallic=0.45)
    rim_mat = _make_preview_material(stage, f"{basket_path}/Looks/BasketRimMetal",
                                     rgb=(0.45, 0.42, 0.36), roughness=0.35, metallic=0.75)

    lx, wy, hz = BASKET_LENGTH, BASKET_WIDTH, BASKET_HEIGHT
    t, b = BASKET_WALL, BASKET_BOTTOM

    # 바닥 + 벽. 위는 열려 있고 앞쪽(FrontLip)은 트레이처럼 낮다.
    _define_local_box(stage, f"{basket_path}/Bottom",
                      (0.0, 0.0, b * 0.5), (lx, wy, b), mat)
    _define_local_box(stage, f"{basket_path}/LeftWall",
                      (0.0, wy * 0.5 - t * 0.5, hz * 0.5), (lx, t, hz), mat)
    _define_local_box(stage, f"{basket_path}/RightWall",
                      (0.0, -wy * 0.5 + t * 0.5, hz * 0.5), (lx, t, hz), mat)
    _define_local_box(stage, f"{basket_path}/BackWall",
                      (-lx * 0.5 + t * 0.5, 0.0, hz * 0.45), (t, wy, hz * 0.9), mat)
    _define_local_box(stage, f"{basket_path}/FrontLip",
                      (lx * 0.5 - t * 0.5, 0.0, hz * 0.28), (t, wy, hz * 0.56), mat)

    # 밝은 테두리 — 단순 박스가 아니라 바스켓처럼 보이게.
    rim_z = hz + t * 0.5
    _define_local_box(stage, f"{basket_path}/LeftTopRim",
                      (0.0, wy * 0.5, rim_z), (lx + t, t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/RightTopRim",
                      (0.0, -wy * 0.5, rim_z), (lx + t, t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/BackTopRim",
                      (-lx * 0.5, 0.0, rim_z), (t, wy + t, t), rim_mat)
    _define_local_box(stage, f"{basket_path}/FrontTopRim",
                      (lx * 0.5, 0.0, hz * 0.58), (t, wy + t, t), rim_mat)

    n_box = len([c for c in stage.GetPrimAtPath(basket_path).GetChildren()
                 if c.GetName() != "Looks"])
    print(f"  [basket] {basket_path}  (박스 {n_box}개)")
    return basket_path


def recenter_vehicle(stage):
    """Vehicle 원점을 차량(rover)에 맞춘다.

    T2 vehicle_origin_T2.usd 는 빌드 시 차량을 spawn 좌표(rover translate
    (5,0,0.5))에 둔 상태로 저장돼 Vehicle 원점이 차량에서 ~5m 떨어져 있다.
    rover·m0609·onrobot_rg2ft 세 컴포넌트를 rover translate 만큼 동일 시프트
    하면 상대 관계는 그대로 유지되면서 rover 원점이 곧 Vehicle 원점이 된다.
    (세 컴포넌트는 FixedJoint 로 묶여 있는데, 동일 시프트는 body 간 상대
     pose 를 바꾸지 않으므로 joint 가 깨지지 않는다.)
    """
    rover = stage.GetPrimAtPath(ROVER_ROOT)
    if not rover.IsValid():
        print("  [recenter] rover prim 없음 — skip")
        return
    rover_t = _get_translate_op(rover)
    if rover_t is None:
        print("  [recenter] rover translate 없음 — skip")
        return
    off = Gf.Vec3d(rover_t.Get())
    if off == Gf.Vec3d(0, 0, 0):
        print("  [recenter] rover 가 이미 원점 — skip")
        return
    for path in (ROVER_ROOT, M0609_ROOT, GRIPPER_ROOT):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        op = _get_translate_op(prim)
        if op is not None:
            op.Set(Gf.Vec3d(op.Get()) - off)
    print(f"  [recenter] -({off[0]:.3f},{off[1]:.3f},{off[2]:.3f}) 시프트 "
          f"→ Vehicle 원점 = 차량")


def fix_mdl_paths(stage):
    """MDL sourceAsset 의 /home/rokey/... 절대경로를 repo 상대경로로 교정.

    vehicle_origin_T2.usd 는 rover MDL 머티리얼을 T2 환경(/home/rokey/...)
    절대경로로 참조해 타 환경에서 외형이 깨진다(머티리얼 미로드 → fallback).
    .mdl 파일명만 살려 repo 내 실제 위치로 상대경로(../rover/SubUSDs/
    materials/) 재작성한다. OmniPBR.mdl 등 Omniverse 기본 MDL 은 그대로 둔다.
    """
    fixed = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Shader":
            continue
        attr = prim.GetAttribute("info:mdl:sourceAsset")
        if not attr:
            continue
        val = attr.Get()
        if not val:
            continue
        old = val.path
        if "SubUSDs/materials/" in old:
            fname = old.rsplit("/", 1)[-1]
            attr.Set(Sdf.AssetPath(f"../rover/SubUSDs/materials/{fname}"))
            fixed += 1
    print(f"  [mdl] sourceAsset 경로 교정 {fixed}개 → ../rover/SubUSDs/materials/")


def set_m0609_home(stage):
    """m0609 6축 조인트를 HOME 자세로 설정 — drive target + 초기 state 둘 다.

    vehicle_origin_T2.usd 는 6축이 전부 0도라 로봇팔이 +Z 로 곧게 펴진
    상태. T2 가 정한 HOME pose(joint_3=90·joint_5=90, '카메라 down')로
    설정해 팔을 차량 위에 안정적으로 접는다.
    UsdPhysics angular drive/state 단위는 degree.
    """
    m0609 = stage.GetPrimAtPath(M0609_ROOT)
    if not m0609.IsValid():
        print("  [m0609] m0609 prim 없음 — skip")
        return
    done = 0
    for p in Usd.PrimRange(m0609):
        if p.GetTypeName() != "PhysicsRevoluteJoint":
            continue
        deg = M0609_HOME_DEG.get(p.GetName())
        if deg is None:
            continue
        # drive target (시뮬이 유지하려는 목표 각도)
        drive = UsdPhysics.DriveAPI.Get(p, "angular")
        if drive:
            ta = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr()
            ta.Set(float(deg))
        # 초기 state (시작 자세)
        sa = p.GetAttribute("state:angular:physics:position")
        if sa:
            sa.Set(float(deg))
        done += 1
    print(f"  [m0609] HOME 자세 설정 {done}/6 joint  (joint_3=90, joint_5=90)")


def main():
    if not ORIGIN_USD.is_file():
        sys.exit(f"[build] ✗ 원본 USD 없음: {ORIGIN_USD}")

    print(f"[build] 베이스: {ORIGIN_USD.name}")
    shutil.copy(ORIGIN_USD, OUTPUT_USD)
    print(f"[build] 복사 → {OUTPUT_USD.name}")

    stage = Usd.Stage.Open(str(OUTPUT_USD))
    if not stage:
        sys.exit(f"[build] ✗ Stage 열기 실패: {OUTPUT_USD}")

    print("[build] 후방 바스켓 부착 …")
    attach_rear_basket(stage, ROVER_BODY)

    print("[build] Vehicle 원점 재정렬 …")
    recenter_vehicle(stage)

    print("[build] MDL 머티리얼 경로 교정 …")
    fix_mdl_paths(stage)

    print("[build] m0609 HOME 자세 설정 …")
    set_m0609_home(stage)

    stage.GetRootLayer().Save()
    print("[build] ✓ vehicle_v1.usd 저장 완료")


if __name__ == "__main__":
    main()
