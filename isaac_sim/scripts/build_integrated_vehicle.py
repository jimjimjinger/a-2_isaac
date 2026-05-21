"""통합 로버 차량 v1 빌드 — vehicle_origin_T2.usd 베이스에 후방 바스켓 추가.

vehicle_origin_T2.usd (rover + M0609 + RG2-FT 결합본, T2 제작) 를 베이스로
후방 바스켓(visual-only)을 더해 vehicle_v1.usd 를 만든다.

순수 pxr(USD) 로만 동작 — Isaac Sim/SimulationApp 불필요.

    python3 isaac_sim/scripts/build_integrated_vehicle.py

참고:
  · 외형 dark 색칠은 vehicle_origin_T2.usd 에 이미 적용돼 있음(`T2DarkBody`
    머티리얼) — 별도 색칠 단계 불필요.
  · D455 wrist 카메라는 미포함 — 자산(Nucleus) 확보 후 v1.1 에서 추가.
  · 휠 freeze·RoverAnchor 등 모드 의존 설정은 의도적으로 USD 에 넣지 않음
    (런타임 모드 레이어가 담당).
"""
from pathlib import Path
import shutil
import sys

from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf

# ── 경로 (스크립트 = isaac_sim/scripts/build_integrated_vehicle.py) ──
_ISAAC_SIM = Path(__file__).resolve().parents[1]
_VEHICLE_DIR = _ISAAC_SIM / "assets" / "vehicle"
ORIGIN_USD = _VEHICLE_DIR / "vehicle_origin_T2.usd"
OUTPUT_USD = _VEHICLE_DIR / "vehicle_v1.usd"

# ── 후방 바스켓 부착 대상 + 치수 (T5 build_rover_m0609_scene.py 에서 이식) ──
ROVER_BODY = "/Root/Vehicle/rover/Body"
# rover Body 기준 로컬 좌표. -X = 로봇팔 반대편 = 후방.
BASKET_LOCAL  = (-0.38, 0.0, 0.02)
BASKET_LENGTH = 0.22
BASKET_WIDTH  = 0.46
BASKET_HEIGHT = 0.18
BASKET_WALL   = 0.035
BASKET_BOTTOM = 0.035


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

    stage.GetRootLayer().Save()
    print("[build] ✓ vehicle_v1.usd 저장 완료")


if __name__ == "__main__":
    main()
