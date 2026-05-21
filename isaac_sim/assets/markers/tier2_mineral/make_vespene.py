#!/usr/bin/env python3
"""베스핀 가스 보주 USD 생성 — mineral_red.usd 대체."""
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

OUT = Path(__file__).parent / "mineral_red.usd"


def make_material(stage, path, base_color, emissive_color, emissive_intensity=0.0):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*base_color))
    shader.CreateInput("roughness",     Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic",      Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("opacity",       Sdf.ValueTypeNames.Float).Set(1.0)
    if emissive_intensity > 0.0:
        ec = Gf.Vec3f(*(c * emissive_intensity for c in emissive_color))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(ec)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def main():
    stage = Usd.Stage.CreateNew(str(OUT))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/VespeneGas")
    stage.SetDefaultPrim(root.GetPrim())

    # ── 머티리얼 — 짙은 초록색, 발광 없음 ────────────────────
    green_mat = make_material(stage, "/VespeneGas/Looks/DarkGreen",
                              base_color=(0.04, 0.28, 0.08),
                              emissive_color=(0, 0, 0))

    # ── 짙은 초록색 큐브 ──────────────────────────────────────
    cube = UsdGeom.Cube.Define(stage, "/VespeneGas/Cube")
    cube.CreateSizeAttr(0.20)
    UsdGeom.XformCommonAPI(cube).SetTranslate((0.0, 0.0, 0.10))
    bind(cube.GetPrim(), green_mat)

    stage.GetRootLayer().Save()
    print(f"저장 완료 → {OUT}")


if __name__ == "__main__":
    main()
