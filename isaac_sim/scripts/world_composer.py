from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
    PXR_AVAILABLE = True
except Exception:  # pragma: no cover - exercised only inside Isaac Sim
    Gf = Sdf = Usd = UsdGeom = UsdLux = UsdShade = None
    PXR_AVAILABLE = False


def require_pxr() -> None:
    if not PXR_AVAILABLE:
        raise RuntimeError(
            "USD export requires Isaac Sim's pxr runtime. "
            "Run the generator with isaac-python, not plain python3, "
            "when you need terrain_only.usd, rocks_merged.usd, or "
            "mars_exploration_world.usd."
        )


def _relpath(target: Path, start: Path) -> str:
    return os.path.relpath(str(Path(target).resolve()), start=str(Path(start).resolve()))


def _relpath_safe(target: Path, start: Path) -> str:
    try:
        return _relpath(target, start)
    except Exception:
        return Path(target).resolve().as_posix()


def _asset_ref(target: Path, base_dir: Optional[Path]) -> str:
    if base_dir is None:
        return Path(target).resolve().as_posix()
    return _relpath_safe(target, base_dir)


def _create_preview_material(
    stage: "Usd.Stage",
    material_path: str,
    texture_dir: Optional[Path],
    uv_scale_m: float,
    terrain_extent_m: tuple[float, float],
    base_dir: Optional[Path],
) -> "UsdShade.Material":
    material = UsdShade.Material.Define(stage, material_path)
    surface_shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
    surface_shader.CreateIdAttr("UsdPreviewSurface")
    surface_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    surface_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surface_shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    if texture_dir is not None:
        albedo = texture_dir / "mars_albedo.png"
        roughness = texture_dir / "mars_roughness.png"
        normal = texture_dir / "mars_normal.png"
        has_textures = albedo.exists() and roughness.exists() and normal.exists()
    else:
        has_textures = False

    if has_textures:
        primvar_reader = UsdShade.Shader.Define(
            stage, f"{material_path}/PrimvarReader"
        )
        primvar_reader.CreateIdAttr("UsdPrimvarReader_float2")
        primvar_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        primvar_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        diffuse_tex = UsdShade.Shader.Define(stage, f"{material_path}/AlbedoTex")
        diffuse_tex.CreateIdAttr("UsdUVTexture")
        diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(albedo, base_dir))
        )
        diffuse_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        diffuse_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        diffuse_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        diffuse_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            primvar_reader, "result"
        )

        rough_tex = UsdShade.Shader.Define(stage, f"{material_path}/RoughnessTex")
        rough_tex.CreateIdAttr("UsdUVTexture")
        rough_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(roughness, base_dir))
        )
        rough_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        rough_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        rough_tex.CreateOutput("r", Sdf.ValueTypeNames.Float)
        rough_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            primvar_reader, "result"
        )

        normal_tex = UsdShade.Shader.Define(stage, f"{material_path}/NormalTex")
        normal_tex.CreateIdAttr("UsdUVTexture")
        normal_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(normal, base_dir))
        )
        normal_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        normal_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        normal_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        normal_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            primvar_reader, "result"
        )

        normal_map = UsdShade.Shader.Define(stage, f"{material_path}/NormalMap")
        normal_map.CreateIdAttr("UsdNormalMap")
        normal_map.CreateInput("in", Sdf.ValueTypeNames.Float3).ConnectToSource(
            normal_tex, "rgb"
        )
        normal_map.CreateOutput("result", Sdf.ValueTypeNames.Normal3f)

        surface_shader.CreateInput(
            "diffuseColor", Sdf.ValueTypeNames.Color3f
        ).ConnectToSource(diffuse_tex, "rgb")
        surface_shader.CreateInput(
            "roughness", Sdf.ValueTypeNames.Float
        ).ConnectToSource(rough_tex, "r")
        surface_shader.CreateInput(
            "normal", Sdf.ValueTypeNames.Normal3f
        ).ConnectToSource(normal_map, "result")
    else:
        surface_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.72, 0.38, 0.22)
        )

    material.CreateSurfaceOutput().ConnectToSource(surface_shader, "surface")
    return material


def _make_uvs(
    xs: "Any",
    ys: "Any",
    x_min: float,
    y_min: float,
    uv_scale_m: float,
) -> list[tuple[float, float]]:
    uvs: list[tuple[float, float]] = []
    for y in ys:
        for x in xs:
            uvs.append(((float(x) - x_min) / uv_scale_m, (float(y) - y_min) / uv_scale_m))
    return uvs


def _configure_stage(stage: "Usd.Stage") -> "UsdGeom.Xform":
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    return world


def build_terrain_prim(
    stage: "Usd.Stage",
    heightmap: "Any",
    x_coords: "Any",
    y_coords: "Any",
    texture_dir: Optional[Path],
    terrain_path: str = "/World/Terrain",
) -> "UsdGeom.Mesh":
    root = UsdGeom.Xform.Define(stage, terrain_path)
    mesh = UsdGeom.Mesh.Define(stage, f"{terrain_path}/TerrainMesh")

    h, w = heightmap.shape
    points = []
    for yi, y in enumerate(y_coords):
        for xi, x in enumerate(x_coords):
            points.append(Gf.Vec3f(float(x), float(y), float(heightmap[yi, xi])))

    face_vertex_indices = []
    face_vertex_counts = []
    for yi in range(h - 1):
        row_start = yi * w
        next_start = (yi + 1) * w
        for xi in range(w - 1):
            v0 = row_start + xi
            v1 = row_start + xi + 1
            v2 = next_start + xi + 1
            v3 = next_start + xi
            face_vertex_indices.extend([v0, v1, v2, v0, v2, v3])
            face_vertex_counts.extend([3, 3])

    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)

    dx = float(x_coords[1] - x_coords[0]) if len(x_coords) > 1 else 1.0
    dy = float(y_coords[1] - y_coords[0]) if len(y_coords) > 1 else 1.0
    dz_dy, dz_dx = __import__("numpy").gradient(heightmap, dy, dx)
    normals = []
    for yi in range(h):
        for xi in range(w):
            nx = -float(dz_dx[yi, xi])
            ny = -float(dz_dy[yi, xi])
            nz = 1.0
            length = (nx * nx + ny * ny + nz * nz) ** 0.5
            normals.append(Gf.Vec3f(nx / length, ny / length, nz / length))
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation("vertex")

    primvar_api = UsdGeom.PrimvarsAPI(mesh)
    st_primvar = primvar_api.CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    x_min = float(x_coords[0])
    y_min = float(y_coords[0])
    uv_scale_m = max(float(x_coords[-1] - x_coords[0]), float(y_coords[-1] - y_coords[0])) / 4.0
    if uv_scale_m <= 0:
        uv_scale_m = 64.0
    st_primvar.Set(_make_uvs(x_coords, y_coords, x_min, y_min, uv_scale_m))

    material_dir = "/World/Terrain/Looks"
    material = _create_preview_material(
        stage,
        f"{material_dir}/MarsSurface",
        texture_dir=texture_dir,
        uv_scale_m=uv_scale_m,
        terrain_extent_m=(float(x_coords[-1] - x_coords[0]), float(y_coords[-1] - y_coords[0])),
        base_dir=Path(stage.GetRootLayer().realPath).parent if stage.GetRootLayer().realPath else None,
    )
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
    return mesh


def build_rocks_prim(
    stage: "Usd.Stage",
    rocks: Iterable[Dict[str, float]],
    terrain_height_at,
    rocks_path: str = "/World/Rocks",
) -> "UsdGeom.Xform":
    root = UsdGeom.Xform.Define(stage, rocks_path)
    for idx, rock in enumerate(rocks):
        x = float(rock["x"])
        y = float(rock["y"])
        radius = float(rock["radius"])
        z = float(terrain_height_at(x, y)) + radius * 0.55
        prim = UsdGeom.Sphere.Define(stage, f"{rocks_path}/rock_{idx:04d}")
        prim.GetRadiusAttr().Set(radius)
        prim.GetDisplayColorAttr().Set([Gf.Vec3f(0.35, 0.28, 0.24)])
        UsdGeom.XformCommonAPI(prim).SetTranslate((x, y, z))
    return root


def build_minerals_prim(
    stage: "Usd.Stage",
    minerals: Iterable[Dict[str, Any]],
    marker_dir: Path,
    minerals_path: str = "/World/Minerals",
) -> "UsdGeom.Xform":
    root = UsdGeom.Xform.Define(stage, minerals_path)
    for mineral in minerals:
        mineral_type = str(mineral["type"])
        marker_path = marker_dir / f"mineral_{mineral_type}.usd"
        if not marker_path.exists():
            continue
        prim_path = f"{minerals_path}/{mineral_type}_{int(mineral['id']):04d}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(
            _asset_ref(marker_path, Path(stage.GetRootLayer().realPath).parent if stage.GetRootLayer().realPath else None)
        )
        position = mineral["position"]
        UsdGeom.XformCommonAPI(prim).SetTranslate(
            (
                float(position["x"]),
                float(position["y"]),
                float(position["z"]),
            )
        )
    return root


def build_base_candidates_prim(
    stage: "Usd.Stage",
    base_candidates: Iterable[Dict[str, Any]],
    base_candidates_path: str = "/World/BaseCandidates",
) -> "UsdGeom.Xform":
    root = UsdGeom.Xform.Define(stage, base_candidates_path)
    for idx, candidate in enumerate(base_candidates):
        center = candidate["center"]
        size_m = candidate["size_m"]
        x = float(center["x"])
        y = float(center["y"])
        size_x = float(size_m[0])
        size_y = float(size_m[1])
        z = 0.15 + idx * 0.02
        prim = UsdGeom.Cube.Define(stage, f"{base_candidates_path}/candidate_{idx:02d}")
        prim.GetSizeAttr().Set(1.0)
        prim.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.5, 0.95)])
        UsdGeom.XformCommonAPI(prim).SetTranslate((x, y, z))
        UsdGeom.XformCommonAPI(prim).SetScale((size_x, size_y, 0.12))
    return root


def build_light_rig(stage: "Usd.Stage") -> None:
    UsdGeom.Xform.Define(stage, "/World/Lights")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate((35.0, 0.0, -25.0))

    sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
    sky.CreateIntensityAttr(150.0)
    sky.CreateColorAttr(Gf.Vec3f(0.95, 0.73, 0.57))


def build_camera(stage: "Usd.Stage") -> "UsdGeom.Camera":
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    UsdGeom.XformCommonAPI(cam).SetTranslate((180.0, -180.0, 140.0))
    UsdGeom.XformCommonAPI(cam).SetRotate((45.0, 0.0, 45.0))
    cam.CreateFocalLengthAttr(35.0)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.2908)
    return cam


def export_terrain_usd(
    heightmap: "Any",
    x_coords: "Any",
    y_coords: "Any",
    out_path: Path,
    texture_dir: Optional[Path],
) -> None:
    require_pxr()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    _configure_stage(stage)
    build_terrain_prim(stage, heightmap, x_coords, y_coords, texture_dir, terrain_path="/Terrain")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Terrain"))
    stage.GetRootLayer().Save()


def export_rocks_usd(
    rocks: Iterable[Dict[str, float]],
    out_path: Path,
    terrain_height_at,
) -> None:
    require_pxr()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    _configure_stage(stage)
    build_rocks_prim(stage, rocks, terrain_height_at, rocks_path="/Rocks")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Rocks"))
    stage.GetRootLayer().Save()


def compose_world(
    world_path: Path,
    terrain_usd: Path,
    rocks_usd: Path,
    marker_dir: Path,
    minerals: Iterable[Dict[str, Any]],
    generated_at: str,
) -> None:
    require_pxr()

    world_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(world_path))
    _configure_stage(stage)

    world_prim = stage.DefinePrim("/World/Terrain", "Xform")
    world_prim.GetReferences().AddReference(_relpath_safe(terrain_usd, world_path.parent))

    rocks_prim = stage.DefinePrim("/World/Rocks", "Xform")
    rocks_prim.GetReferences().AddReference(_relpath_safe(rocks_usd, world_path.parent))

    minerals_root = UsdGeom.Xform.Define(stage, "/World/Minerals")
    for mineral in minerals:
        mineral_type = str(mineral["type"])
        marker_path = marker_dir / f"mineral_{mineral_type}.usd"
        if not marker_path.exists():
            continue
        prim_path = f"/World/Minerals/{mineral_type}_{int(mineral['id']):04d}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(_relpath_safe(marker_path, world_path.parent))
        position = mineral["position"]
        UsdGeom.XformCommonAPI(prim).SetTranslate(
            (
                float(position["x"]),
                float(position["y"]),
                float(position["z"]),
            )
        )

    build_light_rig(stage)
    stage.GetRootLayer().customLayerData = {"generated_at": generated_at}
    stage.GetRootLayer().Save()


def populate_live_stage(
    stage: "Usd.Stage",
    heightmap: "Any",
    x_coords: "Any",
    y_coords: "Any",
    rocks: Iterable[Dict[str, float]],
    minerals: Iterable[Dict[str, Any]],
    base_candidates: Iterable[Dict[str, Any]],
    marker_dir: Path,
    texture_dir: Optional[Path],
    terrain_height_at,
) -> None:
    _configure_stage(stage)
    build_terrain_prim(stage, heightmap, x_coords, y_coords, texture_dir, terrain_path="/World/Terrain")
    build_rocks_prim(stage, rocks, terrain_height_at, rocks_path="/World/Rocks")
    build_minerals_prim(stage, minerals, marker_dir, minerals_path="/World/Minerals")
    build_base_candidates_prim(stage, base_candidates, base_candidates_path="/World/BaseCandidates")
    build_light_rig(stage)
    build_camera(stage)
