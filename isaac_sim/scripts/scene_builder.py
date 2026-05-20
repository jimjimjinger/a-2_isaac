from __future__ import annotations

import os
import random
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from base_generator import build_base_layout
from camera_lighting import setup_camera, setup_lights
from material_generator import build_material_info, ensure_mars_textures
from mineral_generator import build_object_layout, generate_mineral_layout, generate_rock_layout
from mars_material_config import TEXTURE_DIR
from mars_material_system import create_anti_tile_mars_material
from save_utils import (
    ASSETS_DIR,
    GENERATED_TERRAINS_DIR,
    GENERATED_WORLDS_DIR,
    ISAAC_SIM_ROOT,
    ensure_dir,
    export_stage,
    save_heightmap_png,
    save_json,
    save_npy,
    save_preview_png,
    write_placeholder_reference_asset,
)
from terrain_generator import (
    author_terrain_stage,
    build_obstacle_grid,
    build_terrain_mesh_data,
    compute_difficulty,
    compute_slope_deg,
    generate_heightmap,
)

try:
    from pxr import Gf, Usd, UsdGeom, UsdShade  # type: ignore
except Exception:  # pragma: no cover
    Gf = Usd = UsdGeom = UsdShade = None  # type: ignore


MAP_SIZE = 1000
MAP_SIZE_M = 100.0
HEIGHT_SCALE = 8.0
RESOLUTION_M = MAP_SIZE_M / MAP_SIZE
MESH_STRIDE = 5


def _relative_path(target: Path, start: Path) -> str:
    return Path(os.path.relpath(target, start=start)).as_posix()


def _ensure_static_assets() -> None:
    # 새 체크아웃에서도 export가 되도록 대체 에셋을 만든다.
    ensure_dir(TEXTURE_DIR)
    ensure_dir(ASSETS_DIR / "minerals")
    ensure_dir(GENERATED_TERRAINS_DIR)
    ensure_dir(GENERATED_WORLDS_DIR)
    if not (ASSETS_DIR / "command_center.usd").exists():
        write_placeholder_reference_asset(ASSETS_DIR / "command_center.usd", "CommandCenter", (0.72, 0.74, 0.78))
    if not (ASSETS_DIR / "bunker.usd").exists():
        write_placeholder_reference_asset(ASSETS_DIR / "bunker.usd", "Bunker", (0.50, 0.54, 0.58))
    mineral_asset = ASSETS_DIR / "minerals" / "mineral.usd"
    mineral_scene = ASSETS_DIR / "minerals" / "minerals.usdc"
    if not mineral_asset.exists():
        if mineral_scene.exists():
            mineral_asset.write_text(
                """#usda 1.0
(
    defaultPrim = "Mineral"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Mineral"
(
    prepend references = @minerals.usdc@
    prepend apiSchemas = ["CollectionAPI:allMeshes", "MaterialBindingAPI"]
)
{
    double3 xformOp:rotateXYZ = (90, 0, 0)
    double3 xformOp:scale = (0.01, 0.01, 0.01)
    uniform token[] xformOpOrder = ["xformOp:rotateXYZ", "xformOp:scale"]
    rel material:binding = </Mineral/Looks/MineralMaterial>
    uniform token collection:allMeshes:expansionRule = "expandPrims"
    rel collection:allMeshes:includes = </Mineral/Meshes>
    rel material:binding:collection:allMeshes = [</Mineral/Looks/MineralMaterial>, </Mineral.collection:allMeshes>]

    def Scope "Looks"
    {
        def Material "MineralMaterial"
        {
            token outputs:surface.connect = </Mineral/Looks/PreviewSurface.outputs:surface>

            def Shader "PreviewSurface"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor.connect = </Mineral/Looks/BaseColor.outputs:rgb>
                color3f inputs:emissiveColor.connect = </Mineral/Looks/Emissive.outputs:rgb>
                float inputs:metallic.connect = </Mineral/Looks/Metallic.outputs:r>
                float inputs:roughness.connect = </Mineral/Looks/Roughness.outputs:r>
                normal3f inputs:normal.connect = </Mineral/Looks/Normal.outputs:rgb>
            }

            def Shader "UVReader"
            {
                uniform token info:id = "UsdPrimvarReader_float2"
                token inputs:varname = "uvset0"
                float2 outputs:result
            }

            def Shader "BaseColor"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @Default_OBJ_baseColor.jpg@
                token inputs:sourceColorSpace = "sRGB"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </Mineral/Looks/UVReader.outputs:result>
                float3 outputs:rgb
            }

            def Shader "Emissive"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @Default_OBJ_emissive.jpg@
                token inputs:sourceColorSpace = "sRGB"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </Mineral/Looks/UVReader.outputs:result>
                float3 outputs:rgb
            }

            def Shader "Roughness"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @Default_OBJ_metallicRoughness_rough.jpg@
                token inputs:sourceColorSpace = "raw"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </Mineral/Looks/UVReader.outputs:result>
                float outputs:r
            }

            def Shader "Metallic"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @Default_OBJ_metallicRoughness_metal.jpg@
                token inputs:sourceColorSpace = "raw"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </Mineral/Looks/UVReader.outputs:result>
                float outputs:r
            }

            def Shader "Normal"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @Default_OBJ_normal.jpg@
                token inputs:sourceColorSpace = "raw"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float2 inputs:st.connect = </Mineral/Looks/UVReader.outputs:result>
                float3 outputs:rgb
            }
        }
    }
}
""",
                encoding="utf-8",
            )
        else:
            write_placeholder_reference_asset(mineral_asset, "Mineral", (0.92, 0.87, 0.38))


def _build_world_stage(
    *,
    world_path: Path,
    terrain_usd_path: Path,
    terrain_dir: Path,
    object_layout: dict[str, Any],
    terrain_id: str,
    seed: int,
    map_size_m: float,
) -> Any:
    if Usd is None or UsdGeom is None:
        return None

    # 지형을 참조하고 오브젝트를 배치해서 최종 월드를 조립한다.
    stage = Usd.Stage.CreateNew(str(world_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if hasattr(stage, "RemovePrim"):
        stage.RemovePrim("/World/defaultGroundPlane")

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    terrain_prim = stage.DefinePrim("/World/Terrain", "Xform")
    terrain_prim.GetReferences().AddReference(_relative_path(terrain_usd_path, world_path.parent))

    base_root = stage.DefinePrim("/World/Base", "Xform")
    minerals_root = stage.DefinePrim("/World/Minerals", "Xform")
    rocks_root = stage.DefinePrim("/World/Rocks", "Xform")
    del base_root, minerals_root, rocks_root

    for obj in object_layout["objects"]:
        prim_path = obj["prim_path"]
        asset_path = obj["asset_path"]
        prim = stage.DefinePrim(prim_path, "Xform")
        if asset_path.startswith("procedural:"):
            if obj["type"] == "rock":
                rock_geom = UsdGeom.Sphere.Define(stage, f"{prim_path}/Geom")
                rock_geom.GetRadiusAttr().Set(0.5)
                rock_geom.GetDisplayColorAttr().Set([Gf.Vec3f(0.45, 0.30, 0.25)])
        else:
            prim.GetReferences().AddReference(
                _relative_path((ISAAC_SIM_ROOT / asset_path).resolve(), world_path.parent)
            )
        position = obj.get("position", [0.0, 0.0, 0.0])
        rotation = obj.get("rotation", [0.0, 0.0, 0.0])
        scale = obj.get("scale", [1.0, 1.0, 1.0])
        xform = UsdGeom.XformCommonAPI(prim)
        xform.SetTranslate((float(position[0]), float(position[1]), float(position[2])))
        xform.SetRotate((float(rotation[0]), float(rotation[1]), float(rotation[2])))
        xform.SetScale((float(scale[0]), float(scale[1]), float(scale[2])))

    setup_lights(stage)
    setup_camera(stage, map_size_m=map_size_m, seed=seed)

    stage.GetRootLayer().customLayerData = {
        "terrain_id": terrain_id,
        "seed": int(seed),
    }
    return stage


def build_mars_scene(
    seed: int = 1,
    *,
    terrain_id: str | None = None,
    overwrite: bool = True,
) -> dict[str, Any]:
    # 하나의 seed로 지형, 오브젝트, 텍스처, 메타데이터를 모두 만든다.
    _ensure_static_assets()
    random.seed(seed)
    np.random.seed(seed)

    terrain_id = terrain_id or f"terrain_{seed:05d}"
    terrain_dir = GENERATED_TERRAINS_DIR / terrain_id
    world_path = GENERATED_WORLDS_DIR / f"mars_seed_{seed:05d}.usd"

    if terrain_dir.exists() and not overwrite:
        return {
            "terrain_dir": terrain_dir,
            "world_path": world_path,
            "terrain_id": terrain_id,
            "seed": seed,
            "skipped": True,
        }

    ensure_dir(terrain_dir)
    ensure_dir(world_path.parent)

    heightmap = generate_heightmap(seed, map_size=MAP_SIZE, height_scale=HEIGHT_SCALE)
    slope_deg = compute_slope_deg(heightmap, RESOLUTION_M)

    rocks = generate_rock_layout(
        seed=seed,
        slope_deg=slope_deg,
        heightmap=heightmap,
        origin=(-MAP_SIZE_M / 2.0, -MAP_SIZE_M / 2.0),
        resolution_m=RESOLUTION_M,
        map_size_m=MAP_SIZE_M,
        count=80,
        size_range_m=(0.3, 1.5),
        min_spacing_m=1.0,
        slope_threshold_deg=25.0,
        basecamp_center=(0.0, 0.0),
        basecamp_radius_m=3.0,
    )
    minerals = generate_mineral_layout(
        seed=seed,
        heightmap=heightmap,
        rocks=rocks,
        origin=(-MAP_SIZE_M / 2.0, -MAP_SIZE_M / 2.0),
        resolution_m=RESOLUTION_M,
        map_size_m=MAP_SIZE_M,
        count=12,
        min_spacing_m=3.0,
        exclude_basecamp_radius_m=5.0,
        basecamp_center=(0.0, 0.0),
    )
    base_objects = build_base_layout(seed=seed, terrain_size_m=MAP_SIZE_M)
    object_layout = build_object_layout(
        seed=seed,
        terrain_id=terrain_id,
        rocks=rocks,
        minerals=minerals,
        base_objects=base_objects,
    )

    obstacle_grid = build_obstacle_grid(
        slope_deg,
        rocks,
        origin=(-MAP_SIZE_M / 2.0, -MAP_SIZE_M / 2.0),
        resolution_m=RESOLUTION_M,
        slope_thr_deg=25.0,
    )
    difficulty = compute_difficulty(slope_deg, obstacle_grid, rocks, MAP_SIZE_M, RESOLUTION_M)

    textures = ensure_mars_textures(texture_dir=TEXTURE_DIR, seed=seed, overwrite=overwrite)
    material_info = build_material_info(seed, terrain_id, textures)

    terrain_mesh = build_terrain_mesh_data(
        heightmap,
        origin=(-MAP_SIZE_M / 2.0, -MAP_SIZE_M / 2.0),
        resolution_m=RESOLUTION_M,
        mesh_stride=MESH_STRIDE,
    )

    terrain_usd_path = terrain_dir / "terrain_only.usd"
    if Usd is not None and UsdGeom is not None:
        # 지형 메시를 먼저 재사용 가능한 USD로 저장한다.
        terrain_stage = Usd.Stage.CreateNew(str(terrain_usd_path))
        UsdGeom.SetStageUpAxis(terrain_stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(terrain_stage, 1.0)
        material = create_anti_tile_mars_material(
            terrain_stage,
            texture_dir=TEXTURE_DIR,
            texture_paths=textures,
        )
        terrain_mesh_prim = author_terrain_stage(
            terrain_stage,
            terrain_mesh,
            texture_paths=textures,
        )
        if UsdShade is not None:
            UsdShade.MaterialBindingAPI.Apply(terrain_mesh_prim.GetPrim()).Bind(material)
        export_stage(terrain_stage, terrain_usd_path)
    else:
        terrain_usd_path.write_text("#usda 1.0\n", encoding="utf-8")

    if Usd is not None and UsdGeom is not None:
        # 지형 에셋이 저장된 뒤 마스터 씬을 export한다.
        world_stage = _build_world_stage(
            world_path=world_path,
            terrain_usd_path=terrain_usd_path,
            terrain_dir=terrain_dir,
            object_layout=object_layout,
            terrain_id=terrain_id,
            seed=seed,
            map_size_m=MAP_SIZE_M,
        )
        if world_stage is not None:
            export_stage(world_stage, world_path)
    else:
        world_path.write_text("#usda 1.0\n", encoding="utf-8")

    heightmap_png = save_heightmap_png(heightmap, terrain_dir / "heightmap.png")
    preview_png = save_preview_png(heightmap, object_layout, terrain_dir / "preview.png", map_size_m=MAP_SIZE_M)
    save_npy(heightmap, terrain_dir / "heightmap.npy")
    save_npy(obstacle_grid, terrain_dir / "obstacle_grid.npy")

    index_path = GENERATED_TERRAINS_DIR / "index.json"
    if index_path.exists():
        import json

        index = json.loads(index_path.read_text(encoding="utf-8"))
    else:
        index = {
            "version": "1.0",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "terrains": [],
        }
    index["terrains"] = [entry for entry in index.get("terrains", []) if entry.get("id") != terrain_id]
    index["terrains"].append(
        {
            "id": terrain_id,
            "seed": int(seed),
            "split": "train",
            "difficulty": difficulty["score"],
        }
    )
    save_json(index, index_path)

    terrain_info = {
        "seed": int(seed),
        "terrain_id": terrain_id,
        "usd_path": _relative_path(world_path, terrain_dir),
        "world_path": _relative_path(world_path, terrain_dir),
        "map_size": MAP_SIZE,
        "map_size_m": MAP_SIZE_M,
        "resolution_m": RESOLUTION_M,
        "height_scale": HEIGHT_SCALE,
        "heightmap_npy": "heightmap.npy",
        "heightmap_png": "heightmap.png",
        "object_layout": "object_layout.json",
        "material_info": "material_info.json",
        "preview_png": "preview.png",
        "obstacle_grid_npy": "obstacle_grid.npy",
        "difficulty": difficulty,
    }

    object_layout["seed"] = int(seed)
    object_layout["terrain_id"] = terrain_id

    save_json(terrain_info, terrain_dir / "terrain_info.json")
    save_json(object_layout, terrain_dir / "object_layout.json")
    save_json(material_info, terrain_dir / "material_info.json")

    return {
        "terrain_id": terrain_id,
        "seed": seed,
        "terrain_dir": terrain_dir,
        "world_path": world_path,
        "terrain_usd_path": terrain_usd_path,
        "terrain_info": terrain_info,
        "object_layout": object_layout,
        "material_info": material_info,
        "textures": textures,
        "preview_png": preview_png,
        "heightmap_png": heightmap_png,
    }
