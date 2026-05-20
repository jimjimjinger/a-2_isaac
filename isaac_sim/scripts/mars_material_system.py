"""Isaac Sim용 화성 anti-tiling 재질 구성."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from pxr import Sdf, UsdShade

from mars_material_config import TEXTURE_DIR


MARS_ANTI_TILE_MATERIAL_PATH = "/World/Materials/Mars_AntiTile_Soil"


def _ensure_uv_reader(stage, material_path: str):
    st_reader = UsdShade.Shader.Define(stage, f"{material_path}/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    return st_reader


def _add_texture_node(stage, material_path: str, node_name: str, texture_path: Path, color_space: str):
    tex = UsdShade.Shader.Define(stage, f"{material_path}/{node_name}")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture_path)))
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(color_space)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    return tex


def _add_uv_transform(stage, material_path: str, node_name: str, scale: float):
    """텍스처마다 서로 다른 반복률을 주기 위한 UV 변환 노드다."""

    xform = UsdShade.Shader.Define(stage, f"{material_path}/{node_name}")
    xform.CreateIdAttr("UsdTransform2d")
    xform.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set((float(scale), float(scale)))
    xform.CreateInput("rotation", Sdf.ValueTypeNames.Float).Set(0.0)
    xform.CreateInput("translation", Sdf.ValueTypeNames.Float2).Set((0.0, 0.0))
    xform.CreateInput("pivot", Sdf.ValueTypeNames.Float2).Set((0.0, 0.0))
    return xform


def create_anti_tile_mars_material(
    stage,
    texture_dir: Path | str = TEXTURE_DIR,
    texture_paths: Mapping[str, str] | None = None,
):
    """결정론적 베이크 텍스처를 사용해서 화성 재질을 만든다."""

    texture_dir = Path(texture_dir)
    material_path = MARS_ANTI_TILE_MATERIAL_PATH

    paths = {
        "mars_albedo": texture_dir / "mars_albedo.png",
        "mars_normal": texture_dir / "mars_normal.png",
        "mars_roughness": texture_dir / "mars_roughness.png",
        "mars_macro_noise": texture_dir / "mars_macro_noise.png",
        "mars_detail_noise": texture_dir / "mars_detail_noise.png",
        "mars_micro_noise": texture_dir / "mars_micro_noise.png",
        "mars_color_variation": texture_dir / "mars_color_variation.png",
        "mars_crater_mask": texture_dir / "mars_crater_mask.png",
    }

    if texture_paths is not None:
        for key, value in texture_paths.items():
            paths[key] = Path(value)

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.25)

    st_reader = _ensure_uv_reader(stage, material_path)

    # 기본 알베도는 약간만 타일링을 바꿔 읽는다.
    albedo_uv = _add_uv_transform(stage, material_path, "AlbedoUV", 1.15)
    albedo_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    albedo_tex = _add_texture_node(stage, material_path, "AlbedoTexture", paths["mars_albedo"], "sRGB")
    albedo_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(albedo_uv.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(albedo_tex.ConnectableAPI(), "rgb")

    # 거친 면은 더 촘촘한 반복으로 읽어서 작은 노이즈를 살린다.
    rough_uv = _add_uv_transform(stage, material_path, "RoughnessUV", 4.8)
    rough_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    rough_tex = _add_texture_node(stage, material_path, "RoughnessTexture", paths["mars_roughness"], "raw")
    rough_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rough_uv.ConnectableAPI(), "result")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(rough_tex.ConnectableAPI(), "r")

    # 노멀맵은 중간 정도 반복으로 표면 굴곡만 따로 살린다.
    normal_uv = _add_uv_transform(stage, material_path, "NormalUV", 2.2)
    normal_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    normal_tex = _add_texture_node(stage, material_path, "NormalTexture", paths["mars_normal"], "raw")
    normal_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(normal_uv.ConnectableAPI(), "result")
    shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(normal_tex.ConnectableAPI(), "rgb")

    # 보조 노이즈는 베이크 내부에만 남기고, 셰이더는 단순하게 유지한다.
    macro_uv = _add_uv_transform(stage, material_path, "MacroNoiseUV", 0.75)
    macro_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    macro_tex = _add_texture_node(stage, material_path, "MacroNoiseTexture", paths["mars_macro_noise"], "raw")
    macro_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(macro_uv.ConnectableAPI(), "result")

    detail_uv = _add_uv_transform(stage, material_path, "DetailNoiseUV", 6.0)
    detail_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    detail_tex = _add_texture_node(stage, material_path, "DetailNoiseTexture", paths["mars_detail_noise"], "raw")
    detail_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(detail_uv.ConnectableAPI(), "result")

    micro_uv = _add_uv_transform(stage, material_path, "MicroNoiseUV", 14.0)
    micro_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    micro_tex = _add_texture_node(stage, material_path, "MicroNoiseTexture", paths["mars_micro_noise"], "raw")
    micro_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(micro_uv.ConnectableAPI(), "result")

    color_uv = _add_uv_transform(stage, material_path, "ColorVariationUV", 2.6)
    color_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    color_tex = _add_texture_node(stage, material_path, "ColorVariationTexture", paths["mars_color_variation"], "sRGB")
    color_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(color_uv.ConnectableAPI(), "result")
    # 색 변화는 알베도 쪽으로 이미 베이크하므로 별도 하이라이트 연결은 하지 않는다.

    crater_uv = _add_uv_transform(stage, material_path, "CraterMaskUV", 1.25)
    crater_uv.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    crater_tex = _add_texture_node(stage, material_path, "CraterMaskTexture", paths["mars_crater_mask"], "raw")
    crater_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(crater_uv.ConnectableAPI(), "result")

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material
