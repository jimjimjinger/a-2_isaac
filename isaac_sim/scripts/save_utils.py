from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageDraw

try:  # Isaac Sim runtime
    from pxr import Usd  # type: ignore
except Exception:  # pragma: no cover - local dev without Isaac Sim
    Usd = None  # type: ignore


SCRIPT_DIR = Path(__file__).resolve().parent
ISAAC_SIM_ROOT = SCRIPT_DIR.parent
ASSETS_DIR = ISAAC_SIM_ROOT / "assets"
GENERATED_TERRAINS_DIR = ASSETS_DIR / "generated_terrains"
WORLDS_DIR = ISAAC_SIM_ROOT / "worlds"
GENERATED_WORLDS_DIR = WORLDS_DIR / "generated"
TEXTURES_DIR = ASSETS_DIR / "textures"
MINERALS_DIR = ASSETS_DIR / "minerals"


def ensure_dir(path: str | Path) -> Path:
    target = Path(path)
    target.mkdir(parents=True, exist_ok=True)
    return target


def ensure_parent(path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def save_json(data: Mapping[str, Any], path: str | Path) -> Path:
    target = ensure_parent(path)
    with target.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return target


def save_npy(array: np.ndarray, path: str | Path) -> Path:
    target = ensure_parent(path)
    np.save(target, array)
    return target


def _normalize_to_uint8(values: np.ndarray) -> np.ndarray:
    arr = values.astype(np.float32, copy=False)
    minimum = float(arr.min())
    maximum = float(arr.max())
    if maximum <= minimum + 1e-8:
        return np.zeros_like(arr, dtype=np.uint8)
    arr = (arr - minimum) / (maximum - minimum)
    return np.clip(arr * 255.0, 0, 255).astype(np.uint8)


def save_heightmap_png(heightmap: np.ndarray, path: str | Path) -> Path:
    target = ensure_parent(path)
    image = Image.fromarray(_normalize_to_uint8(heightmap), mode="L")
    image.save(target)
    return target


def save_preview_png(
    heightmap: np.ndarray,
    object_layout: Mapping[str, Any],
    path: str | Path,
    *,
    map_size_m: float,
) -> Path:
    target = ensure_parent(path)
    base = _normalize_to_uint8(heightmap)
    rgb = np.stack(
        [
            np.clip(base * 0.95 + 28, 0, 255),
            np.clip(base * 0.55 + 16, 0, 255),
            np.clip(base * 0.34 + 8, 0, 255),
        ],
        axis=-1,
    ).astype(np.uint8)
    resample = getattr(Image, "Resampling", Image).BICUBIC
    image = Image.fromarray(rgb, mode="RGB").resize((1024, 1024), resample)
    draw = ImageDraw.Draw(image)
    half = map_size_m / 2.0
    scale_x = image.width / map_size_m
    scale_y = image.height / map_size_m

    def to_px(x: float, y: float) -> tuple[float, float]:
        px = (x + half) * scale_x
        py = image.height - (y + half) * scale_y
        return float(px), float(py)

    for obj in object_layout.get("objects", []):
        position = obj.get("position", [0.0, 0.0, 0.0])
        if not isinstance(position, Sequence) or len(position) < 2:
            continue
        x, y = float(position[0]), float(position[1])
        px, py = to_px(x, y)
        kind = obj.get("type", "object")
        if kind == "mineral":
            color = (222, 208, 74)
            radius = 5
        elif kind == "rock":
            color = (92, 73, 58)
            radius = 4
        else:
            color = (210, 210, 220)
            radius = 8
        draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=color, outline=(30, 30, 30))

    image.save(target)
    return target


def export_stage(stage: Any, path: str | Path) -> Path:
    # 현재 USD stage를 그대로 디스크에 저장한다.
    if Usd is None:
        raise RuntimeError("pxr.Usd is not available in this environment")
    target = ensure_parent(path)
    stage.GetRootLayer().Export(str(target))
    return target


def write_placeholder_usda(
    path: str | Path,
    *,
    default_prim: str,
    root_name: str,
    body_kind: str = "Cube",
    body_name: str = "Body",
    size: float = 1.0,
    translate_z: float = 0.0,
    color: tuple[float, float, float] = (0.7, 0.7, 0.7),
) -> Path:
    # 실제 에셋이 없을 때 쓸 최소 대체 USD를 만든다.
    target = ensure_parent(path)
    geometry_lines = []
    if body_kind == "Cube":
        geometry_lines.append(f"        float size = {size}")
    elif body_kind == "Sphere":
        geometry_lines.append(f"        float radius = {size * 0.5}")
    elif body_kind == "Cylinder":
        geometry_lines.append(f"        float radius = {size * 0.5}")
        geometry_lines.append(f"        float height = {size}")
    else:
        geometry_lines.append(f"        float size = {size}")
    content = f'''#usda 1.0
(
    defaultPrim = "{default_prim}"
    upAxis = "Z"
    metersPerUnit = 1
)

def Xform "{root_name}"
{{
    def {body_kind} "{body_name}"
    {{
        double3 xformOp:translate = (0, 0, {translate_z})
{os.linesep.join(geometry_lines)}
        color3f primvars:displayColor = ({color[0]}, {color[1]}, {color[2]})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
}}
'''
    target.write_text(content, encoding="utf-8")
    return target


def write_placeholder_reference_asset(path: str | Path, label: str, color: tuple[float, float, float]) -> Path:
    # 광물이나 베이스 같은 참조 에셋용 편의 함수다.
    return write_placeholder_usda(
        path,
        default_prim=label,
        root_name=label,
        body_kind="Sphere",
        body_name="Geom",
        size=1.0,
        translate_z=0.0,
        color=color,
    )
