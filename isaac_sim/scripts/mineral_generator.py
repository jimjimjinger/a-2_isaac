from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from terrain_generator import sample_at


MINERAL_ASSET_PATH = "assets/minerals/mineral.usd"


def _clamp_bounds(origin: Sequence[float], map_size_m: float, margin_m: float) -> tuple[float, float, float, float]:
    min_x = float(origin[0]) + margin_m
    min_y = float(origin[1]) + margin_m
    max_x = float(origin[0]) + map_size_m - margin_m
    max_y = float(origin[1]) + map_size_m - margin_m
    return min_x, min_y, max_x, max_y


def generate_rock_layout(
    *,
    seed: int,
    slope_deg: np.ndarray,
    heightmap: np.ndarray,
    origin: Sequence[float],
    resolution_m: float,
    map_size_m: float,
    count: int = 80,
    size_range_m: tuple[float, float] = (0.3, 1.5),
    min_spacing_m: float = 1.0,
    slope_threshold_deg: float = 25.0,
    basecamp_center: Sequence[float] = (0.0, 0.0),
    basecamp_radius_m: float = 3.0,
) -> list[dict[str, Any]]:
    # 베이스캠프를 막지 않고 급경사도 피하는 위치에 바위를 놓는다.
    rng = np.random.default_rng(int(seed) + 11)
    min_x, min_y, max_x, max_y = _clamp_bounds(origin, map_size_m, 1.0)
    rocks: list[dict[str, Any]] = []
    max_tries = count * 24
    for _ in range(max_tries):
        if len(rocks) >= count:
            break
        x = float(rng.uniform(min_x, max_x))
        y = float(rng.uniform(min_y, max_y))
        if np.hypot(x - float(basecamp_center[0]), y - float(basecamp_center[1])) < basecamp_radius_m + 1.0:
            continue
        if sample_at(slope_deg, x, y, origin, resolution_m) > slope_threshold_deg:
            continue
        if any(np.hypot(x - rock["position"][0], y - rock["position"][1]) < min_spacing_m for rock in rocks):
            continue
        size = float(rng.uniform(*size_range_m))
        yaw = float(rng.uniform(0.0, 360.0))
        z = sample_at(heightmap, x, y, origin, resolution_m) + size * 0.5
        rocks.append(
            {
                "type": "rock",
                "name": f"rock_{len(rocks) + 1:04d}",
                "prim_path": f"/World/Rocks/rock_{len(rocks) + 1:04d}",
                "asset_path": "procedural:sphere",
                "position": [round(x, 3), round(y, 3), round(z, 3)],
                "rotation": [0.0, 0.0, round(yaw, 3)],
                "scale": [round(size, 3), round(size, 3), round(size, 3)],
                "radius": round(size * 0.5, 3),
            }
        )
    return rocks


def generate_mineral_layout(
    *,
    seed: int,
    heightmap: np.ndarray,
    rocks: Sequence[dict[str, Any]],
    origin: Sequence[float],
    resolution_m: float,
    map_size_m: float,
    count: int = 12,
    min_spacing_m: float = 3.0,
    exclude_basecamp_radius_m: float = 5.0,
    basecamp_center: Sequence[float] = (0.0, 0.0),
) -> list[dict[str, Any]]:
    # 간격과 베이스캠프 제외 조건을 적용해 광물을 흩뿌린다.
    rng = np.random.default_rng(int(seed) + 23)
    min_x, min_y, max_x, max_y = _clamp_bounds(origin, map_size_m, 2.0)
    minerals: list[dict[str, Any]] = []
    max_tries = count * 60
    for _ in range(max_tries):
        if len(minerals) >= count:
            break
        x = float(rng.uniform(min_x, max_x))
        y = float(rng.uniform(min_y, max_y))
        if np.hypot(x - float(basecamp_center[0]), y - float(basecamp_center[1])) < exclude_basecamp_radius_m:
            continue
        if any(np.hypot(x - rock["position"][0], y - rock["position"][1]) < rock["radius"] + 0.5 for rock in rocks):
            continue
        if any(np.hypot(x - mineral["position"][0], y - mineral["position"][1]) < min_spacing_m for mineral in minerals):
            continue
        roll = float(rng.random())
        if roll < 0.5:
            variant = "blue"
            value = 10
        elif roll < 0.8:
            variant = "red"
            value = 25
        else:
            variant = "yellow"
            value = 50
        z = sample_at(heightmap, x, y, origin, resolution_m) + 0.10
        idx = len(minerals) + 1
        minerals.append(
            {
                "type": "mineral",
                "variant": variant,
                "value": value,
                "name": f"mineral_{idx:04d}",
                "prim_path": f"/World/Minerals/mineral_{idx:04d}",
                "asset_path": MINERAL_ASSET_PATH,
                "position": [round(x, 3), round(y, 3), round(z, 3)],
                "rotation": [0.0, 0.0, round(float(rng.uniform(0.0, 360.0)), 3)],
                "scale": [1.0, 1.0, 1.0],
            }
        )
    return minerals


def build_object_layout(
    *,
    seed: int,
    terrain_id: str,
    rocks: Sequence[dict[str, Any]],
    minerals: Sequence[dict[str, Any]],
    base_objects: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    # 생성된 모든 오브젝트를 USD 작성용 레이아웃으로 합친다.
    return {
        "seed": int(seed),
        "terrain_id": terrain_id,
        "objects": [*rocks, *minerals, *base_objects],
    }
