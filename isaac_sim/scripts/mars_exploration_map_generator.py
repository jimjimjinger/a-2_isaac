#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[2]
ISAAC_SIM_DIR = REPO_ROOT / "isaac_sim"
ASSET_DIR = ISAAC_SIM_DIR / "assets"
GENERATED_TERRAINS_DIR = ASSET_DIR / "generated_terrains"
TEXTURE_DIR = ASSET_DIR / "textures" / "Mars"
MARKERS_DIR = ASSET_DIR / "markers"
WORLDS_DIR = ISAAC_SIM_DIR / "worlds"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import world_composer as wc

compose_world = wc.compose_world
export_rocks_usd = wc.export_rocks_usd
export_terrain_usd = wc.export_terrain_usd
populate_live_stage = wc.populate_live_stage
PXR_AVAILABLE = wc.PXR_AVAILABLE


@dataclass
class TerrainConfig:
    terrain_size: Tuple[int, int] = (512, 512)
    resolution_m: float = 1.0
    base_relief_amplitude_m: float = 1.8
    micro_relief_amplitude_m: float = 0.35
    crater_count: int = 40
    hill_count: int = 20
    ridge_count: int = 10
    rock_count: int = 1200
    mineral_count: int = 80
    base_candidate_count: int = 3
    rock_spacing_m: float = 1.5
    slope_threshold_deg: float = 18.0
    candidate_slope_threshold_deg: float = 8.0
    mineral_field_count: int = 4
    mineral_field_radius_m: float = 28.0
    texture_set: str = "Mars"
    terrain_reference_tile_m: float = 128.0


@dataclass
class TerrainResult:
    seed: int
    terrain_id: str
    folder: str
    terrain_dir: Path
    heightmap: np.ndarray
    obstacle_grid: np.ndarray
    rock_records: List[Dict[str, float]]
    mineral_records: List[Dict[str, Any]]
    base_candidates: List[Dict[str, Any]]
    metadata: Dict[str, Any]


def now_iso8601() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def default_terrain_id(seed: int) -> str:
    return f"seed_{seed:06d}"


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def meshgrid_meters(cfg: TerrainConfig) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    h, w = cfg.terrain_size
    res = cfg.resolution_m
    width_m = (w - 1) * res
    height_m = (h - 1) * res
    x = np.linspace(-width_m / 2.0, width_m / 2.0, w, dtype=np.float32)
    y = np.linspace(-height_m / 2.0, height_m / 2.0, h, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)
    return x, y, xx, yy


def bilinear_upsample(coarse: np.ndarray, out_shape: Tuple[int, int]) -> np.ndarray:
    in_h, in_w = coarse.shape
    out_h, out_w = out_shape
    x_old = np.linspace(0.0, 1.0, in_w, dtype=np.float32)
    y_old = np.linspace(0.0, 1.0, in_h, dtype=np.float32)
    x_new = np.linspace(0.0, 1.0, out_w, dtype=np.float32)
    y_new = np.linspace(0.0, 1.0, out_h, dtype=np.float32)

    rows = np.empty((in_h, out_w), dtype=np.float32)
    for i in range(in_h):
        rows[i] = np.interp(x_new, x_old, coarse[i])

    out = np.empty((out_h, out_w), dtype=np.float32)
    for j in range(out_w):
        out[:, j] = np.interp(y_new, y_old, rows[:, j])
    return out


def seed_rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(int(seed))


def add_radial_feature(
    heightmap: np.ndarray,
    xx: np.ndarray,
    yy: np.ndarray,
    center_x: float,
    center_y: float,
    radius_m: float,
    magnitude_m: float,
    rim_ratio: float = 0.18,
    inverted: bool = False,
) -> None:
    dx = xx - center_x
    dy = yy - center_y
    dist = np.sqrt(dx * dx + dy * dy)
    core_sigma = max(radius_m * 0.42, 1.0)
    rim_sigma = max(radius_m * 0.12, 0.8)
    core = np.exp(-(dist * dist) / (2.0 * core_sigma * core_sigma))
    rim = np.exp(-((dist - radius_m * 0.92) ** 2) / (2.0 * rim_sigma * rim_sigma))
    if inverted:
        heightmap -= magnitude_m * core
        heightmap += magnitude_m * rim_ratio * rim
    else:
        heightmap += magnitude_m * core


def add_craters(heightmap: np.ndarray, xx: np.ndarray, yy: np.ndarray, rng: np.random.Generator, count: int) -> None:
    x_min, x_max = float(xx.min()), float(xx.max())
    y_min, y_max = float(yy.min()), float(yy.max())
    for _ in range(count):
        center_x = float(rng.uniform(x_min * 0.8, x_max * 0.8))
        center_y = float(rng.uniform(y_min * 0.8, y_max * 0.8))
        radius_m = float(rng.uniform(10.0, 30.0))
        depth_m = float(rng.uniform(0.5, 2.8))
        add_radial_feature(
            heightmap,
            xx,
            yy,
            center_x,
            center_y,
            radius_m,
            depth_m,
            rim_ratio=float(rng.uniform(0.12, 0.24)),
            inverted=True,
        )


def add_hills(heightmap: np.ndarray, xx: np.ndarray, yy: np.ndarray, rng: np.random.Generator, count: int) -> None:
    x_min, x_max = float(xx.min()), float(xx.max())
    y_min, y_max = float(yy.min()), float(yy.max())
    for _ in range(count):
        center_x = float(rng.uniform(x_min * 0.85, x_max * 0.85))
        center_y = float(rng.uniform(y_min * 0.85, y_max * 0.85))
        radius_m = float(rng.uniform(16.0, 48.0))
        height_m = float(rng.uniform(0.4, 2.2))
        add_radial_feature(heightmap, xx, yy, center_x, center_y, radius_m, height_m)


def add_ridges(heightmap: np.ndarray, xx: np.ndarray, yy: np.ndarray, rng: np.random.Generator, count: int) -> None:
    x_min, x_max = float(xx.min()), float(xx.max())
    y_min, y_max = float(yy.min()), float(yy.max())
    for _ in range(count):
        center_x = float(rng.uniform(x_min * 0.85, x_max * 0.85))
        center_y = float(rng.uniform(y_min * 0.85, y_max * 0.85))
        angle = float(rng.uniform(0.0, math.tau))
        length_m = float(rng.uniform(120.0, 220.0))
        width_m = float(rng.uniform(6.0, 16.0))
        height_m = float(rng.uniform(0.3, 1.4))
        dx = xx - center_x
        dy = yy - center_y
        xr = dx * math.cos(angle) + dy * math.sin(angle)
        yr = -dx * math.sin(angle) + dy * math.cos(angle)
        ridge = np.exp(-(yr * yr) / (2.0 * width_m * width_m))
        ridge *= np.exp(-(xr * xr) / (2.0 * (length_m * 0.34) ** 2))
        ridge *= 1.0 + 0.15 * np.sin(xr / max(length_m * 0.2, 1.0))
        heightmap += height_m * ridge


def build_base_relief(cfg: TerrainConfig, rng: np.random.Generator, xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
    coarse = rng.uniform(-1.0, 1.0, size=(32, 32)).astype(np.float32)
    relief = bilinear_upsample(coarse, cfg.terrain_size)
    relief -= float(relief.mean())
    relief /= max(float(np.abs(relief).max()), 1e-6)
    relief *= cfg.base_relief_amplitude_m

    detail = rng.uniform(-1.0, 1.0, size=(96, 96)).astype(np.float32)
    detail = bilinear_upsample(detail, cfg.terrain_size)
    detail -= float(detail.mean())
    detail /= max(float(np.abs(detail).max()), 1e-6)
    detail *= cfg.micro_relief_amplitude_m

    heightmap = relief + detail
    add_craters(heightmap, xx, yy, rng, cfg.crater_count)
    add_hills(heightmap, xx, yy, rng, cfg.hill_count)
    add_ridges(heightmap, xx, yy, rng, cfg.ridge_count)
    heightmap += 0.15 * np.sin(xx / 64.0) * np.cos(yy / 72.0)
    return heightmap.astype(np.float32)


def compute_slope_deg(heightmap: np.ndarray, resolution_m: float) -> np.ndarray:
    dz_dy, dz_dx = np.gradient(heightmap, resolution_m, resolution_m)
    slope_rad = np.arctan(np.sqrt(dz_dx * dz_dx + dz_dy * dz_dy))
    return np.degrees(slope_rad).astype(np.float32)


def height_lookup(heightmap: np.ndarray, x: float, y: float, x_coords: np.ndarray, y_coords: np.ndarray) -> float:
    xi = int(np.clip(np.searchsorted(x_coords, x), 0, len(x_coords) - 1))
    yi = int(np.clip(np.searchsorted(y_coords, y), 0, len(y_coords) - 1))
    return float(heightmap[yi, xi])


def distance_ok(existing: List[Tuple[float, float]], x: float, y: float, min_spacing: float) -> bool:
    min_spacing_sq = min_spacing * min_spacing
    for ex, ey in existing:
        dx = x - ex
        dy = y - ey
        if dx * dx + dy * dy < min_spacing_sq:
            return False
    return True


def in_rectangles(x: float, y: float, rectangles: Iterable[Dict[str, Any]]) -> bool:
    for rect in rectangles:
        cx = float(rect["center"]["x"])
        cy = float(rect["center"]["y"])
        half_w = float(rect["size_m"][0]) / 2.0
        half_h = float(rect["size_m"][1]) / 2.0
        if abs(x - cx) <= half_w and abs(y - cy) <= half_h:
            return True
    return False


def place_rocks(
    cfg: TerrainConfig,
    rng: np.random.Generator,
    heightmap: np.ndarray,
    slope_deg: np.ndarray,
    obstacle_grid: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    base_candidates: List[Dict[str, Any]],
) -> List[Dict[str, float]]:
    rocks: List[Dict[str, float]] = []
    used_positions: List[Tuple[float, float]] = []
    x_min, x_max = float(x_coords.min()), float(x_coords.max())
    y_min, y_max = float(y_coords.min()), float(y_coords.max())
    max_tries = cfg.rock_count * 40

    for _ in range(max_tries):
        if len(rocks) >= cfg.rock_count:
            break
        x = float(rng.uniform(x_min + 2.0, x_max - 2.0))
        y = float(rng.uniform(y_min + 2.0, y_max - 2.0))
        if in_rectangles(x, y, base_candidates):
            continue
        if height_lookup(slope_deg, x, y, x_coords, y_coords) > 28.0:
            continue
        if height_lookup(obstacle_grid, x, y, x_coords, y_coords) > 0.0:
            continue
        if not distance_ok(used_positions, x, y, cfg.rock_spacing_m):
            continue
        radius = float(rng.uniform(0.18, 1.05))
        rocks.append({"x": x, "y": y, "radius": radius})
        used_positions.append((x, y))
    return rocks


def build_obstacle_grid(
    cfg: TerrainConfig,
    heightmap: np.ndarray,
    slope_deg: np.ndarray,
    rocks: List[Dict[str, float]],
    x_coords: np.ndarray,
    y_coords: np.ndarray,
) -> np.ndarray:
    obstacle = (slope_deg > cfg.slope_threshold_deg).astype(np.uint8)
    for rock in rocks:
        x = float(rock["x"])
        y = float(rock["y"])
        radius = float(rock["radius"])
        xi = int(np.clip(np.searchsorted(x_coords, x), 0, len(x_coords) - 1))
        yi = int(np.clip(np.searchsorted(y_coords, y), 0, len(y_coords) - 1))
        cell_radius = max(1, int(math.ceil(radius / cfg.resolution_m)))
        y0 = max(0, yi - cell_radius)
        y1 = min(obstacle.shape[0], yi + cell_radius + 1)
        x0 = max(0, xi - cell_radius)
        x1 = min(obstacle.shape[1], xi + cell_radius + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (yy - yi) ** 2 + (xx - xi) ** 2 <= cell_radius**2
        obstacle[y0:y1, x0:x1] = np.maximum(obstacle[y0:y1, x0:x1], mask.astype(np.uint8))
    return obstacle


def select_base_candidates(
    cfg: TerrainConfig,
    rng: np.random.Generator,
    obstacle_grid: np.ndarray,
    slope_deg: np.ndarray,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
) -> List[Dict[str, Any]]:
    cell = 32
    candidates: List[Dict[str, Any]] = []
    h, w = obstacle_grid.shape
    for y0 in range(0, h - cell + 1, cell):
        for x0 in range(0, w - cell + 1, cell):
            sub_obstacle = obstacle_grid[y0 : y0 + cell, x0 : x0 + cell]
            sub_slope = slope_deg[y0 : y0 + cell, x0 : x0 + cell]
            open_ratio = float(1.0 - sub_obstacle.mean())
            mean_slope = float(sub_slope.mean())
            score = open_ratio * 0.85 + max(0.0, (cfg.candidate_slope_threshold_deg - mean_slope) / cfg.candidate_slope_threshold_deg) * 0.15
            if open_ratio < 0.92:
                continue
            if mean_slope > cfg.candidate_slope_threshold_deg:
                continue
            cx = float(x_coords[min(x0 + cell // 2, len(x_coords) - 1)])
            cy = float(y_coords[min(y0 + cell // 2, len(y_coords) - 1)])
            candidates.append(
                {
                    "center": {"x": round(cx, 2), "y": round(cy, 2)},
                    "size_m": [float(cell * cfg.resolution_m), float(cell * cfg.resolution_m)],
                    "score": round(score, 3),
                    "mean_slope_deg": round(mean_slope, 2),
                    "open_ratio": round(open_ratio, 3),
                }
            )

    candidates.sort(key=lambda item: item["score"], reverse=True)
    selected: List[Dict[str, Any]] = []
    min_distance_m = 90.0
    for candidate in candidates:
        cx = float(candidate["center"]["x"])
        cy = float(candidate["center"]["y"])
        if any(
            (cx - float(other["center"]["x"])) ** 2 + (cy - float(other["center"]["y"])) ** 2 < min_distance_m**2
            for other in selected
        ):
            continue
        selected.append(candidate)
        if len(selected) >= cfg.base_candidate_count:
            break

    if len(selected) < cfg.base_candidate_count:
        fallback_centers = [
            (-160.0, -160.0),
            (0.0, 0.0),
            (160.0, 160.0),
        ]
        for x, y in fallback_centers:
            if len(selected) >= cfg.base_candidate_count:
                break
            selected.append(
                {
                    "center": {"x": x, "y": y},
                    "size_m": [cell * cfg.resolution_m, cell * cfg.resolution_m],
                    "score": 0.5,
                    "mean_slope_deg": float(height_lookup(slope_deg, x, y, x_coords, y_coords)),
                    "open_ratio": 1.0,
                }
            )
    return selected[: cfg.base_candidate_count]


def place_minerals(
    cfg: TerrainConfig,
    rng: np.random.Generator,
    heightmap: np.ndarray,
    slope_deg: np.ndarray,
    rocks: List[Dict[str, float]],
    base_candidates: List[Dict[str, Any]],
    x_coords: np.ndarray,
    y_coords: np.ndarray,
) -> List[Dict[str, Any]]:
    minerals: List[Dict[str, Any]] = []
    rock_positions = [(r["x"], r["y"]) for r in rocks]
    field_count = cfg.mineral_field_count
    minerals_per_field = max(1, cfg.mineral_count // field_count)
    x_min, x_max = float(x_coords.min()), float(x_coords.max())
    y_min, y_max = float(y_coords.min()), float(y_coords.max())

    field_centers: List[Tuple[float, float]] = []
    for _ in range(field_count * 20):
        if len(field_centers) >= field_count:
            break
        x = float(rng.uniform(x_min * 0.7, x_max * 0.7))
        y = float(rng.uniform(y_min * 0.7, y_max * 0.7))
        if in_rectangles(x, y, base_candidates):
            continue
        if height_lookup(slope_deg, x, y, x_coords, y_coords) > 20.0:
            continue
        if not distance_ok(field_centers, x, y, 120.0):
            continue
        field_centers.append((x, y))

    if not field_centers:
        field_centers = [(-120.0, -120.0), (120.0, -120.0), (-120.0, 120.0), (120.0, 120.0)]

    mineral_id = 1
    for field_index, (cx, cy) in enumerate(field_centers):
        for _ in range(minerals_per_field):
            for _attempt in range(100):
                angle = float(rng.uniform(0.0, math.tau))
                radius = float(abs(rng.normal(0.0, cfg.mineral_field_radius_m * 0.35)))
                x = cx + math.cos(angle) * radius
                y = cy + math.sin(angle) * radius
                if x < x_min + 3.0 or x > x_max - 3.0 or y < y_min + 3.0 or y > y_max - 3.0:
                    continue
                if in_rectangles(x, y, base_candidates):
                    continue
                if any((x - rx) ** 2 + (y - ry) ** 2 < 6.25 for rx, ry in rock_positions):
                    continue
                if height_lookup(slope_deg, x, y, x_coords, y_coords) > 18.0:
                    continue

                roll = float(rng.random())
                if roll < 0.5:
                    mineral_type, value = "blue", 10
                elif roll < 0.8:
                    mineral_type, value = "red", 25
                else:
                    mineral_type, value = "yellow", 50

                z = height_lookup(heightmap, x, y, x_coords, y_coords) + 0.18
                minerals.append(
                    {
                        "id": mineral_id,
                        "field_id": field_index + 1,
                        "type": mineral_type,
                        "value": value,
                        "position": {
                            "x": round(x, 2),
                            "y": round(y, 2),
                            "z": round(z, 2),
                        },
                    }
                )
                mineral_id += 1
                break

    return minerals[: cfg.mineral_count]


def compute_statistics(heightmap: np.ndarray, slope_deg: np.ndarray, obstacle_grid: np.ndarray, rocks: List[Dict[str, float]]) -> Dict[str, Any]:
    flat_ratio = float((obstacle_grid == 0).mean())
    return {
        "height_min_m": round(float(heightmap.min()), 3),
        "height_max_m": round(float(heightmap.max()), 3),
        "height_mean_m": round(float(heightmap.mean()), 3),
        "slope_mean_deg": round(float(slope_deg.mean()), 3),
        "slope_max_deg": round(float(slope_deg.max()), 3),
        "passable_ratio": round(flat_ratio, 3),
        "rock_density_per_m2": round(float(len(rocks)) / float(heightmap.size), 6),
    }


def write_index(index_path: Path, result: TerrainResult) -> None:
    if index_path.exists():
        with index_path.open("r", encoding="utf-8") as f:
            index = json.load(f)
    else:
        index = {"terrains": []}

    normalized: List[Dict[str, Any]] = []
    for item in index.get("terrains", []):
        folder = item.get("folder") or item.get("id")
        if not folder:
            continue
        seed = item.get("seed")
        if seed is None and folder.startswith("seed_"):
            try:
                seed = int(folder.split("_", 1)[1])
            except Exception:
                seed = -1
        normalized.append(
            {
                **item,
                "folder": folder,
                "seed": seed if seed is not None else -1,
                "path": item.get("path", f"assets/generated_terrains/{folder}"),
                "world": item.get("world", "worlds/mars_exploration_world.usd"),
            }
        )

    terrains = [item for item in normalized if item.get("folder") != result.folder]
    terrains.append(
        {
            "seed": result.seed,
            "folder": result.folder,
            "path": f"assets/generated_terrains/{result.folder}",
            "world": "worlds/mars_exploration_world.usd",
            "generated_at": result.metadata["generated_at"],
        }
    )
    index = {"terrains": sorted(terrains, key=lambda item: (item["seed"], item["folder"]))}
    with index_path.open("w", encoding="utf-8") as f:
        json.dump(index, f, indent=2, ensure_ascii=False)


def write_meta(result: TerrainResult, cfg: TerrainConfig) -> None:
    meta = {
        "seed": result.seed,
        "terrain_id": result.terrain_id,
        "folder": result.folder,
        "terrain_size": list(cfg.terrain_size),
        "terrain_resolution_m": cfg.resolution_m,
        "rock_count": len(result.rock_records),
        "crater_count": cfg.crater_count,
        "hill_count": cfg.hill_count,
        "ridge_count": cfg.ridge_count,
        "mineral_count": len(result.mineral_records),
        "base_candidate_count": len(result.base_candidates),
        "texture_set": cfg.texture_set,
        "generated_at": result.metadata["generated_at"],
        "base_candidate_areas": result.base_candidates,
        "minerals": result.mineral_records,
        "rocks": result.rock_records,
        "statistics": result.metadata["statistics"],
    }
    with (result.terrain_dir / "meta.json").open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)


def generate_terrain(seed: int, terrain_id: Optional[str], cfg: TerrainConfig) -> TerrainResult:
    rng = seed_rng(seed)
    x_coords, y_coords, xx, yy = meshgrid_meters(cfg)

    heightmap = build_base_relief(cfg, rng, xx, yy)
    slope_deg = compute_slope_deg(heightmap, cfg.resolution_m)
    obstacle_grid = (slope_deg > cfg.slope_threshold_deg).astype(np.uint8)

    terrain_folder = terrain_id or default_terrain_id(seed)
    terrain_dir = ensure_dir(GENERATED_TERRAINS_DIR / terrain_folder)

    base_candidates = select_base_candidates(cfg, rng, obstacle_grid, slope_deg, x_coords, y_coords)
    rocks = place_rocks(cfg, rng, heightmap, slope_deg, obstacle_grid, x_coords, y_coords, base_candidates)
    obstacle_grid = build_obstacle_grid(cfg, heightmap, slope_deg, rocks, x_coords, y_coords)
    minerals = place_minerals(cfg, rng, heightmap, slope_deg, rocks, base_candidates, x_coords, y_coords)

    stats = compute_statistics(heightmap, slope_deg, obstacle_grid, rocks)
    generated_at = now_iso8601()
    metadata = {"generated_at": generated_at, "statistics": stats}

    result = TerrainResult(
        seed=seed,
        terrain_id=terrain_folder,
        folder=terrain_folder,
        terrain_dir=terrain_dir,
        heightmap=heightmap,
        obstacle_grid=obstacle_grid,
        rock_records=rocks,
        mineral_records=minerals,
        base_candidates=base_candidates,
        metadata=metadata,
    )
    return result


def write_outputs(result: TerrainResult, cfg: TerrainConfig, save_usd: bool) -> None:
    ensure_dir(GENERATED_TERRAINS_DIR)
    np.save(result.terrain_dir / "heightmap.npy", result.heightmap)
    np.save(result.terrain_dir / "obstacle_grid.npy", result.obstacle_grid)

    write_meta(result, cfg)
    write_index(GENERATED_TERRAINS_DIR / "index.json", result)

    if save_usd and PXR_AVAILABLE:
        x_coords, y_coords, _, _ = meshgrid_meters(cfg)
        terrain_usd = result.terrain_dir / "terrain_only.usd"
        rocks_usd = result.terrain_dir / "rocks_merged.usd"
        export_terrain_usd(
            result.heightmap,
            x_coords,
            y_coords,
            terrain_usd,
            TEXTURE_DIR if TEXTURE_DIR.exists() else None,
        )
        export_rocks_usd(
            result.rock_records,
            rocks_usd,
            lambda x, y: height_lookup(result.heightmap, x, y, x_coords, y_coords),
        )
        compose_world(
            WORLDS_DIR / "mars_exploration_world.usd",
            terrain_usd,
            rocks_usd,
            MARKERS_DIR,
            result.mineral_records,
            result.metadata["generated_at"],
        )


def _set_active_camera(camera_path: str) -> None:
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.set_active_camera(camera_path)
    except Exception:
        pass


def open_live_stage(result: TerrainResult, cfg: TerrainConfig, headless: bool) -> None:
    try:
        from isaacsim import SimulationApp
    except Exception as exc:
        raise RuntimeError(
            "The --open flag requires isaac-python / Isaac Sim runtime. "
            "Run the same command through isaac-python instead of python3."
        ) from exc

    simulation_app = SimulationApp(
        {
            "headless": bool(headless),
            "renderer": "RayTracedLighting",
        }
    )
    try:
        from omni.isaac.core import World
        from omni.isaac.core.utils.stage import get_current_stage

        world = World(stage_units_in_meters=1.0)
        stage = get_current_stage()
        if stage is None:
            raise RuntimeError("Failed to acquire Isaac Sim stage.")

        x_coords, y_coords, _, _ = meshgrid_meters(cfg)
        populate_live_stage(
            stage=stage,
            heightmap=result.heightmap,
            x_coords=x_coords,
            y_coords=y_coords,
            rocks=result.rock_records,
            minerals=result.mineral_records,
            base_candidates=result.base_candidates,
            marker_dir=MARKERS_DIR,
            texture_dir=TEXTURE_DIR if TEXTURE_DIR.exists() else None,
            terrain_height_at=lambda x, y: height_lookup(result.heightmap, x, y, x_coords, y_coords),
        )
        _set_active_camera("/World/Camera")
        world.reset()

        while simulation_app.is_running():
            world.step(render=not headless)
    finally:
        simulation_app.close()


def maybe_launch_isaac_open(world_path: Path, headless: bool) -> None:
    if not PXR_AVAILABLE:
        return
    try:
        from isaacsim import SimulationApp
    except Exception as exc:
        raise RuntimeError(
            "The --open flag requires isaac-python / Isaac Sim runtime. "
            "Run the same command through isaac-python instead of python3."
        ) from exc

    app = SimulationApp({"headless": bool(headless)})
    try:
        from omni.usd import get_context

        get_context().open_stage(str(world_path))
        for _ in range(5):
            app.update()
        if not headless:
            while app.is_running():
                app.update()
    finally:
        app.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mars exploration terrain foundation generator for Isaac Sim."
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--terrain-id", type=str, default=None)
    parser.add_argument("--save-usd", action="store_true")
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = TerrainConfig()
    result = generate_terrain(args.seed, args.terrain_id, cfg)

    write_outputs(result, cfg, save_usd=bool(args.save_usd))

    print(f"[terrain] seed={result.seed} folder={result.folder}")
    print(f"[terrain] heightmap -> {result.terrain_dir / 'heightmap.npy'}")
    print(f"[terrain] obstacle_grid -> {result.terrain_dir / 'obstacle_grid.npy'}")
    print(f"[terrain] meta -> {result.terrain_dir / 'meta.json'}")
    print(f"[index] -> {GENERATED_TERRAINS_DIR / 'index.json'}")

    if args.save_usd and PXR_AVAILABLE:
        print(f"[usd] terrain -> {result.terrain_dir / 'terrain_only.usd'}")
        print(f"[usd] rocks -> {result.terrain_dir / 'rocks_merged.usd'}")
        print(f"[usd] world -> {WORLDS_DIR / 'mars_exploration_world.usd'}")
    elif args.save_usd and not PXR_AVAILABLE:
        print(
            "[usd] pxr not available in this Python runtime. "
            "USD save was skipped. Run the same command with isaac-python "
            "if you need terrain_only.usd, rocks_merged.usd, or mars_exploration_world.usd."
        )
    else:
        print("[usd] save skipped (use --save-usd if you need USD output).")

    if args.open:
        open_live_stage(result, cfg, headless=args.headless)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
