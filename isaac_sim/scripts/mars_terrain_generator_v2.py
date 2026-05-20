#!/usr/bin/env python3
"""
mars_terrain_generator_v2.py — I1 규약 준수 화성 지형 생성기 (v2).

T1(김현중) `mars_exploration_map_generator.py`의 지형 기능(크레이터·언덕·
능선)을 흡수하되, 출력은 I1 지형 규약(docs/interfaces/INTERFACE_CONTRACTS.md,
terrain_meta_schema.json)을 100% 따른다.

v1(`procedural_terrain_generator.py`, Perlin only) 대비:
  - heightmap = 베이스 기복 + 크레이터 + 언덕 + 능선  (T1 엔진 흡수)
  - 피처 크기를 월드 span 비율로 파라미터화 → 50 m 아레나에 맞게 재조준
  - basecamp은 (0,0) 고정 — base-candidate 자동선택은 흡수 안 함
    (소비 트랙 T3/T4가 가정할 수 있는 유일한 '값'이라 팀 영향 0 유지)

I1 출력:
  generated_terrains/terrain_NNNNN/{heightmap.npy, obstacle_grid.npy,
                                    meta.json, terrain_only.usd*,
                                    rocks_merged.usd*}
  generated_terrains/index.json
  (* USD는 pxr/Isaac Sim 런타임 필요 — plain python3 실행 시 건너뜀)

사용:
  python3 isaac_sim/scripts/mars_terrain_generator_v2.py \\
      --seed 23456 --terrain-id terrain_00002 --split train
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[2]
ISAAC_SIM_DIR = REPO_ROOT / "isaac_sim"
GENERATED_DIR = ISAAC_SIM_DIR / "assets" / "generated_terrains"
TEXTURE_DIR = ISAAC_SIM_DIR / "assets" / "textures" / "Mars"
MARKERS_DIR = ISAAC_SIM_DIR / "assets" / "markers"
WORLDS_DIR = ISAAC_SIM_DIR / "worlds"
SCHEMA_PATH = REPO_ROOT / "docs" / "interfaces" / "terrain_meta_schema.json"

# ─── I1 규약 상수 (동결 — 절대 변경 금지. 로버 실측 검증된 값) ───────────
SIZE_M = 50.0                             # 월드 크기 (정사각)
RESOLUTION_M = 0.05                       # heightmap 해상도 → 1000×1000
GRID = int(SIZE_M / RESOLUTION_M)         # 1000
ORIGIN = (-SIZE_M / 2.0, -SIZE_M / 2.0)   # 좌하단 (-25, -25)

# ─── 생성 파라미터 (v2 — 50 m 월드 기준 재조준) ─────────────────────────
# T1 생성기는 ≈511 m 월드 기준이라 크레이터 반경 10~30 m 등 절대값이었음.
# 여기서는 모두 월드 span 비율(frac)로 표현 → 50 m 아레나에 일관되게 축소.
CFG = {
    "base_amp_m": 1.0,            # 베이스 기복 진폭
    "base_octave_cells": 8,
    "detail_amp_m": 0.22,         # 미세 기복
    "detail_octave_cells": 24,
    "crater_count": 8,
    "crater_radius_frac": (0.03, 0.10),   # × 50 m = 1.5~5.0 m
    "crater_depth_m": (0.20, 0.85),
    "hill_count": 5,
    "hill_radius_frac": (0.06, 0.18),     # 3~9 m
    "hill_height_m": (0.25, 0.90),
    "ridge_count": 3,
    "ridge_length_frac": (0.25, 0.55),    # 12.5~27.5 m
    "ridge_width_frac": (0.03, 0.08),     # 1.5~4 m
    "ridge_height_m": (0.20, 0.60),
    "rock_count": 80,
    "rock_size_m": (0.30, 1.00),
    "rock_spacing_m": 1.0,
    "mineral_count": 12,
    "mineral_spacing_m": 3.0,
    "spawn_count": 50,
    "obstacle_slope_deg": 25.0,   # terrain_00001과 동일 (로버 실측 검증된 값)
    "spawn_slope_deg": 15.0,
    "mineral_slope_deg": 18.0,
    "basecamp_center": (0.0, 0.0),  # I1 Tier 1 — (0,0) 고정
    "basecamp_radius": 3.0,
    "mesh_stride": 5,             # USD 시각 메시 다운샘플 (npy는 풀해상도 유지)
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ─── 1. 좌표계 ──────────────────────────────────────────────────────────
def build_meshgrid():
    """I1 규약: grid[i,j] = height at (origin.x + j*res, origin.y + i*res)."""
    xs = ORIGIN[0] + np.arange(GRID, dtype=np.float32) * RESOLUTION_M
    ys = ORIGIN[1] + np.arange(GRID, dtype=np.float32) * RESOLUTION_M
    xx, yy = np.meshgrid(xs, ys)   # xx[i,j]=xs[j], yy[i,j]=ys[i]
    return xs, ys, xx, yy


def world_to_idx(x: float, y: float):
    j = int(np.clip(round((x - ORIGIN[0]) / RESOLUTION_M), 0, GRID - 1))
    i = int(np.clip(round((y - ORIGIN[1]) / RESOLUTION_M), 0, GRID - 1))
    return i, j


def sample(grid: np.ndarray, x: float, y: float) -> float:
    i, j = world_to_idx(x, y)
    return float(grid[i, j])


# ─── 2. Heightmap (T1 엔진 흡수: 베이스 기복 + 크레이터 + 언덕 + 능선) ──
def _bilinear_upsample(coarse: np.ndarray, out_shape) -> np.ndarray:
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


def _add_radial_feature(hm, xx, yy, cx, cy, radius, magnitude,
                        rim_ratio=0.18, inverted=False):
    """크레이터(inverted=True, 림 포함) / 언덕(inverted=False). T1 로직 흡수."""
    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    core_sigma = max(radius * 0.42, 0.5)
    rim_sigma = max(radius * 0.12, 0.3)
    core = np.exp(-(dist ** 2) / (2.0 * core_sigma ** 2))
    rim = np.exp(-((dist - radius * 0.92) ** 2) / (2.0 * rim_sigma ** 2))
    if inverted:
        hm -= magnitude * core
        hm += magnitude * rim_ratio * rim
    else:
        hm += magnitude * core


def _flatten_basecamp(hm, xx, yy):
    """베이스캠프 반경+여유 안을 부드럽게 평탄화 — 피처와 무관히 착륙장 보장."""
    cx, cy = CFG["basecamp_center"]
    r = CFG["basecamp_radius"]
    margin = 2.0
    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    inner = dist <= r
    target = float(hm[inner].mean()) if inner.any() else 0.0
    t = np.clip((r + margin - dist) / margin, 0.0, 1.0)
    w = t * t * (3.0 - 2.0 * t)            # smoothstep
    return hm * (1.0 - w) + target * w


def generate_heightmap(rng, xx, yy) -> np.ndarray:
    span = SIZE_M

    # 베이스 기복 (저주파) + 미세 기복 (고주파)
    coarse = rng.uniform(-1, 1, (CFG["base_octave_cells"],) * 2).astype(np.float32)
    relief = _bilinear_upsample(coarse, (GRID, GRID))
    relief -= relief.mean()
    relief /= max(float(np.abs(relief).max()), 1e-6)
    relief *= CFG["base_amp_m"]

    detail = rng.uniform(-1, 1, (CFG["detail_octave_cells"],) * 2).astype(np.float32)
    detail = _bilinear_upsample(detail, (GRID, GRID))
    detail -= detail.mean()
    detail /= max(float(np.abs(detail).max()), 1e-6)
    detail *= CFG["detail_amp_m"]

    hm = relief + detail

    bcx, bcy = CFG["basecamp_center"]
    keepout = CFG["basecamp_radius"] + 4.0     # 베이스캠프 근처 피처 배제
    edge = span * 0.40                         # 피처 중심은 ±20 m 안

    # 크레이터
    for _ in range(CFG["crater_count"]):
        cx = cy = r = 0.0
        for _try in range(40):
            cx = float(rng.uniform(-edge, edge))
            cy = float(rng.uniform(-edge, edge))
            r = float(rng.uniform(*CFG["crater_radius_frac"])) * span
            if math.hypot(cx - bcx, cy - bcy) > keepout + r:
                break
        depth = float(rng.uniform(*CFG["crater_depth_m"]))
        _add_radial_feature(hm, xx, yy, cx, cy, r, depth,
                            rim_ratio=float(rng.uniform(0.12, 0.22)), inverted=True)

    # 언덕
    for _ in range(CFG["hill_count"]):
        cx = cy = r = 0.0
        for _try in range(40):
            cx = float(rng.uniform(-edge, edge))
            cy = float(rng.uniform(-edge, edge))
            r = float(rng.uniform(*CFG["hill_radius_frac"])) * span
            if math.hypot(cx - bcx, cy - bcy) > keepout + r:
                break
        _add_radial_feature(hm, xx, yy, cx, cy, r,
                            float(rng.uniform(*CFG["hill_height_m"])))

    # 능선
    for _ in range(CFG["ridge_count"]):
        cx = float(rng.uniform(-edge, edge))
        cy = float(rng.uniform(-edge, edge))
        angle = float(rng.uniform(0, math.tau))
        length = float(rng.uniform(*CFG["ridge_length_frac"])) * span
        width = float(rng.uniform(*CFG["ridge_width_frac"])) * span
        height = float(rng.uniform(*CFG["ridge_height_m"]))
        dx, dy = xx - cx, yy - cy
        xr = dx * math.cos(angle) + dy * math.sin(angle)
        yr = -dx * math.sin(angle) + dy * math.cos(angle)
        ridge = np.exp(-(yr ** 2) / (2.0 * width ** 2))
        ridge *= np.exp(-(xr ** 2) / (2.0 * (length * 0.34) ** 2))
        hm += height * ridge

    hm = _flatten_basecamp(hm, xx, yy)
    return hm.astype(np.float32)


# ─── 3. Slope / obstacle ────────────────────────────────────────────────
def compute_slope_deg(hm: np.ndarray) -> np.ndarray:
    dz_dy, dz_dx = np.gradient(hm, RESOLUTION_M, RESOLUTION_M)
    return np.degrees(np.arctan(np.sqrt(dz_dx ** 2 + dz_dy ** 2))).astype(np.float32)


def build_obstacle_grid(slope_deg, rocks) -> np.ndarray:
    """I1: shape (1000,1000), dtype int8, 0=safe 1=obstacle."""
    obstacle = (slope_deg > CFG["obstacle_slope_deg"]).astype(np.int8)
    for rk in rocks:
        i, j = world_to_idx(rk["x"], rk["y"])
        cell_r = max(1, int(math.ceil(rk["radius"] / RESOLUTION_M)))
        i0, i1 = max(0, i - cell_r), min(GRID, i + cell_r + 1)
        j0, j1 = max(0, j - cell_r), min(GRID, j + cell_r + 1)
        ii, jj = np.ogrid[i0:i1, j0:j1]
        mask = (ii - i) ** 2 + (jj - j) ** 2 <= cell_r ** 2
        obstacle[i0:i1, j0:j1] = np.maximum(obstacle[i0:i1, j0:j1],
                                            mask.astype(np.int8))
    return obstacle.astype(np.int8)


# ─── 4. Rocks / minerals / spawns (rejection sampling) ──────────────────
def place_rocks(rng, slope_deg):
    rocks = []
    bcx, bcy = CFG["basecamp_center"]
    excl = CFG["basecamp_radius"] + 1.0
    lo, hi = ORIGIN[0] + 2.0, ORIGIN[0] + SIZE_M - 2.0
    for _ in range(CFG["rock_count"] * 40):
        if len(rocks) >= CFG["rock_count"]:
            break
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if math.hypot(x - bcx, y - bcy) < excl:
            continue
        if sample(slope_deg, x, y) > CFG["obstacle_slope_deg"]:
            continue
        if any(math.hypot(x - r["x"], y - r["y"]) < CFG["rock_spacing_m"]
               for r in rocks):
            continue
        rocks.append({"x": round(x, 3), "y": round(y, 3),
                      "radius": round(float(rng.uniform(*CFG["rock_size_m"])), 3)})
    return rocks


def place_minerals(rng, hm, slope_deg, rocks):
    minerals = []
    bcx, bcy = CFG["basecamp_center"]
    lo, hi = ORIGIN[0] + 2.0, ORIGIN[0] + SIZE_M - 2.0
    next_id = 1
    for _ in range(CFG["mineral_count"] * 60):
        if len(minerals) >= CFG["mineral_count"]:
            break
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if math.hypot(x - bcx, y - bcy) < 5.0:
            continue
        if any(math.hypot(x - r["x"], y - r["y"]) < r["radius"] + 0.5
               for r in rocks):
            continue
        if any(math.hypot(x - m["position"]["x"], y - m["position"]["y"])
               < CFG["mineral_spacing_m"] for m in minerals):
            continue
        if sample(slope_deg, x, y) > CFG["mineral_slope_deg"]:
            continue
        roll = float(rng.random())
        if roll < 0.5:
            mtype, value = "blue", 10
        elif roll < 0.8:
            mtype, value = "red", 25
        else:
            mtype, value = "yellow", 50
        z = sample(hm, x, y) + 0.10
        minerals.append({
            "id": next_id,
            "type": mtype,
            "position": {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)},
            "value": value,
        })
        next_id += 1
    return minerals


def place_spawns(rng, hm, slope_deg, obstacle):
    spawns = []
    bcx, bcy = CFG["basecamp_center"]
    lo, hi = ORIGIN[0] + 2.0, ORIGIN[0] + SIZE_M - 2.0
    for _ in range(CFG["spawn_count"] * 40):
        if len(spawns) >= CFG["spawn_count"]:
            break
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if math.hypot(x - bcx, y - bcy) < CFG["basecamp_radius"] + 0.5:
            continue
        if sample(slope_deg, x, y) > CFG["spawn_slope_deg"]:
            continue
        if sample(obstacle, x, y) > 0.5:
            continue
        spawns.append({
            "x": round(x, 2), "y": round(y, 2),
            "z": round(sample(hm, x, y) + 0.18, 2),
            "yaw": round(float(rng.uniform(0, 2 * math.pi)), 3),
            "group": "default",
        })
    return spawns


# ─── 5. Difficulty ──────────────────────────────────────────────────────
def _longest_corridor_m(obstacle) -> float:
    """200×200로 5×5 max-pool 후 BFS로 최대 자유영역 → 회랑 길이 추정 (scipy 불요)."""
    s = CFG["mesh_stride"]
    small = obstacle.reshape(GRID // s, s, GRID // s, s).max(axis=(1, 3))
    free = small == 0
    seen = np.zeros_like(free, dtype=bool)
    h, w = free.shape
    best = 0
    for si in range(h):
        for sj in range(w):
            if not free[si, sj] or seen[si, sj]:
                continue
            stack = [(si, sj)]
            seen[si, sj] = True
            count = 0
            while stack:
                ci, cj = stack.pop()
                count += 1
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ni, nj = ci + di, cj + dj
                    if 0 <= ni < h and 0 <= nj < w and free[ni, nj] \
                            and not seen[ni, nj]:
                        seen[ni, nj] = True
                        stack.append((ni, nj))
            best = max(best, count)
    return round(math.sqrt(best) * RESOLUTION_M * s, 2)


def compute_difficulty(slope_deg, obstacle, rocks):
    rock_density = len(rocks) / (SIZE_M * SIZE_M)
    mean_slope = float(slope_deg.mean())
    passable = float((obstacle == 0).mean())
    score = float(np.clip(
        0.3 * (rock_density / 0.03)
        + 0.4 * (mean_slope / 30.0)
        + 0.3 * (1.0 - passable), 0.0, 1.0))
    return {
        "score": round(score, 3),
        "rock_density": round(rock_density, 4),
        "max_slope_deg": round(float(slope_deg.max()), 2),
        "mean_slope_deg": round(mean_slope, 2),
        "passable_ratio": round(passable, 3),
        "longest_corridor_m": _longest_corridor_m(obstacle),
    }


# ─── 6. meta.json / index.json (I1 포맷) ────────────────────────────────
def build_meta(terrain_id, seed, minerals, spawns, difficulty):
    return {
        "terrain_id": terrain_id,
        "version": "1.0",
        "seed": seed,
        "generated_at": now_iso(),
        "size_m": [SIZE_M, SIZE_M],
        "resolution_m": RESOLUTION_M,
        "origin": {"x": ORIGIN[0], "y": ORIGIN[1]},
        "generation_params": {
            "terrain": {
                "method": "base_relief+craters+hills+ridges",   # v2 — T1 엔진 흡수
                "crater_count": CFG["crater_count"],
                "hill_count": CFG["hill_count"],
                "ridge_count": CFG["ridge_count"],
            },
            "rocks": {
                "count": CFG["rock_count"],
                "size_range_m": list(CFG["rock_size_m"]),
                "min_spacing_m": CFG["rock_spacing_m"],
                "slope_threshold_deg": CFG["obstacle_slope_deg"],
                "asset_pool": ["rock_default"],
            },
            "minerals": {
                "count": CFG["mineral_count"],
                "min_spacing_m": CFG["mineral_spacing_m"],
                "exclude_basecamp_radius_m": 5.0,
                "value_distribution": {
                    "blue": {"prob": 0.5, "score": 10},
                    "red": {"prob": 0.3, "score": 25},
                    "yellow": {"prob": 0.2, "score": 50},
                },
            },
        },
        "spawn_locations": spawns,
        "basecamp": {
            "center": {"x": CFG["basecamp_center"][0],
                       "y": CFG["basecamp_center"][1]},
            "radius": CFG["basecamp_radius"],
            "marker_usd": "basecamp_dome.usd",
            "visual_footprint_m": [3.0, 3.0],
            "marker_height_m": 5.5,
            "shape": None,
            "entry_points": [],
            "collision_usd_path": None,
        },
        "minerals": minerals,
        "physics_zones": [
            {"type": "sand",
             "polygon": [[-10, -10], [10, -10], [10, 0], [-10, 0]],
             "static_friction": 0.30, "dynamic_friction": 0.25},
            {"type": "rocky",
             "polygon": [[-25, -25], [-10, -25], [-10, -10], [-25, -10]],
             "static_friction": 0.55, "dynamic_friction": 0.50},
        ],
        "minimap": {
            "grid_size": [25, 25],
            "cell_size_m": 2.0,
            "origin": {"x": ORIGIN[0], "y": ORIGIN[1]},
        },
        "difficulty": difficulty,
    }


def write_index(terrain_id, split, difficulty_score, seed):
    """I1 포맷 index.json. I1 적합 엔트리만 유지 (레거시 비적합 폴더는 제외)."""
    path = GENERATED_DIR / "index.json"
    kept = []
    dropped = []
    if path.exists():
        try:
            existing = json.loads(path.read_text(encoding="utf-8")).get("terrains", [])
        except Exception:
            existing = []
        for e in existing:
            eid = e.get("id")
            if (eid and re.match(r"^terrain_[0-9]{5}$", str(eid))
                    and "split" in e and "difficulty" in e and eid != terrain_id):
                kept.append({"id": eid, "split": e["split"],
                             "difficulty": e["difficulty"],
                             "seed": e.get("seed", -1)})
            elif eid != terrain_id:
                dropped.append(e.get("id") or e.get("folder") or "?")
    kept.append({"id": terrain_id, "split": split,
                 "difficulty": difficulty_score, "seed": seed})
    kept.sort(key=lambda e: e["id"])
    index = {"version": "1.0", "generated_at": now_iso(), "terrains": kept}
    path.write_text(json.dumps(index, indent=2, ensure_ascii=False),
                    encoding="utf-8")
    return index, dropped


# ─── 7. USD (옵션 — pxr 필요. T1 world_composer 재사용) ──────────────────
def maybe_export_usd(terrain_dir, terrain_id, hm, xs, ys, rocks,
                     minerals, basecamp, generated_at) -> bool:
    """terrain_only.usd / rocks_merged.usd + master scene 조립.

    master scene 정책: per-terrain 보존 worlds/<terrain_id>.usd
    + 최신 alias worlds/mars_exploration_world.usd (둘 다).
    pxr(Isaac Sim 런타임) 없으면 전부 건너뜀 — npy/json은 그대로 유효.
    """
    import shutil
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    try:
        import world_composer as wc
    except Exception as exc:
        print(f"[usd] world_composer import 실패 — USD 건너뜀 ({exc})")
        return False
    if not wc.PXR_AVAILABLE:
        print("[usd] pxr 미가용 (plain python3) — USD 건너뜀. "
              "isaac-python으로 재실행 시 USD가 생성됨.")
        return False

    s = CFG["mesh_stride"]
    terrain_usd = terrain_dir / "terrain_only.usd"
    rocks_usd = terrain_dir / "rocks_merged.usd"
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    per_terrain_world = WORLDS_DIR / f"{terrain_id}.usd"
    latest_world = WORLDS_DIR / "mars_exploration_world.usd"
    try:
        wc.export_terrain_usd(hm[::s, ::s], xs[::s], ys[::s], terrain_usd,
                              TEXTURE_DIR if TEXTURE_DIR.exists() else None)
        wc.export_rocks_usd(rocks, rocks_usd, lambda x, y: sample(hm, x, y))
        # master scene — terrain별 보존본
        wc.compose_world(per_terrain_world, terrain_usd, rocks_usd,
                         MARKERS_DIR, minerals, generated_at, basecamp=basecamp)
        # 최신 alias — worlds/ 동일 폴더라 상대 ref 그대로 유효 → byte-copy
        shutil.copyfile(per_terrain_world, latest_world)
    except Exception as exc:
        # USD는 옵션 — 실패해도 I1 핵심 데이터 파일(npy/json)은 그대로 유효.
        print(f"[usd] USD 익스포트 실패 — npy/json 데이터 파일은 정상 생성됨. "
              f"({type(exc).__name__}: {exc})")
        return False
    print(f"[usd] terrain_only.usd / rocks_merged.usd "
          f"(시각 메시 {GRID // s}×{GRID // s}, PBR 화성 텍스처)")
    print(f"[usd] master scene → worlds/{terrain_id}.usd (보존) "
          f"+ worlds/mars_exploration_world.usd (최신 alias)")
    return True


# ─── 8. I1 적합성 자가검증 ──────────────────────────────────────────────
REQUIRED_META = ["terrain_id", "version", "seed", "size_m", "resolution_m",
                 "origin", "spawn_locations", "basecamp", "minerals",
                 "minimap", "difficulty"]


def verify(terrain_dir) -> bool:
    print("\n── I1 적합성 검증 ──")
    ok = True
    hm = np.load(terrain_dir / "heightmap.npy")
    og = np.load(terrain_dir / "obstacle_grid.npy")
    meta = json.loads((terrain_dir / "meta.json").read_text(encoding="utf-8"))

    def check(label, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}")

    check(f"heightmap shape (1000,1000) — {hm.shape}", hm.shape == (GRID, GRID))
    check(f"heightmap dtype float32 — {hm.dtype}", hm.dtype == np.float32)
    check(f"obstacle_grid shape (1000,1000) — {og.shape}", og.shape == (GRID, GRID))
    check(f"obstacle_grid dtype int8 — {og.dtype}", og.dtype == np.int8)
    check("obstacle_grid 값 ⊆ {0,1}",
          set(np.unique(og).tolist()) <= {0, 1})
    missing = [k for k in REQUIRED_META if k not in meta]
    check(f"meta 필수필드 11개" + (f" — 누락 {missing}" if missing else ""),
          not missing)
    check(f"terrain_id 패턴 ^terrain_[0-9]{{5}}$ — {meta.get('terrain_id')}",
          bool(re.match(r"^terrain_[0-9]{5}$", str(meta.get("terrain_id", "")))))
    check("basecamp center/radius/marker_usd 존재",
          all(k in meta.get("basecamp", {})
              for k in ("center", "radius", "marker_usd")))
    check("difficulty score/rock_density/passable_ratio 존재",
          all(k in meta.get("difficulty", {})
              for k in ("score", "rock_density", "passable_ratio")))
    check(f"spawn_locations ≥ 1 — {len(meta.get('spawn_locations', []))}개",
          len(meta.get("spawn_locations", [])) >= 1)
    check("minerals 각 항목 id/type/position/value 존재",
          all(all(k in m for k in ("id", "type", "position", "value"))
              for m in meta.get("minerals", [])))

    try:
        import jsonschema
        jsonschema.validate(meta, json.loads(
            SCHEMA_PATH.read_text(encoding="utf-8")))
        print("  [PASS] jsonschema 정식 검증 (terrain_meta_schema.json)")
    except ImportError:
        print("  [skip] jsonschema 미설치 — 수동 체크만 수행")
    except Exception as exc:
        ok = False
        print(f"  [FAIL] jsonschema 검증 — {exc}")
    return ok


# ─── Main ───────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(description="I1 규약 준수 화성 지형 생성기 v2")
    ap.add_argument("--seed", type=int, default=23456)
    ap.add_argument("--terrain-id", default="terrain_00002")
    ap.add_argument("--split", default="train", choices=["train", "holdout"])
    args = ap.parse_args()

    if not re.match(r"^terrain_[0-9]{5}$", args.terrain_id):
        print(f"[error] --terrain-id 는 terrain_NNNNN 형식이어야 함 "
              f"(받음: {args.terrain_id})")
        return 1

    rng = np.random.default_rng(args.seed)
    terrain_dir = GENERATED_DIR / args.terrain_id
    terrain_dir.mkdir(parents=True, exist_ok=True)

    print(f"[v2] {args.terrain_id} 생성 — seed={args.seed}, "
          f"{GRID}×{GRID} @ {RESOLUTION_M} m ({SIZE_M:.0f}×{SIZE_M:.0f} m)")
    xs, ys, xx, yy = build_meshgrid()

    print("[1/7] heightmap (베이스 기복 + 크레이터 + 언덕 + 능선)...")
    hm = generate_heightmap(rng, xx, yy)
    print(f"      높이 범위 {hm.min():.2f} ~ {hm.max():.2f} m")

    print("[2/7] slope...")
    slope = compute_slope_deg(hm)

    print(f"[3/7] rocks (목표 {CFG['rock_count']})...")
    rocks = place_rocks(rng, slope)
    print(f"      {len(rocks)}개 배치")

    print("[4/7] obstacle_grid (int8)...")
    obstacle = build_obstacle_grid(slope, rocks)

    print(f"[5/7] minerals (목표 {CFG['mineral_count']}) "
          f"+ spawns (목표 {CFG['spawn_count']})...")
    minerals = place_minerals(rng, hm, slope, rocks)
    spawns = place_spawns(rng, hm, slope, obstacle)
    print(f"      minerals {len(minerals)}개, spawns {len(spawns)}개")

    print("[6/7] difficulty + meta + index...")
    difficulty = compute_difficulty(slope, obstacle, rocks)
    meta = build_meta(args.terrain_id, args.seed, minerals, spawns, difficulty)

    np.save(terrain_dir / "heightmap.npy", hm)
    np.save(terrain_dir / "obstacle_grid.npy", obstacle)
    (terrain_dir / "meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    index, dropped = write_index(args.terrain_id, args.split,
                                 difficulty["score"], args.seed)
    if dropped:
        print(f"      index.json 정리 — 비적합(레거시) 엔트리 제외: {dropped}")

    print("[7/7] USD + master scene (옵션)...")
    maybe_export_usd(terrain_dir, args.terrain_id, hm, xs, ys, rocks,
                     minerals, meta["basecamp"], meta["generated_at"])

    ok = verify(terrain_dir)

    print(f"\n{'✅' if ok else '❌'} {args.terrain_id} → {terrain_dir}")
    print(f"   difficulty: score={difficulty['score']} "
          f"passable={difficulty['passable_ratio'] * 100:.0f}% "
          f"mean_slope={difficulty['mean_slope_deg']}° "
          f"corridor={difficulty['longest_corridor_m']} m")
    print(f"   index.json: {len(index['terrains'])} terrains "
          f"({', '.join(t['id'] for t in index['terrains'])})")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
