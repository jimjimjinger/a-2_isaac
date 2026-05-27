#!/usr/bin/env python3
"""
mars_terrain_generator_v5.py — v4 기반 + slope 완만화 + spawn obstacle 안전 (v5).

v4 와 동일한 출력 포맷. 차이점:
  [A] slope 완만화 — rover 통행 가능 영역 ↑, 영상 시각 깔끔.
    - base_amp_m       1.0  → 0.7      (광역 기복 ↓)
    - detail_amp_m     0.22 → 0.10     (미세 노이즈 ↓↓ — mean slope 결정적)
    - crater_depth_m   (0.20,0.85) → (0.15,0.50)
    - crater_radius    (0.03,0.10) → (0.05,0.12)
    - hill_height_m    (0.25,0.90) → (0.20,0.55)
    - hill_radius      (0.06,0.18) → (0.08,0.22)
    - ridge_height_m   (0.20,0.60) → (0.15,0.35)
    - ridge_width      (0.03,0.08) → (0.05,0.10)
    → mean slope ~30% ↓, obstacle 셀 ~55% ↓

  [B] spawn obstacle 침범 차단 — v4 의 place_spawns 는 spawn 중심 한 점만
    obstacle 검사 → rover footprint 가장자리가 obstacle 영역 침범 가능
    (2026-05-27 rover_2 spawn obstacle 안 관찰). disc-check 추가 — spawn
    중심으로부터 spawn_clear_radius_m (1.0m) 안에 obstacle 셀이 단 하나라도
    있으면 reject. rover footprint half 0.62m + safety margin 0.38m.

obstacle_slope_deg=25° / rock spec / mineral spec 등 안전 임계 모두 유지.

사용:
  python3 isaac_sim/scripts/mars_terrain_generator_v5.py \\
      --seed 25025 --terrain-id terrain_00025 --split train

──── 아래는 v3 원문 docstring (참고용) ────
v3 docstring: I1 규약 준수 화성 지형 생성기.

T1(김현중)이 기획한 지형 생성 기능(크레이터·언덕·능선)을 흡수하되,
출력은 I1 지형 규약(docs/interfaces/INTERFACE_CONTRACTS.md,
terrain_meta_schema.json)을 100% 따른다.

이전 Perlin-only 시안(stopgap) 대비:
  - heightmap = 베이스 기복 + 크레이터 + 언덕 + 능선  (T1 엔진 흡수)
  - 피처 크기를 월드 span 비율로 파라미터화 → 50 m 아레나에 맞게 재조준
  - basecamp은 (0,0) 고정 — base-candidate 자동선택은 흡수 안 함
    (소비 트랙 T3/T4가 가정할 수 있는 유일한 '값'이라 팀 영향 0 유지)
  - terrain_only.usd 에 50 m 아레나 경계 충돌벽 4면 내장 → 로버 맵 밖 낙하 방지

I1 출력:
  generated_terrains/terrain_NNNNN/{heightmap.npy, obstacle_grid.npy,
                                    meta.json, terrain_only.usd*,
                                    rocks_merged.usd*}
  generated_terrains/index.json
  (* USD는 pxr/Isaac Sim 런타임 필요 — plain python3 실행 시 건너뜀)

v3 추가:
  - StarCraft 2 epic obstacle 4개를 상/하/좌/우 landmark로 배치
  - visual USD reference와 obstacle_grid footprint를 함께 생성

사용:
  python3 isaac_sim/scripts/mars_terrain_generator_v3.py \\
      --seed 23456 --terrain-id terrain_00023 --split train
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import numpy as np

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
    PXR_AVAILABLE = True
except Exception:  # pragma: no cover - Isaac Sim 런타임에서만 동작
    Gf = Sdf = Usd = UsdGeom = UsdLux = UsdPhysics = UsdShade = None
    PXR_AVAILABLE = False

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[2]
ISAAC_SIM_DIR = REPO_ROOT / "isaac_sim"
GENERATED_DIR = ISAAC_SIM_DIR / "assets" / "generated_terrains"
TEXTURE_DIR = ISAAC_SIM_DIR / "assets" / "textures" / "Mars"
MARKERS_DIR = ISAAC_SIM_DIR / "assets" / "markers"
WORLDS_DIR = ISAAC_SIM_DIR / "worlds"
SCHEMA_PATH = REPO_ROOT / "docs" / "interfaces" / "terrain_meta_schema.json"
WESTERN_ROCK_USD = ISAAC_SIM_DIR / "assets" / "Western_Stylised_Rock" / "scene.usdc"
EPIC_OBSTACLE_DIR = ISAAC_SIM_DIR / "assets" / "epic_obstacles"
# 원본 메시 X-span: bbox -193~+189 cm (metersPerUnit=0.01) → 3.82 m
_ROCK_NATURAL_WIDTH_M = 3.82

# ─── I1 규약 상수 (동결 — 절대 변경 금지. 로버 실측 검증된 값) ───────────
SIZE_M = 50.0                             # 월드 크기 (정사각)
RESOLUTION_M = 0.05                       # heightmap 해상도 → 1000×1000
GRID = int(SIZE_M / RESOLUTION_M)         # 1000
ORIGIN = (-SIZE_M / 2.0, -SIZE_M / 2.0)   # 좌하단 (-25, -25)

# ─── 생성 파라미터 (v2 — 50 m 월드 기준 재조준) ─────────────────────────
# T1 생성기는 ≈511 m 월드 기준이라 크레이터 반경 10~30 m 등 절대값이었음.
# 여기서는 모두 월드 span 비율(frac)로 표현 → 50 m 아레나에 일관되게 축소.
CFG = {
    # v5 [A] slope 완만화 — rover 통행 영역 ↑, 영상 시각 깔끔.
    "base_amp_m": 0.7,            # 1.0 → 0.7
    "base_octave_cells": 8,
    "detail_amp_m": 0.10,         # 0.22 → 0.10 (mean slope 결정적)
    "detail_octave_cells": 24,
    "crater_count": 8,
    "crater_radius_frac": (0.05, 0.12),   # 약간 넓힘
    "crater_depth_m": (0.15, 0.50),       # 깊은 크레이터 ↓
    "hill_count": 5,
    "hill_radius_frac": (0.08, 0.22),
    "hill_height_m": (0.20, 0.55),
    "ridge_count": 3,
    "ridge_length_frac": (0.25, 0.55),    # 유지
    "ridge_width_frac": (0.05, 0.10),     # 두껍게 (slope ↓)
    "ridge_height_m": (0.15, 0.35),       # 능선 가파름 결정적 ↓
    "rock_count": 50,
    "rock_size_m": (0.30, 1.00),    # v3 와 동일 (시각 다양성 유지)
    "rock_spacing_m": 3.5,          # v3=1.0 → 3.5. inflation 0.8 후 통로 확보
    # mineral 옆 rock 차단 거리 — place_minerals 의 rock keepout 에 사용.
    # v3 의 하드코드 0.5 를 명시적 CFG 로 격상 + 0.5→2.0 강화. arm reach
    # ~1.2 m 와 inflation 0.8 합쳐서 광물 접근 path 확보.
    "mineral_clearance_from_rock_m": 2.0,
    # v5 [B] spawn footprint 보호 — disc-check radius.
    # rover footprint half ~0.62m + safety margin 0.38m. place_spawns 가
    # spawn 중심 주변 이 반경 안에 obstacle 셀이 하나라도 있으면 reject.
    "spawn_clear_radius_m": 1.0,
    "mineral_count": 12,
    "mineral_spacing_m": 3.0,
    "spawn_count": 50,
    "obstacle_slope_deg": 25.0,   # terrain_00001과 동일 (로버 실측 검증된 값)
    "spawn_slope_deg": 15.0,
    "mineral_slope_deg": 18.0,
    "basecamp_center": (0.0, 0.0),  # I1 Tier 1 — (0,0) 고정
    # keepout·평탄화·spawn/rock/mineral 배제가 모두 이 값에서 파생된다.
    # basecamp 은 8×8 m 구조물(아래 collision Cube)이므로, 그 대각 반경
    # (√32 ≈ 5.66 m)을 감싸도록 6.0 으로 둔다. 옛 3.0 은 3 m 베이스캠프
    # 가정의 잔재 — 구조물보다 작아 배제가 박스를 못 덮었다.
    "basecamp_radius": 6.0,
    # basecamp_dome.usd 내부 정적 충돌 Cube 한 변 (8×8×8 m, 중심 (0,0)).
    # 로버가 실제로 막히는 건 이 Cube. obstacle_grid(_stamp_basecamp)는 이
    # 값과 동기화할 것 (자산 변경 시 같이 수정).
    "basecamp_collision_size_m": 8.0,
    "mesh_stride": 5,             # USD 시각 메시 다운샘플 (npy는 풀해상도 유지)
    # ─ 맵경계 낙하 방지 — 50 m 아레나 둘레 정적 충돌벽 ─
    "boundary_wall_thickness_m": 0.5,      # 벽 두께
    "boundary_wall_height_above_m": 3.0,   # 지형 최고점 위로 솟는 높이 (점프 차단)
    "boundary_wall_depth_below_m": 1.0,    # 지형 최저점 아래로 묻히는 깊이 (빈틈 차단)
}


EPIC_OBSTACLES = [
    {
        "id": "north_battlecruiser",
        "type": "battlecruiser",
        "asset_usd": "battlecruiser_starcraft2/obstacle.usd",
        "x": 0.0,
        "y": 16.0,
        "yaw": 0.0,
        "footprint_m": [5.18, 7.00],
        "height_m": 3.62,
    },
    {
        "id": "south_barracks",
        "type": "barracks",
        "asset_usd": "barracks_starcraft2/obstacle.usd",
        "x": 0.0,
        "y": -16.0,
        "yaw": math.radians(180.0),
        "footprint_m": [7.00, 4.91],
        "height_m": 4.61,
    },
    {
        "id": "east_goliath",
        "type": "goliath",
        "asset_usd": "goliath_blackops/obstacle.usd",
        "x": 16.0,
        "y": 0.0,
        "yaw": math.radians(90.0),
        "footprint_m": [5.69, 6.00],
        "height_m": 6.74,
    },
    {
        "id": "west_scv",
        "type": "scv",
        "asset_usd": "scv_starcraft2/obstacle.usd",
        "x": -16.0,
        "y": 0.0,
        "yaw": math.radians(-90.0),
        "footprint_m": [5.00, 4.17],
        "height_m": 4.13,
    },
]


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


def _point_in_epic_obstacle(x: float, y: float, obstacle: Dict[str, Any],
                            margin_m: float = 0.0) -> bool:
    """Rotated rectangle footprint test for epic obstacle keepout."""
    cx, cy = float(obstacle["x"]), float(obstacle["y"])
    yaw = float(obstacle.get("yaw", 0.0))
    width, depth = [float(v) + 2.0 * margin_m for v in obstacle["footprint_m"]]
    dx, dy = x - cx, y - cy
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return abs(local_x) <= 0.5 * width and abs(local_y) <= 0.5 * depth


def _inside_any_epic_obstacle(x: float, y: float, margin_m: float = 0.0) -> bool:
    return any(_point_in_epic_obstacle(x, y, obs, margin_m)
               for obs in EPIC_OBSTACLES)


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


def _flatten_epic_obstacle_pads(hm, xx, yy):
    """큰 landmark 장애물이 뜨거나 과하게 파묻히지 않도록 footprint 주변을 평탄화."""
    out = hm.copy()
    margin = 1.2
    for obs in EPIC_OBSTACLES:
        cx, cy = float(obs["x"]), float(obs["y"])
        yaw = float(obs.get("yaw", 0.0))
        width, depth = [float(v) for v in obs["footprint_m"]]
        dx, dy = xx - cx, yy - cy
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        ax = np.abs(local_x) - 0.5 * width
        ay = np.abs(local_y) - 0.5 * depth
        inside = (ax <= 0.0) & (ay <= 0.0)
        if not inside.any():
            continue
        target = float(out[inside].mean())
        # Outside signed distance approximation for a rounded rectangular blend band.
        outside_dx = np.maximum(ax, 0.0)
        outside_dy = np.maximum(ay, 0.0)
        outside_dist = np.sqrt(outside_dx ** 2 + outside_dy ** 2)
        inside_dist = np.minimum(np.maximum(ax, ay), 0.0)
        signed_dist = outside_dist + inside_dist
        t = np.clip((margin - signed_dist) / margin, 0.0, 1.0)
        w = t * t * (3.0 - 2.0 * t)
        out = out * (1.0 - w) + target * w
    return out


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
    hm = _flatten_epic_obstacle_pads(hm, xx, yy)
    return hm.astype(np.float32)


# ─── 3. Slope / obstacle ────────────────────────────────────────────────
def compute_slope_deg(hm: np.ndarray) -> np.ndarray:
    dz_dy, dz_dx = np.gradient(hm, RESOLUTION_M, RESOLUTION_M)
    return np.degrees(np.arctan(np.sqrt(dz_dx ** 2 + dz_dy ** 2))).astype(np.float32)


def _stamp_disc(obstacle, cx, cy, radius_m) -> None:
    """world (cx,cy) 중심, 반경 radius_m 원을 obstacle(1)로 OR-in 한다."""
    i, j = world_to_idx(cx, cy)
    cell_r = max(1, int(math.ceil(radius_m / RESOLUTION_M)))
    i0, i1 = max(0, i - cell_r), min(GRID, i + cell_r + 1)
    j0, j1 = max(0, j - cell_r), min(GRID, j + cell_r + 1)
    ii, jj = np.ogrid[i0:i1, j0:j1]
    mask = (ii - i) ** 2 + (jj - j) ** 2 <= cell_r ** 2
    obstacle[i0:i1, j0:j1] = np.maximum(obstacle[i0:i1, j0:j1],
                                        mask.astype(np.int8))


def _stamp_basecamp(obstacle) -> None:
    """베이스캠프 충돌체를 obstacle(1)로 마킹한다.

    basecamp_dome.usd 는 중심에 8×8×8 m 정적 충돌 Cube 를 갖는다 — 시각
    메시(visual_footprint 3×3 m)보다 크고, 로버가 실제로 막히는 건 이 Cube다.
    그런데 heightmap 은 _flatten_basecamp 로 평탄화돼 경사 기준에 안 걸리고
    바위도 베이스캠프 keepout 밖에만 놓이므로, 명시적으로 안 찍으면
    obstacle_grid 가 베이스캠프를 자유공간으로 본다 — 로버가 통과 경로를
    계획하다 실제 씬에선 충돌. 축 정렬 박스라 정사각 풋프린트로 찍는다.
    """
    bcx, bcy = CFG["basecamp_center"]
    half = 0.5 * CFG["basecamp_collision_size_m"]
    i0, j0 = world_to_idx(bcx - half, bcy - half)
    i1, j1 = world_to_idx(bcx + half, bcy + half)
    obstacle[i0:i1 + 1, j0:j1 + 1] = 1


def _stamp_rotated_rect(obstacle, cx, cy, width_m, depth_m, yaw_rad,
                        margin_m=0.0) -> None:
    """world 중심 회전 사각 footprint를 obstacle(1)로 OR-in 한다."""
    width_m = float(width_m) + 2.0 * float(margin_m)
    depth_m = float(depth_m) + 2.0 * float(margin_m)
    half_diag = 0.5 * math.hypot(width_m, depth_m)
    ci, cj = world_to_idx(cx, cy)
    cell_r = max(1, int(math.ceil(half_diag / RESOLUTION_M)))
    i0, i1 = max(0, ci - cell_r), min(GRID, ci + cell_r + 1)
    j0, j1 = max(0, cj - cell_r), min(GRID, cj + cell_r + 1)
    ys = ORIGIN[1] + np.arange(i0, i1, dtype=np.float32) * RESOLUTION_M
    xs = ORIGIN[0] + np.arange(j0, j1, dtype=np.float32) * RESOLUTION_M
    xx, yy = np.meshgrid(xs, ys)
    dx, dy = xx - float(cx), yy - float(cy)
    local_x = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    local_y = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    mask = ((np.abs(local_x) <= 0.5 * width_m)
            & (np.abs(local_y) <= 0.5 * depth_m))
    obstacle[i0:i1, j0:j1] = np.maximum(obstacle[i0:i1, j0:j1],
                                        mask.astype(np.int8))


def _stamp_epic_obstacles(obstacle) -> None:
    for obs in EPIC_OBSTACLES:
        width, depth = obs["footprint_m"]
        _stamp_rotated_rect(
            obstacle,
            float(obs["x"]),
            float(obs["y"]),
            float(width),
            float(depth),
            float(obs.get("yaw", 0.0)),
            margin_m=0.35,
        )


def build_obstacle_grid(slope_deg, rocks) -> np.ndarray:
    """I1: shape (1000,1000), dtype int8, 0=safe 1=obstacle.

    obstacle = (1) 경사 임계 초과 셀  (2) 바위 풋프린트  (3) 베이스캠프 충돌체
               (4) v3 epic obstacle footprint.
    """
    obstacle = (slope_deg > CFG["obstacle_slope_deg"]).astype(np.int8)
    for rk in rocks:
        _stamp_disc(obstacle, rk["x"], rk["y"], rk["radius"])
    _stamp_basecamp(obstacle)
    _stamp_epic_obstacles(obstacle)
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
        if _inside_any_epic_obstacle(x, y, margin_m=1.0):
            continue
        if sample(slope_deg, x, y) > CFG["obstacle_slope_deg"]:
            continue
        if any(math.hypot(x - r["x"], y - r["y"]) < CFG["rock_spacing_m"]
               for r in rocks):
            continue
        rocks.append({"x": round(x, 3), "y": round(y, 3),
                      "radius": round(float(rng.uniform(*CFG["rock_size_m"])), 3)})
    return rocks


def place_minerals(rng, hm, slope_deg, rocks, obstacle=None):
    minerals = []
    bcx, bcy = CFG["basecamp_center"]
    lo, hi = ORIGIN[0] + 2.0, ORIGIN[0] + SIZE_M - 2.0
    next_id = 1
    for _ in range(CFG["mineral_count"] * 60):
        if len(minerals) >= CFG["mineral_count"]:
            break
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if math.hypot(x - bcx, y - bcy) < CFG["basecamp_radius"]:
            continue
        if obstacle is not None and sample(obstacle, x, y) > 0.5:
            continue
        if _inside_any_epic_obstacle(x, y, margin_m=0.8):
            continue
        if any(math.hypot(x - r["x"], y - r["y"])
               < r["radius"] + CFG["mineral_clearance_from_rock_m"]
               for r in rocks):
            continue
        if any(math.hypot(x - m["position"]["x"], y - m["position"]["y"])
               < CFG["mineral_spacing_m"] for m in minerals):
            continue
        if sample(slope_deg, x, y) > CFG["mineral_slope_deg"]:
            continue
        roll = float(rng.random())
        if roll < 0.5:
            mtype, value = "blue_mineral", 10
        elif roll < 0.8:
            mtype, value = "green_gas", 25
        else:
            mtype, value = "yellow_mineral", 50
        z = sample(hm, x, y) + 0.10
        minerals.append({
            "id": next_id,
            "type": mtype,
            "position": {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)},
            "value": value,
        })
        next_id += 1
    return minerals


def _spawn_clear_of_static_obstacles(x: float, y: float,
                                     rocks, radius_m: float) -> bool:
    """spawn 위치가 rocks / epic_obstacles / basecamp 의 footprint 와
    radius_m 마진 이상 떨어졌는지 데이터 기반 직접 검사.
    obstacle_grid 의 axis convention 추론 불필요 — 가장 robust.

    검사 대상:
      1. rocks: center 간 거리 < rock.radius + radius_m → reject
      2. epic obstacles: center 간 거리 < footprint_max/2 + radius_m → reject
      3. basecamp: center 간 거리 < basecamp_collision_size_m/√2 + radius_m
    """
    # rocks
    for rk in rocks:
        if math.hypot(x - rk["x"], y - rk["y"]) < rk["radius"] + radius_m:
            return False
    # epic obstacles — footprint_m 의 대각 반지름 사용 (회전 무관 보수)
    for obs in EPIC_OBSTACLES:
        ox = float(obs["x"])
        oy = float(obs["y"])
        fw, fd = float(obs["footprint_m"][0]), float(obs["footprint_m"][1])
        keepout = 0.5 * math.hypot(fw, fd) + radius_m
        if math.hypot(x - ox, y - oy) < keepout:
            return False
    # basecamp — 8x8m 정사각이라 대각 반지름 ~5.66m + spawn margin
    bcx, bcy = CFG["basecamp_center"]
    bc_half = 0.5 * CFG["basecamp_collision_size_m"]
    bc_keepout = bc_half * math.sqrt(2.0) + radius_m
    if math.hypot(x - bcx, y - bcy) < bc_keepout:
        return False
    return True


def place_spawns(rng, hm, slope_deg, obstacle, minerals=None, rocks=None):
    spawns = []
    rocks = rocks or []
    bcx, bcy = CFG["basecamp_center"]
    lo, hi = ORIGIN[0] + 2.0, ORIGIN[0] + SIZE_M - 2.0
    mineral_pts = [(m["position"]["x"], m["position"]["y"])
                   for m in (minerals or [])]
    spawn_clearance = 1.5   # 로버 간 최소 간격 (m)
    mineral_clearance = 0.8  # 미네랄과 최소 간격 (m)
    spawn_clear_r = CFG["spawn_clear_radius_m"]
    for _ in range(CFG["spawn_count"] * 60):
        if len(spawns) >= CFG["spawn_count"]:
            break
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        if math.hypot(x - bcx, y - bcy) < CFG["basecamp_radius"] + 0.5:
            continue
        if sample(slope_deg, x, y) > CFG["spawn_slope_deg"]:
            continue
        # v5 [B] spawn footprint 가 rocks / epic obstacles / basecamp 침범 금지.
        # 데이터 기반 직접 거리 검사 — obstacle_grid 의 axis convention 추론
        # 불필요. rover footprint half ~0.62 + safety ~0.38 = 1.0m 마진.
        if not _spawn_clear_of_static_obstacles(x, y, rocks, spawn_clear_r):
            continue
        # 기존 spawn과 최소 간격 체크 (로버끼리 겹침 방지)
        if any(math.hypot(x - s["x"], y - s["y"]) < spawn_clearance
               for s in spawns):
            continue
        # mineral과 최소 간격 체크 (RigidBody mineral과 충돌 방지)
        if any(math.hypot(x - mx, y - my) < mineral_clearance
               for mx, my in mineral_pts):
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
def build_epic_obstacle_meta(hm):
    out = []
    for obs in EPIC_OBSTACLES:
        x, y = float(obs["x"]), float(obs["y"])
        out.append({
            "id": obs["id"],
            "type": obs["type"],
            "asset_usd": obs["asset_usd"],
            "position": {
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(sample(hm, x, y), 2),
            },
            "yaw": round(float(obs.get("yaw", 0.0)), 3),
            "footprint_m": [round(float(v), 2) for v in obs["footprint_m"]],
            "height_m": round(float(obs["height_m"]), 2),
        })
    return out


def build_meta(terrain_id, seed, minerals, spawns, difficulty, epic_obstacles):
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
                "mineral_clearance_from_rock_m":
                    CFG["mineral_clearance_from_rock_m"],
                "slope_threshold_deg": CFG["obstacle_slope_deg"],
                "asset_pool": ["rock_default"],
                "generator": "v5",
            },
            "epic_obstacles": {
                "count": len(epic_obstacles),
                "placement": "cardinal_landmarks",
                "asset_pool": [o["asset_usd"] for o in epic_obstacles],
            },
            "minerals": {
                "count": CFG["mineral_count"],
                "min_spacing_m": CFG["mineral_spacing_m"],
                "exclude_basecamp_radius_m": CFG["basecamp_radius"],
                "value_distribution": {
                    "blue_mineral":   {"prob": 0.5, "score": 10},
                    "green_gas":      {"prob": 0.3, "score": 25},
                    "yellow_mineral": {"prob": 0.2, "score": 50},
                },
            },
        },
        "epic_obstacles": epic_obstacles,
        "spawn_locations": spawns,
        "basecamp": {
            "center": {"x": CFG["basecamp_center"][0],
                       "y": CFG["basecamp_center"][1]},
            "radius": CFG["basecamp_radius"],
            "marker_usd": "basecamp_dome.usd",
            "visual_footprint_m": [8.0, 8.0],
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


# ─── 7. USD 출력 (pxr 필요 — Isaac Sim 런타임. plain python3에선 건너뜀) ──
# v1처럼 USD 출력을 이 파일에 자체 구현 (별도 world_composer 모듈 없이 단일 파일).
def require_pxr() -> None:
    if not PXR_AVAILABLE:
        raise RuntimeError(
            "USD 출력은 Isaac Sim의 pxr 런타임이 필요합니다. terrain_only.usd / "
            "rocks_merged.usd / master scene가 필요하면 plain python3 대신 "
            "isaac-python으로 실행하세요."
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


def _define_translated_reference(stage, prim_path: str, asset_path: Path,
                                 translate: Iterable[float],
                                 base_dir: Optional[Path]):
    """Create a translated Xform wrapper and reference an asset below it.

    Some referenced USD assets author their own root xform stack, so applying the
    translate directly on the same prim can be dropped during composition.  A
    dedicated parent Xform keeps the world pose stable regardless of the asset's
    internal transform stack.
    """
    wrapper = UsdGeom.Xform.Define(stage, prim_path)
    UsdGeom.XformCommonAPI(wrapper.GetPrim()).SetTranslate(
        Gf.Vec3d(*[float(v) for v in translate]))
    ref_prim = UsdGeom.Xform.Define(stage, f"{prim_path}/Reference")
    ref_prim.GetPrim().GetReferences().AddReference(
        _asset_ref(asset_path, base_dir))
    return wrapper, ref_prim


def _create_preview_material(stage, material_path, texture_dir, base_dir):
    """UsdPreviewSurface + 화성 PBR 텍스처 (albedo / roughness / normal)."""
    material = UsdShade.Material.Define(stage, material_path)
    surface = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    if texture_dir is not None:
        albedo = texture_dir / "mars_albedo.png"
        roughness = texture_dir / "mars_roughness.png"
        normal = texture_dir / "mars_normal.png"
        has_textures = albedo.exists() and roughness.exists() and normal.exists()
    else:
        has_textures = False

    if has_textures:
        reader = UsdShade.Shader.Define(stage, f"{material_path}/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        diffuse_tex = UsdShade.Shader.Define(stage, f"{material_path}/AlbedoTex")
        diffuse_tex.CreateIdAttr("UsdUVTexture")
        diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(albedo, base_dir)))
        diffuse_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        diffuse_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        diffuse_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        diffuse_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")

        rough_tex = UsdShade.Shader.Define(stage, f"{material_path}/RoughnessTex")
        rough_tex.CreateIdAttr("UsdUVTexture")
        rough_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(roughness, base_dir)))
        rough_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        rough_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        rough_tex.CreateOutput("r", Sdf.ValueTypeNames.Float)
        rough_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")

        # UsdPreviewSurface 표준: UsdUVTexture를 normal에 직결 ('UsdNormalMap'은
        # 표준 셰이더 아님). raw 색공간 + scale(2,2,2)/bias(-1,-1,-1)로 [0,1]
        # 텍셀을 [-1,1] 법선으로 remap.
        normal_tex = UsdShade.Shader.Define(stage, f"{material_path}/NormalTex")
        normal_tex.CreateIdAttr("UsdUVTexture")
        normal_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(_asset_ref(normal, base_dir)))
        normal_tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
        normal_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        normal_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        normal_tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(2.0, 2.0, 2.0, 1.0))
        normal_tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(-1.0, -1.0, -1.0, 0.0))
        normal_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        normal_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")

        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            diffuse_tex.ConnectableAPI(), "rgb")
        surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
            rough_tex.ConnectableAPI(), "r")
        surface.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
            normal_tex.ConnectableAPI(), "rgb")
    else:
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.72, 0.38, 0.22))

    material.CreateSurfaceOutput().ConnectToSource(
        surface.ConnectableAPI(), "surface")
    return material


def _make_uvs(xs, ys, x_min, y_min, uv_scale_m):
    uvs = []
    for y in ys:
        for x in xs:
            uvs.append(((float(x) - x_min) / uv_scale_m,
                        (float(y) - y_min) / uv_scale_m))
    return uvs


def _configure_stage(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    return world


def build_terrain_prim(stage, heightmap, x_coords, y_coords, texture_dir,
                       terrain_path="/Terrain"):
    """heightmap → 삼각형 메시 + PBR 머티리얼. 머티리얼은 terrain_path 하위에
    둔다 — defaultPrim 밖에 있으면 reference 합성 시 바인딩이 끊긴다."""
    UsdGeom.Xform.Define(stage, terrain_path)
    mesh = UsdGeom.Mesh.Define(stage, f"{terrain_path}/TerrainMesh")

    h, w = heightmap.shape
    points = []
    for yi, y in enumerate(y_coords):
        for xi, x in enumerate(x_coords):
            points.append(Gf.Vec3f(float(x), float(y), float(heightmap[yi, xi])))

    fvi, fvc = [], []
    for yi in range(h - 1):
        row_start, next_start = yi * w, (yi + 1) * w
        for xi in range(w - 1):
            v0, v1 = row_start + xi, row_start + xi + 1
            v2, v3 = next_start + xi + 1, next_start + xi
            fvi.extend([v0, v1, v2, v0, v2, v3])
            fvc.extend([3, 3])

    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexIndicesAttr(fvi)
    mesh.CreateFaceVertexCountsAttr(fvc)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)

    dx = float(x_coords[1] - x_coords[0]) if len(x_coords) > 1 else 1.0
    dy = float(y_coords[1] - y_coords[0]) if len(y_coords) > 1 else 1.0
    dz_dy, dz_dx = np.gradient(heightmap, dy, dx)
    normals = []
    for yi in range(h):
        for xi in range(w):
            nx, ny, nz = -float(dz_dx[yi, xi]), -float(dz_dy[yi, xi]), 1.0
            length = (nx * nx + ny * ny + nz * nz) ** 0.5
            normals.append(Gf.Vec3f(nx / length, ny / length, nz / length))
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation("vertex")

    st_primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    uv_scale_m = max(float(x_coords[-1] - x_coords[0]),
                     float(y_coords[-1] - y_coords[0])) / 4.0
    if uv_scale_m <= 0:
        uv_scale_m = 64.0
    st_primvar.Set(_make_uvs(x_coords, y_coords, float(x_coords[0]),
                             float(y_coords[0]), uv_scale_m))

    base_dir = (Path(stage.GetRootLayer().realPath).parent
                if stage.GetRootLayer().realPath else None)
    material = _create_preview_material(
        stage, f"{terrain_path}/Looks/MarsSurface", texture_dir, base_dir)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)

    # 정적 collider — 로버가 지형 위에 안착하도록 triangle-mesh 충돌 baking.
    # (RigidBodyAPI 없음 = static. approximation "none" = 정확한 삼각형 메시.)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")
    return mesh


def build_boundary_walls(stage, terrain_z_min, terrain_z_max,
                         walls_path="/Terrain/BoundaryWalls"):
    """50 m 아레나 경계에 투명 정적 충돌벽 4면을 세운다 — 로버 맵 밖 낙하 방지.

    벽 특성:
      - 보이지 않음 (visibility=invisible) — collision-only. 화성 지형 몰입을
        해치지 않으면서 로버를 물리적으로만 막는다. PhysX 충돌은 visibility 와
        무관히 동작한다.
      - static (RigidBodyAPI 없음 = 고정). UsdGeom.Cube 라 PhysX 가 analytic
        box collider 로 처리 — mesh 근사 불요.
      - 수직 범위는 지형 min/max 에서 유도: 최저점 아래로 묻고(빈틈 차단)
        최고점 위로 솟게 한다(점프 차단). 어떤 지형 기복에서도 안전.
      - 남/북 벽을 ±두께만큼 연장해 동/서 벽과 코너에서 겹침 → 모서리 갭 없음.

    walls_path 는 terrain_only.usd 의 defaultPrim(/Terrain) 하위 → world 합성
    시 reference 한 번으로 지형 메시와 함께 따라온다. master scene 수정 불요.

    재실행 안전: Define 은 기존 prim 을 반환하므로 idempotent.
    반환: 생성한 벽 개수(4).
    """
    thickness = float(CFG["boundary_wall_thickness_m"])
    z_bottom = float(terrain_z_min) - float(CFG["boundary_wall_depth_below_m"])
    z_top = float(terrain_z_max) + float(CFG["boundary_wall_height_above_m"])
    z_center = 0.5 * (z_bottom + z_top)
    height = z_top - z_bottom

    x_min, x_max = ORIGIN[0], ORIGIN[0] + SIZE_M       # -25, +25
    y_min, y_max = ORIGIN[1], ORIGIN[1] + SIZE_M       # -25, +25
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)

    UsdGeom.Xform.Define(stage, walls_path)
    # (이름, 중심, scale=박스 크기). 남/북 벽은 SIZE_M+2*두께 → 코너 겹침.
    walls = (
        ("wall_y_max", (x_mid, y_max, z_center),
         (SIZE_M + 2.0 * thickness, thickness, height)),
        ("wall_y_min", (x_mid, y_min, z_center),
         (SIZE_M + 2.0 * thickness, thickness, height)),
        ("wall_x_max", (x_max, y_mid, z_center),
         (thickness, SIZE_M, height)),
        ("wall_x_min", (x_min, y_mid, z_center),
         (thickness, SIZE_M, height)),
    )
    for name, center, scale in walls:
        cube = UsdGeom.Cube.Define(stage, f"{walls_path}/{name}")
        cube.CreateSizeAttr(1.0)                       # 단위 큐브 → scale 로 박스화
        cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5),
                               Gf.Vec3f(0.5, 0.5, 0.5)])
        xf = UsdGeom.XformCommonAPI(cube.GetPrim())
        xf.SetTranslate(Gf.Vec3d(*center))
        xf.SetScale(Gf.Vec3f(*scale))
        # 정적 collider — RigidBodyAPI 없음 = static.
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        # collision-only — 렌더링에서 숨김 (충돌은 그대로 유지).
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    return len(walls)


def build_rocks_prim(stage, rocks, terrain_height_at, rocks_path="/Rocks"):
    UsdGeom.Xform.Define(stage, rocks_path)
    use_mesh = WESTERN_ROCK_USD.exists()
    base_dir = (Path(stage.GetRootLayer().realPath).parent
                if stage.GetRootLayer().realPath else None)

    for idx, rock in enumerate(rocks):
        x, y = float(rock["x"]), float(rock["y"])
        radius = float(rock["radius"])
        prim_path = f"{rocks_path}/rock_{idx:04d}"

        if use_mesh:
            # cm→m 단위변환(×0.01) 후 원하는 반경에 맞춰 스케일
            scale = float(0.01 * (radius * 2.0) / _ROCK_NATURAL_WIDTH_M)
            z = float(terrain_height_at(x, y))
            xform = UsdGeom.Xform.Define(stage, prim_path)
            # XformOp 직접 설정 — XformCommonAPI 충돌 방지
            xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            xform.AddRotateXOp().Set(90.0)          # Y-up → Z-up
            xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
            ref = _asset_ref(WESTERN_ROCK_USD, base_dir)
            xform.GetPrim().GetReferences().AddReference(ref)
            UsdPhysics.CollisionAPI.Apply(xform.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(
                xform.GetPrim()).CreateApproximationAttr().Set("convexHull")
        else:
            # fallback: sphere proxy (pxr 없거나 에셋 누락 시)
            z = float(terrain_height_at(x, y)) + radius * 0.55
            prim = UsdGeom.Sphere.Define(stage, prim_path)
            prim.GetRadiusAttr().Set(radius)
            prim.GetDisplayColorAttr().Set([Gf.Vec3f(0.35, 0.28, 0.24)])
            UsdGeom.XformCommonAPI(prim).SetTranslate((x, y, z))
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())


def build_light_rig(stage):
    UsdGeom.Xform.Define(stage, "/World/Lights")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(2200.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate((35.0, 0.0, -25.0))
    sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
    sky.CreateIntensityAttr(150.0)
    sky.CreateColorAttr(Gf.Vec3f(0.95, 0.73, 0.57))


def build_epic_obstacles_prim(stage, epic_obstacles, terrain_height_at,
                              obstacles_path="/EpicObstacles"):
    UsdGeom.Xform.Define(stage, obstacles_path)
    base_dir = (Path(stage.GetRootLayer().realPath).parent
                if stage.GetRootLayer().realPath else None)
    for obs in epic_obstacles:
        asset_path = EPIC_OBSTACLE_DIR / obs["asset_usd"]
        if not asset_path.exists():
            print(f"[v3] epic obstacle asset 없음: {asset_path}")
            continue
        x, y = float(obs["position"]["x"]), float(obs["position"]["y"])
        z = float(terrain_height_at(x, y))
        yaw_deg = math.degrees(float(obs.get("yaw", 0.0)))
        prim_path = f"{obstacles_path}/{obs['id']}"
        xform = UsdGeom.Xform.Define(stage, prim_path)
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xform.AddRotateZOp().Set(yaw_deg)
        xform.GetPrim().GetReferences().AddReference(
            _asset_ref(asset_path, base_dir))


def export_terrain_usd(heightmap, x_coords, y_coords, out_path, texture_dir,
                       terrain_z_min, terrain_z_max):
    """terrain_only.usd — defaultPrim=/Terrain (reference 가능하도록).

    지형 메시 + 맵경계 충돌벽을 모두 /Terrain 하위에 둔다 → world 합성 시
    reference 한 번으로 둘 다 따라온다. terrain_z_min/max 는 풀해상도 heightmap
    에서 구한 값(다운샘플 메시가 아닌) — 벽 수직 범위를 정확히 잡기 위함.
    """
    require_pxr()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    _configure_stage(stage)
    build_terrain_prim(stage, heightmap, x_coords, y_coords, texture_dir,
                       terrain_path="/Terrain")
    build_boundary_walls(stage, terrain_z_min, terrain_z_max,
                         walls_path="/Terrain/BoundaryWalls")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Terrain"))
    stage.GetRootLayer().Save()


def export_rocks_usd(rocks, out_path, terrain_height_at):
    """rocks_merged.usd — sphere prim 모음, defaultPrim=/Rocks."""
    require_pxr()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    _configure_stage(stage)
    build_rocks_prim(stage, rocks, terrain_height_at, rocks_path="/Rocks")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Rocks"))
    stage.GetRootLayer().Save()


def export_epic_obstacles_usd(epic_obstacles, out_path, terrain_height_at):
    """epic_obstacles.usd — v3 landmark 장애물 모음, defaultPrim=/EpicObstacles."""
    require_pxr()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    _configure_stage(stage)
    build_epic_obstacles_prim(
        stage, epic_obstacles, terrain_height_at, obstacles_path="/EpicObstacles")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/EpicObstacles"))
    stage.GetRootLayer().Save()


def compose_world(world_path, terrain_usd, rocks_usd, epic_obstacles_usd,
                  marker_dir, minerals, generated_at, basecamp=None):
    """master scene — terrain/rocks/basecamp/광물 마커를 reference로 묶고 조명 추가."""
    require_pxr()
    world_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(world_path))
    _configure_stage(stage)

    terrain_prim = stage.DefinePrim("/World/Terrain", "Xform")
    terrain_prim.GetReferences().AddReference(
        _relpath_safe(terrain_usd, world_path.parent))
    rocks_prim = stage.DefinePrim("/World/Rocks", "Xform")
    rocks_prim.GetReferences().AddReference(
        _relpath_safe(rocks_usd, world_path.parent))
    if epic_obstacles_usd is not None and Path(epic_obstacles_usd).exists():
        epic_prim = stage.DefinePrim("/World/EpicObstacles", "Xform")
        epic_prim.GetReferences().AddReference(
            _relpath_safe(epic_obstacles_usd, world_path.parent))

    # Basecamp — markers/ 의 USD를 reference (mineral과 동일 패턴).
    # 해당 파일만 교체하면 원하는 basecamp 모양이 그대로 로드된다.
    if basecamp is not None:
        marker_path = marker_dir / basecamp.get("marker_usd", "basecamp_dome.usd")
        if marker_path.exists():
            center = basecamp.get("center", {"x": 0.0, "y": 0.0})
            _define_translated_reference(
                stage,
                "/World/Basecamp",
                marker_path,
                (float(center["x"]), float(center["y"]), 0.0),
                world_path.parent,
            )

    UsdGeom.Xform.Define(stage, "/World/Minerals")
    for mineral in minerals:
        mtype = str(mineral["type"])
        marker_path = marker_dir / "tier2_mineral" / f"{mtype}.usd"
        if not marker_path.exists():
            continue
        pos = mineral["position"]
        wrapper, _ = _define_translated_reference(
            stage,
            f"/World/Minerals/{mtype}_{int(mineral['id']):04d}",
            marker_path,
            (float(pos["x"]), float(pos["y"]), float(pos["z"])),
            world_path.parent,
        )
        # Physics: 충돌 + rigid body → 로버 그리퍼로 pick & place 가능
        wprim = wrapper.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        UsdPhysics.CollisionAPI.Apply(wprim)
        UsdPhysics.MeshCollisionAPI.Apply(wprim).CreateApproximationAttr().Set("convexHull")
        UsdPhysics.MassAPI.Apply(wprim).CreateMassAttr().Set(0.3)  # 0.3 kg

    build_light_rig(stage)
    stage.GetRootLayer().customLayerData = {"generated_at": generated_at}
    stage.GetRootLayer().Save()


def maybe_export_usd(terrain_dir, terrain_id, hm, xs, ys, rocks,
                     minerals, basecamp, epic_obstacles, generated_at) -> bool:
    """terrain_only.usd / rocks_merged.usd + master scene 조립.

    master scene 정책: per-terrain 보존 worlds/<terrain_id>.usd
    + 최신 alias worlds/mars_exploration_world.usd (둘 다).
    pxr(Isaac Sim 런타임) 없으면 전부 건너뜀 — npy/json은 그대로 유효.
    """
    if not PXR_AVAILABLE:
        print("[usd] pxr 미가용 (plain python3) — USD 건너뜀. "
              "isaac-python으로 재실행 시 USD가 생성됨.")
        return False

    s = CFG["mesh_stride"]
    terrain_usd = terrain_dir / "terrain_only.usd"
    rocks_usd = terrain_dir / "rocks_merged.usd"
    epic_obstacles_usd = terrain_dir / "epic_obstacles.usd"
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    per_terrain_world = WORLDS_DIR / f"{terrain_id}.usd"
    latest_world = WORLDS_DIR / "mars_exploration_world.usd"
    try:
        export_terrain_usd(hm[::s, ::s], xs[::s], ys[::s], terrain_usd,
                           TEXTURE_DIR if TEXTURE_DIR.exists() else None,
                           float(hm.min()), float(hm.max()))
        export_rocks_usd(rocks, rocks_usd, lambda x, y: sample(hm, x, y))
        export_epic_obstacles_usd(
            epic_obstacles, epic_obstacles_usd, lambda x, y: sample(hm, x, y))
        # master scene — terrain별 보존본
        compose_world(per_terrain_world, terrain_usd, rocks_usd, epic_obstacles_usd,
                      MARKERS_DIR, minerals, generated_at, basecamp=basecamp)
        # 최신 alias — worlds/ 동일 폴더라 상대 ref 그대로 유효 → byte-copy
        shutil.copyfile(per_terrain_world, latest_world)
    except Exception as exc:
        # USD는 옵션 — 실패해도 I1 핵심 데이터 파일(npy/json)은 그대로 유효.
        print(f"[usd] USD 익스포트 실패 — npy/json 데이터 파일은 정상 생성됨. "
              f"({type(exc).__name__}: {exc})")
        return False
    print(f"[usd] terrain_only.usd / rocks_merged.usd / epic_obstacles.usd "
          f"(시각 메시 {GRID // s}×{GRID // s}, PBR 화성 텍스처, 경계 충돌벽 4면)")
    print(f"[usd] master scene → worlds/{terrain_id}.usd (보존) "
          f"+ worlds/mars_exploration_world.usd (최신 alias)")
    return True


# ─── 8. PNG 오버뷰 (옵션 — matplotlib 필요) ─────────────────────────────
def maybe_export_preview(terrain_dir) -> bool:
    """terrain_dir의 npy/meta를 읽어 heightmap.png + preview.png 생성.

    생성 흐름에서도, 기존 terrain 백필에도 쓸 수 있다 (terrain_dir만 주면 됨).
    matplotlib 없으면 건너뜀 — I1 5파일과 무관한 옵션 산출물.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")  # headless 렌더
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except Exception as exc:
        print(f"[png] matplotlib 미가용 — PNG 오버뷰 건너뜀 ({exc})")
        return False

    hm = np.load(terrain_dir / "heightmap.npy")
    og = np.load(terrain_dir / "obstacle_grid.npy")
    meta = json.loads((terrain_dir / "meta.json").read_text(encoding="utf-8"))
    ox, oy = meta["origin"]["x"], meta["origin"]["y"]
    sx, sy = meta["size_m"]
    extent = [ox, ox + sx, oy, oy + sy]

    # (1) heightmap.png — 높이맵 단독 (픽셀 1:1, 장식 없음)
    plt.imsave(str(terrain_dir / "heightmap.png"), hm, cmap="terrain",
               origin="lower")

    # (2) preview.png — 미션 합성 맵 (높이 + obstacle + 광물 + basecamp)
    fig, ax = plt.subplots(figsize=(8, 8), dpi=128)
    im = ax.imshow(hm, cmap="terrain", origin="lower", extent=extent)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="height (m)")
    ax.imshow(np.ma.masked_where(og == 0, og), cmap="Reds", origin="lower",
              extent=extent, alpha=0.5, vmin=0, vmax=1)
    colors = {"blue_mineral": "#3a6ff0", "green_gas": "#3cc35e", "yellow_mineral": "#f0d022"}
    for m in meta.get("minerals", []):
        p = m["position"]
        ax.scatter(p["x"], p["y"], c=colors.get(m["type"], "white"), s=80,
                   edgecolors="black", linewidths=0.8, zorder=5)
    for obs in meta.get("epic_obstacles", []):
        p = obs["position"]
        width, depth = obs["footprint_m"]
        ax.scatter(p["x"], p["y"], marker="s", s=180, c="#7a22ce",
                   edgecolors="white", linewidths=1.0, zorder=7)
        ax.text(p["x"], p["y"], obs["type"], color="white", fontsize=7,
                ha="center", va="center", zorder=8)
    bc = meta["basecamp"]["center"]
    ax.scatter(bc["x"], bc["y"], marker="*", s=420, c="lime",
               edgecolors="black", linewidths=1.0, zorder=6)
    ax.add_patch(Circle((bc["x"], bc["y"]), meta["basecamp"]["radius"],
                         fill=False, color="lime", lw=1.5))
    d = meta.get("difficulty", {})
    ax.set_title(f"{meta.get('terrain_id', '?')}  ·  seed {meta.get('seed', '?')}"
                 f"  ·  difficulty {d.get('score', '?')}"
                 f"  ·  passable {d.get('passable_ratio', 0) * 100:.0f}%")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(str(terrain_dir / "preview.png"))
    plt.close(fig)
    print("[png] heightmap.png + preview.png 생성")
    return True


# ─── 9. I1 적합성 자가검증 ──────────────────────────────────────────────
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
    ap = argparse.ArgumentParser(description="I1 규약 준수 화성 지형 생성기 v3")
    ap.add_argument("--seed", type=int, default=23456)
    ap.add_argument("--terrain-id", default="terrain_00023")
    ap.add_argument("--split", default="train", choices=["train", "holdout"])
    args = ap.parse_args()

    if not re.match(r"^terrain_[0-9]{5}$", args.terrain_id):
        print(f"[error] --terrain-id 는 terrain_NNNNN 형식이어야 함 "
              f"(받음: {args.terrain_id})")
        return 1

    rng = np.random.default_rng(args.seed)
    terrain_dir = GENERATED_DIR / args.terrain_id
    terrain_dir.mkdir(parents=True, exist_ok=True)

    print(f"[v3] {args.terrain_id} 생성 — seed={args.seed}, "
          f"{GRID}×{GRID} @ {RESOLUTION_M} m ({SIZE_M:.0f}×{SIZE_M:.0f} m)")
    xs, ys, xx, yy = build_meshgrid()

    print("[1/7] heightmap (베이스 기복 + 크레이터 + 언덕 + 능선)...")
    hm = generate_heightmap(rng, xx, yy)
    print(f"      높이 범위 {hm.min():.2f} ~ {hm.max():.2f} m")

    print("[2/7] slope...")
    slope = compute_slope_deg(hm)

    print(f"[3/7] rocks (목표 {CFG['rock_count']}, epic obstacle keepout 적용)...")
    rocks = place_rocks(rng, slope)
    print(f"      {len(rocks)}개 배치")

    print("[4/7] obstacle_grid (int8, basecamp + epic obstacles 포함)...")
    obstacle = build_obstacle_grid(slope, rocks)

    print(f"[5/7] minerals (목표 {CFG['mineral_count']}) "
          f"+ spawns (목표 {CFG['spawn_count']})...")
    minerals = place_minerals(rng, hm, slope, rocks, obstacle=obstacle)
    spawns = place_spawns(rng, hm, slope, obstacle, minerals=minerals, rocks=rocks)
    print(f"      minerals {len(minerals)}개, spawns {len(spawns)}개")

    print("[6/7] difficulty + meta + index...")
    difficulty = compute_difficulty(slope, obstacle, rocks)
    epic_obstacles = build_epic_obstacle_meta(hm)
    meta = build_meta(args.terrain_id, args.seed, minerals, spawns,
                      difficulty, epic_obstacles)

    np.save(terrain_dir / "heightmap.npy", hm)
    np.save(terrain_dir / "obstacle_grid.npy", obstacle)
    (terrain_dir / "meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    index, dropped = write_index(args.terrain_id, args.split,
                                 difficulty["score"], args.seed)
    if dropped:
        print(f"      index.json 정리 — 비적합(레거시) 엔트리 제외: {dropped}")

    print("[7/7] USD + master scene + PNG 오버뷰 (옵션)...")
    maybe_export_usd(terrain_dir, args.terrain_id, hm, xs, ys, rocks,
                     minerals, meta["basecamp"], meta["epic_obstacles"],
                     meta["generated_at"])
    maybe_export_preview(terrain_dir)

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
