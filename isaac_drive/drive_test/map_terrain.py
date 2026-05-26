# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""terrain_00022 지형+바위 통합 height-field.

Isaac Lab RayCaster 는 메시 1개만 스캔하므로, 지형(heightmap.npy)과
바위(obstacle_grid.npy)를 단일 height-field 메시로 합친다.  바위 셀을
ROCK_BUMP_M 만큼 솟구쳐 RayCaster 가 지형과 바위를 한 메시에서 함께
본다 → detector 가 '갑자기 솟은 돌출'로 바위를 판정할 수 있다.

  · heightmap.npy     (1000×1000 float32, m)  — 매끈한 지형 표고
  · obstacle_grid.npy (1000×1000 int8 0/1)    — 바위 + 베이스캠프 영역

obstacle_grid 에는 바위뿐 아니라 베이스캠프(중앙 ~63㎡ 큰 덩어리)도
들어 있다.  형태학적 opening 으로 큰 덩어리(베이스캠프)를 떼어내고
작은 덩어리(바위)만 솟구친다 — 베이스캠프는 평지로 남아 차량 스폰
지점이 된다.

좌표: heightmap/obstacle_grid 는 [y행, x열], [0,0]=좌하단.  height-field
는 [행=+x, 열=+y] 규약이라 .T 로 전치해 맞춘다.

⚠️ npy 자산은 drive_test 밖 — 경로 참조만, 수정하지 않는다.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np

from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains.height_field import HfDiscreteObstaclesTerrainCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass

# --- 자산 경로 (drive_test/map_terrain.py → parents[2] = a2_isaac) ---
_TERRAIN_DIR = (
    Path(__file__).resolve().parents[2]
    / "isaac_sim" / "assets" / "generated_terrains" / "terrain_00022"
)
HEIGHTMAP_PATH = str(_TERRAIN_DIR / "heightmap.npy")
OBSTACLE_GRID_PATH = str(_TERRAIN_DIR / "obstacle_grid.npy")

# 바위 돌출 높이 (m). 지형의 국소 비평탄성(0.4m 반경에서 최대 ~0.08m)보다
# 충분히 커서 detector 가 바위를 확실히 잡고(임계 0.15m), 동시에 위에 겹치는
# rocks_merged.usd 실제 바위(0.3~1.0m) 안에 대체로 가려질 만큼 낮게 둔다.
ROCK_BUMP_M = 0.4
# 베이스캠프 분리용 opening 반경 (셀, 0.1m/셀). 바위(최대 ~1m)보다 크고
# 베이스캠프(~9m)보다 작게 → 큰 덩어리만 남겨 돌출 마스크에서 뺀다.
_BASECAMP_OPEN_R = 15


def _block_pool(a: np.ndarray, factor: int, how: str) -> np.ndarray:
    """factor×factor 블록 풀링 다운샘플 (how='mean' 또는 'max')."""
    if factor <= 1:
        return a
    rows, cols = a.shape
    r2, c2 = rows // factor, cols // factor
    blk = a[: r2 * factor, : c2 * factor].reshape(r2, factor, c2, factor)
    return blk.mean(axis=(1, 3)) if how == "mean" else blk.max(axis=(1, 3))


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    """반경 r(원형 SE) 이진 팽창."""
    m = np.asarray(mask, dtype=bool)
    out = m.copy()
    h, w = m.shape
    r2 = r * r
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r2:
                continue
            sy = slice(max(0, -dy), h - max(0, dy))
            dy_ = slice(max(0, dy), h - max(0, -dy))
            sx = slice(max(0, -dx), w - max(0, dx))
            dx_ = slice(max(0, dx), w - max(0, -dx))
            out[dy_, dx_] |= m[sy, sx]
    return out


def _opening(mask: np.ndarray, r: int) -> np.ndarray:
    """침식→팽창. 반경 r 보다 가는 덩어리는 사라지고 큰 덩어리만 남는다."""
    eroded = ~_dilate(~np.asarray(mask, dtype=bool), r)
    return _dilate(eroded, r)


def _fit_shape(a: np.ndarray, num_r: int, num_c: int) -> np.ndarray:
    """a 를 (num_r, num_c) 로 크롭/가장자리 패딩 (height_field 정확 shape 요구)."""
    if a.shape == (num_r, num_c):
        return a
    out = np.zeros((num_r, num_c), dtype=a.dtype)
    r = min(num_r, a.shape[0])
    c = min(num_c, a.shape[1])
    out[:r, :c] = a[:r, :c]
    if r < num_r:
        out[r:, :c] = out[r - 1 : r, :c]
    if c < num_c:
        out[:, c:] = out[:, c - 1 : c]
    return out


@height_field_to_mesh
def terrain_00022_hf(difficulty: float, cfg) -> np.ndarray:
    """terrain_00022 지형 + 바위 돌출을 합친 height-field.

    반환: int16 height field (단위: cfg.vertical_scale).
    """
    hm = np.load(HEIGHTMAP_PATH).astype(np.float32)   # (1000,1000) [y,x] m
    og = np.load(OBSTACLE_GRID_PATH) > 0              # (1000,1000) [y,x]

    # [y,x] → [x,y] (height-field 행=+x 규약)
    hm = np.ascontiguousarray(hm.T)
    og = np.ascontiguousarray(og.T)

    # cfg 해상도로 다운샘플 (지형=평균, 장애물=최대 풀링으로 보존)
    num_r = int(round(cfg.size[0] / cfg.horizontal_scale))
    num_c = int(round(cfg.size[1] / cfg.horizontal_scale))
    factor = max(1, round(hm.shape[0] / num_r))
    hm = _block_pool(hm, factor, "mean")
    og = _block_pool(og.astype(np.float32), factor, "max") > 0.5

    # 베이스캠프(큰 덩어리)를 떼어내고 바위(작은 덩어리)만 돌출 마스크로.
    basecamp = _opening(og, _BASECAMP_OPEN_R)
    rock_mask = og & ~basecamp

    # 지형 + 바위 돌출 → int16 height field.
    h = hm + ROCK_BUMP_M * rock_mask.astype(np.float32)
    h = _fit_shape(h, num_r, num_c)
    return np.rint(h / cfg.vertical_scale).astype(np.int16)


@configclass
class HfTerrain00022Cfg(HfDiscreteObstaclesTerrainCfg):
    """terrain_00022 지형+바위 통합 height-field 설정."""

    function: Callable = terrain_00022_hf
    # 부모(HfDiscreteObstaclesTerrainCfg) 필수 필드 — 이 terrain 에선
    # 안 쓰지만 Isaac Lab validation 이 MISSING 을 막아 더미값을 둔다.
    obstacle_width_range: tuple[float, float] = (0.5, 0.5)
    obstacle_height_range: tuple[float, float] = (0.3, 0.3)
    num_obstacles: int = 0


# 단일 타일(1×1) — 50×50m 맵, env 1개용.
# horizontal_scale 0.1m → 500×500 height-field (heightmap 0.05m 의 2× 다운샘플).
MAP_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(50.0, 50.0),
    border_width=0.0,
    num_rows=1,
    num_cols=1,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={"map": HfTerrain00022Cfg(proportion=1.0)},
)
