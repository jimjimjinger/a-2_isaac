# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""terrain_00022 의 obstacle_grid 를 활용한 valid world-xy 샘플링 유틸.

좌표 규약 — **모두 world 좌표** 로 처리한다 (obstacle_grid 가 world 좌표계라).
multi-env 에선 각 env 마다 world center 가 다르므로 호출자가 env_origin 을 넘긴다.

  · is_valid_xy(xy_world, ...) → 그 world 위치가 valid 한지 (bool)
  · sample_valid_xy(n, center_world, ...) → center 주변에서 valid 한 world xy 샘플
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

# terrain_00022 metadata (drive_test 와 동일 정책 — 외부 파일 읽기만).
_OBSTACLE_GRID_PATH = (
    Path(__file__).resolve().parents[3]
    / "isaac_sim" / "assets" / "generated_terrains" / "terrain_00022"
    / "obstacle_grid.npy"
)

# meta.json 기준 — origin=(-25,-25), resolution=0.05m, shape=(1000,1000).
GRID_ORIGIN_X = -25.0
GRID_ORIGIN_Y = -25.0
GRID_RES = 0.05
MAP_HALF_EXTENT = 24.0   # world ±24m 안만 valid (terrain 가장자리 마진 1m).

_cached_grid: torch.Tensor | None = None
_inflated_cache: dict[tuple, torch.Tensor] = {}
_cached_heightmap: torch.Tensor | None = None

_HEIGHTMAP_PATH = (
    Path(__file__).resolve().parents[3]
    / "isaac_sim" / "assets" / "generated_terrains" / "terrain_00022"
    / "heightmap.npy"
)


def get_obstacle_grid(device: torch.device | str = "cpu") -> torch.Tensor:
    """obstacle_grid 를 (rows, cols) bool 텐서로 반환 — 모듈 레벨 캐싱."""
    global _cached_grid
    if _cached_grid is None or _cached_grid.device != torch.device(device):
        try:
            arr = np.load(_OBSTACLE_GRID_PATH)
        except Exception as exc:  # noqa: BLE001
            print(f"[sampling] obstacle_grid 로드 실패: {exc}.  rejection 비활성.")
            _cached_grid = torch.zeros((1000, 1000), dtype=torch.bool,
                                       device=torch.device(device))
            return _cached_grid
        _cached_grid = torch.tensor(arr > 0, dtype=torch.bool,
                                    device=torch.device(device))
    return _cached_grid


def get_heightmap(device: torch.device | str = "cpu") -> torch.Tensor:
    """terrain_00022 의 heightmap.npy 를 (rows, cols) float 텐서로 반환.

    meta.json: shape=(1000,1000), origin=(-25,-25), res=0.05m.  값은 world z.
    grid[i, j] 의 world 위치: x=GRID_ORIGIN_X+j*RES, y=GRID_ORIGIN_Y+i*RES.
    """
    global _cached_heightmap
    if _cached_heightmap is None or _cached_heightmap.device != torch.device(device):
        try:
            arr = np.load(_HEIGHTMAP_PATH)
        except Exception as exc:  # noqa: BLE001
            print(f"[sampling] heightmap 로드 실패: {exc}.  z=0 폴백.")
            _cached_heightmap = torch.zeros((1000, 1000), dtype=torch.float32,
                                            device=torch.device(device))
            return _cached_heightmap
        _cached_heightmap = torch.tensor(arr, dtype=torch.float32,
                                         device=torch.device(device))
    return _cached_heightmap


def terrain_height_at(xy_world: torch.Tensor) -> torch.Tensor:
    """world xy 의 지면 z 를 heightmap 에서 nearest-셀 조회.

    Args:
        xy_world: (..., 2) tensor, world 좌표.
    Returns:
        (...,) tensor, 그 위치의 world z.
    """
    device = xy_world.device
    hm = get_heightmap(device=device)
    rows, cols = hm.shape
    x = xy_world[..., 0]
    y = xy_world[..., 1]
    gj = ((x - GRID_ORIGIN_X) / GRID_RES).long().clamp_(0, cols - 1)
    gi = ((y - GRID_ORIGIN_Y) / GRID_RES).long().clamp_(0, rows - 1)
    return hm[gi, gj]


def get_inflated_grid(clearance_radius: float, device) -> torch.Tensor:
    """obstacle_grid 를 clearance_radius 만큼 dilate(팽창)한 bool 텐서 — 캐시.

    차량 footprint 안에 obstacle 셀이 없는 위치만 valid 로 보려고 max_pool2d
    로 obstacle 영역을 확장한다.  bool 결과: True = 그 셀은 차량 footprint 안에
    obstacle 이 있어 spawn/주행 불가.
    """
    device = torch.device(device)
    key = (round(clearance_radius, 3), str(device))
    if key in _inflated_cache:
        return _inflated_cache[key]
    raw = get_obstacle_grid(device=device).float()
    n = max(1, int(math.ceil(clearance_radius / GRID_RES)))   # 셀 단위 반경
    kernel = 2 * n + 1
    dilated = F.max_pool2d(
        raw.unsqueeze(0).unsqueeze(0),
        kernel_size=kernel, stride=1, padding=n,
    ).squeeze() > 0
    _inflated_cache[key] = dilated
    return dilated


def is_valid_xy(
    xy_world: torch.Tensor,
    basecamp_radius: float = 6.5,
    map_half_extent: float = MAP_HALF_EXTENT,
    clearance_radius: float = 0.0,
) -> torch.Tensor:
    """world xy 가 (베이스캠프 밖 ∧ 장애물 밖 ∧ 맵 안) 인지.

    Args:
        xy_world: (..., 2) tensor, **world** 좌표.
        basecamp_radius: 베이스캠프(world 원점) 회피 반경 (m).
        map_half_extent: 맵 가장자리 마진 — |x|,|y| < 이 값.
        clearance_radius: 차량 footprint 반경 (m).  0 이면 한 셀만 검사 (기존 동작).
            >0 이면 그 반경 안의 어떤 셀에도 obstacle 이 있으면 reject — 차량
            바퀴까지 obstacle 안 박히게 함.
    Returns:
        (...,) bool tensor.
    """
    device = xy_world.device
    if clearance_radius > 0.0:
        grid = get_inflated_grid(clearance_radius, device=device)
    else:
        grid = get_obstacle_grid(device=device)
    rows, cols = grid.shape

    x = xy_world[..., 0]
    y = xy_world[..., 1]

    in_box = (x.abs() < map_half_extent) & (y.abs() < map_half_extent)
    out_bc = (x * x + y * y) > (basecamp_radius * basecamp_radius)

    gj = ((x - GRID_ORIGIN_X) / GRID_RES).long().clamp_(0, cols - 1)
    gi = ((y - GRID_ORIGIN_Y) / GRID_RES).long().clamp_(0, rows - 1)
    free = ~grid[gi, gj]

    return in_box & out_bc & free


def sample_valid_xy(
    n: int,
    device: torch.device | str,
    center_xy: torch.Tensor | None = None,
    sample_radius: float = 8.0,
    basecamp_radius: float = 6.5,
    map_half_extent: float = MAP_HALF_EXTENT,
    clearance_radius: float = 0.0,
    min_dist_to_ref: float = 0.0,
    max_dist_to_ref: float = float("inf"),
    ref_xy: torch.Tensor | None = None,
    fallback_world: tuple[float, float] = (10.0, 0.0),
    oversample_factor: int = 20,
    max_rounds: int = 5,
) -> torch.Tensor:
    """벡터화 rejection 샘플링 — **world 좌표** n 개 반환.

    각 슬롯 i 는 center_xy[i] (또는 center_xy 없으면 (0,0)) 중심에서 ±sample_radius
    박스 안의 world xy 를 추첨한다.  obstacle_grid / basecamp / map 경계 reject.

    Args:
        n: 결과 슬롯 수.
        device: 결과 텐서 device.
        center_xy: (n, 2) world 또는 None — 슬롯별 샘플링 중심.  None 이면 (0,0).
        sample_radius: 중심 기준 샘플링 박스 ±반경 (m).
        basecamp_radius: 베이스캠프 회피 반경.
        map_half_extent: 맵 경계 마진.
        min_dist_to_ref / max_dist_to_ref: ref_xy 와의 거리 제약 (보통 spawn↔goal).
        ref_xy: (n, 2) world 또는 None.
        fallback_world: rejection 다 실패 시 슬롯에 채울 world 좌표.
        oversample_factor: 후보 oversample 배율.
        max_rounds: 재시도 횟수.
    Returns:
        (n, 2) tensor — **world** 좌표.
    """
    device = torch.device(device)
    if center_xy is None:
        center = torch.zeros(n, 2, device=device)
    else:
        center = center_xy.to(device)

    out = torch.empty(n, 2, device=device)
    accepted = torch.zeros(n, dtype=torch.bool, device=device)

    for _ in range(max_rounds):
        need_mask = ~accepted
        n_need = int(need_mask.sum().item())
        if n_need == 0:
            break

        # 미수락 슬롯 기준으로 oversample 후보 만들고, 그 슬롯 중심 기준 박스에서 뽑음.
        slot_idx = torch.where(need_mask)[0]                       # (n_need,)
        slot_center = center[slot_idx]                             # (n_need, 2)
        # 슬롯마다 oversample_factor 개 후보 → 총 m 개.
        m = n_need * oversample_factor
        # 각 후보가 어느 슬롯 거인지 매핑: [slot0]*F + [slot1]*F + ...
        slot_map = slot_idx.repeat_interleave(oversample_factor)   # (m,)
        center_per_cand = slot_center.repeat_interleave(oversample_factor, dim=0)  # (m, 2)
        noise = torch.empty(m, 2, device=device).uniform_(-sample_radius, sample_radius)
        cand = center_per_cand + noise

        valid = is_valid_xy(cand, basecamp_radius=basecamp_radius,
                            map_half_extent=map_half_extent,
                            clearance_radius=clearance_radius)
        if ref_xy is not None and (min_dist_to_ref > 0.0 or max_dist_to_ref < float("inf")):
            ref_per_cand = ref_xy[slot_map]   # (m, 2)
            d = (cand - ref_per_cand).norm(dim=-1)
            valid = valid & (d >= min_dist_to_ref) & (d <= max_dist_to_ref)

        if not bool(valid.any()):
            continue

        # 슬롯별 첫 valid 한 후보 한 개씩 채택 — 간단한 dedup.
        for sid_int in slot_idx.tolist():
            if accepted[sid_int]:
                continue
            sid_t = torch.tensor(sid_int, device=device)
            ok_in_slot = valid & (slot_map == sid_t)
            if not bool(ok_in_slot.any()):
                continue
            idx = int(torch.where(ok_in_slot)[0][0].item())
            out[sid_int] = cand[idx]
            accepted[sid_int] = True

    if not accepted.all():
        fb = torch.tensor(fallback_world, device=device)
        out[~accepted] = fb
    return out
