# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""평지 + 큐브 장애물 1개 지형 — 단일 height-field 메시.

avoid_test/rover_avoid/fixed_obs_env_cfg.py 의 fixed_obstacle_terrain 을
'장애물 1개·랜덤 없음' 으로 단순화한 것.

RayCaster 는 메시 1개만 인식하므로, 장애물을 별도 prim 이 아닌 지형 메시
자체에 '높이 돌출' 로 포함한다.  장애물 크기는 avoid_test 와 동일하게
0.5m(두께) × 0.5m(폭) × 0.3m(높이).
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains.height_field import HfDiscreteObstaclesTerrainCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# 장애물 1개 height-field 함수
# ---------------------------------------------------------------------------
@height_field_to_mesh
def one_obstacle_terrain(difficulty: float, cfg) -> np.ndarray:
    """타일 중앙(차량 스폰) 기준 +x 로 obstacle_distance 앞에 큐브 1개.

    좌표 규칙 (avoid_test 와 동일): 행(row)=world +x(차량 전방), 열(col)=world +y.
    반환: int16 height field (단위: vertical_scale).
    """
    num_r = int(cfg.size[0] / cfg.horizontal_scale)
    num_c = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((num_r, num_c), dtype=np.float32)

    cx, cy = num_r // 2, num_c // 2  # 차량 스폰 = 타일 중앙

    d = int(cfg.obstacle_distance / cfg.horizontal_scale)             # 전방 거리(셀)
    t = max(1, int(cfg.obstacle_thickness / cfg.horizontal_scale))    # x 두께(셀)
    w = max(1, int((cfg.obstacle_width / 2.0) / cfg.horizontal_scale))  # y 반폭(셀)
    h = int(cfg.obstacle_height / cfg.vertical_scale)                 # 높이

    # 전방 d 셀, 좌우 중앙에 큐브 생성.
    hf[cx + d : cx + d + t, cy - w : cy + w] = h
    return np.rint(hf).astype(np.int16)


@configclass
class HfOneObstacleCfg(HfDiscreteObstaclesTerrainCfg):
    """평지 + 정면 고정 큐브 1개 height-field 설정."""

    function: Callable = one_obstacle_terrain
    # TerrainGeneratorCfg 가 자동 주입: size, horizontal_scale, vertical_scale.
    obstacle_distance: float = 3.5    # 차량 정면 기준 거리 (m)
    obstacle_thickness: float = 0.5   # x 방향 두께 (m) — avoid_test 와 동일
    obstacle_width: float = 0.5       # y 방향 폭 (m) — avoid_test 와 동일
    obstacle_height: float = 0.3      # 높이 (m) — avoid_test 와 동일
    # 부모(HfDiscreteObstaclesTerrainCfg) 필수 필드 — 이 terrain 에선 안 쓰지만
    # Isaac Lab validation 이 MISSING 을 허용하지 않아 더미값 필요.
    obstacle_width_range: tuple[float, float] = (0.5, 0.5)
    obstacle_height_range: tuple[float, float] = (0.3, 0.3)
    num_obstacles: int = 0


# 단일 타일(1×1) — env 1개용.
ONE_OBSTACLE_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(20.0, 20.0),
    border_width=2.0,
    num_rows=1,
    num_cols=1,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={"one_obs": HfOneObstacleCfg(proportion=1.0)},
)
