# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""커리큘럼 stage 2 — 랜덤 위치 작은 큐브 장애물 3개 + 먼 goal.

stage 1 (fixed_obs_env_cfg) 과의 차이:
  stage 1 : 큰 벽 1개가 정면 고정, goal 6~7m
  stage 2 : 작은 큐브 3개가 로봇~goal 경로에 랜덤 배치, goal 10~12m
            → 정책이 '여러 장애물을 연속으로 우회'하는 법을 배운다

레이아웃 (로봇 기준 +x 방향):
  [스폰] ─> [큐브 3개: 0.5m각·0.3m높, x 2.5~8.5m / y ±2.5m 랜덤] ─> [goal 10~12m]

⚠ 큐브 높이 0.3m 주의:
  이 로버는 로커-보기(rocker-bogie) 서스펜션이라 낮은 장애물을 '타고 넘을'
  수 있다. 학습/재생에서 로봇이 큐브를 피하지 않고 올라타면 → 회피 학습이
  안 된 것이므로 HfThreeCubeTerrainCfg.cube_height 를 0.6~0.8m 로 키울 것.

goal 침범 방지:
  큐브 x ∈ [2.5, 8.5], goal x ∈ [10, 12] → x 구간이 안 겹쳐 가릴 수 없다.

stage 1 파일(fixed_obs_env_cfg.py)은 건드리지 않는다 — 스폰 설정만 import.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.terrains.height_field import HfDiscreteObstaclesTerrainCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass

from isaaclab.managers import RewardTermCfg as RewTerm

from . import mdp
from .avoid_env_cfg import AvoidEnvCfg, AvoidSceneCfg, CommandsCfg, RewardsCfg
from .fixed_obs_env_cfg import FixedObsEventCfg


# ---------------------------------------------------------------------------
# 큐브 3개 terrain 함수 + Cfg
# ---------------------------------------------------------------------------
@height_field_to_mesh
def three_cube_terrain(difficulty: float, cfg) -> np.ndarray:
    """로봇~goal 경로에 작은 큐브 num_cubes 개를 랜덤 위치로 생성한다.

    좌표 규칙: 행(row) = world +x (전방), 열(col) = world +y (좌측).
    전방 x 구간을 num_cubes 등분해 구간마다 큐브 1개씩 — x 끼리 안 겹치고
    경로 전체에 고르게 깔린다. 각 큐브의 정확한 x·y 는 랜덤.
    반환: int16 height field (단위: vertical_scale).
    """
    # cfg.size / horizontal_scale 를 int() 로 자르면 부동소수점 오차로 셀 수가
    # 1 모자랄 수 있다 → height_field_to_mesh 가 기대하는 shape 과 어긋나
    # broadcast 에러. round() 로 정확한 셀 수를 맞춘다.
    num_r = round(cfg.size[0] / cfg.horizontal_scale)
    num_c = round(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((num_r, num_c), dtype=np.float32)
    cx, cy = num_r // 2, num_c // 2  # 타일 중앙 = 로봇 스폰

    half = max(1, round((cfg.cube_size / 2.0) / cfg.horizontal_scale))  # 큐브 절반(셀)
    h = int(cfg.cube_height / cfg.vertical_scale)

    # 전방 x 범위를 num_cubes 등분 → 구간마다 큐브 1개, y 는 랜덤.
    edges = np.linspace(cfg.cube_x_range[0], cfg.cube_x_range[1], cfg.num_cubes + 1)
    for i in range(cfg.num_cubes):
        x = np.random.uniform(edges[i], edges[i + 1])
        y = np.random.uniform(*cfg.cube_y_range)
        rc = cx + int(x / cfg.horizontal_scale)
        cc = cy + int(y / cfg.horizontal_scale)
        r0, r1 = max(0, rc - half), min(num_r, rc + half)
        c0, c1 = max(0, cc - half), min(num_c, cc + half)  # 타일 경계 클립
        hf[r0:r1, c0:c1] = h
    return np.rint(hf).astype(np.int16)


@configclass
class HfThreeCubeTerrainCfg(HfDiscreteObstaclesTerrainCfg):
    """랜덤 위치 작은 큐브 3개 height-field 설정."""

    function: Callable = three_cube_terrain
    num_cubes: int = 3                                  # 큐브 개수
    cube_height: float = 0.3                            # 큐브 높이 (m) — ⚠ 위 주석 참고
    cube_size: float = 0.5                              # 큐브 한 변 footprint (m)
    cube_x_range: tuple[float, float] = (2.5, 8.5)      # 큐브 전방 배치 범위 (m)
    cube_y_range: tuple[float, float] = (-2.5, 2.5)     # 큐브 좌우 배치 범위 (m)
    # 부모(HfDiscreteObstaclesTerrainCfg) 더미 필드 — 이 terrain 에선 미사용.
    obstacle_width_range: tuple[float, float] = (0.5, 0.5)
    obstacle_height_range: tuple[float, float] = (0.4, 0.4)
    num_obstacles: int = 0


STAGE2_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(36.0, 24.0),       # 전방 36m (goal 12m + 주행 여유) × 폭 24m
    border_width=2.0,
    num_rows=4,
    num_cols=4,              # 4×4 = 16타일 — 타일마다 다른 랜덤 배치
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={"three_cube": HfThreeCubeTerrainCfg(proportion=1.0)},
)


# ---------------------------------------------------------------------------
# Scene — terrain 만 stage 2 용으로 교체
# ---------------------------------------------------------------------------
@configclass
class Stage2SceneCfg(AvoidSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=STAGE2_TERRAIN_CFG,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )


# ---------------------------------------------------------------------------
# Commands — goal 을 stage 1(6~7m) 보다 멀리 (10~12m)
# ---------------------------------------------------------------------------
@configclass
class Stage2CommandsCfg(CommandsCfg):
    target_pose = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=True,
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=(10.0, 12.0),   # 큐브(최대 8.5m) 뒤 충분한 거리
            pos_y=(-0.3, 0.3),
            heading=(-0.1, 0.1),
        ),
    )


# ---------------------------------------------------------------------------
# Rewards — stage 1 보상 8종 + progress(전진율) 보상 추가
# ---------------------------------------------------------------------------
@configclass
class Stage2RewardsCfg(RewardsCfg):
    """stage 1 보상 8종 상속 + 'progress' 추가.

    먼 goal + 큐브 3개에서 로봇이 장애물 앞에 가만히 서는(freeze) local
    optimum 에 빠지는 걸 막는다 — 목표 쪽으로 다가가는 속도에 보상을 줘서
    '머무르면 0, 전진하면 +' 로 만든다. weight 가 핵심 튜닝 노브.
    """

    progress = RewTerm(
        func=mdp.progress_reward,
        weight=10.0,
        params={"command_name": "target_pose"},
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class Stage2EnvCfg(AvoidEnvCfg):
    """커리큘럼 stage 2 — 랜덤 큐브 3개 + 먼 goal."""

    scene: Stage2SceneCfg = Stage2SceneCfg(num_envs=2, env_spacing=36.0)
    events: FixedObsEventCfg = FixedObsEventCfg()           # stage 1 스폰 재사용
    commands: Stage2CommandsCfg = Stage2CommandsCfg()
    rewards: Stage2RewardsCfg = Stage2RewardsCfg()          # progress 보상 추가


@configclass
class Stage2EnvCfg_PLAY(Stage2EnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 36.0
        self.observations.policy.enable_corruption = False
