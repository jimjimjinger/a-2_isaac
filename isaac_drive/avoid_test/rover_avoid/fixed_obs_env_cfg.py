# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""고정 장애물 1개 환경 — 커리큘럼 첫 단계.

레이아웃 (로봇 기준 +x 방향):
  [스폰, 정면] ──3.5m──> [큐브 0.5×0.5×0.3m] ──> [goal 6.5m, 로봇 중심선상]

장애물 좌우(y) 위치는 **3 고정 케이스**뿐 (``_CASE_LATERALS``):
  1) 정중앙       — 로봇 중심선상
  2) 살짝 왼쪽    — 중심선보다 +0.4 m
  3) 살짝 오른쪽  — 중심선보다 −0.4 m
``num_cols=3`` 이라 3 타일이 케이스를 1개씩 맡아 **비율 1:1:1** 로 스폰된다.

goal 은 로봇 **중심선상**(정면)에 고정하고, 로봇도 정면(yaw=0)으로 스폰한다.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.terrains.height_field import HfDiscreteObstaclesTerrainCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass

from . import mdp
from .avoid_env_cfg import AvoidEnvCfg, AvoidSceneCfg, CommandsCfg, EventCfg


# ---------------------------------------------------------------------------
# 고정 장애물 terrain 함수 + Cfg
# ---------------------------------------------------------------------------
# 장애물 좌우 위치 3 고정 케이스 (m). +좌 / −우 — 로봇 중심선 기준.
# "살짝" 오프셋(0.4 m)을 바꾸려면 _SLIGHT 만 수정.
_SLIGHT = 0.4
_CASE_LATERALS = (0.0, +_SLIGHT, -_SLIGHT)  # 정중앙 / 살짝 왼쪽 / 살짝 오른쪽

# 타일마다 케이스를 순환 배정하기 위한 카운터.
# terrain 생성 시 타일당 1회 호출 → num_cols=3 이면 3 타일이 케이스 1개씩.
_tile_counter = 0


@height_field_to_mesh
def fixed_obstacle_terrain(difficulty: float, cfg) -> np.ndarray:
    """타일 전방 고정 거리에 큐브 장애물 1개를 생성한다.

    좌표 규칙: 행(row) = world +x (로봇 전방), 열(col) = world +y (좌측).
    좌우(y) 위치는 ``_CASE_LATERALS`` 3 케이스를 타일마다 순환 배정한다
    → 정중앙 / 살짝 왼쪽 / 살짝 오른쪽 이 비율 1:1:1.
    반환: int16 height field (단위: vertical_scale).
    """
    global _tile_counter
    lateral = _CASE_LATERALS[_tile_counter % len(_CASE_LATERALS)]
    _tile_counter += 1

    num_r = int(cfg.size[0] / cfg.horizontal_scale)
    num_c = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((num_r, num_c), dtype=np.float32)

    cx, cy = num_r // 2, num_c // 2  # 로봇 스폰 = 타일 중앙

    d = int(cfg.obstacle_distance / cfg.horizontal_scale)
    t = max(1, int(cfg.obstacle_thickness / cfg.horizontal_scale))
    w = max(1, int((cfg.obstacle_width / 2.0) / cfg.horizontal_scale))
    off = int(lateral / cfg.horizontal_scale)
    h = int(cfg.obstacle_height / cfg.vertical_scale)

    # 전방 d 셀 · 좌우 off 셀 이동한 위치에 큐브 생성.
    c = cy + off
    c0, c1 = max(0, c - w), min(num_c, c + w)
    hf[cx + d : cx + d + t, c0:c1] = h
    return np.rint(hf).astype(np.int16)


@configclass
class HfFixedObstacleTerrainCfg(HfDiscreteObstaclesTerrainCfg):
    """전방 거리 고정·좌우 3케이스 큐브 장애물 height-field 설정."""

    function: Callable = fixed_obstacle_terrain
    # TerrainGeneratorCfg 가 자동 주입: size, horizontal_scale, vertical_scale
    obstacle_distance: float = 3.5    # 로봇 정면 기준 (m)
    obstacle_thickness: float = 0.5   # x 방향 두께 (m)
    obstacle_width: float = 0.5       # y 방향 폭 (m)
    obstacle_height: float = 0.3      # 높이 (m)
    # 부모(HfDiscreteObstaclesTerrainCfg) 필드 — 이 terrain 에선 사용 안 하지만
    # Isaac Lab validation 이 MISSING 을 허용하지 않아 더미값 필요.
    obstacle_width_range: tuple[float, float] = (0.5, 0.5)
    obstacle_height_range: tuple[float, float] = (0.3, 0.3)
    num_obstacles: int = 0


# 1×3 타일 — 열(col) 3개가 케이스(정중앙/왼/오)를 1개씩 맡아 비율 1:1:1.
# 학습 시 env 들이 3 열에 고르게 분산 → 3 케이스 등비율 스폰.
FIXED_OBS_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(20.0, 20.0),
    border_width=2.0,
    num_rows=1,
    num_cols=3,              # 케이스 3개 → 타일 3개, 1:1:1
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={"fixed_obs": HfFixedObstacleTerrainCfg(proportion=1.0)},
)


# ---------------------------------------------------------------------------
# Scene — terrain 만 교체, 나머지(RayCaster·ContactSensor) 상속
# ---------------------------------------------------------------------------
@configclass
class FixedObsSceneCfg(AvoidSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=FIXED_OBS_TERRAIN_CFG,
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
# Events — 고정 스폰: 정면(yaw=0), 좌우(y)=0 → goal 과 중심선 일치
# ---------------------------------------------------------------------------
@configclass
class FixedObsEventCfg(EventCfg):
    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.2, 0.2),    # 시작 거리만 살짝 변동
                "y": (0.0, 0.0),     # 좌우 0 — goal·장애물과 중심선 일치
                "yaw": (0.0, 0.0),   # 정면 +x 고정
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


# ---------------------------------------------------------------------------
# Commands — goal 을 로봇 중심선상(정면) 6.5m 에 고정
# ---------------------------------------------------------------------------
@configclass
class FixedObsCommandsCfg(CommandsCfg):
    target_pose = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=True,
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=(6.5, 6.5),    # 장애물(3.5m) 뒤 정면 6.5m 고정
            pos_y=(0.0, 0.0),    # 로봇 중심선상 — 직선 정면
            heading=(0.0, 0.0),
        ),
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class FixedObsEnvCfg(AvoidEnvCfg):
    """고정 장애물 1개 (3 케이스) — 단순 커리큘럼 첫 단계."""

    scene: FixedObsSceneCfg = FixedObsSceneCfg(num_envs=2, env_spacing=20.0)
    events: FixedObsEventCfg = FixedObsEventCfg()
    commands: FixedObsCommandsCfg = FixedObsCommandsCfg()


@configclass
class FixedObsEnvCfg_PLAY(FixedObsEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 9   # 3 케이스 × 3 → 등비율 재생
        self.scene.env_spacing = 20.0
        self.observations.policy.enable_corruption = False
