"""mars_terrain_cfg.py — Isaac Lab 화성 지형 설정 (v2).

두 가지 지형 모드를 제공:
  1. MARS_TERRAIN_CFG        (기본) — 절차적 height-field 생성 (훈련용)
  2. MARS_USD_TERRAIN_CFG           — 기존 terrain_00001~00024.usd 사용 (평가용)

지형 파일 위치: isaac_sim/worlds/terrain_000XX.usd  (24개)
  각 파일은 terrain_only.usd + rocks_merged.usd 로 구성됨.
  worlds/ 아래의 terrain_000XX.usd 는 두 레이어를 합친 씬 파일.

화성 환경:
  - 중력: −3.72 m/s²
  - 마찰: static 0.8 / dynamic 0.6  (화성 regolith)
  - 조명: 화성 황혼 색조 돔 라이트
"""
from __future__ import annotations

import os
import random

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from isaaclab.terrains.height_field import (
    HfDiscreteObstaclesTerrainCfg,
    HfInvertedPyramidSlopedTerrainCfg,
    HfPyramidSlopedTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
)
from isaaclab.utils import configclass

# ── 경로 ────────────────────────────────────────────────────────────────────
_REPO        = os.path.join(os.path.dirname(__file__), "../../..")
_TERRAIN_DIR = os.path.join(_REPO, "isaac_sim/worlds")
_ASSET_DIR   = os.path.join(_REPO, "isaac_sim/assets/generated_terrains")

# 기존 terrain USD 파일 목록 (terrain_00001 ~ terrain_00024)
_TERRAIN_USD_FILES: list[str] = sorted(
    os.path.join(_TERRAIN_DIR, f)
    for f in os.listdir(_TERRAIN_DIR)
    if f.startswith("terrain_") and f.endswith(".usd")
)

# ── 물리 재질 ────────────────────────────────────────────────────────────────
MARS_PHYSICS_MATERIAL = sim_utils.RigidBodyMaterialCfg(
    friction_combine_mode     = "multiply",
    restitution_combine_mode  = "multiply",
    static_friction           = 0.8,   # 화성 regolith
    dynamic_friction          = 0.6,
    restitution               = 0.0,
)

# ── 조명 ─────────────────────────────────────────────────────────────────────
MARS_DOME_LIGHT_CFG = sim_utils.DomeLightCfg(
    intensity = 400.0,
    color     = (0.92, 0.62, 0.38),   # 화성 황혼 색조
)


# ══════════════════════════════════════════════════════════════════════════════
# 1) 절차적 지형 (훈련용) — TerrainGenerator
# ══════════════════════════════════════════════════════════════════════════════

@configclass
class MarsRoughTerrainCfg(HfRandomUniformTerrainCfg):
    """화성 기본 거친 지형."""
    proportion       = 0.25
    size             = (8.0, 8.0)
    horizontal_scale = 0.1
    vertical_scale   = 0.01
    slope_threshold  = 0.75
    noise_range      = (-0.15, 0.15)
    noise_step       = 0.05
    border_width     = 0.25


@configclass
class MarsCraterTerrainCfg(HfInvertedPyramidSlopedTerrainCfg):
    """역피라미드 크레이터 지형 — 로버가 빠질 수 있는 깊은 웅덩이."""
    proportion       = 0.25
    size             = (8.0, 8.0)
    horizontal_scale = 0.1
    vertical_scale   = 0.005
    slope_range      = (0.2, 0.5)   # 11° ~ 29°
    platform_width   = 1.0


@configclass
class MarsSlopeTerrainCfg(HfPyramidSlopedTerrainCfg):
    """경사면 지형."""
    proportion       = 0.25
    size             = (8.0, 8.0)
    horizontal_scale = 0.1
    vertical_scale   = 0.005
    slope_range      = (0.1, 0.4)   # 6° ~ 23°
    platform_width   = 1.0


@configclass
class MarsBumpTerrainCfg(HfDiscreteObstaclesTerrainCfg):
    """돌멩이가 흩어진 울퉁불퉁한 지형."""
    proportion            = 0.25
    size                  = (8.0, 8.0)
    horizontal_scale      = 0.1
    vertical_scale        = 0.005
    num_obstacles         = 30
    obstacle_height_mode  = "choice"
    obstacle_height_range = (0.05, 0.25)
    obstacle_width_range  = (0.1, 0.5)
    platform_width        = 1.0


# TerrainGenerator 설정
MARS_TERRAIN_GEN_CFG = TerrainGeneratorCfg(
    seed              = 42,
    size              = (8.0, 8.0),
    border_width      = 0.5,
    num_rows          = 4,
    num_cols          = 4,
    horizontal_scale  = 0.1,
    vertical_scale    = 0.005,
    slope_threshold   = 0.75,
    use_cache         = False,
    sub_terrains      = {
        "flat":   HfRandomUniformTerrainCfg(
            proportion       = 0.25,
            size             = (8.0, 8.0),
            horizontal_scale = 0.1,
            vertical_scale   = 0.005,
            noise_range      = (-0.02, 0.02),
            noise_step       = 0.02,
            border_width     = 0.25,
        ),
        "rough":  MarsRoughTerrainCfg(),
        "crater": MarsCraterTerrainCfg(),
        "slope":  MarsSlopeTerrainCfg(),
    },
)

# 절차적 TerrainImporter (훈련 기본값)
MARS_TERRAIN_CFG = TerrainImporterCfg(
    prim_path             = "/World/terrain",
    terrain_type          = "generator",
    terrain_generator     = MARS_TERRAIN_GEN_CFG,
    max_init_terrain_level = 0,      # 커리큘럼: 쉬운 지형부터
    collision_group       = -1,
    physics_material      = MARS_PHYSICS_MATERIAL,
    visual_material       = sim_utils.MdlFileCfg(
        mdl_path    = "{NVIDIA_NUCLEUS_DIR}/Materials/Base/Masonry/Concrete_Rough.mdl",
        project_uvw = True,
    ),
    debug_vis             = False,
)


# ══════════════════════════════════════════════════════════════════════════════
# 2) 기존 USD 지형 (평가·시각화용) — terrain_000XX.usd 직접 사용
# ══════════════════════════════════════════════════════════════════════════════

def get_mars_usd_terrain_cfg(
    terrain_idx: int | None = None,
    random_select: bool = True,
) -> TerrainImporterCfg:
    """기존 terrain_000XX.usd 파일을 사용하는 TerrainImporterCfg 반환.

    Args:
        terrain_idx: 0-based 인덱스 (0~23). None이면 random_select 사용.
        random_select: True이면 파일을 랜덤 선택.

    Returns:
        TerrainImporterCfg (terrain_type="usd")
    """
    if not _TERRAIN_USD_FILES:
        raise FileNotFoundError(
            f"terrain USD 파일 없음: {_TERRAIN_DIR}/terrain_000XX.usd"
        )

    if terrain_idx is not None:
        idx = terrain_idx % len(_TERRAIN_USD_FILES)
    elif random_select:
        idx = random.randint(0, len(_TERRAIN_USD_FILES) - 1)
    else:
        idx = 0

    usd_path = _TERRAIN_USD_FILES[idx]

    return TerrainImporterCfg(
        prim_path        = "/World/terrain",
        terrain_type     = "usd",
        usd_path         = usd_path,
        collision_group  = -1,
        physics_material = MARS_PHYSICS_MATERIAL,
        debug_vis        = False,
    )


# 기본 USD 지형 설정 (첫 번째 파일 사용)
MARS_USD_TERRAIN_CFG = get_mars_usd_terrain_cfg(terrain_idx=0, random_select=False)

# 학습 커리큘럼: USD 지형 중 절반(00001~00012)은 훈련, 나머지는 평가용
MARS_TRAIN_TERRAIN_FILES = _TERRAIN_USD_FILES[:12] if len(_TERRAIN_USD_FILES) >= 12 else _TERRAIN_USD_FILES
MARS_EVAL_TERRAIN_FILES  = _TERRAIN_USD_FILES[12:] if len(_TERRAIN_USD_FILES) > 12 else _TERRAIN_USD_FILES


def get_random_train_terrain_cfg() -> TerrainImporterCfg:
    """훈련용 USD 지형 중 랜덤 선택."""
    if not MARS_TRAIN_TERRAIN_FILES:
        return MARS_TERRAIN_CFG  # fallback to procedural
    usd_path = random.choice(MARS_TRAIN_TERRAIN_FILES)
    return TerrainImporterCfg(
        prim_path        = "/World/terrain",
        terrain_type     = "usd",
        usd_path         = usd_path,
        collision_group  = -1,
        physics_material = MARS_PHYSICS_MATERIAL,
        debug_vis        = False,
    )


# ══════════════════════════════════════════════════════════════════════════════
# 유틸: 사용 가능한 지형 파일 목록 출력
# ══════════════════════════════════════════════════════════════════════════════

def list_available_terrains() -> None:
    print(f"[mars_terrain_cfg] 사용 가능한 terrain USD 파일: {len(_TERRAIN_USD_FILES)}개")
    for i, f in enumerate(_TERRAIN_USD_FILES):
        print(f"  [{i:2d}] {os.path.basename(f)}")


if __name__ == "__main__":
    list_available_terrains()
