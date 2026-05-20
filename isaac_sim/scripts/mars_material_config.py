"""공유되는 화성 텍스처/재질 설정값."""

from __future__ import annotations

from pathlib import Path

from save_utils import ASSETS_DIR


BASE_DIR = Path(__file__).resolve().parent
TEXTURE_DIR = ASSETS_DIR / "textures" / "Mars"
SEED = 20260520
TEXTURE_SIZE = 1024

BASE_COLOR_RANGE = (
    (0.32, 0.18, 0.12),
    (0.46, 0.26, 0.16),
)
MACRO_NOISE_STRENGTH = 0.18
DETAIL_NOISE_STRENGTH = 0.08
MICRO_NOISE_STRENGTH = 0.03
CRATER_MASK_STRENGTH = 0.20
ROUGHNESS_RANGE = (0.72, 0.90)
NORMAL_STRENGTH = 6.6

TERRAIN_UV_REPEAT = 3.0
MACRO_UV_REPEAT = 0.9
DETAIL_UV_REPEAT = 6.5
MICRO_UV_REPEAT = 18.0
