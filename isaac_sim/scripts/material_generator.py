"""화성 텍스처를 베이크하고 재질 시스템이 읽을 파일을 만든다."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from mars_material_config import (
    BASE_COLOR_RANGE,
    CRATER_MASK_STRENGTH,
    DETAIL_NOISE_STRENGTH,
    DETAIL_UV_REPEAT,
    MACRO_NOISE_STRENGTH,
    MACRO_UV_REPEAT,
    MICRO_NOISE_STRENGTH,
    MICRO_UV_REPEAT,
    NORMAL_STRENGTH,
    ROUGHNESS_RANGE,
    SEED as DEFAULT_SEED,
    TEXTURE_DIR,
    TEXTURE_SIZE,
)


_RESAMPLE_BICUBIC = getattr(Image, "Resampling", Image).BICUBIC


def _normalize01(arr: np.ndarray) -> np.ndarray:
    arr = arr.astype(np.float32, copy=False)
    lo = float(arr.min())
    hi = float(arr.max())
    if hi - lo < 1e-8:
        return np.zeros_like(arr, dtype=np.float32)
    return (arr - lo) / (hi - lo)


def _box_blur(arr: np.ndarray, passes: int = 1) -> np.ndarray:
    out = arr.astype(np.float32, copy=True)
    for _ in range(passes):
        out = (
            out
            + np.roll(out, 1, axis=0)
            + np.roll(out, -1, axis=0)
            + np.roll(out, 1, axis=1)
            + np.roll(out, -1, axis=1)
        ) / 5.0
    return out


def _resized_noise(rng: np.random.Generator, low_res: int, size: int, channels: int = 1) -> np.ndarray:
    if channels == 1:
        noise = (rng.random((low_res, low_res)) * 255).astype(np.uint8)
    else:
        noise = (rng.random((low_res, low_res, channels)) * 255).astype(np.uint8)
    image = Image.fromarray(noise)
    return np.asarray(image.resize((size, size), _RESAMPLE_BICUBIC), dtype=np.float32) / 255.0


def _multi_scale_noise(
    seed: int,
    size: int,
    base_res: int,
    octaves: int,
    persistence: float,
    lacunarity: float,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    noise = np.zeros((size, size), dtype=np.float32)
    amplitude = 1.0
    frequency = 1.0
    amplitude_sum = 0.0

    for octave in range(octaves):
        low_res = max(4, int(round(base_res * frequency)))
        octave_noise = _resized_noise(rng, low_res, size, channels=1)
        octave_noise = _box_blur(octave_noise, passes=1 + (octave % 2))
        noise += amplitude * octave_noise
        amplitude_sum += amplitude
        amplitude *= persistence
        frequency *= lacunarity

    if amplitude_sum > 0.0:
        noise /= amplitude_sum
    return _normalize01(noise)


def _make_crater_mask(seed: int, size: int) -> np.ndarray:
    rng = np.random.default_rng(seed + 913)
    mask = np.zeros((size, size), dtype=np.float32)
    x = np.linspace(-1.0, 1.0, size, dtype=np.float32)
    y = np.linspace(-1.0, 1.0, size, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    crater_count = int(rng.integers(7, 13))
    for _ in range(crater_count):
        cx = rng.uniform(-0.82, 0.82)
        cy = rng.uniform(-0.82, 0.82)
        radius = rng.uniform(0.06, 0.16)
        depth = rng.uniform(0.35, 0.95)
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        bowl = depth * np.exp(-(dist**2) / (2.0 * (radius * 0.55) ** 2))
        rim = 0.42 * depth * np.exp(-((dist - radius) ** 2) / (2.0 * (radius * 0.18) ** 2))
        mask += bowl + rim

    mask = _box_blur(mask, passes=3)
    mask = _normalize01(mask)
    return np.clip(mask * CRATER_MASK_STRENGTH, 0.0, 1.0)


def _make_height_field(
    seed: int,
    size: int,
    macro_noise: np.ndarray,
    detail_noise: np.ndarray,
    crater_mask: np.ndarray,
) -> np.ndarray:
    x = np.linspace(-1.0, 1.0, size, dtype=np.float32)
    y = np.linspace(-1.0, 1.0, size, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    base = 0.55 * macro_noise + 0.25 * detail_noise + 0.20 * crater_mask
    base += 0.08 * np.sin(1.5 * np.pi * xx + 0.6)
    base += 0.05 * np.cos(1.2 * np.pi * yy - 0.4)
    base = _box_blur(base, passes=2)

    rng = np.random.default_rng(seed + 201)
    hills = np.zeros((size, size), dtype=np.float32)
    hill_count = int(rng.integers(5, 8))
    for _ in range(hill_count):
        cx = rng.uniform(-1.0, 1.0)
        cy = rng.uniform(-1.0, 1.0)
        amp = rng.uniform(0.08, 0.22)
        sigma = rng.uniform(0.12, 0.28)
        dist = (xx - cx) ** 2 + (yy - cy) ** 2
        hills += amp * np.exp(-dist / (2.0 * sigma**2))

    return _normalize01(base + hills)


def _compute_normal_map(height: np.ndarray, strength: float) -> np.ndarray:
    dx = np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)
    dy = np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)
    nx = -dx * strength
    ny = -dy * strength
    nz = np.ones_like(height, dtype=np.float32)
    inv_len = 1.0 / np.maximum(np.sqrt(nx * nx + ny * ny + nz * nz), 1e-6)
    normal = np.stack(
        [
            nx * inv_len * 0.5 + 0.5,
            ny * inv_len * 0.5 + 0.5,
            nz * inv_len * 0.5 + 0.5,
        ],
        axis=-1,
    )
    return np.clip(normal, 0.0, 1.0)


def generate_all_textures(
    texture_dir: Path | str | None = None,
    seed: int | None = None,
    size: int | None = None,
    force: bool = False,
) -> dict[str, Path]:
    """화성 재질에 필요한 베이크 텍스처 세트를 생성한다."""

    texture_dir = Path(texture_dir) if texture_dir is not None else Path(TEXTURE_DIR)
    seed = int(seed if seed is not None else DEFAULT_SEED)
    size = int(size if size is not None else TEXTURE_SIZE)

    texture_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "mars_albedo": texture_dir / "mars_albedo.png",
        "mars_normal": texture_dir / "mars_normal.png",
        "mars_roughness": texture_dir / "mars_roughness.png",
        "mars_macro_noise": texture_dir / "mars_macro_noise.png",
        "mars_detail_noise": texture_dir / "mars_detail_noise.png",
        "mars_micro_noise": texture_dir / "mars_micro_noise.png",
        "mars_color_variation": texture_dir / "mars_color_variation.png",
        "mars_crater_mask": texture_dir / "mars_crater_mask.png",
    }

    if not force and all(path.exists() for path in paths.values()):
        return paths

    macro_noise = _multi_scale_noise(
        seed=seed + 11,
        size=size,
        base_res=max(6, int(round((size / 34.0) * max(MACRO_UV_REPEAT, 0.5)))),
        octaves=5,
        persistence=0.60,
        lacunarity=1.85,
    )
    detail_noise = _multi_scale_noise(
        seed=seed + 29,
        size=size,
        base_res=max(12, int(round((size / 14.0) * max(DETAIL_UV_REPEAT / 6.0, 0.5)))),
        octaves=6,
        persistence=0.54,
        lacunarity=2.05,
    )
    micro_noise = _multi_scale_noise(
        seed=seed + 41,
        size=size,
        base_res=max(18, int(round((size / 18.0) * max(MICRO_UV_REPEAT / 18.0, 0.75)))),
        octaves=5,
        persistence=0.44,
        lacunarity=2.55,
    )
    crater_mask = _make_crater_mask(seed=seed, size=size)
    height_field = _make_height_field(
        seed=seed,
        size=size,
        macro_noise=macro_noise,
        detail_noise=0.72 * detail_noise + 0.28 * micro_noise,
        crater_mask=crater_mask,
    )

    base_low = np.array(BASE_COLOR_RANGE[0], dtype=np.float32)
    base_high = np.array(BASE_COLOR_RANGE[1], dtype=np.float32)
    base_color = (0.60 * base_low + 0.40 * base_high).astype(np.float32)

    color_variation = np.stack(
        [
            0.5 + 0.5 * (0.58 * macro_noise + 0.26 * detail_noise + 0.16 * micro_noise),
            0.5 + 0.5 * (0.30 * macro_noise + 0.48 * detail_noise + 0.22 * micro_noise),
            0.5 + 0.5 * (0.16 * macro_noise + 0.56 * detail_noise + 0.28 * micro_noise),
        ],
        axis=-1,
    )
    color_variation = np.clip(color_variation, 0.0, 1.0)
    color_variation[..., 0] *= 1.18
    color_variation[..., 1] *= 0.92
    color_variation[..., 2] *= 0.82

    albedo = np.zeros((size, size, 3), dtype=np.float32)
    albedo[:] = base_color
    albedo += (macro_noise[..., None] - 0.5) * np.array([0.22, 0.12, 0.07], dtype=np.float32) * MACRO_NOISE_STRENGTH
    albedo += (detail_noise[..., None] - 0.5) * np.array([0.10, 0.05, 0.03], dtype=np.float32) * DETAIL_NOISE_STRENGTH
    albedo += (micro_noise[..., None] - 0.5) * np.array([0.05, 0.025, 0.015], dtype=np.float32) * MICRO_NOISE_STRENGTH
    albedo += (color_variation - 0.5) * np.array([0.08, 0.03, 0.015], dtype=np.float32)
    albedo -= crater_mask[..., None] * np.array([0.07, 0.05, 0.035], dtype=np.float32)

    banding = 0.04 * np.sin(5.0 * np.pi * height_field + 1.3)
    albedo += banding[..., None] * np.array([0.08, 0.03, 0.015], dtype=np.float32)
    rust_tint = 0.03 * macro_noise + 0.05 * detail_noise
    albedo += rust_tint[..., None] * np.array([0.12, 0.03, 0.01], dtype=np.float32)
    albedo -= (0.5 - micro_noise)[..., None] * np.array([0.01, 0.008, 0.005], dtype=np.float32)
    albedo = np.clip(albedo, 0.0, 1.0)

    rough_min, rough_max = ROUGHNESS_RANGE
    roughness = rough_max
    roughness = roughness - 0.05 * macro_noise - 0.03 * detail_noise
    roughness = roughness - 0.02 * micro_noise
    roughness = roughness + 0.08 * crater_mask + 0.02 * height_field
    roughness = np.clip(roughness, rough_min, rough_max)

    normal_height = 0.66 * height_field + 0.20 * macro_noise + 0.10 * detail_noise + 0.04 * micro_noise
    normal_height = _box_blur(normal_height, passes=1)
    normal = _compute_normal_map(normal_height, strength=NORMAL_STRENGTH)

    def _save_rgb(path: Path, arr: np.ndarray) -> None:
        Image.fromarray((np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)).save(path)

    def _save_gray(path: Path, arr: np.ndarray) -> None:
        Image.fromarray((np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)).save(path)

    print("ALBEDO MIN :", float(albedo.min()))
    print("ALBEDO MAX :", float(albedo.max()))
    print("ALBEDO MEAN:", float(albedo.mean()))
    print("ROUGHNESS MIN :", float(roughness.min()))
    print("ROUGHNESS MAX :", float(roughness.max()))
    print("ROUGHNESS MEAN:", float(roughness.mean()))

    _save_rgb(paths["mars_albedo"], albedo)
    _save_rgb(paths["mars_color_variation"], color_variation)
    _save_gray(paths["mars_macro_noise"], macro_noise)
    _save_gray(paths["mars_detail_noise"], detail_noise)
    _save_gray(paths["mars_micro_noise"], micro_noise)
    _save_gray(paths["mars_crater_mask"], crater_mask)
    _save_gray(paths["mars_roughness"], roughness)
    _save_rgb(paths["mars_normal"], normal)

    return paths


def ensure_mars_textures(
    *,
    texture_dir: Path | str = TEXTURE_DIR,
    seed: int = DEFAULT_SEED,
    overwrite: bool = False,
    texture_size: int = TEXTURE_SIZE,
) -> dict[str, Path]:
    """기존 호출부 호환용 래퍼다."""

    return generate_all_textures(
        texture_dir=texture_dir,
        seed=seed,
        size=texture_size,
        force=overwrite,
    )


def build_material_info(seed: int, terrain_id: str, texture_paths: dict[str, str | Path]) -> dict[str, Any]:
    """생성된 텍스처 파일 이름과 재질 메타데이터를 함께 저장한다."""

    return {
        "seed": int(seed),
        "terrain_id": terrain_id,
        "material_name": "Mars_AntiTile_Soil",
        "shader": "UsdPreviewSurface",
        "textures": {
            "albedo": Path(texture_paths["mars_albedo"]).name,
            "normal": Path(texture_paths["mars_normal"]).name,
            "roughness": Path(texture_paths["mars_roughness"]).name,
            "macro_noise": Path(texture_paths["mars_macro_noise"]).name,
            "detail_noise": Path(texture_paths["mars_detail_noise"]).name,
            "micro_noise": Path(texture_paths["mars_micro_noise"]).name,
            "color_variation": Path(texture_paths["mars_color_variation"]).name,
            "crater_mask": Path(texture_paths["mars_crater_mask"]).name,
        },
    }
