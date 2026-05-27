"""실제 맵(terrain_xxxxx) 자산을 navigation 모듈용으로 로드.

T1 이 제공하는 I1 자산을 읽어 ObstacleGrid / FogMap 을 초기화한다.

  terrain_dir/
    ├─ meta.json          맵 크기·origin·minimap·basecamp·minerals
    ├─ obstacle_grid.npy  (N, N) 0/1  raw rock 영역 (resolution_m 해상도)
    └─ heightmap.npy      (N, N) float  (현재 미사용)

좌표 규약: world (x, y) 는 맵 중심이 원점. meta.origin 은 좌하단.
obstacle_grid.npy 는 [i, j] = [y행, x열], [0,0] = 좌하단 으로 가정한다.
"""
import json
import os

import numpy as np

from .obstacle_grid import ObstacleGrid
from .fog_map import FogMap


def _block_max(arr, f):
    """f×f 블록 max pooling 다운샘플. 장애물(1)을 보존한다."""
    if f <= 1:
        return arr
    R, C = arr.shape
    R2, C2 = R // f, C // f
    return arr[:R2 * f, :C2 * f].reshape(R2, f, C2, f).max(axis=(1, 3))


def _dilate(mask, r_cells):
    """mask 를 반경 r_cells 원형 SE 로 이진 팽창.

    정사각형(체비쇼프) 팽창은 모서리 방향으로 √2 배 과팽창하므로,
    로봇 반경에 맞춰 원형으로 한다.
    """
    m = np.asarray(mask, dtype=bool)
    if r_cells < 1:
        return m
    out = m.copy()
    H, W = m.shape
    r = int(np.ceil(r_cells))
    r2 = r_cells * r_cells
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r2 or (dx == 0 and dy == 0):
                continue
            sy_src = slice(max(0, -dy), H - max(0, dy))
            sy_dst = slice(max(0, dy), H - max(0, -dy))
            sx_src = slice(max(0, -dx), W - max(0, dx))
            sx_dst = slice(max(0, dx), W - max(0, -dx))
            out[sy_dst, sx_dst] |= m[sy_src, sx_src]
    return out


def load_terrain(terrain_dir, cell_size=0.1, robot_radius=0.8,
                 reveal_radius=2.0, grid_n=3):
    """terrain_dir 의 meta.json + obstacle_grid.npy 로드.

    Args:
        terrain_dir:  terrain_xxxxx 폴더 경로.
        cell_size:    navigation 격자 목표 해상도 (m/cell). raw 보다 거칠게.
        robot_radius: 장애물 inflate 반경 (m).
        reveal_radius: 센서 reveal 반경 (m).
        grid_n:       sector 그리드 (NxN).

    Returns:
        (meta, ogrid, fog)
        meta:  meta.json dict
        ogrid: ObstacleGrid — rock 영역 robot_radius inflate + 외곽 막힘.
        fog:   FogMap — raw rock 영역을 obstacle_mask 로 (ratio 분모 제외).
    """
    with open(os.path.join(terrain_dir, "meta.json")) as f:
        meta = json.load(f)
    raw = np.load(os.path.join(terrain_dir, "obstacle_grid.npy"))
    raw = (raw > 0).astype(np.uint8)

    W, H = float(meta["size_m"][0]), float(meta["size_m"][1])
    res = float(meta["resolution_m"])

    # raw 해상도(res) → 목표 해상도(cell_size) 다운샘플
    factor = max(1, int(round(cell_size / res)))
    raw_ds = _block_max(raw, factor)
    rows, cols = raw_ds.shape
    eff_cell = W / cols   # 다운샘플 결과에 맞춘 실제 셀 크기

    # ObstacleGrid: rock 을 robot_radius 만큼 팽창 + 맵 외곽 막기 (A* 입력)
    ogrid = ObstacleGrid(map_size=(W, H), cell_size=eff_cell,
                         robot_radius=robot_radius)
    r_cells = robot_radius / eff_cell
    inflated = _dilate(raw_ds, r_cells).astype(np.uint8)
    m = max(1, int(round(r_cells)))
    inflated[:m, :] = 1
    inflated[-m:, :] = 1
    inflated[:, :m] = 1
    inflated[:, -m:] = 1
    ogrid.set_grid(inflated)

    # FogMap: raw rock 영역 (팽창 X) 을 obstacle_mask 로 (reveal ratio 분모)
    fog = FogMap(map_size=(W, H), cell_size=eff_cell,
                 reveal_radius=reveal_radius, grid_n=grid_n)
    fog.set_obstacle_mask(raw_ds)

    print(f"[terrain_loader] {meta.get('terrain_id', '?')}: {W:.0f}×{H:.0f}m, "
          f"raw {raw.shape} → grid {rows}×{cols} (cell={eff_cell:.3f}m), "
          f"inflate={robot_radius}m")
    return meta, ogrid, fog
