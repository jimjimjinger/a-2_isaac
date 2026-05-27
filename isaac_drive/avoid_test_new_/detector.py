# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""하향 RayCaster 히트로 '갑자기 솟은 돌출(바위)'을 장애물로 판정하는 헬퍼.

play_wasd.py / check_drive.py 가 공유한다.

판정 방식 — 국소 돌출량(local prominence) + 연결요소 분리:
  ① 격자 각 셀의 높이에서 반경 PROM_RADIUS 셀의 8이웃 평균을 뺀다.
     평면(기울기 무관)은 0, 완만한 굽음도 작은 값, 갑자기 솟은 바위만
     크게 튄다 → height_thresh 초과 = 장애물 셀.
  ② 장애물 셀들을 8-연결 연결요소로 묶는다.  덩어리 1개 = 장애물 1개.
     → 레이캐스터 안에 바위가 여러 개 들어오면 각각 따로 인식한다.

  ※ 옛 방식('격자 최저점 = 바닥, 0.15m 위 = 장애물')은 기복 지형에서
    경사 자체를 장애물로 오인식했다.  prominence 는 기울기를 흡수하고
    '갑자기 솟은 돌출'만 잡는다.
  ※ terrain_00022 측정상 PROM_RADIUS=2(0.4m)에서 순수 지형 prominence
    는 최대 ~0.08m → height_thresh=0.15m 면 경사·언덕에 오탐하지 않는다.
"""

from __future__ import annotations

import math

import torch

# 격자 이웃 비교 반경 (셀). RayCaster 격자는 0.2m/셀 → 2셀 = 0.4m.
PROM_RADIUS = 2


def _yaw_from_quat(q) -> float:
    """wxyz 쿼터니언 → yaw (rad)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _direction_label(fwd: float, lat: float, center_margin: float) -> str:
    """차량 기준 (전방 fwd, 좌측 lat) → 방향 라벨."""
    fb = "앞쪽" if fwd > center_margin else ("뒤쪽" if fwd < -center_margin else "")
    lr = "왼쪽" if lat > center_margin else ("오른쪽" if lat < -center_margin else "")
    if fb and lr:
        return f"{fb}-{lr}"
    if fb or lr:
        return fb or lr
    return "정중앙"


def _connected_components(mask: list) -> list:
    """8-연결 연결요소 라벨링 (붙어있는 셀끼리 한 덩어리로).

    Args:
        mask: (H×W) bool 중첩 리스트.
    Returns:
        list[list[(i, j)]] — 덩어리별 셀 좌표 목록.
    """
    h = len(mask)
    w = len(mask[0]) if h else 0
    seen = [[False] * w for _ in range(h)]
    comps = []
    for i in range(h):
        for j in range(w):
            if not mask[i][j] or seen[i][j]:
                continue
            stack = [(i, j)]              # flood fill (8-연결)
            seen[i][j] = True
            cells = []
            while stack:
                y, x = stack.pop()
                cells.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < h and 0 <= nx < w
                                and mask[ny][nx] and not seen[ny][nx]):
                            seen[ny][nx] = True
                            stack.append((ny, nx))
            comps.append(cells)
    return comps


def _grid_shape(scanner, n_rays: int) -> tuple[int, int]:
    """RayCaster GridPattern 격자의 (행, 열) 수 — 정사각·직사각 모두 대응.

    ray 개수만으로는 직사각형 격자를 복원할 수 없으므로 GridPatternCfg
    (size·resolution·ordering)에서 격자 생성 방식을 그대로 재현한다.
    설정을 못 읽으면 정사각형으로 폴백한다.
    """
    try:
        pc = scanner.cfg.pattern_cfg
        res = float(pc.resolution)
        nx = len(torch.arange(-pc.size[0] / 2.0, pc.size[0] / 2.0 + 1.0e-9, res))
        ny = len(torch.arange(-pc.size[1] / 2.0, pc.size[1] / 2.0 + 1.0e-9, res))
        # ordering "xy" → 안쪽 루프 x → reshape(ny, nx). "yx" → reshape(nx, ny).
        rows, cols = (ny, nx) if getattr(pc, "ordering", "xy") == "xy" else (nx, ny)
        if rows * cols == n_rays and min(rows, cols) > 0:
            return rows, cols
    except Exception:  # noqa: BLE001
        pass
    side = int(round(n_rays ** 0.5))           # 폴백 — 정사각형 가정
    return (side, side) if side * side == n_rays else (0, 0)


def detect_obstacles(
    scanner,
    env_idx: int = 0,
    height_thresh: float = 0.15,
    center_margin: float = 0.3,
    min_cells: int = 1,
) -> list[dict]:
    """하향 RayCaster 히트에서 '갑자기 솟은 돌출'을 장애물 단위로 찾는다.

    각 ray 히트 높이를 격자로 보고 셀별 '국소 돌출량'을 구한 뒤, 임계값을
    넘은 셀들을 연결요소로 묶어 장애물 덩어리를 분리한다.  덩어리마다
    중심 위치·돌출 높이·방향을 따로 돌려준다 → 여러 개를 동시에 인식한다.

    Args:
        scanner: env.scene["height_scanner"] (RayCaster 센서).
        env_idx: 환경 인덱스.
        height_thresh: 장애물로 칠 최소 국소 돌출량 (m).
        center_margin: '중앙' 으로 칠 ±범위 (m).
        min_cells: 장애물로 칠 최소 덩어리 셀 수 (잡음 제거용).

    Returns:
        list[dict] — 장애물 목록 (차량에서 가까운 순). 없으면 빈 리스트.
        각 dict:
          n_cells (int)  : 그 장애물의 돌출 셀 수
          peak    (float): 국소 평면 대비 최대 돌출 (m)
          fwd     (float): 장애물 중심의 차량 전방거리 (m, +앞 / -뒤)
          lat     (float): 장애물 중심의 차량 좌측거리 (m, +좌 / -우)
          label   (str)  : 방향 라벨 ('앞쪽-왼쪽' 등)
    """
    hits = scanner.data.ray_hits_w[env_idx]    # (num_rays, 3) 월드 히트좌표
    n_rays = hits.shape[0]
    rows, cols = _grid_shape(scanner, n_rays)  # 격자 행·열 (정사각·직사각 자동)
    r = PROM_RADIUS
    if min(rows, cols) <= 2 * r:
        return []

    z = hits[:, 2].reshape(rows, cols)         # 높이 격자

    # --- 셀별 국소 돌출량 : 중심 − (반경 r 의 8이웃 평균) ---
    # 평면은 8이웃 평균 = 중심 → 0. 굽은 지형도 작은 값. 돌출만 크다.
    core = z[r : rows - r, r : cols - r]
    acc = torch.zeros_like(core)
    cnt = 0
    for dr in (-r, 0, r):
        for dc in (-r, 0, r):
            if dr == 0 and dc == 0:
                continue
            acc = acc + z[r + dr : rows - r + dr, r + dc : cols - r + dc]
            cnt += 1
    prom = core - acc / cnt                    # (rows-2r, cols-2r)

    # 비유한(ray miss) 셀은 prom 도 비유한 → 자동 제외.
    obs = torch.isfinite(prom) & (prom > height_thresh)
    if not bool(obs.any()):
        return []

    # --- 돌출 셀을 연결요소(8-연결)로 묶어 장애물 덩어리 분리 ---
    comps = _connected_components(obs.detach().cpu().numpy().tolist())

    # ray 인덱스 매핑 + 센서 포즈 (월드 → 차량 기준 변환용)
    flat_idx = torch.arange(n_rays, device=hits.device).reshape(rows, cols)
    core_idx = flat_idx[r : rows - r, r : cols - r]   # prom 셀 → ray 인덱스
    prom_cpu = prom.detach().cpu()
    pos = scanner.data.pos_w[env_idx]          # 센서 월드 위치 (차량 위 10m)
    quat = scanner.data.quat_w[env_idx]        # 센서 방향 (차량 yaw)
    yaw = _yaw_from_quat(quat)
    cy, sy = math.cos(yaw), math.sin(yaw)

    obstacles = []
    for cells in comps:
        if len(cells) < min_cells:
            continue
        idxs = torch.tensor([int(core_idx[a, b]) for a, b in cells],
                            device=hits.device, dtype=torch.long)
        ohits = hits[idxs]                     # (k, 3) 이 덩어리의 월드 히트
        dx = ohits[:, 0] - pos[0]
        dy = ohits[:, 1] - pos[1]
        fwd = float((dx * cy + dy * sy).mean())     # 차량 전방 성분 (중심)
        lat = float((-dx * sy + dy * cy).mean())    # 차량 좌측 성분 (중심)
        peak = max(float(prom_cpu[a, b]) for a, b in cells)
        obstacles.append({
            "n_cells": len(cells),
            "peak": peak,
            "fwd": fwd,
            "lat": lat,
            "label": _direction_label(fwd, lat, center_margin),
        })

    # 차량에서 가까운 순으로 정렬.
    obstacles.sort(key=lambda o: o["fwd"] ** 2 + o["lat"] ** 2)
    return obstacles
