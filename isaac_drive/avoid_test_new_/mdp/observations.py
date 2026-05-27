# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""정책 입력 관측 함수들.

설계 핵심 — **모두 body-frame** (회전 invariance 학습 부담을 제거):
  · goal_body_xy_dist  : goal 의 차량 기준 (전방, 좌측, 거리) — 3-dim.
  · goal_yaw_sincos    : 차량 yaw 와 goal 방향의 각도차를 (sin, cos) 로  — 2-dim.
  · raycast_prominence : 레이캐스트 격자의 '국소 돌출량'(=장애물 신호) flatten.
                         지면 슬로프를 흡수해 바위만 남긴 신호 (drive_test detector 와 동일 식).
  · base_lin_vel       : 차량 body-frame 선속도 3-dim.
  · base_ang_vel       : 차량 body-frame 각속도 3-dim.
  · last_action        : 직전 RL 출력 (residual) 2-dim (action_rate 스무딩에 유리).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _quat_to_yaw(q: torch.Tensor) -> torch.Tensor:
    """(..., 4) wxyz → (...) yaw."""
    w = q[..., 0]
    x = q[..., 1]
    y = q[..., 2]
    z = q[..., 3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return torch.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Observation terms
# ---------------------------------------------------------------------------
def _sanitize(t: torch.Tensor, clip: float = 50.0) -> torch.Tensor:
    """NaN/inf 를 0 으로, 절댓값을 clip 로 제한 — RL 정책에 들어가기 전 안전판."""
    return torch.nan_to_num(t, nan=0.0, posinf=clip, neginf=-clip).clamp(-clip, clip)


def goal_body_xy_dist(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """차량 기준 goal 위치 + 거리.  (N, 3) — (fwd, lat, dist)."""
    asset: Articulation = env.scene[asset_cfg.name]
    goal_w = env.command_manager.get_command(command_name)  # (N, 2)
    pos = asset.data.root_pos_w  # (N, 3)
    yaw = _quat_to_yaw(asset.data.root_quat_w)  # (N,)
    cy = torch.cos(yaw)
    sy = torch.sin(yaw)
    dx = goal_w[:, 0] - pos[:, 0]
    dy = goal_w[:, 1] - pos[:, 1]
    fwd = dx * cy + dy * sy
    lat = -dx * sy + dy * cy
    dist = (dx * dx + dy * dy).sqrt()
    return _sanitize(torch.stack([fwd, lat, dist], dim=-1))


def goal_yaw_sincos(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """차량 정면과 goal 방향 사이 각도의 (sin, cos).  (N, 2) — discontinuity-free."""
    asset: Articulation = env.scene[asset_cfg.name]
    goal_w = env.command_manager.get_command(command_name)
    pos = asset.data.root_pos_w
    yaw = _quat_to_yaw(asset.data.root_quat_w)
    dx = goal_w[:, 0] - pos[:, 0]
    dy = goal_w[:, 1] - pos[:, 1]
    desired_yaw = torch.atan2(dy, dx)
    err = desired_yaw - yaw
    return _sanitize(torch.stack([torch.sin(err), torch.cos(err)], dim=-1), clip=1.5)


def raycast_prominence(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    prom_radius: int = 2,
) -> torch.Tensor:
    """레이캐스트 격자에서 8-이웃 평균 대비 국소 돌출량.

    drive_test/detector.py 와 같은 식 — 평면·언덕은 0 근처, 바위 셀만 +로 튐.
    Returns (N, (rows-2r)*(cols-2r)) flatten 텐서.  rows/cols 는 GridPattern.
    """
    scanner: RayCaster = env.scene[sensor_cfg.name]
    hits = scanner.data.ray_hits_w  # (N, num_rays, 3)
    n_rays = hits.shape[1]

    # GridPattern 으로부터 (rows, cols) 복원.
    pc = scanner.cfg.pattern_cfg
    res = float(pc.resolution)
    nx = len(torch.arange(-pc.size[0] / 2.0, pc.size[0] / 2.0 + 1.0e-9, res))
    ny = len(torch.arange(-pc.size[1] / 2.0, pc.size[1] / 2.0 + 1.0e-9, res))
    rows, cols = (ny, nx) if getattr(pc, "ordering", "xy") == "xy" else (nx, ny)
    if rows * cols != n_rays:
        # 모양 안 맞으면 fail-safe 로 z 값만 반환.
        return hits[..., 2].clamp(-2.0, 2.0)

    N = env.num_envs
    z = hits[..., 2].view(N, rows, cols)

    r = prom_radius
    core = z[:, r : rows - r, r : cols - r]
    acc = torch.zeros_like(core)
    cnt = 0
    for dr in (-r, 0, r):
        for dc in (-r, 0, r):
            if dr == 0 and dc == 0:
                continue
            acc = acc + z[:, r + dr : rows - r + dr, r + dc : cols - r + dc]
            cnt += 1
    prom = core - acc / cnt   # (N, rows-2r, cols-2r)

    # 비유한(ray miss) 셀은 0으로.  너무 큰 값은 clamp.
    prom = torch.where(torch.isfinite(prom), prom, torch.zeros_like(prom))
    prom = prom.clamp(-1.0, 1.0)
    return _sanitize(prom.reshape(N, -1), clip=1.0)


def base_lin_vel_b(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """차량 body-frame 선속도 (N, 3)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return _sanitize(asset.data.root_lin_vel_b, clip=10.0)


def base_ang_vel_b(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """차량 body-frame 각속도 (N, 3)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return _sanitize(asset.data.root_ang_vel_b, clip=10.0)
