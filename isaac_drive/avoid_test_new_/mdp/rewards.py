# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""학습 보상 함수들.

설계 — Anymal_Navigation 의 아이디어를 부분 채택 + 우리 컨텍스트에 맞게 보강:

  · progress                  : body-frame 속도 · goal 방향 (벡터 내적).
                                state 추적 없는 즉시 보상 (스텝당 m/s).
  · position_tanh             : tanh(dist/std) 기반 매끄러운 distance 보상.
                                coarse(std 큼) + fine(std 작음) 두 개 겹쳐 사용.
  · obstacle_proximity_penalty: 레이캐스트 prominence 가장 큰 셀이 가까울수록 -.
                                soft signal — 충돌(이진) 보다 학습신호 풍부.
  · collision_penalty         : 차량 body 접촉 ≥ 임계 → -1 (weight 로 크게 음수화).
  · goal_reached_bonus        : 도착 반경 안 → +1 (성공 메트릭·종료 동반).
  · goal_alignment            : 차량 정면과 goal 방향의 cos.  Ackermann 은 strafing
                                불가라 yaw 정렬이 의미 있다 (저가중치 권장).
  · action_rate_l2            : 액션 변화 L2 — 부드러운 주행 유도 (저가중치).
  · time_penalty              : 매 스텝 -1 — 빙빙 돌기·시간끌기 방지.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _safe(t: torch.Tensor, clip: float = 100.0) -> torch.Tensor:
    """NaN/inf 를 0 으로 치환하고 절댓값 clip — reward 가 학습 망가뜨리는 거 방지."""
    return torch.nan_to_num(t, nan=0.0, posinf=clip, neginf=-clip).clamp(-clip, clip)


def _goal_body_dir(env: "ManagerBasedRLEnv",
                   asset: Articulation,
                   command_name: str) -> tuple[torch.Tensor, torch.Tensor]:
    """(unit_dir_body (N,2), dist (N,)) — 차량 기준 goal 방향 단위벡터·거리."""
    goal_w = env.command_manager.get_command(command_name)
    pos = asset.data.root_pos_w
    q = asset.data.root_quat_w
    w = q[..., 0]; x = q[..., 1]; y = q[..., 2]; z = q[..., 3]
    yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    cy = torch.cos(yaw); sy = torch.sin(yaw)
    dx = goal_w[:, 0] - pos[:, 0]
    dy = goal_w[:, 1] - pos[:, 1]
    fwd = dx * cy + dy * sy
    lat = -dx * sy + dy * cy
    dist = (dx * dx + dy * dy).sqrt()
    inv = 1.0 / dist.clamp(min=1.0e-6)
    return torch.stack([fwd * inv, lat * inv], dim=-1), dist


# ---------------------------------------------------------------------------
# Reward terms
# ---------------------------------------------------------------------------
def progress(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """body-frame 속도의 goal 방향 성분.  + 면 goal 쪽으로 진행 중."""
    asset: Articulation = env.scene[asset_cfg.name]
    dir_b, _ = _goal_body_dir(env, asset, command_name)
    vel_b = asset.data.root_lin_vel_b[:, :2]
    return _safe((vel_b * dir_b).sum(dim=-1), clip=10.0)   # (N,)


def position_tanh(
    env: "ManagerBasedRLEnv",
    std: float,
    command_name: str = "goal_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """1 - tanh(dist / std) — 거리에 따른 부드러운 +보상.  goal 가까울수록 1."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, dist = _goal_body_dir(env, asset, command_name)
    return _safe(1.0 - torch.tanh(dist / std), clip=1.5)


def obstacle_proximity_penalty(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    prom_radius: int = 2,
    height_thresh: float = 0.15,
    near_threshold: float = 1.5,
    consider_back: float = 0.5,
) -> torch.Tensor:
    """가장 가까운 prominent 장애물 셀까지의 거리 기반 soft 페널티.

    near_threshold 안에 prominent(>height_thresh) 셀이 있으면, 거리에 반비례한
    +값(weight 음수화로 -페널티) 을 돌려준다.  거리 0 → 1, 거리 ≥ near_threshold → 0.

    Args:
        consider_back: 차량 뒤쪽 이 거리(m) 까지도 페널티 대상에 포함 (옆구리·바퀴 안전).
    """
    scanner: RayCaster = env.scene[sensor_cfg.name]
    hits = scanner.data.ray_hits_w
    sensor_pos = scanner.data.pos_w
    n_rays = hits.shape[1]

    pc = scanner.cfg.pattern_cfg
    res = float(pc.resolution)
    nx = len(torch.arange(-pc.size[0] / 2.0, pc.size[0] / 2.0 + 1.0e-9, res))
    ny = len(torch.arange(-pc.size[1] / 2.0, pc.size[1] / 2.0 + 1.0e-9, res))
    rows, cols = (ny, nx) if getattr(pc, "ordering", "xy") == "xy" else (nx, ny)
    if rows * cols != n_rays:
        return torch.zeros(env.num_envs, device=env.device)

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
    prom = core - acc / cnt

    is_obs = torch.isfinite(prom) & (prom > height_thresh)
    if not is_obs.any():
        return torch.zeros(N, device=env.device)

    # core 셀의 (차량까지) 평면 거리.  센서가 차량 위 10m 에 있고 yaw 정렬되므로,
    # core_xy - sensor_xy 의 평면 길이가 곧 차량으로부터의 거리.
    xy = hits[..., :2].view(N, rows, cols, 2)
    core_xy = xy[:, r : rows - r, r : cols - r, :]
    dxy = core_xy - sensor_pos[:, :2].view(N, 1, 1, 2)
    dist = dxy.norm(dim=-1)   # (N, rows-2r, cols-2r)

    # 차량 기준 fwd 가 너무 뒤(-consider_back 보다 더 뒤)인 셀은 제외.
    # 센서가 차량 기준 (0, 0) 이고 ray_alignment=yaw 라 sensor_pos.xy == 차량 xy.
    # → dxy 를 yaw 로 회전해 body frame fwd 를 구함.
    asset = env.scene["robot"]
    q = asset.data.root_quat_w
    yaw = torch.atan2(
        2.0 * (q[..., 0] * q[..., 3] + q[..., 1] * q[..., 2]),
        1.0 - 2.0 * (q[..., 2] ** 2 + q[..., 3] ** 2),
    )
    cy = torch.cos(yaw).view(N, 1, 1)
    sy = torch.sin(yaw).view(N, 1, 1)
    fwd = dxy[..., 0] * cy + dxy[..., 1] * sy

    valid = is_obs & (fwd > -consider_back)
    # valid 한 셀만 거리 보고 가장 가까운 거 찾기.  invalid 는 큰 거리로 마스킹.
    big = torch.full_like(dist, near_threshold * 10.0)
    dist_masked = torch.where(valid, dist, big)
    min_dist = dist_masked.view(N, -1).min(dim=-1).values

    # near_threshold 안 → (1 - min_dist/near) 양수, 밖 → 0.
    penalty = torch.clamp(1.0 - min_dist / near_threshold, min=0.0)
    return _safe(penalty, clip=1.5)


def collision_penalty(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """차량 body 임계 접촉력 발생 → 1, 아니면 0.  weight 로 큰 음수."""
    sensor: ContactSensor = env.scene[sensor_cfg.name]
    forces = sensor.data.net_forces_w
    if forces is None:
        return torch.zeros(env.num_envs, device=env.device)
    fmax = forces.norm(dim=-1).max(dim=-1).values
    return (fmax > force_threshold).float()


def goal_reached_bonus(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    threshold: float = 0.6,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """goal 반경 안 → 1.  도착 종료와 함께 한 번만 들어오는 큰 +."""
    asset: Articulation = env.scene[asset_cfg.name]
    _, dist = _goal_body_dir(env, asset, command_name)
    return (dist < threshold).float()


def time_penalty(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """매 스텝 1.  weight 로 작은 음수 (-0.01 정도)."""
    return torch.ones(env.num_envs, device=env.device)


def action_rate_l2(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """직전 액션 대비 변화량 L2 — 부드러운 주행 유도."""
    am = env.action_manager
    return _safe(torch.sum((am.action - am.prev_action) ** 2, dim=-1), clip=100.0)


def goal_alignment(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """차량 정면 단위벡터 ⋅ goal 방향 단위벡터.  -1~+1.

    Ackermann 차량은 횡이동 불가 → goal 방향과 정면이 일치해야 효율적 이동.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    dir_b, _ = _goal_body_dir(env, asset, command_name)
    # body-frame 에서 정면은 (1, 0).  내적은 dir_b[:, 0].
    return _safe(dir_b[:, 0], clip=1.5)
