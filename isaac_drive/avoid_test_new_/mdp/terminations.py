# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""에피소드 종료 조건들.

  · goal_reached : 도착 (성공) — goal 반경 안.
  · collision    : 충돌 (실패) — 차량 body 접촉력 임계 초과.
  · out_of_bounds: 맵 밖 (실패) — env_origin 기준 일정 거리 이상 벗어남.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .commands import RandomGoalCommand


def goal_reached(
    env: "ManagerBasedRLEnv",
    command_name: str = "goal_pose",
    threshold: float = 0.6,
) -> torch.Tensor:
    """goal 반경 안으로 들어오면 True."""
    cmd: "RandomGoalCommand" = env.command_manager.get_term(command_name)
    return cmd.current_dist() < threshold


def collision(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """차량 body 접촉력이 임계 초과 → True."""
    sensor: ContactSensor = env.scene[sensor_cfg.name]
    forces = sensor.data.net_forces_w
    if forces is None:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    fmax = forces.norm(dim=-1).max(dim=-1).values
    return fmax > force_threshold


def out_of_bounds(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    limit: float = 24.0,
) -> torch.Tensor:
    """차량이 env_origin 기준 ±limit 박스 밖으로 나가면 True."""
    asset: Articulation = env.scene[asset_cfg.name]
    rel = asset.data.root_pos_w[:, :2] - env.scene.env_origins[:, :2]
    return (rel.abs() > limit).any(dim=-1)


def world_out_of_bounds(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    world_limit: float = 24.0,
    min_z: float = -2.0,
) -> torch.Tensor:
    """차량이 world 절대 박스(|x|,|y| > world_limit) 또는 z 가 min_z 아래로 떨어지면.

    terrain_00022 의 50×50 메시 밖으로 나가면 raycaster 가 NaN 만 뱉어 학습 망가짐.
    이걸 강제 종료로 막는다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w
    xy_out = (pos[:, 0].abs() > world_limit) | (pos[:, 1].abs() > world_limit)
    z_out = pos[:, 2] < min_z
    return xy_out | z_out


def vehicle_tilt(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    max_tilt_deg: float = 60.0,
) -> torch.Tensor:
    """차량 roll/pitch 가 max_tilt_deg 초과 → 뒤집힘 직전, 종료."""
    asset: Articulation = env.scene[asset_cfg.name]
    # body z 축 (월드) 의 수직 성분 — 정상이면 +1, 뒤집히면 -1, 90° 누우면 0.
    q = asset.data.root_quat_w   # (N, 4) wxyz
    w = q[..., 0]; x = q[..., 1]; y = q[..., 2]; z = q[..., 3]
    # body-z 의 world up(z) 성분: R[2, 2] = 1 - 2(x^2 + y^2)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    cos_thr = math.cos(math.radians(max_tilt_deg))
    return up_z < cos_thr
