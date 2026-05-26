# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""로버 장애물 회피 태스크의 커스텀 관측 항목.

- height_scan_grid : 하향 RayCaster 의 격자별 지형 높이 → 장애물 감지
- distance_to_goal : 로버→목표 거리 (베이스 프레임)
- angle_to_goal    : 로버 진행방향 기준 목표 방위각
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def height_scan_grid(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    offset: float = 0.0,
) -> torch.Tensor:
    """하향 RayCaster 격자의 '센서→지형 거리' 를 관측 벡터로 반환.

    센서는 로버 위 일정 높이에서 아래로 격자 ray 를 쏜다.
      - 평지   : ray 가 멀리(바닥까지) 닿음 → 큰 값
      - 장애물 : ray 가 장애물 윗면에 가까이 닿음 → 작은 값
    정책은 '작은 값 = 앞에 장애물' 을 학습한다.

    Returns:
        (num_envs, num_rays) 텐서. ray_hits 가 빗나가 inf 인 경우 0 으로 치환.
    """
    sensor: RayCaster = env.scene.sensors[sensor_cfg.name]
    # ray_hits_w: (num_envs, num_rays, 3) — 각 ray 가 지형에 닿은 월드 좌표.
    hits_z = sensor.data.ray_hits_w[..., 2]
    sensor_z = sensor.data.pos_w[:, 2].unsqueeze(1)
    dist = sensor_z - hits_z - offset
    return torch.nan_to_num(dist, nan=0.0, posinf=0.0, neginf=0.0)


def distance_to_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "target_pose",
) -> torch.Tensor:
    """로버→목표 유클리드 거리 (베이스 프레임). Shape (num_envs, 1)."""
    command = env.command_manager.get_command(command_name)
    distance = torch.norm(command[:, :2], p=2, dim=-1, keepdim=True)
    return distance


def angle_to_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "target_pose",
) -> torch.Tensor:
    """로버 진행방향([1,0]) 기준 목표의 방위각 (rad). Shape (num_envs, 1).

    0 이면 목표가 정면, +면 왼쪽, -면 오른쪽.
    """
    command = env.command_manager.get_command(command_name)
    angle = torch.atan2(command[:, 1], command[:, 0])
    return angle.unsqueeze(-1)
