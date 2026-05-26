# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""로버 장애물 회피 태스크의 종료 조건.

  - goal_reached      : 목표 도달 → 성공 종료
  - collision         : 장애물 충돌 → 실패 종료
  - too_far_from_goal : 목표에서 과도하게 멀어짐 → 실패 종료 (안전장치)

시간 초과(time_out)는 isaaclab 업스트림 mdp.time_out 을 그대로 쓴다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def goal_reached(
    env: ManagerBasedRLEnv, command_name: str = "target_pose", threshold: float = 0.5
) -> torch.Tensor:
    """로버가 목표 threshold(m) 안으로 들어오면 True (성공)."""
    command = env.command_manager.get_command(command_name)
    distance = torch.norm(command[:, :2], p=2, dim=-1)
    return distance < threshold


def collision(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
    threshold: float = 1.0,
) -> torch.Tensor:
    """로버 몸체 접촉센서에 힘이 잡히면 True (장애물 충돌, 실패)."""
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = torch.norm(sensor.data.net_forces_w.view(env.num_envs, -1, 3), dim=-1)
    return torch.sum(forces, dim=-1) > threshold


def too_far_from_goal(
    env: ManagerBasedRLEnv, command_name: str = "target_pose", max_distance: float = 15.0
) -> torch.Tensor:
    """목표에서 max_distance(m) 보다 멀어지면 True (헤맴 방지 안전장치)."""
    command = env.command_manager.get_command(command_name)
    distance = torch.norm(command[:, :2], p=2, dim=-1)
    return distance > max_distance


def obstacle_hit(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    radius: float = 0.6,
    height_threshold: float = 0.15,
) -> torch.Tensor:
    """레이캐스트로 장애물이 로봇 중심 radius(m) 안에 들어오면 True (충돌, 실패).

    몸체 접촉센서 기반 ``collision`` 은 낮은 장애물에 바퀴만 걸리면(몸체가
    안 닿음) 못 잡는다. 이 항목은 하향 RayCaster 로 '장애물이 로봇 코앞'
    인 상황을 충돌로 판정해, 바퀴 걸림도 즉시 종료(재소환)시킨다.

    바닥보다 height_threshold(m) 이상 솟은 ray 가 로봇 중심에서 radius 안에
    하나라도 있으면 충돌로 본다.
    """
    sensor: RayCaster = env.scene.sensors[sensor_cfg.name]
    hits = sensor.data.ray_hits_w                           # (N, rays, 3)
    sensor_xy = sensor.data.pos_w[:, :2].unsqueeze(1)       # (N, 1, 2)
    horiz = torch.norm(hits[..., :2] - sensor_xy, dim=-1)   # (N, rays)
    hit_z = torch.nan_to_num(hits[..., 2], nan=0.0, posinf=0.0, neginf=0.0)
    near_obstacle = (hit_z > height_threshold) & (horiz < radius)
    return torch.any(near_obstacle, dim=-1)
