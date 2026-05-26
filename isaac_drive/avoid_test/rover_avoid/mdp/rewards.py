# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""로버 장애물 회피 태스크의 커스텀 보상 항목.

설계 원칙 (m0609_lift ver2 의 교훈 반영):
  - dense(조밀) 유도 보상 + sparse(희소) 도달 보너스 조합
  - 대부분 항목을 ``max_episode_length`` 로 나눠 per-step 크기를 작게 유지
    → 에피소드 합이 O(1) 스케일이 되고, weight 로 항목 간 균형을 잡는다
  - 과한 shaping 은 local optimum 을 유발 → 항목 수를 절제

장애물 회피는 별도 보상 항목이 아니라 ``collision_penalty`` (충돌 페널티)
+ height-scan 관측의 조합으로 학습된다 — 정책이 관측으로 장애물을 보고,
부딪히면 손해이므로 스스로 우회를 배운다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _goal_distance(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """로버→목표 거리 (베이스 프레임). Shape (num_envs,)."""
    command = env.command_manager.get_command(command_name)
    return torch.norm(command[:, :2], p=2, dim=-1)


def goal_distance_reward(env: ManagerBasedRLEnv, command_name: str = "target_pose") -> torch.Tensor:
    """목표에 가까울수록 큰 dense 보상. 거리가 멀면 0 으로 부드럽게 감쇠."""
    distance = _goal_distance(env, command_name)
    return (1.0 / (1.0 + 0.11 * distance * distance)) / env.max_episode_length


def goal_reached_reward(
    env: ManagerBasedRLEnv, command_name: str = "target_pose", threshold: float = 0.5
) -> torch.Tensor:
    """목표 도달 시 보너스 — 남은 시간에 비례 (빨리 갈수록 큼). sparse."""
    distance = _goal_distance(env, command_name)
    remaining = (env.max_episode_length - env.episode_length_buf) / env.max_episode_length
    return torch.where(distance < threshold, remaining, torch.zeros_like(remaining))


def progress_reward(env: ManagerBasedRLEnv, command_name: str = "target_pose") -> torch.Tensor:
    """목표 쪽으로 다가가는 속도(전진율)에 비례한 보상.

    멈춰 있으면 0, 목표로 다가가면 +, 멀어지면 −. 장애물 앞에서 가만히
    서거나 앞뒤로만 깨작거리는(freeze) local optimum 을 깬다 — 머무르면
    보상이 0 이라, 정책이 '우회해서라도 계속 전진' 하도록 유도한다.
    """
    command = env.command_manager.get_command(command_name)
    goal_xy = command[:, :2]
    goal_dir = goal_xy / (torch.norm(goal_xy, p=2, dim=-1, keepdim=True) + 1e-6)
    vel_b = env.scene["robot"].data.root_lin_vel_b[:, :2]
    progress = torch.sum(vel_b * goal_dir, dim=-1)  # 목표 방향 속도 (m/s)
    return progress / env.max_episode_length


def heading_to_goal_reward(env: ManagerBasedRLEnv, command_name: str = "target_pose") -> torch.Tensor:
    """로버가 목표 방향을 바라볼수록 보상 (가까울수록 가중치 큼)."""
    command = env.command_manager.get_command(command_name)
    distance = torch.norm(command[:, :2], p=2, dim=-1)
    angle = torch.atan2(command[:, 1], command[:, 0])
    reward = (1.0 / (1.0 + distance)) * (1.0 / (1.0 + torch.abs(angle)))
    return reward / env.max_episode_length


def angle_to_goal_penalty(env: ManagerBasedRLEnv, command_name: str = "target_pose") -> torch.Tensor:
    """목표가 옆/뒤(|angle|>2rad)에 있을 때 페널티 — 방향 못 잡고 헤매는 것 방지."""
    command = env.command_manager.get_command(command_name)
    angle = torch.atan2(command[:, 1], command[:, 0])
    return torch.where(
        torch.abs(angle) > 2.0, torch.abs(angle) / env.max_episode_length, torch.zeros_like(angle)
    )


def collision_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
    threshold: float = 1.0,
) -> torch.Tensor:
    """몸체 접촉센서에 힘이 잡히면(=장애물 충돌) 페널티 1.0.

    로버 몸체(Body)는 평소 공중에 떠 있으므로 net contact force 가 잡히면
    장애물에 부딪힌 것으로 본다.
    """
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = torch.norm(sensor.data.net_forces_w.view(env.num_envs, -1, 3), dim=-1)
    collided = torch.sum(forces, dim=-1) > threshold
    return collided.float()


def obstacle_proximity_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    radius: float = 0.9,
    height_threshold: float = 0.15,
) -> torch.Tensor:
    """로봇 가까이(radius m 이내)에 장애물이 있으면 페널티.

    collision_penalty 는 '부딪힌 뒤'에만 신호를 주지만, 이 항목은 '부딪히기
    전 너무 가까움'을 미리 벌한다 → 정책이 아슬아슬하게 스치지 않고 여유있게
    우회하도록 유도한다 (바퀴 걸림 방지).

    height-scan ray 중 '바닥보다 솟은 지형(장애물)' 이면서 로봇 중심에서
    radius 안에 있는 셀 수를 센다.
    """
    sensor: RayCaster = env.scene.sensors[sensor_cfg.name]
    hits = sensor.data.ray_hits_w                           # (N, rays, 3)
    sensor_xy = sensor.data.pos_w[:, :2].unsqueeze(1)       # (N, 1, 2)
    horiz = torch.norm(hits[..., :2] - sensor_xy, dim=-1)   # (N, rays)
    hit_z = torch.nan_to_num(hits[..., 2], nan=0.0, posinf=0.0, neginf=0.0)
    near_obstacle = (hit_z > height_threshold) & (horiz < radius)
    count = torch.sum(near_obstacle.float(), dim=-1)
    return count / env.max_episode_length


def oscillation_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """행동(선속도/각속도)이 급변하면 페널티 — 부드러운 주행 유도."""
    diff = env.action_manager.action - env.action_manager.prev_action
    return torch.sum(diff * diff, dim=-1) / env.max_episode_length


def backward_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """후진(선속도 행동 < 0) 페널티 — 앞으로 가며 회피하도록 유도.

    ⚠️ 잔차(Residual) 액션에서는 행동[0]이 선속도가 아니라 조향 보정값이라
    의미가 없다. ResidualAckermannAction 을 쓰면 이 항목 대신
    steering_residual_penalty 를 사용한다.
    """
    is_backward = env.action_manager.action[:, 0] < 0.0
    return torch.where(
        is_backward,
        torch.ones(env.num_envs, device=env.device) / env.max_episode_length,
        torch.zeros(env.num_envs, device=env.device),
    )


def steering_residual_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """RL 조향 보정값의 크기 페널티 (잔차 RL 전용).

    잔차 RL 에서 정책은 베이스(goto-goal) 명령에 더할 조향 보정만 낸다.
    장애물이 없을 땐 보정 0(=베이스 경로 유지)이 바람직하므로, 보정 크기를
    약하게 벌해 '필요할 때만 비키도록' 유도한다.  충돌 페널티(-5)가 훨씬
    크므로 장애물 앞에서는 당연히 보정한다.
    """
    action = env.action_manager.action  # (N, 1) — 조향 보정값
    return torch.sum(action * action, dim=-1) / env.max_episode_length


def obstacle_hit_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("height_scanner"),
    radius: float = 0.6,
    height_threshold: float = 0.15,
) -> torch.Tensor:
    """장애물이 로봇 중심 radius(m) 안에 들어오면(=충돌) 페널티 1.0.

    몸체 접촉센서 기반 collision_penalty 가 못 잡는 '바퀴 걸림'(낮은 장애물)
    을 하향 RayCaster 로 잡아 명확한 충돌 페널티를 준다. terminations.py 의
    obstacle_hit 와 같은 판정.
    """
    sensor: RayCaster = env.scene.sensors[sensor_cfg.name]
    hits = sensor.data.ray_hits_w
    sensor_xy = sensor.data.pos_w[:, :2].unsqueeze(1)
    horiz = torch.norm(hits[..., :2] - sensor_xy, dim=-1)
    hit_z = torch.nan_to_num(hits[..., 2], nan=0.0, posinf=0.0, neginf=0.0)
    near_obstacle = (hit_z > height_threshold) & (horiz < radius)
    return torch.any(near_obstacle, dim=-1).float()
