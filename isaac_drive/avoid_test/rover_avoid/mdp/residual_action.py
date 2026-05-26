# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""잔차(residual) Ackermann 액션 — RL 은 회피(조향 보정)만 학습한다.

베이스 컨트롤러가 goal 좌표로 향하는 명령(선속도·각속도)을 규칙기반으로
만들고, RL 정책은 거기에 더할 **조향 보정값(1차원)** 만 출력한다.

  최종 각속도 = 베이스 각속도(goal 향함) + RL 조향 보정
  최종 선속도 = 베이스 선속도 (고정)

장애물이 없으면 보정 0 → 베이스 경로 그대로 goal 로 직진하고, 장애물이
있으면 보정으로 우회한다.  즉 'goal 찾아가기'는 베이스가 공짜로 해주고,
RL 은 '얼마나 비킬지'(회피)만 배운다.
"""

from __future__ import annotations

from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch
from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from .ackermann_actions import ackermann

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv  # noqa: F401


class ResidualAckermannAction(ActionTerm):
    """베이스 goto-goal 명령 + RL 조향 보정 → Ackermann 휠 명령.

    action_dim = 1 — 정책은 조향 보정값(rad/s 스케일 전)만 출력한다.
    """

    cfg: "ResidualAckermannActionCfg"
    _asset: Articulation

    def __init__(self, cfg: "ResidualAckermannActionCfg", env) -> None:
        super().__init__(cfg, env)

        # 조향/구동 조인트 id 찾기.
        self._drive_joint_ids, self._drive_joint_names = self._asset.find_joints(cfg.drive_joint_names)
        self._steering_joint_ids, self._steering_joint_names = self._asset.find_joints(
            cfg.steering_joint_names
        )

        # 조인트를 cfg 의 순서(FL,FR,...)대로 재정렬 — Ackermann 결과와 매칭.
        s_order, d_order = cfg.steering_order, cfg.drive_order
        sorted_steer = sorted(self._steering_joint_names, key=lambda x: s_order.index(x[:2]))
        sorted_drive = sorted(self._drive_joint_names, key=lambda x: d_order.index(x[:2]))
        s_pos = {n: i for i, n in enumerate(self._steering_joint_names)}
        d_pos = {n: i for i, n in enumerate(self._drive_joint_names)}
        self._sorted_steering_ids = [self._steering_joint_ids[s_pos[n]] for n in sorted_steer]
        self._sorted_drive_ids = [self._drive_joint_ids[d_pos[n]] for n in sorted_drive]

        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)

    @property
    def action_dim(self) -> int:
        return 1  # 조향 보정값 1차원

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor) -> None:
        # 정책 출력 → 보정 각속도 (rad/s).
        self._raw_actions[:] = actions
        self._processed_actions[:] = self._raw_actions * self.cfg.residual_scale

    def apply_actions(self) -> None:
        # --- 베이스 컨트롤러: goal 로 향하는 규칙기반 명령 ---
        # target_pose 커맨드는 베이스 프레임 목표위치 → [x_전방, y_좌측].
        cmd = self._env.command_manager.get_command(self.cfg.command_name)
        angle_to_goal = torch.atan2(cmd[:, 1], cmd[:, 0])  # 0=정면 +좌 -우
        base_ang = torch.clamp(
            self.cfg.heading_gain * angle_to_goal,
            -self.cfg.max_base_ang,
            self.cfg.max_base_ang,
        )
        base_lin = torch.full_like(base_ang, self.cfg.base_speed)

        # --- RL 조향 보정 더하기 ---
        final_ang = base_ang + self._processed_actions[:, 0]

        # --- Ackermann 변환 → 휠 명령 ---
        joint_pos, joint_vel = ackermann(base_lin, final_ang, self.cfg, self.device)
        self._asset.set_joint_velocity_target(joint_vel, joint_ids=self._sorted_drive_ids)
        self._asset.set_joint_position_target(joint_pos, joint_ids=self._sorted_steering_ids)


@configclass
class ResidualAckermannActionCfg(ActionTermCfg):
    """잔차 Ackermann 액션 설정 — RL 은 조향 보정(1D)만 출력."""

    class_type: type[ActionTerm] = ResidualAckermannAction

    # --- Ackermann 기하 (ackermann() 가 cfg 에서 직접 읽음) ---
    wheelbase_length: float = MISSING
    middle_wheel_distance: float = MISSING
    rear_and_front_wheel_distance: float = MISSING
    wheel_radius: float = MISSING
    min_steering_radius: float = 0.8
    offset: float = 0.0  # Ackermann 기하 offset (ACK_OFFSET)
    steering_joint_names: list[str] = MISSING
    drive_joint_names: list[str] = MISSING
    steering_order = ["FL", "FR", "RL", "RR"]
    drive_order = ["FL", "FR", "CL", "CR", "RL", "RR"]

    # --- 베이스 goto-goal 컨트롤러 ---
    command_name: str = "target_pose"
    """베이스 컨트롤러가 향할 목표 커맨드 이름."""
    base_speed: float = 0.8
    """베이스 전진 선속도 (m/s) — 고정."""
    heading_gain: float = 2.0
    """base_ang = heading_gain × (목표 방위각). 클수록 빠르게 목표를 향함."""
    max_base_ang: float = 1.0
    """베이스 각속도 클램프 (rad/s)."""

    # --- RL 조향 보정 ---
    residual_scale: float = 1.0
    """정책 출력값 → 보정 각속도 (rad/s) 스케일."""
