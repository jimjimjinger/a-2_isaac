"""recovery_mdp.py — Observation / Reward / Termination / Event 함수 모음.

Isaac Lab manager-based 환경에서 각 term이 호출하는 순수 함수들.
모두 (env, ...) 시그니처를 가지며 torch.Tensor를 반환한다.
"""
from __future__ import annotations

import math
import torch
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz


# ── Observation 함수 ─────────────────────────────────────────────────────────

def _rover_euler(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")):
    rover = env.scene[asset_cfg.name]
    quat = rover.data.root_quat_w          # (N, 4) wxyz
    roll, pitch, yaw = euler_xyz_from_quat(quat)
    return roll, pitch, yaw


def rover_roll(env: ManagerBasedRLEnv,
               asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    r, _, _ = _rover_euler(env, asset_cfg)
    return r.unsqueeze(-1)


def rover_pitch(env: ManagerBasedRLEnv,
                asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    _, p, _ = _rover_euler(env, asset_cfg)
    return p.unsqueeze(-1)


def rover_yaw(env: ManagerBasedRLEnv,
              asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    _, _, y = _rover_euler(env, asset_cfg)
    return y.unsqueeze(-1)


def rover_pos_z(env: ManagerBasedRLEnv,
                asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    rover = env.scene[asset_cfg.name]
    return rover.data.root_pos_w[:, 2:3]


def rover_lin_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    rover = env.scene[asset_cfg.name]
    return rover.data.root_lin_vel_w        # (N, 3)


def rover_ang_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    rover = env.scene[asset_cfg.name]
    return rover.data.root_ang_vel_w        # (N, 3)


def arm_joint_pos(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("m0609")) -> torch.Tensor:
    arm: Articulation = env.scene[asset_cfg.name]
    return arm.data.joint_pos              # (N, 6)


def arm_joint_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("m0609")) -> torch.Tensor:
    arm: Articulation = env.scene[asset_cfg.name]
    return arm.data.joint_vel              # (N, 6)


def rover_drive_vel(env: ManagerBasedRLEnv,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    """drive 바퀴 6개 각속도 — SceneEntityCfg(joint_names=[".*Drive_Continuous"]) 필요."""
    rover: Articulation = env.scene[asset_cfg.name]
    return rover.data.joint_vel[:, asset_cfg.joint_ids]   # (N, 6)


def rover_steer_pos(env: ManagerBasedRLEnv,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    """steer 관절 4개 위치 — SceneEntityCfg(joint_names=[".*Steer_Revolute"]) 필요."""
    rover: Articulation = env.scene[asset_cfg.name]
    return rover.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 4)


# ── Reward 함수 ─────────────────────────────────────────────────────────────

def upright_cosine_reward(env: ManagerBasedRLEnv,
                          asset_cfg: SceneEntityCfg = SceneEntityCfg("rover")) -> torch.Tensor:
    """cos(roll)*cos(pitch): upright=1.0, 90° 옆으로=0.0. 강한 shaping signal."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    return torch.cos(roll) * torch.cos(pitch)    # (N,)


def height_reward(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                  fallen_z: float = 0.30,
                  upright_z: float = 0.60) -> torch.Tensor:
    """rover 몸체 높이를 [0, 1]로 정규화. 기립할수록 큰 보상."""
    rover = env.scene[asset_cfg.name]
    pos_z = rover.data.root_pos_w[:, 2]
    return torch.clamp((pos_z - fallen_z) / (upright_z - fallen_z), 0.0, 1.0)


def success_bonus(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                  threshold_deg: float = 15.0) -> torch.Tensor:
    """roll/pitch 모두 threshold 이하일 때 1.0 (weight=500으로 큰 sparse 보상)."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    upright = (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)
    return upright.float()                       # (N,)


def fallen_penalty(env: ManagerBasedRLEnv,
                   asset_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                   tilt_threshold_deg: float = 75.0) -> torch.Tensor:
    """tilt > threshold 상태가 지속되면 1.0 (weight<0으로 페널티). 정체 방지."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    thr = math.radians(tilt_threshold_deg)
    return (tilt > thr).float()                  # (N,)


def time_alive_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """매 스텝 1.0 (weight<0으로 시간 압박 패널티). 빠른 기립 유도."""
    return torch.ones(env.num_envs, device=env.device)


def wheel_drive_reward(env: ManagerBasedRLEnv,
                       rover_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                       tilt_threshold_deg: float = 45.0) -> torch.Tensor:
    """넘어진 상태(tilt > threshold)에서 바퀴를 돌릴수록 [0, 1] 보상."""
    roll, pitch, _ = _rover_euler(env, rover_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    fallen = (tilt > math.radians(tilt_threshold_deg)).float()

    rover: Articulation = env.scene[rover_cfg.name]
    wheel_vel = rover.data.joint_vel[:, rover_cfg.joint_ids]
    wheel_rms = torch.sqrt(torch.mean(wheel_vel ** 2, dim=-1) + 1e-6)

    return fallen * torch.clamp(wheel_rms / 10.0, 0.0, 1.0)   # 10 rad/s를 1.0으로 정규화


def joint_vel_penalty(env: ManagerBasedRLEnv,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("m0609")) -> torch.Tensor:
    arm: Articulation = env.scene[asset_cfg.name]
    return torch.sum(arm.data.joint_vel ** 2, dim=-1)  # (N,)


def joint_limit_penalty(env: ManagerBasedRLEnv,
                        asset_cfg: SceneEntityCfg = SceneEntityCfg("m0609"),
                        margin: float = 0.1) -> torch.Tensor:
    """관절이 소프트 한계에 접근하면 페널티."""
    arm: Articulation = env.scene[asset_cfg.name]
    pos = arm.data.joint_pos
    lo  = arm.data.soft_joint_pos_limits[..., 0]
    hi  = arm.data.soft_joint_pos_limits[..., 1]
    exceed = torch.clamp(lo + margin - pos, min=0.0) + \
             torch.clamp(pos - (hi - margin), min=0.0)
    return exceed.sum(dim=-1)                          # (N,)


# ── Termination 함수 ─────────────────────────────────────────────────────────

def rover_upright_termination(env: ManagerBasedRLEnv,
                               asset_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                               threshold_deg: float = 15.0) -> torch.Tensor:
    """rover가 upright 상태에 도달하면 에피소드 성공 종료."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    return (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)  # (N,) bool


# ── Event 함수 (초기화) ───────────────────────────────────────────────────────

def reset_rover_fallen(env: ManagerBasedRLEnv,
                        env_ids: torch.Tensor,
                        asset_cfg: SceneEntityCfg = SceneEntityCfg("rover"),
                        roll_range: tuple = (1.047, 2.094),
                        pitch_range: tuple = (-0.524, 0.524)) -> None:
    """Rover를 랜덤으로 넘어진 자세로 초기화. 바퀴/스티어 관절도 0으로 리셋."""
    rover: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    roll  = torch.empty(n, device=device).uniform_(*roll_range)
    pitch = torch.empty(n, device=device).uniform_(*pitch_range)
    yaw   = torch.zeros(n, device=device)
    quat  = quat_from_euler_xyz(roll, pitch, yaw)

    pos = torch.zeros(n, 3, device=device)
    pos[:, 0] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 1] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 2] = 0.5   # articulation은 바퀴 높이 포함 — 약간 높게 시작

    rover.write_root_pose_to_sim(
        torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    rover.write_root_velocity_to_sim(
        torch.zeros(n, 6, device=device), env_ids=env_ids)

    # 바퀴/스티어 관절을 기본값+제로 속도로 리셋
    default_pos = rover.data.default_joint_pos[env_ids]
    default_vel = torch.zeros_like(default_pos)
    rover.write_joint_state_to_sim(default_pos, default_vel, env_ids=env_ids)


def reset_arm_default(env: ManagerBasedRLEnv,
                       env_ids: torch.Tensor,
                       asset_cfg: SceneEntityCfg = SceneEntityCfg("m0609")) -> None:
    """M0609 팔을 홈 자세로 초기화."""
    arm: Articulation = env.scene[asset_cfg.name]
    default_pos = arm.data.default_joint_pos[env_ids]
    default_vel = torch.zeros_like(default_pos)
    arm.write_joint_state_to_sim(default_pos, default_vel, env_ids=env_ids)
