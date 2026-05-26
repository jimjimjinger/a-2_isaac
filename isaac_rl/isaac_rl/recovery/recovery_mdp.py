"""recovery_mdp.py — Observation / Reward / Termination / Event 함수 모음.

Isaac Lab manager-based 환경에서 각 term이 호출하는 순수 함수들.
vehicle_v3.usd의 rover + m0609가 하나의 articulation으로 로드됨.
ArticulationRoot = m0609/base_link (rover body에 고정됨).
"""
from __future__ import annotations

import math
import torch
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz


# ── Observation 함수 ─────────────────────────────────────────────────────────

def _rover_euler(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")):
    vehicle: Articulation = env.scene[asset_cfg.name]
    quat = vehicle.data.root_quat_w          # (N, 4) wxyz
    roll, pitch, yaw = euler_xyz_from_quat(quat)
    return roll, pitch, yaw


def rover_roll(env: ManagerBasedRLEnv,
               asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    r, _, _ = _rover_euler(env, asset_cfg)
    return r.unsqueeze(-1)


def rover_pitch(env: ManagerBasedRLEnv,
                asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    _, p, _ = _rover_euler(env, asset_cfg)
    return p.unsqueeze(-1)


def rover_yaw(env: ManagerBasedRLEnv,
              asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    _, _, y = _rover_euler(env, asset_cfg)
    return y.unsqueeze(-1)


def rover_pos_z(env: ManagerBasedRLEnv,
                asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_pos_w[:, 2:3]


def rover_lin_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_lin_vel_w        # (N, 3)


def rover_ang_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_ang_vel_w        # (N, 3)


def arm_joint_pos(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """joint_ids는 SceneEntityCfg(joint_names=["joint_[1-6]"])로 필터링."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 6)


def arm_joint_vel(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_vel[:, asset_cfg.joint_ids]   # (N, 6)


def rover_drive_vel(env: ManagerBasedRLEnv,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """SceneEntityCfg(joint_names=[".*Drive_Continuous"]) 필요."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_vel[:, asset_cfg.joint_ids]   # (N, 6)


def rover_steer_pos(env: ManagerBasedRLEnv,
                    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """SceneEntityCfg(joint_names=[".*Steer_Revolute"]) 필요."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 4)


# ── Reward 함수 ─────────────────────────────────────────────────────────────

def upright_cosine_reward(env: ManagerBasedRLEnv,
                          asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """cos(roll)*cos(pitch): upright=1.0, 90° 기울어짐=0.0."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    return torch.cos(roll) * torch.cos(pitch)    # (N,)


def height_reward(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                  fallen_z: float = 0.2,
                  upright_z: float = 0.7) -> torch.Tensor:
    """articulation root(m0609 base_link) 높이를 [0,1]로 정규화. 기립할수록 큰 보상."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_z = vehicle.data.root_pos_w[:, 2]
    return torch.clamp((pos_z - fallen_z) / (upright_z - fallen_z), 0.0, 1.0)


def success_bonus(env: ManagerBasedRLEnv,
                  asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                  threshold_deg: float = 15.0) -> torch.Tensor:
    """roll/pitch 모두 threshold 이하일 때 1.0."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    upright = (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)
    return upright.float()


def fallen_penalty(env: ManagerBasedRLEnv,
                   asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                   tilt_threshold_deg: float = 75.0) -> torch.Tensor:
    """tilt > threshold 상태면 1.0 (weight<0으로 패널티)."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    thr = math.radians(tilt_threshold_deg)
    return (tilt > thr).float()


def time_alive_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """매 스텝 1.0 (weight<0으로 시간 압박 패널티)."""
    return torch.ones(env.num_envs, device=env.device)


def wheel_drive_reward(env: ManagerBasedRLEnv,
                       vehicle_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                       tilt_threshold_deg: float = 45.0) -> torch.Tensor:
    """넘어진 상태(tilt > threshold)에서 drive 바퀴를 돌릴수록 [0, 1] 보상."""
    roll, pitch, _ = _rover_euler(env, vehicle_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    fallen = (tilt > math.radians(tilt_threshold_deg)).float()

    vehicle: Articulation = env.scene[vehicle_cfg.name]
    wheel_vel = vehicle.data.joint_vel[:, vehicle_cfg.joint_ids]
    wheel_rms = torch.sqrt(torch.mean(wheel_vel ** 2, dim=-1) + 1e-6)

    return fallen * torch.clamp(wheel_rms / 10.0, 0.0, 1.0)


def joint_vel_penalty(env: ManagerBasedRLEnv,
                      asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """SceneEntityCfg(joint_names=["joint_[1-6]"])로 arm 관절만 필터링."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return torch.sum(vehicle.data.joint_vel[:, asset_cfg.joint_ids] ** 2, dim=-1)


def rocker_joint_pos(env: ManagerBasedRLEnv,
                     asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """rocker-bogie 서스펜션 관절 위치. SceneEntityCfg에 joint_names 필터 필요."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 5)


def suspension_misalignment_penalty(env: ManagerBasedRLEnv,
                                    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """rocker-bogie 관절이 중립(0°)에서 벗어난 총 편차. weight<0으로 패널티."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]    # (N, 5)
    return torch.sum(pos ** 2, dim=-1)


def steer_misalignment_penalty(env: ManagerBasedRLEnv,
                                asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """steer 관절이 중립(0°)에서 벗어난 총 편차. upright 후 정렬 유도. weight<0으로 패널티."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]    # (N, 4)
    # upright 상태에서만 패널티 적용 (넘어진 동안은 steer 자유롭게 사용)
    quat = vehicle.data.root_quat_w
    roll, pitch, _ = euler_xyz_from_quat(quat)
    tilt = torch.abs(roll) + torch.abs(pitch)
    upright_mask = (tilt < math.radians(30.0)).float()
    return upright_mask * torch.sum(pos ** 2, dim=-1)


def near_success_reward(env: ManagerBasedRLEnv,
                        asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                        sigma_deg: float = 30.0) -> torch.Tensor:
    """Gaussian 보상: upright에 가까울수록 급격히 커짐. success_bonus(15°) 앞단 gradient 제공."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    sigma = math.radians(sigma_deg)
    return torch.exp(-0.5 * (roll ** 2 + pitch ** 2) / (sigma ** 2))


def recovery_angular_vel_reward(env: ManagerBasedRLEnv,
                                asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle")) -> torch.Tensor:
    """기울기 감소 방향 각속도를 보상. 적극적인 자세 복원 행동을 유도."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    vehicle: Articulation = env.scene[asset_cfg.name]
    ang_vel = vehicle.data.root_ang_vel_w   # (N, 3) world frame
    # roll > 0이면 ang_vel_x < 0 (반시계)이 복원 방향, roll < 0이면 반대
    roll_recovery  = -roll  * ang_vel[:, 0]
    pitch_recovery = -pitch * ang_vel[:, 1]
    return torch.clamp((roll_recovery + pitch_recovery) / (math.pi * 2.0), -1.0, 1.0)


def joint_limit_penalty(env: ManagerBasedRLEnv,
                        asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                        margin: float = 0.1) -> torch.Tensor:
    """arm 관절이 소프트 한계에 접근하면 패널티."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]
    lo  = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    hi  = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]
    exceed = torch.clamp(lo + margin - pos, min=0.0) + \
             torch.clamp(pos - (hi - margin), min=0.0)
    return exceed.sum(dim=-1)


# ── Termination 함수 ─────────────────────────────────────────────────────────

def rover_upright_termination(env: ManagerBasedRLEnv,
                               asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                               threshold_deg: float = 15.0) -> torch.Tensor:
    """rover가 upright 상태에 도달하면 에피소드 성공 종료."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    return (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)  # (N,) bool


# ── Event 함수 (초기화) ───────────────────────────────────────────────────────

def reset_vehicle_fallen(env: ManagerBasedRLEnv,
                         env_ids: torch.Tensor,
                         asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
                         roll_range: tuple = (1.047, 2.094),
                         pitch_range: tuple = (-0.524, 0.524)) -> None:
    """Vehicle(rover+arm)를 넘어진 자세로 초기화. arm 관절도 기본값으로 리셋."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    roll  = torch.empty(n, device=device).uniform_(*roll_range)
    # 좌우 양방향 낙하 학습: 50% 확률로 반대 방향 초기화
    sign  = torch.randint(0, 2, (n,), device=device).float() * 2 - 1
    roll  = roll * sign
    pitch = torch.empty(n, device=device).uniform_(*pitch_range)
    yaw   = torch.zeros(n, device=device)
    quat  = quat_from_euler_xyz(roll, pitch, yaw)

    pos = torch.zeros(n, 3, device=device)
    pos[:, 0] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 1] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 2] = 0.5

    vehicle.write_root_pose_to_sim(
        torch.cat([pos, quat], dim=-1), env_ids=env_ids)
    vehicle.write_root_velocity_to_sim(
        torch.zeros(n, 6, device=device), env_ids=env_ids)

    # 모든 관절(arm + drive + steer)을 기본값으로 리셋
    default_pos = vehicle.data.default_joint_pos[env_ids]
    default_vel = torch.zeros_like(default_pos)
    vehicle.write_joint_state_to_sim(default_pos, default_vel, env_ids=env_ids)
