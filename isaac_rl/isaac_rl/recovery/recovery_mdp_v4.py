"""recovery_mdp_v4.py — v4 추가 MDP 함수.

v3 recovery_mdp의 모든 함수를 재수출하고 v4 전용 함수를 추가한다.

추가 기능:
  - v4 에피소드 상태 관리 (_v4 dict:
      recovered / recovered_pos / recovered_yaw / recovered_fwd_xy /
      mission_done / bonus_given / self_collision_hit)
  - Forward phase obs: base_forward_dir, forward_vel, forward_distance
  - State flag obs:    v4_state_update_obs (side-effect master), mission_resume_flag_obs
  - Wheel obs:         wheel_velocities_obs
  - Forward rewards:   forward_velocity_reward, forward_progress_reward, forward_ready_reward_v4
  - Rocking rewards:   rocking_motion_reward (pre-recovery),
                       rocking_oscillation_penalty (post-recovery)
  - Sparse:            mission_resume_success_bonus
  - Penalty:           self_collision_penalty (contact sensor 기반 one-shot)
  - Termination:       mission_success_termination
  - Reset:             reset_vehicle_random_fall_v4 (v4 state 초기화 포함)
"""
from __future__ import annotations

import math
import torch

from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

# ── v3 함수 전체 재수출 ──────────────────────────────────────────────────────
from recovery_mdp import *          # noqa: F401, F403
from recovery_mdp import (          # private helpers (star-import 제외)
    _rover_euler,
    _get_stable_frames,
    _local_z_in_world,
    reset_vehicle_random_fall,
)


# ── v4 내부 상태 ─────────────────────────────────────────────────────────────

def _get_v4(env: ManagerBasedRLEnv) -> dict:
    """Lazy-init: 에피소드 상태 dict를 env에 붙여 유지."""
    if not hasattr(env, "_v4"):
        N, dev = env.num_envs, env.device
        env._v4 = {
            "recovered":          torch.zeros(N, dtype=torch.bool, device=dev),
            "recovered_pos":      torch.zeros(N, 3,                 device=dev),
            "recovered_yaw":      torch.zeros(N,                    device=dev),
            "recovered_fwd_xy":   torch.zeros(N, 2,                 device=dev),
            "mission_done":       torch.zeros(N, dtype=torch.bool, device=dev),
            "bonus_given":        torch.zeros(N, dtype=torch.bool, device=dev),
            "self_collision_hit": torch.zeros(N, dtype=torch.bool, device=dev),
        }
    return env._v4


# ── v4 Observation 함수 ──────────────────────────────────────────────────────

def v4_state_update_obs(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    wheel_sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
    required_stable: int = 8,
    forward_goal_m: float = 2.0,
    threshold_deg: float = 15.0,
    min_wheel_contact_ratio: float = 0.5,
) -> torch.Tensor:
    """v4 에피소드 상태를 매 스텝 갱신하고 recovered_flag를 obs로 반환. (N, 1)

    Side-effect:
      - recovered     : stable_frames >= required AND |roll|,|pitch| < thr → True
      - recovered_pos : 최초 복원 시점의 world 위치 기록 (이후 불변)
      - mission_done  : recovered 후 XY 이동 거리 >= goal → True

    반드시 stable_frames_normalized 이후 ObsGroup 순서에 배치해야 함.
    """
    st   = _get_v4(env)
    sf   = _get_stable_frames(env)
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr  = math.radians(threshold_deg)

    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_w = vehicle.data.root_pos_w  # (N, 3)
    _, _, yaw = _rover_euler(env, asset_cfg)

    # ── recovered 갱신 ─────────────────────────────────────────────────────
    is_stable = (sf >= required_stable) & (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)
    newly_rec = is_stable & ~st["recovered"]
    st["recovered_pos"][newly_rec] = pos_w[newly_rec].clone()
    st["recovered_yaw"][newly_rec] = yaw[newly_rec].clone()
    st["recovered_fwd_xy"][newly_rec] = torch.stack(
        [torch.cos(yaw[newly_rec]), torch.sin(yaw[newly_rec])],
        dim=-1,
    )
    st["recovered"][newly_rec]     = True

    # ── wheel contact gate ────────────────────────────────────────────────
    wheel_contact_ok = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    try:
        wheel_sensor: ContactSensor = env.scene[wheel_sensor_cfg.name]
        wheel_force_mag = torch.nan_to_num(
            torch.norm(wheel_sensor.data.net_forces_w, dim=-1), nan=0.0
        )
        wheel_contact_ratio = (wheel_force_mag > 1.0).float().mean(dim=-1)
        wheel_contact_ok = wheel_contact_ratio >= min_wheel_contact_ratio
    except KeyError:
        pass

    # ── mission_done 갱신 ─────────────────────────────────────────────────
    disp_xy = pos_w[:, :2] - st["recovered_pos"][:, :2]
    forward_distance = (disp_xy * st["recovered_fwd_xy"]).sum(dim=-1).clamp_min(0.0)
    newly_done = (
        st["recovered"]
        & wheel_contact_ok
        & (forward_distance >= forward_goal_m)
        & ~st["mission_done"]
    )
    st["mission_done"][newly_done] = True

    return st["recovered"].float().unsqueeze(-1)  # (N, 1)


def mission_resume_flag_obs(env: ManagerBasedRLEnv) -> torch.Tensor:
    """전진 goal 달성 flag. (N, 1) — v4_state_update_obs 이후 호출."""
    return _get_v4(env)["mission_done"].float().unsqueeze(-1)


def base_forward_dir_obs(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """rover 전방 방향을 (cos_yaw, sin_yaw)로 인코딩. (N, 2)"""
    _, _, yaw = _rover_euler(env, asset_cfg)
    return torch.stack([torch.cos(yaw), torch.sin(yaw)], dim=-1)


def forward_vel_obs(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """rover 전방 방향 선속도 투영값. (N, 1)"""
    vehicle: Articulation = env.scene[asset_cfg.name]
    lin_vel = vehicle.data.root_lin_vel_w         # (N, 3)
    q = vehicle.data.root_quat_w                  # (N, 4) wxyz
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # local +X in world frame (rotation matrix row 0)
    fwd = torch.stack([
        1 - 2*(y*y + z*z),
        2*(x*y + w*z),
        2*(x*z - w*y),
    ], dim=-1)                                    # (N, 3)
    return (lin_vel * fwd).sum(dim=-1, keepdim=True)  # (N, 1)


def forward_distance_obs(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """recovered 후 전진 거리(복원 시점 heading projection). 복원 전 = 0. (N, 1)"""
    st = _get_v4(env)
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_w = vehicle.data.root_pos_w
    disp_xy = pos_w[:, :2] - st["recovered_pos"][:, :2]
    forward_distance = (disp_xy * st["recovered_fwd_xy"]).sum(dim=-1).clamp_min(0.0)
    return (forward_distance * st["recovered"].float()).unsqueeze(-1)  # (N, 1)


def wheel_velocities_obs(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """drive wheel joint 속도 [rad/s]. (N, 6)

    asset_cfg에 joint_names=[".*Drive_Continuous"] 지정.
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_vel[:, asset_cfg.joint_ids]  # (N, 6)


# ── v4 Reward 함수 ────────────────────────────────────────────────────────────

def forward_velocity_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    max_vel: float = 0.8,
) -> torch.Tensor:
    """recovered 상태에서 전방 속도에 비례한 보상. (N,)

    recovered=False (누운 상태) 에서는 보상 0 → 바퀴만 굴러 이동 불가.
    """
    st = _get_v4(env)
    vehicle: Articulation = env.scene[asset_cfg.name]
    lin_vel = vehicle.data.root_lin_vel_w
    q = vehicle.data.root_quat_w
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    fwd = torch.stack([1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)], dim=-1)
    fwd_vel = (lin_vel * fwd).sum(dim=-1).clamp(0.0, max_vel)
    return st["recovered"].float() * (fwd_vel / max_vel)


def forward_progress_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    goal_m: float = 2.0,
) -> torch.Tensor:
    """recovered 후 목표 거리 달성 비율 [0,1]. (N,)"""
    st = _get_v4(env)
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_w = vehicle.data.root_pos_w
    disp_xy = pos_w[:, :2] - st["recovered_pos"][:, :2]
    forward_distance = (disp_xy * st["recovered_fwd_xy"]).sum(dim=-1).clamp_min(0.0)
    return st["recovered"].float() * (forward_distance / goal_m).clamp(0.0, 1.0)


def forward_ready_reward_v4(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold_deg: float = 20.0,
) -> torch.Tensor:
    """recovered + upright + 저속 → 전진 준비 완료 시그널. (N,)"""
    st = _get_v4(env)
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    is_upright = (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)

    vehicle: Articulation = env.scene[asset_cfg.name]
    low_speed = torch.norm(vehicle.data.root_lin_vel_w, dim=-1) < 0.3

    return st["recovered"].float() * (is_upright & low_speed).float()


def rocking_motion_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("arm_contact_sensor"),
    min_tilt_deg: float = 20.0,
    max_roll_rate: float = 2.0,
) -> torch.Tensor:
    """recovered 이전, arm-ground contact 상태에서 roll rocking을 유도하는 보상. (N,)

    - upright 되기 전까지만 보상
    - arm-ground contact가 있을 때만 활성화
    - roll angular velocity가 커질수록 증가
    """
    st = _get_v4(env)
    if torch.all(st["recovered"]):
        return torch.zeros(env.num_envs, device=env.device)

    vehicle: Articulation = env.scene[asset_cfg.name]
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.sqrt(roll ** 2 + pitch ** 2)
    pre_recovery = (~st["recovered"]) & (tilt > math.radians(min_tilt_deg))

    try:
        sensor: ContactSensor = env.scene[sensor_cfg.name]
        force_mag = torch.nan_to_num(
            torch.norm(sensor.data.net_forces_w, dim=-1), nan=0.0
        )
        arm_contact = (force_mag > 1.0).any(dim=-1)
    except KeyError:
        arm_contact = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    roll_rate = torch.abs(vehicle.data.root_ang_vel_w[:, 0])
    roll_score = torch.clamp(roll_rate / max_roll_rate, 0.0, 1.0)
    return pre_recovery.float() * arm_contact.float() * roll_score


def rocking_oscillation_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    max_roll_rate: float = 1.5,
) -> torch.Tensor:
    """recovered 이후 rocking/oscillation을 억제하는 패널티. (N,)

    recovered 이전에는 0, recovered 이후 roll angular velocity에 비례해 패널티.
    """
    st = _get_v4(env)
    vehicle: Articulation = env.scene[asset_cfg.name]
    roll_rate = torch.abs(vehicle.data.root_ang_vel_w[:, 0])
    return st["recovered"].float() * torch.clamp(roll_rate / max_roll_rate, 0.0, 1.0)


def mission_resume_success_bonus(env: ManagerBasedRLEnv) -> torch.Tensor:
    """전진 goal 달성 시 episode당 1회만 지급 (bonus_given 플래그). (N,)"""
    st = _get_v4(env)
    eligible = st["mission_done"] & ~st["bonus_given"]
    st["bonus_given"][eligible] = True
    return eligible.float()  # weight=200.0 이 곱해짐


def self_collision_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("self_collision_sensor"),
    force_threshold: float = 5.0,
) -> torch.Tensor:
    """arm-chassis 비정상 contact를 episode당 1회만 패널티로 반환. (N,)

    ContactSensor는 arm bodies vs chassis body로만 구성한다.
    """
    st = _get_v4(env)
    try:
        sensor: ContactSensor = env.scene[sensor_cfg.name]
    except KeyError:
        return torch.zeros(env.num_envs, device=env.device)

    force_mag = torch.nan_to_num(
        torch.norm(sensor.data.net_forces_w, dim=-1), nan=0.0
    )  # (N, num_sensor_bodies)
    hit = (force_mag > force_threshold).any(dim=-1)
    newly_hit = hit & ~st["self_collision_hit"]
    st["self_collision_hit"][newly_hit] = True
    return newly_hit.float()


# ── v4 Termination ────────────────────────────────────────────────────────────

def mission_success_termination(
    env: ManagerBasedRLEnv,
    forward_goal_m: float = 2.0,
) -> torch.Tensor:
    """최종 성공 종료: recovered 후 전진 goal 달성. (N,) bool"""
    return _get_v4(env)["mission_done"]


# ── v4 Reset ─────────────────────────────────────────────────────────────────

def reset_vehicle_random_fall_v4(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    height: float = 0.5,
) -> None:
    """v3 전복 reset + v4 에피소드 상태 초기화."""
    # v3 reset (pose / joint / velocity)
    reset_vehicle_random_fall(env, env_ids, asset_cfg, height)

    # v4 state 초기화
    if hasattr(env, "_v4"):
        st = env._v4
        st["recovered"][env_ids]          = False
        st["recovered_pos"][env_ids]      = 0.0
        st["recovered_yaw"][env_ids]      = 0.0
        st["recovered_fwd_xy"][env_ids]   = 0.0
        st["mission_done"][env_ids]       = False
        st["bonus_given"][env_ids]        = False
        st["self_collision_hit"][env_ids] = False
