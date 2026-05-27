"""recovery_mdp.py — Observation / Reward / Termination / Event 함수 전체 구현.

vehicle_v3.usd의 rover + m0609 단일 articulation 기반.
ArticulationRoot: /Root/Vehicle/m0609/base_link

stable_frames 상태 추적:
  - _recovery_stable_frames: env 객체에 붙여두는 (num_envs,) long tensor
  - stable_frames_normalized() (observation term) 에서 매 스텝 갱신
  - reset functions 에서 env_ids에 해당하는 값을 0으로 초기화
  - reward/termination functions에서 read-only 접근
"""
from __future__ import annotations

import math
import torch

from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz


# ── Internal helpers ─────────────────────────────────────────────────────────

def _rover_euler(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
):
    vehicle: Articulation = env.scene[asset_cfg.name]
    return euler_xyz_from_quat(vehicle.data.root_quat_w)  # roll, pitch, yaw


def _get_stable_frames(env: ManagerBasedRLEnv) -> torch.Tensor:
    """lazy-init: env에 stable_frames counter 없으면 생성."""
    if not hasattr(env, "_recovery_stable_frames"):
        env._recovery_stable_frames = torch.zeros(
            env.num_envs, device=env.device, dtype=torch.long
        )
    return env._recovery_stable_frames


def _local_z_in_world(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """vehicle local +Z 축을 world frame에 표현. shape (N, 3).

    upright = (0, 0, 1),  옆으로 넘어짐 ≈ (±1, 0, 0),  뒤집힘 = (0, 0, -1)
    quaternion wxyz 기준 회전 행렬 3열:
      [2(xz+wy), 2(yz-wx), 1-2(x²+y²)]
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    q = vehicle.data.root_quat_w            # (N, 4)  w, x, y, z
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return torch.stack(
        [
            2 * (x * z + w * y),
            2 * (y * z - w * x),
            1 - 2 * (x * x + y * y),
        ],
        dim=-1,
    )


# ── Observation 함수 ─────────────────────────────────────────────────────────

def upright_vec(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """vehicle local +Z 를 world frame으로 투영. shape (N, 3)."""
    return _local_z_in_world(env, asset_cfg)


def rover_roll(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    r, _, _ = _rover_euler(env, asset_cfg)
    return r.unsqueeze(-1)                  # (N, 1)


def rover_pitch(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    _, p, _ = _rover_euler(env, asset_cfg)
    return p.unsqueeze(-1)


def rover_yaw(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    _, _, y = _rover_euler(env, asset_cfg)
    return y.unsqueeze(-1)


def rover_pos_z(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_pos_w[:, 2:3]  # (N, 1)


def rover_lin_vel(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_lin_vel_w      # (N, 3)


def rover_ang_vel(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.root_ang_vel_w      # (N, 3)


def arm_joint_pos(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 6)


def arm_joint_vel(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_vel[:, asset_cfg.joint_ids]   # (N, 6)


def rover_drive_vel(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_vel[:, asset_cfg.joint_ids]


def rover_steer_pos(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]


def rocker_joint_pos(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    vehicle: Articulation = env.scene[asset_cfg.name]
    return vehicle.data.joint_pos[:, asset_cfg.joint_ids]   # (N, 5)


def arm_ee_pos(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """tool0 end-effector의 루트 기준 상대 위치 (world frame 차분). shape (N, 3).

    asset_cfg에 body_names=["tool0"] 이 지정되어야 한다.
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    ee_pos_w   = vehicle.data.body_pos_w[:, asset_cfg.body_ids[0], :]  # (N, 3)
    root_pos_w = vehicle.data.root_pos_w                                # (N, 3)
    return ee_pos_w - root_pos_w


def wheel_contact_obs(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
) -> torch.Tensor:
    """바퀴-지형 접촉 비율 [0,1]. shape (N, 1).

    ContactSensor의 net_forces_w 크기 > 1 N 인 body 비율을 반환.
    """
    sensor: ContactSensor = env.scene[sensor_cfg.name]
    force_mag = torch.nan_to_num(
        torch.norm(sensor.data.net_forces_w, dim=-1), nan=0.0
    )  # (N, num_bodies)
    frac = (force_mag > 1.0).float().mean(dim=-1, keepdim=True)  # (N, 1)
    return frac


def stable_frames_normalized(
    env: ManagerBasedRLEnv,
    required: int = 30,
) -> torch.Tensor:
    """연속 upright 유지 프레임 수를 [0,1]로 정규화해 반환. (N, 1)

    side-effect: env._recovery_stable_frames 를 매 스텝 갱신.
      - upright (|roll|,|pitch| < 15°): +1 증가
      - 그 외: 0으로 리셋
    Observations는 Rewards/Terminations보다 먼저 계산되므로
    같은 스텝 내 reward/termination에서 안전하게 읽을 수 있다.
    """
    vehicle: Articulation = env.scene["vehicle"]
    roll, pitch, _ = euler_xyz_from_quat(vehicle.data.root_quat_w)
    thr = math.radians(15.0)

    sf = _get_stable_frames(env)
    is_upright = (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)
    sf = torch.where(is_upright, sf + 1, torch.zeros_like(sf))
    env._recovery_stable_frames = sf

    return (sf.float() / max(required, 1)).clamp(0.0, 1.0).unsqueeze(-1)


# ── Dense reward 함수 ─────────────────────────────────────────────────────────

def upright_cosine_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """(cos(roll)·cos(pitch)+1)/2: upright=1.0, 90°기울=0.5, 뒤집힘=0.0 (항상 양수)."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    return (torch.cos(roll) * torch.cos(pitch) + 1.0) * 0.5  # (N,) range [0,1]


def height_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    fallen_z: float = 0.2,
    upright_z: float = 0.7,
) -> torch.Tensor:
    """articulation root 높이를 [0,1]로 정규화. 기립할수록 큰 보상."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_z = vehicle.data.root_pos_w[:, 2]
    return torch.clamp((pos_z - fallen_z) / (upright_z - fallen_z), 0.0, 1.0)


def stable_upright_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold_deg: float = 15.0,
    required: int = 30,
) -> torch.Tensor:
    """연속 upright 프레임 수에 비례하는 dense 보상 [0, 1]."""
    sf = _get_stable_frames(env)
    return (sf.float() / max(required, 1)).clamp(0.0, 1.0)


def near_success_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    sigma_deg: float = 30.0,
) -> torch.Tensor:
    """Gaussian: upright 근접 시 급격히 커지는 gradient 제공 보상."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    sigma = math.radians(sigma_deg)
    return torch.exp(-0.5 * (roll ** 2 + pitch ** 2) / (sigma ** 2))


def wheel_contact_reward(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_sensor"),
) -> torch.Tensor:
    """바퀴-지형 접촉 비율 [0,1] 보상. shape (N,).

    ContactSensor에 등록된 body 중 지형과 접촉 중인 비율을 반환.
    """
    sensor: ContactSensor = env.scene[sensor_cfg.name]
    force_mag = torch.nan_to_num(
        torch.norm(sensor.data.net_forces_w, dim=-1), nan=0.0
    )  # (N, num_bodies)
    return (force_mag > 1.0).float().mean(dim=-1)  # (N,) in [0,1]


def forward_ready_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    upright_threshold_deg: float = 20.0,
) -> torch.Tensor:
    """기립 성공 후 저속·수평 상태 = 전진 주행 준비 완료 보상."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(upright_threshold_deg)
    is_upright = (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)

    vehicle: Articulation = env.scene[asset_cfg.name]
    lin_speed = torch.norm(vehicle.data.root_lin_vel_w, dim=-1)
    low_speed = lin_speed < 0.5

    upright_z = _local_z_in_world(env, asset_cfg)[:, 2]
    good_z = upright_z > 0.9

    return (is_upright & low_speed & good_z).float()


def wheel_drive_reward(
    env: ManagerBasedRLEnv,
    vehicle_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    tilt_threshold_deg: float = 45.0,
) -> torch.Tensor:
    """넘어진 상태에서 drive 바퀴를 적극적으로 돌릴수록 보상."""
    roll, pitch, _ = _rover_euler(env, vehicle_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    fallen = (tilt > math.radians(tilt_threshold_deg)).float()

    vehicle: Articulation = env.scene[vehicle_cfg.name]
    wheel_vel = vehicle.data.joint_vel[:, vehicle_cfg.joint_ids]
    wheel_rms = torch.sqrt(torch.mean(wheel_vel ** 2, dim=-1) + 1e-6)
    return fallen * torch.clamp(wheel_rms / 10.0, 0.0, 1.0)


def recovery_angular_vel_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """기울기 감소 방향 각속도를 보상 — 적극적 자세 복원 유도."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    vehicle: Articulation = env.scene[asset_cfg.name]
    ang_vel = vehicle.data.root_ang_vel_w
    roll_recovery  = -roll  * ang_vel[:, 0]
    pitch_recovery = -pitch * ang_vel[:, 1]
    return torch.clamp(
        (roll_recovery + pitch_recovery) / (math.pi * 2.0), -1.0, 1.0
    )


def arm_ground_push_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """넘어진 방향으로 팔을 뻗어 땅을 짚고 미는 동작 보상.

    asset_cfg에 body_names=["tool0"] 이 지정되어야 한다.
    """
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.sqrt(roll ** 2 + pitch ** 2)
    fallen_mask = (tilt > math.radians(45.0)).float()

    vehicle: Articulation = env.scene[asset_cfg.name]

    tip_z   = vehicle.data.body_pos_w[:, asset_cfg.body_ids[0], 2]
    rover_z = vehicle.data.root_pos_w[:, 2]

    reach = torch.clamp((rover_z - tip_z) / 0.5, 0.0, 1.0)

    tip_vel_z = vehicle.data.body_lin_vel_w[:, asset_cfg.body_ids[0], 2]
    push = torch.clamp(-tip_vel_z / 1.0, 0.0, 1.0)

    return fallen_mask * (0.7 * reach + 0.3 * push)


def arm_vel_penalty_conditional(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """기립 후에만 arm 속도 패널티 적용 — 넘어진 상태에서는 팔 자유롭게."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.sqrt(roll ** 2 + pitch ** 2)
    upright_mask = (tilt < math.radians(45.0)).float()

    vehicle: Articulation = env.scene[asset_cfg.name]
    vel_sq_sum = torch.sum(
        vehicle.data.joint_vel[:, asset_cfg.joint_ids] ** 2, dim=-1
    )
    return upright_mask * vel_sq_sum


# ── Sparse reward 함수 ────────────────────────────────────────────────────────

def stable_success_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold_deg: float = 15.0,
    required: int = 30,
) -> torch.Tensor:
    """stable_frames >= required 달성 시 1.0 (sparse 보너스)."""
    sf = _get_stable_frames(env)
    return (sf >= required).float()


def success_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold_deg: float = 15.0,
) -> torch.Tensor:
    """간단한 순간 성공 체크 (stability 요구 없음)."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    return ((torch.abs(roll) < thr) & (torch.abs(pitch) < thr)).float()


# ── Penalty 함수 ─────────────────────────────────────────────────────────────

def fallen_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    tilt_threshold_deg: float = 75.0,
) -> torch.Tensor:
    """tilt > threshold 상태 지속 시 1.0 패널티 (정체 방지)."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    tilt = torch.abs(roll) + torch.abs(pitch)
    return (tilt > math.radians(tilt_threshold_deg)).float()


def time_alive_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """매 스텝 1.0 (빠른 기립 압박)."""
    return torch.ones(env.num_envs, device=env.device)


def joint_vel_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """arm 관절 속도 제곱합 패널티 (과격한 arm 동작 억제)."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    return torch.sum(vehicle.data.joint_vel[:, asset_cfg.joint_ids] ** 2, dim=-1)


def joint_limit_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    margin: float = 0.1,
) -> torch.Tensor:
    """arm 관절이 소프트 한계 margin 내에 접근하면 패널티."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]
    lo  = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    hi  = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]
    exceed = (
        torch.clamp(lo + margin - pos, min=0.0)
        + torch.clamp(pos - (hi - margin), min=0.0)
    )
    return exceed.sum(dim=-1)


def suspension_misalignment_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """rocker-bogie 관절이 중립(0°)에서 벗어난 총 편차 패널티."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(pos ** 2, dim=-1)


def steer_misalignment_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> torch.Tensor:
    """기립 후 steer가 중립(0)에서 벗어나는 패널티 (기립 중에는 자유)."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos = vehicle.data.joint_pos[:, asset_cfg.joint_ids]
    roll, pitch, _ = euler_xyz_from_quat(vehicle.data.root_quat_w)
    tilt = torch.abs(roll) + torch.abs(pitch)
    upright_mask = (tilt < math.radians(30.0)).float()
    return upright_mask * torch.sum(pos ** 2, dim=-1)


def angular_velocity_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold: float = 5.0,
) -> torch.Tensor:
    """angular speed > threshold 초과분 패널티 (폭발적 회전 억제)."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    ang_speed = torch.norm(vehicle.data.root_ang_vel_w, dim=-1)
    return torch.clamp(ang_speed - threshold, min=0.0)


# ── Termination 함수 ──────────────────────────────────────────────────────────

def stable_upright_termination(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    sensor_cfg: SceneEntityCfg | None = None,
    threshold_deg: float = 15.0,
    required: int = 30,
) -> torch.Tensor:
    """성공 종료: stable_frames >= required + 바퀴 지면 접촉.

    sensor_cfg가 주어지면 ContactSensor에서 접촉 여부도 함께 확인한다.
    """
    sf = _get_stable_frames(env)
    stable = sf >= required

    if sensor_cfg is None:
        return stable

    sensor: ContactSensor = env.scene[sensor_cfg.name]
    force_mag = torch.nan_to_num(
        torch.norm(sensor.data.net_forces_w, dim=-1), nan=0.0
    )  # (N, num_bodies)
    any_contact = (force_mag > 1.0).any(dim=-1)  # (N,)

    return stable & any_contact


def rover_upright_termination(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    threshold_deg: float = 15.0,
) -> torch.Tensor:
    """순간 upright 체크 (stability 요구 없음) — 간단한 성공 종료."""
    roll, pitch, _ = _rover_euler(env, asset_cfg)
    thr = math.radians(threshold_deg)
    return (torch.abs(roll) < thr) & (torch.abs(pitch) < thr)


def simulation_explosion_termination(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    max_lin_vel: float = 20.0,
    max_ang_vel: float = 30.0,
) -> torch.Tensor:
    """속도가 물리적으로 불가능한 수준 → 시뮬 불안정 종료."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    lin_speed = torch.norm(vehicle.data.root_lin_vel_w, dim=-1)
    ang_speed = torch.norm(vehicle.data.root_ang_vel_w, dim=-1)
    nan_detected = torch.isnan(lin_speed) | torch.isnan(ang_speed)
    return (lin_speed > max_lin_vel) | (ang_speed > max_ang_vel) | nan_detected


def vehicle_lost_termination(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    min_z: float = -5.0,
    max_z: float = 10.0,
) -> torch.Tensor:
    """vehicle이 지면 아래로 가라앉거나 공중으로 날아간 경우 종료."""
    vehicle: Articulation = env.scene[asset_cfg.name]
    pos_z = vehicle.data.root_pos_w[:, 2]
    return (pos_z < min_z) | (pos_z > max_z)


def action_rate_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """연속 action 변화량 L2 패널티 — 급격한 동작 억제."""
    curr = env.action_manager.action
    prev = env.action_manager._prev_action
    diff = curr - prev
    return torch.sum(diff ** 2, dim=-1)


def update_prev_action(env: ManagerBasedRLEnv) -> torch.Tensor:
    """이전 action 저장 (observation term으로 매 스텝 호출)."""
    env._prev_action = env.action_manager.action.clone()
    return env._prev_action


# ── Event 함수 (reset / randomization) ────────────────────────────────────────

def reset_vehicle_random_fall(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    height: float = 0.5,
) -> None:
    """Vehicle을 3가지 전복 패턴으로 랜덤 초기화.

    모드 비율:
      40% — 옆으로 넘어짐 (side):      |roll| ∈ [60°, 120°], |pitch| < 20°
      30% — 뒤집힘    (upside-down):   |roll| ∈ [140°, 180°], |pitch| < 20°
      30% — 비스듬히  (diagonal):      |roll| ∈ [30°, 70°],   |pitch| ∈ [20°, 50°]

    stable_frames 카운터와 모든 관절을 default로 리셋.
    (팔 자세 랜덤화는 randomize_arm_pose 이벤트가 이후에 담당)
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    if hasattr(env, "_recovery_stable_frames"):
        env._recovery_stable_frames[env_ids] = 0

    rand  = torch.rand(n, device=device)
    sign_r = torch.randint(0, 2, (n,), device=device).float() * 2 - 1  # ±1
    sign_p = torch.randint(0, 2, (n,), device=device).float() * 2 - 1

    # ── 모드별 각도 샘플 ─────────────────────────────────────────────────────
    # side
    r0 = torch.empty(n, device=device).uniform_(math.radians(60),  math.radians(120)) * sign_r
    p0 = torch.empty(n, device=device).uniform_(0.0,               math.radians(20))  * sign_p

    # upside-down
    r1 = torch.empty(n, device=device).uniform_(math.radians(140), math.pi)           * sign_r
    p1 = torch.empty(n, device=device).uniform_(0.0,               math.radians(20))  * sign_p

    # diagonal
    r2 = torch.empty(n, device=device).uniform_(math.radians(30),  math.radians(70))  * sign_r
    p2 = torch.empty(n, device=device).uniform_(math.radians(20),  math.radians(50))  * sign_p

    m_side = rand < 0.40
    m_up   = (rand >= 0.40) & (rand < 0.70)

    roll  = torch.where(m_side, r0, torch.where(m_up, r1, r2))
    pitch = torch.where(m_side, p0, torch.where(m_up, p1, p2))
    yaw   = torch.empty(n, device=device).uniform_(-math.pi, math.pi)

    quat = quat_from_euler_xyz(roll, pitch, yaw)

    pos = torch.zeros(n, 3, device=device)
    pos[:, 0] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 1] = torch.empty(n, device=device).uniform_(-0.5, 0.5)
    pos[:, 2] = height

    vehicle.write_root_pose_to_sim(
        torch.cat([pos, quat], dim=-1), env_ids=env_ids
    )
    vehicle.write_root_velocity_to_sim(
        torch.zeros(n, 6, device=device), env_ids=env_ids
    )
    default_pos = vehicle.data.default_joint_pos[env_ids]
    vehicle.write_joint_state_to_sim(
        default_pos, torch.zeros_like(default_pos), env_ids=env_ids
    )


def randomize_arm_pose(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    pos_range: tuple = (-0.3, 0.3),
) -> None:
    """arm 관절만 default + uniform noise 로 초기화.

    asset_cfg에 joint_names=["joint_[1-6]"] 이 지정되어야 한다.
    soft limit 내로 클램핑해 물리 폭발 방지.
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device
    num_arm = len(asset_cfg.joint_ids)

    default_arm = vehicle.data.default_joint_pos[:, asset_cfg.joint_ids][env_ids]  # (n, 6)
    noise = torch.empty(n, num_arm, device=device).uniform_(*pos_range)
    new_pos = default_arm + noise

    # soft limit 클램핑
    lo = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0][env_ids]
    hi = vehicle.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1][env_ids]
    new_pos = torch.clamp(new_pos, lo, hi)

    vehicle.write_joint_state_to_sim(
        new_pos,
        torch.zeros(n, num_arm, device=device),
        joint_ids=asset_cfg.joint_ids,
        env_ids=env_ids,
    )


def randomize_physics_params(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
) -> None:
    """Sim-to-real gap 대비용 작은 속도 노이즈 주입.

    reset_vehicle_random_fall / randomize_arm_pose 이후에 실행됨.
    """
    vehicle: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    noise_lin = torch.randn(n, 3, device=device) * 0.10
    noise_ang = torch.randn(n, 3, device=device) * 0.05
    vehicle.write_root_velocity_to_sim(
        torch.cat([noise_lin, noise_ang], dim=-1), env_ids=env_ids
    )


# ── 하위 호환: 이전 이벤트 함수 (더 이상 env_cfg에서 참조하지 않음) ────────────

def reset_vehicle_fallen(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    roll_range: tuple = (1.047, 2.094),
    pitch_range: tuple = (-0.524, 0.524),
    height: float = 0.5,
) -> None:
    """[deprecated] reset_vehicle_random_fall 로 대체됨."""
    reset_vehicle_random_fall(env, env_ids, asset_cfg, height)


def reset_crater_fall(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("vehicle"),
    crater_prob: float = 0.3,
    roll_range: tuple = (1.396, 1.745),
    pitch_range: tuple = (-0.349, 0.349),
    crater_depth: float = 0.4,
) -> None:
    """[deprecated] reset_vehicle_random_fall 로 대체됨."""
    pass
