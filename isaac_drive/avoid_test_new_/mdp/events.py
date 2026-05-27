# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""에피소드 리셋 이벤트 — 무작위 spawn pose 를 obstacle_grid 로 reject 한다."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from . import sampling

# reset 시 m0609 팔을 HOME 으로 강제 — drive_test 와 동일 정책.
from rover_vehicle import keep_arm_folded  # noqa: E402

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_root_safe(
    env: "ManagerBasedRLEnv",
    env_ids: torch.Tensor,
    sample_radius: float = 5.0,
    basecamp_radius: float = 6.5,
    clearance_radius: float = 1.0,
    z_clearance: float = 0.4,
    force_center_world: tuple[float, float] | None = None,
    yaw_range: tuple[float, float] = (0.0, 2.0 * math.pi),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> None:
    """spawn 위치를 obstacle_grid·heightmap 기반으로 안전하게 잡는다.

    Args:
        sample_radius: spawn 샘플링 박스 반경 (m).
        basecamp_radius: 베이스캠프(world 원점) 회피 반경.  0 이면 회피 안 함.
        clearance_radius: 차량 footprint 반경 — 이 안에 obstacle 있으면 reject.
        z_clearance: 지면 z 위에 띄울 마진 (m).
        force_center_world: spawn 샘플링 중심을 강제 (world 좌표).  None 이면
            env_origin 사용 (multi-env 정상 모드).  test 시 (0, 0) 같은 값으로
            모든 env 를 같은 위치(베이스캠프)에서 spawn 가능.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    n = len(env_ids)
    if n == 0:
        return
    device = env.device
    env_ids_t = (env_ids if isinstance(env_ids, torch.Tensor)
                 else torch.as_tensor(list(env_ids), device=device, dtype=torch.long))

    # spawn 샘플링 중심 결정.
    if force_center_world is not None:
        center_xy = torch.tensor(force_center_world, device=device,
                                 dtype=torch.float32).expand(n, 2)
    else:
        center_xy = env.scene.env_origins[env_ids_t, :2]

    world_xy = sampling.sample_valid_xy(
        n=n,
        device=device,
        center_xy=center_xy,
        sample_radius=sample_radius,
        basecamp_radius=basecamp_radius,
        clearance_radius=clearance_radius,
        fallback_world=force_center_world if force_center_world is not None else (0.0, 0.0),
    )

    root_state = asset.data.default_root_state[env_ids_t].clone()    # (n, 13)
    root_state[:, 0:2] = world_xy
    # heightmap 으로 그 xy 의 정확한 지면 z 조회 후 z_clearance 만큼만 띄움.
    ground_z = sampling.terrain_height_at(world_xy)
    root_state[:, 2] = ground_z + z_clearance

    yaw = torch.empty(n, device=device).uniform_(*yaw_range)
    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    quat = torch.zeros(n, 4, device=device)
    quat[:, 0] = cy
    quat[:, 3] = sy
    root_state[:, 3:7] = quat
    root_state[:, 7:13] = 0.0

    asset.write_root_pose_to_sim(root_state[:, :7], env_ids=env_ids_t)
    asset.write_root_velocity_to_sim(root_state[:, 7:], env_ids=env_ids_t)
    # m0609 팔도 HOME 으로 초기화 — reset 직후부터 접힌 상태 유지.
    keep_arm_folded(asset)


def reset_root_flat_patches(
    env: "ManagerBasedRLEnv",
    env_ids: torch.Tensor,
    patch_name: str = "spawn",
    yaw_range: tuple[float, float] = (0.0, 2.0 * math.pi),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> None:
    """terrain.flat_patches[patch_name] 에서 spawn 좌표를 추첨.

    terrain_generator 가 미리 박스 충돌 없는 valid 좌표를 만들어 둔 것을
    그대로 가져다 쓰므로 추가 reject 가 필요 없다.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    terrain = env.scene["terrain"]
    if patch_name not in terrain.flat_patches:
        raise RuntimeError(
            f"flat_patches['{patch_name}']' not found.  "
            f"Available: {list(terrain.flat_patches.keys())}"
        )
    valid = terrain.flat_patches[patch_name]   # (levels, types, patches, 3)
    n_patches = valid.shape[2]

    n = len(env_ids)
    if n == 0:
        return
    device = env.device
    env_ids_t = (env_ids if isinstance(env_ids, torch.Tensor)
                 else torch.as_tensor(list(env_ids), device=device, dtype=torch.long))

    levels = terrain.terrain_levels[env_ids_t]
    types = terrain.terrain_types[env_ids_t]
    ids = torch.randint(0, n_patches, (n,), device=device)
    spawn_xyz = valid[levels, types, ids, :3]   # (n, 3) world

    root_state = asset.data.default_root_state[env_ids_t].clone()
    root_state[:, 0:2] = spawn_xyz[:, :2]
    # z 는 default_root_state 의 spawn 높이(중력 안착 마진 포함) 사용.

    yaw = torch.empty(n, device=device).uniform_(*yaw_range)
    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    quat = torch.zeros(n, 4, device=device)
    quat[:, 0] = cy
    quat[:, 3] = sy
    root_state[:, 3:7] = quat
    root_state[:, 7:13] = 0.0

    asset.write_root_pose_to_sim(root_state[:, :7], env_ids=env_ids_t)
    asset.write_root_velocity_to_sim(root_state[:, 7:], env_ids=env_ids_t)
    # m0609 팔도 HOME 으로 초기화 — flat 단계도 동일 정책.
    keep_arm_folded(asset)
