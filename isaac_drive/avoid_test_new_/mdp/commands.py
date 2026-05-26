# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""무작위 goal 좌표(2D) 를 매 에피소드 resample 하는 CommandTerm.

  · 각 env 마다 env_origin 기준 local xy 를 샘플링하고, world 좌표로 변환해 저장.
  · obstacle_grid + 베이스캠프 반경으로 reject → 갈 수 없는 goal 안 만듦.
  · spawn 위치(robot.data.root_pos_w) 와의 최소 거리·최대 거리 제약으로
    너무 가깝거나·너무 먼 goal 을 거른다.

ResidualAckermannAction 과 observations 가 같은 이름("goal_pose")으로 이
command 를 꺼내 쓴다.  반환 텐서는 (N, 2) world (gx, gy).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils import configclass

from . import sampling

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# 빨간 구 (반경 0.3m) — goal 위치 시각화용 마커.
_GOAL_SPHERE_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/Command/goal_position",
    markers={
        "goal": sim_utils.SphereCfg(
            radius=0.3,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.1)),
        ),
    },
)


class RandomGoalCommand(CommandTerm):
    """매 에피소드 시작 시 env_origin 주변의 valid 한 (gx, gy) 를 샘플링.

    반환 텐서: (N, 2) world frame.  base controller·observations 가 직접
    body frame 으로 변환해 쓴다.
    """

    cfg: "RandomGoalCommandCfg"

    def __init__(self, cfg: "RandomGoalCommandCfg", env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.goal_pos_w = torch.zeros(self.num_envs, 2, device=self.device)
        # 메트릭.
        self.metrics["dist_to_goal"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        return (
            f"RandomGoalCommand(num_envs={self.num_envs}, "
            f"sample_radius={self.cfg.sample_radius}, basecamp_r={self.cfg.basecamp_radius})"
        )

    # ---- properties ------------------------------------------------------
    @property
    def command(self) -> torch.Tensor:
        """world (gx, gy) — shape (N, 2)."""
        return self.goal_pos_w

    # ---- resample / update ----------------------------------------------
    def _resample_command(self, env_ids: Sequence[int]):
        n = len(env_ids)
        if n == 0:
            return
        env_ids_t = (env_ids if isinstance(env_ids, torch.Tensor)
                     else torch.tensor(list(env_ids), device=self.device))

        # goal 샘플링 중심 결정 — force_center_world 가 cfg 에 있으면 그걸,
        # 아니면 env_origin 사용.
        if self.cfg.force_center_world is not None:
            center_xy = torch.tensor(self.cfg.force_center_world,
                                     device=self.device, dtype=torch.float32).expand(n, 2)
        else:
            center_xy = self._env.scene.env_origins[env_ids_t, :2]
        robot_xy = self.robot.data.root_pos_w[env_ids_t, :2]      # (n, 2) world

        self.goal_pos_w[env_ids_t] = sampling.sample_valid_xy(
            n=n,
            device=self.device,
            center_xy=center_xy,
            sample_radius=self.cfg.sample_radius,
            basecamp_radius=self.cfg.basecamp_radius,
            ref_xy=robot_xy,
            min_dist_to_ref=self.cfg.min_goal_dist,
            max_dist_to_ref=self.cfg.max_goal_dist,
            fallback_world=self.cfg.fallback_world,
        )

    def _update_command(self):
        # world goal 은 한 에피소드 내 고정 — update 할 게 없음.
        pass

    def _update_metrics(self):
        d = (self.goal_pos_w - self.robot.data.root_pos_w[:, :2]).norm(dim=-1)
        self.metrics["dist_to_goal"][:] = d

    # ---- helpers used by rewards.py -------------------------------------
    def current_dist(self) -> torch.Tensor:
        """현재 goal 까지 거리 (N,)."""
        return (self.goal_pos_w - self.robot.data.root_pos_w[:, :2]).norm(dim=-1)

    # ---- debug visualization (FlatPatchesGoalCommand 와 동일 패턴) -------
    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "_goal_marker"):
                self._goal_marker = VisualizationMarkers(self.cfg.goal_marker_cfg)
            self._goal_marker.set_visibility(True)
        elif hasattr(self, "_goal_marker"):
            self._goal_marker.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not hasattr(self, "_goal_marker"):
            return
        pos = torch.zeros(self.num_envs, 3, device=self.device)
        pos[:, :2] = self.goal_pos_w
        pos[:, 2] = 0.5
        self._goal_marker.visualize(translations=pos)


class FlatPatchesGoalCommand(CommandTerm):
    """terrain.flat_patches 에 사전 계산된 valid 좌표들에서 goal 을 샘플.

    terrain_generator 에서 FlatPatchSamplingCfg 로 만들어둔 '안전한 좌표'를
    바로 쓰기 때문에 박스 충돌 reject 가 필요 없다.  결과: (N, 2) world.
    """

    cfg: "FlatPatchesGoalCommandCfg"

    def __init__(self, cfg: "FlatPatchesGoalCommandCfg",
                 env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.terrain = env.scene["terrain"]
        if cfg.patch_name not in self.terrain.flat_patches:
            raise RuntimeError(
                f"flat_patches['{cfg.patch_name}']' not found.  "
                f"Available: {list(self.terrain.flat_patches.keys())}"
            )
        # shape: (num_levels, num_types, num_patches, 3) — world.
        self.valid_targets: torch.Tensor = self.terrain.flat_patches[cfg.patch_name]
        self.goal_pos_w = torch.zeros(self.num_envs, 2, device=self.device)
        self.metrics["dist_to_goal"] = torch.zeros(self.num_envs, device=self.device)

    def __str__(self) -> str:
        return (
            f"FlatPatchesGoalCommand(patch='{self.cfg.patch_name}', "
            f"min_goal_dist={self.cfg.min_goal_dist})"
        )

    @property
    def command(self) -> torch.Tensor:
        return self.goal_pos_w

    def _resample_command(self, env_ids: Sequence[int]):
        n = len(env_ids)
        if n == 0:
            return
        env_ids_t = (env_ids if isinstance(env_ids, torch.Tensor)
                     else torch.as_tensor(list(env_ids),
                                          device=self.device, dtype=torch.long))

        levels = self.terrain.terrain_levels[env_ids_t]
        types = self.terrain.terrain_types[env_ids_t]
        n_patches = self.valid_targets.shape[2]
        robot_xy = self.robot.data.root_pos_w[env_ids_t, :2]

        # 미수락 슬롯 — min_dist 통과할 때까지 재추첨.
        accepted = torch.zeros(n, dtype=torch.bool, device=self.device)
        chosen = torch.zeros(n, 2, device=self.device)
        for _ in range(8):
            need = ~accepted
            n_need = int(need.sum().item())
            if n_need == 0:
                break
            ids = torch.randint(0, n_patches, (n_need,), device=self.device)
            tgt = self.valid_targets[levels[need], types[need], ids, :2]   # (n_need, 2)
            d = (tgt - robot_xy[need]).norm(dim=-1)
            ok = (d >= self.cfg.min_goal_dist) & (d <= self.cfg.max_goal_dist)
            sel_idx = torch.where(need)[0]
            ok_idx = sel_idx[ok]
            chosen[ok_idx] = tgt[ok]
            accepted[ok_idx] = True
        # fallback — 못 채운 슬롯엔 첫 패치 그대로.
        if not accepted.all():
            need = ~accepted
            n_need = int(need.sum().item())
            tgt = self.valid_targets[levels[need], types[need], 0, :2]
            chosen[need] = tgt
        self.goal_pos_w[env_ids_t] = chosen

    def _update_command(self):
        pass

    def _update_metrics(self):
        d = (self.goal_pos_w - self.robot.data.root_pos_w[:, :2]).norm(dim=-1)
        self.metrics["dist_to_goal"][:] = d

    def current_dist(self) -> torch.Tensor:
        return (self.goal_pos_w - self.robot.data.root_pos_w[:, :2]).norm(dim=-1)

    # ---- debug visualization -------------------------------------------
    def _set_debug_vis_impl(self, debug_vis: bool):
        """red sphere 마커 on/off."""
        if debug_vis:
            if not hasattr(self, "_goal_marker"):
                self._goal_marker = VisualizationMarkers(self.cfg.goal_marker_cfg)
            self._goal_marker.set_visibility(True)
        elif hasattr(self, "_goal_marker"):
            self._goal_marker.set_visibility(False)

    def _debug_vis_callback(self, event):
        """매 프레임 마커 위치를 현재 goal 로 이동."""
        if not hasattr(self, "_goal_marker"):
            return
        # 마커는 3D 위치 — z 는 환경 ground 위로 약간 띄움.
        pos = torch.zeros(self.num_envs, 3, device=self.device)
        pos[:, :2] = self.goal_pos_w
        pos[:, 2] = 0.5
        self._goal_marker.visualize(translations=pos)


@configclass
class FlatPatchesGoalCommandCfg(CommandTermCfg):
    """FlatPatchesGoalCommand 설정 — terrain.flat_patches 에서 goal 샘플."""

    class_type: type[CommandTerm] = FlatPatchesGoalCommand

    asset_name: str = MISSING
    """robot articulation 이름 (scene.robot)."""

    patch_name: str = "goal"
    """terrain_generator FlatPatchSamplingCfg 의 키 — 같은 이름을 sub_terrains
    의 flat_patch_sampling dict 에 정의해야 한다."""

    min_goal_dist: float = 3.0
    """spawn 으로부터 최소 거리 (m)."""

    max_goal_dist: float = 10.0
    """spawn 으로부터 최대 거리 (m).  flat env 는 한 패치 12m 작으니 작게."""

    goal_marker_cfg: VisualizationMarkersCfg = _GOAL_SPHERE_CFG
    """debug_vis=True 일 때 띄울 goal 마커.  기본 빨간 구 반경 0.3m."""


@configclass
class RandomGoalCommandCfg(CommandTermCfg):
    """RandomGoalCommand 설정 — env_origin 주변에서 obstacle_grid 기반 goal 추첨."""

    class_type: type[CommandTerm] = RandomGoalCommand

    asset_name: str = MISSING
    """robot articulation 이름 (scene.robot)."""

    sample_radius: float = 8.0
    """env_origin 기준 goal 샘플링 박스 반경 (m).  terrain_00022 + 36env 면 8 적당."""

    basecamp_radius: float = 6.5
    """베이스캠프(world 원점) 회피 반경 (m)."""

    min_goal_dist: float = 3.0
    """spawn 으로부터 최소 goal 거리 (m)."""

    max_goal_dist: float = 12.0
    """spawn 으로부터 최대 goal 거리 (m)."""

    fallback_world: tuple[float, float] = (10.0, 0.0)
    """rejection 다 실패 시 채울 world 좌표 (m, m)."""

    force_center_world: tuple[float, float] | None = None
    """goal 샘플링 중심을 강제 (world 좌표).  None 이면 env_origin 사용.
    test 시 (0, 0) 으로 두면 모든 env 의 goal 이 베이스캠프 주변에서 추첨됨."""

    goal_marker_cfg: VisualizationMarkersCfg = _GOAL_SPHERE_CFG
    """debug_vis=True 일 때 띄울 goal 마커."""
