# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""룰베이스 base controller + RL residual 을 합성한 Ackermann 액션.

학습 전략 — residual policy learning:
  · 매 스텝, 환경의 (goal, raycaster) 상태로 base_lin/base_ang 를 룰베이스로
    계산해 깔아둔다.
       - Goal-seek      : 차량→goal 의 body-frame yaw_err 로 P 제어
                          (lin = cruise·max(0,cos(yaw_err)), ang = K·yaw_err).
       - 장애물 방향     : 레이캐스트 격자에서 '국소 돌출(prominence)'으로
                          전방 코리도 안 장애물을 골라내, 가중 평균 lat 의
                          부호로 회피 방향만 깔아둔다 (왼쪽 장애물→우회전,
                          오른쪽 장애물→좌회전, 가까울수록·정중앙일수록 강).
  · 정책은 (lin_residual, ang_residual) 만 출력 — base 에 더해진 뒤 안전한
    범위로 클램프되고 Ackermann 운동학으로 매핑된다.
  · 마지막 layer 가중치를 0 으로 초기화하면 학습 시작 시점에 정확히 base
    controller 거동(이미 룰베이스 수준).  RL 은 그 위에 보정만 학습.

obs/reward/termination 에서 동일한 goal command 와 raycaster 를 공유한다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils import configclass

from . import ackermann_actions
from .actions_cfg import AckermannActionCfg

# m0609 팔을 HOME(접힘) 으로 매 step 고정 — drive_test 와 동일 정책.
# 외부 모듈에서 가져온다 (rover_vehicle.py 가 avoid_test_new_/ 안에 있음).
from rover_vehicle import keep_arm_folded  # noqa: E402

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Action term
# ---------------------------------------------------------------------------
class ResidualAckermannAction(ackermann_actions.AckermannAction):
    """Base controller(룰베이스) + RL residual → Ackermann 운동학.

    RL 정책의 출력 (lin_residual, ang_residual) 은 cfg.lin_residual_scale /
    cfg.ang_residual_scale 로 물리 단위로 변환된 뒤 base 값에 더해진다.
    학습 시작 시 정책의 출력이 ~0 이면 차량은 룰베이스 그대로 동작한다.
    """

    cfg: "ResidualAckermannActionCfg"

    def __init__(self, cfg: "ResidualAckermannActionCfg", env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        # 레이캐스트 격자 행·열 (lazy 초기화 — 첫 호출 시 cfg.pattern 에서 계산).
        self._rows: int | None = None
        self._cols: int | None = None

    @property
    def action_dim(self) -> int:
        return 2  # (lin_residual, ang_residual)

    # ----- helpers ------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(q: torch.Tensor) -> torch.Tensor:
        """(..., 4) wxyz quaternion → (...) yaw (rad).  Isaac Lab 규약."""
        w = q[..., 0]
        x = q[..., 1]
        y = q[..., 2]
        z = q[..., 3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return torch.atan2(siny_cosp, cosy_cosp)

    def _ensure_grid(self) -> None:
        """레이캐스트 GridPattern 의 (rows, cols) 를 cfg 에서 한 번만 계산."""
        if self._rows is not None:
            return
        scanner = self._env.scene[self.cfg.raycaster_name]
        n_rays = scanner.data.ray_hits_w.shape[1]
        pc = scanner.cfg.pattern_cfg
        res = float(pc.resolution)
        nx = len(torch.arange(-pc.size[0] / 2.0, pc.size[0] / 2.0 + 1.0e-9, res))
        ny = len(torch.arange(-pc.size[1] / 2.0, pc.size[1] / 2.0 + 1.0e-9, res))
        rows, cols = (ny, nx) if getattr(pc, "ordering", "xy") == "xy" else (nx, ny)
        if rows * cols != n_rays:
            raise RuntimeError(
                f"raycaster grid shape mismatch: {rows}×{cols} != n_rays={n_rays}"
            )
        self._rows = rows
        self._cols = cols

    # ----- base controller ----------------------------------------------
    def _compute_base(self) -> tuple[torch.Tensor, torch.Tensor]:
        """벡터화 base controller — 전 env 동시.

        Returns:
            base_lin, base_ang : (num_envs,) tensor 각각.
        """
        cfg = self.cfg
        N = self.num_envs

        # ----- goal-seek ------------------------------------------------
        goal = self._env.command_manager.get_command(cfg.goal_command_name)  # (N, ≥2)
        pos = self._asset.data.root_pos_w  # (N, 3)
        quat = self._asset.data.root_quat_w  # (N, 4) wxyz
        yaw = self._quat_to_yaw(quat)  # (N,)
        cy = torch.cos(yaw)
        sy = torch.sin(yaw)

        dx_g = goal[:, 0] - pos[:, 0]
        dy_g = goal[:, 1] - pos[:, 1]
        # body-frame yaw_err — atan2(좌측, 전방).
        yaw_err = torch.atan2(-dx_g * sy + dy_g * cy, dx_g * cy + dy_g * sy)

        base_lin = cfg.cruise_speed * torch.clamp(torch.cos(yaw_err), min=0.0)
        base_ang = torch.clamp(
            cfg.goal_turn_k * yaw_err, min=-cfg.max_ang, max=cfg.max_ang
        )

        # ----- 장애물 방향 (raycaster prominence) ------------------------
        self._ensure_grid()
        rows, cols = self._rows, self._cols
        r = cfg.prom_radius
        scanner = self._env.scene[cfg.raycaster_name]
        hits = scanner.data.ray_hits_w  # (N, rows*cols, 3)
        sensor_pos = scanner.data.pos_w  # (N, 3)

        z = hits[..., 2].view(N, rows, cols)  # (N, rows, cols)

        # 8-이웃(반경 r) 평균 대비 국소 돌출 — 평면·언덕 흡수.
        core = z[:, r : rows - r, r : cols - r]
        acc = torch.zeros_like(core)
        cnt = 0
        for dr in (-r, 0, r):
            for dc in (-r, 0, r):
                if dr == 0 and dc == 0:
                    continue
                acc = acc + z[:, r + dr : rows - r + dr, r + dc : cols - r + dc]
                cnt += 1
        prom = core - acc / cnt  # (N, rows-2r, cols-2r)

        # core 셀의 body-frame fwd/lat — 월드 hit (x, y) 에서 환산.
        xy = hits[..., :2].view(N, rows, cols, 2)
        core_xy = xy[:, r : rows - r, r : cols - r, :]
        dx_c = core_xy[..., 0] - sensor_pos[:, 0].view(N, 1, 1)
        dy_c = core_xy[..., 1] - sensor_pos[:, 1].view(N, 1, 1)
        cy2 = cy.view(N, 1, 1)
        sy2 = sy.view(N, 1, 1)
        fwd_c = dx_c * cy2 + dy_c * sy2
        lat_c = -dx_c * sy2 + dy_c * cy2

        # 장애물 셀 = 유한 + 임계 돌출. 진로 막힘 = 전방 + corridor 안.
        is_obs = torch.isfinite(prom) & (prom > cfg.height_thresh)
        is_blocking = (
            is_obs
            & (fwd_c > 0.0)
            & (fwd_c < cfg.front_range)
            & (lat_c.abs() < cfg.corridor)
        )

        has_block = is_blocking.view(N, -1).any(dim=-1)  # (N,)

        # 가중(prominence) 평균 lat — 부호로 회피 방향, 크기로 강도 강화.
        weight = (is_blocking.float() * prom.clamp(min=0.0))
        w_sum = weight.view(N, -1).sum(dim=-1).clamp(min=1.0e-6)
        lat_mean = (weight * lat_c).view(N, -1).sum(dim=-1) / w_sum  # (N,)

        # avoid_dir = -sign(lat_mean) : 왼쪽 장애물(+lat) → -1(우회전),
        #                                오른쪽 장애물(-lat) → +1(좌회전).
        avoid_dir = torch.where(
            lat_mean >= 0.0,
            -torch.ones_like(lat_mean),
            torch.ones_like(lat_mean),
        )

        # 위치 기반 강화 — 정중앙(|lat|≈0)일수록·가까울수록 강한 base 회피각.
        # 두 가중치(0~1) 의 큰 쪽을 쓴다.
        # |lat|=0 → 1, |lat|>=corridor → 0.
        center_w = torch.clamp(1.0 - lat_mean.abs() / cfg.corridor, min=0.0, max=1.0)
        # fwd 평균 — 가까울수록 강화. (fwd_mean=0 → 1, fwd_mean>=front_range → 0)
        fwd_sum = (weight * fwd_c).view(N, -1).sum(dim=-1) / w_sum
        near_w = torch.clamp(1.0 - fwd_sum / cfg.front_range, min=0.0, max=1.0)
        intensity = torch.maximum(center_w, near_w)
        avoid_ang_mag = cfg.avoid_base_ang + intensity * (
            cfg.avoid_max_ang - cfg.avoid_base_ang
        )

        base_ang = torch.where(has_block, avoid_dir * avoid_ang_mag, base_ang)
        base_lin = torch.where(has_block, base_lin * cfg.avoid_lin_scale, base_lin)

        return base_lin, base_ang

    # ----- ActionTerm API -----------------------------------------------
    def process_actions(self, actions: torch.Tensor) -> None:
        """RL residual + 룰베이스 base → 최종 (lin, ang) 으로 합성."""
        # 입력 액션의 NaN/inf 도 막음 (학습 초기 불안정 시 보호).
        actions = torch.nan_to_num(actions, nan=0.0, posinf=1.0, neginf=-1.0)
        self._raw_actions[:] = actions
        base_lin, base_ang = self._compute_base()
        # base controller 출력의 NaN/inf 차단 (raycaster·quat 비정상 시).
        base_lin = torch.nan_to_num(base_lin, nan=0.0, posinf=0.0, neginf=0.0)
        base_ang = torch.nan_to_num(base_ang, nan=0.0, posinf=0.0, neginf=0.0)

        # RL 출력(보통 tanh 후 [-1,1])을 물리 단위로 스케일.
        lin_res = actions[:, 0] * self.cfg.lin_residual_scale
        ang_res = actions[:, 1] * self.cfg.ang_residual_scale

        final_lin = (base_lin + lin_res).clamp(-self.cfg.max_lin, self.cfg.max_lin)
        final_ang = (base_ang + ang_res).clamp(-self.cfg.max_ang, self.cfg.max_ang)

        # 최종 액션 NaN/inf 마지막 가드.
        self._processed_actions = torch.nan_to_num(
            torch.stack([final_lin, final_ang], dim=-1),
            nan=0.0, posinf=0.0, neginf=0.0,
        )

    def apply_actions(self):
        # 부모(AckermannAction.apply_actions): _processed_actions (lin, ang)
        # → Ackermann 운동학 → 휠/조향 조인트 명령.
        super().apply_actions()
        # m0609 팔을 매 step HOME 으로 강제 — drive_test 와 동일 정책.
        # 학습 중 팔이 풀어 펴지지 않게 모든 env 에 일괄 적용.
        keep_arm_folded(self._asset)


# ---------------------------------------------------------------------------
# Cfg
# ---------------------------------------------------------------------------
@configclass
class ResidualAckermannActionCfg(AckermannActionCfg):
    """ResidualAckermannAction 의 환경설정 — AckermannActionCfg 확장.

    AckermannActionCfg 의 모든 필드(wheelbase 등) 를 그대로 받고, base
    controller·residual 스케일·안전 클램프 파라미터를 추가한다.
    """

    class_type: type[ActionTerm] = ResidualAckermannAction

    # command_manager 에서 goal pose 를 받아올 term 이름.  cfg 에서 같은
    # 이름의 CommandTerm 을 정의해야 한다 (xy world pose, 보통 random).
    goal_command_name: str = "goal_pose"

    # scene 안 raycaster 이름.  drive_test 와 동일하게 "height_scanner".
    raycaster_name: str = "height_scanner"

    # ----- goal-seek base ----------------------------------------------
    cruise_speed: float = 2.5
    """전방 정렬 후 base 전진 속도 (m/s)."""

    goal_turn_k: float = 2.5
    """yaw_err → base_ang 의 P 게인 (rad/s · rad)."""

    # ----- 장애물 prominence 판정 --------------------------------------
    prom_radius: int = 2
    """국소 돌출 비교 이웃 반경 (셀).  drive_test detector 와 동일."""

    height_thresh: float = 0.15
    """국소 돌출이 이 (m) 이상이면 장애물 셀로 본다."""

    front_range: float = 2.0
    """전방 장애물로 칠 최대 전방거리 (m)."""

    corridor: float = 0.7
    """진로 침범 corridor — |lat| < corridor 인 전방 장애물만 base 회피 발동."""

    # ----- 장애물 만났을 때 base ang/lin --------------------------------
    avoid_base_ang: float = 1.0
    """장애물 모서리 근처 base 회피 각속도 (rad/s)."""

    avoid_max_ang: float = 2.2
    """장애물 정중앙·근접 시 base 회피 각속도 (rad/s)."""

    avoid_lin_scale: float = 0.65
    """회피 중일 때 base_lin 에 곱하는 감속 계수 (0~1)."""

    # ----- residual 스케일 ----------------------------------------------
    lin_residual_scale: float = 1.0
    """RL 의 lin_residual [-1,1] 을 (m/s) 로 변환할 스케일."""

    ang_residual_scale: float = 1.5
    """RL 의 ang_residual [-1,1] 을 (rad/s) 로 변환할 스케일."""

    # ----- 최종 안전 클램프 ---------------------------------------------
    max_lin: float = 3.0
    """최종 (base+residual) 선속도 클램프 (±, m/s)."""

    max_ang: float = 2.5
    """최종 (base+residual) 각속도 클램프 (±, rad/s)."""
