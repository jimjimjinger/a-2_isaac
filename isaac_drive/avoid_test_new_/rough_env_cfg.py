# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""terrain_00022 (50×50m 지형+바위 80개) ManagerBasedRLEnvCfg — residual RL 2단계.

1단계(`flat_env_cfg.py`) 의 평지+박스 환경에서 학습한 정책(best.pt)을 시작점으로,
실제 지형(언덕·크레이터+바위)에 fine-tune 한다.

  · 지형 : drive_test 와 동일한 terrain_00022_new.usdc.  TerrainImporter "usd".
  · 액션 : ResidualAckermannAction — 1단계와 동일 base controller.
  · 관측 : 1단계와 동일 (raycaster · body-frame goal · vel · last_action).
  · 보상 : 1단계와 동일.
  · 종료 : 도착 / 충돌 / out-of-bounds (env_origin 기준) / timeout.
  · 이벤트: reset_root_safe — obstacle_grid 로 spawn 추첨.
  · command: RandomGoalCommand — obstacle_grid 로 goal 추첨.

env 배치: terrain_00022 단일 USD 를 모든 env 가 공유.  6×6=36 env 가 env_spacing=8m
그리드(48×48m) 로 terrain 안에 분포 → 각 env 의 env_origin 이 terrain 의 다른
부분에 위치 → 자연스러운 obstacle 다양성.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import mdp  # noqa: E402
from rover_vehicle import (  # noqa: E402
    ACK_OFFSET,
    BODY_LINK_NAME,
    FRONT_REAR_DISTANCE,
    MIDDLE_WHEEL_DISTANCE,
    VEHICLE_CFG,
    WHEEL_RADIUS,
    WHEELBASE_LENGTH,
)

# drive_test 에서 복사된 terrain_00022_new.usdc 의 경로.
MERGED_TERRAIN_USD = str(Path(__file__).parent / "terrain_00022_new.usdc")


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
@configclass
class RoughSceneCfg(InteractiveSceneCfg):
    """terrain_00022 단일 USD + 차량 + 레이캐스트 + 접촉센서 + 조명."""

    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="usd",
        usd_path=MERGED_TERRAIN_USD,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    robot: ArticulationCfg = VEHICLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # drive_test 와 동일 RayCaster — 4×2.4m 격자.
    height_scanner = RayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 10.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=(4.0, 2.4)),
        debug_vis=False,
        mesh_prim_paths=["/World/terrain"],
        max_distance=100.0,
        update_period=0.0,
    )

    contact_sensor = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        update_period=0.0,
        history_length=1,
        debug_vis=False,
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )


# ---------------------------------------------------------------------------
# Commands — RandomGoalCommand (obstacle_grid 기반)
# ---------------------------------------------------------------------------
@configclass
class CommandsCfg:
    # 테스트 모드 — 모든 env 가 베이스캠프(world 원점) 주변에 spawn 하고,
    # goal 은 베이스캠프에서 3~12m 떨어진 무작위 world 좌표.  방향만 무작위
    # 인 셈이라 차량마다 다른 goal 을 향해 부채꼴로 흩어진다.
    goal_pose = mdp.RandomGoalCommandCfg(
        asset_name="robot",
        sample_radius=22.0,          # world (0,0) 기준 ±22m 박스 (terrain 가장자리 마진 2m)
        basecamp_radius=0.0,         # 베이스캠프 회피 X — 우리가 거기서 출발하니까
        min_goal_dist=8.0,           # spawn 에서 최소 8m — 깊숙한 목표
        max_goal_dist=22.0,          # 최대 22m — 맵 거의 끝까지
        force_center_world=(0.0, 0.0),    # goal 중심 = world 원점.
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=True,
    )


# ---------------------------------------------------------------------------
# Actions — flat 과 동일 (residual + base controller)
# ---------------------------------------------------------------------------
@configclass
class ActionsCfg:
    drive = mdp.ResidualAckermannActionCfg(
        asset_name="robot",
        wheelbase_length=WHEELBASE_LENGTH,
        middle_wheel_distance=MIDDLE_WHEEL_DISTANCE,
        rear_and_front_wheel_distance=FRONT_REAR_DISTANCE,
        wheel_radius=WHEEL_RADIUS,
        min_steering_radius=0.8,
        steering_joint_names=[".*Steer_Revolute"],
        drive_joint_names=[".*Drive_Continuous"],
        offset=ACK_OFFSET,
        scale=(1.0, 1.0),
        goal_command_name="goal_pose",
        raycaster_name="height_scanner",
        cruise_speed=2.5,
        goal_turn_k=2.5,
        front_range=2.0,
        corridor=0.7,
        height_thresh=0.15,
        prom_radius=2,
        avoid_base_ang=1.0,
        avoid_max_ang=2.2,
        avoid_lin_scale=0.65,
        lin_residual_scale=1.0,
        ang_residual_scale=1.5,
        max_lin=3.0,
        max_ang=2.5,
    )


# ---------------------------------------------------------------------------
# Observations — flat 과 동일 (정책 호환 위해 차원·순서 일치)
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        goal_xy_dist = ObsTerm(func=mdp.goal_body_xy_dist,
                               params={"command_name": "goal_pose"})
        goal_yaw = ObsTerm(func=mdp.goal_yaw_sincos,
                           params={"command_name": "goal_pose"})
        raycast = ObsTerm(func=mdp.raycast_prominence)
        lin_vel = ObsTerm(func=mdp.base_lin_vel_b)
        ang_vel = ObsTerm(func=mdp.base_ang_vel_b)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ---------------------------------------------------------------------------
# Rewards — flat 과 동일
# ---------------------------------------------------------------------------
@configclass
class RewardsCfg:
    progress = RewTerm(
        func=mdp.progress, weight=1.5,
        params={"command_name": "goal_pose"},
    )
    position_coarse = RewTerm(
        func=mdp.position_tanh, weight=0.5,
        params={"std": 3.0, "command_name": "goal_pose"},
    )
    position_fine = RewTerm(
        func=mdp.position_tanh, weight=0.3,
        params={"std": 0.5, "command_name": "goal_pose"},
    )
    obstacle_proximity = RewTerm(
        func=mdp.obstacle_proximity_penalty, weight=-1.0,
        params={"near_threshold": 1.2, "consider_back": 0.4},
    )
    collision = RewTerm(
        func=mdp.collision_penalty, weight=-400.0,
        params={"force_threshold": 1.0},
    )
    goal_bonus = RewTerm(
        func=mdp.goal_reached_bonus, weight=200.0,
        params={"threshold": 0.6, "command_name": "goal_pose"},
    )
    goal_alignment = RewTerm(
        func=mdp.goal_alignment, weight=0.1,
        params={"command_name": "goal_pose"},
    )
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.005)
    time_penalty = RewTerm(func=mdp.time_penalty, weight=-0.01)


# ---------------------------------------------------------------------------
# Terminations
# ---------------------------------------------------------------------------
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    goal = DoneTerm(
        func=mdp.goal_reached,
        params={"threshold": 0.6, "command_name": "goal_pose"},
    )
    collision = DoneTerm(
        func=mdp.collision, params={"force_threshold": 1.0},
    )
    # 테스트 모드 — 모든 env 가 world (0,0) 에서 출발하므로 env_origin 기준
    # out_of_bounds 는 의미 없음 (envs 가 grid 로 흩어진 환경 cfg 상의 위치와
    # 차량 실제 위치가 어긋남).  world 경계만 확인.
    world_oob = DoneTerm(
        func=mdp.world_out_of_bounds,
        params={"world_limit": 24.0, "min_z": -2.0},
    )
    # 차량 뒤집힘 직전 — roll/pitch 60° 초과.  부서지기 전에 reset.
    tilt = DoneTerm(
        func=mdp.vehicle_tilt, params={"max_tilt_deg": 60.0},
    )


# ---------------------------------------------------------------------------
# Events — reset 시 안전한 spawn
# ---------------------------------------------------------------------------
@configclass
class EventsCfg:
    # 테스트 모드 — 모든 env 가 베이스캠프(world 원점) 주변 ±1.5m 평지에 spawn.
    # 베이스캠프는 8×8m 평지라 차량들이 겹쳐도 무난.  env_origin 무시.
    reset_robot = EventTerm(
        func=mdp.reset_root_safe,
        mode="reset",
        params={
            "sample_radius": 1.5,
            "basecamp_radius": 0.0,          # 베이스캠프 안에서 spawn 허용.
            "clearance_radius": 0.0,         # 평지라 footprint 검사 불필요.
            "z_clearance": 0.4,
            "force_center_world": (0.0, 0.0),    # spawn 중심 = world 원점.
            "yaw_range": (0.0, 2.0 * math.pi),
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class RoughEnvCfg(ManagerBasedRLEnvCfg):
    """terrain_00022 + 36env (6×6 grid, spacing 8m) ManagerBasedRLEnvCfg."""

    scene: RoughSceneCfg = RoughSceneCfg(num_envs=36, env_spacing=8.0)
    commands: CommandsCfg = CommandsCfg()
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 40.0          # flat 30s → rough 40s (지형 험해 시간 ↑)
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation
        self.viewer.eye = (-12.0, 12.0, 8.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
