# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""평지 + 무작위 박스 장애물 ManagerBasedRLEnvCfg — residual RL 1단계 학습.

terrain_00022 (실지형 + 80바위) 로 가기 전, 단순한 평면에 큐브 장애물만
깔린 환경에서 우선 학습한다.  reward·obs·termination 셋업을 검증하고
정책이 "goal 가기 + 큐브 회피" 기본기를 익힌 뒤, 다음 단계에서 terrain_00022
환경으로 fine-tune 할 수 있다.

  · 지형 : TerrainGenerator + MeshRepeatedBoxesTerrainCfg (큐브 0.5×0.5×0.5m).
           sub-terrain 별 6~10개 박스, 중앙 2m 반경은 platform 으로 비워둠.
  · 액션 : ResidualAckermannAction — goal-seek 룰베이스 + RL residual.
  · 관측 : body-frame goal·레이캐스트 prominence·base 속도·prev action.
  · 보상 : progress + position_tanh + obstacle_proximity + collision + goal_bonus
           + goal_alignment + action_rate + time_penalty.
  · 종료 : 도착 / 충돌 / out-of-bounds / timeout.
  · 이벤트: reset 시 flat_patches['spawn'] 에서 안전한 위치로 spawn.
"""

from __future__ import annotations

import math
import os
import sys

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
from isaaclab.terrains import (
    FlatPatchSamplingCfg,
    TerrainGeneratorCfg,
    TerrainImporterCfg,
)
from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshRepeatedBoxesTerrainCfg
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

# ---------------------------------------------------------------------------
# Terrain — flat + repeated boxes (curriculum 비활성).
# ---------------------------------------------------------------------------
FLAT_BOXES_TERRAIN = TerrainGeneratorCfg(
    seed=27182,
    curriculum=False,
    size=(12.0, 12.0),
    num_rows=4,
    num_cols=4,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    border_width=4.0,
    sub_terrains={
        "boxes": MeshRepeatedBoxesTerrainCfg(
            proportion=1.0,
            object_type="box",
            object_params_start=MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=6, size=(0.5, 0.5), height=0.5,
            ),
            object_params_end=MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=10, size=(0.5, 0.5), height=0.5,
            ),
            platform_width=2.0,
            platform_height=0.0,
            # 안전 좌표 사전 샘플링 — spawn·goal 둘 다 여기서 가져온다.
            flat_patch_sampling={
                "spawn": FlatPatchSamplingCfg(
                    num_patches=80, patch_radius=0.7, max_height_diff=0.05,
                ),
                "goal": FlatPatchSamplingCfg(
                    num_patches=150, patch_radius=0.7, max_height_diff=0.05,
                ),
            },
        ),
    },
)


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
@configclass
class FlatSceneCfg(InteractiveSceneCfg):
    """평지 + 박스 장애물 + 차량 + 레이캐스트 + 접촉센서 + 조명."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=FLAT_BOXES_TERRAIN,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    # 차량 — drive_test 와 동일.
    robot: ArticulationCfg = VEHICLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # 하향 RayCaster — drive_test 와 동일 설정.  /World/ground 한 메시(생성된 박스
    # 포함) 만 스캔 → prominence 로 박스만 골라낸다.
    height_scanner = RayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 10.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=(4.0, 2.4)),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
        max_distance=100.0,
        update_period=0.0,
    )

    # 차량 body 접촉센서 — 충돌 페널티·종료 신호로 사용.
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
# Commands — goal_pose (FlatPatchesGoalCommand)
# ---------------------------------------------------------------------------
@configclass
class CommandsCfg:
    """무작위 goal — terrain.flat_patches['goal'] 에서 sample."""

    goal_pose = mdp.FlatPatchesGoalCommandCfg(
        asset_name="robot",
        patch_name="goal",
        min_goal_dist=2.5,
        max_goal_dist=8.0,
        resampling_time_range=(1.0e9, 1.0e9),   # 사실상 reset 시에만.
        debug_vis=True,    # 뷰어에 빨간 구로 goal 위치 표시.
    )


# ---------------------------------------------------------------------------
# Actions — ResidualAckermann (룰베이스 base + RL residual)
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
# Observations — body-frame, 정책 입력
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg:
    """정책 group — 모두 body-frame."""

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
# Rewards
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
    out_of_bounds = DoneTerm(
        func=mdp.out_of_bounds, params={"limit": 7.0},
    )
    tilt = DoneTerm(
        func=mdp.vehicle_tilt, params={"max_tilt_deg": 60.0},
    )


# ---------------------------------------------------------------------------
# Events — reset 시 안전한 spawn
# ---------------------------------------------------------------------------
@configclass
class EventsCfg:
    reset_robot = EventTerm(
        func=mdp.reset_root_flat_patches,
        mode="reset",
        params={
            "patch_name": "spawn",
            "yaw_range": (0.0, 2.0 * math.pi),
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class FlatEnvCfg(ManagerBasedRLEnvCfg):
    """평지+박스 ManagerBasedRLEnvCfg."""

    scene: FlatSceneCfg = FlatSceneCfg(num_envs=64, env_spacing=12.0)
    commands: CommandsCfg = CommandsCfg()
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self):
        # 제어 30Hz (decimation=4, sim 120Hz).  Ackermann 조향엔 충분.
        self.decimation = 4
        self.episode_length_s = 30.0
        self.sim.dt = 1.0 / 120.0
        self.sim.render_interval = self.decimation

        # 뷰어 — 차량 위 비스듬.
        self.viewer.eye = (-8.0, 6.0, 5.0)
        self.viewer.lookat = (0.0, 0.0, 0.0)
