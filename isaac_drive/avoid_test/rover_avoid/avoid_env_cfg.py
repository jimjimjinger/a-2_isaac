# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""평지 + 장애물 회피 RL 환경 설정 (Isaac Lab manager-based).

m0609_lift_code_ver2/m0609_lift/lift_env_cfg.py 와 같은 역할 — 씬·관측·액션·
커맨드·이벤트·보상을 선언형 Cfg 로 구성한다.

지형 설계 (사용자 선택: env마다 다른 고정 레이아웃):
  Isaac Lab 의 ``HfDiscreteObstaclesTerrain`` 으로 '평지 + 랜덤 박스 장애물'을
  하나의 height-field 메시로 생성한다.  RayCaster 는 메시 1개만 인식하므로
  장애물을 별도 prim 이 아닌 지형 메시 자체에 포함시켜야 한다.
  ``num_rows × num_cols`` 타일마다 장애물 배치가 다르게 생성되므로, 병렬 env
  들이 서로 다른 타일에 올라가 'env마다 다른 고정 레이아웃'이 된다.

1단계(현재): 보상은 비워 둔 골격.  씬·센서·액션·커맨드만 동작 확인.
2단계: rewards.py / terminations.py 를 채우고 학습 스크립트를 붙인다.
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.terrains.height_field import HfDiscreteObstaclesTerrainCfg
from isaaclab.utils import configclass

from . import mdp
from .rover import (
    ACK_OFFSET,
    BODY_LINK_NAME,
    FRONT_REAR_DISTANCE,
    MIDDLE_WHEEL_DISTANCE,
    ROVER_CFG,
    WHEEL_RADIUS,
    WHEELBASE_LENGTH,
)

# ---------------------------------------------------------------------------
# 지형 — 평지 + 랜덤 박스 장애물 (단일 height-field 메시)
# ---------------------------------------------------------------------------
OBSTACLE_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(20.0, 20.0),       # 타일 1개 크기 (m)
    border_width=4.0,        # 타일 격자 둘레 벽
    num_rows=4,
    num_cols=4,              # 4×4 = 16개 타일 → 16가지 장애물 배치
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "obstacles": HfDiscreteObstaclesTerrainCfg(
            proportion=1.0,
            obstacle_height_mode="fixed",
            obstacle_height_range=(0.8, 0.8),   # 로버가 못 넘는 높이
            obstacle_width_range=(0.6, 1.6),
            num_obstacles=8,                    # 타일당 장애물 개수
            platform_width=3.5,                 # 중앙 빈 공간 (로버 스폰 자리)
        ),
    },
)


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
@configclass
class AvoidSceneCfg(InteractiveSceneCfg):
    """평지+장애물 지형, 로버, 하향 레이캐스터, 접촉센서, 조명."""

    # 지형 — 위에서 만든 generator 로 평지+장애물 메시 생성.
    terrain = TerrainImporterCfg(
        prim_path="/World/terrain",
        terrain_type="generator",
        terrain_generator=OBSTACLE_TERRAIN_CFG,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    # 로버 — env 마다 복제.
    robot: ArticulationCfg = ROVER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # 하향 RayCaster — 로버 위 10m 에서 아래로 격자 ray.
    # GridPattern 3m×3m, 해상도 0.2m → 16×16 = 256 ray (로버 중심 사방 ±1.5m).
    # 장애물은 지형 메시의 '높이 돌출'로 잡힌다.
    height_scanner = RayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 10.0)),
        ray_alignment="yaw",  # 로버 yaw 만 따라 회전 (롤/피치 무시)
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=(3.0, 3.0)),
        debug_vis=True,        # 뷰어에 ray 히트점 표시 — 1단계 시각 확인용
        mesh_prim_paths=["/World/terrain"],
        max_distance=100.0,
    )

    # 접촉센서 — 로버 몸체(Body). 바퀴는 항상 바닥에 닿지만 몸체는 평소
    # 공중에 떠 있으므로, 몸체에 힘이 잡히면 = 장애물에 부딪힌 것.
    contact_sensor = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        update_period=0.0,
        debug_vis=True,
    )

    # 조명.
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )


# ---------------------------------------------------------------------------
# MDP — Actions
# ---------------------------------------------------------------------------
@configclass
class ActionsCfg:
    """액션: 잔차(Residual) Ackermann — 1차원 (조향 보정값).

    베이스 컨트롤러가 goal 로 향하는 선속도·각속도를 만들고, RL 정책은
    거기에 더할 조향 보정만 출력한다 → RL 은 'goal 찾아가기'가 아니라
    '회피'만 학습한다.
    """

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
        # 베이스 goto-goal 컨트롤러.
        command_name="target_pose",
        base_speed=0.8,
        heading_gain=2.0,
        max_base_ang=1.0,
        # RL 조향 보정.
        residual_scale=1.0,
    )


# ---------------------------------------------------------------------------
# MDP — Observations
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg:
    """관측 그룹 — 목표상대 2 + height-scan 256 + 직전행동 1 = 259차원."""

    @configclass
    class PolicyCfg(ObsGroup):
        distance = ObsTerm(
            func=mdp.distance_to_goal,
            params={"command_name": "target_pose"},
            scale=0.1,
        )
        angle = ObsTerm(
            func=mdp.angle_to_goal,
            params={"command_name": "target_pose"},
            scale=1.0 / math.pi,
        )
        height_scan = ObsTerm(
            func=mdp.height_scan_grid,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            scale=0.1,
        )
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True   # Play 변형에서 False 로 끔
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ---------------------------------------------------------------------------
# MDP — Commands (매 에피소드 랜덤 목표)
# ---------------------------------------------------------------------------
@configclass
class CommandsCfg:
    """목표 포즈 커맨드 — env 원점 주변에서 랜덤 2D 위치를 1회 샘플."""

    target_pose = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=True,  # 목표 방향을 향하도록 heading 자동 설정
        # 에피소드보다 긴 재샘플 주기 → 한 에피소드 동안 목표 고정.
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=True,       # 뷰어에 목표 위치 화살표 마커 표시
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=(-7.0, 7.0),
            pos_y=(-7.0, 7.0),
            heading=(-math.pi, math.pi),
        ),
    )


# ---------------------------------------------------------------------------
# MDP — Events
# ---------------------------------------------------------------------------
@configclass
class EventCfg:
    """리셋 이벤트 — 로버를 타일 중앙 부근에 랜덤 방향으로 배치."""

    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "yaw": (-math.pi, math.pi)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


# ---------------------------------------------------------------------------
# MDP — Rewards (2단계에서 채움)
# ---------------------------------------------------------------------------
@configclass
class RewardsCfg:
    """보상 항목 — Franka/RLRoverLab 내비게이션 레시피 기반.

    weight 사다리: 목표 접근(5)·도달(5) 양(+) > 충돌(-5) 페널티.
    대부분 함수 내부에서 /max_episode_length 정규화됨.
    """

    # 1) 목표 접근 — dense 유도.
    goal_distance = RewTerm(
        func=mdp.goal_distance_reward,
        weight=5.0,
        params={"command_name": "target_pose"},
    )
    # 2) 목표 도달 보너스 — sparse.
    goal_reached = RewTerm(
        func=mdp.goal_reached_reward,
        weight=5.0,
        params={"command_name": "target_pose", "threshold": 0.5},
    )
    # 3) 목표 바라보기 — 방향 정렬 유도.
    heading = RewTerm(
        func=mdp.heading_to_goal_reward,
        weight=3.0,
        params={"command_name": "target_pose"},
    )
    # 4) 목표가 옆/뒤일 때 페널티.
    angle_penalty = RewTerm(
        func=mdp.angle_to_goal_penalty,
        weight=-1.5,
        params={"command_name": "target_pose"},
    )
    # 5) 장애물 충돌 페널티 — 회피 학습의 핵심 신호.
    collision = RewTerm(
        func=mdp.collision_penalty,
        weight=-5.0,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor"), "threshold": 1.0},
    )
    # 6) 부드러운 조향 보정 — 보정값 급변 페널티.
    oscillation = RewTerm(func=mdp.oscillation_penalty, weight=-0.05)
    # 6-2) 조향 보정 크기 페널티 — 장애물 없을 땐 보정 0(베이스 경로 유지) 유도.
    #      (잔차 RL: 행동[0]이 선속도가 아니라 조향 보정이라 backward_penalty 는 제외)
    steering_residual = RewTerm(func=mdp.steering_residual_penalty, weight=-0.1)
    # 7) 장애물 근접 페널티 — 부딪히기 전 여유있게 우회 (바퀴 걸림 방지).
    obstacle_proximity = RewTerm(
        func=mdp.obstacle_proximity_penalty,
        weight=-1.5,
        params={"sensor_cfg": SceneEntityCfg("height_scanner"), "radius": 0.9},
    )
    # 8) 레이캐스트 충돌 페널티 — 낮은 장애물 바퀴걸림(몸체 접촉 못 잡음) 대응.
    obstacle_hit = RewTerm(
        func=mdp.obstacle_hit_penalty,
        weight=-5.0,
        params={"sensor_cfg": SceneEntityCfg("height_scanner"), "radius": 0.6},
    )


# ---------------------------------------------------------------------------
# MDP — Terminations
# ---------------------------------------------------------------------------
@configclass
class TerminationsCfg:
    """종료 조건 — 시간초과 / 목표도달(성공) / 충돌(실패) / 과도이탈."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    goal_reached = DoneTerm(
        func=mdp.goal_reached,
        params={"command_name": "target_pose", "threshold": 0.5},
    )
    collision = DoneTerm(
        func=mdp.collision,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor"), "threshold": 1.0},
    )
    # 레이캐스트 충돌 — 낮은 장애물 바퀴걸림도 즉시 종료(재소환).
    obstacle_hit = DoneTerm(
        func=mdp.obstacle_hit,
        params={"sensor_cfg": SceneEntityCfg("height_scanner"), "radius": 0.6},
    )
    too_far = DoneTerm(
        func=mdp.too_far_from_goal,
        params={"command_name": "target_pose", "max_distance": 15.0},
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class AvoidEnvCfg(ManagerBasedRLEnvCfg):
    """평지 장애물 회피 환경 설정."""

    scene: AvoidSceneCfg = AvoidSceneCfg(num_envs=2, env_spacing=8.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        # 제어 주기 5Hz: sim 30Hz, decimation ×6.
        self.decimation = 6
        self.episode_length_s = 60.0
        self.sim.dt = 1.0 / 30.0
        self.sim.render_interval = self.decimation
        self.viewer.eye = (12.0, 12.0, 9.0)

        # 센서 갱신 주기 = 제어 주기.
        self.scene.height_scanner.update_period = self.sim.dt * self.decimation
        self.scene.contact_sensor.update_period = self.sim.dt * self.decimation

        # PhysX GPU 버퍼 — 장애물 많은 씬 대비 여유 있게.
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 2**21
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 2**21
        # narrow-phase 충돌 스택 — 4096 env 처럼 대규모 병렬 시 6륜 로버 contact
        # 가 기본값(2**26≈67MB)을 살짝 넘겨 'Contacts dropped' 가 난다. 넉넉히 키움.
        self.sim.physx.gpu_collision_stack_size = 2**28
        # contact patch 버퍼 — 초반 학습(랜덤 정책)에 4096 로버가 큐브·지형에
        # 동시다발 충돌하며 기본값(163840)을 넘긴다. 넉넉히 키움.
        self.sim.physx.gpu_max_rigid_patch_count = 2**19


@configclass
class AvoidEnvCfg_PLAY(AvoidEnvCfg):
    """재생용 변형 — env 16개, 관측 노이즈 OFF (m0609 PLAY 와 동일 패턴)."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 8.0
        self.observations.policy.enable_corruption = False
