# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""WASD 수동 주행 + 레이캐스트 지형 스캔 테스트 환경 (Isaac Lab manager-based).

avoid_test/rover_avoid/avoid_env_cfg.py 의 씬 구성(하향 RayCaster + Ackermann
액션)을 그대로 가져와 통합 차량 vehicle_v1.usd 에 붙인다.

  · 지형 : terrain_00022 — 지형+바위 병합 단일 메시 USD (terrain_00022_new)
  · 센서 : 하향 RayCaster (GridPattern 격자 — 정사각/직사각 가능)
  · 액션 : Ackermann 조향 — 2차원 (선속도, 각속도)

RL 이 아니라 수동 주행이므로 보상·종료·커맨드가 없는 ManagerBasedEnv
(씬·관측·액션·이벤트만) 를 쓴다.
"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import mdp
from rover_vehicle import (
    ACK_OFFSET,
    BODY_LINK_NAME,
    FRONT_REAR_DISTANCE,
    MIDDLE_WHEEL_DISTANCE,
    VEHICLE_CFG,
    WHEEL_RADIUS,
    WHEELBASE_LENGTH,
)

# 지형+바위 병합 단일 메시 USD — build_merged_terrain.py 가 만든 drive_test
# 안의 파일.  terrain_only.usd + rocks_merged.usd 의 삼각형을 한 Mesh 로 구운 것.
MERGED_TERRAIN_USD = str(Path(__file__).parent / "terrain_00022_new.usdc")


# ---------------------------------------------------------------------------
# Scene — terrain_00022 지형+바위 병합 USD, 차량, 하향 레이캐스터, 조명
# ---------------------------------------------------------------------------
@configclass
class DriveSceneCfg(InteractiveSceneCfg):
    """terrain_00022 지형+바위 병합 USD, 통합 차량, 하향 RayCaster, 조명."""

    # 지형 — terrain_only.usd(지형) + rocks_merged.usd(바위 80개) 의 삼각형을
    # 단일 Mesh 하나로 병합한 terrain_00022_new.usdc (build_merged_terrain.py
    # 산출물).  진짜 geometry 그대로라 모양이 동일하고, Mesh 가 1개라 메시
    # 1개만 보는 RayCaster 가 지형과 바위를 한 번에 스캔한다.
    # 50×50m 맵 중앙 = 베이스캠프 → env 원점 (0,0,0) 에 차량이 스폰된다.
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

    # 차량 — vehicle_v1.usd.
    robot: ArticulationCfg = VEHICLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # 하향 RayCaster — 차량 위 10m 에서 아래로 격자 ray (avoid_test 방식).
    # GridPattern size=(전방축, 좌우) m · 해상도 0.2m — 정사각/직사각 자유.
    # detector 가 격자 행·열 수를 RayCaster 설정에서 읽어 자동 대응한다.
    # /World/terrain(병합 메시 = 지형+바위)을 스캔 → 바위가 ray 에 잡히고,
    # detector 가 '국소 돌출량'으로 바위만 골라낸다 (완만한 경사·언덕은 무시).
    height_scanner = RayCasterCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODY_LINK_NAME}",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 10.0)),
        ray_alignment="yaw",  # 차량 yaw 만 따라 회전 (롤/피치 무시)
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=(4.0, 2.4)),
        debug_vis=True,        # 뷰어에 ray 히트점 표시
        mesh_prim_paths=["/World/terrain"],
        max_distance=100.0,
    )

    # 조명.
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )


# ---------------------------------------------------------------------------
# MDP — Actions : Ackermann 조향 (avoid_test 와 동일)
# ---------------------------------------------------------------------------
@configclass
class ActionsCfg:
    """액션: Ackermann 조향 — 2차원 (선속도 m/s, 각속도 rad/s)."""

    drive = mdp.AckermannActionCfg(
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
    )


# ---------------------------------------------------------------------------
# MDP — Observations : ManagerBasedEnv 가 관측 그룹 1개를 요구 → 최소 구성
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg:
    """관측 — 수동 주행이라 정책 입력이 필요 없으므로 직전 행동만."""

    @configclass
    class PolicyCfg(ObsGroup):
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ---------------------------------------------------------------------------
# MDP — Events : 리셋 시 차량을 맵 중앙(베이스캠프)·정면(+x) 으로
# ---------------------------------------------------------------------------
@configclass
class EventCfg:
    """리셋 이벤트 — env.reset() 시 차량을 스폰 위치·정면으로 되돌린다."""

    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
@configclass
class DriveEnvCfg(ManagerBasedEnvCfg):
    """vehicle_v1 WASD 주행 + 레이캐스트 테스트 환경."""

    scene: DriveSceneCfg = DriveSceneCfg(num_envs=1, env_spacing=20.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        # sim·제어·렌더 60Hz (decimation ×1).
        # decimation 을 1 로 둬 매 프레임 keep_arm_folded() 로 팔 자세를
        # 고정할 수 있게 한다 (decimation 이 크면 한 step 안에서 팔이 흐트러짐).
        self.decimation = 1
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        # 뷰어 카메라 — 차량 뒤 비스듬히, 정면 장애물이 보이도록.
        self.viewer.eye = (-6.0, 6.0, 5.0)
        self.viewer.lookat = (3.0, 0.0, 0.0)
        # 레이캐스터는 매 sim step 갱신 → 장애물 인식 반응성 확보.
        self.scene.height_scanner.update_period = 0.0
