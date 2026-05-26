"""recovery_env_cfg.py — Isaac Lab ManagerBasedRLEnv 설정 (v2: 화성 지형 + 강화 물리).

씬 구성:
  - Vehicle (vehicle_v3_physx.usd) : rover + m0609 단일 articulation
    ArticulationRoot: /Root/Vehicle/m0609/base_link
  - 화성 height-field 지형 (평지/크레이터/경사/울퉁불퉁)
  - 화성 중력 −3.72 m/s²

Observation (dim=40):
  upright_vec          3   — local z축의 world 투영 (기립 벡터)
  vehicle_roll/pitch   2   — Euler 자세각
  vehicle_pos_z        1   — 지면 대비 높이
  vehicle_lin_vel      3   — 선속도
  vehicle_ang_vel      3   — 각속도
  arm_joint_pos        6   — M0609 joint_1~6
  arm_joint_vel        6   — M0609 joint_1~6
  drive_vel            6   — 6개 Drive 바퀴 각속도
  steer_pos            4   — 4개 Steer 관절 위치
  rocker_pos           5   — 서스펜션 관절 위치
  stable_frames        1   — 연속 기립 유지 프레임 수 (정규화)

Action (dim=16):
  m0609 joint position targets  (6)   — Δ±0.5 rad/step
  rover drive wheel velocities  (6)   — ±15 rad/s
  rover steer position targets  (4)   — Δ±0.3 rad/step
"""
from __future__ import annotations

import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as loco_mdp
import recovery_mdp as mdp
from mars_terrain_cfg import MARS_DOME_LIGHT_CFG, MARS_TERRAIN_CFG

# ── 에셋 경로 ──────────────────────────────────────────────────────────────
_REPO       = os.path.join(os.path.dirname(__file__), "../../..")
_ASSET_DIR  = os.path.join(_REPO, "isaac_sim/assets/vehicle")

# vehicle_v3_physx.usd 없으면 원본 fallback
_PHYSX_USD  = os.path.join(_ASSET_DIR, "vehicle_v3_physx.usd")
_ORIG_USD   = os.path.join(_ASSET_DIR, "vehicle_v3.usd")
VEHICLE_USD = _PHYSX_USD if os.path.exists(_PHYSX_USD) else _ORIG_USD

ARM_ACTION_SCALE   = 0.1    # ±0.1 rad delta per step
WHEEL_ACTION_SCALE = 5.0    # rad/s
STEER_ACTION_SCALE = 0.1    # ±0.1 rad delta per step

# 성공 판정: 15° 이하로 이 프레임 수 이상 유지해야 성공
STABLE_FRAMES_REQUIRED = 8    # 50Hz × 0.16s (더 완화: 초기 성공 신호 확보)


# ── Vehicle ArticulationCfg ──────────────────────────────────────────────────
VEHICLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=VEHICLE_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            max_contact_impulse=500.0,
            max_linear_velocity=10.0,
            max_angular_velocity=100.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,          # arm-rover 내부 충돌 제거 (2-3x speedup)
            solver_position_iteration_count=8,      # 16→8: ~1.5x speedup
            solver_velocity_iteration_count=1,      # 2→1
            sleep_threshold=0.05,                   # 0.005→0.05: 정지 body 빠르게 sleep
            stabilization_threshold=0.01,           # 0.001→0.01
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.005,
            rest_offset=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.6),
        joint_pos={
            "joint_1": 0.0,
            "joint_2": -0.5,
            "joint_3":  1.2,
            "joint_4": 0.0,
            "joint_5":  0.5,
            "joint_6": 0.0,
            ".*Drive_Continuous": 0.0,
            ".*Steer_Revolute":   0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            velocity_limit=2.0,
            effort_limit=200.0,
            stiffness=400.0,
            damping=40.0,
        ),
        "drive": ImplicitActuatorCfg(
            joint_names_expr=[".*Drive_Continuous"],
            velocity_limit=30.0,
            effort_limit=150.0,
            stiffness=0.0,
            damping=5.0,
        ),
        "steer": ImplicitActuatorCfg(
            joint_names_expr=[".*Steer_Revolute"],
            velocity_limit=3.0,
            effort_limit=50.0,
            stiffness=8000.0,
            damping=100.0,
        ),
        # rocker-bogie 패시브 관절: position 제어 없음, 댐핑만
        # RL/RR_Steer_Revolute 는 steer 액추에이터에서 이미 커버 → 여기서 제외
        "suspension": ImplicitActuatorCfg(
            joint_names_expr=[
                "FR_Rocker_Revolute", "FL_Rocker_Revolute",
                "Differential_Revolute",
                "RL_Rocker_Revolute", "RR_Rocker_Revolute",
            ],
            velocity_limit=5.0,
            effort_limit=500.0,
            stiffness=0.0,
            damping=50.0,
        ),
    },
)


# ── Scene ───────────────────────────────────────────────────────────────────
@configclass
class RecoverySceneCfg(InteractiveSceneCfg):
    # 화성 지형 (height-field 기반)
    terrain = MARS_TERRAIN_CFG.replace(prim_path="/World/terrain")

    # sky_light 제거 — headless 학습에서 scene graph 오버헤드 불필요

    # rover + m0609 arm 단일 articulation
    vehicle: ArticulationCfg = VEHICLE_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Vehicle"
    )


# ── Observations ─────────────────────────────────────────────────────────────
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # 기립 벡터 (local z → world): upright=(0,0,1), 뒤집힘=(0,0,-1)
        upright_vec   = ObsTerm(func=mdp.upright_vec,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})

        # 자세각
        rover_roll    = ObsTerm(func=mdp.rover_roll,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_pitch   = ObsTerm(func=mdp.rover_pitch,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})

        # 높이
        rover_pos_z   = ObsTerm(func=mdp.rover_pos_z,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})

        # 속도
        rover_lin_vel = ObsTerm(func=mdp.rover_lin_vel,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_ang_vel = ObsTerm(func=mdp.rover_ang_vel,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})

        # arm 관절 상태
        arm_joint_pos = ObsTerm(
            func=mdp.arm_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle",
                                                 joint_names=["joint_[1-6]"])},
        )
        arm_joint_vel = ObsTerm(
            func=mdp.arm_joint_vel,
            params={"asset_cfg": SceneEntityCfg("vehicle",
                                                 joint_names=["joint_[1-6]"])},
        )

        # 바퀴 상태
        rover_drive_vel = ObsTerm(
            func=mdp.rover_drive_vel,
            params={"asset_cfg": SceneEntityCfg("vehicle",
                                                 joint_names=[".*Drive_Continuous"])},
        )
        rover_steer_pos = ObsTerm(
            func=mdp.rover_steer_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle",
                                                 joint_names=[".*Steer_Revolute"])},
        )

        # 서스펜션(rocker-bogie) 관절 위치
        rocker_joint_pos = ObsTerm(
            func=mdp.rocker_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
                "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
                "RL_Rocker_Revolute", "RR_Rocker_Revolute",
            ])},
        )

        # 연속 기립 유지 프레임 (정규화 [0,1])
        stable_frames = ObsTerm(func=mdp.stable_frames_normalized,
                                params={"required": STABLE_FRAMES_REQUIRED})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ── Actions ───────────────────────────────────────────────────────────────────
@configclass
class ActionsCfg:
    arm_action = loco_mdp.JointPositionActionCfg(
        asset_name="vehicle",
        joint_names=["joint_[1-6]"],
        scale=ARM_ACTION_SCALE,
        use_default_offset=True,
    )
    wheel_action = loco_mdp.JointVelocityActionCfg(
        asset_name="vehicle",
        joint_names=[".*Drive_Continuous"],
        scale=WHEEL_ACTION_SCALE,
    )
    steer_action = loco_mdp.JointPositionActionCfg(
        asset_name="vehicle",
        joint_names=[".*Steer_Revolute"],
        scale=STEER_ACTION_SCALE,
        use_default_offset=True,
    )


# ── Events ────────────────────────────────────────────────────────────────────
@configclass
class EventCfg:
    # 주 reset: 넘어진 자세 (side-fall 60~120°)
    reset_vehicle_fallen = EventTerm(
        func=mdp.reset_vehicle_fallen,
        mode="reset",
        params={
            "asset_cfg":   SceneEntityCfg("vehicle"),
            "roll_range":  (math.radians(60), math.radians(120)),
            "pitch_range": (-math.radians(30), math.radians(30)),
            "height":      0.5,
        },
    )

    # 크레이터 추락 시나리오 reset (30% 확률로 대체)
    reset_crater_fall = EventTerm(
        func=mdp.reset_crater_fall,
        mode="reset",
        params={
            "asset_cfg":     SceneEntityCfg("vehicle"),
            "crater_prob":   0.3,
            "roll_range":    (math.radians(80), math.radians(100)),
            "pitch_range":   (math.radians(-20), math.radians(20)),
            "crater_depth":  0.4,
        },
    )

    # 물리 파라미터 랜덤화 (sim-to-real gap 대비)
    randomize_physics = EventTerm(
        func=mdp.randomize_physics_params,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )


# ── Rewards ───────────────────────────────────────────────────────────────────
@configclass
class RewardsCfg:
    # ─── Dense 보상 (매 스텝) ─────────────────────────────────────────────

    # cos 기반 기립 방향 보상 — [0,1] 항상 양수 (초기자세=0.5, 기립=1.0)
    upright_cosine = RewTerm(
        func=mdp.upright_cosine_reward,
        weight=10.0,     # 20→10: 항상양수로 바뀌어 절대값 크기 조정
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )

    # 기립 근접 시 gaussian 집중 보상 (마지막 30° 구간 강화)
    near_success = RewTerm(
        func=mdp.near_success_reward,
        weight=15.0,     # 신규: 기립 직전 gradient 강화
        params={"asset_cfg": SceneEntityCfg("vehicle"), "sigma_deg": 30.0},
    )

    # 높이 보상 (지면 대비 차체 높이가 올라올수록)
    height_reward = RewTerm(
        func=mdp.height_reward,
        weight=5.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "fallen_z": 0.2, "upright_z": 0.7},
    )

    # 기립 안정성 보상 (연속 유지 프레임에 비례)
    stable_upright = RewTerm(
        func=mdp.stable_upright_reward,
        weight=20.0,     # 8→20: 안정 유지 강하게 유도
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0,
                "required": STABLE_FRAMES_REQUIRED},
    )

    # ─── Sparse 보상 (성공 이벤트) ────────────────────────────────────────

    # 성공 완료 보너스 (stable_frames >= required)
    success_bonus = RewTerm(
        func=mdp.stable_success_bonus,
        weight=200.0,    # 150→200: 성공 신호 강화
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0,
                "required": STABLE_FRAMES_REQUIRED},
    )

    # ─── 패널티 ──────────────────────────────────────────────────────────

    # 뒤집힌 상태 지속 패널티 (크게 완화 — 초기 자세이므로)
    fallen_penalty = RewTerm(
        func=mdp.fallen_penalty,
        weight=-1.0,     # -5.0→-1.0: 초기 자세 페널티 완화
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "tilt_threshold_deg": 75.0},
    )

    # 시간 패널티 (빠른 복원 유도)
    time_penalty = RewTerm(
        func=mdp.time_alive_penalty,
        weight=-0.1,     # -0.3→-0.1: 탐색 억제하지 않도록 완화
    )

    # arm 관절 속도 패널티
    arm_vel_penalty = RewTerm(
        func=mdp.joint_vel_penalty,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("vehicle",
                                             joint_names=["joint_[1-6]"])},
    )

    # arm 관절 한계 패널티
    joint_limit_penalty = RewTerm(
        func=mdp.joint_limit_penalty,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("vehicle",
                                             joint_names=["joint_[1-6]"])},
    )

    # 서스펜션 정렬 패널티
    suspension_misalign = RewTerm(
        func=mdp.suspension_misalignment_penalty,
        weight=-0.2,     # -1.0→-0.2: 복구 자세 중 서스펜션 자유도 허용
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
            "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
            "RL_Rocker_Revolute", "RR_Rocker_Revolute",
        ])},
    )

    # steer 중립 패널티 (upright 후에만)
    steer_misalign = RewTerm(
        func=mdp.steer_misalignment_penalty,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("vehicle",
                                             joint_names=[".*Steer_Revolute"])},
    )

    # 과도한 각속도 패널티 (폭발적 회전 방지)
    ang_vel_penalty = RewTerm(
        func=mdp.angular_velocity_penalty,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold": 5.0},
    )

    # 연속 action 변화량 패널티 (급격한 동작 → explosion 방지)
    action_rate = RewTerm(
        func=mdp.action_rate_penalty,
        weight=-0.01,
    )

    # 전진 준비 보상 (기립 후 바퀴 구동 가능성)
    forward_ready = RewTerm(
        func=mdp.forward_ready_reward,
        weight=3.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "upright_threshold_deg": 20.0},
    )


# ── Terminations ──────────────────────────────────────────────────────────────
@configclass
class TerminationsCfg:
    # 타임아웃
    time_out = DoneTerm(func=loco_mdp.time_out, time_out=True)

    # 성공: stable_frames >= required
    rover_upright = DoneTerm(
        func=mdp.stable_upright_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0,
                "required": STABLE_FRAMES_REQUIRED},
    )

    # 시뮬레이션 폭발 (속도/위치 이상)
    simulation_explosion = DoneTerm(
        func=mdp.simulation_explosion_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "max_lin_vel": 20.0,
                "max_ang_vel": 30.0},
    )

    # 차량 소실 (지면 아래 또는 허공으로 날아감)
    vehicle_lost = DoneTerm(
        func=mdp.vehicle_lost_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "min_z": -5.0, "max_z": 10.0},
    )


# ── 최종 환경 설정 ────────────────────────────────────────────────────────────
@configclass
class RoverRecoveryEnvCfg(ManagerBasedRLEnvCfg):
    scene:        RecoverySceneCfg = RecoverySceneCfg(num_envs=256, env_spacing=8.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions:      ActionsCfg      = ActionsCfg()
    events:       EventCfg        = EventCfg()
    rewards:      RewardsCfg      = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        # ── Physics 주파수 (핵심 throughput 결정 인자) ──────────────────────────
        # 100 Hz 물리 / 2 = 50 Hz 정책  (이전: 200Hz/4 → 2x 물리 연산 감소)
        self.decimation       = 2
        self.episode_length_s = 15.0   # 10→15s: 복구 동작에 더 많은 시간 허용
        self.sim.dt           = 0.01         # 100 Hz (was 0.005 = 200 Hz)
        # headless 학습 중 render 호출 최소화
        # render_interval: physics step 기준, 20 steps = 10 policy step마다 1회
        self.sim.render_interval = 20

        self.sim.gravity      = (0.0, 0.0, -3.72)   # 화성 중력

        # ── PhysX 설정 ─────────────────────────────────────────────────────────
        self.sim.physx.solver_type              = 1      # TGS
        self.sim.physx.enable_ccd               = False  # GPU 미지원
        self.sim.physx.enable_stabilization     = False  # 오버헤드 제거 (was True)
        self.sim.physx.bounce_threshold_velocity     = 0.5   # 0.2→0.5: 불필요 bounce sim 감소
        self.sim.physx.friction_offset_threshold     = 0.04
        # GPU 버퍼: 128 env 기준
        # 128 env × 16 body × ~20 contacts = ~40k → 262144으로 충분
        self.sim.physx.gpu_max_rigid_contact_count   = 262144
        self.sim.physx.gpu_max_rigid_patch_count     = 131072
