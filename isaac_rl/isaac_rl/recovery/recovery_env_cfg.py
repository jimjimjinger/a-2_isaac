"""recovery_env_cfg.py — Isaac Lab ManagerBasedRLEnv 설정.

씬 구성:
  - Vehicle (vehicle_v3.usd) : rover 바퀴 + rover에 부착된 m0609 arm이 하나의 단일 articulation
    ArticulationRoot: /Root/Vehicle/m0609/base_link (m0609 베이스, rover body에 고정됨)
  - 평지 ground plane

Observation (dim=37):
  vehicle_roll, vehicle_pitch, vehicle_yaw    (3)
  vehicle_pos_z                               (1)
  vehicle_lin_vel_x, y, z                    (3)
  vehicle_ang_vel_x, y, z                    (3)
  arm_joint_pos   joint_1~6                  (6)
  arm_joint_vel   joint_1~6                  (6)
  drive_vel       6 drive wheels             (6)
  steer_pos       4 steer joints             (4)
  rocker_pos      5 suspension joints        (5)  ← 신규

Action (dim=16):
  m0609 joint position targets  (6)
  rover drive wheel velocities  (6)
  rover steer position targets  (4)
"""
from __future__ import annotations

import math

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

# ── 에셋 경로 ──────────────────────────────────────────────────────────────
_REPO      = "/home/kimi/dev_ws/rover_ws/src/a2_isaac"
VEHICLE_USD = f"{_REPO}/isaac_sim/assets/vehicle/vehicle_v3.usd"

ARM_ACTION_SCALE   = 0.5    # ±0.5 rad delta per step
WHEEL_ACTION_SCALE = 15.0   # rad/s for drive wheels
STEER_ACTION_SCALE = 0.3    # ±0.3 rad (≈±17°) delta per step


# ── Vehicle ArticulationCfg (rover + m0609 + gripper 단일 articulation) ────
# ArticulationRoot: /Root/Vehicle/m0609/base_link  (rover body에 고정된 arm 베이스)
VEHICLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=VEHICLE_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
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
            stiffness=8000.0,   # 100→8000: USD 설계값 일치, 복원 중 steer 흔들림 방지
            damping=100.0,      # 10→100: 빠른 수렴으로 정렬 안정화
        ),
    },
)


# ── Scene ───────────────────────────────────────────────────────────────────
@configclass
class RecoverySceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/SkyLight",
        spawn=sim_utils.DomeLightCfg(intensity=500.0,
                                     color=(0.95, 0.73, 0.57)),
    )
    # rover + m0609 arm이 결합된 단일 articulation
    vehicle: ArticulationCfg = VEHICLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Vehicle")


# ── Observations ────────────────────────────────────────────────────────────
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # vehicle(rover) 자세 — articulation root = m0609 base_link (rover body에 고정)
        rover_roll    = ObsTerm(func=mdp.rover_roll,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_pitch   = ObsTerm(func=mdp.rover_pitch,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_yaw     = ObsTerm(func=mdp.rover_yaw,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_pos_z   = ObsTerm(func=mdp.rover_pos_z,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_lin_vel = ObsTerm(func=mdp.rover_lin_vel,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        rover_ang_vel = ObsTerm(func=mdp.rover_ang_vel,
                                params={"asset_cfg": SceneEntityCfg("vehicle")})
        # arm 관절 상태
        arm_joint_pos = ObsTerm(
            func=mdp.arm_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
        )
        arm_joint_vel = ObsTerm(
            func=mdp.arm_joint_vel,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
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
        # 서스펜션(rocker-bogie) 관절 위치 — 정렬 상태 인식용
        rocker_joint_pos = ObsTerm(
            func=mdp.rocker_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
                "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
                "RL_Rocker_Revolute", "RR_Rocker_Revolute",
            ])},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ── Actions ─────────────────────────────────────────────────────────────────
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


# ── Events (초기화 랜덤화) ───────────────────────────────────────────────────
@configclass
class EventCfg:
    reset_vehicle_fallen = EventTerm(
        func=mdp.reset_vehicle_fallen,
        mode="reset",
        params={
            "asset_cfg":   SceneEntityCfg("vehicle"),
            "roll_range":  (math.radians(60), math.radians(120)),
            "pitch_range": (-math.radians(30), math.radians(30)),
        },
    )


# ── Rewards ─────────────────────────────────────────────────────────────────
@configclass
class RewardsCfg:
    # cos 기반 자세 보상 — 매 스텝 기립 방향 gradient 제공
    upright_cosine = RewTerm(
        func=mdp.upright_cosine_reward,
        weight=20.0,            # 15→20
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
    # 높이 보상
    height_reward = RewTerm(
        func=mdp.height_reward,
        weight=8.0,             # 5→8
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "fallen_z": 0.2, "upright_z": 0.7},
    )
    # 성공 보너스 — 완전 기립만 목표 (near_success 제거로 해킹 경로 차단)
    success_bonus = RewTerm(
        func=mdp.success_bonus,
        weight=500.0,           # 400→500
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0},
    )
    # 넘어진 상태 페널티 — 강화해서 기립 동기 부여
    fallen_penalty = RewTerm(
        func=mdp.fallen_penalty,
        weight=-5.0,            # -3→-5
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "tilt_threshold_deg": 75.0},
    )
    # 시간 패널티 — 빠른 복원 유도
    time_penalty = RewTerm(
        func=mdp.time_alive_penalty,
        weight=-0.3,            # -0.2→-0.3
    )
    # arm 관절 속도 패널티
    arm_vel_penalty = RewTerm(
        func=mdp.joint_vel_penalty,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
    )
    # arm 관절 한계 패널티
    joint_limit_penalty = RewTerm(
        func=mdp.joint_limit_penalty,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
    )
    # 서스펜션 정렬 패널티 — rocker-bogie 관절이 중립(0°)에서 벗어나면 패널티
    suspension_misalign = RewTerm(
        func=mdp.suspension_misalignment_penalty,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
            "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
            "RL_Rocker_Revolute", "RR_Rocker_Revolute",
        ])},
    )
    # steer 중립 패널티 — upright 상태에서 steer가 0°에서 벗어나면 패널티
    steer_misalign = RewTerm(
        func=mdp.steer_misalignment_penalty,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("vehicle",
                                             joint_names=[".*Steer_Revolute"])},
    )
    # near_success  → 제거: 흔들거리며 보상 긁는 해킹 경로
    # recovery_ang_vel → 제거: 실효 기여 0.009로 무효
    # wheel_drive_bonus → 제거: 바퀴만 굴리는 행동 강화 부작용


# ── Terminations ─────────────────────────────────────────────────────────────
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=loco_mdp.time_out, time_out=True)
    rover_upright = DoneTerm(
        func=mdp.rover_upright_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0},
    )


# ── 최종 환경 설정 ───────────────────────────────────────────────────────────
@configclass
class RoverRecoveryEnvCfg(ManagerBasedRLEnvCfg):
    scene:        RecoverySceneCfg = RecoverySceneCfg(num_envs=256, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions:      ActionsCfg      = ActionsCfg()
    events:       EventCfg        = EventCfg()
    rewards:      RewardsCfg      = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 4           # 200Hz 물리 / 4 = 50Hz 정책
        self.episode_length_s = 20.0  # 15→20초: 복원에 더 많은 시간 허용
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -3.72)  # 화성 중력
