"""recovery_env_cfg.py — Isaac Lab ManagerBasedRLEnv 설정 (v3).

변경 사항 (v2 → v3):
  - 초기 상태: 옆(40%) / 뒤집힘(30%) / 비스듬히(30%) 통합 랜덤화
  - 팔 초기 자세 랜덤화, 마찰 계수 랜덤화 추가
  - 행동: M0609 6축만 (바퀴·스티어 제외) → dim 16→6
  - 관측: drive_vel/steer_pos 제거, arm EE위치(3) + 바퀴접촉(1) 추가 → dim 40→31
  - 보상: wheel_contact_reward 추가
  - 성공 종료: upright + 바퀴 접촉 + stable frames

Observation (dim=31):
  upright_vec       3  — local Z 기립 벡터
  rover_roll        1  — roll 자세각
  rover_pitch       1  — pitch 자세각
  rover_pos_z       1  — 높이
  rover_ang_vel     3  — 각속도
  arm_joint_pos     6  — M0609 joint_1~6
  arm_joint_vel     6  — M0609 joint_1~6
  arm_ee_pos        3  — end-effector 상대 위치 (root 기준)
  wheel_contact     1  — 바퀴 접촉 비율 [0,1]
  rocker_joint_pos  5  — 서스펜션 상태 (terrain contact proxy)
  stable_frames     1  — 연속 기립 프레임 (정규화)

Action (dim=6):
  M0609 joint position targets — Δ±0.3 rad/step
"""
from __future__ import annotations

import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.events import randomize_rigid_body_material
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as loco_mdp
import recovery_mdp as mdp
from mars_terrain_cfg import MARS_TERRAIN_CFG

# ── 에셋 경로 ──────────────────────────────────────────────────────────────────
_REPO      = os.path.join(os.path.dirname(__file__), "../../..")
_ASSET_DIR = os.path.join(_REPO, "isaac_sim/assets/vehicle")
_PHYSX_USD = os.path.join(_ASSET_DIR, "vehicle_v3_physx.usd")
_ORIG_USD  = os.path.join(_ASSET_DIR, "vehicle_v3.usd")
VEHICLE_USD = _PHYSX_USD if os.path.exists(_PHYSX_USD) else _ORIG_USD

ARM_ACTION_SCALE       = 0.3
STABLE_FRAMES_REQUIRED = 5


# ── Vehicle ArticulationCfg ────────────────────────────────────────────────────
VEHICLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=VEHICLE_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
            enable_gyroscopic_forces=True,
            max_contact_impulse=500.0,
            max_linear_velocity=10.0,
            max_angular_velocity=100.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.05,
            stabilization_threshold=0.01,
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


# ── Scene ──────────────────────────────────────────────────────────────────────
@configclass
class RecoverySceneCfg(InteractiveSceneCfg):
    terrain = MARS_TERRAIN_CFG.replace(prim_path="/World/Terrain")

    vehicle: ArticulationCfg = VEHICLE_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Vehicle"
    )

    # 바퀴-지형 접촉 감지
    # contact reporter API가 붙은 drive wheel prim만 선택한다.
    contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Vehicle/Vehicle/rover/(FL_Drive|FR_Drive|CL_Drive|CR_Drive|RL_Drive|RR_Drive)",
        history_length=1,
        filter_prim_paths_expr=["/World/Terrain"],
    )


# ── Observations (dim=31) ──────────────────────────────────────────────────────
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        upright_vec = ObsTerm(
            func=mdp.upright_vec,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        rover_roll = ObsTerm(
            func=mdp.rover_roll,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        rover_pitch = ObsTerm(
            func=mdp.rover_pitch,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        rover_pos_z = ObsTerm(
            func=mdp.rover_pos_z,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        rover_ang_vel = ObsTerm(
            func=mdp.rover_ang_vel,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        arm_joint_pos = ObsTerm(
            func=mdp.arm_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
        )
        arm_joint_vel = ObsTerm(
            func=mdp.arm_joint_vel,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
        )
        # end-effector 위치 (root 기준 상대 위치)
        arm_ee_pos = ObsTerm(
            func=mdp.arm_ee_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", body_names=["tool0"])},
        )
        # 바퀴 접촉 비율 [0,1]
        wheel_contact = ObsTerm(
            func=mdp.wheel_contact_obs,
            params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
        )
        # 서스펜션 관절 (terrain contact proxy)
        rocker_joint_pos = ObsTerm(
            func=mdp.rocker_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
                "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
                "RL_Rocker_Revolute", "RR_Rocker_Revolute",
            ])},
        )
        stable_frames = ObsTerm(
            func=mdp.stable_frames_normalized,
            params={"required": STABLE_FRAMES_REQUIRED},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ── Actions (dim=6: 팔만) ──────────────────────────────────────────────────────
@configclass
class ActionsCfg:
    arm_action = loco_mdp.JointPositionActionCfg(
        asset_name="vehicle",
        joint_names=["joint_[1-6]"],
        scale=ARM_ACTION_SCALE,
        use_default_offset=True,
    )


# ── Events ─────────────────────────────────────────────────────────────────────
@configclass
class EventCfg:
    # 통합 전복 reset: 옆(40%) / 뒤집힘(30%) / 비스듬히(30%)
    reset_vehicle = EventTerm(
        func=mdp.reset_vehicle_random_fall,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("vehicle"),
            "height": 0.5,
        },
    )

    # 팔 초기 자세 랜덤화 (reset_vehicle 이후 실행)
    randomize_arm_pose = EventTerm(
        func=mdp.randomize_arm_pose,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"]),
            "pos_range": (-0.3, 0.3),
        },
    )

    # 초기 속도 노이즈
    randomize_physics = EventTerm(
        func=mdp.randomize_physics_params,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )

    # 마찰 계수 랜덤화 (sim-to-real gap 대비)
    randomize_friction = EventTerm(
        func=randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("vehicle"),
            "static_friction_range":  (0.3, 1.2),
            "dynamic_friction_range": (0.3, 1.0),
            "restitution_range":      (0.0, 0.05),
            "num_buckets": 64,
        },
    )


# ── Rewards ─────────────────────────────────────────────────────────────────────
@configclass
class RewardsCfg:
    # ─── Dense ──────────────────────────────────────────────────────────────
    upright_cosine = RewTerm(
        func=mdp.upright_cosine_reward,
        weight=10.0,
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
    near_success = RewTerm(
        func=mdp.near_success_reward,
        weight=20.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "sigma_deg": 45.0},
    )
    height_reward = RewTerm(
        func=mdp.height_reward,
        weight=5.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "fallen_z": 0.2, "upright_z": 0.7},
    )
    recovery_ang_vel = RewTerm(
        func=mdp.recovery_angular_vel_reward,
        weight=5.0,
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
    stable_upright = RewTerm(
        func=mdp.stable_upright_reward,
        weight=20.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0, "required": STABLE_FRAMES_REQUIRED},
    )
    arm_recovery = RewTerm(
        func=mdp.arm_ground_push_reward,
        weight=8.0,
        params={"asset_cfg": SceneEntityCfg("vehicle", body_names=["tool0"])},
    )
    # 바퀴 지면 접촉 복원 보상
    wheel_contact = RewTerm(
        func=mdp.wheel_contact_reward,
        weight=15.0,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
    )

    # ─── Sparse ─────────────────────────────────────────────────────────────
    success_bonus = RewTerm(
        func=mdp.stable_success_bonus,
        weight=200.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 15.0, "required": STABLE_FRAMES_REQUIRED},
    )

    # ─── 패널티 ─────────────────────────────────────────────────────────────
    fallen_penalty = RewTerm(
        func=mdp.fallen_penalty,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "tilt_threshold_deg": 75.0},
    )
    time_penalty = RewTerm(
        func=mdp.time_alive_penalty,
        weight=-0.1,
    )
    arm_vel_penalty = RewTerm(
        func=mdp.arm_vel_penalty_conditional,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
    )
    ang_vel_penalty = RewTerm(
        func=mdp.angular_velocity_penalty,
        weight=-0.02,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "threshold": 5.0},
    )
    action_rate = RewTerm(
        func=mdp.action_rate_penalty,
        weight=-0.01,
    )


# ── Terminations ───────────────────────────────────────────────────────────────
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=loco_mdp.time_out, time_out=True)

    # 성공: 기립 + 바퀴 접촉 + stable frames
    rover_upright = DoneTerm(
        func=mdp.stable_upright_termination,
        params={
            "asset_cfg":   SceneEntityCfg("vehicle"),
            "sensor_cfg":  SceneEntityCfg("contact_sensor"),
            "threshold_deg": 15.0,
            "required":    STABLE_FRAMES_REQUIRED,
        },
    )
    simulation_explosion = DoneTerm(
        func=mdp.simulation_explosion_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "max_lin_vel": 20.0, "max_ang_vel": 30.0},
    )
    vehicle_lost = DoneTerm(
        func=mdp.vehicle_lost_termination,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "min_z": -5.0, "max_z": 10.0},
    )


# ── 최종 환경 설정 ─────────────────────────────────────────────────────────────
@configclass
class RoverRecoveryEnvCfg(ManagerBasedRLEnvCfg):
    scene:        RecoverySceneCfg = RecoverySceneCfg(num_envs=256, env_spacing=8.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions:      ActionsCfg      = ActionsCfg()
    events:       EventCfg        = EventCfg()
    rewards:      RewardsCfg      = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation       = 2
        self.episode_length_s = 15.0
        self.sim.dt           = 0.01
        self.sim.render_interval = 20
        self.sim.gravity      = (0.0, 0.0, -3.72)

        self.sim.physx.solver_type                   = 1
        self.sim.physx.enable_ccd                    = False
        self.sim.physx.enable_stabilization          = False
        self.sim.physx.bounce_threshold_velocity     = 0.5
        self.sim.physx.friction_offset_threshold     = 0.04
        self.sim.physx.gpu_max_rigid_contact_count   = 524288  # ContactSensor 추가로 증가
        self.sim.physx.gpu_max_rigid_patch_count     = 262144
