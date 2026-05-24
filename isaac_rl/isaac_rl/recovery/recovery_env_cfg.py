"""rover_recovery_env_cfg.py — Isaac Lab ManagerBasedRLEnv 설정.

씬 구성:
  - Rover (vehicle_v3.usd, 6 drive + 4 steer 관절) : 넘어진 상태로 랜덤 초기화
  - M0609 arm              : Rover 옆 고정 베이스
  - 평지 ground plane

Observation (dim=32):
  rover_roll, rover_pitch, rover_yaw         (3)
  rover_pos_z                                (1)
  rover_lin_vel_x, y, z                      (3)
  rover_ang_vel_x, y, z                      (3)
  m0609_joint_pos   joint_1~6               (6)
  m0609_joint_vel   joint_1~6               (6)
  rover_drive_vel   6 drive wheels           (6)
  rover_steer_pos   4 steer joints           (4)

Action (dim=12):
  m0609 joint position targets  (6)
  rover drive wheel velocities  (6)
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
M0609_USD  = f"{_REPO}/isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim/m0609_isaac_sim.usd"
ROVER_USD  = f"{_REPO}/isaac_sim/assets/vehicle/vehicle_v3.usd"

ARM_ACTION_SCALE   = 0.5    # ±0.5 rad delta per step
WHEEL_ACTION_SCALE = 15.0   # rad/s for drive wheels


# ── M0609 ArticulationCfg ───────────────────────────────────────────────────
M0609_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=M0609_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(1.2, 0.0, 0.0),
        joint_pos={
            "joint_1": 0.0,
            "joint_2": -0.5,
            "joint_3": 1.2,
            "joint_4": 0.0,
            "joint_5": 0.5,
            "joint_6": 0.0,
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
    },
)


# ── Rover ArticulationCfg (vehicle_v3.usd — 실제 바퀴 관절 포함) ────────────
ROVER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ROVER_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            ".*Drive_Continuous": 0.0,
            ".*Steer_Revolute":   0.0,
        },
    ),
    actuators={
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
            stiffness=100.0,
            damping=10.0,
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
    rover: ArticulationCfg = ROVER_CFG.replace(prim_path="{ENV_REGEX_NS}/Rover")
    m0609: ArticulationCfg = M0609_CFG.replace(prim_path="{ENV_REGEX_NS}/M0609")


# ── Observations ────────────────────────────────────────────────────────────
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # rover 자세
        rover_roll    = ObsTerm(func=mdp.rover_roll)
        rover_pitch   = ObsTerm(func=mdp.rover_pitch)
        rover_yaw     = ObsTerm(func=mdp.rover_yaw)
        rover_pos_z   = ObsTerm(func=mdp.rover_pos_z)
        rover_lin_vel = ObsTerm(func=mdp.rover_lin_vel)
        rover_ang_vel = ObsTerm(func=mdp.rover_ang_vel)
        # M0609 상태
        arm_joint_pos = ObsTerm(func=mdp.arm_joint_pos,
                                params={"asset_cfg": SceneEntityCfg("m0609")})
        arm_joint_vel = ObsTerm(func=mdp.arm_joint_vel,
                                params={"asset_cfg": SceneEntityCfg("m0609")})
        # Rover 바퀴 상태
        rover_drive_vel = ObsTerm(
            func=mdp.rover_drive_vel,
            params={"asset_cfg": SceneEntityCfg("rover",
                                                joint_names=[".*Drive_Continuous"])},
        )
        rover_steer_pos = ObsTerm(
            func=mdp.rover_steer_pos,
            params={"asset_cfg": SceneEntityCfg("rover",
                                                joint_names=[".*Steer_Revolute"])},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ── Actions ─────────────────────────────────────────────────────────────────
@configclass
class ActionsCfg:
    arm_action = loco_mdp.JointPositionActionCfg(
        asset_name="m0609",
        joint_names=["joint_[1-6]"],
        scale=ARM_ACTION_SCALE,
        use_default_offset=True,
    )
    wheel_action = loco_mdp.JointVelocityActionCfg(
        asset_name="rover",
        joint_names=[".*Drive_Continuous"],
        scale=WHEEL_ACTION_SCALE,
    )


# ── Events (초기화 랜덤화) ───────────────────────────────────────────────────
@configclass
class EventCfg:
    reset_rover_fallen = EventTerm(
        func=mdp.reset_rover_fallen,
        mode="reset",
        params={
            "asset_cfg":   SceneEntityCfg("rover"),
            "roll_range":  (math.radians(60), math.radians(120)),
            "pitch_range": (-math.radians(30), math.radians(30)),
        },
    )
    reset_arm_default = EventTerm(
        func=mdp.reset_arm_default,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("m0609")},
    )


# ── Rewards ─────────────────────────────────────────────────────────────────
@configclass
class RewardsCfg:
    # ── 주 보상: cos 기반 자세 보상 (upright에 가까울수록 강하게) ──────────────
    upright_cosine = RewTerm(
        func=mdp.upright_cosine_reward,
        weight=10.0,
        params={"asset_cfg": SceneEntityCfg("rover")},
    )
    # ── 높이 보상 (몸체가 올라올수록 보상) ─────────────────────────────────────
    height_reward = RewTerm(
        func=mdp.height_reward,
        weight=5.0,
        params={"asset_cfg": SceneEntityCfg("rover"),
                "fallen_z": 0.30, "upright_z": 0.60},
    )
    # ── 성공 보너스 (sparse, 매우 큼) ──────────────────────────────────────────
    success_bonus = RewTerm(
        func=mdp.success_bonus,
        weight=500.0,
        params={"asset_cfg": SceneEntityCfg("rover"),
                "threshold_deg": 15.0},
    )
    # ── 넘어진 상태 페널티 (tilt > 75° 지속 시 강한 패널티) ─────────────────────
    fallen_penalty = RewTerm(
        func=mdp.fallen_penalty,
        weight=-3.0,
        params={"asset_cfg": SceneEntityCfg("rover"),
                "tilt_threshold_deg": 75.0},
    )
    # ── 시간 패널티 (빠른 기립 유도) ───────────────────────────────────────────
    time_penalty = RewTerm(
        func=mdp.time_alive_penalty,
        weight=-0.2,
    )
    # ── 넘어진 상태에서 바퀴 회전 유도 ─────────────────────────────────────────
    wheel_drive_bonus = RewTerm(
        func=mdp.wheel_drive_reward,
        weight=2.0,
        params={
            "rover_cfg":          SceneEntityCfg("rover",
                                                 joint_names=[".*Drive_Continuous"]),
            "tilt_threshold_deg": 45.0,
        },
    )
    # ── 팔 관절 속도 패널티 (부드러운 동작 유도) ────────────────────────────────
    arm_vel_penalty = RewTerm(
        func=mdp.joint_vel_penalty,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("m0609")},
    )
    # ── 팔 관절 한계 페널티 ─────────────────────────────────────────────────────
    joint_limit_penalty = RewTerm(
        func=mdp.joint_limit_penalty,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("m0609")},
    )


# ── Terminations ─────────────────────────────────────────────────────────────
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=loco_mdp.time_out, time_out=True)
    rover_upright = DoneTerm(
        func=mdp.rover_upright_termination,
        params={"asset_cfg": SceneEntityCfg("rover"),
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
        self.episode_length_s = 15.0  # 15초 (750 step) — 기립에 충분한 시간
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -3.72)  # 화성 중력
