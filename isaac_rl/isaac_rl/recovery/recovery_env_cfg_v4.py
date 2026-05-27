"""recovery_env_cfg_v4.py — Isaac Lab ManagerBasedRLEnv 설정 (v4).

변경 사항 (v3 → v4):
  - Action: arm(6) + wheel_drive(6) = 12dim  (steering 제외)
  - Obs:    31 + 12 = 43dim
              신규: base_forward_dir(2), forward_vel(1), forward_distance(1),
                    recovered_flag(1), mission_resume_flag(1), wheel_velocities(6)
  - Reward: recovery dense 유지 (scale 소폭 조정) +
            rocking pre-recovery + forward phase (recovered gate) +
            self_collision_penalty
  - Success: recovered → forward_distance >= 2m (mission_success_termination)
  - Episode: 20s (15s → 20s)

Observation (dim=43):
  upright_vec          3
  rover_roll           1
  rover_pitch          1
  rover_pos_z          1
  rover_ang_vel        3
  arm_joint_pos        6
  arm_joint_vel        6
  arm_ee_pos           3
  wheel_contact        1
  rocker_joint_pos     5
  stable_frames        1  ← v4_state_update_obs 이전에 배치
  ─────────────────── 31 (v3 동일)
  recovered_flag       1  ← side-effect: v4 state 갱신
  mission_resume_flag  1
  base_forward_dir     2
  forward_vel          1
  forward_distance     1
  wheel_velocities     6
  ─────────────────── +12 = 43

Action (dim=12):
  joint_1~6           6  — arm position targets (Δ±0.3 rad)
  Drive_Continuous x6 6  — wheel velocity targets (±8 rad/s)
"""
from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.events import randomize_rigid_body_material
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as loco_mdp

import recovery_mdp_v4 as mdp
from recovery_env_cfg import (
    STABLE_FRAMES_REQUIRED,
    RecoverySceneCfg,
    RoverRecoveryEnvCfg,
)
from mars_terrain_cfg import get_v4_curriculum_terrain_cfg

WHEEL_ACTION_SCALE = 8.0   # ±8 rad/s → r=0.1m 기준 ±0.8 m/s
# default: stage 3 regression, stage 4 only for final fine-tune
CURRICULUM_STAGE   = int(os.getenv("ROVER_RECOVERY_STAGE", "3"))
FORWARD_GOAL_M     = float(os.getenv("ROVER_FORWARD_GOAL_M", "1.5"))
CURRICULUM_TERRAIN_CFG = get_v4_curriculum_terrain_cfg(CURRICULUM_STAGE)


# ── Actions (dim=12) ──────────────────────────────────────────────────────────
@configclass
class ActionsCfgV4:
    arm_action = loco_mdp.JointPositionActionCfg(
        asset_name="vehicle",
        joint_names=["joint_[1-6]"],
        scale=0.3,
        use_default_offset=True,
    )
    wheel_action = loco_mdp.JointVelocityActionCfg(
        asset_name="vehicle",
        joint_names=[".*Drive_Continuous"],
        scale=WHEEL_ACTION_SCALE,
    )


# ── Scene (v4) ────────────────────────────────────────────────────────────────
@configclass
class RecoverySceneCfgV4(RecoverySceneCfg):
    terrain = CURRICULUM_TERRAIN_CFG.replace(prim_path="/World/Terrain")

    # drive wheels only
    contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Vehicle/Vehicle/rover/(FL_Drive|FR_Drive|CL_Drive|CR_Drive|RL_Drive|RR_Drive)",
        history_length=1,
        filter_prim_paths_expr=["/World/Terrain"],
    )

    # arm vs chassis contact only
    self_collision_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Vehicle/Vehicle/m0609/(base_link|base|link_1|link_2|link_3|link_4|link_5|link_6|tool0)",
        history_length=1,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Vehicle/Vehicle/rover/Body"],
    )

    # arm tool vs terrain contact
    arm_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Vehicle/Vehicle/m0609/tool0",
        history_length=1,
        filter_prim_paths_expr=["/World/Terrain"],
    )


# ── Observations (dim=43) ─────────────────────────────────────────────────────
@configclass
class ObservationsCfgV4:
    @configclass
    class PolicyCfg(ObsGroup):
        # ── v3 기존 31차원 ────────────────────────────────────────────────
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
        arm_ee_pos = ObsTerm(
            func=mdp.arm_ee_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", body_names=["tool0"])},
        )
        wheel_contact = ObsTerm(
            func=mdp.wheel_contact_obs,
            params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
        )
        rocker_joint_pos = ObsTerm(
            func=mdp.rocker_joint_pos,
            params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=[
                "FR_Rocker_Revolute", "FL_Rocker_Revolute", "Differential_Revolute",
                "RL_Rocker_Revolute", "RR_Rocker_Revolute",
            ])},
        )
        # stable_frames 는 v4_state_update_obs 이전에 반드시 배치
        stable_frames = ObsTerm(
            func=mdp.stable_frames_normalized,
            params={"required": STABLE_FRAMES_REQUIRED},
        )

        # ── v4 신규 12차원 ────────────────────────────────────────────────
        # recovered_flag: side-effect로 _v4 state 전체 갱신
        recovered_flag = ObsTerm(
            func=mdp.v4_state_update_obs,
            params={
                "asset_cfg":      SceneEntityCfg("vehicle"),
                "required_stable": STABLE_FRAMES_REQUIRED,
                "forward_goal_m":  FORWARD_GOAL_M,
                "threshold_deg":   20.0,
            },
        )
        mission_resume_flag = ObsTerm(func=mdp.mission_resume_flag_obs)
        base_forward_dir = ObsTerm(
            func=mdp.base_forward_dir_obs,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        forward_vel = ObsTerm(
            func=mdp.forward_vel_obs,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        forward_distance = ObsTerm(
            func=mdp.forward_distance_obs,
            params={"asset_cfg": SceneEntityCfg("vehicle")},
        )
        # wheel_velocities: joint_names 로 drive wheel 필터
        wheel_velocities = ObsTerm(
            func=mdp.wheel_velocities_obs,
            params={"asset_cfg": SceneEntityCfg(
                "vehicle", joint_names=[".*Drive_Continuous"]
            )},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# ── Events ────────────────────────────────────────────────────────────────────
@configclass
class EventCfgV4:
    # v4 reset: v3 전복 + v4 state 초기화
    reset_vehicle = EventTerm(
        func=mdp.reset_vehicle_random_fall_v4,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("vehicle"),
            "height": 0.5,
        },
    )
    randomize_arm_pose = EventTerm(
        func=mdp.randomize_arm_pose,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"]),
            "pos_range": (-0.3, 0.3),
        },
    )
    randomize_physics = EventTerm(
        func=mdp.randomize_physics_params,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
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


# ── Rewards ───────────────────────────────────────────────────────────────────
@configclass
class RewardsCfgV4:
    # ── Recovery dense (v3 계승, scale 소폭 조정) ────────────────────────────
    upright_cosine = RewTerm(
        func=mdp.upright_cosine_reward, weight=6.0,
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
    near_success = RewTerm(
        func=mdp.near_success_reward, weight=8.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "sigma_deg": 45.0},
    )
    height_reward = RewTerm(
        func=mdp.height_reward, weight=2.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "fallen_z": 0.2, "upright_z": 0.7},
    )
    recovery_ang_vel = RewTerm(
        func=mdp.recovery_angular_vel_reward, weight=2.0,
        params={"asset_cfg": SceneEntityCfg("vehicle")},
    )
    stable_upright = RewTerm(
        func=mdp.stable_upright_reward, weight=8.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 20.0, "required": STABLE_FRAMES_REQUIRED},
    )
    arm_recovery = RewTerm(
        func=mdp.arm_ground_push_reward, weight=5.0,
        params={"asset_cfg": SceneEntityCfg("vehicle", body_names=["tool0"])},
    )
    wheel_contact = RewTerm(
        func=mdp.wheel_contact_reward, weight=6.0,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor")},
    )
    rocking_motion = RewTerm(
        func=mdp.rocking_motion_reward, weight=6.0,
        params={
            "asset_cfg": SceneEntityCfg("vehicle"),
            "sensor_cfg": SceneEntityCfg("arm_contact_sensor"),
            "min_tilt_deg": 20.0,
            "max_roll_rate": 2.0,
        },
    )
    rocking_oscillation_penalty = RewTerm(
        func=mdp.rocking_oscillation_penalty, weight=-4.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "max_roll_rate": 1.5},
    )

    # ── Forward phase (recovered flag로 gate) ────────────────────────────────
    forward_velocity = RewTerm(
        func=mdp.forward_velocity_reward, weight=18.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "max_vel": 0.8},
    )
    forward_progress = RewTerm(
        func=mdp.forward_progress_reward, weight=20.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "goal_m": FORWARD_GOAL_M},
    )
    forward_ready = RewTerm(
        func=mdp.forward_ready_reward_v4, weight=8.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "threshold_deg": 20.0},
    )

    # ── Sparse bonus ─────────────────────────────────────────────────────────
    recovery_bonus = RewTerm(                              # 중간 목표: 기립
        func=mdp.stable_success_bonus, weight=30.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"),
                "threshold_deg": 20.0, "required": STABLE_FRAMES_REQUIRED},
    )
    mission_resume_bonus = RewTerm(                        # 최종 목표: 전진
        func=mdp.mission_resume_success_bonus, weight=200.0,
    )

    # ── Penalty ───────────────────────────────────────────────────────────────
    fallen_penalty = RewTerm(
        func=mdp.fallen_penalty, weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "tilt_threshold_deg": 75.0},
    )
    time_penalty = RewTerm(
        func=mdp.time_alive_penalty, weight=-0.02,
    )
    joint_limit_penalty = RewTerm(
        func=mdp.joint_limit_penalty, weight=-1.5,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
    )
    arm_vel_penalty = RewTerm(
        func=mdp.arm_vel_penalty_conditional, weight=-0.003,
        params={"asset_cfg": SceneEntityCfg("vehicle", joint_names=["joint_[1-6]"])},
    )
    ang_vel_penalty = RewTerm(
        func=mdp.angular_velocity_penalty, weight=-0.025,
        params={"asset_cfg": SceneEntityCfg("vehicle"), "threshold": 5.0},
    )
    action_rate = RewTerm(
        func=mdp.action_rate_penalty, weight=-0.005,
    )
    self_collision = RewTerm(
        func=mdp.self_collision_penalty, weight=-100.0,
        params={"sensor_cfg": SceneEntityCfg("self_collision_sensor")},
    )


# ── Terminations ──────────────────────────────────────────────────────────────
@configclass
class TerminationsCfgV4:
    time_out = DoneTerm(func=loco_mdp.time_out, time_out=True)

    # v4 primary success: recovered → forward 2m
    mission_success = DoneTerm(
        func=mdp.mission_success_termination,
        params={"forward_goal_m": FORWARD_GOAL_M},
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
class RoverRecoveryEnvCfgV4(RoverRecoveryEnvCfg):
    """v3 기반 v4: action 12dim, obs 43dim, forward 2m 성공 조건."""

    actions:      ActionsCfgV4      = ActionsCfgV4()
    observations: ObservationsCfgV4 = ObservationsCfgV4()
    events:       EventCfgV4        = EventCfgV4()
    rewards:      RewardsCfgV4      = RewardsCfgV4()
    terminations: TerminationsCfgV4 = TerminationsCfgV4()
    scene:        RecoverySceneCfgV4 = RecoverySceneCfgV4(num_envs=256, env_spacing=8.0)

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 20.0            # 15 → 20 (복원 + 전진 시간)
        # gpu contact slot 증가 (wheel velocity action으로 contact 증가)
        self.sim.physx.gpu_max_rigid_contact_count = 786432
        self.sim.physx.gpu_max_rigid_patch_count   = 393216
