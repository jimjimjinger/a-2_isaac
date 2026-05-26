# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""통합 차량 vehicle_v1 의 ArticulationCfg (장애물 회피 RL 태스크용).

vehicle_v1.usd = Mars_Rover 베이스 + m0609 팔 + RG2-FT 그리퍼 → 단일
articulation (27 DOF). 주행 RL 에서는 로버 부분만 제어하고, 팔·그리퍼는
HOME 자세로 고정(freeze)한다.

조인트 명명 규칙:
  - 조향:  ``FL/FR/RL/RR`` + ``_Steer_Revolute``   — position 제어 (4)
  - 구동:  6개 휠 + ``_Drive_Continuous``           — velocity 제어 (6)
  - 패시브: ``_Rocker_Revolute``, ``Differential_Revolute``  (로커 서스펜션, 무동력, 5)
  - 팔:    m0609 ``joint_1``~``joint_6``            — HOME 자세 고정 (6)
  - 그리퍼: RG2-FT ``*finger*`` / ``*knuckle*``      — USD baked 게인 유지 (6)

articulation root 는 ``/Root/Vehicle/m0609/base_link`` — Isaac Lab 이 spawn
prim 하위에서 자동 탐색하므로 prim_path 는 ``.../Robot`` 그대로 둔다.
"""

from __future__ import annotations

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# repo 루트 기준 공유 차량 USD 경로.
# rover.py = a2_isaac/isaac_drive/avoid_test/rover_avoid/rover.py → parents[3] = a2_isaac
_REPO_ROOT = Path(__file__).resolve().parents[3]
ROVER_USD_PATH = str(_REPO_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v1.usd")

# Ackermann 기하 파라미터 (RLRoverLab aau_rover_simple) — avoid_env_cfg 의
# AckermannActionCfg 에서도 동일 값을 쓴다.
WHEELBASE_LENGTH = 0.849
MIDDLE_WHEEL_DISTANCE = 0.894
FRONT_REAR_DISTANCE = 0.77
WHEEL_RADIUS = 0.1
ACK_OFFSET = -0.0135

# 하향 RayCaster 와 ContactSensor 가 부착되는 로버 몸체 링크.
# vehicle_v1 은 rover Body 가 /Root/Vehicle/rover/Body 로 중첩돼 있으므로,
# spawn prim_path("{ENV}/Robot") 기준 상대 경로로 지정한다.
BODY_LINK_NAME = "Vehicle/rover/Body"

# m0609 팔 HOME 자세 (build_integrated_vehicle.py 의 M0609_HOME_DEG).
# 주행 RL 에서는 팔을 쓰지 않으므로 이 자세로 고정한다.
M0609_HOME = {
    "joint_1": 0.0, "joint_2": 0.0, "joint_3": math.radians(90.0),
    "joint_4": 0.0, "joint_5": math.radians(90.0), "joint_6": 0.0,
}


ROVER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ROVER_USD_PATH,
        # ContactSensor 가 동작하려면 반드시 True.
        activate_contact_sensors=True,
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.04,
            rest_offset=0.01,
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_linear_velocity=1.5,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
            disable_gravity=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # env 원점 대비 위치. z 를 살짝 띄워 지형 위로 안전하게 안착시킨다.
        pos=(0.0, 0.0, 0.4),
        joint_pos={".*Steer_Revolute": 0.0, **M0609_HOME},
        joint_vel={".*Steer_Revolute": 0.0, ".*Drive_Continuous": 0.0},
    ),
    actuators={
        # 조향 — 위치 제어 (높은 stiffness).
        "steering": ImplicitActuatorCfg(
            joint_names_expr=[".*Steer_Revolute"],
            velocity_limit_sim=6.0,
            effort_limit_sim=12.0,
            stiffness=8000.0,
            damping=1000.0,
        ),
        # 구동 — 속도 제어 (낮은 stiffness, 높은 damping).
        "drive": ImplicitActuatorCfg(
            joint_names_expr=[".*Drive_Continuous"],
            velocity_limit_sim=6.0,
            effort_limit_sim=12.0,
            stiffness=100.0,
            damping=4000.0,
        ),
        # 로커 서스펜션 + 디퍼렌셜 — 무동력 패시브 조인트.
        "passive": ImplicitActuatorCfg(
            joint_names_expr=[".*Rocker_Revolute", "Differential_Revolute"],
            velocity_limit_sim=15.0,
            effort_limit_sim=0.0,
            stiffness=0.0,
            damping=0.0,
        ),
        # m0609 팔 — 주행 태스크에선 쓰지 않음. HOME 자세로 강하게 고정(freeze).
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            velocity_limit_sim=10.0,
            effort_limit_sim=1000.0,
            stiffness=10000.0,
            damping=1000.0,
        ),
        # RG2-FT 그리퍼 — 주행 태스크에선 안 씀. 현재 자세로 고정(freeze).
        # Isaac Lab 은 ImplicitActuatorCfg 에 stiffness/damping 을 필수로 요구하므로
        # (생략 시 MISSING 검증 에러) 명시한다. 손가락은 가벼워 게인이 작아도 충분.
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*(finger|knuckle).*"],
            velocity_limit_sim=10.0,
            effort_limit_sim=100.0,
            stiffness=1000.0,
            damping=100.0,
        ),
    },
)


# --- m0609 팔 HOME 고정 ---------------------------------------------------
# 액추에이터 stiffness 만으로는 팔이 HOME 을 못 버틴다 — Isaac Lab 이 팔
# 조인트의 위치 타깃을 0 으로 깔아두므로(USD 의 접힘 드라이브 타깃 무시),
# stiffness 가 커도 팔을 0(펴진 자세)으로 끌어당기거나 흐트러진다.
# 그래서 매 step 아래 함수로 팔을 접힌 HOME 에 직접 고정한다 (VehicleAvoidEnv).
_ARM_JOINT_EXPR = ["joint_[1-6]"]
_arm_joint_ids = None


def keep_arm_folded(robot) -> None:
    """m0609 팔을 HOME(접힌 자세)로 고정한다.

    매 step
      · 팔 위치 '타깃' 을 HOME(default_joint_pos)으로
      · 팔 조인트 '상태' 를 HOME 으로 (물리 드리프트를 매 step 되돌림)
    둘 다 해서 팔을 접힌 채 고정한다. 회피 학습에서 팔은 움직일 필요가
    없으므로 사실상 고정해도 무방하다. VehicleAvoidEnv 가 매 step·reset
    직후 호출한다.

    Args:
        robot: env.scene["robot"] (Articulation).
    """
    global _arm_joint_ids
    if _arm_joint_ids is None:
        _arm_joint_ids, _ = robot.find_joints(_ARM_JOINT_EXPR)
    home = robot.data.default_joint_pos[:, _arm_joint_ids]
    zero_vel = robot.data.default_joint_vel[:, _arm_joint_ids]
    robot.set_joint_position_target(home, joint_ids=_arm_joint_ids)
    robot.write_joint_state_to_sim(home, zero_vel, joint_ids=_arm_joint_ids)
