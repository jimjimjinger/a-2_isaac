# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""통합 차량 vehicle_v1.usd 의 ArticulationCfg (WASD 주행 + 레이캐스트 테스트).

avoid_test/rover_avoid/rover.py 를 vehicle_v1.usd 에 맞춰 옮긴 것.

vehicle_v1.usd 구조 (점검 결과):
  /Root/Vehicle/rover    — 6륜 Mars Rover 베이스  (조인트명 = avoid_test 와 동일)
  /Root/Vehicle/m0609    — M0609 로봇팔   (joint_1~6)
  /Root/Vehicle/onrobot_rg2ft — RG2-FT 그리퍼 (finger/knuckle 조인트 6개)
  → 단일 articulation, 총 27 DOF.

rover 부분 조인트 명명이 Mars_Rover 와 동일하므로 avoid_test 의 조향/구동/
패시브 액추에이터 설정을 그대로 쓰고, 팔·그리퍼 그룹을 추가해 27 DOF 를
모두 커버한다 (Isaac Lab 은 모든 조인트가 액추에이터 그룹에 속하길 요구).

⚠️ vehicle_v1.usd 는 drive_test 밖의 자산 — 경로 참조만, 수정하지 않는다.
"""

from __future__ import annotations

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# drive_test/rover_vehicle.py → parents[2] = a2_isaac (repo 루트)
_REPO_ROOT = Path(__file__).resolve().parents[2]
VEHICLE_USD_PATH = str(_REPO_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v1.usd")

# Ackermann 기하 파라미터 — rover 부분이 Mars_Rover 베이스라 avoid_test 와 동일.
WHEELBASE_LENGTH = 0.849
MIDDLE_WHEEL_DISTANCE = 0.894
FRONT_REAR_DISTANCE = 0.77
WHEEL_RADIUS = 0.1
ACK_OFFSET = -0.0135

# 하향 RayCaster 가 부착되는 rover 몸체 링크.
# USD 내부 경로 /Root/Vehicle/rover/Body — 스폰 prim({ENV}/Robot) 기준 상대경로.
BODY_LINK_NAME = "Vehicle/rover/Body"


VEHICLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=VEHICLE_USD_PATH,
        # ContactSensor 가 body 의 contact 력을 읽으려면 spawn 시 prim 에 contact
        # reporter API 가 켜져야 한다 — UsdFileCfg(SpawnerCfg) 의 플래그.
        # drive_test 는 ContactSensor 안 써서 필요 없었음.
        activate_contact_sensors=True,
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.04,
            rest_offset=0.01,
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # 빠른 주행용 — 차체 선속도 상한 (1.5 → 3.0 → 4.5).
            max_linear_velocity=4.5,
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
        # env 원점 = 맵 중앙 베이스캠프(평지, 표고 ~0.11m). 그 위로 z 를
        # 살짝 띄워 스폰 → 중력으로 지형 위에 안착.
        pos=(0.0, 0.0, 0.7),
        joint_pos={
            ".*Steer_Revolute": 0.0,
            # M0609 HOME 자세 — README (0,0,90,0,90,0) deg.
            "joint_3": math.radians(90.0),
            "joint_5": math.radians(90.0),
        },
        joint_vel={".*Steer_Revolute": 0.0, ".*Drive_Continuous": 0.0},
    ),
    actuators={
        # --- rover (avoid_test rover.py 와 동일) ---
        # 조향 — 위치 제어 (높은 stiffness).
        "steering": ImplicitActuatorCfg(
            joint_names_expr=[".*Steer_Revolute"],
            velocity_limit_sim=6.0,
            effort_limit_sim=12.0,
            stiffness=8000.0,
            damping=1000.0,
        ),
        # 구동 — 속도 제어 (낮은 stiffness, 높은 damping).
        # velocity_limit_sim 이 빠른 주행의 실제 상한: 바퀴 ω = v / r 이라
        # LIN_SPEED 3.0 m/s → 휠 30 rad/s 가 필요해 25.0 → 38.0 으로 상향.
        "drive": ImplicitActuatorCfg(
            joint_names_expr=[".*Drive_Continuous"],
            velocity_limit_sim=38.0,
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
        # --- m0609 팔 ---
        # 접힌 HOME 자세를 단단히 유지하도록 명시 게인을 준다.
        # stiffness=None(USD 값)으로 두면 USD m0609 조인트 드라이브가 약해
        # 시뮬레이션 중 팔이 풀려 펴진다 → 위치제어로 HOME 에 고정한다.
        # init_state.joint_pos(joint_3=90°, joint_5=90°)가 곧 접힌 HOME.
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            velocity_limit_sim=10.0,
            effort_limit_sim=1000.0,
            stiffness=10000.0,
            damping=1000.0,
        ),
        # --- RG2-FT 그리퍼 — USD 에 박힌 드라이브 게인 유지 ---
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*finger_joint", ".*knuckle_joint"],
            stiffness=None,
            damping=None,
        ),
    },
)


_ARM_JOINT_EXPR = ["joint_[1-6]"]
_arm_joint_ids = None


def keep_arm_folded(robot) -> None:
    """m0609 팔을 HOME(접힌 자세)로 고정한다.

    팔은 Ackermann 액션에 묶여있지 않다.  Isaac Lab 은 팔 조인트의 위치
    타깃을 0 으로 깔아두므로(USD 의 접힘 드라이브 타깃을 무시), 그냥 두면
    팔이 0(펴진 자세)으로 끌려가거나 드라이브가 약해 흐트러진다.

    이 함수는 매 step
      · 팔 위치 '타깃' 을 HOME 으로  (드라이브가 HOME 을 향하게)
      · 팔 조인트 '상태' 를 HOME 으로 (물리 드리프트를 매 step 되돌림)
    둘 다 해서 팔을 접힌 채 단단히 유지한다.  주행 테스트에서 팔은 움직일
    필요가 없으므로 이렇게 사실상 고정해도 무방하다.

    env.reset() 직후, 그리고 매 env.step() 직후 호출한다.

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
