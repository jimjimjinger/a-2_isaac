# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""M0609-lift 스타일 로버 장애물 회피 RL 태스크 패키지 (avoid_test).

구성 (m0609_lift_code_ver2 와 동일한 매니저 기반 구조):
  rover.py          — 통합 차량 vehicle_v1 ArticulationCfg (27 DOF)
  vehicle_env.py    — VehicleAvoidEnv (매 step m0609 팔 HOME 고정)
  avoid_env_cfg.py  — 씬·관측·액션·커맨드·이벤트·보상 설정
  mdp/              — Ackermann 액션 + 커스텀 관측/보상/종료 항목
  agents/           — RSL-RL PPO 설정

모든 태스크 entry_point 는 VehicleAvoidEnv 를 가리킨다 (팔 고정 위해).

Task IDs:
  Isaac-Rover-Avoid-v0          — 랜덤 장애물 학습용
  Isaac-Rover-Avoid-Play-v0     — 랜덤 장애물 재생용
  Isaac-Rover-FixedObs-v0       — 고정 장애물 1개 학습용 (커리큘럼 stage 1)
  Isaac-Rover-FixedObs-Play-v0  — 고정 장애물 재생용
  Isaac-Rover-Stage2-v0         — 랜덤 큐브 3개 회피 학습용 (커리큘럼 stage 2)
  Isaac-Rover-Stage2-Play-v0    — stage 2 재생용
"""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Rover-Avoid-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.avoid_env_cfg:AvoidEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Rover-Avoid-Play-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.avoid_env_cfg:AvoidEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Rover-FixedObs-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.fixed_obs_env_cfg:FixedObsEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Rover-FixedObs-Play-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.fixed_obs_env_cfg:FixedObsEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Rover-Stage2-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stage2_env_cfg:Stage2EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Rover-Stage2-Play-v0",
    entry_point="rover_avoid.vehicle_env:VehicleAvoidEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stage2_env_cfg:Stage2EnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RoverAvoidPPORunnerCfg",
    },
    disable_env_checker=True,
)
