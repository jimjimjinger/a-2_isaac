# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""avoid_test_new_ 의 MDP 항목 모음.

drive_test 와 동일한 Ackermann 액션에 더해, 룰베이스 base controller(goal-seek
+ 장애물 방향 결정) 위에 RL 정책이 residual 을 얹는 ResidualAckermannAction
을 추가로 제공한다.

  · AckermannActionCfg          — drive_test 와 동일.
  · ResidualAckermannActionCfg  — base(룰베이스) + residual(RL) 합성.
"""

from isaaclab.envs.mdp import *  # noqa: F401, F403

from . import events, observations, rewards, terminations  # noqa: F401
from .actions_cfg import AckermannActionCfg  # noqa: F401
from .commands import (  # noqa: F401
    FlatPatchesGoalCommand,
    FlatPatchesGoalCommandCfg,
    RandomGoalCommand,
    RandomGoalCommandCfg,
)
from .events import reset_root_flat_patches, reset_root_safe  # noqa: F401
from .observations import (  # noqa: F401
    base_ang_vel_b,
    base_lin_vel_b,
    goal_body_xy_dist,
    goal_yaw_sincos,
    raycast_prominence,
)
from .residual_action import (  # noqa: F401
    ResidualAckermannAction,
    ResidualAckermannActionCfg,
)
from .rewards import (  # noqa: F401
    action_rate_l2,
    collision_penalty,
    goal_alignment,
    goal_reached_bonus,
    obstacle_proximity_penalty,
    position_tanh,
    progress,
    time_penalty,
)
from .terminations import (  # noqa: F401
    collision,
    goal_reached,
    out_of_bounds,
    vehicle_tilt,
    world_out_of_bounds,
)
