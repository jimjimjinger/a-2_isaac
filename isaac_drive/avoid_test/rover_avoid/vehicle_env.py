# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""ManagerBasedRLEnv 서브클래스 — 매 step m0609 팔을 접힌 HOME 으로 고정.

vehicle_v1.usd 의 m0609 팔은 회피 액션(Ackermann)에 묶이지 않아 그냥 두면
시뮬레이션 중 흐트러진다 (접힘 → 펴짐/처짐). 액추에이터 stiffness 만으로는
위치 타깃이 0 으로 남아 못 잡으므로, 환경이 매 step `keep_arm_folded()` 를
호출해 팔을 고정한다.

rsl_rl 학습 루프는 `env.step()` 을 내부에서 돌리므로, 이렇게 환경 자체에
넣어야 학습 중에도 팔이 고정된다. `__init__.py` 의 모든 태스크 entry_point
가 이 클래스를 가리킨다.
"""

from __future__ import annotations

from isaaclab.envs import ManagerBasedRLEnv

from .rover import keep_arm_folded


class VehicleAvoidEnv(ManagerBasedRLEnv):
    """m0609 팔을 접힌 HOME 으로 고정하는 장애물 회피 RL 환경."""

    def reset(self, *args, **kwargs):
        result = super().reset(*args, **kwargs)
        keep_arm_folded(self.scene["robot"])
        return result

    def step(self, action):
        # super().step() 안에서 물리 진행 + 종료 env auto-reset 까지 끝난다.
        # 그 뒤 모든 env(새로 reset 된 것 포함) 팔을 HOME 으로 고정.
        result = super().step(action)
        keep_arm_folded(self.scene["robot"])
        return result
