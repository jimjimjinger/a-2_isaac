# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""rsl_rl PPO 학습용 하이퍼파라미터 — 평지+박스 (residual RL).

rsl-rl 5.0.1 API — `actor`/`critic` 을 각각 `RslRlMLPModelCfg` 로 따로 준다.
deprecated 였던 `policy: RslRlPpoActorCriticCfg` 는 안 씀.

residual 학습의 특징:
  · 정책 초기 출력 ≈ 0 이면 룰베이스 거동을 그대로 따라가도록 base 가 깔려 있음.
  · 따라서 `init_std` 를 약간 작게(0.3) 두어 초반 무작위 액션 영향 줄임.
  · 작은 정책망(128, 128) 으로 시작 — 잘 안 풀리면 크게.
"""

from __future__ import annotations

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlMLPModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class FlatEnvPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """평지+박스 환경의 PPO Runner 설정 (rsl-rl 5.x API)."""

    num_steps_per_env: int = 32
    """env 당 한 iteration 에서 모을 스텝."""

    max_iterations: int = 1500
    save_interval: int = 25
    experiment_name: str = "avoid_test_new_flat"
    run_name: str = ""

    # ----- Actor (정책) — Gaussian 분포로 stochastic 출력 -------------
    actor: RslRlMLPModelCfg = RslRlMLPModelCfg(
        class_name="MLPModel",
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(
            class_name="GaussianDistribution",
            init_std=0.3,            # residual 학습 — 초기 노이즈 약간 작게
            std_type="scalar",
        ),
    )

    # ----- Critic (가치함수) — deterministic 스칼라 출력 --------------
    critic: RslRlMLPModelCfg = RslRlMLPModelCfg(
        class_name="MLPModel",
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=None,
    )

    # ----- PPO 알고리즘 하이퍼파라미터 ---------------------------------
    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class RoughEnvPPORunnerCfg(FlatEnvPPORunnerCfg):
    """terrain_00022 fine-tune 용 PPO Runner — flat 정책을 출발점으로.

    learning_rate 만 낮춰 catastrophic forgetting 방지.  나머지(actor/critic·
    algorithm 의 클리핑 등)는 1단계와 동일 유지.
    """

    experiment_name: str = "avoid_test_new_rough"
    max_iterations: int = 1500

    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        class_name="PPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5.0e-4,        # 1e-3 → 5e-4 (fine-tune)
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
