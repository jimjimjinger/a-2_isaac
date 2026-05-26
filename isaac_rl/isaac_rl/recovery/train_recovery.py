"""train_recovery.py — Isaac Lab PPO 학습 진입점.

실행 방법:
  /mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p \\
    ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery/train_recovery.py \\
    --num_envs 64 --headless --max_iterations 3000

결과:
  logs/recovery/<timestamp>/
    ├── checkpoints/model_<step>.pt
    ├── params/env.yaml
    └── summaries/ (TensorBoard)
"""
from __future__ import annotations

import argparse
import importlib.metadata as metadata
import os
import sys
from datetime import datetime

from isaaclab.app import AppLauncher

# ── 인수 파싱 ──────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Rover Recovery RL 학습")
parser.add_argument("--num_envs",        type=int,   default=64)
parser.add_argument("--max_iterations",  type=int,   default=3000)
parser.add_argument("--seed",            type=int,   default=42)
parser.add_argument("--log_dir",         type=str,   default="logs/recovery")
parser.add_argument("--checkpoint",      type=str,   default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ── Isaac Lab / RL 임포트 (앱 시작 후) ────────────────────────────────────
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)
from rsl_rl.runners import OnPolicyRunner

from recovery_env_cfg import RoverRecoveryEnvCfg

RSL_RL_VERSION = metadata.version("rsl-rl-lib")
print(f"[train] rsl-rl-lib version: {RSL_RL_VERSION}")


# ── PPO 에이전트 설정 (deprecated policy 형식 — handle_deprecated_rsl_rl_cfg 로 변환) ─
RESUME_LR = 1e-4   # fixed LR: adaptive가 1e-5로 고착되는 문제 방지


@configclass
class RoverRecoveryAgentCfg(RslRlOnPolicyRunnerCfg):
    seed              = 42
    num_steps_per_env = 128    # 96→128: 1000스텝 에피소드(20s) 대비 더 긴 horizon
    max_iterations    = 3000
    save_interval     = 200
    experiment_name   = "rover_recovery"
    empirical_normalization = True
    obs_groups        = {}

    policy = RslRlPpoActorCriticCfg(
        init_noise_std          = 1.0,
        actor_obs_normalization  = False,
        critic_obs_normalization = False,
        actor_hidden_dims        = [256, 128, 64],
        critic_hidden_dims       = [256, 128, 64],
        activation               = "elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef       = 1.0,
        use_clipped_value_loss = True,
        clip_param            = 0.2,
        entropy_coef          = 0.008,
        num_learning_epochs   = 5,
        num_mini_batches      = 8,
        learning_rate         = RESUME_LR,
        schedule              = "fixed",   # adaptive→fixed: LR 1e-5 고착 방지
        gamma                 = 0.99,
        lam                   = 0.95,
        desired_kl            = 0.02,      # 0.01→0.02: KL tolerance 완화
        max_grad_norm         = 1.0,
    )


# ── 환경 생성 ─────────────────────────────────────────────────────────────
env_cfg = RoverRecoveryEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.seed = args.seed

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

# ── 에이전트 설정 ──────────────────────────────────────────────────────────
agent_cfg = RoverRecoveryAgentCfg()
agent_cfg.max_iterations = args.max_iterations
agent_cfg.seed = args.seed
agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)

# ── 로그 경로 ─────────────────────────────────────────────────────────────
log_root = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", args.log_dir
))
log_dir = os.path.join(log_root, datetime.now().strftime("%Y%m%d_%H%M%S"))

# ── 학습 실행 ─────────────────────────────────────────────────────────────
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device="cuda:0")

if args.checkpoint:
    runner.load(args.checkpoint)
    # 저장된 optimizer LR(1e-5 수준)을 리셋해서 학습이 다시 진행되도록 함
    for pg in runner.alg.optimizer.param_groups:
        pg["lr"] = RESUME_LR
    print(f"[train] 체크포인트 로드: {args.checkpoint}")
    print(f"[train] optimizer LR 리셋: {RESUME_LR}")

print(f"\n{'='*60}")
print(f"  Rover Recovery RL 학습 시작")
print(f"  환경 수  : {args.num_envs}")
print(f"  최대 iter: {args.max_iterations}")
print(f"  로그 경로: {log_dir}")
print(f"  TensorBoard: tensorboard --logdir {log_root}")
print(f"{'='*60}\n")

runner.learn(
    num_learning_iterations=args.max_iterations,
    init_at_random_ep_len=True,
)

# ── 최종 정책 저장 ────────────────────────────────────────────────────────
policy_out = os.path.join(
    os.path.dirname(__file__), "..", "..", "policies", "recovery_policy.pt"
)
os.makedirs(os.path.dirname(policy_out), exist_ok=True)
torch.save(runner.alg.actor_critic.state_dict() if hasattr(runner.alg, "actor_critic")
           else {"actor_state_dict": runner.alg.actor.state_dict(),
                 "critic_state_dict": runner.alg.critic.state_dict()}, policy_out)
print(f"\n[train] 정책 저장 완료: {policy_out}")

env.close()
simulation_app.close()
