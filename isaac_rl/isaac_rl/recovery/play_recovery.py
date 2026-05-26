"""play_recovery.py — 학습된 정책으로 시각화 실행.

실행 방법:
  cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery
  /mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p play_recovery.py \\
      --checkpoint <path/to/model_XXXX.pt> \\
      --num_envs 4
"""
from __future__ import annotations

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Rover Recovery 정책 시각화")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs",   type=int, default=4)
parser.add_argument("--num_steps",  type=int, default=1000)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 시각화 강제 활성화
args.headless = False

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)
from rsl_rl.runners import OnPolicyRunner
import importlib.metadata as metadata

from recovery_env_cfg import RoverRecoveryEnvCfg
from isaaclab.utils import configclass

RSL_RL_VERSION = metadata.version("rsl-rl-lib")


# train_recovery를 import하면 최상위 코드(AppLauncher 등)가 재실행되므로 여기서 직접 정의
@configclass
class RoverRecoveryAgentCfg(RslRlOnPolicyRunnerCfg):
    seed              = 42
    num_steps_per_env = 128  # obs dim=37, action dim=16
    max_iterations    = 3000
    save_interval     = 200
    experiment_name   = "rover_recovery"
    empirical_normalization = True
    obs_groups        = {}

    policy = RslRlPpoActorCriticCfg(
        init_noise_std           = 1.0,
        actor_obs_normalization  = False,
        critic_obs_normalization = False,
        actor_hidden_dims        = [256, 128, 64],
        critic_hidden_dims       = [256, 128, 64],
        activation               = "elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef        = 1.0,
        use_clipped_value_loss = True,
        clip_param             = 0.2,
        entropy_coef           = 0.008,
        num_learning_epochs    = 5,
        num_mini_batches       = 8,
        learning_rate          = 1e-4,
        schedule               = "fixed",
        gamma                  = 0.99,
        lam                    = 0.95,
        desired_kl             = 0.02,
        max_grad_norm          = 1.0,
    )

env_cfg = RoverRecoveryEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.episode_length_s = 20.0

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

agent_cfg = RoverRecoveryAgentCfg()
agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)

runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device="cuda:0")
runner.load(args.checkpoint)
print(f"\n[play] 체크포인트 로드: {args.checkpoint}")
print(f"[play] {args.num_envs}개 환경, {args.num_steps} step 실행\n")

policy = runner.get_inference_policy(device="cuda:0")

obs = env.get_observations()
for step in range(args.num_steps):
    with torch.no_grad():
        actions = policy(obs)
    obs, rewards, dones, infos = env.step(actions)
    if (step + 1) % 100 == 0:
        print(f"  step {step+1:4d} | mean_reward={rewards.mean().item():.3f}")

env.close()
simulation_app.close()
