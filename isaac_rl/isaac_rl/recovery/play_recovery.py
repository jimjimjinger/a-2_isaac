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

RSL_RL_VERSION = metadata.version("rsl-rl-lib")


from train_recovery import RoverRecoveryAgentCfg

env_cfg = RoverRecoveryEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.episode_length_s = 15.0

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

agent_cfg = RoverRecoveryAgentCfg()
agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)

runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device="cuda:0")
runner.load(args.checkpoint)
print(f"\n[play] 체크포인트 로드: {args.checkpoint}")
print(f"[play] {args.num_envs}개 환경, {args.num_steps} step 실행\n")

policy = runner.get_inference_policy(device="cuda:0")

obs, _ = env.get_observations()
for step in range(args.num_steps):
    with torch.no_grad():
        actions = policy(obs)
    obs, rewards, dones, infos = env.step(actions)
    if (step + 1) % 100 == 0:
        print(f"  step {step+1:4d} | mean_reward={rewards.mean().item():.3f}")

env.close()
simulation_app.close()
