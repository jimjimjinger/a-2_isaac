"""play_recovery.py — 학습된 정책을 Isaac Sim GUI에서 시각화.

실행:
  cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery
  /mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p play_recovery.py \
      --checkpoint /home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery/20260526_173647/model_800.pt \
      --num_envs 4
"""
from __future__ import annotations
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Rover Recovery 정책 시각화")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs",   type=int, default=4)
parser.add_argument("--num_steps",  type=int, default=10000)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = False  # GUI 강제 활성화

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
import importlib.metadata as metadata
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


@configclass
class RoverRecoveryAgentCfg(RslRlOnPolicyRunnerCfg):
    seed              = 42
    num_steps_per_env = 32
    max_iterations    = 1
    save_interval     = 99999
    experiment_name   = "rover_recovery_play"
    empirical_normalization = True
    obs_groups        = {}
    policy = RslRlPpoActorCriticCfg(
        init_noise_std          = 0.3,
        actor_obs_normalization  = False,
        critic_obs_normalization = False,
        actor_hidden_dims        = [256, 128, 64],
        critic_hidden_dims       = [256, 128, 64],
        activation               = "elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True,
        clip_param=0.2, entropy_coef=0.003,
        num_learning_epochs=4, num_mini_batches=4,
        learning_rate=3e-4, schedule="adaptive",
        gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=0.5,
    )


env_cfg = RoverRecoveryEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.seed = 42

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

agent_cfg = RoverRecoveryAgentCfg()
agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)

runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir="/tmp/play_log", device="cuda:0")
runner.load(args.checkpoint)

print(f"\n[play] 체크포인트: {args.checkpoint}")
print(f"[play] 환경 수: {args.num_envs}  |  최대 스텝: {args.num_steps}")
print(f"[play] 창이 열리면 로버가 자동으로 복구 동작 실행\n")

policy = runner.get_inference_policy(device="cuda:0")

obs, _ = env.reset()
success, total, step = 0, 0, 0

try:
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
        obs, rewards, dones, extras = env.step(actions)

        if dones.any():
            time_outs = extras.get("time_outs", torch.zeros_like(dones, dtype=torch.bool))
            for i in dones.nonzero(as_tuple=False).flatten().tolist():
                total += 1
                if not time_outs[i]:
                    success += 1

        step += 1
        if step % 200 == 0 and total > 0:
            print(f"  step {step:5d} | 에피소드 {total:3d}개 | 성공률 {success/total:.1%}")

        if step >= args.num_steps:
            break
finally:
    if total > 0:
        print(f"\n[play] 최종: {success}/{total} 성공 = {success/total:.1%}")
    env.close()
    simulation_app.close()
