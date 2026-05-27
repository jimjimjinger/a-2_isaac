# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""평지+박스 환경에서 residual RL 학습 진입점 (rsl_rl PPO).

  cd .../isaac_drive/avoid_test_new_
  /home/rokey/dev_ws/venv/isaaclab/bin/python train.py --headless

옵션:
  --num_envs N         env 개수 (기본 cfg 의 64).
  --max_iterations N   학습 iteration (기본 1500).
  --seed N             랜덤 시드.
  --resume PATH        체크포인트에서 이어 학습 (None 이면 처음부터).
  --headless           GUI 끄고 학습 (CPU/GPU 만 사용 — 빠름).
  --logdir PATH        로그·체크포인트 저장 위치 (기본 logs/<timestamp>).
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="residual RL PPO 학습")
parser.add_argument("--env", choices=["flat", "rough"], default="flat",
                    help="환경 선택 — flat(평지+박스, 1단계) / rough(terrain_00022, 2단계 fine-tune).")
parser.add_argument("--num_envs", type=int, default=None,
                    help="env 개수 (기본은 각 EnvCfg.scene.num_envs).")
parser.add_argument("--max_iterations", type=int, default=None,
                    help="학습 iteration (기본 PPORunnerCfg.max_iterations).")
parser.add_argument("--seed", type=int, default=42, help="랜덤 시드.")
parser.add_argument("--resume", type=str, default=None,
                    help="이어 학습할 체크포인트(.pt) 경로 — rough 단계에서 best.pt 지정 권장.")
parser.add_argument("--logdir", type=str, default=None,
                    help="로그·체크포인트 저장 위치 (기본 logs/<exp_name>/<timestamp>).")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""--- 시뮬레이터 기동 후 임포트 ---"""

import importlib.metadata  # noqa: E402

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab.utils.io import dump_yaml  # noqa: E402

from agents.rsl_rl_ppo_cfg import FlatEnvPPORunnerCfg, RoughEnvPPORunnerCfg  # noqa: E402
from flat_env_cfg import FlatEnvCfg  # noqa: E402
from rough_env_cfg import RoughEnvCfg  # noqa: E402


def main() -> None:
    # 환경·러너 cfg 선택.
    if args_cli.env == "flat":
        env_cfg = FlatEnvCfg()
        runner_cfg = FlatEnvPPORunnerCfg()
    elif args_cli.env == "rough":
        env_cfg = RoughEnvCfg()
        runner_cfg = RoughEnvPPORunnerCfg()
    else:
        raise ValueError(f"unknown --env: {args_cli.env}")

    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    if args_cli.max_iterations is not None:
        runner_cfg.max_iterations = args_cli.max_iterations
    runner_cfg.seed = args_cli.seed
    # 설치된 rsl-rl 버전에 맞게 deprecated 필드(stochastic, init_noise_std 등) 정리.
    rsl_rl_version = importlib.metadata.version("rsl-rl-lib")
    handle_deprecated_rsl_rl_cfg(runner_cfg, rsl_rl_version)

    # 로그 디렉토리.
    if args_cli.logdir:
        log_dir = args_cli.logdir
    else:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = os.path.join(THIS_DIR, "logs", runner_cfg.experiment_name, stamp)
    os.makedirs(log_dir, exist_ok=True)
    print(f"\n[train] log_dir = {log_dir}\n")

    # cfg 덤프 — 재현성용.
    try:
        dump_yaml(os.path.join(log_dir, "env_cfg.yaml"), env_cfg.to_dict())
        dump_yaml(os.path.join(log_dir, "runner_cfg.yaml"), runner_cfg.to_dict())
    except Exception as exc:  # noqa: BLE001
        print(f"[train] cfg dump skipped: {exc}")

    # 환경 생성 + rsl_rl 래퍼.
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    print(f"[train] num_envs    = {env.num_envs}")
    try:
        obs_shape = env.unwrapped.observation_manager.group_obs_dim["policy"]
        print(f"[train] obs_dim     = {obs_shape}")
    except Exception:
        pass
    print(f"[train] action_dim  = {env.num_actions}")
    print(f"[train] device      = {env.device}")
    print(f"[train] max_iters   = {runner_cfg.max_iterations}\n")

    # Runner.
    runner = OnPolicyRunner(env, runner_cfg.to_dict(), log_dir=log_dir, device=env.device)
    if args_cli.resume:
        print(f"[train] resuming from {args_cli.resume}")
        runner.load(args_cli.resume)

    # 학습.
    runner.learn(num_learning_iterations=runner_cfg.max_iterations,
                 init_at_random_ep_len=True)

    # 마지막 체크포인트 저장.
    save_path = os.path.join(log_dir, "model_final.pt")
    runner.save(save_path)
    print(f"\n[train] saved final policy → {save_path}\n")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
