# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""학습된 residual 정책을 평지+박스 환경에서 평가.

  cd .../isaac_drive/avoid_test_new_
  /home/rokey/dev_ws/venv/isaaclab/bin/python play.py --checkpoint logs/.../model_final.pt

옵션:
  --checkpoint PATH    학습된 정책 .pt (없으면 base controller 만 — residual=0).
  --num_envs N         평가 env 개수 (기본 4).

env 한 개에 GUI 로 띄워 확인 — 차량이 무작위 goal 들로 어떻게 가는지.
goal·spawn 은 매 에피소드 자동 무작위 (terrain.flat_patches 에서 샘플).
"""

from __future__ import annotations

import argparse
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="residual RL 평가")
parser.add_argument("--env", choices=["flat", "rough"], default="flat",
                    help="환경 — flat(1단계 평지+박스) / rough(2단계 terrain_00022).")
parser.add_argument("--checkpoint", type=str, default=None,
                    help="학습된 정책 .pt.  생략 시 residual=0 (base controller 만).")
parser.add_argument("--num_envs", type=int, default=4, help="평가 env 개수.")
parser.add_argument("--seed", type=int, default=0, help="랜덤 시드.")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

# GUI 강제 — 평가는 보면서 함.
if not getattr(args_cli, "headless", False):
    pass   # default = GUI

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""--- 시뮬레이터 기동 후 임포트 ---"""

import torch  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from agents.rsl_rl_ppo_cfg import FlatEnvPPORunnerCfg, RoughEnvPPORunnerCfg  # noqa: E402
from flat_env_cfg import FlatEnvCfg  # noqa: E402
from rough_env_cfg import RoughEnvCfg  # noqa: E402


def main() -> None:
    if args_cli.env == "flat":
        env_cfg = FlatEnvCfg()
        runner_cfg_cls = FlatEnvPPORunnerCfg
    elif args_cli.env == "rough":
        env_cfg = RoughEnvCfg()
        runner_cfg_cls = RoughEnvPPORunnerCfg
    else:
        raise ValueError(f"unknown --env: {args_cli.env}")
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    # 정책 로드 or 0-residual (base controller 만).
    policy = None
    if args_cli.checkpoint:
        import importlib.metadata
        from rsl_rl.runners import OnPolicyRunner
        from isaaclab_rl.rsl_rl import handle_deprecated_rsl_rl_cfg
        runner_cfg = runner_cfg_cls()
        handle_deprecated_rsl_rl_cfg(runner_cfg, importlib.metadata.version("rsl-rl-lib"))
        runner = OnPolicyRunner(env, runner_cfg.to_dict(), log_dir=None, device=env.device)
        runner.load(args_cli.checkpoint)
        policy = runner.get_inference_policy(device=env.device)
        print(f"\n[play] policy loaded: {args_cli.checkpoint}\n")
    else:
        print("\n[play] no checkpoint — base controller 만으로 평가 (residual=0)\n")

    obs, _ = env.reset()

    print("=" * 60)
    print(f"  env: {env.num_envs}  action_dim: {env.num_actions}")
    print("  ESC (뷰어 창) 또는 Ctrl-C 로 종료.")
    print("=" * 60 + "\n")

    try:
        while simulation_app.is_running():
            with torch.no_grad():
                if policy is None:
                    action = torch.zeros(env.num_envs, env.num_actions, device=env.device)
                else:
                    action = policy(obs)
            obs, _, _, _ = env.step(action)
    except KeyboardInterrupt:
        print("\n[play] 중단")
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
