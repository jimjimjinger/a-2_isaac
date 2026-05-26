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
import math
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

# ── 인수 파싱 ──────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Rover Recovery RL 학습")
parser.add_argument("--num_envs",        type=int,   default=64)
parser.add_argument("--max_iterations",  type=int,   default=3000)
parser.add_argument("--seed",            type=int,   default=42)
parser.add_argument("--log_dir",         type=str,   default="logs/recovery")
parser.add_argument("--checkpoint",      type=str,   default=None)
parser.add_argument("--eval_episodes",    type=int,   default=100)
parser.add_argument("--eval_envs",        type=int,   default=4)
parser.add_argument("--eval_interval",    type=int,   default=100)
parser.add_argument("--policy_std_max",   type=float, default=2.0)
parser.add_argument("--policy_std_min",   type=float, default=1e-3)
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
RESUME_LR = 3e-5   # 청크 재시작 LR: 너무 크면 첫 50iter reward 붕괴


def _policy_std_tensor(policy: object) -> torch.Tensor | None:
    """현재 policy의 표준편차 텐서를 가능한 한 안정적으로 추출한다."""
    for attr in ("output_std", "std", "log_std"):
        if hasattr(policy, attr):
            value = getattr(policy, attr)
            if attr == "log_std":
                return torch.exp(value)
            return value
    if hasattr(policy, "action_std"):
        return getattr(policy, "action_std")
    return None


def _clamp_policy_std(policy: object, std_min: float, std_max: float) -> tuple[torch.Tensor | None, bool]:
    """policy std를 안전한 범위로 clamp하고, 현재 std 텐서를 반환한다."""
    clamped = False
    if hasattr(policy, "std") and isinstance(getattr(policy, "std"), torch.nn.Parameter):
        policy.std.data.clamp_(std_min, std_max)
        clamped = True
    elif hasattr(policy, "log_std") and isinstance(getattr(policy, "log_std"), torch.nn.Parameter):
        policy.log_std.data.clamp_(math.log(std_min), math.log(std_max))
        clamped = True

    return _policy_std_tensor(policy), clamped


def _log_policy_std(writer, std_tensor: torch.Tensor, step: int, std_max: float) -> None:
    """policy std 요약값과 histogram을 TensorBoard에 기록한다."""
    if writer is None:
        return

    std_tensor = std_tensor.detach().float()
    std_flat = std_tensor.reshape(-1)
    std_mean = std_flat.mean().item()
    std_max_value = std_flat.max().item()
    std_min_value = std_flat.min().item()

    writer.add_scalar("policy_std/mean", std_mean, step)
    writer.add_scalar("policy_std/max", std_max_value, step)
    writer.add_scalar("policy_std/min", std_min_value, step)
    writer.add_histogram("policy_std/hist", std_flat.cpu(), step)

    if std_max_value > std_max:
        print(
            f"[warn] policy std exceeded threshold at iter {step}: "
            f"max={std_max_value:.4f} > {std_max:.4f}"
        )


def _extract_timeout_flags(extras: dict, env: object) -> torch.Tensor | None:
    """Isaac Lab extras에서 timeout 플래그를 최대한 안전하게 읽는다."""
    if isinstance(extras, dict):
        for key in ("time_outs", "timeouts"):
            if key in extras:
                value = extras[key]
                if isinstance(value, torch.Tensor):
                    return value

    base_env = getattr(env, "unwrapped", env)
    termination_manager = getattr(base_env, "termination_manager", None)
    if termination_manager is not None and hasattr(termination_manager, "time_outs"):
        value = termination_manager.time_outs
        if isinstance(value, torch.Tensor):
            return value

    return None


def evaluate_policy(
    policy: object,
    env: object,
    eval_episodes: int,
) -> dict[str, float]:
    """deterministic policy로 success / timeout rate를 평가한다."""
    policy.eval()
    base_env = getattr(env, "unwrapped", env)

    snapshot_state = base_env.scene.get_state(is_relative=False)
    snapshot_episode_length = base_env.episode_length_buf.clone()
    snapshot_common_step = int(base_env.common_step_counter)
    snapshot_action = base_env.action_manager._action.clone()
    snapshot_prev_action = base_env.action_manager._prev_action.clone()
    snapshot_reset_buf = getattr(base_env, "reset_buf", None)
    if isinstance(snapshot_reset_buf, torch.Tensor):
        snapshot_reset_buf = snapshot_reset_buf.clone()
    snapshot_reset_terminated = getattr(base_env, "reset_terminated", None)
    if isinstance(snapshot_reset_terminated, torch.Tensor):
        snapshot_reset_terminated = snapshot_reset_terminated.clone()
    snapshot_reset_time_outs = getattr(base_env, "reset_time_outs", None)
    if isinstance(snapshot_reset_time_outs, torch.Tensor):
        snapshot_reset_time_outs = snapshot_reset_time_outs.clone()
    snapshot_stable_frames = getattr(base_env, "_recovery_stable_frames", None)
    if isinstance(snapshot_stable_frames, torch.Tensor):
        snapshot_stable_frames = snapshot_stable_frames.clone()

    obs, _ = env.reset(seed=None)
    completed = 0
    success = 0
    timeouts = 0

    try:
        while completed < eval_episodes:
            with torch.inference_mode():
                actions = policy.act_inference(obs)
            obs, _, dones, extras = env.step(actions)

            if not torch.any(dones):
                continue

            timeout_flags = _extract_timeout_flags(extras, base_env)
            dones_idx = torch.nonzero(dones, as_tuple=False).flatten()

            if timeout_flags is None:
                timeout_flags = torch.zeros_like(dones, dtype=torch.bool)

            for env_id in dones_idx.tolist():
                completed += 1
                if bool(timeout_flags[env_id].item()):
                    timeouts += 1
                else:
                    success += 1
                if completed >= eval_episodes:
                    break
    finally:
        base_env.reset_to(snapshot_state, env_ids=None, seed=None, is_relative=False)
        base_env.episode_length_buf = snapshot_episode_length
        base_env.common_step_counter = snapshot_common_step
        base_env.action_manager._action = snapshot_action
        base_env.action_manager._prev_action = snapshot_prev_action
        if snapshot_reset_buf is not None:
            base_env.reset_buf = snapshot_reset_buf
        if snapshot_reset_terminated is not None:
            base_env.reset_terminated = snapshot_reset_terminated
        if snapshot_reset_time_outs is not None:
            base_env.reset_time_outs = snapshot_reset_time_outs
        if snapshot_stable_frames is not None:
            base_env._recovery_stable_frames = snapshot_stable_frames

    return {
        "eval/success_rate": success / max(completed, 1),
        "eval/timeout_rate": timeouts / max(completed, 1),
        "eval/episodes": float(completed),
    }


@configclass
class RoverRecoveryAgentCfg(RslRlOnPolicyRunnerCfg):
    seed              = 42
    num_steps_per_env = 32     # 64→32 revert: 8GB GPU에서 OOM 발생
    max_iterations    = 3000
    save_interval     = 100
    experiment_name   = "rover_recovery"
    empirical_normalization = True
    obs_groups        = {}

    policy = RslRlPpoActorCriticCfg(
        init_noise_std          = 0.3,     # 1.0→0.3: 체크포인트 재시작 시 noise 억제
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
        entropy_coef           = 0.003,    # 0.02→0.003: std 발산 방지 (핵심)
        num_learning_epochs    = 4,
        num_mini_batches       = 4,
        learning_rate          = 3e-4,
        schedule               = "adaptive",
        gamma                  = 0.99,
        lam                    = 0.95,
        desired_kl             = 0.01,     # 0.02→0.01: KL 강화로 큰 policy 변화 억제
        max_grad_norm          = 0.5,
    )


# ── 환경 생성 ─────────────────────────────────────────────────────────────
env_cfg = RoverRecoveryEnvCfg()
env_cfg.scene.num_envs = args.num_envs
env_cfg.seed = args.seed

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

# ── Scene 진단 출력 ─────────────────────────────────────────────────────────
_env      = env.unwrapped
_vehicle  = _env.scene["vehicle"]
_cfg      = _env.cfg
_physx    = _cfg.sim.physx
_artprop  = _cfg.scene.vehicle.spawn.articulation_props

_gpu_total = torch.cuda.get_device_properties(0).total_memory / 1024**3
_gpu_used  = torch.cuda.memory_allocated(0) / 1024**3
_physics_hz = int(1.0 / _cfg.sim.dt)
_policy_hz  = int(1.0 / (_cfg.sim.dt * _cfg.decimation))

print(f"\n{'='*64}")
print(f"  [ Simulation Diagnostics ]")
print(f"  GPU                : {torch.cuda.get_device_name(0)}")
print(f"  GPU memory         : {_gpu_used:.2f} GB used / {_gpu_total:.1f} GB total")
print(f"  Num envs           : {_env.num_envs}")
print(f"  Vehicle bodies     : {_vehicle.num_bodies}")
print(f"  Vehicle joints     : {_vehicle.num_joints}")
print(f"  Physics dt         : {_cfg.sim.dt*1000:.1f} ms  ({_physics_hz} Hz)")
print(f"  Policy decimation  : {_cfg.decimation}  →  policy {_policy_hz} Hz")
print(f"  Render interval    : every {_cfg.sim.render_interval} physics steps")
print(f"  Self collision     : {_artprop.enabled_self_collisions}")
print(f"  Pos solver iters   : {_artprop.solver_position_iteration_count}")
print(f"  Vel solver iters   : {_artprop.solver_velocity_iteration_count}")
print(f"  Enable stabilize   : {_physx.enable_stabilization}")
print(f"  Max contact slots  : {_physx.gpu_max_rigid_contact_count}")
print(f"  Max patch slots    : {_physx.gpu_max_rigid_patch_count}")
print(f"  Episode length     : {_cfg.episode_length_s} s")
print(f"  Steps per rollout  : {_env.num_envs} envs × 32 steps")
print(f"{'='*64}\n")
print(f"  [hint] GPU 사용률 모니터: watch -n1 nvidia-smi")
print(f"  [hint] 상세 프로파일  : nvtx / nsys profile")
print(f"{'='*64}\n")

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

_t_train_start = time.perf_counter()

policy = runner.alg.get_policy()
best_metrics = {"eval/success_rate": -1.0, "eval/timeout_rate": 1.0}
best_checkpoint = os.path.join(log_dir, "best_rover_upright.pt")

try:
    start_it = runner.current_learning_iteration
    total_it = start_it + args.max_iterations
    obs = env.get_observations().to(runner.device)
    runner.alg.train_mode()

    if runner.is_distributed:
        print(f"Synchronizing parameters for rank {runner.gpu_global_rank}...")
        runner.alg.broadcast_parameters()

    runner.logger.init_logging_writer()

    for it in range(start_it, total_it):
        start = time.time()

        with torch.inference_mode():
            for _ in range(runner.cfg["num_steps_per_env"]):
                actions = runner.alg.act(obs)
                obs, rewards, dones, extras = env.step(actions.to(env.device))

                if runner.cfg.get("check_for_nan", True):
                    from rsl_rl.utils import check_nan
                    check_nan(obs, rewards, dones)

                obs, rewards, dones = (
                    obs.to(runner.device),
                    rewards.to(runner.device),
                    dones.to(runner.device),
                )

                runner.alg.process_env_step(obs, rewards, dones, extras)
                intrinsic_rewards = runner.alg.intrinsic_rewards if runner.cfg["algorithm"]["rnd_cfg"] else None
                runner.logger.process_env_step(rewards, dones, extras, intrinsic_rewards)

        stop = time.time()
        collect_time = stop - start
        start = stop

        runner.alg.compute_returns(obs)
        loss_dict = runner.alg.update()

        raw_std_tensor = _policy_std_tensor(policy)
        if raw_std_tensor is not None:
            _log_policy_std(runner.logger.writer, raw_std_tensor, it, args.policy_std_max)
        std_tensor, _ = _clamp_policy_std(policy, args.policy_std_min, args.policy_std_max)

        stop = time.time()
        learn_time = stop - start
        runner.current_learning_iteration = it

        runner.logger.log(
            it=it,
            start_it=start_it,
            total_it=total_it,
            collect_time=collect_time,
            learn_time=learn_time,
            loss_dict=loss_dict,
            learning_rate=runner.alg.learning_rate,
            action_std=std_tensor if std_tensor is not None else None,
            rnd_weight=runner.alg.rnd.weight if runner.cfg["algorithm"]["rnd_cfg"] else None,
        )

        if runner.logger.writer is not None and (
            it % runner.cfg["save_interval"] == 0 or it == total_it - 1
        ):
            runner.save(os.path.join(runner.logger.log_dir, f"model_{it}.pt"))

        if (it + 1) % args.eval_interval == 0 or it == total_it - 1:
            try:
                eval_stats = evaluate_policy(policy, env, args.eval_episodes)
                obs = env.get_observations().to(runner.device)
                for tag, value in eval_stats.items():
                    if runner.logger.writer is not None:
                        runner.logger.writer.add_scalar(tag, value, it)

                print(
                    f"[eval] iter {it:5d} | success={eval_stats['eval/success_rate']:.3f} "
                    f"| timeout={eval_stats['eval/timeout_rate']:.3f} "
                    f"| episodes={int(eval_stats['eval/episodes'])}"
                )

                better_success = eval_stats["eval/success_rate"] > best_metrics["eval/success_rate"]
                same_success_better_timeout = (
                    eval_stats["eval/success_rate"] == best_metrics["eval/success_rate"]
                    and eval_stats["eval/timeout_rate"] < best_metrics["eval/timeout_rate"]
                )
                if better_success or same_success_better_timeout:
                    best_metrics = eval_stats
                    runner.save(best_checkpoint, infos=eval_stats)
                    print(f"[eval] best checkpoint updated: {best_checkpoint}")
            finally:
                runner.alg.train_mode()

    _t_train_total = time.perf_counter() - _t_train_start
    _samples_total = args.max_iterations * runner.cfg["num_steps_per_env"] * args.num_envs
    print(f"\n{'='*60}")
    print(f"  [ Training Performance ]")
    print(f"  Total time    : {_t_train_total:.1f} s")
    print(f"  Per iter      : {_t_train_total / args.max_iterations * 1000:.1f} ms/iter")
    print(f"  Env steps/sec : {_samples_total / _t_train_total:.0f}  (all envs combined)")
    print(f"  Policy SPS    : {args.max_iterations * runner.cfg['num_steps_per_env'] / _t_train_total:.0f}  (batched steps/sec)")
    print(f"  Best success  : {best_metrics['eval/success_rate']:.3f}")
    print(f"  Best timeout  : {best_metrics['eval/timeout_rate']:.3f}")
    print(f"{'='*60}\n")

    # ── 최종 정책 저장 ────────────────────────────────────────────────────
    policy_out = os.path.join(
        os.path.dirname(__file__), "..", "..", "policies", "recovery_policy.pt"
    )
    os.makedirs(os.path.dirname(policy_out), exist_ok=True)
    torch.save(policy.state_dict(), policy_out)
    print(f"\n[train] 정책 저장 완료: {policy_out}")
finally:
    env.close()
    simulation_app.close()
