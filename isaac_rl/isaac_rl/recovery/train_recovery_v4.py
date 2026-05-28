"""train_recovery_v4.py — v4 학습 진입점.

v3와 동일한 학습 루프, v4 환경 설정 사용.

변경:
  - env_cfg: RoverRecoveryEnvCfgV4 (action 12dim, obs 43dim, forward 2m goal)
  - evaluate_policy: _v4 state 스냅샷 저장/복원 추가
  - experiment_name: "rover_recovery_v4"
  - network: actor/critic hidden [256, 128, 64] 유지

실행 방법:
  /mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p \\
    ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery/train_recovery_v4.py \\
    --num_envs 128 --headless --max_iterations 1000
"""
from __future__ import annotations

import argparse
import importlib.metadata as metadata
import math
import os
import time
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Rover Recovery RL v4 학습")
parser.add_argument("--num_envs",       type=int,   default=128)
parser.add_argument("--max_iterations", type=int,   default=3000)
parser.add_argument("--seed",           type=int,   default=42)
parser.add_argument("--log_dir",        type=str,   default="logs/recovery_v4")
parser.add_argument("--checkpoint",     type=str,   default=None)
parser.add_argument("--eval_episodes",  type=int,   default=100)
parser.add_argument("--eval_interval",  type=int,   default=999999)
parser.add_argument("--resume_log_dir", type=str,   default=None)
parser.add_argument("--policy_std_max", type=float, default=2.0)
parser.add_argument("--policy_std_min", type=float, default=1e-3)
# fine-tune 안정화 오버라이드
parser.add_argument("--lr",            type=float, default=None, help="learning rate override (e.g. 5e-5 for fine-tune)")
parser.add_argument("--clip_param",    type=float, default=None, help="PPO clip param override (e.g. 0.1)")
parser.add_argument("--entropy_coef",  type=float, default=None, help="entropy coef override (e.g. 0.001)")
parser.add_argument("--mini_batches",  type=int,   default=None, help="num mini-batches override (e.g. 8)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

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

from recovery_env_cfg_v4 import RoverRecoveryEnvCfgV4

RSL_RL_VERSION = metadata.version("rsl-rl-lib")
print(f"[train_v4] rsl-rl-lib version: {RSL_RL_VERSION}")


# ── 유틸 ─────────────────────────────────────────────────────────────────────

def _policy_std_tensor(policy):
    for attr in ("output_std", "std", "log_std"):
        if hasattr(policy, attr):
            v = getattr(policy, attr)
            return torch.exp(v) if attr == "log_std" else v
    return getattr(policy, "action_std", None)


def _clamp_policy_std(policy, std_min, std_max):
    clamped = False
    if hasattr(policy, "std") and isinstance(getattr(policy, "std"), torch.nn.Parameter):
        policy.std.data.clamp_(std_min, std_max)
        clamped = True
    elif hasattr(policy, "log_std") and isinstance(getattr(policy, "log_std"), torch.nn.Parameter):
        policy.log_std.data.clamp_(math.log(std_min), math.log(std_max))
        clamped = True
    return _policy_std_tensor(policy), clamped


def _log_policy_std(writer, std_tensor, step, std_max):
    if writer is None:
        return
    std_flat = std_tensor.detach().float().reshape(-1)
    writer.add_scalar("policy_std/mean", std_flat.mean().item(), step)
    writer.add_scalar("policy_std/max",  std_flat.max().item(),  step)
    writer.add_scalar("policy_std/min",  std_flat.min().item(),  step)
    writer.add_histogram("policy_std/hist", std_flat.cpu(), step)
    if std_flat.max().item() > std_max:
        print(f"[warn] policy std exceeded {std_max:.2f} at iter {step}")


def _extract_timeout_flags(extras, env):
    if isinstance(extras, dict):
        for key in ("time_outs", "timeouts"):
            if key in extras and isinstance(extras[key], torch.Tensor):
                return extras[key]
    tm = getattr(getattr(env, "unwrapped", env), "termination_manager", None)
    if tm is not None and hasattr(tm, "time_outs"):
        v = tm.time_outs
        if isinstance(v, torch.Tensor):
            return v
    return None


def _snapshot_v4(base_env):
    """v4 에피소드 state dict 스냅샷."""
    v4 = getattr(base_env, "_v4", None)
    if v4 is None:
        return None
    return {k: v.clone() for k, v in v4.items()}


def _restore_v4(base_env, snapshot):
    if snapshot is not None:
        base_env._v4 = snapshot


def evaluate_policy(policy, env, eval_episodes):
    """deterministic policy로 success / timeout rate 평가."""
    policy.eval()
    base_env = getattr(env, "unwrapped", env)

    # ── 스냅샷 저장 ───────────────────────────────────────────────────────────
    snap_state          = base_env.scene.get_state(is_relative=False)
    snap_ep_len         = base_env.episode_length_buf.clone()
    snap_step           = int(base_env.common_step_counter)
    snap_action         = base_env.action_manager._action.clone()
    snap_prev_action    = base_env.action_manager._prev_action.clone()
    snap_reset_buf      = getattr(base_env, "reset_buf",        None)
    snap_reset_term     = getattr(base_env, "reset_terminated", None)
    snap_reset_tout     = getattr(base_env, "reset_time_outs",  None)
    snap_stable_frames  = getattr(base_env, "_recovery_stable_frames", None)
    snap_v4             = _snapshot_v4(base_env)

    for snap in (snap_reset_buf, snap_reset_term, snap_reset_tout, snap_stable_frames):
        if isinstance(snap, torch.Tensor):
            snap = snap.clone()

    obs, _ = env.reset(seed=None)
    completed, success, timeouts = 0, 0, 0

    try:
        while completed < eval_episodes:
            with torch.inference_mode():
                actions = policy.act_inference(obs)
            obs, _, dones, extras = env.step(actions)

            if not torch.any(dones):
                continue

            timeout_flags = _extract_timeout_flags(extras, base_env)
            if timeout_flags is None:
                timeout_flags = torch.zeros_like(dones, dtype=torch.bool)

            for env_id in torch.nonzero(dones, as_tuple=False).flatten().tolist():
                completed += 1
                if bool(timeout_flags[env_id].item()):
                    timeouts += 1
                else:
                    success += 1
                if completed >= eval_episodes:
                    break
    finally:
        # ── 스냅샷 복원 ───────────────────────────────────────────────────────
        base_env.reset_to(snap_state, env_ids=None, seed=None, is_relative=False)
        base_env.episode_length_buf      = snap_ep_len
        base_env.common_step_counter     = snap_step
        base_env.action_manager._action  = snap_action
        base_env.action_manager._prev_action = snap_prev_action
        if snap_reset_buf    is not None: base_env.reset_buf        = snap_reset_buf
        if snap_reset_term   is not None: base_env.reset_terminated = snap_reset_term
        if snap_reset_tout   is not None: base_env.reset_time_outs  = snap_reset_tout
        if snap_stable_frames is not None: base_env._recovery_stable_frames = snap_stable_frames
        _restore_v4(base_env, snap_v4)

    return {
        "eval/success_rate": success   / max(completed, 1),
        "eval/timeout_rate": timeouts  / max(completed, 1),
        "eval/episodes":     float(completed),
    }


# ── PPO 에이전트 설정 ─────────────────────────────────────────────────────────
@configclass
class RoverRecoveryAgentCfgV4(RslRlOnPolicyRunnerCfg):
    seed              = 42
    num_steps_per_env = 32
    max_iterations    = 3000
    save_interval     = 20
    experiment_name   = "rover_recovery_v4"
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
        value_loss_coef        = 1.0,
        use_clipped_value_loss = True,
        clip_param             = 0.2,
        entropy_coef           = 0.003,
        num_learning_epochs    = 4,
        num_mini_batches       = 4,
        learning_rate          = 3e-4,
        schedule               = "adaptive",
        gamma                  = 0.99,
        lam                    = 0.95,
        desired_kl             = 0.01,
        max_grad_norm          = 0.5,
    )


# ── 환경 생성 ─────────────────────────────────────────────────────────────────
env_cfg = RoverRecoveryEnvCfgV4()
env_cfg.scene.num_envs = args.num_envs
env_cfg.seed = args.seed

env = ManagerBasedRLEnv(cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

_env     = env.unwrapped
_vehicle = _env.scene["vehicle"]
_cfg     = _env.cfg
_physx   = _cfg.sim.physx
_artprop = _cfg.scene.vehicle.spawn.articulation_props

_gpu_total  = torch.cuda.get_device_properties(0).total_memory / 1024**3
_gpu_used   = torch.cuda.memory_allocated(0) / 1024**3
_physics_hz = int(1.0 / _cfg.sim.dt)
_policy_hz  = int(1.0 / (_cfg.sim.dt * _cfg.decimation))

print(f"\n{'='*64}")
print(f"  [ v4 Simulation Diagnostics ]")
print(f"  GPU                : {torch.cuda.get_device_name(0)}")
print(f"  GPU memory         : {_gpu_used:.2f} GB / {_gpu_total:.1f} GB")
print(f"  Num envs           : {_env.num_envs}")
print(f"  Vehicle bodies     : {_vehicle.num_bodies}")
print(f"  Vehicle joints     : {_vehicle.num_joints}")
print(f"  Physics dt         : {_cfg.sim.dt*1000:.1f} ms  ({_physics_hz} Hz)")
print(f"  Policy decimation  : {_cfg.decimation}  →  {_policy_hz} Hz")
print(f"  Episode length     : {_cfg.episode_length_s} s  (v4: 20s)")
print(f"  Action dim         : 12  (arm 6 + wheel 6)")
print(f"  Obs dim            : 43  (v3 31 + forward 12)")
print(f"  Forward goal       : 2.0 m")
print(f"{'='*64}\n")

# ── 에이전트 ─────────────────────────────────────────────────────────────────
agent_cfg = RoverRecoveryAgentCfgV4()
agent_cfg.max_iterations = args.max_iterations
agent_cfg.seed = args.seed

# fine-tune 오버라이드 적용
if args.lr           is not None: agent_cfg.algorithm.learning_rate      = args.lr
if args.clip_param   is not None: agent_cfg.algorithm.clip_param         = args.clip_param
if args.entropy_coef is not None: agent_cfg.algorithm.entropy_coef       = args.entropy_coef
if args.mini_batches is not None: agent_cfg.algorithm.num_mini_batches   = args.mini_batches

agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)

log_root = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", args.log_dir
))
log_dir = args.resume_log_dir or os.path.join(
    log_root, datetime.now().strftime("%Y%m%d_%H%M%S")
)

runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device="cuda:0")

if args.checkpoint:
    runner.load(args.checkpoint)
    if args.lr is not None:
        # checkpoint LR 무시하고 지정값 사용
        for pg in runner.alg.optimizer.param_groups:
            pg["lr"] = args.lr
        runner.alg.learning_rate = args.lr
        print(f"[train_v4] LR 오버라이드: {args.lr:.2e}")
    else:
        restored_lr = runner.alg.optimizer.param_groups[0]["lr"]
        runner.alg.learning_rate = restored_lr
        print(f"[train_v4] LR 복원: {restored_lr:.2e}")
    print(f"[train_v4] checkpoint: {args.checkpoint}")

_alg = agent_cfg.algorithm
print(f"\n{'='*60}")
print(f"  Rover Recovery v4 학습 시작")
print(f"  num_envs    : {args.num_envs}")
print(f"  max_iter    : {args.max_iterations}")
print(f"  lr          : {_alg.learning_rate:.2e}")
print(f"  clip_param  : {_alg.clip_param}")
print(f"  entropy_coef: {_alg.entropy_coef}")
print(f"  mini_batches: {_alg.num_mini_batches}")
print(f"  std_max     : {args.policy_std_max}")
print(f"  log_dir     : {log_dir}")
print(f"  TensorBoard : tensorboard --logdir {log_root}")
print(f"{'='*60}\n")

_t_start = time.perf_counter()

policy = runner.alg.get_policy()
best_metrics = {"eval/success_rate": -1.0, "eval/timeout_rate": 1.0}
best_ckpt = os.path.join(log_dir, "best_rover_v4.pt")

try:
    start_it = runner.current_learning_iteration
    total_it  = start_it + args.max_iterations
    obs = env.get_observations().to(runner.device)
    runner.alg.train_mode()

    if runner.is_distributed:
        runner.alg.broadcast_parameters()

    runner.logger.init_logging_writer()

    for it in range(start_it, total_it):
        t0 = time.time()

        with torch.inference_mode():
            for _ in range(runner.cfg["num_steps_per_env"]):
                actions = runner.alg.act(obs)
                obs, rewards, dones, extras = env.step(actions.to(env.device))
                obs, rewards, dones = (
                    obs.to(runner.device),
                    rewards.to(runner.device),
                    dones.to(runner.device),
                )
                runner.alg.process_env_step(obs, rewards, dones, extras)
                intr = runner.alg.intrinsic_rewards if runner.cfg["algorithm"]["rnd_cfg"] else None
                runner.logger.process_env_step(rewards, dones, extras, intr)

        collect_time = time.time() - t0
        t1 = time.time()

        runner.alg.compute_returns(obs)
        loss_dict = runner.alg.update()

        raw_std = _policy_std_tensor(policy)
        if raw_std is not None:
            _log_policy_std(runner.logger.writer, raw_std, it, args.policy_std_max)
        std_tensor, _ = _clamp_policy_std(policy, args.policy_std_min, args.policy_std_max)

        learn_time = time.time() - t1
        runner.current_learning_iteration = it

        runner.logger.log(
            it=it, start_it=start_it, total_it=total_it,
            collect_time=collect_time, learn_time=learn_time,
            loss_dict=loss_dict,
            learning_rate=runner.alg.learning_rate,
            action_std=std_tensor,
            rnd_weight=runner.alg.rnd.weight if runner.cfg["algorithm"]["rnd_cfg"] else None,
        )

        if runner.logger.writer and (it % runner.cfg["save_interval"] == 0 or it == total_it - 1):
            runner.save(os.path.join(runner.logger.log_dir, f"model_{it}.pt"))

        if (it + 1) % args.eval_interval == 0 or it == total_it - 1:
            try:
                stats = evaluate_policy(policy, env, args.eval_episodes)
                obs = env.get_observations().to(runner.device)
                for tag, val in stats.items():
                    if runner.logger.writer:
                        runner.logger.writer.add_scalar(tag, val, it)
                print(
                    f"[eval] iter {it:5d} | success={stats['eval/success_rate']:.3f}"
                    f" | timeout={stats['eval/timeout_rate']:.3f}"
                )
                if (stats["eval/success_rate"] > best_metrics["eval/success_rate"] or
                        (stats["eval/success_rate"] == best_metrics["eval/success_rate"] and
                         stats["eval/timeout_rate"] < best_metrics["eval/timeout_rate"])):
                    best_metrics = stats
                    runner.save(best_ckpt, infos=stats)
                    print(f"[eval] best checkpoint: {best_ckpt}")
            finally:
                runner.alg.train_mode()

    _t_total = time.perf_counter() - _t_start
    _samples  = args.max_iterations * runner.cfg["num_steps_per_env"] * args.num_envs
    print(f"\n{'='*60}")
    print(f"  [ v4 Training Done ]  total={_t_total:.1f}s")
    print(f"  env steps/sec : {_samples / _t_total:.0f}")
    print(f"  best success  : {best_metrics['eval/success_rate']:.3f}")
    print(f"{'='*60}\n")

    policy_out = os.path.join(
        os.path.dirname(__file__), "..", "..", "policies", "recovery_policy_v4.pt"
    )
    os.makedirs(os.path.dirname(policy_out), exist_ok=True)
    torch.save(policy.state_dict(), policy_out)
    print(f"[train_v4] 정책 저장: {policy_out}")

finally:
    env.close()
    simulation_app.close()
