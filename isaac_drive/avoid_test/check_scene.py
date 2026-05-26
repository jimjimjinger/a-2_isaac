# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""1단계 진단: 평지+장애물 씬 + 로버 수동주행 + 레이캐스트/충돌 점검.

m0609_lift_code_ver2/test_grasp_lift.py 와 같은 역할 — RL 정책 없이, 씬이
물리적으로 올바른지 뷰어로 + 리포트로 확인한다.

확인 항목:
  1) 로버가 스폰되고 조향/구동 조인트가 매핑되는가
  2) Ackermann 액션으로 전진·회전이 되는가 (pos 가 변하는가)
  3) 하향 RayCaster 가 장애물을 '높이 돌출'로 감지하는가
  4) ContactSensor 가 장애물 충돌 시 힘을 보고하는가

실행 (IsaacLab venv 로):
    cd .../isaac_drive/avoid_test
    python check_scene.py --num_envs 2
    cat /tmp/avoid_scene_report.txt

뷰어 없이 돌리려면 --headless 추가.
"""

import argparse
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="avoid_test 씬 진단")
parser.add_argument("--num_envs", type=int, default=2, help="병렬 환경 수 (눈 확인용 1~2 권장)")
parser.add_argument("--env", type=str, default="fixed",
                    choices=["fixed", "avoid", "stage2"],
                    help="환경: fixed=고정장애물1개(0.5×0.5×0.3m) / "
                         "avoid=랜덤8개 / stage2=큐브3개")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""--- 시뮬레이터 기동 후 임포트 ---"""

import torch

from rover_avoid.avoid_env_cfg import AvoidEnvCfg
from rover_avoid.fixed_obs_env_cfg import FixedObsEnvCfg
from rover_avoid.stage2_env_cfg import Stage2EnvCfg
from rover_avoid.vehicle_env import VehicleAvoidEnv

# --env 선택 → env cfg 클래스.
_ENV_CFG = {"fixed": FixedObsEnvCfg, "avoid": AvoidEnvCfg, "stage2": Stage2EnvCfg}

REPORT_PATH = "/tmp/avoid_scene_report.txt"


def main() -> None:
    out = open(REPORT_PATH, "w", buffering=1)
    out.write(f"=== AVOID-TEST 씬 진단 (env={args_cli.env}) ===\n")
    try:
        cfg = _ENV_CFG[args_cli.env]()
        cfg.scene.num_envs = args_cli.num_envs
        env = VehicleAvoidEnv(cfg)   # 매 step m0609 팔 HOME 고정
        obs, _ = env.reset()

        robot = env.scene["robot"]
        scanner = env.scene["height_scanner"]
        contact = env.scene["contact_sensor"]
        device = env.device

        # --- (1) 스폰 / 조인트 매핑 점검 ---
        out.write("\n[1] 스폰 & 조인트 매핑\n")
        out.write(f"  num_envs   : {env.num_envs}   device: {device}\n")
        out.write(f"  joint_names: {list(robot.data.joint_names)}\n")
        out.write(f"  body_names : {list(robot.data.body_names)}\n")
        out.write(f"  action_dim : {env.action_manager.action.shape[1]}\n")
        out.write(f"  obs dim    : {tuple(obs['policy'].shape)}\n")

        # --- (1.5) 레이캐스터 기초 점검 ---
        # 몇 step 굴려 센서 버퍼를 채운다.
        zero = torch.zeros((env.num_envs, env.action_manager.action.shape[1]), device=device)
        for _ in range(5):
            env.step(zero)

        hits = scanner.data.ray_hits_w[0]                 # (num_rays, 3)
        finite = torch.isfinite(hits[:, 2])
        sc_z = hits[finite, 2]
        out.write("\n[2] 하향 RayCaster\n")
        out.write(f"  ray 개수        : {hits.shape[0]}  (정상 히트 {int(finite.sum())})\n")
        if sc_z.numel() > 0:
            out.write(f"  히트 z 범위     : {float(sc_z.min()):+.3f} ~ {float(sc_z.max()):+.3f} m\n")
            out.write(f"  센서 월드 z     : {float(scanner.data.pos_w[0, 2]):+.3f} m\n")
            out.write("  (평지=낮고 고른 z, 장애물 위=z 솟음. 둘이 구분되면 OK)\n")

        # --- (3) 베이스 컨트롤러 자동 주행 (RL 보정=0) ---
        # 액션이 잔차(1D 조향 보정)이므로 0 을 주면 베이스 goto-goal
        # 컨트롤러만 동작한다 → 회피 없이 goal 로 직진. goal_d 가 줄면
        # 베이스 컨트롤러 OK. (FixedObs 면 장애물에서 contact_N 이 솟음
        #  = RL 회피 학습이 필요한 이유를 그대로 보여줌.)
        out.write("\n[3] 베이스 컨트롤러 주행 (RL 보정=0) — goal 접근·장애물·충돌\n")
        out.write(f"  {'step':>5}  {'pos(x,y)':>16}  {'goal_d':>7}  "
                  f"{'obstacle_cells':>14}  {'contact_N':>10}\n")
        action = torch.zeros(
            (env.num_envs, env.action_manager.action.shape[1]), device=device
        )
        for step in range(1, 301):
            env.step(action)
            if step % 25 != 0:
                continue
            pos = robot.data.root_pos_w[0].cpu()
            cmd = env.command_manager.get_command("target_pose")[0].cpu()
            goal_d = float(torch.norm(cmd[:2]))
            # height-scan: 지면보다 0.15m 이상 솟은 ray = 장애물 셀.
            z = scanner.data.ray_hits_w[0, :, 2]
            z = z[torch.isfinite(z)]
            obstacle_cells = int((z > z.min() + 0.15).sum()) if z.numel() else 0
            # 몸체 접촉력 — 장애물 충돌 시 솟구침.
            cforce = float(torch.norm(contact.data.net_forces_w[0], dim=-1).max())
            hit = "  <== 충돌!" if cforce > 1.0 else ""
            out.write(f"  {step:5d}  ({pos[0]:+.2f},{pos[1]:+.2f})    "
                      f"{goal_d:7.2f}  {obstacle_cells:14d}  {cforce:10.2f}{hit}\n")

        out.write("\n[판정 가이드]\n")
        out.write("  - [1] joint_names 에 *_Steer_Revolute/*_Drive_Continuous, action_dim=1 이면 OK\n")
        out.write("  - [2] ray 개수 256(3×3 격자)·정상 히트면 레이캐스트 OK\n")
        out.write("  - [3] goal_d 가 step 따라 줄면 베이스 goto-goal 컨트롤러 OK\n")
        out.write("  - [3] obstacle_cells 가 장애물 근처에서 0보다 커지면 장애물 감지 OK\n")
        out.write("  - [3] RL 보정=0 이라 장애물에서 contact_N 이 솟는 게 정상 (회피는 학습으로)\n")
        out.write("\n[DONE] 씬 진단 완료\n")

        # GUI(--headless 아님)면 주행 시퀀스 후에도 뷰어를 열어둬, 차량·팔·
        # 레이캐스트·지형을 눈으로 둘러볼 수 있게 한다. 창을 닫으면 종료.
        if not args_cli.headless:
            print("[check_scene] 진단 끝 — 뷰어 창을 닫으면 종료됩니다.")
            idle = torch.zeros(
                (env.num_envs, env.action_manager.action.shape[1]), device=device
            )
            while simulation_app.is_running():
                env.step(idle)
        env.close()

    except Exception as exc:  # noqa: BLE001
        import traceback
        out.write(f"\n[ERROR] {type(exc).__name__}: {exc}\n")
        out.write(traceback.format_exc())
    finally:
        out.close()
        print(f"[check_scene] 리포트 저장: {REPORT_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
