# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""drive_test 자체검증 — 키보드 없이 자동 전진하며 씬을 점검한다.

avoid_test/check_scene.py 와 같은 역할.  WASD 없이도 다음을 확인한다:
  1) vehicle_v1.usd 가 스폰되고 27 DOF 조인트가 매핑되는가
  2) RayCaster 가 부착·동작하는가
  3) Ackermann 액션으로 전진하는가 (pos x 증가)
  4) 전진하다 정면 큐브가 격자에 들어오면 장애물로 인식되는가

stdout 은 Isaac Sim 로그에 묻히므로 리포트를 파일로 쓴다.

실행:
    cd .../isaac_drive/drive_test
    /home/rokey/dev_ws/venv/isaaclab/bin/python check_drive.py --headless
    cat /tmp/drive_check_report.txt
"""

import argparse
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="drive_test 씬 자체검증")
parser.add_argument("--steps", type=int, default=400, help="자동 전진 step 수")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""--- 시뮬레이터 기동 후 임포트 ---"""

import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from detector import detect_obstacles  # noqa: E402
from drive_env_cfg import DriveEnvCfg  # noqa: E402
from rover_vehicle import keep_arm_folded  # noqa: E402

REPORT = "/tmp/drive_check_report.txt"


def main() -> None:
    out = open(REPORT, "w", buffering=1)
    out.write("=== drive_test 자체검증 ===\n")
    try:
        env = ManagerBasedEnv(DriveEnvCfg())
        robot = env.scene["robot"]
        env.reset()
        keep_arm_folded(robot)
        scanner = env.scene["height_scanner"]
        device = env.device

        # --- (1) 스폰 / 조인트 매핑 ---
        out.write("\n[1] 스폰 & 조인트 매핑\n")
        out.write(f"  device      : {device}\n")
        out.write(f"  DOF 수      : {len(robot.data.joint_names)}\n")
        out.write(f"  joint_names : {list(robot.data.joint_names)}\n")
        out.write(f"  body 수     : {len(robot.data.body_names)}\n")
        out.write(f"  action_dim  : {env.action_manager.total_action_dim}\n")

        # --- (2) RayCaster 기초 점검 — 몇 step 굴려 버퍼 채우기 ---
        zero = torch.zeros((env.num_envs, env.action_manager.total_action_dim), device=device)
        for _ in range(5):
            env.step(zero)
            keep_arm_folded(robot)
        n0 = len(detect_obstacles(scanner))
        ray_n = scanner.data.ray_hits_w.shape[1]
        out.write("\n[2] 하향 RayCaster\n")
        out.write(f"  ray 개수        : {ray_n}\n")
        out.write(f"  스폰 직후 장애물 : {n0}개  (베이스캠프 평지 → 0 이 정상)\n")

        # --- (3~4) 자동 전진하며 pos·장애물 인식 로그 ---
        out.write("\n[3] 자동 전진 (lin=0.8) — pos 변화 · 장애물 개수 + 최근접 위치\n")
        out.write(f"  {'step':>5}  {'pos_x':>7}  {'개수':>4}  "
                  f"{'방향':>9}  {'전방m':>7}  {'좌측m':>7}  비고\n")
        action = torch.tensor([[0.8, 0.0]], dtype=torch.float32, device=device)
        first_detect_step = None
        first_detect_x = None
        for step in range(1, args_cli.steps + 1):
            env.step(action)
            keep_arm_folded(robot)
            obstacles = detect_obstacles(scanner)
            if obstacles and first_detect_step is None:
                first_detect_step = step
                first_detect_x = float(robot.data.root_pos_w[0, 0].cpu())
            if step % 25 == 0 or step == first_detect_step:
                pos = robot.data.root_pos_w[0].cpu()
                note = "  <== 첫 인식!" if step == first_detect_step else ""
                near = obstacles[0] if obstacles else {"label": "", "fwd": 0.0, "lat": 0.0}
                out.write(f"  {step:5d}  {float(pos[0]):7.2f}  {len(obstacles):4d}  "
                          f"{(near['label'] or '-'):>9}  {near['fwd']:7.2f}  "
                          f"{near['lat']:7.2f}{note}\n")

        # --- 판정 ---
        out.write("\n[판정]\n")
        moved = float(robot.data.root_pos_w[0, 0].cpu())
        out.write(f"  - 전진 거리(x)     : {moved:+.2f} m  "
                  f"=> {'OK 주행됨' if moved > 0.5 else 'FAIL 안 움직임'}\n")
        if first_detect_step is not None:
            out.write(f"  - 장애물 첫 인식    : step {first_detect_step}, "
                      f"x={first_detect_x:.2f} m  => OK 레이캐스트 인식됨\n")
        else:
            out.write("  - 장애물 첫 인식    : 없음  => FAIL (전진 부족 or 인식 안 됨)\n")
        out.write(f"  - DOF 27 / drive·steer 조인트 매핑은 [1] 의 joint_names 확인\n")
        out.write("\n[DONE] 자체검증 완료\n")
        env.close()

    except Exception as exc:  # noqa: BLE001
        import traceback
        out.write(f"\n[ERROR] {type(exc).__name__}: {exc}\n")
        out.write(traceback.format_exc())
    finally:
        out.close()
        print(f"[check_drive] 리포트 저장: {REPORT}")


if __name__ == "__main__":
    main()
    simulation_app.close()
