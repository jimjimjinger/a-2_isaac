# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""stage 1 height-scan 큐브 감지 점검.

로버를 큐브(정면 3.5m) 쪽으로 직진시키며, 매 스텝 height-scan 격자에서
'큐브로 솟은 셀 수(obstacle_cells)' 를 기록한다.

  - 전진 중 obstacle_cells 가 0 → 양수로 올라가면 → RayCaster 가 큐브 감지 OK
  - 큐브 근처를 지나는데도 끝까지 0 이면 → 큐브가 너무 작아 ray 사이로 빠짐
    → cube_size 를 키우거나 height-scan resolution 을 촘촘하게 해야 함

실행:
    cd .../isaac_drive/avoid_test
    /home/rokey/dev_ws/IsaacLab/isaaclab.sh -p debug_fixed_obs.py --num_envs 2 --headless
    cat /tmp/stage1_scan_check.txt
"""

import argparse
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="stage 1 height-scan 큐브 감지 점검")
parser.add_argument("--num_envs", type=int, default=2)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

from isaaclab.envs import ManagerBasedRLEnv

from rover_avoid.fixed_obs_env_cfg import FixedObsEnvCfg

REPORT_PATH = "/tmp/stage1_scan_check.txt"


def main() -> None:
    out = open(REPORT_PATH, "w", buffering=1)
    out.write("=== stage 1 height-scan 큐브 감지 점검 ===\n")
    try:
        cfg = FixedObsEnvCfg()
        cfg.scene.num_envs = args_cli.num_envs
        env = ManagerBasedRLEnv(cfg)
        env.reset()

        robot = env.scene["robot"]
        scanner = env.scene["height_scanner"]
        dev = env.device

        out.write("큐브: 정면 3.5m · 0.5m각 · 0.3m높\n")
        out.write("height-scan: 21x21=441 ray, 격자 간격 0.2m, 4x4m 범위\n")
        out.write("로버를 직진시키며 '바닥보다 0.15m 이상 솟은 ray 셀' 수를 기록.\n\n")
        out.write(f"  {'step':>5} {'robot_x(m)':>11} {'scan_max_z':>11} {'obstacle_cells':>15}\n")

        # 직진 액션 (선속도 0.8, 각속도 0).
        action = torch.tensor([[0.8, 0.0]] * env.num_envs, dtype=torch.float32, device=dev)
        x0 = robot.data.root_pos_w[0, 0].item()
        detected_any = False

        for step in range(1, 51):
            env.step(action)
            hits_z = scanner.data.ray_hits_w[0, :, 2]
            hits_z = hits_z[torch.isfinite(hits_z)]
            if hits_z.numel() == 0:
                continue
            ground = hits_z.min().item()
            scan_max = hits_z.max().item()
            obstacle_cells = int((hits_z > ground + 0.15).sum())
            if obstacle_cells > 0:
                detected_any = True
            rx = robot.data.root_pos_w[0, 0].item() - x0
            if step % 2 == 0:
                mark = "  <== 큐브 감지" if obstacle_cells > 0 else ""
                out.write(f"  {step:5d} {rx:11.2f} {scan_max:11.3f} {obstacle_cells:15d}{mark}\n")

        out.write("\n[판정]\n")
        if detected_any:
            out.write("  OK — 전진 중 큐브가 height-scan 에 잡혔다. 정책이 큐브를 '본다'.\n")
        else:
            out.write("  실패 — 큐브가 한 번도 안 잡혔다. ray 사이로 빠지는 중.\n")
            out.write("         cube_size ↑ 또는 height-scan resolution 을 촘촘히 할 것.\n")
        env.close()

    except Exception as exc:  # noqa: BLE001
        import traceback
        out.write(f"\n[ERROR] {type(exc).__name__}: {exc}\n")
        out.write(traceback.format_exc())
    finally:
        out.close()
        print(f"[debug_fixed_obs] 리포트 저장: {REPORT_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
