# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""vehicle_v1 WASD 수동 주행 + 2D 맵 goal 클릭 자동 주행 — terrain_00022.

별도 matplotlib 창에 차량 현재 위치·방향이 실시간으로 표시된다.  맵을
좌클릭하면 그 (x, y) 좌표를 goal 로 잡아 차량이 자동으로 거기까지 간다 —
가는 도중 전방 장애물은 ObstacleAvoider 가 알아서 우회한다.  도착·우클릭·
R·WASD 입력 중 어느 것이든 일어나면 goal 이 해제되고 수동 모드로 돌아온다.

  조작:
    좌클릭 (2D 맵) : 그 좌표를 goal 로 설정 → 자동 주행 시작
    우클릭 (2D 맵) : goal 해제
    W / S / A / D  : 수동 주행 (자동 주행을 즉시 끊음)
    R              : 차량·상태 리셋 (스폰 위치로, goal 해제)
    ESC            : 종료

  자동 주행 (no obstacle):
    · goal 방향으로 yaw 정렬 (P 제어) + 전진.
    · |yaw_err| 클수록 전진속도 줄여 제자리 선회에 가깝게.
    · goal 반경 GOAL_TOL 안으로 들어오면 정지·goal 해제.

  자동 주행 중 장애물 감지:
    · ObstacleAvoider 가 AVOID / REVERSE 모드로 진입 → avoider 명령 그대로.
    · 전방이 깨끗해져 CRUISE 로 복귀하면 다시 goal 방향으로 정렬.

  ※ 키 입력은 Isaac Sim 뷰어 창이 포커스일 때만 동작.

실행:
    cd .../isaac_drive/drive_test
    /home/rokey/dev_ws/venv/isaaclab/bin/python play_avoid.py
"""

import argparse
import math
import os
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="vehicle_v1 WASD + 2D 맵 goal 자동 주행")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

# GUI 로 기동 (--headless 를 주면 키 입력 불가).
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""--- 시뮬레이터 기동 후 임포트 ---"""

import carb  # noqa: E402
import omni.appwindow  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from avoidance import ObstacleAvoider  # noqa: E402
from detector import detect_obstacles  # noqa: E402
from drive_env_cfg import DriveEnvCfg  # noqa: E402
from goal_map import GoalMap  # noqa: E402
from rover_vehicle import keep_arm_folded  # noqa: E402

# 주행 파라미터.  속도·회피각을 한 단계 올린 세팅.
LIN_SPEED = 3.0      # 수동 전/후진 (m/s)
ANG_SPEED = 2.0      # 수동 좌/우회전 (rad/s)
GOAL_CRUISE = 2.5    # goal 자동 주행 시 전진 속도 (m/s)
GOAL_TURN_K = 2.5    # goal yaw 정렬 P 게인 (rad/s · rad)
GOAL_TOL = 0.6       # 도착 판정 반경 (m)

# --- 키보드 상태 추적 ---
KEY = carb.input.KeyboardInput
_pressed: set = set()

_MODE_KR = {"CRUISE": "직진", "GRAZE": "옆비킴", "AVOID": "회피", "REVERSE": "후진"}


def _on_keyboard(event, *args):
    """carb 키보드 이벤트 → _pressed 집합 갱신."""
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        _pressed.add(event.input)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _pressed.discard(event.input)
    return True


def _yaw_from_quat(q) -> float:
    """Isaac Lab root_quat_w (w, x, y, z) → z 축 yaw (rad)."""
    w = float(q[0]); x = float(q[1]); y = float(q[2]); z = float(q[3])
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main() -> None:
    env = ManagerBasedEnv(DriveEnvCfg())
    robot = env.scene["robot"]
    env.reset()
    keep_arm_folded(robot)

    # 키보드 구독 (뷰어 창 기준).
    appwindow = omni.appwindow.get_default_app_window()
    keyboard = appwindow.get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    kbd_sub = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard)

    scanner = env.scene["height_scanner"]
    device = env.device
    # 회피 각속도·속도를 윗단(LIN_SPEED·GOAL_CRUISE)과 맞춰 더 빠르게·더 크게.
    avoider = ObstacleAvoider(
        cruise_speed=GOAL_CRUISE,
        avoid_speed=1.6,
        base_ang=1.0,
        max_ang=2.2,
    )
    gmap = GoalMap(half_extent=25.0)

    print("\n" + "=" * 60)
    print("  vehicle_v1  WASD 수동 + 2D 맵 goal 자동 주행  (terrain_00022)")
    print("-" * 60)
    print("  2D 맵에 좌클릭 → 그 좌표를 goal 로 자동 주행 시작")
    print("                우클릭 → goal 해제")
    print("  W / S / A / D : 수동 주행 (자동 주행을 즉시 끊음)")
    print("  R : 리셋   ESC : 종료")
    print("  ※ 키 입력은 Isaac Sim 뷰어 창 포커스일 때만 동작.")
    print("=" * 60 + "\n")

    prev_state = None              # 직전 (mode, avoid_dir)
    last_obs_print = 0.0           # 직전 장애물 위치 출력 시각 (1초 주기)
    OBS_PRINT_INTERVAL = 1.0       # 장애물 위치 출력 주기 (초)
    try:
        while simulation_app.is_running():
            # --- 종료 / 리셋 ---
            if KEY.ESCAPE in _pressed:
                break
            if KEY.R in _pressed:
                env.reset()
                keep_arm_folded(robot)
                avoider.reset()
                gmap.clear_goal()
                _pressed.discard(KEY.R)
                prev_state = None
                last_obs_print = 0.0
                print("[리셋] 차량을 스폰 위치로, goal 해제\n")

            # --- 차량 pose → 맵 갱신 ---
            pos = robot.data.root_pos_w[0]
            quat = robot.data.root_quat_w[0]
            x = float(pos[0])
            y = float(pos[1])
            yaw = _yaw_from_quat(quat)
            gmap.update(x, y, yaw)

            # --- 키 입력 ---
            w = KEY.W in _pressed
            s = KEY.S in _pressed
            a = KEY.A in _pressed
            d = KEY.D in _pressed
            manual = w or s or a or d

            # 자동 주행 중 WASD → goal 해제 (수동 우선).
            if manual and gmap.goal is not None:
                gmap.clear_goal()
                avoider.reset()
                prev_state = None
                print("[수동 복귀] WASD 입력 감지 → goal 해제\n")

            # --- 액션 결정 ---
            obstacles = detect_obstacles(scanner)

            if manual:
                lin = (w - s) * LIN_SPEED
                ang = (a - d) * ANG_SPEED
            elif gmap.goal is not None:
                gx, gy = gmap.goal
                dx = gx - x
                dy = gy - y
                dist = math.hypot(dx, dy)
                if dist < GOAL_TOL:
                    # 도착.
                    lin, ang = 0.0, 0.0
                    gmap.clear_goal()
                    avoider.reset()
                    prev_state = None
                    print(f"[도착] ({x:+.2f}, {y:+.2f}) m\n")
                else:
                    av_lin, av_ang = avoider.compute_action(obstacles)
                    if avoider.mode == "CRUISE":
                        # 회피 없음 → goal 방향으로 정렬·전진.
                        desired_yaw = math.atan2(dy, dx)
                        yaw_err = (desired_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
                        ang = max(-ANG_SPEED, min(ANG_SPEED, GOAL_TURN_K * yaw_err))
                        # yaw_err 가 크면 전진속도를 줄임 → 제자리 선회에 가깝게.
                        lin = GOAL_CRUISE * max(0.0, math.cos(yaw_err))
                    else:
                        # AVOID / REVERSE → avoider 가 만든 명령 그대로.
                        lin, ang = av_lin, av_ang
            else:
                # 수동도 자동도 아니면 정지.
                lin, ang = 0.0, 0.0

            action = torch.tensor([[lin, ang]], dtype=torch.float32, device=device)
            env.step(action)
            keep_arm_folded(robot)

            # --- 매 프레임: 감지된 장애물 월드 좌표 → discovered 맵에 누적 ---
            if obstacles:
                cy, sy = math.cos(yaw), math.sin(yaw)
                world_pts = [
                    (x + o["fwd"] * cy - o["lat"] * sy,
                     y + o["fwd"] * sy + o["lat"] * cy)
                    for o in obstacles
                ]
                gmap.add_detections(world_pts)
            else:
                world_pts = []

            # --- 장애물 위치 1초 주기 출력 (자동·수동 무관, 감지되면 항상) ---
            now = time.monotonic()
            if obstacles and (now - last_obs_print) >= OBS_PRINT_INTERVAL:
                last_obs_print = now
                print(f"🚧 장애물 {len(obstacles)}개  "
                      f"(차량 ({x:+.2f}, {y:+.2f}) m, yaw {math.degrees(yaw):+.1f}°)")
                for k, (o, (wx, wy)) in enumerate(zip(obstacles, world_pts), 1):
                    print(f"   #{k} {o['label']}  "
                          f"전방 {o['fwd']:+.2f}m · 좌측 {o['lat']:+.2f}m  "
                          f"→ world ({wx:+.2f}, {wy:+.2f}) m")

            # --- 자동 주행 중일 때만 회피 상태 변화 출력 ---
            if gmap.goal is not None and not manual:
                state = (avoider.mode, avoider.avoid_dir)
                if state != prev_state:
                    prev_state = state
                    tag = _MODE_KR.get(avoider.mode, avoider.mode)
                    if avoider.mode in ("AVOID", "GRAZE"):
                        tag += "(좌)" if avoider.avoid_dir > 0 else "(우)"
                    print(f"  →  [{tag}]  명령 lin={lin:+.2f} m/s · ang={ang:+.2f} rad/s")
    except KeyboardInterrupt:
        print("\n[중단] 사용자 종료")
    finally:
        kbd_sub = None  # noqa: F841  구독 해제
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
