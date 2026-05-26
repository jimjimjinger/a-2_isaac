# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""vehicle_v1 WASD 수동 주행 + 레이캐스트 지형 스캔 테스트.

terrain_00022 (50×50m 화성형 지형 + 바위) 위에서 통합 차량 vehicle_v1.usd 를
키보드로 조작한다.  하향 RayCaster(avoid_test 와 동일 설정)가 5×5m 격자로
지형을 스캔하고, detector 가 히트 높이의 '국소 돌출량'으로 갑자기 솟은
부분(바위)을 장애물로 판정한다 (완만한 경사·언덕은 무시).

  조작:
    W / S : 전진 / 후진
    A / D : 좌회전 / 우회전
    R     : 차량 리셋 (스폰 위치·정면으로)
    ESC   : 종료

  ※ 키 입력은 Isaac Sim 뷰어 창이 포커스일 때만 동작.
  ※ 뷰어 카메라 fly 모드(마우스 우클릭 드래그)와 WASD 가 겹치므로,
     주행 중에는 우클릭을 누르지 않는다.

실행:
    cd .../isaac_drive/drive_test
    /home/rokey/dev_ws/venv/isaaclab/bin/python play_wasd.py
"""

import argparse
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="vehicle_v1 WASD 주행 테스트")
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

from detector import detect_obstacles  # noqa: E402
from drive_env_cfg import DriveEnvCfg  # noqa: E402
from rover_vehicle import keep_arm_folded  # noqa: E402

# 주행 파라미터 — 빠른 수동 주행용 (이전 0.8 → 상향).
# 이 속도를 내려면 rover_vehicle.py 의 drive velocity_limit_sim 과
# rigid_props.max_linear_velocity 도 함께 올라가 있어야 한다.
LIN_SPEED = 2.0   # 전/후진 선속도 (m/s)
ANG_SPEED = 1.5   # 좌/우회전 각속도 (rad/s)

# --- 키보드 상태 추적 ---
KEY = carb.input.KeyboardInput
_pressed: set = set()


def _on_keyboard(event, *args):
    """carb 키보드 이벤트 → _pressed 집합 갱신."""
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        _pressed.add(event.input)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _pressed.discard(event.input)
    return True


def main() -> None:
    env = ManagerBasedEnv(DriveEnvCfg())
    robot = env.scene["robot"]
    env.reset()
    keep_arm_folded(robot)   # m0609 팔을 접힌 HOME 으로 고정

    # 키보드 구독 (뷰어 창 기준).
    appwindow = omni.appwindow.get_default_app_window()
    keyboard = appwindow.get_keyboard()
    input_iface = carb.input.acquire_input_interface()
    kbd_sub = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard)

    scanner = env.scene["height_scanner"]
    device = env.device

    print("\n" + "=" * 60)
    print("  vehicle_v1  WASD 주행 + 레이캐스트 장애물 인식 테스트")
    print("-" * 60)
    print("  W / S : 전진 / 후진      A / D : 좌 / 우회전")
    print("  R     : 차량 리셋        ESC   : 종료")
    print("  맵: terrain_00022 (50×50m 생성 지형 + 바위 80개)")
    print("  ※ 뷰어 창이 포커스일 때만 키가 먹습니다.")
    print("=" * 60 + "\n")

    prev_n = 0          # 직전 프레임 장애물 개수 (개수가 바뀔 때만 출력)
    try:
        while simulation_app.is_running():
            # --- 키 입력 → Ackermann 액션 ---
            if KEY.ESCAPE in _pressed:
                break
            if KEY.R in _pressed:
                env.reset()
                keep_arm_folded(robot)
                _pressed.discard(KEY.R)
                prev_n = 0
                print("[리셋] 차량을 스폰 위치로 되돌림\n")

            lin = ((KEY.W in _pressed) - (KEY.S in _pressed)) * LIN_SPEED
            ang = ((KEY.A in _pressed) - (KEY.D in _pressed)) * ANG_SPEED
            action = torch.tensor([[lin, ang]], dtype=torch.float32, device=device)

            env.step(action)
            keep_arm_folded(robot)   # 팔을 접힌 HOME 으로 매 프레임 고정

            # --- 레이캐스트 장애물 인식 (덩어리별로 여러 개 동시) ---
            obstacles = detect_obstacles(scanner)
            if len(obstacles) != prev_n:
                # 장애물 개수가 바뀔 때만 출력 (매 프레임 도배 방지).
                prev_n = len(obstacles)
                if prev_n == 0:
                    print("    장애물 벗어남\n")
                else:
                    print(f"🚧  장애물 {prev_n}개 인식!")
                    for k, o in enumerate(obstacles, 1):
                        print(f"     #{k} {o['label']}  "
                              f"전방 {o['fwd']:+.2f}m · 좌측 {o['lat']:+.2f}m  "
                              f"(격자 {o['n_cells']}셀, 돌출 {o['peak']:.2f}m)")
    except KeyboardInterrupt:
        print("\n[중단] 사용자 종료")
    finally:
        kbd_sub = None  # noqa: F841  구독 해제
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
